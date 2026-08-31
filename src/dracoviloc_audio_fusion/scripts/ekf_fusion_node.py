#!/usr/bin/env python3
"""
ekf_fusion_node.py  -  bearing-only EKF for the DRACOVILOC anti-UAV tracker

    /odas/sst                    (PointStamped,  ~86 Hz)  ─┐
    /audio_classifier/detection  (Vector3Stamped, AST)    ─┼─► EKF ─► /fused_target_pose
    /gre_classifier/detection    (Vector3Stamped, GRE)    ─┤
    /camera/yolo_detection       (PointStamped,  ~30 Hz)  ─┘

WHY BEARING-ONLY
----------------
The UMA-16 operates in the far field. ODAS emits unit vectors constrained to
x^2 + y^2 + z^2 = 1 - a direction, never a distance. Range is unobservable
from one stationary array, so a Cartesian [x,y,z,vx,vy,vz] state would carry
three components no measurement can correct, and the filter would report a
covariance it has not earned.

    x = [azimuth, elevation, azimuth_rate, elevation_rate]

Range is not estimated, not published, not implied. Sufficient for slew-to-cue:
pointing a camera needs a direction, not a distance.

WHY IT IS AN EKF AND NOT A KF
-----------------------------
The acoustic measurement, converted to spherical angles before it reaches the
filter, is linear - H is a constant selection matrix.

The VISUAL measurement is not. YOLO reports angular error in the CAMERA
optical frame while the state lives in the tracking frame, and the camera
rides the cobot wrist. Relating them means rotating the direction vector
through a time-varying transform and taking arctangents. That needs a
Jacobian - see _h_visual and _H_visual.

AUDIO AND VISUAL ARE INDEPENDENT
---------------------------------
Each measurement callback (_acoustic_cb, _visual_cb) can initialise the
filter alone and update it alone; neither requires the other to be present
or healthy. If the UMA-16/ODAS side goes down, /odas/sst and the classifier
topics simply stop arriving and _acoustic_cb stops firing - _visual_cb keeps
updating from YOLO undisturbed, and vice versa.

audio_enabled and visual_enabled make that explicit and controllable rather
than an accident of which nodes happen to be publishing: each defaults to
True but can be forced off independently (e.g. to bench-test one modality
while the other's launch files still run). They are read once at startup,
like every other parameter here - see USAGE.

TWO CLASSIFIERS, ONE GATE
-------------------------
AST and GRE classify the same separated audio by different routes: a
transformer on 128-bin mel patches every 0.5 s, and a causal GRU on 64-bin
log-mel every 0.1 s.

They are consulted independently and EITHER may open the gate. Requiring
agreement would reject the cases where one is simply outside its comfort zone,
and the point of running two is that they fail differently.

They are NOT equally trustworthy. GRE's published figures - event recall
81.8%, precision 47.4%, ~41 false alarms/hour on n=11, mono-channel - are far
weaker than AST's 0.984 balanced accuracy. Weighting them equally would let a
detector that is wrong half the time tighten R as hard as one that rarely is.
gre_trust scales GRE's confidence before comparison: at 0.5, GRE at 90% counts
the same as AST at 45%.

BOTH sets of numbers were measured on RAW audio. This pipeline feeds ODAS
separated channels, which is out of distribution for both. Re-measure before
trusting either.

ANGLE WRAPPING
--------------
Azimuth is on a circle. Every innovation MUST be wrapped to (-pi, pi] and the
state azimuth re-wrapped after. Without this, a target crossing +/-180 deg
gives an innovation of ~2*pi, the filter reads a violent manoeuvre, and the
estimate is thrown across the sphere - silently, with the covariance still
small.

NO FEEDER
---------
With raw.interface = soundcard in configuration.cfg, ODAS opens the UMA-16
directly and there is no Butterworth stage. /sss becomes full-band, which is
what both classifiers were trained on - but SRP-PHAT now sees content above
the 4083 Hz spatial aliasing ceiling of a 42 mm array. Watch /sst for
instability; that is the cost of this trade.

USAGE
-----
    python3 ekf_fusion_node.py --ros-args -p use_tf_for_visual:=false
    python3 ekf_fusion_node.py --ros-args -p gre_trust:=0.3
    python3 ekf_fusion_node.py --ros-args -p audio_enabled:=false   # visual-only bench test
    python3 ekf_fusion_node.py --ros-args -p visual_enabled:=false  # audio-only bench test
"""

import math
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3Stamped


AZ, EL, AZ_RATE, EL_RATE = 0, 1, 2, 3
N_STATES = 4


def wrap_pi(angle):
    """Wrap to (-pi, pi]. Applied to every azimuth innovation."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def unit_vector(az, el):
    ce = math.cos(el)
    return np.array([ce * math.cos(az), ce * math.sin(az), math.sin(el)])


@dataclass
class Classification:
    is_drone: bool
    confidence: float
    stamp: float
    source: str = ''


class BearingOnlyEKF:
    """The filter proper - no ROS dependencies, so it can be unit-tested."""

    def __init__(self, q_az, q_el, p0_angle, p0_rate):
        self.x = np.zeros(N_STATES)
        self.P = np.eye(N_STATES)
        self.q_az = q_az
        self.q_el = q_el
        self.p0_angle = p0_angle
        self.p0_rate = p0_rate
        self.initialised = False
        self.last_t = None

    def initialise(self, az, el, t):
        """Seed from the first accepted measurement.

        Angles inherit roughly the measurement variance. Rates start at zero
        with a LARGE variance: we know where the target is but nothing about
        how it moves, and pretending otherwise makes the filter reject the
        second measurement as an outlier.
        """
        self.x = np.array([az, el, 0.0, 0.0])
        self.P = np.diag([self.p0_angle, self.p0_angle,
                          self.p0_rate, self.p0_rate])
        self.initialised = True
        self.last_t = t

    def predict(self, t):
        if not self.initialised:
            return
        dt = t - self.last_t
        if dt <= 0.0:
            # Out-of-order or duplicate stamp. Predicting backwards would
            # shrink the covariance illegitimately.
            return
        # Cap a long gap so a sensor dropout does not produce an enormous Q
        # that makes the filter forget everything it knew.
        dt = min(dt, 1.0)
        self.last_t = t

        #        [1  0  dt  0 ]
        #   F =  [0  1  0   dt]
        #        [0  0  1   0 ]
        #        [0  0  0   1 ]
        F = np.eye(N_STATES)
        F[AZ, AZ_RATE] = dt
        F[EL, EL_RATE] = dt

        # Continuous white-noise acceleration, discretised. Per angle:
        #        [dt^3/3   dt^2/2]
        #   Q =  [dt^2/2   dt    ] * q
        # The off-diagonals encode that position error and rate error
        # accumulated over the same interval are correlated. Dropping them - a
        # common shortcut - makes the filter overconfident in its rates, and
        # those correlations are what let a position-only visual measurement
        # correct the rates at all.
        Q = np.zeros((N_STATES, N_STATES))
        dt2, dt3 = dt * dt, dt * dt * dt
        Q[AZ, AZ] = self.q_az * dt3 / 3.0
        Q[AZ, AZ_RATE] = Q[AZ_RATE, AZ] = self.q_az * dt2 / 2.0
        Q[AZ_RATE, AZ_RATE] = self.q_az * dt
        Q[EL, EL] = self.q_el * dt3 / 3.0
        Q[EL, EL_RATE] = Q[EL_RATE, EL] = self.q_el * dt2 / 2.0
        Q[EL_RATE, EL_RATE] = self.q_el * dt

        self.x = F @ self.x
        self.x[AZ] = wrap_pi(self.x[AZ])
        self.P = F @ self.P @ F.T + Q

    def update(self, z, h, H, R, gate_chi2=None):
        """Returns (accepted, nis).

        nis is the normalised innovation squared. A running mean near dim(z)
        means Q and R are consistent with reality; persistently larger means
        the filter is overconfident.
        """
        y = z - h
        y[0] = wrap_pi(y[0])          # azimuth innovation MUST be wrapped
        y[1] = wrap_pi(y[1])

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False, float('inf')

        nis = float(y.T @ S_inv @ y)
        if gate_chi2 is not None and nis > gate_chi2:
            # Outlier. Rejecting protects the track from a reflection or a
            # misassociated detection; the prediction still carries us on.
            return False, nis

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.x[AZ] = wrap_pi(self.x[AZ])

        # Joseph form: keeps P symmetric positive definite even with an
        # imperfect K. The textbook (I - KH)P form degrades over thousands of
        # updates at 86 Hz.
        I_KH = np.eye(N_STATES) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True, nis


class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        p = self.declare_parameter
        self.q_az = p('q_azimuth', 0.5).value
        self.q_el = p('q_elevation', 0.5).value

        self.sigma_acoustic_best = p('sigma_acoustic_best', 0.05).value    # ~2.9 deg
        self.sigma_acoustic_worst = p('sigma_acoustic_worst', 0.30).value  # ~17 deg
        # Optics resolve bearing to a fraction of a degree; the array is
        # limited by nThetas = 181 (~1 deg) and degraded by reverberation.
        self.sigma_visual = p('sigma_visual', 0.005).value                 # ~0.3 deg

        # Chi-squared gates, 2 DOF. 13.8 ~ 99.9%, 9.2 ~ 99%.
        self.gate_acoustic = p('gate_acoustic', 13.8).value
        self.gate_visual = p('gate_visual', 9.2).value

        self.min_confidence = p('min_confidence', 0.5).value
        self.class_timeout = p('classification_timeout', 5.0).value
        self.track_timeout = p('track_timeout', 2.0).value
        # See TWO CLASSIFIERS, ONE GATE above for why this is not 1.0.
        self.gre_trust = p('gre_trust', 0.5).value

        # See AUDIO AND VISUAL ARE INDEPENDENT above.
        self.audio_enabled = p('audio_enabled', True).value
        self.visual_enabled = p('visual_enabled', True).value

        self.tracking_frame = p('tracking_frame', 'odas').value
        self.camera_frame = p('camera_frame', 'd435i_link').value
        self.use_tf_for_visual = p('use_tf_for_visual', True).value

        self.p0_angle = p('p0_angle', 0.25).value
        self.p0_rate = p('p0_rate', 4.0).value

        self.ekf = BearingOnlyEKF(self.q_az, self.q_el,
                                  self.p0_angle, self.p0_rate)
        # Separate caches: a track may be confirmed by one classifier and not
        # the other, and merging them would lose which said what.
        self.ast_class = {}
        self.gre_class = {}
        self.active_track = None
        self.last_accepted = None
        self.stats = {'acoustic_ok': 0, 'acoustic_rej': 0,
                      'visual_ok': 0, 'visual_rej': 0, 'gated_out': 0,
                      'gated_no_class': 0, 'gated_low_conf': 0,
                      'track_locked': 0,
                      'by_ast': 0, 'by_gre': 0}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(
            PointStamped, '/fused_target_pose', qos)

        self.create_subscription(
            PointStamped, '/odas/sst', self._acoustic_cb, qos)
        self.create_subscription(
            Vector3Stamped, '/audio_classifier/detection', self._ast_cb, qos)
        self.create_subscription(
            Vector3Stamped, '/gre_classifier/detection', self._gre_cb, qos)
        self.create_subscription(
            PointStamped, '/camera/yolo_detection', self._visual_cb, qos)

        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'bearing-only EKF | q=({self.q_az}, {self.q_el}) | '
            f'sigma_ac={self.sigma_acoustic_best}-{self.sigma_acoustic_worst} '
            f'sigma_vis={self.sigma_visual} | gre_trust={self.gre_trust} | '
            f'audio_enabled={self.audio_enabled} '
            f'visual_enabled={self.visual_enabled}')

    # ------------------------------------------------------------------
    # Measurement models
    # ------------------------------------------------------------------

    @staticmethod
    def _H_acoustic():
        """Acoustic Jacobian.

        The unit vector is converted to (azimuth, elevation) BEFORE reaching
        the filter, so the measurement is the first two state components:

            h(x) = [az, el]
            H    = [1 0 0 0]
                   [0 1 0 0]

        Converting outside the filter keeps this exact rather than linearised,
        so the acoustic path contributes no approximation error at all.
        """
        H = np.zeros((2, N_STATES))
        H[0, AZ] = 1.0
        H[1, EL] = 1.0
        return H

    def _camera_rotation(self):
        """Rotation tracking_frame -> camera_frame, or None.

        The camera rides the wrist, so this changes continuously and must be
        looked up per measurement rather than cached.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.tracking_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'no {self.tracking_frame} -> {self.camera_frame} tf: {e}',
                throttle_duration_sec=10.0)
            return None
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def _h_visual(self, R_c, az, el):
        """Predicted YOLO measurement: angular offset from the optical axis.

        Boresight is +Z of camera_frame, so a centred target gives
        u_cam = [0, 0, 1] and both offsets are zero.
        """
        u_cam = R_c @ unit_vector(az, el)
        ux, uy, uz = u_cam
        return np.array([math.atan2(ux, uz), math.atan2(uy, uz)]), u_cam

    def _H_visual(self, R_c, az, el, u_cam):
        """Visual Jacobian, by the chain rule.

            dh/dstate = (dh/du_cam) @ R_c @ (du_world/dstate)

        Term 1 - arctangent derivatives. With e_x = atan2(ux, uz):
            de_x/dux =  uz / (ux^2 + uz^2)
            de_x/duz = -ux / (ux^2 + uz^2)
        symmetrically for e_y in (uy, uz).

        Term 2 - R_c, constant w.r.t. the state at this instant.

        Term 3 - the direction vector's sensitivity to the angles:
            du/daz = [-cos(el)sin(az),  cos(el)cos(az), 0     ]
            du/del = [-sin(el)cos(az), -sin(el)sin(az), cos(el)]

        The rate columns are zero: YOLO observes position, not velocity. The
        rates are still corrected indirectly, through the position-rate
        correlation P carries from the prediction step.
        """
        ux, uy, uz = u_cam
        eps = 1e-9
        # Guard the singularity at u_cam ~ [0, +-1, 0] - target exactly on the
        # camera's lateral axis. Outside any real FOV, but the arithmetic must
        # not explode if it occurs.
        dx = max(ux * ux + uz * uz, eps)
        dy = max(uy * uy + uz * uz, eps)

        J_ang = np.array([
            [uz / dx, 0.0, -ux / dx],
            [0.0, uz / dy, -uy / dy],
        ])

        sa, ca = math.sin(az), math.cos(az)
        se, ce = math.sin(el), math.cos(el)
        J_state = np.zeros((3, N_STATES))
        J_state[:, AZ] = np.array([-ce * sa, ce * ca, 0.0])
        J_state[:, EL] = np.array([-se * ca, -se * sa, ce])

        return J_ang @ R_c @ J_state

    def _acoustic_R(self, confidence):
        """Measurement covariance scaled by classifier confidence.

        Linear interpolation between the best and worst sigma. A confident
        verdict tightens R and lets the bearing pull harder; a marginal one
        widens it so the filter leans on its prediction and on vision.

        Interpolating SIGMA - not variance, and not 1/confidence - keeps the
        mapping bounded and monotonic. Dividing by confidence, the tempting
        shortcut, sends R to infinity as confidence approaches zero and makes
        the filter discard an entire modality over one bad frame.
        """
        c = min(max(confidence, 0.0), 1.0)
        sigma = (self.sigma_acoustic_worst
                 + c * (self.sigma_acoustic_best - self.sigma_acoustic_worst))
        return np.eye(2) * (sigma ** 2)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _ast_cb(self, msg: Vector3Stamped):
        track_id = int(round(msg.vector.x))
        self.ast_class[track_id] = Classification(
            is_drone=(msg.vector.y >= 0.5),
            confidence=float(msg.vector.z),
            stamp=self._stamp_to_sec(msg.header.stamp),
            source='AST')

    def _gre_cb(self, msg: Vector3Stamped):
        track_id = int(round(msg.vector.x))
        self.gre_class[track_id] = Classification(
            is_drone=(msg.vector.y >= 0.5),
            confidence=float(msg.vector.z),
            stamp=self._stamp_to_sec(msg.header.stamp),
            source='GRE')

    def _lookup_class(self, track_id, now):
        """Best live verdict for a track, across both classifiers.

        Either may open the gate - they read the same signal by different
        routes, so requiring agreement would reject the cases where one is
        simply outside its comfort zone.

        GRE's confidence is scaled by gre_trust before comparison. Verdicts
        expire: at 86 Hz a bearing arrives ~86 times per 1 Hz classification,
        so a stale positive would otherwise gate updates through indefinitely.
        """
        best = None
        for store, weight in ((self.ast_class, 1.0),
                              (self.gre_class, self.gre_trust)):
            c = store.get(track_id)
            if c is None:
                continue
            if now - c.stamp > self.class_timeout:
                del store[track_id]
                continue
            if not c.is_drone:
                continue
            scaled = Classification(True, c.confidence * weight,
                                    c.stamp, c.source)
            if best is None or scaled.confidence > best.confidence:
                best = scaled
        return best

    def _acoustic_cb(self, msg: PointStamped):
        """ODAS bearing. frame_id carries the track id as a string."""
        if not self.audio_enabled:
            return
        t = self._stamp_to_sec(msg.header.stamp)

        try:
            track_id = int(msg.header.frame_id)
        except (ValueError, TypeError):
            self.get_logger().warn(
                f'frame_id "{msg.header.frame_id}" is not a track id',
                throttle_duration_sec=10.0)
            return

        c = self._lookup_class(track_id, t)
        if c is None:
            self.stats['gated_out'] += 1
            self.stats['gated_no_class'] += 1
            return
        if c.confidence < self.min_confidence:
            self.stats['gated_out'] += 1
            self.stats['gated_low_conf'] += 1
            return

        # Follow one track at a time. Switching only after the current one has
        # gone quiet - otherwise two simultaneous sources (a drone and its
        # reflection) drag the estimate back and forth.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.active_track is not None and track_id != self.active_track:
            if (self.last_accepted is not None
                    and now - self.last_accepted < self.track_timeout):
                self.stats['track_locked'] += 1
                return
            self.get_logger().info(
                f'switching track {self.active_track} -> {track_id}')
            self.active_track = None

        v = np.array([msg.point.x, msg.point.y, msg.point.z])
        n = np.linalg.norm(v)
        if n < 1e-6:
            return
        v /= n
        az = math.atan2(v[1], v[0])
        el = math.asin(max(-1.0, min(1.0, v[2])))

        if not self.ekf.initialised:
            self.ekf.initialise(az, el, t)
            self.active_track = track_id
            self.last_accepted = now
            self.get_logger().info(
                f'initialised on track {track_id} via {c.source}: '
                f'az {math.degrees(az):+.1f} el {math.degrees(el):+.1f}')
            self._publish(msg.header.stamp)
            return

        self.ekf.predict(t)
        ok, nis = self.ekf.update(
            z=np.array([az, el]),
            h=np.array([self.ekf.x[AZ], self.ekf.x[EL]]),
            H=self._H_acoustic(),
            R=self._acoustic_R(c.confidence),
            gate_chi2=self.gate_acoustic)

        if ok:
            self.stats['acoustic_ok'] += 1
            self.stats['by_ast' if c.source == 'AST' else 'by_gre'] += 1
            self.active_track = track_id
            self.last_accepted = now
            self._publish(msg.header.stamp)
        else:
            self.stats['acoustic_rej'] += 1

    def _visual_cb(self, msg: PointStamped):
        """YOLO angular error. frame_id is 'drone' or 'none'."""
        if not self.visual_enabled or msg.header.frame_id != 'drone':
            return

        t = self._stamp_to_sec(msg.header.stamp)
        z = np.array([msg.point.x, msg.point.y])

        R_c = self._camera_rotation() if self.use_tf_for_visual else None

        if not self.ekf.initialised:
            # Vision can initialise alone, but only with a known camera pose -
            # otherwise there is no way to place a camera-frame offset on the
            # sphere.
            if R_c is None:
                return
            # Boresight in the tracking frame is R_c^T @ [0,0,1] - the third
            # ROW of R_c. Offset from there by the measurement.
            bore = R_c[2, :]
            az0 = math.atan2(bore[1], bore[0]) + z[0]
            el0 = math.asin(max(-1.0, min(1.0, bore[2]))) + z[1]
            self.ekf.initialise(wrap_pi(az0), el0, t)
            self.last_accepted = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info('initialised from vision')
            self._publish(msg.header.stamp)
            return

        self.ekf.predict(t)

        if R_c is not None:
            h, u_cam = self._h_visual(R_c, self.ekf.x[AZ], self.ekf.x[EL])
            H = self._H_visual(R_c, self.ekf.x[AZ], self.ekf.x[EL], u_cam)
        else:
            # DEGRADED FALLBACK - no TF available.
            #
            # Assume the boresight coincides with the current estimate, which
            # is approximately true while the servo loop is converged. The
            # measurement then reduces to a direct angular correction.
            #
            # This couples the measurement model to the estimate it is meant
            # to correct, so errors can reinforce rather than cancel. It keeps
            # the system running without TF; it is not a substitute for
            # publishing the camera transform.
            h = np.array([self.ekf.x[AZ], self.ekf.x[EL]])
            z = np.array([self.ekf.x[AZ] + z[0], self.ekf.x[EL] + z[1]])
            H = self._H_acoustic()

        ok, nis = self.ekf.update(
            z=z, h=h, H=H,
            R=np.eye(2) * (self.sigma_visual ** 2),
            gate_chi2=self.gate_visual)

        if ok:
            self.stats['visual_ok'] += 1
            self.last_accepted = self.get_clock().now().nanoseconds * 1e-9
            self._publish(msg.header.stamp)
        else:
            self.stats['visual_rej'] += 1

    # ------------------------------------------------------------------

    def _publish(self, stamp):
        """x = azimuth, y = elevation (radians), z = 1.0 fixed.

        z is a unit direction, NOT a range. Any consumer treating it as depth
        is misreading this message.
        """
        out = PointStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.tracking_frame
        out.point.x = float(self.ekf.x[AZ])
        out.point.y = float(self.ekf.x[EL])
        out.point.z = 1.0
        self.pub.publish(out)

    def _report(self):
        s = self.stats
        if not self.ekf.initialised:
            self.get_logger().info(
                f'waiting for a classified drone bearing | '
                f'gated {s["gated_out"]} '
                f'(no-class {s["gated_no_class"]}, '
                f'low-confidence {s["gated_low_conf"]}) | '
                f'AST cache {len(self.ast_class)} '
                f'GRE cache {len(self.gre_class)}')
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self.last_accepted if self.last_accepted else float('inf')
        self.get_logger().info(
            f'az {math.degrees(self.ekf.x[AZ]):+7.2f} '
            f'el {math.degrees(self.ekf.x[EL]):+6.2f} deg | '
            f'rates {math.degrees(self.ekf.x[AZ_RATE]):+6.1f} '
            f'{math.degrees(self.ekf.x[EL_RATE]):+5.1f} deg/s | '
            f'sigma {math.degrees(math.sqrt(self.ekf.P[AZ, AZ])):.2f} deg | '
            f'ac {s["acoustic_ok"]}/{s["acoustic_ok"] + s["acoustic_rej"]} '
            f'(AST {s["by_ast"]} GRE {s["by_gre"]}) '
            f'gate(no-class {s["gated_no_class"]}, '
            f'low-conf {s["gated_low_conf"]}, '
            f'track-lock {s["track_locked"]}) '
            f'vis {s["visual_ok"]}/{s["visual_ok"] + s["visual_rej"]} | '
            f'last {age:.1f}s')

        # Coasting on prediction alone. The estimate is still published but
        # its covariance is growing; treat it as increasingly unreliable.
        if age > self.track_timeout:
            self.get_logger().warn(
                f'no accepted measurement for {age:.1f}s - coasting')


def main():
    rclpy.init()
    node = EkfFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
