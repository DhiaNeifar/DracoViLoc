#!/usr/bin/env python3
"""
ast_classifier_node.py  -  drone classification on ODAS separated audio

Subscribes to /sss (odas_ros separated sources) and /sst (tracked bearings),
runs the AST TensorRT engine per separated channel, and publishes a per-track
verdict for the fusion stage.

    /sss (4ch audio) ─┐
                      ├─► AST ─► /audio_classifier/detection
    /sst (track ids) ─┘          [track_id, is_drone, confidence]

BAND WARNING
------------
/sss contains whatever the feeder sent ODAS. ODAS separates the /raw stream; it
cannot restore frequencies the Butterworth already removed. With the feeder at
180-3600 Hz the separated channels hold NO energy above 3600 Hz, and the AST
model - which expects 3000-8000 Hz - receives a 600 Hz sliver.

    Run the feeder wide:   python3 uma16_feeder_node.py --lo 180 --hi 7800

Then RE-VERIFY /sst. Content above the 4083 Hz aliasing ceiling of a 42 mm
array corrupts SRP-PHAT, so widening the band trades DOA quality for classifier
input. If tracking becomes unstable, the answer is beamforming an unfiltered
stream toward each /sst bearing instead of using ODAS SSS at all.

CUDA CONTEXT
------------
detect_drone_realtime.py creates its CUDA context via `import pycuda.autoinit`
in its own startup path. Importing TRTEngine from it does NOT trigger that, so
cuda.Stream() fails with:

    LogicError: explicit_context_dependent failed: invalid device context

autoinit is therefore imported explicitly below, BEFORE TRTEngine is
constructed. The context it creates is bound to the importing thread; rclpy
callbacks may run on a different one, so _classify pushes and pops it around
each inference. Pushing an already-current context is legal - CUDA keeps a
stack - so this is safe either way.

TRACK ASSOCIATION
-----------------
/sss is a bare 4-channel AudioFrame with no source ids - SssSocketServer adds
none. Channel count comes from len(sst.N_inactive).

ODAS emits separated channels in the same slot order as SST tracks, so
channel i corresponds to sources[i] of the concurrent /sst message. This node
caches the latest /sst and tags each channel's verdict with that slot's id.

Slots with id == 0 are empty and are skipped. Tracks below --min-activity are
skipped too: N_inactive = 250 keeps dead tracks alive for ~2.9 s at activity 0.

MODEL INTERFACE (model/config.json + preprocessor_config.json)
--------------------------------------------------------------
    sampling_rate  16000 Hz      <- /sss is 44100, resampled 160/441 here
    window         1.0 s = 16000 samples
    hop            0.5 s = 8000 samples
    features       128 mel bins x 128 frames
    labels         0 = no_drone, 1 = drone

USAGE
-----
    source /opt/ros/humble/setup.bash
    source ~/odas_ws/install/setup.bash
    source ~/trt_env/bin/activate            # venv AFTER ros
    python3 ast_classifier_node.py --project-dir ~/drone \\
                                   --engine ~/drone/drone_ast.engine

OUTPUT
------
std_msgs/String carrying JSON, one message per classified window:

    {"stamp": 1787..., "track_id": 33, "channel": 2,
     "is_drone": true, "confidence": 0.873, "consecutive": 3}

JSON avoids a custom msg package while testing. Replace with a proper
dracoviloc_msgs/AudioDetection once the interface settles - the EKF wants
typed fields.
"""

import argparse
import json
import sys
import traceback
from pathlib import Path

print('[AST] Python process started; importing dependencies...', flush=True)
try:
    import numpy as np
    from scipy.signal import resample_poly

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

    from geometry_msgs.msg import Vector3Stamped
    from audio_utils_msgs.msg import AudioFrame
    from odas_ros_msgs.msg import OdasSstArrayStamped
except BaseException:
    print('[AST] DEPENDENCY IMPORT FAILED:', file=sys.stderr, flush=True)
    traceback.print_exc()
    raise
print('[AST] Dependencies imported.', flush=True)


AST_SR = 16000
WINDOW_SAMPLES = 16000      # 1.0 s
HOP_SAMPLES = 8000          # 0.5 s
# /sss fS is 44100 in configuration.cfg; 16000/44100 reduces to 160/441.
RESAMPLE_UP = 160
RESAMPLE_DOWN = 441


class AstClassifierNode(Node):
    def __init__(self, args):
        super().__init__('ast_classifier_node')
        self.args = args

        sys.path.insert(0, str(args.project_dir))
        print('[AST] Initializing CUDA and TensorRT runtime...', flush=True)
        try:
            # MUST come before TRTEngine - it creates the CUDA context that
            # cuda.Stream() needs. See the CUDA CONTEXT note above.
            import pycuda.autoinit
            from detect_drone_realtime import TRTEngine
            import ast_patch                     # numpy shim, MUST precede the extractor
            from transformers import ASTFeatureExtractor
        except ImportError as e:
            raise SystemExit(
                f'[ast] cannot import TRT/transformers: {e}\n'
                f'[ast] activate the venv AFTER sourcing ROS, and check\n'
                f'[ast]   grep system-site ~/trt_env/pyvenv.cfg   -> must be true')

        self.cuda_ctx = pycuda.autoinit.context
        print('[AST] CUDA runtime ready.', flush=True)

        self.get_logger().info(f'loading engine {args.engine}')
        self.engine = TRTEngine(str(args.engine))
        print('[AST] TensorRT engine loaded.', flush=True)

        model_dir = args.model_dir or (args.project_dir / 'model')
        print(f'[AST] Loading feature extractor from {model_dir}...', flush=True)
        self.extractor = ASTFeatureExtractor.from_pretrained(str(model_dir))
        self.get_logger().info(
            f'engine input {self.engine.input_shape} '
            f'output {self.engine.output_shape}')

        # --- state -------------------------------------------------------
        self.n_ch = args.channels
        self.buffers = [np.zeros(0, dtype=np.float32) for _ in range(self.n_ch)]
        # State tracking for all channels
        self.streaks = [0] * self.n_ch
        self.latest_conf = [0.0] * self.n_ch
        self.latest_verdict = [False] * self.n_ch
        self.latest_sst = None
        self.windows_done = 0
        self.sss_seen = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(
            Vector3Stamped, '/audio_classifier/detection', qos)
        self.create_subscription(AudioFrame, '/sss', self._sss_cb, qos)
        self.create_subscription(
            OdasSstArrayStamped, '/sst', self._sst_cb, qos)

        self.create_timer(5.0, self._status)
        self.get_logger().info(
            f'{self.n_ch} channels, threshold {args.threshold}, '
            f'{args.consecutive} consecutive windows')
        print('[AST] READY - confidence will print for every classified window.',
              flush=True)

    def _status(self):
        """Name the reason nothing is being classified, rather than sit mute."""
        if self.windows_done > 0:
            return
        if self.sss_seen == 0:
            self.get_logger().warn(
                'no /sss messages - is ODAS running? check: ros2 topic hz /sss')
        elif self.latest_sst is None:
            self.get_logger().warn(
                f'{self.sss_seen} /sss frames but no /sst yet')
        else:
            live = [s.id for s in self.latest_sst.sources
                    if s.id != 0 and s.activity >= self.args.min_activity]
            self.get_logger().warn(
                f'{self.sss_seen} /sss frames, no usable track '
                f'(live ids: {live}, min_activity={self.args.min_activity})')

    def _sst_cb(self, msg):
        self.latest_sst = msg

    def _track_id_for(self, ch):
        """Slot index -> (track id, activity). id 0 means the slot is empty.

        --always-classify bypasses ODAS's track/activity gate entirely: every
        channel is treated as active, using a negative synthetic id (-(ch+1))
        when there is no real ODAS track. Negative ids never collide with a
        real /odas/sst track id, so ekf_fusion_node's association by id still
        behaves correctly if this is ever run with the EKF attached - it just
        won't gate on these classifications, same as if AST published nothing
        for that channel. This exists to validate the model/engine end to end
        without depending on ODAS ever forming a track.
        """
        if self.latest_sst is not None and ch < len(self.latest_sst.sources):
            s = self.latest_sst.sources[ch]
            if s.id != 0:
                return s.id, s.activity
        if self.args.always_classify:
            return -(ch + 1), 1.0
        return 0, 0.0

    def _sss_cb(self, msg: AudioFrame):
        self.sss_seen += 1

        if msg.channel_count != self.n_ch:
            self.get_logger().warn(
                f'expected {self.n_ch} channels, got {msg.channel_count}; '
                f'--channels must equal len(sst.N_inactive)',
                throttle_duration_sec=10.0)
            return

        # sss.separated is nBits = 16 in configuration.cfg -> int16.
        pcm = np.frombuffer(bytes(msg.data), dtype='<i2')
        if pcm.size == 0:
            return
        frames = pcm.reshape(-1, self.n_ch).astype(np.float32) / 32768.0

        for ch in range(self.n_ch):
            track_id, activity = self._track_id_for(ch)
            if track_id == 0 or activity < self.args.min_activity:
                # Empty or dead slot. Drop any partial buffer so a track that
                # later reappears in this slot does not inherit the previous
                # source's audio - that would splice two different sources
                # into a single classification window.
                self.buffers[ch] = np.zeros(0, dtype=np.float32)
                self.streaks[ch] = 0
                self.latest_conf[ch] = 0.0
                self.latest_verdict[ch] = False
                continue

            chunk = resample_poly(frames[:, ch], RESAMPLE_UP, RESAMPLE_DOWN)
            self.buffers[ch] = np.concatenate([self.buffers[ch], chunk])

            while self.buffers[ch].size >= WINDOW_SAMPLES:
                window = self.buffers[ch][:WINDOW_SAMPLES]
                self.buffers[ch] = self.buffers[ch][HOP_SAMPLES:]
                self._classify(ch, track_id, window, msg.header.stamp)

    def _classify(self, ch, track_id, window, stamp):
        feats = self.extractor(
            window, sampling_rate=AST_SR, return_tensors='np')
        x = np.asarray(feats['input_values'], dtype=np.float32)

        if tuple(x.shape) != tuple(self.engine.input_shape):
            self.get_logger().error(
                f'feature shape {x.shape} != engine {self.engine.input_shape}',
                throttle_duration_sec=10.0)
            return

        # rclpy callbacks may run off the thread that owns the context.
        self.cuda_ctx.push()
        try:
            logits = np.asarray(self.engine.infer(x)).reshape(-1)
        finally:
            self.cuda_ctx.pop()

        # Stable softmax - raw exp overflows on confident logits.
        e = np.exp(logits - logits.max())
        prob_drone = float((e / e.sum())[1])      # config.json: 1 = drone

        self.windows_done += 1

        if prob_drone >= self.args.threshold:
            self.streaks[ch] += 1
        else:
            self.streaks[ch] = 0

        is_drone = self.streaks[ch] >= self.args.consecutive
        self.latest_conf[ch] = prob_drone
        self.latest_verdict[ch] = is_drone

        out = Vector3Stamped()
        out.header.stamp = stamp
        out.header.frame_id = 'odas'
        out.vector.x = float(track_id)              # track id
        out.vector.y = 1.0 if is_drone else 0.0     # verdict
        out.vector.z = float(prob_drone)            # confidence
        self.pub.publish(out)

        # Multi-channel status display across all channels
        parts = []
        for i in range(self.n_ch):
            tid, act = self._track_id_for(i)
            if tid == 0 or act < self.args.min_activity:
                parts.append(f"[CH {i}] IDLE")
            else:
                conf = self.latest_conf[i] * 100.0
                tag = " [DRONE]" if self.latest_verdict[i] else ""
                parts.append(f"[CH {i} (ID {tid})] {conf:5.1f}%{tag}")
        self.get_logger().info("AST confidence | " + " | ".join(parts))



def parse_args(argv):
    p = argparse.ArgumentParser()
    p.add_argument('--project-dir', type=Path,
                   default=Path.home() / 'drone',
                   help='holds detect_drone_realtime.py (and model/ by default)')
    p.add_argument('--model-dir', type=Path, default=None,
                   help='override if model/ is not under --project-dir')
    p.add_argument('--engine', type=Path,
                   default=Path.home() / 'drone' / 'drone_ast.engine')
    p.add_argument('--channels', type=int, default=4,
                   help='must equal len(sst.N_inactive) in configuration.cfg')
    p.add_argument('--threshold', type=float, default=0.5)
    p.add_argument('--consecutive', type=int, default=3)
    p.add_argument('--min-activity', type=float, default=0.3,
                   help='skip slots whose SST track is below this')
    p.add_argument('--always-classify', type=str, default='false',
                   choices=['true', 'false'],
                   help='true: classify every /sss channel continuously, '
                        'ignoring whether ODAS has formed a track. For '
                        'validating the model/engine independent of ODAS '
                        'SSL/SST tuning; classifications on channels with no '
                        'real track use a synthetic negative id and will not '
                        'gate the EKF. String, not a flag, so ExecuteProcess '
                        'launch arguments can pass it unconditionally.')
    args = p.parse_args(argv)
    args.always_classify = args.always_classify == 'true'
    return args


def main():
    node = None
    try:
        argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
        args = parse_args(argv)
        print(
            f'[AST] Starting: engine={args.engine} threshold={args.threshold} '
            f'min_activity={args.min_activity} always_classify={args.always_classify}',
            flush=True)
        rclpy.init()
        node = AstClassifierNode(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException:
        print('[AST] FATAL STARTUP/RUNTIME ERROR:', file=sys.stderr, flush=True)
        traceback.print_exc()
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
