#!/usr/bin/env python3
"""
gre_classifier_node.py  -  GRE drone detection on ODAS separated audio

Marouane's causal GRU detector (rt_drone_detector), one instance per separated
source, publishing per-track verdicts for the EKF.

    /sss (4ch audio) ─┐
                      ├─► GRE ─► /gre_classifier/detection
    /sst (track ids) ─┘          [track_id, is_drone, confidence]

Note the topic: /gre_classifier/detection, NOT /audio_classifier/detection.
AST owns that one. Two publishers on a single topic would make the verdicts
indistinguishable to the EKF, which caches one classification per track id.

ONE DETECTOR PER CHANNEL - THIS IS THE WHOLE DESIGN
---------------------------------------------------
The model is a CAUSAL GRU: it carries a hidden state h from one 100 ms
decision to the next, and TensorRTDroneDetector also holds the streaming
feature extractor's filter state and the hysteresis decision state.

That state is per-source. Four separated channels therefore need FOUR
independent detector instances. Sharing one would interleave four sources'
histories into a single recurrent state and the output would be meaningless.

The engine is tiny (GRU, 32 hidden units, 1 layer), so four copies cost very
little memory - unlike the AST transformer, where this would matter.

STATE RESET ON TRACK LOSS
-------------------------
When an SST slot goes empty, its detector must forget everything: hidden
state, feature extractor state, hysteresis state. Otherwise the next track to
occupy that slot inherits the previous source's recurrent history, and its
first second of classification is contaminated.

TensorRTDroneDetector exposes no reset(), so this node reinstantiates the
detector for that channel. That reloads the engine, which is why resets are
rate-limited by --reset-cooldown - a track flickering in and out would
otherwise reload engines continuously.

Worth asking Marouane for a reset() method that zeroes _h, the extractor
state and _decision_state in place. It would make this both cheaper and less
fragile than reconstruction.

SAMPLE RATE - DIFFERENT FROM AST
--------------------------------
model_logmel_meta.json says sample_rate = 24000, where AST wants 16000.
/sss is 44100, so the ratio here is 24000/44100 = 80/147 (both reduce by 300).
Using AST's 160/441 would feed the model audio at 1.5x speed and every
rotor harmonic would land in the wrong mel band.

DECISION LOGIC ALREADY INCLUDED
-------------------------------
push_audio() returns dicts already carrying the hysteresis decision from
meta.json: threshold_on 0.1, threshold_off 0.05, min_presence_s 0.1,
min_absence_s 0.1. This node does NOT add its own streak counter the way the
AST node does - that would be a second debounce stacked on a tuned one, and
would only delay detections.

vector.y carries that decision; vector.z carries the raw score, published
every frame regardless. The EKF scales its measurement covariance by the
score, so a marginal detection and a confident one must stay distinguishable.

CALIBRATION CONTEXT - READ BEFORE TRUSTING THE NUMBERS
------------------------------------------------------
The published figures (event recall 81.8%, precision 47.4%, ~41 false
alarms/hour) come from a MONO-CHANNEL detector on RAW audio, n=11 events.

This node feeds it something different: blind-separated audio from ODAS,
band-limited by the feeder's Butterworth, with inter-source leakage. That is
out of distribution relative to training. It will produce scores; those scores
are not the calibrated ones. Re-measure before relying on the thresholds.

USAGE
-----
    source /opt/ros/humble/setup.bash
    source ~/odas_ws/install/setup.bash
    source ~/gre_env/bin/activate            # venv AFTER ros
    python3 gre_classifier_node.py --repo ~/project/GRE/rt_drone_detector
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3Stamped
from audio_utils_msgs.msg import AudioFrame
from odas_ros_msgs.msg import OdasSstArrayStamped


# /sss fS is 44100; the model wants 24000. 44100 = 147*300, 24000 = 80*300.
RESAMPLE_UP = 80
RESAMPLE_DOWN = 147

# OVERLAP-SAVE RESAMPLING - do not resample each 512-sample /sss hop directly.
#
# resample_poly is a ~1470-tap FIR. Called on a bare 512-sample block it
# assumes silence before the block and zero-pads, so every call produces a
# startup transient - 86 discontinuities per second at the /sss rate. Worse,
# 512 * 80 / 147 = 278.64 is not an integer, so the rounding drifts the output
# time grid from one call to the next.
#
# AST hides this in a 1 s averaged spectrogram. GRE is a causal GRU: each
# decision carries the previous one's hidden state, so these errors accumulate
# instead of averaging out.
#
# The fix is to only ever resample raw-audio blocks that are an exact multiple
# of RESAMPLE_DOWN, and to pad each block on BOTH sides with RESAMPLE_BLOCK
# raw samples of context before resampling, keeping only the resampled middle
# third that corresponds to the block itself.
#
# Both sides, not just the past: resample_poly's default filter is zero-phase
# (symmetric), so an output sample depends on ~RESAMPLE_BLOCK raw samples on
# EITHER side of it, not only on what came before. Priming with past history
# alone still leaves the last stretch of every block wrong, because those
# outputs need raw samples that have not arrived yet - verified numerically
# (up to 0.36 absolute error on a unit-variance test signal with history-only
# priming, 0 with both sides added). With both sides added, the result is
# bit-exact to resampling the raw stream in one continuous call.
#
# Cost: a block can only be finalized once RESAMPLE_BLOCK more raw samples
# (its lookahead) exist after it, so the first sample of a block waits up to
# 2*RESAMPLE_BLOCK raw samples - 2*1470/44100 =~ 67 ms - against a 100 ms GRE
# decision period.
RESAMPLE_BLOCK = RESAMPLE_DOWN * 10  # 1470 raw samples
RESAMPLE_HOP = RESAMPLE_BLOCK * RESAMPLE_UP // RESAMPLE_DOWN  # 800, exact


class GreClassifierNode(Node):
    def __init__(self, args):
        super().__init__('gre_classifier_node')
        self.args = args

        repo = args.repo
        sys.path.insert(0, str(repo / 'shared'))
        try:
            import pycuda.autoinit          # creates the CUDA context
            import utils
            from realtime_trt import TensorRTDroneDetector
        except ImportError as e:
            raise SystemExit(
                f'[gre] import failed: {e}\n'
                f'[gre] activate ~/gre_env AFTER sourcing ROS.\n'
                f'[gre] if this is a numpy _DTypeMeta error, downgrade the\n'
                f'[gre] offending package - numpy 1.26 is fixed by JetPack.')

        self.cuda_ctx = pycuda.autoinit.context
        self._DetectorClass = TensorRTDroneDetector

        self.config = utils.load_config(str(repo / 'config' / 'default.yaml'))
        self.engine_path = args.engine or (repo / 'model' / 'model_logmel.engine')
        self.meta_path = args.meta or (repo / 'model' / 'model_logmel_meta.json')

        if not Path(self.engine_path).exists():
            raise SystemExit(
                f'[gre] no engine at {self.engine_path}\n'
                f'[gre] build it - engines are machine specific:\n'
                f'[gre]   trtexec --onnx=model/model_logmel.onnx \\\n'
                f'[gre]           --saveEngine=model/model_logmel.engine --fp16')

        self.n_ch = args.channels
        self.get_logger().info(
            f'loading {self.n_ch} detectors from {self.engine_path}')
        self.detectors = [self._new_detector() for _ in range(self.n_ch)]

        # --- state -------------------------------------------------------
        # Raw (pre-resample) samples per channel accumulated across /sss
        # messages: the not-yet-finalized block plus, once enough has
        # arrived, its lookahead - see RESAMPLE_BLOCK above.
        self.raw_pending = [np.zeros(0, dtype=np.float32) for _ in range(self.n_ch)]
        # Last RESAMPLE_BLOCK raw samples actually resampled per channel, kept
        # only to prime the FIR's past side for the next block. Zero at
        # start/reset: a one-time startup transient on the first block, not a
        # per-call one.
        self.resample_history = [
            np.zeros(RESAMPLE_BLOCK, dtype=np.float32) for _ in range(self.n_ch)]
        self.t_elapsed = [0.0] * self.n_ch
        self.active_id = [0] * self.n_ch
        self.last_reset = [0.0] * self.n_ch
        self.latest_sst = None
        self.frames_done = 0
        self.sss_seen = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(
            Vector3Stamped, '/gre_classifier/detection', qos)
        self.create_subscription(AudioFrame, '/sss', self._sss_cb, qos)
        self.create_subscription(
            OdasSstArrayStamped, '/sst', self._sst_cb, qos)

        self.create_timer(5.0, self._status)
        self.get_logger().info(
            f'{self.n_ch} channels, 44100 -> 24000 Hz '
            f'({RESAMPLE_UP}/{RESAMPLE_DOWN})')

    def _new_detector(self):
        return self._DetectorClass(
            str(self.engine_path), str(self.meta_path), self.config)

    def _status(self):
        if self.frames_done > 0:
            return
        if self.sss_seen == 0:
            self.get_logger().warn(
                'no /sss - is ODAS running? check: ros2 topic hz /sss')
        elif self.latest_sst is None:
            self.get_logger().warn(f'{self.sss_seen} /sss frames, no /sst yet')
        else:
            live = [s.id for s in self.latest_sst.sources
                    if s.id != 0 and s.activity >= self.args.min_activity]
            self.get_logger().warn(
                f'{self.sss_seen} /sss frames, no usable track '
                f'(live ids: {live}, min_activity={self.args.min_activity})')

    def _sst_cb(self, msg):
        self.latest_sst = msg

    def _track_id_for(self, ch):
        if self.latest_sst is None or ch >= len(self.latest_sst.sources):
            return 0, 0.0
        s = self.latest_sst.sources[ch]
        return s.id, s.activity

    def _reset_channel(self, ch, now):
        """Wipe a channel's recurrent state. Rate-limited - see the header."""
        self.raw_pending[ch] = np.zeros(0, dtype=np.float32)
        self.resample_history[ch] = np.zeros(RESAMPLE_BLOCK, dtype=np.float32)
        self.t_elapsed[ch] = 0.0
        self.active_id[ch] = 0
        if now - self.last_reset[ch] < self.args.reset_cooldown:
            return
        self.last_reset[ch] = now
        self.cuda_ctx.push()
        try:
            self.detectors[ch] = self._new_detector()
        finally:
            self.cuda_ctx.pop()

    def _sss_cb(self, msg: AudioFrame):
        self.sss_seen += 1
        now = self.get_clock().now().nanoseconds * 1e-9

        if msg.channel_count != self.n_ch:
            self.get_logger().warn(
                f'expected {self.n_ch} channels, got {msg.channel_count}; '
                f'--channels must equal len(sst.N_inactive)',
                throttle_duration_sec=10.0)
            return

        # sss.separated is nBits = 16 -> int16.
        pcm = np.frombuffer(bytes(msg.data), dtype='<i2')
        if pcm.size == 0:
            return
        frames = pcm.reshape(-1, self.n_ch).astype(np.float32) / 32768.0

        for ch in range(self.n_ch):
            track_id, activity = self._track_id_for(ch)

            if track_id == 0 or activity < self.args.min_activity:
                if self.active_id[ch] != 0:
                    self._reset_channel(ch, now)
                continue

            # A DIFFERENT track now occupies this slot. Same reasoning as an
            # empty slot: the GRU must not carry the old source's history.
            if self.active_id[ch] not in (0, track_id):
                self._reset_channel(ch, now)
            self.active_id[ch] = track_id

            self.raw_pending[ch] = np.concatenate(
                [self.raw_pending[ch], frames[:, ch]]).astype(np.float32)

            hops = []
            while len(self.raw_pending[ch]) >= 2 * RESAMPLE_BLOCK:
                block = self.raw_pending[ch][:RESAMPLE_BLOCK]
                lookahead = self.raw_pending[ch][RESAMPLE_BLOCK:2 * RESAMPLE_BLOCK]
                # Drop only the finalized block; lookahead stays in the
                # buffer to become the start of a future block.
                self.raw_pending[ch] = self.raw_pending[ch][RESAMPLE_BLOCK:]

                primed = np.concatenate(
                    [self.resample_history[ch], block, lookahead])
                resampled = resample_poly(primed, RESAMPLE_UP, RESAMPLE_DOWN)
                # Keep only the middle third: the part fully supported by
                # real raw samples on both sides.
                hops.append(resampled[RESAMPLE_HOP:2 * RESAMPLE_HOP])

                self.resample_history[ch] = block

            if not hops:
                continue
            audio = np.concatenate(hops).astype(np.float32)

            self.cuda_ctx.push()
            try:
                results = self.detectors[ch].push_audio(audio)
            finally:
                self.cuda_ctx.pop()

            for r in results:
                self._publish(ch, track_id, r, msg.header.stamp)

    def _publish(self, ch, track_id, result, stamp):
        self.frames_done += 1

        score = float(result.get('score', 0.0))
        decision = bool(result.get('decision', False))

        out = Vector3Stamped()
        out.header.stamp = stamp          # capture time - the EKF needs it
        out.header.frame_id = 'odas'
        out.vector.x = float(track_id)
        out.vector.y = 1.0 if decision else 0.0
        out.vector.z = score
        self.pub.publish(out)

        self.get_logger().info(
            f'GRE confidence={score:.3f} decision='
            f'{"DRONE" if decision else "not-drone"} '
            f'track={track_id} ch={ch}',
            throttle_duration_sec=1.0)

        if result.get('event_start'):
            self.get_logger().info(
                f'DRONE start on track {track_id} (ch {ch}) score {score:.2f}')
        elif result.get('event_end'):
            self.get_logger().info(
                f'drone end on track {track_id} (ch {ch})')
        elif self.args.verbose:
            bar = '#' * int(score * 20)
            self.get_logger().info(
                f'ch{ch} id{track_id} {score:5.2f} [{bar:<20}]'
                f'{"  DRONE" if decision else ""}')


def parse_args(argv):
    p = argparse.ArgumentParser()
    p.add_argument('--repo', type=Path,
                   default=Path.home() / 'project' / 'GRE' / 'rt_drone_detector')
    p.add_argument('--engine', type=Path, default=None)
    p.add_argument('--meta', type=Path, default=None)
    p.add_argument('--channels', type=int, default=4,
                   help='must equal len(sst.N_inactive) in configuration.cfg')
    p.add_argument('--min-activity', type=float, default=0.3)
    p.add_argument('--reset-cooldown', type=float, default=2.0,
                   help='min seconds between detector reconstructions')
    p.add_argument('--verbose', type=str, default='false', choices=['true', 'false'],
                   help='String, not a flag, so ExecuteProcess launch '
                        'arguments can pass it unconditionally.')
    args = p.parse_args(argv)
    args.verbose = args.verbose == 'true'
    return args


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    args = parse_args(argv)

    rclpy.init()
    node = GreClassifierNode(args)
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
