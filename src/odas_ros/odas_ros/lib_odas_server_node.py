from abc import ABC, abstractmethod
import array
import json
import math
import socket
import threading
import time
import os
import signal
import subprocess
import queue
from typing import ByteString, Callable, Dict, List, Optional, Tuple

import libconf
import io

import rclpy
import rclpy.node
import rclpy.time

from odas_ros_msgs.msg import OdasSst, OdasSstArrayStamped, OdasSsl, OdasSslArrayStamped
from audio_utils_msgs.msg import AudioFrame


SSL_SST_QUEUE_SIZE = 10
SSS_SST_PUBLISHER_DEPTH = 32

# Paired SSS/SST session policy (report section 7).
SESSION_INVALIDATE_TIMEOUT = 0.5
MAX_CONSECUTIVE_PAIR_VIOLATIONS = 3
MAX_PENDING_PAIRS = 64
SST_TIMEOUT = 0.25


def nbits_to_format(nbits):
    if nbits == 8:
        return 'signed_8'
    elif nbits == 16:
        return 'signed_16'
    elif nbits == 32:
        return 'signed_32'
    else:
        raise ValueError('Not supported format (nbits={})'.format(nbits))


def apply_sss_gain(data: ByteString, gain: float) -> ByteString:
    """Scale signed-16 PCM by a linear gain with int16 saturation.

    gain == 1.0 is an identity pass-through so the default hot path is a
    plain copy. Saturation clamps instead of wrapping, so an over-hot source
    distorts but never corrupts frame alignment.
    """
    if gain == 1.0:
        return data
    samples = array.array('h')
    samples.frombytes(bytes(data))
    scaled = array.array(
        'h', (max(-32768, min(32767, int(sample * gain))) for sample in samples))
    return scaled.tobytes()


class PcmFrameExtractor:
    """Reconstructs fixed-size PCM frames from an arbitrary TCP byte chunk stream.

    A fresh instance must be created per accepted connection so tail bytes are
    never carried across connections. After every drain the accumulator holds
    strictly less than one frame: 0 <= len(accumulator) < frame_size.
    """

    def __init__(self, frame_size: int):
        if frame_size <= 0:
            raise ValueError('frame_size must be positive, got {}'.format(frame_size))
        self._frame_size = frame_size
        self._accumulator = bytearray()

    @property
    def frame_size(self) -> int:
        return self._frame_size

    @property
    def pending_bytes(self) -> int:
        return len(self._accumulator)

    def append(self, data: bytes) -> List[bytes]:
        """Append a received chunk and return every complete frame, in order."""
        self._accumulator += data
        frames = []
        while len(self._accumulator) >= self._frame_size:
            frames.append(bytes(self._accumulator[:self._frame_size]))
            del self._accumulator[:self._frame_size]
        assert 0 <= len(self._accumulator) < self._frame_size
        return frames

    def discard_tail(self) -> int:
        """Discard any incomplete tail (connection closed mid-frame).

        Returns the number of discarded bytes.
        """
        tail_size = len(self._accumulator)
        self._accumulator.clear()
        return tail_size


def convert_sst_snapshot(sst: dict, expected_slot_count: int) -> Optional[List[dict]]:
    """Convert an ODAS SST JSON snapshot into its fixed-slot representation.

    Every entry is converted, including id == 0 (empty fixed slot), and the JSON
    order is preserved exactly so that /sst.sources[N] always describes
    /sss channel N. Returns None (rejecting the whole snapshot) when the
    snapshot does not contain exactly expected_slot_count 'src' entries.
    """
    sources = sst.get('src')
    if not isinstance(sources, list) or len(sources) != expected_slot_count:
        return None

    snapshot = []
    for source in sources:
        snapshot.append({
            'id': source['id'],
            'x': source['x'],
            'y': source['y'],
            'z': source['z'],
            'activity': source['activity'],
        })
    return snapshot


class PairCoordinator:
    """Deterministic SSS/SST ordinal pairing coordinator (report section 7).

    Shared by the SstSocketServer and SssSocketServer threads. All session
    state (generation, connection tokens, ordinals, pending maps, counters) is
    serialized under a single lock. Pairing is strictly by identical ordinal:
    SSS ordinal k pairs only with SST timeStamp == baseline + (k - 1). There is
    never a nearest-timestamp or arrival-order fallback.

    The class is ROS-free so it can be unit tested without spinning rclpy;
    diagnostics are emitted through injected log callables and actions
    (publish a pair, close sockets) are returned as event dicts.
    """

    STREAM_SSS = 'sss'
    STREAM_SST = 'sst'

    def __init__(self,
                 slot_count: int,
                 hop_duration: float,
                 session_invalidate_timeout: float = SESSION_INVALIDATE_TIMEOUT,
                 max_consecutive_pair_violations: int = MAX_CONSECUTIVE_PAIR_VIOLATIONS,
                 max_pending_pairs: int = MAX_PENDING_PAIRS,
                 sst_timeout: float = SST_TIMEOUT,
                 clock: Callable[[], float] = time.monotonic,
                 log_info: Callable[[str], None] = lambda msg: None,
                 log_warn: Callable[[str], None] = lambda msg: None,
                 log_error: Callable[[str], None] = lambda msg: None):
        if slot_count <= 0:
            raise ValueError('slot_count must be positive, got {}'.format(slot_count))
        if not (math.isfinite(hop_duration) and hop_duration > 0.0):
            raise ValueError('hop_duration must be positive and finite, got {}'.format(hop_duration))
        self._slot_count = slot_count
        self._hop_duration = hop_duration
        self._session_invalidate_timeout = session_invalidate_timeout
        self._max_consecutive_pair_violations = max_consecutive_pair_violations
        self._max_pending_pairs = max_pending_pairs
        self._sst_timeout = sst_timeout
        self._clock = clock
        self._log_info = log_info
        self._log_warn = log_warn
        self._log_error = log_error

        self._lock = threading.Lock()
        self._next_token = 1
        self._paired_enabled = True
        self._skew_violation_count = 0
        self._last_skew_log = None
        self._clear_session_state_locked()

    def is_paired_enabled(self) -> bool:
        with self._lock:
            return self._paired_enabled

    def is_session_active(self) -> bool:
        """True when both current SSS and SST connections are established."""
        with self._lock:
            return self._sss_token is not None and self._sst_token is not None

    def register_connection(self, stream: str) -> Tuple[int, List[dict]]:
        """Register an accepted socket for one stream; returns (token, events).

        When both streams are connected a new session generation is created:
        pending state is cleared, the next SSS ordinal becomes 1 and the SST
        baseline will be recorded from the first received SST timeStamp.
        """
        with self._lock:
            token = self._next_token
            self._next_token += 1
            if stream == self.STREAM_SSS:
                self._sss_token = token
            elif stream == self.STREAM_SST:
                self._sst_token = token
            else:
                raise ValueError('Unknown stream: {}'.format(stream))

            events = []
            if self._sss_token is not None and self._sst_token is not None:
                events = self._start_session_locked()
            return token, events

    def connection_lost(self, token: Optional[int]) -> List[dict]:
        """Handle a TCP disconnect: immediate session invalidation.

        Data from stale (invalidated or already closed) tokens is ignored.
        """
        with self._lock:
            if token is None:
                return []
            if token == self._sss_token or token == self._sst_token:
                return self._invalidate_session_locked('disconnect of {} connection'.format(
                    self.STREAM_SSS if token == self._sss_token else self.STREAM_SST))
            return []

    def submit_sss_frame(self, token: int, frame: bytes, recv_time: float) -> List[dict]:
        """Submit one fully reconstructed PCM frame; its ordinal increments once."""
        with self._lock:
            if not self._paired_enabled or token != self._sss_token:
                return []
            ordinal = self._next_sss_ordinal
            self._next_sss_ordinal += 1
            self._pending_sss[ordinal] = (frame, recv_time)
            events = self._purge_expired_locked(recv_time)
            events += self._enforce_bound_locked(self._pending_sss, self.STREAM_SSS)
            events += self._try_pair_locked(ordinal)
            events += self._check_tier_two_locked()
            return events

    def submit_sst_snapshot(self, token: int, time_stamp, snapshot: List[dict], recv_time: float) -> List[dict]:
        """Submit one fixed-slot SST snapshot keyed by its JSON timeStamp.

        The ordinal is normalized as timeStamp - baseline + 1. Before the
        session is validated (first completed pair), a non-integer first
        timeStamp or a timeStamp that does not increment by exactly 1 disables
        paired mode. After validation, duplicate/decreasing ordinals are
        dropped and skipped ordinals drain only the affected pairing window.
        """
        with self._lock:
            if not self._paired_enabled or token != self._sst_token:
                return []

            if self._baseline is None:
                if not self._is_integer(time_stamp):
                    return self._disable_paired_mode_locked(
                        'first SST timeStamp is not an integer: {!r}'.format(time_stamp))
                self._baseline = time_stamp
                self._last_sst_time_stamp = time_stamp
                self._log_info('SST baseline recorded: timeStamp {} (session generation {}, {} fixed slots)'.format(
                    time_stamp, self._generation, self._slot_count))
                events = [{'type': 'baseline_recorded', 'baseline': time_stamp, 'generation': self._generation}]
            elif not self._validated:
                if not self._is_integer(time_stamp) or time_stamp != self._last_sst_time_stamp + 1:
                    return self._disable_paired_mode_locked(
                        'SST timeStamp increments by exactly 1 violated before validation: {} -> {!r}'.format(
                            self._last_sst_time_stamp, time_stamp))
                self._last_sst_time_stamp = time_stamp
                events = []
            else:
                events = []
                if self._is_integer(time_stamp) and time_stamp == self._last_sst_time_stamp + 1:
                    self._last_sst_time_stamp = time_stamp
                elif self._is_integer(time_stamp) and time_stamp <= self._last_sst_time_stamp:
                    events += self._violation_locked('duplicate_ordinal',
                        'SST timeStamp {} does not increment past {}; snapshot dropped'.format(
                            time_stamp, self._last_sst_time_stamp))
                    events += self._check_tier_two_locked()
                    return events
                else:
                    skipped = (range(self._last_sst_time_stamp + 1, time_stamp)
                               if self._is_integer(time_stamp) else [])
                    events += self._violation_locked('ordinal_gap',
                        'SST timeStamp jumped from {} to {!r}; ordinals {} have no SST counterpart'.format(
                            self._last_sst_time_stamp, time_stamp,
                            [o - self._baseline + 1 for o in skipped]))
                    for ordinal in [o - self._baseline + 1 for o in skipped]:
                        if ordinal in self._pending_sss:
                            del self._pending_sss[ordinal]
                    if self._is_integer(time_stamp):
                        self._last_sst_time_stamp = time_stamp

            ordinal = time_stamp - self._baseline + 1
            if ordinal in self._pending_sst:
                events += self._violation_locked('duplicate_ordinal',
                    'SST ordinal {} already pending; snapshot dropped'.format(ordinal))
                events += self._check_tier_two_locked()
                return events
            self._pending_sst[ordinal] = (snapshot, recv_time)
            events += self._purge_expired_locked(recv_time)
            events += self._enforce_bound_locked(self._pending_sst, self.STREAM_SST)
            events += self._try_pair_locked(ordinal)
            events += self._check_tier_two_locked()
            return events

    # ------------------------------------------------------------------
    # Internal state (always called with self._lock held).
    # ------------------------------------------------------------------

    @staticmethod
    def _is_integer(value) -> bool:
        return isinstance(value, int) and not isinstance(value, bool)

    def _clear_session_state_locked(self):
        self._generation = getattr(self, '_generation', 0)
        self._sss_token = None
        self._sst_token = None
        self._pending_sss: Dict[int, Tuple[bytes, float]] = {}
        self._pending_sst: Dict[int, Tuple[List[dict], float]] = {}
        self._next_sss_ordinal = 1
        self._last_sst_time_stamp = None
        self._baseline = None
        self._validated = False
        self._session_epoch = None
        self._consecutive_violations = 0

    def _start_session_locked(self) -> List[dict]:
        sss_token = self._sss_token
        sst_token = self._sst_token
        self._clear_session_state_locked()
        self._generation += 1
        self._sss_token = sss_token
        self._sst_token = sst_token
        self._log_info('Paired SSS/SST session {} started; next SSS ordinal = 1, waiting for SST baseline'.format(
            self._generation))
        return [{'type': 'session_started', 'generation': self._generation}]

    def _invalidate_session_locked(self, reason: str) -> List[dict]:
        generation = self._generation
        self._clear_session_state_locked()
        self._log_error('Paired SSS/SST session {} invalidated: {}'.format(generation, reason))
        return [{'type': 'session_invalidated', 'reason': reason, 'generation': generation}]

    def _disable_paired_mode_locked(self, reason: str) -> List[dict]:
        self._paired_enabled = False
        self._pending_sss.clear()
        self._pending_sst.clear()
        self._log_error('Paired SSS/SST mode disabled, falling back to independent publication: ' + reason)
        return [{'type': 'paired_mode_disabled', 'reason': reason}]

    def _violation_locked(self, kind: str, detail: str) -> List[dict]:
        self._consecutive_violations += 1
        self._log_error('SSS/SST pairing violation ({}) [consecutive={}/{}]: {}'.format(
            kind, self._consecutive_violations, self._max_consecutive_pair_violations, detail))
        return [{'type': 'violation', 'kind': kind, 'detail': detail,
                 'consecutive_violations': self._consecutive_violations}]

    def _check_tier_two_locked(self) -> List[dict]:
        if self._consecutive_violations >= self._max_consecutive_pair_violations:
            return self._invalidate_session_locked(
                '{} consecutive pairing violations reached (max {})'.format(
                    self._consecutive_violations, self._max_consecutive_pair_violations))
        return []

    def _purge_expired_locked(self, now: float) -> List[dict]:
        events = []
        cutoff = now - self._session_invalidate_timeout
        for stream, pending in ((self.STREAM_SSS, self._pending_sss),
                                (self.STREAM_SST, self._pending_sst)):
            for ordinal in sorted(o for o, (_, rt) in pending.items() if rt < cutoff):
                del pending[ordinal]
                events += self._violation_locked('pair_timeout',
                    'unmatched {} entry for ordinal {} exceeded session_invalidate_timeout={} s'.format(
                        stream, ordinal, self._session_invalidate_timeout))
        return events

    def _enforce_bound_locked(self, pending: Dict[int, tuple], stream: str) -> List[dict]:
        events = []
        while len(pending) > self._max_pending_pairs:
            oldest = min(pending)
            del pending[oldest]
            events += self._violation_locked('pending_overflow',
                '{} pending {} entries exceed max {}; drained oldest ordinal {}'.format(
                    len(pending) + 1, stream, self._max_pending_pairs, oldest))
        return events

    def _try_pair_locked(self, ordinal: int) -> List[dict]:
        if ordinal not in self._pending_sss or ordinal not in self._pending_sst:
            return []
        sss_frame, sss_recv_time = self._pending_sss.pop(ordinal)
        sst_snapshot, sst_recv_time = self._pending_sst.pop(ordinal)
        self._consecutive_violations = 0

        if self._session_epoch is None:
            self._session_epoch = max(sss_recv_time, sst_recv_time)
            self._log_info(
                'First paired SSS/SST frame: ordinal {} pairs SSS frame 1 with SST baseline {} '
                '(session generation {}, {} fixed slots, session epoch {:.6f})'.format(
                    ordinal, self._baseline, self._generation, self._slot_count, self._session_epoch))
        self._validated = True

        stamp = self._session_epoch + (ordinal - 1) * self._hop_duration
        events = []
        skew = abs(sss_recv_time - sst_recv_time)
        if skew > self._sst_timeout:
            self._skew_violation_count += 1
            now = self._clock()
            if self._last_skew_log is None or now - self._last_skew_log >= 1.0:
                self._last_skew_log = now
                self._log_warn('SSS/SST bridge receive-time skew for ordinal {}: {:.3f} s exceeds sst_timeout={} s '
                               '(count={}); ordinal association unchanged'.format(
                                   ordinal, skew, self._sst_timeout, self._skew_violation_count))
            events.append({'type': 'pair_timestamp_skew', 'ordinal': ordinal,
                           'skew': skew, 'count': self._skew_violation_count})
        events.append({'type': 'pair', 'ordinal': ordinal, 'stamp': stamp,
                       'sss_data': sss_frame, 'sst_snapshot': sst_snapshot})
        return events


class SocketServer(ABC):
    def __init__(self, node: rclpy.node.Node, port: int):
        self._node = node
        self._node.get_logger().info("Creating server socket on port: " + str(port))
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket. SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind(('', port))
        self._server_socket.listen(5)
        self._server_socket.settimeout(0.1)

        self._thread = threading.Thread(target=self._run)
        self._is_stopped = True

    def start(self):
        self._is_stopped = False
        self._thread.start()

    def close(self):
        self._is_stopped = True
        self._server_socket.close()
        self._thread.join()

    def _run(self):
         while not self._is_stopped:
            try:
                client_socket, _ = self._server_socket.accept()
            except (socket.timeout, OSError):
                continue

            try:
                self._handle_client(client_socket)
            finally:
                client_socket.close()

    @abstractmethod
    def _handle_client(self, client_socket):
        pass


class RawSocketServer(SocketServer):
    def __init__(self,
                 node: rclpy.node.Node,
                 configuration: dict,
                 audio_frame_timestamp_queue: queue.Queue,
                 audio_queue_size: int):
        super().__init__(node, configuration['raw']['interface']['port'])
        self._audio_frame_timestamp_queue = audio_frame_timestamp_queue

        self._raw_nbits = configuration['raw']['nBits']
        self._raw_format = nbits_to_format(self._raw_nbits)
        self._raw_channel_count = configuration['raw']['nChannels']
        self._raw_sampling_frequency = configuration['raw']['fS']
        self._raw_frame_sample_count = configuration['raw']['hopSize']

        self._raw_queue = queue.Queue(maxsize=audio_queue_size)
        self._raw_sub = self._node.create_subscription(AudioFrame, 'raw', self._raw_audio_cb, audio_queue_size)

    def _raw_audio_cb(self, msg: AudioFrame):
        if (msg.format != self._raw_format or
            msg.channel_count != self._raw_channel_count or
            msg.sampling_frequency != self._raw_sampling_frequency or
            msg.frame_sample_count != self._raw_frame_sample_count):
            self._node.get_logger().error(
                'Invalid frame (msg.format={}, msg.channel_count={}, msg.sampling_frequency={}, msg.frame_sample_count={})'
                .format(msg.format, msg.channel_count, msg.sampling_frequency, msg.frame_sample_count))
            return

        if self._audio_frame_timestamp_queue is not None:
            self._audio_frame_timestamp_queue.put(msg.header.stamp)
        self._raw_queue.put(msg.data)

    def close(self):
        self._is_stopped = True
        self._raw_queue.put(None)
        super().close()

    def _handle_client(self, client_socket: socket.socket):
        while not self._is_stopped:
            data = self._raw_queue.get()
            if data is None:
                break
            try:
                client_socket.sendall(data)
            except OSError:
                break


class JsonSocketServer(SocketServer):

    def __init__(self, node: rclpy.node.Node, port: int):
        super().__init__(node, port)
        self._json_buffer = str()

    def _handle_client(self, client_socket: socket.socket):
        recv_size = 8192
        while not self._is_stopped:
            data = client_socket.recv(recv_size)
            if not data:
                break

            data = data.decode('utf-8')

            for message in self._split_json(data):
                try:
                    data = json.loads(message)
                    self._handle_data(data)
                except Exception as e:
                    self._node.get_logger().error(str(type(self)) + str(e) + 'message: ' + str(message) + ' *** data: ' + str(data))
                    continue

    def _split_json(self, data: str):
        self._json_buffer += data
        json_start_index = 0
        json_stop_index = 0
        depth = 0

        for i, c in enumerate(self._json_buffer):
            if c == '}' and depth == 0:
                continue
            elif c == '{':
                depth += 1
                if depth == 1:
                    json_start_index = i

            elif c == '}':
                depth -= 1
                if depth == 0:
                    # We have a complete struct
                    json_stop_index = i + 1
                    yield self._json_buffer[json_start_index:json_stop_index]

        self._json_buffer = self._json_buffer[json_stop_index:]

    @abstractmethod
    def _handle_data(self, data):
        pass


class SslSocketServer(JsonSocketServer):
    def __init__(self, node: rclpy.node.Node, configuration: dict, frame_id: str):
        super().__init__(node, configuration['ssl']['potential']['interface']['port'])
        self._frame_id = frame_id
        self._ssl_pub = self._node.create_publisher(OdasSslArrayStamped, 'ssl', SSL_SST_QUEUE_SIZE)

    def _handle_data(self, ssl: dict):
        odas_ssl_array_stamped_msg = OdasSslArrayStamped()
        odas_ssl_array_stamped_msg.header.stamp = self._node.get_clock().now().to_msg()
        odas_ssl_array_stamped_msg.header.frame_id = self._frame_id

        for source in ssl['src']:
            odas_ssl = OdasSsl()
            odas_ssl.x = source['x']
            odas_ssl.y = source['y']
            odas_ssl.z = source['z']
            odas_ssl.e = source['E']
            odas_ssl_array_stamped_msg.sources.append(odas_ssl)

        if rclpy.ok():
            self._ssl_pub.publish(odas_ssl_array_stamped_msg)


class _PairedStreamMixin:
    """Glue between a socket server thread and the shared PairCoordinator."""

    STREAM_SSS = PairCoordinator.STREAM_SSS
    STREAM_SST = PairCoordinator.STREAM_SST

    def _init_paired_stream(self, coordinator: Optional[PairCoordinator]):
        self._coordinator = coordinator
        self._pair_publisher: Optional[Callable[[int, float, bytes, List[dict]], None]] = None
        self._peer_server = None
        self._current_client_socket: Optional[socket.socket] = None
        self._connection_token: Optional[int] = None

    def set_pair_publisher(self, pair_publisher: Callable[[int, float, bytes, List[dict]], None]):
        self._pair_publisher = pair_publisher

    def set_peer_server(self, peer_server):
        self._peer_server = peer_server

    def close_current_client(self):
        """Shut down the current client socket so ODAS follows its normal send-error exit."""
        client_socket = self._current_client_socket
        if client_socket is not None:
            try:
                client_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _wait_for_session_barrier(self):
        """Hold processing at the connection barrier until both SSS and SST
        connections of the current session are established; kernel TCP
        buffering preserves early data."""
        while (not self._is_stopped and self._coordinator is not None and
               not self._coordinator.is_session_active()):
            time.sleep(0.01)

    def _handle_coordinator_events(self, events: List[dict]):
        for event in events or []:
            event_type = event['type']
            if event_type == 'pair':
                if self._pair_publisher is not None:
                    self._pair_publisher(event['ordinal'], event['stamp'],
                                         event['sss_data'], event['sst_snapshot'])
            elif event_type == 'session_invalidated':
                self.close_current_client()
                if self._peer_server is not None:
                    self._peer_server.close_current_client()


class SstSocketServer(JsonSocketServer, _PairedStreamMixin):
    def __init__(self,
                 node: rclpy.node.Node,
                 configuration: dict,
                 frame_id: str,
                 coordinator: Optional[PairCoordinator] = None):
        super().__init__(node, configuration['sst']['tracked']['interface']['port'])
        self._init_paired_stream(coordinator)
        self._frame_id = frame_id
        self._slot_count = len(configuration['sst']['N_inactive'])
        self._sst_pub = self._node.create_publisher(OdasSstArrayStamped, 'sst', SSS_SST_PUBLISHER_DEPTH)

    def _handle_client(self, client_socket: socket.socket):
        self._current_client_socket = client_socket
        self._connection_token = None
        if self._coordinator is not None:
            self._connection_token, events = self._coordinator.register_connection(self.STREAM_SST)
            self._handle_coordinator_events(events)
            self._wait_for_session_barrier()
        try:
            super()._handle_client(client_socket)
        finally:
            self._current_client_socket = None
            if self._coordinator is not None:
                self._handle_coordinator_events(self._coordinator.connection_lost(self._connection_token))
            self._connection_token = None

    def _handle_data(self, sst: dict):
        snapshot = convert_sst_snapshot(sst, self._slot_count)
        if snapshot is None:
            src_count = len(sst.get('src', [])) if isinstance(sst.get('src'), list) else 'non-list'
            self._node.get_logger().error(
                'Rejecting SST snapshot: expected {} fixed slots (one per /sss channel), got {}; whole snapshot discarded'
                .format(self._slot_count, src_count))
            return

        if (self._coordinator is not None and self._connection_token is not None and
                self._coordinator.is_paired_enabled()):
            events = self._coordinator.submit_sst_snapshot(
                self._connection_token, sst.get('timeStamp'), snapshot, time.monotonic())
            self._handle_coordinator_events(events)
        else:
            self._send_sst(snapshot)

    def _send_sst(self, snapshot: List[dict]):
        odas_sst_array_stamped_msg = OdasSstArrayStamped()
        odas_sst_array_stamped_msg.header.stamp = self._node.get_clock().now().to_msg()
        odas_sst_array_stamped_msg.header.frame_id = self._frame_id
        self._fill_sst_sources(odas_sst_array_stamped_msg, snapshot)

        if rclpy.ok():
            self._sst_pub.publish(odas_sst_array_stamped_msg)

    def publish_pair_snapshot(self, snapshot: List[dict], stamp_msg):
        odas_sst_array_stamped_msg = OdasSstArrayStamped()
        odas_sst_array_stamped_msg.header.stamp = stamp_msg
        odas_sst_array_stamped_msg.header.frame_id = self._frame_id
        self._fill_sst_sources(odas_sst_array_stamped_msg, snapshot)

        if rclpy.ok():
            self._sst_pub.publish(odas_sst_array_stamped_msg)

    @staticmethod
    def _fill_sst_sources(msg: OdasSstArrayStamped, snapshot: List[dict]):
        # /sst.sources[N] always describes /sss channel N; id == 0 is an
        # empty fixed slot. Entries are never filtered, sorted, or reordered.
        for source in snapshot:
            odas_sst = OdasSst()
            odas_sst.id = source['id']
            odas_sst.x = source['x']
            odas_sst.y = source['y']
            odas_sst.z = source['z']
            odas_sst.activity = source['activity']
            msg.sources.append(odas_sst)


class SssSocketServer(SocketServer, _PairedStreamMixin):
    def __init__(self,
                 node: rclpy.node.Node,
                 configuration: dict,
                 audio_frame_timestamp_queue: queue.Queue,
                 frame_id: str,
                 audio_queue_size: int,
                 coordinator: Optional[PairCoordinator] = None,
                 gain: float = 1.0):
        super().__init__(node, configuration['sss']['separated']['interface']['port'])
        self._init_paired_stream(coordinator)
        self._audio_frame_timestamp_queue = audio_frame_timestamp_queue
        self._frame_id = frame_id
        self._gain = gain

        self._sss_nbits = configuration['sss']['separated']['nBits']
        self._sss_format = nbits_to_format(self._sss_nbits)
        self._sss_channel_count = len(configuration['sst']['N_inactive'])
        self._sss_sampling_frequency = configuration['sss']['separated']['fS']
        self._sss_frame_sample_count = configuration['sss']['separated']['hopSize']

        self._frame_size = self._sss_nbits // 8 * self._sss_channel_count * self._sss_frame_sample_count
        recv_size = self._sss_nbits // 8 * self._sss_channel_count * self._sss_frame_sample_count
        assert self._frame_size == recv_size and self._frame_size > 0
        self._recv_chunk_size = self._frame_size * 16

        self._sss_pub = self._node.create_publisher(AudioFrame, 'sss', SSS_SST_PUBLISHER_DEPTH)

    def _handle_client(self, client_socket: socket.socket):
        self._current_client_socket = client_socket
        self._connection_token = None
        if self._coordinator is not None:
            self._connection_token, events = self._coordinator.register_connection(self.STREAM_SSS)
            self._handle_coordinator_events(events)
            self._wait_for_session_barrier()

        # Fresh accumulator per connection: tail bytes are never carried over.
        extractor = PcmFrameExtractor(self._frame_size)

        try:
            while not self._is_stopped:
                data = client_socket.recv(self._recv_chunk_size)
                if not data:
                    break

                for frame in extractor.append(data):
                    if (self._coordinator is not None and self._connection_token is not None and
                            self._coordinator.is_paired_enabled()):
                        events = self._coordinator.submit_sss_frame(
                            self._connection_token, frame, time.monotonic())
                        self._handle_coordinator_events(events)
                    else:
                        self._send_sss(frame)
        finally:
            tail_size = extractor.discard_tail()
            if tail_size > 0:
                self._node.get_logger().error(
                    'SSS connection closed with {} incomplete tail bytes discarded '
                    '(frame_size={}, incomplete PCM frame lost)'.format(tail_size, self._frame_size))
            self._current_client_socket = None
            if self._coordinator is not None:
                self._handle_coordinator_events(self._coordinator.connection_lost(self._connection_token))
            self._connection_token = None

    def _send_sss(self, data: ByteString):
        audio_frame_msg = AudioFrame()
        audio_frame_msg.header.stamp = self._get_timestamp()
        audio_frame_msg.header.frame_id = self._frame_id
        audio_frame_msg.format = self._sss_format
        audio_frame_msg.channel_count = self._sss_channel_count
        audio_frame_msg.sampling_frequency = self._sss_sampling_frequency
        audio_frame_msg.frame_sample_count = self._sss_frame_sample_count
        audio_frame_msg.data = apply_sss_gain(data, self._gain)

        if rclpy.ok():
            self._sss_pub.publish(audio_frame_msg)

    def publish_pair_frame(self, data: ByteString, stamp_msg):
        audio_frame_msg = AudioFrame()
        audio_frame_msg.header.stamp = stamp_msg
        audio_frame_msg.header.frame_id = self._frame_id
        audio_frame_msg.format = self._sss_format
        audio_frame_msg.channel_count = self._sss_channel_count
        audio_frame_msg.sampling_frequency = self._sss_sampling_frequency
        audio_frame_msg.frame_sample_count = self._sss_frame_sample_count
        audio_frame_msg.data = apply_sss_gain(data, self._gain)

        if rclpy.ok():
            self._sss_pub.publish(audio_frame_msg)

    def _get_timestamp(self):
        if self._audio_frame_timestamp_queue is None:
            return self._node.get_clock().now().to_msg()
        else:
            return self._audio_frame_timestamp_queue.get()


class OdasServerNode(rclpy.node.Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self._configuration_path = self.declare_parameter('configuration_path', '').get_parameter_value().string_value
        self._configuration = self._load_configuration(self._configuration_path)
        frame_id = self.declare_parameter('frame_id', '').get_parameter_value().string_value
        # Kept for backward compatibility; the SSS/SST publishers no longer use it.
        self.declare_parameter('audio_queue_size', 1).get_parameter_value().integer_value
        # Digital gain applied to /sss before publication. The UMA16v2 has no
        # analog boost on Linux (ALSA capture is an attenuator at max 0 dB), so
        # quiet sources reach classifiers and recorders at very low level.
        # Enabled by default; set to 0.0 to publish the raw ODAS separation level.
        sss_gain_db = self.declare_parameter('sss_gain_db', 24.0).get_parameter_value().double_value
        self._sss_gain = math.pow(10.0, sss_gain_db / 20.0)
        self.get_logger().info('/sss gain: {} dB ({:.2f}x)'.format(sss_gain_db, self._sss_gain))

        if self._verify_raw_and_sss_configuration():
            audio_frame_timestamp_queue = queue.Queue()
        else:
            audio_frame_timestamp_queue = None

        self._coordinator = self._create_pair_coordinator()

        if self._verify_raw_configuration():
            self._raw_socket_server = RawSocketServer(self,
                                                      self._configuration,
                                                      audio_frame_timestamp_queue,
                                                      self._get_audio_queue_size())
        else:
            self._raw_socket_server = None

        if self._verify_ssl_configuration():
            self._ssl_socket_server = SslSocketServer(self, self._configuration, frame_id)
        else:
            self._ssl_socket_server = None

        if self._verify_sst_configuration():
            self._sst_socket_server = SstSocketServer(self, self._configuration, frame_id,
                                                      coordinator=self._coordinator)
        else:
            self._sst_socket_server = None

        if self._verify_sss_configuration():
            self._sss_socket_server = SssSocketServer(self,
                                                      self._configuration,
                                                      audio_frame_timestamp_queue,
                                                      frame_id,
                                                      self._get_audio_queue_size(),
                                                      coordinator=self._coordinator,
                                                      gain=self._sss_gain)
        else:
            self._sss_socket_server = None

        if (self._coordinator is not None and self._sst_socket_server is not None and
                self._sss_socket_server is not None):
            pair_publisher = self._make_pair_publisher()
            self._sst_socket_server.set_pair_publisher(pair_publisher)
            self._sss_socket_server.set_pair_publisher(pair_publisher)
            self._sst_socket_server.set_peer_server(self._sss_socket_server)
            self._sss_socket_server.set_peer_server(self._sst_socket_server)
            self.get_logger().info('Paired SSS/SST publication enabled (ordinal pairing, publisher depth {})'
                                   .format(SSS_SST_PUBLISHER_DEPTH))
        elif self._coordinator is not None:
            self.get_logger().info('Only one of SST/SSS socket outputs configured; '
                                   'independent publication mode preserved')

    def _get_audio_queue_size(self):
        return self.get_parameter('audio_queue_size').get_parameter_value().integer_value

    def _create_pair_coordinator(self) -> Optional[PairCoordinator]:
        # Paired mode requires both socket outputs; otherwise each stream keeps
        # its existing independent publication behavior.
        if not (self._verify_sst_configuration() and self._verify_sss_configuration()):
            return None

        hop_size = self._configuration['sss']['separated']['hopSize']
        sampling_frequency = self._configuration['sss']['separated']['fS']
        hop_duration = hop_size / sampling_frequency
        if not (math.isfinite(hop_duration) and hop_duration > 0.0):
            self.get_logger().error(
                'Invalid SSS configuration (hopSize={}, fS={}): cannot derive a positive hop duration; '
                'paired SSS/SST mode disabled'.format(hop_size, sampling_frequency))
            return None

        slot_count = len(self._configuration['sst']['N_inactive'])
        logger = self.get_logger()
        return PairCoordinator(
            slot_count=slot_count,
            hop_duration=hop_duration,
            log_info=logger.info,
            log_warn=logger.warning,
            log_error=logger.error)

    def _make_pair_publisher(self):
        def publish_pair(ordinal: int, stamp: float, sss_data: bytes, sst_snapshot: List[dict]):
            stamp_msg = rclpy.time.Time(seconds=float(stamp)).to_msg()
            self._sss_socket_server.publish_pair_frame(sss_data, stamp_msg)
            self._sst_socket_server.publish_pair_snapshot(sst_snapshot, stamp_msg)

        return publish_pair

    def _load_configuration(self, configuration_path: str):
        with io.open(configuration_path) as f:
            return libconf.load(f)

    def _verify_raw_configuration(self):
        return self._configuration['raw']['interface']['type'] == 'socket'

    def _verify_ssl_configuration(self):
        if self._configuration['ssl']['potential']['interface']['type'] != 'socket':
            return False
        elif self._configuration['ssl']['potential']['format'] != 'json':
            raise ValueError('The ssl format must be "json"')
        else:
            return True

    def _verify_sst_configuration(self):
        if self._configuration['sst']['tracked']['interface']['type'] != 'socket':
            return False
        elif self._configuration['sst']['tracked']['format'] != 'json':
            raise ValueError('The sst format must be "json"')
        else:
            return True

    def _verify_sss_configuration(self):
        return self._configuration['sss']['separated']['interface']['type'] == 'socket'

    def _verify_raw_and_sss_configuration(self):
        if (self._configuration['raw']['interface']['type'] != 'socket' or
            self._configuration['sss']['separated']['interface']['type'] != 'socket'):
            return False

        if self._configuration['raw']['fS'] != self._configuration['sss']['separated']['fS']:
            raise ValueError('Raw and sss sampling frequencies must match.')
        if self._configuration['raw']['hopSize'] != self._configuration['sss']['separated']['hopSize']:
            raise ValueError('Raw and sss hop sizes must match.')

        return True

    def run(self):
        if self._raw_socket_server:
            self._raw_socket_server.start()
            self.get_logger().info("Raw socket server started")
        if self._ssl_socket_server:
            self._ssl_socket_server.start()
            self.get_logger().info("Sound Source Localization socket server started")
        if self._sst_socket_server:
            self._sst_socket_server.start()
            self.get_logger().info("Sound Source Tracking socket server started")
        if self._sss_socket_server:
            self._sss_socket_server.start()
            self.get_logger().info("Sound Source Separation socket server started")

        executable_args = ["ros2",
                           "launch",
                           "odas_ros",
                           "odas_core_node.launch.xml",
                           "configuration_path:=" + self._configuration_path]

        odas_core_process = subprocess.Popen(executable_args, cwd=os.curdir)

        try:
            rclpy.spin(self)
        finally:
            if self._raw_socket_server:
                self._raw_socket_server.close()
            if self._ssl_socket_server:
                self._ssl_socket_server.close()
            if self._sst_socket_server:
                self._sst_socket_server.close()
            if self._sss_socket_server:
                self._sss_socket_server.close()

            odas_core_process.terminate()
            odas_core_process.wait()
