#!/usr/bin/env python3
"""
uma16_feeder_node.py  -  ROS 2 audio source for odas_ros

Captures 16 ch int32 from the UMA-16, applies a stateful Butterworth bandpass
identically to every channel, and publishes audio_utils_msgs/AudioFrame on the
topic `raw`.

WHY THIS REPLACES THE SOCKET FEEDER
-----------------------------------
In lib_odas_server_node.py, RawSocketServer is an audio PRODUCER, not a
consumer:

    self._raw_sub = create_subscription(AudioFrame, 'raw', self._raw_audio_cb, n)
    def _raw_audio_cb(self, msg): self._raw_queue.put(msg.data)
    def _handle_client(self, sock): sock.send(self._raw_queue.get())

Audio enters through the ROS topic and is SENT OUT over the raw port to the
ODAS core. A feeder connecting to that port is treated as a second core
wanting to receive audio, and since SocketServer._run serves one client at a
time (_handle_client blocks until shutdown), whichever process connects first
owns the socket forever and the other is stranded in the accept backlog.

    OLD (ODAS Studio) :  feeder binds  <- odaslive connects
    OLD (attempted)   :  server binds  <- feeder connects        BROKEN
    NEW (odas_ros)    :  feeder publishes /raw -> server -> core

STARTUP
-------
  1. ros2 launch odas_ros odas.launch.xml \
         configuration_path:=$HOME/odas_ws/src/odas_ros/odas_ros/config/configuration.cfg \
         audio_queue_size:=8 rviz:=true
     (this also spawns odas_core_node by itself - do not start odaslive)
  2. ros2 run --prefix 'python3' ... or simply:
     python3 uma16_feeder_node.py

Order is forgiving: publish before the core connects and the frames simply
queue; the core drains them on connect.

audio_queue_size MATTERS. _raw_audio_cb calls queue.put() with no timeout on a
Queue(maxsize=audio_queue_size). The launch default is 1, so a single late
frame blocks the server's executor thread. Use 8.

CONFIG REQUIREMENTS (configuration.cfg)
---------------------------------------
  raw: { fS = 44100; hopSize = 512; nBits = 32; nChannels = 16;
         interface: { type = "socket"; ip = "127.0.0.1"; port = 9200; } }

The four values above are validated frame-by-frame by _raw_audio_cb; any
mismatch logs 'Invalid frame' and silently drops the audio. They must equal
--fs, --hop, 32 and --channels here.

Leave raw.interface as socket. Switching it to soundcard makes
_verify_raw_configuration() return False, the raw server is never created, and
the core reads the microphone directly - bypassing this filter.

USAGE
-----
  python3 uma16_feeder_node.py                      # 180-3600 Hz localisation band
  python3 uma16_feeder_node.py --lo 3000 --hi 9000  # classifier band
  python3 uma16_feeder_node.py --no-publish         # level check only
  python3 uma16_feeder_node.py --bypass             # unfiltered A/B

If the device comes back busy:
  pkill -9 -f odas_core_node; pkill -9 -f odaslive; pkill arecord
  fuser -k /dev/snd/pcmC*D0c 2>/dev/null; sleep 1
"""

import argparse
import os
import queue
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from audio_utils_msgs.msg import AudioFrame


# Spatial aliasing ceiling for the UMA-16: c / (2 * d) = 343 / (2 * 0.042).
# Content above this cannot be localised unambiguously by this array.
ALIASING_CEILING_HZ = 4083.0

# nbits_to_format() in lib_odas_server_node.py maps 32 -> 'signed_32'.
# The subscriber rejects any frame whose format string differs.
AUDIO_FORMAT = 'signed_32'


def detect_uma_device():
    """Return hw:CARD=<name>,DEV=0 for the UMA-16, or None.

    Card INDICES shift when another audio device enumerates first (the USB
    camera) or on a different boot order - that is what produces
    "Cannot get card index for 2". The symbolic name is stable, so resolve it
    at startup instead of hardcoding.
    """
    try:
        with open("/proc/asound/cards") as f:
            text = f.read()
    except OSError:
        return None

    for match in re.finditer(r"^\s*(\d+)\s+\[(\S+)\s*\]", text, re.MULTILINE):
        name = match.group(2)
        line_end = text.find("\n", match.end())
        description = text[match.end():line_end if line_end > 0 else None]
        if "uma" in (name + description).lower():
            return f"hw:CARD={name},DEV=0"
    return None


def build_filter(order, lo, hi, fs, nch):
    """One sos shared by all channels, independent per-channel state.

    Sharing the coefficients matters: any per-channel difference in phase
    response would corrupt the inter-channel time differences that SRP-PHAT
    depends on, cleaning the audio while ruining the DOA.
    """
    nyq = fs / 2.0
    if not (0 < lo < hi < nyq):
        raise SystemExit(f"band {lo}-{hi} Hz invalid for fs={fs}")
    sos = butter(order, [lo, hi], btype="bandpass", fs=fs, output="sos")
    zi = np.repeat(sosfilt_zi(sos)[:, :, None], nch, axis=2)
    return sos, zi


class ArecordSource:
    """Streams fixed-size blocks from `arecord` stdout. Bypasses PortAudio.

    arecord runs in its own process group (start_new_session=True) so a Ctrl-C
    on the node does not race it. close() is then the single path that
    terminates it, preventing the child from surviving and holding the ALSA
    capture device.
    """

    def __init__(self, alsa_device, fs, nch, hop):
        if shutil.which("arecord") is None:
            raise SystemExit("arecord not found - install alsa-utils")
        self.frame_bytes = 4 * nch
        self.block_bytes = hop * self.frame_bytes
        self.nch = nch
        cmd = ["arecord", "-D", alsa_device, "-f", "S32_LE",
               "-c", str(nch), "-r", str(fs), "-t", "raw",
               "--period-size", str(hop), "--buffer-size", str(hop * 8)]
        print(f"[feeder] {' '.join(cmd)}", flush=True)
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE,
                                     start_new_session=True)

        # Fail fast and legibly rather than blocking on an empty pipe.
        time.sleep(0.4)
        if self.proc.poll() is not None:
            err = self.stderr_text()
            hint = ""
            if "busy" in err.lower():
                hint = ("\n[feeder] the card is held by another process. Run:\n"
                        "         fuser -v /dev/snd/pcmC*D0c\n"
                        "         pkill -9 -f odas_core_node; pkill -9 -f odaslive")
            elif "no such" in err.lower() or "card index" in err.lower():
                hint = ("\n[feeder] device not found. Run: cat /proc/asound/cards\n"
                        "         then pass --alsa-device hw:CARD=<name>,DEV=0")
            raise SystemExit(f"[feeder] arecord failed:\n{err}{hint}")

    def read(self):
        buf = self.proc.stdout.read(self.block_bytes)
        if not buf or len(buf) < self.block_bytes:
            return None
        return np.frombuffer(buf, dtype="<i4").reshape(-1, self.nch)

    def close(self):
        if self.proc.poll() is not None:
            self._close_pipes()
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            self.proc.wait(timeout=2)
        except Exception:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=2)
            except Exception:
                pass
        self._close_pipes()

    def _close_pipes(self):
        for pipe in (self.proc.stdout, self.proc.stderr):
            try:
                if pipe is not None:
                    pipe.close()
            except Exception:
                pass

    def stderr_text(self):
        try:
            return self.proc.stderr.read().decode(errors="replace")
        except Exception:
            return ""


class Uma16FeederNode(Node):
    """Capture -> bandpass -> publish AudioFrame on `raw`."""

    def __init__(self, args):
        super().__init__('uma16_feeder_node')
        self.args = args

        self.sos, self.zi = build_filter(args.order, args.lo, args.hi,
                                         args.fs, args.channels)
        self.full_scale = float(2 ** 31 - 1)
        self.hops_per_sec = args.fs / args.hop

        # RELIABLE + KEEP_LAST to match the server's default subscription QoS.
        # Depth here should be >= the launch's audio_queue_size.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=args.qos_depth,
        )
        self.pub = None if args.no_publish else self.create_publisher(
            AudioFrame, 'raw', qos)

        self.src = ArecordSource(args.alsa_device, args.fs,
                                 args.channels, args.hop)
        self.stop = threading.Event()
        self.sent = 0
        self.peak_seen = 0

        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        a = self.args
        band = "bypass" if a.bypass else f"{a.lo:.0f}-{a.hi:.0f} Hz order {a.order}"
        self.get_logger().info(
            f"fs={a.fs} ch={a.channels} hop={a.hop} filter={band}")
        self.get_logger().info(
            f"{a.hop * a.channels * 4} bytes/frame, "
            f"{self.hops_per_sec:.1f} frames/s -> topic 'raw'")
        self.thread.start()

    def _make_msg(self, payload):
        msg = AudioFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        # These four are checked field-by-field in _raw_audio_cb. Any mismatch
        # against configuration.cfg logs 'Invalid frame' and drops the audio.
        msg.format = AUDIO_FORMAT
        msg.channel_count = self.args.channels
        msg.sampling_frequency = self.args.fs
        msg.frame_sample_count = self.args.hop
        msg.data = payload
        return msg

    def _loop(self):
        a = self.args
        while not self.stop.is_set() and rclpy.ok():
            block = self.src.read()
            if block is None:
                err = self.src.stderr_text()
                if err.strip():
                    self.get_logger().error(err.strip())
                self.get_logger().warn("capture ended")
                self.stop.set()
                break

            self.peak_seen = max(self.peak_seen, int(np.abs(block).max()))

            if a.bypass:
                out = block
            else:
                x = block.astype(np.float32) / self.full_scale
                y, self.zi = sosfilt(self.sos, x, axis=0, zi=self.zi)
                y *= a.gain
                np.clip(y, -1.0, 1.0, out=y)
                out = (y * self.full_scale).astype(np.int32)

            if self.pub is not None:
                payload = np.ascontiguousarray(out, dtype="<i4").tobytes()
                self.pub.publish(self._make_msg(payload))

            self.sent += 1
            if self.sent % int(self.hops_per_sec) == 0:
                secs = self.sent / self.hops_per_sec
                peak_db = 20 * np.log10(max(self.peak_seen, 1) / self.full_scale)
                flag = ""
                if peak_db < -50:
                    flag = "  <- very low, check gain"
                elif peak_db > -1:
                    flag = "  <- clipping"
                self.get_logger().info(
                    f"{secs:5.0f} s  frames={self.sent}  "
                    f"peak={peak_db:+.1f} dBFS{flag}")
                self.peak_seen = 0

    def shutdown(self):
        self.stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        self.src.close()
        self.get_logger().info("capture device released")


def parse_args(argv):
    p = argparse.ArgumentParser()
    p.add_argument("--alsa-device", default=None,
                   help="default: auto-detect the UMA-16 from /proc/asound/cards")
    p.add_argument("--fs", type=int, default=44100,
                   help="must equal raw.fS")
    p.add_argument("--channels", type=int, default=16,
                   help="must equal raw.nChannels")
    p.add_argument("--hop", type=int, default=512,
                   help="must equal raw.hopSize")
    p.add_argument("--frame-id", default="odas",
                   help="must match the launch's frame_id argument")
    p.add_argument("--qos-depth", type=int, default=8,
                   help="publisher depth; keep >= the launch audio_queue_size")
    p.add_argument("--lo", type=float, default=180.0)
    p.add_argument("--hi", type=float, default=3600.0)
    p.add_argument("--order", type=int, default=4)
    p.add_argument("--gain", type=float, default=1.0)
    p.add_argument("--bypass", action="store_true",
                   help="stream unfiltered (A/B reference)")
    p.add_argument("--no-publish", action="store_true",
                   help="capture and show levels without publishing")
    return p.parse_args(argv)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    args = parse_args(argv)

    if args.alsa_device is None:
        args.alsa_device = detect_uma_device()
        if args.alsa_device is None:
            raise SystemExit(
                "[feeder] could not find the UMA-16 in /proc/asound/cards.\n"
                "[feeder] check it is plugged in (lsusb | grep -i minidsp), or\n"
                "[feeder] pass --alsa-device hw:CARD=<name>,DEV=0 explicitly.")
        print(f"[feeder] auto-detected device: {args.alsa_device}", flush=True)

    if not args.bypass and args.hi > ALIASING_CEILING_HZ:
        print(f"[feeder] NOTE: {args.hi:.0f} Hz is above the "
              f"{ALIASING_CEILING_HZ:.0f} Hz spatial aliasing ceiling of a "
              f"42 mm array.", flush=True)
        print("[feeder]       Fine for classification, but DOA above that "
              "is ambiguous.", flush=True)

    rclpy.init()
    node = Uma16FeederNode(args)
    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
