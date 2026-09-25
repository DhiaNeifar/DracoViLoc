"""Exercise real callbacks with deterministic inference and no DDS/CUDA startup."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time
from audio_utils_msgs.msg import AudioFrame
from odas_ros_msgs.msg import OdasSst, OdasSstArrayStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mobilenetv2'))
from classifier_node import MobileNetV2Classifier, parse_args
from processing import ChannelState


def make_node(always=False):
    node = object.__new__(MobileNetV2Classifier)
    node.args = parse_args(['--engine-path', 'unused', '--always-classify', str(always).lower()])
    node.channels = [ChannelState() for _ in range(4)]
    node.latest_sst = None
    node.last_audio_stamp = None
    node.audio_seen = node.windows_done = node.dropped_audio = 0
    node.engine = SimpleNamespace(infer=lambda w: np.array([0.1, 0.9]))
    node.messages = []
    node.publisher = SimpleNamespace(publish=node.messages.append)
    node.test_time = 100.0
    node._now = lambda: node.test_time
    node.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None,
                                              warning=lambda *a, **k: None)
    return node


def feed(node, count, track=1, active=1.0, vector=(3.0, 4.0, 0.0), stale=False, malformed=False):
    for _ in range(count):
        node.test_time += 512 / 44100
        stamp = Time(seconds=node.test_time - (2 if stale else 0)).to_msg()
        sst = OdasSstArrayStamped()
        sst.header.stamp = stamp
        sst.header.frame_id = 'table_mic_link'
        sst.sources = [OdasSst(id=track, x=vector[0], y=vector[1], z=vector[2], activity=active)]
        sst.sources += [OdasSst() for _ in range(3)]
        node._sst_cb(sst)
        audio = AudioFrame()
        audio.header.stamp = stamp
        audio.format = 'signed_16'
        audio.sampling_frequency = 44100
        audio.channel_count = 4
        audio.frame_sample_count = 512
        audio.data = b'' if malformed else np.zeros((512, 4), dtype='<i2').tobytes()
        node._audio_cb(audio)


def test_accepted_direction_and_track_replacement():
    node = make_node()
    feed(node, 190)
    assert len(node.messages) >= 1
    out = node.messages[-1]
    np.testing.assert_allclose([out.vector.x, out.vector.y, out.vector.z], [0.6, 0.8, 0])
    assert out.header.frame_id == 'table_mic_link'
    assert node.test_time - (out.header.stamp.sec + out.header.stamp.nanosec * 1e-9) < 0.5
    accepted_for_track_a = len(node.messages)
    feed(node, 100, track=2)
    assert len(node.messages) == accepted_for_track_a  # first positive cannot confirm a new track
    feed(node, 100, track=2)
    assert len(node.messages) > accepted_for_track_a
    feed(node, 1, active=0.0)
    assert not node.channels[0].votes


@pytest.mark.parametrize('settings', [dict(track=0), dict(active=0.0),
                                     dict(vector=(0.0, 0.0, 0.0)),
                                     dict(vector=(float('nan'), 1.0, 0.0)),
                                     dict(stale=True), dict(malformed=True)])
def test_invalid_inputs_never_publish(settings):
    node = make_node()
    feed(node, 200, **settings)
    assert not node.messages


def test_diagnostic_classification_is_not_actionable():
    node = make_node(always=True)
    feed(node, 200, track=0)
    assert node.windows_done >= 12
    assert not node.messages


def test_stale_sst_resets_state():
    node = make_node()
    feed(node, 190)
    node.latest_sst.header.stamp = Time(seconds=1).to_msg()
    assert not node._fresh_source(0, node.test_time, node.test_time)


@pytest.mark.parametrize('option', ['--threshold', '--sst-timeout', '--max-audio-age'])
def test_nonfinite_options_rejected(option):
    with pytest.raises(SystemExit):
        parse_args(['--engine-path', 'unused', option, 'nan'])
