"""Raw-frame timestamps must only accumulate while something consumes them."""

import queue
from types import SimpleNamespace

from stub_ros import lib_odas_server_node

RawSocketServer = lib_odas_server_node.RawSocketServer


class FakeCoordinator:
    def __init__(self, paired):
        self.paired = paired

    def is_paired_enabled(self):
        return self.paired


def make_server(coordinator):
    server = RawSocketServer.__new__(RawSocketServer)
    server._bump = lambda *args: None
    server._raw_format = 'signed_32'
    server._raw_channel_count = 16
    server._raw_sampling_frequency = 44100
    server._raw_frame_sample_count = 512
    server._audio_frame_timestamp_queue = queue.Queue()
    server._raw_queue = queue.Queue()
    server._coordinator = coordinator
    return server


def make_frame(stamp):
    return SimpleNamespace(
        format='signed_32', channel_count=16, sampling_frequency=44100,
        frame_sample_count=512, data=b'\x00' * 8, header=SimpleNamespace(stamp=stamp))


def test_paired_mode_does_not_queue_timestamps():
    server = make_server(FakeCoordinator(paired=True))
    for index in range(1000):
        server._raw_audio_cb(make_frame(index))
    assert server._audio_frame_timestamp_queue.qsize() == 0
    assert server._raw_queue.qsize() == 1000


def test_unpaired_mode_queues_one_timestamp_per_frame():
    for coordinator in (None, FakeCoordinator(paired=False)):
        server = make_server(coordinator)
        for index in range(3):
            server._raw_audio_cb(make_frame(index))
        assert [server._audio_frame_timestamp_queue.get() for _ in range(3)] == [0, 1, 2]


def test_timestamps_resume_after_paired_mode_is_disabled():
    coordinator = FakeCoordinator(paired=True)
    server = make_server(coordinator)
    server._raw_audio_cb(make_frame(0))
    coordinator.paired = False
    server._raw_audio_cb(make_frame(1))
    assert server._audio_frame_timestamp_queue.get_nowait() == 1
