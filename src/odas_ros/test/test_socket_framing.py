"""Tests for TCP PCM frame reconstruction (report section 4)."""

import pytest

from stub_ros import lib_odas_server_node

PcmFrameExtractor = lib_odas_server_node.PcmFrameExtractor
convert_sst_snapshot = lib_odas_server_node.convert_sst_snapshot


def make_frame(index, size):
    """Deterministic, distinguishable frame payload."""
    return bytes(((index * 31 + j) % 251) + 1 for j in range(size))


def feed_chunks(extractor, payload, chunk_size):
    frames = []
    for offset in range(0, len(payload), chunk_size):
        frames += extractor.append(payload[offset:offset + chunk_size])
    return frames


class TestPcmFrameExtractor:
    def test_single_complete_frame(self):
        extractor = PcmFrameExtractor(frame_size=4096)
        frames = extractor.append(make_frame(0, 4096))
        assert frames == [make_frame(0, 4096)]
        assert extractor.pending_bytes == 0

    @pytest.mark.parametrize('chunk_size', [1, 3, 17, 255, 1000, 2048, 4095, 4096, 4097])
    def test_arbitrary_chunking_reconstructs_byte_identical_frames(self, chunk_size):
        frame_size = 4096
        frame_count = 5
        payload = b''.join(make_frame(i, frame_size) for i in range(frame_count))
        extractor = PcmFrameExtractor(frame_size=frame_size)

        frames = feed_chunks(extractor, payload, chunk_size)

        assert frames == [make_frame(i, frame_size) for i in range(frame_count)]
        assert extractor.pending_bytes == 0

    def test_accumulator_tail_always_smaller_than_frame_after_drain(self):
        frame_size = 4096
        payload = b''.join(make_frame(i, frame_size) for i in range(4)) + b'x' * 100
        extractor = PcmFrameExtractor(frame_size=frame_size)
        for offset in range(0, len(payload), 777):
            extractor.append(payload[offset:offset + 777])
            assert 0 <= extractor.pending_bytes < frame_size
        assert extractor.pending_bytes == 100

    def test_multiple_frames_in_one_chunk(self):
        extractor = PcmFrameExtractor(frame_size=16)
        frames = extractor.append(make_frame(0, 16) + make_frame(1, 16) + make_frame(2, 16))
        assert frames == [make_frame(0, 16), make_frame(1, 16), make_frame(2, 16)]
        assert extractor.pending_bytes == 0

    def test_frame_n_end_and_n_plus_1_start_in_one_chunk(self):
        extractor = PcmFrameExtractor(frame_size=16)
        assert extractor.append(make_frame(0, 16)) == [make_frame(0, 16)]
        # Head of frame 1 arrives, then one chunk carries the tail of frame 1
        # and the start of frame 2.
        assert extractor.append(make_frame(1, 16)[:12]) == []
        assert extractor.pending_bytes == 12
        frames = extractor.append(make_frame(1, 16)[12:] + make_frame(2, 16))
        assert frames == [make_frame(1, 16), make_frame(2, 16)]
        assert extractor.pending_bytes == 0

    def test_repeated_small_chunks_accumulate_until_frame_complete(self):
        extractor = PcmFrameExtractor(frame_size=64)
        frames = []
        for _ in range(63):
            frames += extractor.append(b'a')
        assert frames == []
        assert extractor.pending_bytes == 63
        frames += extractor.append(b'b')
        assert frames == [b'a' * 63 + b'b']
        assert extractor.pending_bytes == 0

    def test_partial_tail_discarded_at_disconnect(self):
        extractor = PcmFrameExtractor(frame_size=16)
        extractor.append(make_frame(0, 16) + b'partial-tail')
        assert extractor.pending_bytes == 12
        discarded = extractor.discard_tail()
        assert discarded == 12
        assert extractor.pending_bytes == 0

    def test_new_connection_starts_with_empty_accumulator(self):
        first = PcmFrameExtractor(frame_size=16)
        first.append(b'leftover-tail')
        # A new connection uses a fresh extractor; nothing carries over.
        second = PcmFrameExtractor(frame_size=16)
        assert second.pending_bytes == 0
        assert second.append(make_frame(0, 16)) == [make_frame(0, 16)]

    def test_invalid_frame_size_rejected(self):
        with pytest.raises(ValueError):
            PcmFrameExtractor(frame_size=0)


class TestConvertSstSnapshot:
    SLOT_COUNT = 4

    def convert(self, slots):
        sst = {'timeStamp': 1000, 'src': [
            {'id': track_id, 'x': 1.0, 'y': 2.0, 'z': 3.0, 'activity': 0.5}
            for track_id in slots
        ]}
        return convert_sst_snapshot(sst, self.SLOT_COUNT)

    def test_single_active_slot_keeps_position(self):
        snapshot = self.convert([0, 7, 0, 0])
        assert [s['id'] for s in snapshot] == [0, 7, 0, 0]

    def test_non_contiguous_sources_keep_slots(self):
        snapshot = self.convert([7, 0, 9, 0])
        assert [s['id'] for s in snapshot] == [7, 0, 9, 0]

    def test_all_empty_slots(self):
        snapshot = self.convert([0, 0, 0, 0])
        assert [s['id'] for s in snapshot] == [0, 0, 0, 0]
        assert all(s['activity'] == 0.5 for s in snapshot)

    def test_slot_count_mismatch_rejected(self):
        assert self.convert([1, 2, 3]) is None
        assert self.convert([1, 2, 3, 4, 5]) is None
        assert convert_sst_snapshot({'src': 'not-a-list'}, self.SLOT_COUNT) is None
        assert convert_sst_snapshot({}, self.SLOT_COUNT) is None

    def test_order_preserved_and_fields_copied(self):
        sst = {'src': [
            {'id': 3, 'x': 0.1, 'y': 0.2, 'z': 0.3, 'activity': 0.9},
            {'id': 1, 'x': 1.1, 'y': 1.2, 'z': 1.3, 'activity': 0.1},
        ]}
        snapshot = convert_sst_snapshot(sst, 2)
        assert [s['id'] for s in snapshot] == [3, 1]
        assert snapshot[0] == {'id': 3, 'x': 0.1, 'y': 0.2, 'z': 0.3, 'activity': 0.9}
        assert snapshot[1] == {'id': 1, 'x': 1.1, 'y': 1.2, 'z': 1.3, 'activity': 0.1}


class TestApplySssGain:
    def test_identity_passthrough_returns_same_object(self):
        import array
        data = array.array('h', [0, 100, -100, 32767, -32768]).tobytes()
        assert lib_odas_server_node.apply_sss_gain(data, 1.0) is data

    def test_scale(self):
        import array
        samples = [100, -200, 1000, -3276, 7]
        data = array.array('h', samples).tobytes()
        out = array.array('h')
        out.frombytes(lib_odas_server_node.apply_sss_gain(data, 2.0))
        assert list(out) == [200, -400, 2000, -6552, 14]

    def test_saturation_clamps_both_rails(self):
        import array
        samples = [30000, -30000, 1000, -32768, 32767]
        data = array.array('h', samples).tobytes()
        out = array.array('h')
        out.frombytes(lib_odas_server_node.apply_sss_gain(data, 10.0))
        assert list(out) == [32767, -32768, 10000, -32768, 32767]

    def test_fractional_gain_rounds_toward_zero(self):
        import array
        data = array.array('h', [101, -101]).tobytes()
        out = array.array('h')
        out.frombytes(lib_odas_server_node.apply_sss_gain(data, 0.5))
        assert list(out) == [50, -50]

    def test_full_frame_size_unchanged(self):
        import array
        data = array.array('h', [37] * (4096 // 2)).tobytes()
        out = lib_odas_server_node.apply_sss_gain(data, 1.5)
        assert len(out) == len(data)
