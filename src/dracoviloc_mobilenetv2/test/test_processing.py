import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.signal import resample_poly

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mobilenetv2'))
from processing import ChannelState, StreamingResampler, decode_audio


@pytest.mark.parametrize('chunk_size', [1, 512, 1000, 4410])
def test_stream_matches_continuous_resampling(chunk_size):
    raw = np.random.default_rng(5).standard_normal(44100 * 3).astype(np.float32)
    resampler = StreamingResampler()
    pieces = [resampler.push(raw[i:i + chunk_size]) for i in range(0, len(raw), chunk_size)]
    actual = np.concatenate(pieces)
    expected = resample_poly(raw, 160, 441)
    assert len(actual) == 46400  # final 0.1 s is retained as future FIR context
    np.testing.assert_allclose(actual, expected[:len(actual)], atol=1e-4, rtol=0)


def test_exact_windows_and_reset():
    state = ChannelState()
    raw = np.random.default_rng(1).standard_normal(44100 * 3).astype(np.float32)
    windows = []
    for i in range(0, len(raw), 512):
        windows.extend(state.push(raw[i:i + 512], 10 + i / 44100))
    assert [end for _, end in windows] == [11, 11.5, 12, 12.5]
    ref = resample_poly(raw, 160, 441)
    for index, (waveform, _) in enumerate(windows):
        np.testing.assert_allclose(waveform, ref[index * 8000:index * 8000 + 16000], atol=1e-4)
    state.vote(0.9)
    state.set_identity((10, 'table_mic_link', True))
    assert not state.votes and state.buffer.size == 0 and state.start_time is None
    restarted = list(state.push(raw, 100))
    np.testing.assert_allclose(restarted[0][0], windows[0][0], atol=1e-4)
    assert restarted[0][1] == 101


def test_voting_requires_two_consecutive_positive_windows():
    state = ChannelState()
    assert [state.vote(p) for p in [0.75, 0.1, 0.9, 0.8, 0.1]] == [
        False, False, False, True, False]
    other = ChannelState()
    assert [other.vote(0.9) for _ in range(2)] == [False, True]
    state.set_identity((2, 'table_mic_link', True))
    assert not state.vote(0.9)
    assert len(other.votes) == 2


@pytest.mark.parametrize('kwargs', [dict(threshold=float('nan')), dict(threshold=1.1),
                                    dict(votes_required=0), dict(votes_required=3)])
def test_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        ChannelState(**kwargs)


def test_audio_contract():
    pcm = np.array([[-32768, 32767, 0, 100]], dtype='<i2')
    msg = SimpleNamespace(format='signed_16', sampling_frequency=44100, channel_count=4,
                          frame_sample_count=1, data=pcm.tobytes())
    np.testing.assert_equal(decode_audio(msg, 4), pcm.astype(np.float32) / 32768)
    for field, bad in [('format', 'float'), ('sampling_frequency', 16000),
                       ('channel_count', 2), ('data', b'')]:
        original = getattr(msg, field)
        setattr(msg, field, bad)
        with pytest.raises(ValueError):
            decode_audio(msg, 4)
        setattr(msg, field, original)
