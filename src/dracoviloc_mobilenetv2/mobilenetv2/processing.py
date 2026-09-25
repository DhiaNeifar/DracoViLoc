"""Pure CPU streaming preparation for the model's embedded audio frontend."""
from collections import deque

import numpy as np
from scipy.signal import resample_poly

INPUT_RATE = 44100
MODEL_RATE = 16000
UP, DOWN = 160, 441
BLOCK = 4410
OUTPUT_BLOCK = 1600
WINDOW = 16000
HOP = 8000


def decode_audio(msg, channels):
    if (msg.format != 'signed_16' or msg.sampling_frequency != INPUT_RATE
            or msg.channel_count != channels or msg.frame_sample_count <= 0):
        raise ValueError('expected nonempty signed_16, 44100 Hz ODAS audio with '
                         f'{channels} channels')
    if len(msg.data) != msg.frame_sample_count * channels * 2:
        raise ValueError('audio payload length does not match its declared shape')
    return (np.frombuffer(bytes(msg.data), dtype='<i2').reshape(-1, channels)
            .astype(np.float32) / 32768.0)


class StreamingResampler:
    """Keep both FIR context sides, only emitting blocks on the exact sample grid."""
    def __init__(self):
        self.pending = np.empty(0, dtype=np.float32)
        self.history = np.zeros(BLOCK, dtype=np.float32)

    def push(self, samples):
        self.pending = np.concatenate((self.pending, samples))
        output = []
        while self.pending.size >= 2 * BLOCK:
            block = self.pending[:BLOCK]
            primed = np.concatenate((self.history, self.pending[:2 * BLOCK]))
            converted = resample_poly(primed, UP, DOWN)
            output.append(converted[OUTPUT_BLOCK:2 * OUTPUT_BLOCK])
            self.history = block.copy()
            self.pending = self.pending[BLOCK:]
        return np.concatenate(output) if output else np.empty(0, dtype=np.float32)


class ChannelState:
    def __init__(self, threshold=0.75, votes_required=2, vote_window=2):
        if not 0.0 <= threshold <= 1.0:
            raise ValueError('threshold must be in [0, 1]')
        if not 1 <= votes_required <= vote_window:
            raise ValueError('require 1 <= votes_required <= vote_window')
        self.threshold = threshold
        self.votes_required = votes_required
        self.vote_window = vote_window
        self.identity = None
        self.reset()

    def reset(self):
        self.resampler = StreamingResampler()
        self.buffer = np.empty(0, dtype=np.float32)
        self.votes = deque(maxlen=self.vote_window)
        self.start_time = None
        self.window_offset = 0

    def set_identity(self, identity):
        if identity != self.identity:
            self.reset()
            self.identity = identity

    def push(self, samples, start_time):
        if self.start_time is None:
            self.start_time = start_time
        self.buffer = np.concatenate((self.buffer, self.resampler.push(samples)))
        while self.buffer.size >= WINDOW:
            waveform = self.buffer[:WINDOW].copy()
            end_time = self.start_time + (self.window_offset + WINDOW) / MODEL_RATE
            self.buffer = self.buffer[HOP:]
            self.window_offset += HOP
            yield waveform, end_time

    def vote(self, probability):
        if not np.isfinite(probability) or not 0.0 <= probability <= 1.0:
            raise ValueError('model produced an invalid probability')
        self.votes.append(probability >= self.threshold)
        return sum(self.votes) >= self.votes_required
