"""Satisfies the AST feature extractor's residual torch calls using numpy.

transformers computes the mel filterbank in numpy when torchaudio is absent,
but still calls torch.from_numpy / torch.nn.ZeroPad2d / .numpy() to pad the
result. This injects a minimal shim providing exactly those three operations.
Import before instantiating AutoFeatureExtractor.
"""
import numpy as np
from transformers.models.audio_spectrogram_transformer import (
    feature_extraction_audio_spectrogram_transformer as _ast,
)


class _Tensor(np.ndarray):
    def numpy(self):
        return np.asarray(self)


def _wrap(a):
    return np.asarray(a).view(_Tensor)


class _ZeroPad2d:
    def __init__(self, padding):
        self.left, self.right, self.top, self.bottom = padding

    def __call__(self, x):
        return _wrap(np.pad(
            np.asarray(x),
            ((self.top, self.bottom), (self.left, self.right)),
            mode="constant",
        ))


class _nn:
    ZeroPad2d = _ZeroPad2d


class _TorchShim:
    nn = _nn

    @staticmethod
    def from_numpy(a):
        return _wrap(a)


_ast.torch = _TorchShim
