import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "first_arrival.py"
SPEC = importlib.util.spec_from_file_location("first_arrival", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
Estimator = MODULE.FirstArrivalEstimator
POSITIONS = MODULE.UMA16_POSITIONS


def pulse_block(direction, reflection=None, samples=512, amplitude=0.3):
    fs = 44100
    speed = 343.0
    result = np.zeros((samples, 16), dtype=np.float64)
    direction = np.asarray(direction, dtype=np.float64)
    delays = np.rint(-(POSITIONS @ direction[:2]) / speed * fs).astype(int)
    delays -= delays.min()
    wave = np.hanning(17) * amplitude
    for channel, delay in enumerate(delays):
        start = 100 + delay
        result[start:start + wave.size, channel] += wave
    if reflection is not None:
        reflected_direction, extra_samples, gain = reflection
        reflected_direction = np.asarray(reflected_direction, dtype=np.float64)
        reflected = np.rint(-(POSITIONS @ reflected_direction[:2]) / speed * fs).astype(int)
        reflected -= reflected.min()
        for channel, delay in enumerate(reflected):
            start = 100 + extra_samples + delay
            result[start:start + wave.size, channel] += wave * gain
    return result


def warmed_estimator(**kwargs):
    kwargs.setdefault("noise_window_s", 0.05)
    estimator = Estimator(min_confidence=0.55, **kwargs)
    rng = np.random.default_rng(7)
    for index in range(12):
        estimator.process(rng.normal(0.0, 1.0e-5, (512, 16)), index * 512 / 44100)
    return estimator


def test_stronger_delayed_reflection_does_not_replace_direct_arrival():
    estimator = warmed_estimator()
    direct = np.array([0.45, -0.25, np.sqrt(1.0 - 0.45 ** 2 - 0.25 ** 2)])
    reflection = np.array([-0.7, 0.2, np.sqrt(1.0 - 0.7 ** 2 - 0.2 ** 2)])
    block = pulse_block(direct, (reflection, 80, 2.5))
    result = estimator.process(block, 1.0)
    assert result is not None
    assert np.dot(result.direction, direct) > 0.97
    assert result.inliers >= 12


def test_noise_only_does_not_publish():
    estimator = warmed_estimator()
    rng = np.random.default_rng(9)
    assert estimator.process(rng.normal(0.0, 1.0e-5, (512, 16)), 1.0) is None


def test_startup_calibration_cannot_create_false_anchor():
    estimator = Estimator(noise_window_s=0.05, min_confidence=0.55)
    vertical_transient = np.ones((512, 16)) * 0.2
    assert estimator.process(vertical_transient, 0.0) is None


def test_incoherent_arrivals_are_rejected():
    estimator = warmed_estimator()
    block = np.zeros((512, 16))
    wave = np.hanning(17) * 0.4
    for channel in range(16):
        start = 40 + channel * 15
        block[start:start + wave.size, channel] = wave
    assert estimator.process(block, 1.0) is None


def test_refractory_period_rejects_late_copy_of_same_event():
    estimator = warmed_estimator(refractory_ms=50.0)
    direction = np.array([0.3, 0.2, np.sqrt(0.87)])
    assert estimator.process(pulse_block(direction), 1.0) is not None
    assert estimator.process(pulse_block(direction), 1.02) is None


def test_invalid_channel_count_is_rejected():
    estimator = warmed_estimator()
    try:
        estimator.process(np.zeros((512, 4)), 1.0)
    except ValueError:
        return
    raise AssertionError("invalid channel count was accepted")
