"""Pure signal-processing core for passive first-arrival direction estimates."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy.signal import butter, sosfilt


UMA16_POSITIONS = np.array([
    [-0.021, -0.063], [-0.063, -0.063], [-0.021, -0.021], [-0.063, -0.021],
    [-0.021, +0.021], [-0.063, +0.021], [-0.021, +0.063], [-0.063, +0.063],
    [+0.063, +0.063], [+0.021, +0.063], [+0.063, +0.021], [+0.021, +0.021],
    [+0.063, -0.021], [+0.021, -0.021], [+0.063, -0.063], [+0.021, -0.063],
], dtype=np.float64)


@dataclass(frozen=True)
class FirstArrivalEstimate:
    timestamp: float
    direction: np.ndarray
    confidence: float
    inliers: int
    residual_us: float
    onset_snr_db: float


class FirstArrivalEstimator:
    """Detect coherent channel onsets and fit a far-field plane wave.

    This deliberately uses absolute onset crossings. Selecting the left-most
    GCC-PHAT lag would be incorrect because correlation lag is a signed
    inter-microphone delay, not source-to-array propagation time.
    """

    def __init__(
        self, sample_rate=44100, positions=UMA16_POSITIONS, sound_speed=343.0,
        low_hz=180.0, high_hz=3600.0, noise_window_s=1.0,
        onset_snr_db=10.0, max_arrival_span_ms=1.2,
        residual_us=150.0, min_inliers=12, min_confidence=0.70,
        refractory_ms=50.0, min_noise_floor=1.0e-5,
    ):
        self.sample_rate = int(sample_rate)
        self.positions = np.asarray(positions, dtype=np.float64)
        if self.positions.ndim != 2 or self.positions.shape[1] != 2:
            raise ValueError("positions must be an Nx2 array")
        self.channels = self.positions.shape[0]
        self.sound_speed = float(sound_speed)
        self.onset_ratio = 10.0 ** (float(onset_snr_db) / 20.0)
        self.max_span_samples = max(
            1, int(round(float(max_arrival_span_ms) * self.sample_rate / 1000.0)))
        self.residual_s = float(residual_us) * 1.0e-6
        self.min_inliers = int(min_inliers)
        self.min_confidence = float(min_confidence)
        self.refractory_s = float(refractory_ms) / 1000.0
        self.min_noise_floor = float(min_noise_floor)
        if not (0 < low_hz < high_hz < self.sample_rate / 2):
            raise ValueError("invalid analysis band")
        if not 3 <= self.min_inliers <= self.channels:
            raise ValueError("min_inliers must be between 3 and channel count")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")

        self.sos = butter(
            4, [low_hz, high_hz], btype="bandpass", fs=self.sample_rate,
            output="sos")
        # Zero state avoids the artificial onset produced by applying a
        # unit-step steady-state zi to near-zero microphone samples.
        self.zi = np.zeros((self.sos.shape[0], 2, self.channels), dtype=np.float64)
        blocks = max(8, int(math.ceil(noise_window_s * self.sample_rate / 512.0)))
        self.noise = deque(maxlen=blocks)
        self.previous = np.zeros((self.max_span_samples, self.channels), dtype=np.float64)
        self.last_event_time = -math.inf

    def _fit(self, arrivals):
        valid = np.isfinite(arrivals)
        if valid.sum() < self.min_inliers:
            return None
        inliers = valid.copy()
        direction_xy = None
        residuals = None
        for _ in range(4):
            indexes = np.flatnonzero(inliers)
            times = arrivals[indexes] / self.sample_rate
            times -= times.mean()
            coordinates = self.positions[indexes]
            coordinates -= coordinates.mean(axis=0)
            # arrival_i - mean(arrival) = -dot(position_i, direction) / c
            direction_xy, _, _, _ = np.linalg.lstsq(
                -coordinates / self.sound_speed, times, rcond=None)
            predicted = -(self.positions - self.positions[indexes].mean(axis=0)) @ (
                direction_xy / self.sound_speed)
            observed = arrivals / self.sample_rate
            observed -= np.nanmean(observed[indexes])
            residuals = np.abs(observed - predicted)
            next_inliers = valid & (residuals <= self.residual_s)
            if next_inliers.sum() < self.min_inliers:
                return None
            if np.array_equal(next_inliers, inliers):
                break
            inliers = next_inliers

        horizontal = float(np.linalg.norm(direction_xy))
        if not math.isfinite(horizontal) or horizontal > 1.05:
            return None
        if horizontal > 1.0:
            direction_xy = direction_xy / horizontal
            horizontal = 1.0
        direction = np.array([
            direction_xy[0], direction_xy[1],
            math.sqrt(max(0.0, 1.0 - horizontal * horizontal)),
        ])
        rms = float(np.sqrt(np.mean(np.square(residuals[inliers]))))
        ratio = float(inliers.sum()) / self.channels
        geometry_score = max(0.0, 1.0 - rms / self.residual_s)
        confidence = 0.5 * ratio + 0.5 * geometry_score
        return direction, confidence, int(inliers.sum()), rms * 1.0e6

    def process(self, samples, block_start_time):
        samples = np.asarray(samples, dtype=np.float64)
        if samples.ndim != 2 or samples.shape[1] != self.channels:
            raise ValueError(f"expected Nx{self.channels} samples")
        if samples.shape[0] == 0 or not np.isfinite(samples).all():
            return None

        filtered, self.zi = sosfilt(self.sos, samples, axis=0, zi=self.zi)
        rms = np.sqrt(np.mean(np.square(filtered), axis=0))
        if len(self.noise) < self.noise.maxlen:
            self.noise.append(rms)
            self.previous = filtered[-self.max_span_samples:].copy()
            return None
        if self.noise:
            history = np.stack(tuple(self.noise))
            floors = np.median(history, axis=0)
            deviations = np.median(np.abs(history - floors[None, :]), axis=0)
        else:
            floors = np.full(self.channels, self.min_noise_floor)
            deviations = np.zeros(self.channels)
        floors = np.maximum(floors, self.min_noise_floor)
        thresholds = np.maximum(floors * self.onset_ratio, floors + 6.0 * deviations)
        combined = np.concatenate((self.previous, filtered), axis=0)
        above = np.abs(combined) >= thresholds[None, :]
        arrivals = np.full(self.channels, np.nan)
        for channel in range(self.channels):
            crossings = np.flatnonzero(above[:, channel])
            if crossings.size:
                arrivals[channel] = crossings[0]

        event_time = float(block_start_time)
        valid = np.isfinite(arrivals)
        coherent = valid.sum() >= self.min_inliers
        if coherent:
            coherent = np.nanmax(arrivals) - np.nanmin(arrivals) <= self.max_span_samples
        refractory = event_time - self.last_event_time < self.refractory_s
        estimate = None
        if coherent and not refractory:
            fitted = self._fit(arrivals)
            if fitted is not None:
                direction, confidence, inliers, residual = fitted
                peak_ratio = float(np.median(np.max(np.abs(combined), axis=0) / floors))
                snr_db = 20.0 * math.log10(max(peak_ratio, 1.0e-12))
                first = float(np.nanmin(arrivals) - len(self.previous)) / self.sample_rate
                timestamp = event_time + first
                if confidence >= self.min_confidence:
                    estimate = FirstArrivalEstimate(
                        timestamp, direction, confidence, inliers, residual, snr_db)
                    self.last_event_time = timestamp

        # Only quiet blocks train the background model; otherwise a loud event
        # would raise its own threshold and suppress the next direct arrival.
        if not coherent:
            self.noise.append(rms)
        self.previous = filtered[-self.max_span_samples:].copy()
        return estimate
