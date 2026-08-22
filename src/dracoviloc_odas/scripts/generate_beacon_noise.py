#!/usr/bin/env python3
"""Generate repeatable band-limited pink noise for phone playback."""

import argparse
from pathlib import Path

import numpy as np
from scipy.io import wavfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--lo", type=float, required=True)
    parser.add_argument("--hi", type=float, required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--sample-rate", type=int, default=44100)
    parser.add_argument("--level-dbfs", type=float, default=-6.0)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    nyquist = args.sample_rate / 2.0
    if not 0.0 < args.lo < args.hi < nyquist:
        raise SystemExit("require 0 < lo < hi < Nyquist")
    if args.duration <= 0.0:
        raise SystemExit("duration must be positive")

    count = int(round(args.duration * args.sample_rate))
    frequencies = np.fft.rfftfreq(count, 1.0 / args.sample_rate)
    rng = np.random.default_rng(args.seed)
    spectrum = rng.normal(size=len(frequencies)) + 1j * rng.normal(size=len(frequencies))
    spectrum[0] = 0.0
    spectrum /= np.sqrt(np.maximum(frequencies, 1.0))

    taper = min(150.0, (args.hi - args.lo) / 4.0)
    envelope = np.zeros_like(frequencies)
    core = (frequencies >= args.lo + taper) & (frequencies <= args.hi - taper)
    envelope[core] = 1.0
    lower = (frequencies >= args.lo) & (frequencies < args.lo + taper)
    upper = (frequencies > args.hi - taper) & (frequencies <= args.hi)
    envelope[lower] = 0.5 - 0.5 * np.cos(
        np.pi * (frequencies[lower] - args.lo) / taper)
    envelope[upper] = 0.5 + 0.5 * np.cos(
        np.pi * (frequencies[upper] - (args.hi - taper)) / taper)
    signal = np.fft.irfft(spectrum * envelope, n=count)

    fade_samples = min(int(0.05 * args.sample_rate), count // 2)
    fade = np.linspace(0.0, 1.0, fade_samples)
    signal[:fade_samples] *= fade
    signal[-fade_samples:] *= fade[::-1]
    peak = np.max(np.abs(signal))
    signal *= (10.0 ** (args.level_dbfs / 20.0)) / max(peak, 1.0e-12)
    pcm = np.round(np.clip(signal, -1.0, 1.0) * 32767.0).astype(np.int16)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    wavfile.write(args.output, args.sample_rate, pcm)
    print(f"Wrote {args.output}: {args.duration:.1f} s, {args.lo:.0f}-{args.hi:.0f} Hz "
          f"pink noise, peak {args.level_dbfs:.1f} dBFS")


if __name__ == "__main__":
    main()
