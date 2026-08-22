#!/usr/bin/env python3
"""Compare quiet and target WAV recordings and recommend localization bands."""

import argparse
import csv
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import welch


def load_wav(path):
    sample_rate, samples = wavfile.read(path)
    if samples.ndim == 1:
        samples = samples[:, None]
    if np.issubdtype(samples.dtype, np.integer):
        scale = float(max(abs(np.iinfo(samples.dtype).min), np.iinfo(samples.dtype).max))
        samples = samples.astype(np.float64) / scale
    else:
        samples = samples.astype(np.float64)
    samples -= np.mean(samples, axis=0, keepdims=True)
    return sample_rate, samples


def average_psd(samples, sample_rate):
    nperseg = min(8192, samples.shape[0])
    frequencies, spectra = welch(
        samples, fs=sample_rate, axis=0, nperseg=nperseg,
        noverlap=nperseg // 2, window="hann", detrend=False)
    return frequencies, np.median(spectra, axis=1)


def smooth(values, bins):
    bins = max(1, int(bins))
    kernel = np.ones(bins, dtype=np.float64) / bins
    return np.convolve(values, kernel, mode="same")


def contiguous_regions(mask, frequencies, max_gap_hz, minimum_width_hz):
    true_indices = np.flatnonzero(mask)
    if not len(true_indices):
        return []
    groups = [[true_indices[0]]]
    for index in true_indices[1:]:
        if frequencies[index] - frequencies[groups[-1][-1]] <= max_gap_hz:
            groups[-1].append(index)
        else:
            groups.append([index])
    return [group for group in groups
            if frequencies[group[-1]] - frequencies[group[0]] >= minimum_width_hz]


def rms_dbfs(samples):
    rms = np.sqrt(np.mean(np.square(samples)))
    return 20.0 * np.log10(max(rms, 1.0e-12))


def main():
    parser = argparse.ArgumentParser(
        description="Find frequency bands where a target recording exceeds ambient noise.")
    parser.add_argument("quiet_wav", type=Path)
    parser.add_argument("target_wav", type=Path)
    parser.add_argument("--minimum-frequency", type=float, default=300.0)
    parser.add_argument("--maximum-frequency", type=float, default=3800.0,
                        help="keep below the UMA16v2 spatial-aliasing ceiling")
    parser.add_argument("--minimum-snr", type=float, default=6.0)
    parser.add_argument("--minimum-bandwidth", type=float, default=300.0)
    parser.add_argument("--maximum-gap", type=float, default=100.0)
    parser.add_argument("--csv", type=Path, help="optional full spectrum CSV output")
    args = parser.parse_args()

    quiet_rate, quiet = load_wav(args.quiet_wav)
    target_rate, target = load_wav(args.target_wav)
    if quiet_rate != target_rate:
        raise SystemExit("recordings must use the same sample rate")
    frequencies, quiet_psd = average_psd(quiet, quiet_rate)
    target_frequencies, target_psd = average_psd(target, target_rate)
    if not np.array_equal(frequencies, target_frequencies):
        raise SystemExit("recordings produced incompatible frequency bins")

    bin_width = frequencies[1] - frequencies[0]
    smoothing_bins = max(1, round(100.0 / bin_width))
    quiet_psd = smooth(quiet_psd, smoothing_bins)
    target_psd = smooth(target_psd, smoothing_bins)
    epsilon = 1.0e-20
    snr_db = 10.0 * np.log10((target_psd + epsilon) / (quiet_psd + epsilon))
    target_db = 10.0 * np.log10(target_psd + epsilon)
    useful = (
        (frequencies >= args.minimum_frequency)
        & (frequencies <= args.maximum_frequency)
        & (snr_db >= args.minimum_snr)
        & (target_db >= np.max(target_db) - 35.0)
    )
    regions = contiguous_regions(
        useful, frequencies, args.maximum_gap, args.minimum_bandwidth)
    ranked = []
    for region in regions:
        low, high = frequencies[region[0]], frequencies[region[-1]]
        median_snr = float(np.median(snr_db[region]))
        peak_snr = float(np.max(snr_db[region]))
        ranked.append((median_snr, high - low, low, high, peak_snr))
    ranked.sort(reverse=True)

    print(f"quiet:  {args.quiet_wav}  RMS={rms_dbfs(quiet):+.1f} dBFS")
    print(f"target: {args.target_wav}  RMS={rms_dbfs(target):+.1f} dBFS")
    print(f"analysis range: {args.minimum_frequency:.0f}-{args.maximum_frequency:.0f} Hz")
    if not ranked:
        print("No sufficiently wide band met the requested SNR threshold.")
        print("Try a louder/closer source or lower --minimum-snr cautiously.")
    else:
        print("Recommended broadband bands (best median SNR first):")
        for rank, (median_snr, width, low, high, peak_snr) in enumerate(ranked[:5], 1):
            print(f"  {rank}. {low:.0f}-{high:.0f} Hz  width={width:.0f} Hz  "
                  f"median SNR={median_snr:.1f} dB  peak={peak_snr:.1f} dB")
        best = ranked[0]
        print(f"\nFeeder trial: --lo {best[2]:.0f} --hi {best[3]:.0f}")

    if args.csv:
        with args.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["frequency_hz", "quiet_psd_db", "target_psd_db", "snr_db"])
            for frequency, quiet_value, target_value, snr in zip(
                    frequencies, quiet_psd, target_psd, snr_db):
                writer.writerow([
                    frequency,
                    10.0 * np.log10(quiet_value + epsilon),
                    10.0 * np.log10(target_value + epsilon),
                    snr])
        print(f"Spectrum CSV written to {args.csv}")


if __name__ == "__main__":
    main()
