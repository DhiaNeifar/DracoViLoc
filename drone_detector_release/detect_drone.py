#!/usr/bin/env python3
"""Run the local SAMID drone model on one audio file.

Send this script together with the complete ``samid-finetuned`` directory:

    package/
    |-- detect_drone.py
    `-- model/
        |-- model.safetensors
        |-- config.json
        `-- preprocessor_config.json

Install dependencies:

    python -m pip install torch transformers numpy scipy soundfile

Run:

    python detect_drone.py recording.wav

The script automatically uses CUDA/FP16 when available and CPU/FP32 otherwise.
"""

from __future__ import annotations

import argparse
import sys
from fractions import Fraction
from pathlib import Path

import numpy as np
import soundfile as sf
import torch
from scipy.ndimage import median_filter
from scipy.signal import resample_poly
from transformers import AutoFeatureExtractor, AutoModelForAudioClassification


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL = SCRIPT_DIR / "model"

WINDOW_SECONDS = 1.0
HOP_SECONDS = 0.5
THRESHOLD = 0.5
MEDIAN_KERNEL = 5
CONSECUTIVE_WINDOWS = 3
SILENCE_DBFS = -55.0
BATCH_SIZE = 32


def choose_device(force_cpu: bool) -> tuple[torch.device, torch.dtype]:
    if not force_cpu and torch.cuda.is_available():
        torch.backends.cuda.matmul.allow_tf32 = True
        return torch.device("cuda"), torch.float16
    return torch.device("cpu"), torch.float32


def find_drone_class(id2label: dict) -> int:
    for index, label in id2label.items():
        normalized = str(label).lower().replace("-", "_").replace(" ", "_")
        if "drone" in normalized and not normalized.startswith(("no", "non")):
            return int(index)
    raise ValueError(f"The model has no recognizable drone label: {id2label}")


def load_audio(path: Path, target_rate: int) -> np.ndarray:
    audio, source_rate = sf.read(path, dtype="float32", always_2d=True)
    if audio.size == 0:
        raise ValueError("The audio file is empty")

    # Average stereo or multichannel recordings into one mono channel.
    audio = audio.mean(axis=1)

    if source_rate != target_rate:
        ratio = Fraction(target_rate, source_rate).limit_denominator(1000)
        audio = resample_poly(audio, ratio.numerator, ratio.denominator).astype(np.float32)

    if not np.isfinite(audio).all():
        raise ValueError("The audio contains NaN or infinite samples")
    return np.asarray(audio, dtype=np.float32)


def make_windows(audio: np.ndarray, size: int, hop: int) -> list[np.ndarray]:
    if len(audio) <= size:
        return [audio]

    starts = list(range(0, len(audio) - size + 1, hop))
    chunks = [audio[start : start + size] for start in starts]

    # Keep a final partial window only when at least half a window remains.
    next_start = starts[-1] + hop
    tail = audio[next_start:]
    if starts[-1] + size < len(audio) and len(tail) >= size // 2:
        chunks.append(tail)
    return chunks


def dbfs(audio: np.ndarray) -> float:
    rms = float(np.sqrt(np.mean(np.square(audio, dtype=np.float64)) + 1e-12))
    return 20.0 * np.log10(rms)


@torch.inference_mode()
def score_windows(
    model,
    extractor,
    chunks: list[np.ndarray],
    drone_class: int,
    sample_rate: int,
    device: torch.device,
    dtype: torch.dtype,
    batch_size: int,
    silence_dbfs: float,
) -> list[float]:
    scores = [0.0] * len(chunks)
    audible = [index for index, chunk in enumerate(chunks) if dbfs(chunk) >= silence_dbfs]

    for start in range(0, len(audible), batch_size):
        indices = audible[start : start + batch_size]
        batch = [chunks[index] for index in indices]
        inputs = extractor(batch, sampling_rate=sample_rate, return_tensors="pt")
        values = inputs["input_values"].to(device=device, dtype=dtype)
        logits = model(input_values=values).logits
        probabilities = torch.softmax(logits, dim=-1)[:, drone_class]

        for index, probability in zip(indices, probabilities.float().cpu().numpy()):
            scores[index] = float(probability)

    return scores


def smooth_scores(scores: list[float], kernel: int) -> np.ndarray:
    kernel = max(1, int(kernel))
    if kernel % 2 == 0:
        kernel += 1
    return median_filter(np.asarray(scores, dtype=np.float32), size=kernel, mode="nearest")


def positive_runs(scores: np.ndarray, threshold: float) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None

    for index, positive in enumerate(scores >= threshold):
        if positive and start is None:
            start = index
        elif not positive and start is not None:
            runs.append((start, index))
            start = None

    if start is not None:
        runs.append((start, len(scores)))
    return runs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audio", type=Path, help="audio file to analyze")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="model directory (default: model beside this script)",
    )
    parser.add_argument("--threshold", type=float, default=THRESHOLD)
    parser.add_argument("--window-seconds", type=float, default=WINDOW_SECONDS)
    parser.add_argument("--hop-seconds", type=float, default=HOP_SECONDS)
    parser.add_argument("--median-kernel", type=int, default=MEDIAN_KERNEL)
    parser.add_argument("--consecutive", type=int, default=CONSECUTIVE_WINDOWS)
    parser.add_argument("--silence-dbfs", type=float, default=SILENCE_DBFS)
    parser.add_argument("--batch-size", type=int, default=BATCH_SIZE)
    parser.add_argument("--cpu", action="store_true", help="force CPU inference")
    args = parser.parse_args()

    if not 0.0 < args.threshold < 1.0:
        parser.error("--threshold must be between 0 and 1")
    if args.window_seconds <= 0 or args.hop_seconds <= 0:
        parser.error("window and hop durations must be positive")
    if args.median_kernel < 1 or args.consecutive < 1 or args.batch_size < 1:
        parser.error("median kernel, consecutive count, and batch size must be positive")
    return args


def main() -> int:
    args = parse_args()
    audio_path = args.audio.expanduser().resolve()
    model_path = args.model.expanduser().resolve()

    if not audio_path.is_file():
        print(f"ERROR: audio file not found: {audio_path}", file=sys.stderr)
        return 2
    if not model_path.is_dir():
        print(f"ERROR: model directory not found: {model_path}", file=sys.stderr)
        return 2

    required_files = ("model.safetensors", "config.json", "preprocessor_config.json")
    missing = [name for name in required_files if not (model_path / name).is_file()]
    if missing:
        print(f"ERROR: model directory is missing: {', '.join(missing)}", file=sys.stderr)
        return 2

    device, dtype = choose_device(args.cpu)
    print(f"Loading model from: {model_path}")
    print(f"Device: {device} ({str(dtype).split('.')[-1]})")

    extractor = AutoFeatureExtractor.from_pretrained(model_path, local_files_only=True)
    model = AutoModelForAudioClassification.from_pretrained(
        model_path, local_files_only=True
    ).to(device=device, dtype=dtype).eval()

    drone_class = find_drone_class(model.config.id2label)
    sample_rate = int(extractor.sampling_rate)
    audio = load_audio(audio_path, sample_rate)
    window_size = int(round(args.window_seconds * sample_rate))
    hop_size = int(round(args.hop_seconds * sample_rate))
    chunks = make_windows(audio, window_size, hop_size)

    raw = score_windows(
        model=model,
        extractor=extractor,
        chunks=chunks,
        drone_class=drone_class,
        sample_rate=sample_rate,
        device=device,
        dtype=dtype,
        batch_size=args.batch_size,
        silence_dbfs=args.silence_dbfs,
    )
    smoothed = smooth_scores(raw, args.median_kernel)

    # A short file cannot supply three windows, so require all available windows.
    required = min(args.consecutive, len(smoothed))
    runs = [
        (start, stop)
        for start, stop in positive_runs(smoothed, args.threshold)
        if stop - start >= required
    ]

    duration = len(audio) / sample_rate
    print()
    print("=" * 64)
    print(f"Audio:              {audio_path.name}")
    print(f"Duration:           {duration:.2f} seconds")
    print(f"Analysis windows:   {len(chunks)}")
    print(f"Raw maximum:        {max(raw) * 100:.1f}%")
    print(f"Smoothed mean:      {float(np.mean(smoothed)) * 100:.1f}%")
    print(f"Smoothed maximum:   {float(np.max(smoothed)) * 100:.1f}%")
    print(f"Decision threshold: {args.threshold * 100:.1f}%")
    print(f"Required positives: {required}")

    if runs:
        print("\nDetected event(s):")
        for number, (start, stop) in enumerate(runs, 1):
            begin = start * args.hop_seconds
            end = min(duration, (stop - 1) * args.hop_seconds + args.window_seconds)
            peak = float(np.max(smoothed[start:stop]))
            print(f"  {number}. {begin:.2f}s to {end:.2f}s, peak {peak * 100:.1f}%")

    print("\nPrediction:", "DRONE DETECTED" if runs else "NO DRONE DETECTED")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
