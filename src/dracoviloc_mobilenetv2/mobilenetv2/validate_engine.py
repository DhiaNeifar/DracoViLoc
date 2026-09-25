#!/usr/bin/env python3
"""Validate the actual runtime runner against the supplied PyTorch reference."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from trt_engine import TrtEngine


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine', required=True, type=Path)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--threshold', type=float, default=0.75)
    args = parser.parse_args()
    with np.load(args.reference, allow_pickle=False) as data:
        waveforms, labels, reference = data['waveforms'], data['labels'], data['pytorch_probs']
    engine = TrtEngine(args.engine)
    outputs, elapsed = [], []
    try:
        for waveform in waveforms:
            start = time.perf_counter()
            outputs.append(engine.infer(waveform))
            elapsed.append(time.perf_counter() - start)
    finally:
        engine.close()
    outputs = np.asarray(outputs)
    difference = np.abs(outputs[:, 1] - reference[:, 1])
    predicted = outputs[:, 1] >= args.threshold
    mismatches = int(np.count_nonzero(predicted != (reference[:, 1] >= args.threshold)))
    report = {
        'clips': len(waveforms), 'max_probability_error': float(difference.max()),
        'mean_probability_error': float(difference.mean()),
        'threshold_decision_mismatches': mismatches,
        'true_positives': int(np.sum(predicted & (labels == 1))),
        'false_positives': int(np.sum(predicted & (labels == 0))),
        'median_inference_ms': float(np.median(elapsed) * 1000),
        'passed': bool(np.isfinite(outputs).all() and difference.max() < 0.01 and mismatches == 0),
    }
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
