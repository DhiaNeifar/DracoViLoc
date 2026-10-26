#!/usr/bin/env python3
"""Pick the MobileNetV2 TensorRT engine and Python interpreter for this machine.

TensorRT engines only load on the GPU they were built for, so
models/mobilenetv2/metadata.json lists each built engine with its GPU
("runtime", "local_runtime", or entries of an optional "runtimes" list).
The engine whose GPU name matches this machine's GPU is selected. The
interpreter is the first candidate that can import TensorRT and PyCUDA.

This file has no ROS dependency. The launch file imports it, and the GUI and
remote agent run it as a script:

    python3 runtime_selection.py --model-dir <checkout>/models/mobilenetv2
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SYSTEM_PYTHON = '/usr/bin/python3'
PROBE = 'import tensorrt, pycuda.driver'


def detect_gpu_name() -> str:
    """Return the NVIDIA GPU name, or '' when no NVIDIA GPU is present."""
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
            capture_output=True, text=True, timeout=5.0)
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip().splitlines()[0].strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    # Jetson boards without nvidia-smi still identify themselves here.
    try:
        model = Path('/proc/device-tree/model').read_text(errors='ignore').strip('\x00 \n')
        if 'jetson' in model.lower() or 'orin' in model.lower():
            return model
    except OSError:
        pass
    return ''


def _runtimes(metadata: dict) -> list[dict]:
    entries = [metadata.get('runtime'), metadata.get('local_runtime')]
    entries += metadata.get('runtimes', [])
    return [entry for entry in entries if isinstance(entry, dict)
            and entry.get('gpu') and entry.get('engine')]


def _matches(gpu: str, runtime_gpu: str) -> bool:
    gpu, runtime_gpu = gpu.lower(), runtime_gpu.lower()
    return runtime_gpu in gpu or gpu in runtime_gpu


def _python_works(python: str) -> bool:
    try:
        return subprocess.run(
            [python, '-c', PROBE], capture_output=True, timeout=30.0).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def select_runtime(model_dir: Path, gpu: str | None = None,
                   probe=_python_works) -> dict:
    """Return {'gpu', 'engine', 'python'}; raise RuntimeError if unsupported."""
    model_dir = Path(model_dir).expanduser().resolve()
    gpu = detect_gpu_name() if gpu is None else gpu
    if not gpu:
        raise RuntimeError(
            'No NVIDIA GPU detected; MobileNetV2 needs TensorRT on an NVIDIA GPU.')
    metadata = json.loads((model_dir / 'metadata.json').read_text(encoding='utf-8'))
    runtime = next((entry for entry in _runtimes(metadata)
                    if _matches(gpu, entry['gpu'])), None)
    if runtime is None:
        known = ', '.join(entry['gpu'] for entry in _runtimes(metadata)) or 'none'
        raise RuntimeError(
            f'No MobileNetV2 engine is built for GPU "{gpu}" (engines exist for: {known}). '
            'Build one with scripts/build_mobilenetv2_engine.py and list it in '
            f'{model_dir / "metadata.json"}.')
    engine = model_dir / runtime['engine']
    project_root = model_dir.parent.parent
    candidates = [runtime.get('python'), str(project_root / 'trt_env/bin/python3'),
                  SYSTEM_PYTHON]
    candidates = [c for i, c in enumerate(candidates)
                  if c and c not in candidates[:i] and os.access(c, os.X_OK)]
    python = next((c for c in candidates if probe(c)), None)
    if python is None:
        raise RuntimeError(
            'No Python interpreter with TensorRT and PyCUDA found (tried: '
            f'{", ".join(candidates) or "none"}).')
    return {'gpu': gpu, 'engine': str(engine), 'python': python}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', required=True, type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(select_runtime(args.model_dir)))
    except (RuntimeError, OSError, ValueError) as error:
        print(json.dumps({'error': str(error)}))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
