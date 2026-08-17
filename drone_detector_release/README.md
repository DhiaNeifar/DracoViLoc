# Drone WAV Classifier

This folder is a self-contained acoustic drone classifier. Give the script one
WAV file and it reports either:

```text
Prediction: DRONE DETECTED
```

or:

```text
Prediction: NO DRONE DETECTED
```

The bundled checkpoint is a byte-for-byte copy of the newest complete
fine-tuned model available in the source project:
`models/candidates/20260815-strong-seed2026`.

## Folder contents

```text
drone_detector_release/
|-- detect_drone.py
|-- requirements.txt
|-- .gitattributes
`-- model/
    |-- model.safetensors
    |-- config.json
    |-- preprocessor_config.json
    `-- training_report.json
```

## Clone from GitHub

The model weight file is approximately 341 MB, which exceeds GitHub's normal
100 MB file limit. This folder configures the weight file for Git LFS. Install
Git LFS before cloning or pushing the repository:

```powershell
git lfs install
git clone <repository-url>
cd <repository-folder>\drone_detector_release
git lfs pull
```

Confirm that `model/model.safetensors` is approximately 341 MB. If it is only a
small text pointer, run `git lfs pull` again.

## Install

The source project uses Python 3.12.6. The dependency versions in
`requirements.txt` are pinned to the exact versions recorded in its `.venv`,
including the CUDA 12.8 build of PyTorch.

### Windows PowerShell

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### Linux

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

The requirements use the PyTorch CUDA 12.8 package index to reproduce the
source environment. The script automatically uses CUDA/FP16 when CUDA is
available and CPU/FP32 otherwise.

## Run

Pass a single WAV path:

```powershell
python detect_drone.py "C:\path\to\recording.wav"
```

Example output:

```text
Prediction: DRONE DETECTED
```

The script also displays the device, duration, window count, scores, threshold,
and detected time ranges. It exits with code `0` after a valid classification
and code `2` when the input or bundled model path is invalid.

## Bundled model

| Property | Value |
|---|---:|
| Architecture | Audio Spectrogram Transformer |
| Classes | `no_drone`, `drone` |
| Parameters | 85,370,114 |
| Weight-file size | 341,509,144 bytes |
| Input sample rate | 16,000 Hz |
| Input window | 1 second |
| Selected training epoch | 5 |
| Training windows | 120,041 |
| Validation windows | 18,134 |
| Balanced validation accuracy | 98.45% |
| Drone validation recall | 96.94% |
| No-drone validation recall | 99.95% |
