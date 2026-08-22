#!/usr/bin/env python3
"""Export an Ultralytics YOLO model to a static ONNX model."""

from argparse import ArgumentParser
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        required=True,
        help="Path to the Ultralytics .pt checkpoint",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Square input image size")
    args = parser.parse_args()

    model_path = args.model.resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {model_path}")

    model = YOLO(str(model_path))
    exported_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        batch=1,
        dynamic=False,
        simplify=True,
        nms=False,
        device="cpu",
    )

    print(f"ONNX model exported to: {exported_path}")


if __name__ == "__main__":
    main()
