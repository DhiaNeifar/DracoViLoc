#!/usr/bin/env python3
"""Export an Ultralytics YOLO model to a static ONNX model.

With --metadata, also writes <model>.onnx.json recording the source checkpoint
(and its SHA-256), the tool versions and the ONNX input/output shapes, so the
control center can tell whether an ONNX file still matches its checkpoint.
"""

import hashlib
import json
from argparse import ArgumentParser
from datetime import datetime, timezone
from pathlib import Path

from ultralytics import YOLO


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def tensor_shapes(values) -> dict:
    return {value.name: [dim.dim_value or dim.dim_param for dim in value.type.tensor_type.shape.dim]
            for value in values}


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument(
        "--model",
        type=Path,
        required=True,
        help="Path to the Ultralytics .pt checkpoint",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Square input image size")
    parser.add_argument("--metadata", action="store_true",
                        help="Write <model>.onnx.json next to the exported model")
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

    if args.metadata:
        import onnx
        import torch
        import ultralytics

        onnx_path = Path(exported_path)
        graph = onnx.load(str(onnx_path)).graph
        metadata = {
            "source": model_path.name,
            "source_sha256": sha256(model_path),
            "onnx_sha256": sha256(onnx_path),
            "exported_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "imgsz": args.imgsz,
            "ultralytics": ultralytics.__version__,
            "torch": torch.__version__,
            "onnx": onnx.__version__,
            "inputs": tensor_shapes(graph.input),
            "outputs": tensor_shapes(graph.output),
        }
        metadata_path = onnx_path.with_name(onnx_path.name + ".json")
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"Export record written to: {metadata_path}")
        print(f"Inputs: {metadata['inputs']}  Outputs: {metadata['outputs']}")


if __name__ == "__main__":
    main()
