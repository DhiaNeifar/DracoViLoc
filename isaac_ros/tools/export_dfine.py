#!/usr/bin/env python3
"""Export a D-FINE checkpoint (.pth) to a static ONNX model (batch 1, 640x640).

The ONNX model has the RT-DETR interface used by isaac_ros_rtdetr:
inputs ``images`` [1,3,640,640] (RGB, 0..1) and ``orig_target_sizes`` [1,2]
(int64 width, height), outputs ``labels`` [1,300] (int64), ``boxes``
[1,300,4] (x1,y1,x2,y2 in orig_target_sizes pixels) and ``scores`` [1,300].

Only the model definition of the D-FINE source tree (--dfine) is used. Its
package imports training-only libraries (TensorBoard, COCO evaluation, ...);
those are replaced by empty modules, so the pinned export environment needs
nothing beyond torch and onnx. The number of classes is read from the
checkpoint; the model size (n, s, m, l, x) selects the matching
configs/dfine/custom/dfine_hgnetv2_<size>_custom.yml.

With --metadata, also writes <model>.onnx.json like export_yolo.py.
"""

import hashlib
import importlib.abc
import importlib.machinery
import json
import sys
import types
from argparse import ArgumentParser
from datetime import datetime, timezone
from pathlib import Path

TRAINING_ONLY = ("torch.utils.tensorboard", "tensorboard", "faster_coco_eval", "pycocotools",
                 "scipy", "calflops", "transformers", "loguru")


class _Placeholder(types.ModuleType):
    """Any attribute is a do-nothing class, enough for import-time references."""

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return type(name, (), {"__init__": lambda self, *args, **kwargs: None})


class _TrainingOnlyStubs(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, name, path, target=None):
        if any(name == stub or name.startswith(stub + ".") for stub in TRAINING_ONLY):
            return importlib.machinery.ModuleSpec(name, self, is_package=True)
        return None

    def create_module(self, spec):
        module = _Placeholder(spec.name)
        module.__path__ = []
        return module

    def exec_module(self, module):
        pass


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
    parser.add_argument("--model", type=Path, required=True, help="D-FINE .pth checkpoint")
    parser.add_argument("--size", required=True, choices=["n", "s", "m", "l", "x"])
    parser.add_argument("--dfine", type=Path, required=True, help="D-FINE source tree")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--metadata", action="store_true",
                        help="Write <model>.onnx.json next to the exported model")
    args = parser.parse_args()

    model_path = args.model.resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {model_path}")

    sys.meta_path.insert(0, _TrainingOnlyStubs())
    sys.path.insert(0, str(args.dfine.resolve()))
    import torch
    from src.core import YAMLConfig

    checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
    state = checkpoint["ema"]["module"] if "ema" in checkpoint else checkpoint["model"]
    num_classes = int(state["decoder.enc_score_head.weight"].shape[0])
    config = args.dfine / f"configs/dfine/custom/dfine_hgnetv2_{args.size}_custom.yml"
    cfg = YAMLConfig(str(config), num_classes=num_classes,
                     eval_spatial_size=[args.imgsz, args.imgsz])
    cfg.yaml_cfg["HGNetv2"]["pretrained"] = False
    cfg.model.load_state_dict(state)  # strict: a wrong size or class count fails here
    print(f"Loaded {model_path.name}: D-FINE-{args.size.upper()}, {num_classes} class(es), "
          f"epoch {checkpoint.get('last_epoch')}", flush=True)

    class Deployed(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.model = cfg.model.deploy()
            self.postprocessor = cfg.postprocessor.deploy()

        def forward(self, images, orig_target_sizes):
            return self.postprocessor(self.model(images), orig_target_sizes)

    model = Deployed().eval()
    images = torch.rand(1, 3, args.imgsz, args.imgsz)
    sizes = torch.tensor([[args.imgsz, args.imgsz]], dtype=torch.int64)
    onnx_path = model_path.with_suffix(".onnx")
    with torch.no_grad():
        model(images, sizes)
        torch.onnx.export(model, (images, sizes), str(onnx_path),
                          input_names=["images", "orig_target_sizes"],
                          output_names=["labels", "boxes", "scores"],
                          opset_version=17, do_constant_folding=True)

    import onnx
    import onnxslim
    slim = onnxslim.slim(onnx.load(str(onnx_path)))
    onnx.checker.check_model(slim)
    onnx.save(slim, str(onnx_path))
    print(f"ONNX model exported to: {onnx_path}", flush=True)

    if args.metadata:
        graph = onnx.load(str(onnx_path)).graph
        metadata = {
            "source": model_path.name,
            "source_sha256": sha256(model_path),
            "onnx_sha256": sha256(onnx_path),
            "exported_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "family": "dfine",
            "size": args.size,
            "num_classes": num_classes,
            "imgsz": args.imgsz,
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
