"""YOLO model registry: checkpoint -> ONNX -> TensorRT plan for this GPU.

Where things live:

* ``models/yolo/<id>.pt``           trained checkpoint (git LFS, source of truth)
* ``models/yolo/<id>.onnx[.json]``  portable export + export record (git LFS);
                                    exported once, usable on every machine
* ``<isaac>/models/<id>.onnx``      staged copy read by the YOLO container
* ``<isaac>/models/<id>.<gpu>.plan[.json]``
                                    TensorRT plan + build record; machine
                                    specific, so the GPU is part of its name

Export runs the image's pinned CPU environment ($YOLO_EXPORT_PYTHON) without
network; plans are built by trtexec with the Isaac TensorRT node's defaults
(FP16, 64 MiB workspace, batch 1), then benchmarked once.

Also a command-line tool:  python -m dracoviloc_gui.yolo_models
{list | export ID | build-plan ID | prepare ID|auto | resolve ID|auto}
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

from .host_platform import AUTO, gpu_name, isaac_root, project_image_version, resolve_yolo_image

CONTAINER_WORKSPACE = "/workspaces/isaac_ros-dev"
TRTEXEC = "/usr/src/tensorrt/bin/trtexec"
PLAN_ARGS = ("--fp16", "--memPoolSize=workspace:64M")
DATE = re.compile(r"(20\d{6})")
CACHE = Path.home() / ".cache/dracoviloc"

OK, STALE, MISSING, UNAVAILABLE = "ok", "stale", "missing", "unavailable"


@dataclass
class ModelInfo:
    id: str
    date: str
    legacy: bool
    has_checkpoint: bool
    onnx_state: str
    onnx_detail: str
    plan_state: str
    plan_detail: str
    plan_file: str

    @property
    def ready(self) -> bool:
        return self.plan_state == OK

    def to_dict(self) -> dict:
        data = asdict(self)
        data["ready"] = self.ready
        return data


def gpu_tag(name: str | None = None) -> str:
    """Short, file-name-safe GPU identifier, e.g. 'orin' or 'quadro-t2000-with-max-q-design'."""
    name = gpu_name() if name is None else name
    name = re.sub(r"\(.*?\)", "", name).lower()
    name = re.sub(r"\b(nvidia|geforce)\b", "", name)
    return re.sub(r"[^a-z0-9]+", "-", name).strip("-") or "unknown-gpu"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _cached_sha256(path: Path) -> str:
    """sha256 cached by (path, size, mtime): checks run often, files rarely change."""
    stat = path.stat()
    key = hashlib.sha1(f"{path.resolve()}|{stat.st_size}|{stat.st_mtime_ns}".encode()).hexdigest()
    cache = CACHE / "sha256" / key
    try:
        return cache.read_text().strip()
    except OSError:
        pass
    value = sha256(path)
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(value)
    return value


def _read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def model_date(model_id: str, fallback: Path | None = None) -> str:
    match = DATE.search(model_id)
    if match:
        raw = match.group(1)
        return f"{raw[:4]}-{raw[4:6]}-{raw[6:]}"
    if fallback and fallback.exists():
        return datetime.fromtimestamp(fallback.stat().st_mtime).strftime("%Y-%m-%d")
    return ""


def tensorrt_version(image: str) -> str:
    """TensorRT version inside an image (cached per image ID; no GPU needed)."""
    try:
        image_id = subprocess.run(["docker", "image", "inspect", image, "--format", "{{.Id}}"],
                                  capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""
    if not image_id:
        return ""
    cache = CACHE / "tensorrt" / image_id.replace(":", "_")
    try:
        return cache.read_text().strip()
    except OSError:
        pass
    result = subprocess.run(
        ["docker", "run", "--rm", "--network", "none", "--entrypoint", "dpkg-query", image,
         "-W", "-f=${Version}", "libnvinfer10"], capture_output=True, text=True, timeout=60)
    version = result.stdout.strip() if result.returncode == 0 else ""
    if version:
        cache.parent.mkdir(parents=True, exist_ok=True)
        cache.write_text(version)
    return version


class Registry:
    def __init__(self, project_root: Path, workspace: Path | None = None,
                 gpu: str | None = None, image: str | None = None) -> None:
        self.project_root = Path(project_root)
        self.models = self.project_root / "models/yolo"
        self.workspace = Path(workspace) if workspace else isaac_root()
        self.staged = self.workspace / "models"
        self.gpu = gpu_name() if gpu is None else gpu
        self.tag = gpu_tag(self.gpu)
        self._image = image

    @property
    def image(self) -> str:
        if self._image is None:
            self._image = resolve_yolo_image(AUTO)
        return self._image

    def plan_path(self, model_id: str) -> Path:
        return self.staged / f"{model_id}.{self.tag}.plan"

    def container_paths(self, model_id: str, legacy: bool = False) -> tuple[str, str]:
        plan = f"{model_id}.plan" if legacy else self.plan_path(model_id).name
        return f"{CONTAINER_WORKSPACE}/models/{model_id}.onnx", f"{CONTAINER_WORKSPACE}/models/{plan}"

    # ------------------------------------------------------------------ status
    def onnx_status(self, model_id: str) -> tuple[str, str]:
        checkpoint = self.models / f"{model_id}.pt"
        onnx = self.models / f"{model_id}.onnx"
        if not onnx.exists():
            return MISSING, "Not exported yet"
        record = _read_json(onnx.with_name(onnx.name + ".json"))
        if record is None:
            return OK, "Present (no export record)"
        if checkpoint.exists() and record.get("source_sha256") != _cached_sha256(checkpoint):
            return STALE, "The checkpoint changed after this export"
        outputs = ", ".join(f"{name} {shape}" for name, shape in record.get("outputs", {}).items())
        return OK, f"Exported {record.get('exported_at', '')[:10]} · {outputs}"

    def plan_status(self, model_id: str, onnx_state: str, trt_version: str | None = None) -> tuple[str, str]:
        if not self.gpu:
            return UNAVAILABLE, "No NVIDIA GPU on this machine"
        if onnx_state not in {OK}:
            return MISSING, "Needs an up-to-date ONNX export first"
        plan = self.plan_path(model_id)
        if not plan.exists():
            return MISSING, f"No plan for {self.gpu}"
        record = _read_json(plan.with_name(plan.name + ".json")) or {}
        onnx_sha = _cached_sha256(self.models / f"{model_id}.onnx")
        if record.get("onnx_sha256") != onnx_sha:
            return STALE, "Built from a different ONNX file"
        if trt_version and record.get("tensorrt") and record["tensorrt"] != trt_version:
            return STALE, f"Built with TensorRT {record['tensorrt']}, image has {trt_version}"
        speed = record.get("throughput_qps")
        return OK, (f"FP16 plan for {self.gpu}" + (f" · {speed:.0f} inferences/s" if speed else ""))

    def list(self, check_tensorrt: bool = True) -> list[ModelInfo]:
        trt = tensorrt_version(self.image) if (check_tensorrt and self.gpu and self.image) else None
        ids = sorted({path.stem for pattern in ("*.pt", "*.onnx")
                      for path in self.models.glob(pattern)})
        models = []
        for model_id in ids:
            onnx_state, onnx_detail = self.onnx_status(model_id)
            plan_state, plan_detail = self.plan_status(model_id, onnx_state, trt)
            models.append(ModelInfo(
                model_id, model_date(model_id, self.models / f"{model_id}.pt"), False,
                (self.models / f"{model_id}.pt").exists(), onnx_state, onnx_detail,
                plan_state, plan_detail, str(self.plan_path(model_id))))
        # Models that only exist in the Isaac workspace (deployed before this
        # registry). Their plan's GPU is unknown, so they are never chosen
        # automatically but can still be selected by name.
        for onnx in sorted(self.staged.glob("*.onnx")):
            model_id = onnx.stem
            if model_id in ids:
                continue
            plan = self.staged / f"{model_id}.plan"
            models.append(ModelInfo(
                model_id, model_date(model_id, onnx), True, False, OK,
                "Isaac workspace only (legacy)",
                OK if plan.exists() else MISSING,
                "Legacy plan: GPU unknown" if plan.exists() else "No plan",
                str(plan)))
        return sorted(models, key=lambda model: (model.date, model.id), reverse=True)

    def resolve(self, choice: str, models: list[ModelInfo] | None = None) -> ModelInfo:
        """Model used for a run: the chosen one, or for 'auto' the newest ready one."""
        models = self.list() if models is None else models
        if choice in ("", AUTO):
            ready = [model for model in models if model.ready and not model.legacy]
            if not ready:
                raise LookupError(
                    f"No YOLO model has a TensorRT plan for {self.gpu or 'this machine'} yet; "
                    "build one in Setup & builds > YOLO models.")
            return ready[0]
        model = next((model for model in models if model.id == choice), None)
        if model is None:
            raise LookupError(f"YOLO model '{choice}' not found")
        if not model.ready:
            raise LookupError(f"YOLO model '{choice}' is not ready: {model.plan_detail}")
        return model

    # ----------------------------------------------------------------- actions
    def _docker(self, extra: list[str], command: list[str], gpu: bool = False) -> subprocess.Popen:
        from .jobs import container_user_args
        args = ["docker", "run", "--rm", "--network", "none", *container_user_args(), *extra]
        if gpu:
            args[2:2] = ["--runtime", "nvidia"]
        print("$ " + " ".join(args + [self.image] + command), flush=True)
        return subprocess.Popen(args + [self.image] + command, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1)

    def export(self, model_id: str) -> int:
        checkpoint = self.models / f"{model_id}.pt"
        if not checkpoint.exists():
            print(f"No checkpoint {checkpoint}", file=sys.stderr)
            return 2
        tools = self.project_root / "isaac_ros/tools"
        process = self._docker(
            ["--name", "dracoviloc_yolo_export", "-v", f"{self.models}:/models",
             "-v", f"{tools}:/tools:ro", "--workdir", "/models"],
            ["bash", "-c", f'exec "$YOLO_EXPORT_PYTHON" /tools/export_yolo.py '
                           f'--model /models/{model_id}.pt --metadata'])
        for line in process.stdout:
            print(line, end="", flush=True)
        return process.wait()

    def stage(self, model_id: str) -> Path:
        """Copy the ONNX into the Isaac workspace if the copy there differs."""
        source = self.models / f"{model_id}.onnx"
        target = self.staged / source.name
        if target.exists() and _cached_sha256(target) == _cached_sha256(source):
            print(f"{target} is already up to date", flush=True)
            return target
        self.staged.mkdir(parents=True, exist_ok=True)
        partial = target.with_name(target.name + ".part")
        shutil.copyfile(source, partial)
        os.replace(partial, target)
        print(f"Staged {source} -> {target}", flush=True)
        return target

    def build_plan(self, model_id: str) -> int:
        if not self.gpu:
            print("No NVIDIA GPU on this machine; TensorRT plans cannot be built here.", file=sys.stderr)
            return 2
        onnx_state, detail = self.onnx_status(model_id)
        if onnx_state != OK:
            print(f"ONNX for {model_id} is {onnx_state}: {detail}", file=sys.stderr)
            return 2
        self.stage(model_id)
        plan = self.plan_path(model_id)
        container_models = f"{CONTAINER_WORKSPACE}/models"
        partial = f"{container_models}/{plan.name}.part"
        script = " && ".join([
            f"{TRTEXEC} --onnx={container_models}/{model_id}.onnx --saveEngine={partial} "
            + " ".join(PLAN_ARGS),
            # Load the new plan once more and benchmark it, to prove it works.
            f"{TRTEXEC} --loadEngine={partial} --warmUp=500 --duration=3",
        ])
        process = self._docker(
            ["--name", "dracoviloc_yolo_plan", "-v", f"{self.workspace}:{CONTAINER_WORKSPACE}"],
            ["bash", "-c", script], gpu=True)
        throughput = None
        for line in process.stdout:
            print(line, end="", flush=True)
            match = re.search(r"Throughput: ([0-9.]+) qps", line)
            if match:
                throughput = float(match.group(1))
        code = process.wait()
        built = plan.with_name(plan.name + ".part")
        if code != 0:
            built.unlink(missing_ok=True)
            return code
        os.replace(built, plan)
        record = {
            "model": model_id,
            "onnx_sha256": _cached_sha256(self.models / f"{model_id}.onnx"),
            "gpu": self.gpu,
            "image": self.image,
            "image_version": project_image_version(self.project_root),
            "tensorrt": tensorrt_version(self.image),
            "trtexec_args": list(PLAN_ARGS),
            "throughput_qps": throughput,
            "built_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
        plan.with_name(plan.name + ".json").write_text(json.dumps(record, indent=2) + "\n")
        print(f"Plan ready: {plan}" + (f" ({throughput:.0f} inferences/s)" if throughput else ""))
        return 0

    def prepare(self, model_id: str) -> int:
        """Export (if needed) and build the plan (if needed) for one model."""
        if model_id in ("", AUTO):
            candidates = [model for model in self.list() if not model.legacy and model.has_checkpoint]
            if not candidates:
                print("No YOLO checkpoints in models/yolo", file=sys.stderr)
                return 2
            model_id = candidates[0].id
            print(f"Newest model: {model_id}", flush=True)
        if self.onnx_status(model_id)[0] != OK:
            code = self.export(model_id)
            if code != 0:
                return code
        trt = tensorrt_version(self.image)
        if self.plan_status(model_id, OK, trt)[0] != OK:
            return self.build_plan(model_id)
        print(f"{model_id} is ready on {self.gpu}", flush=True)
        return 0


def newest_needs_work(registry: Registry) -> ModelInfo | None:
    """The newest checkpoint, if it is not ready on this machine."""
    candidates = [model for model in registry.list() if not model.legacy and model.has_checkpoint]
    if not candidates or candidates[0].ready or candidates[0].plan_state == UNAVAILABLE:
        return None
    return candidates[0]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m dracoviloc_gui.yolo_models",
                                     description=__doc__.split("\n\n")[0])
    parser.add_argument("--project-root", type=Path,
                        default=Path(os.environ.get("DRACOVILOC_ROOT", Path(__file__).resolve().parents[2])))
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list")
    for name in ("export", "build-plan", "prepare", "resolve"):
        commands.add_parser(name).add_argument("model")
    args = parser.parse_args(argv)
    registry = Registry(args.project_root)
    if args.command == "list":
        for model in registry.list():
            print(f"{model.id:40} {model.date:10} onnx={model.onnx_state:8} plan={model.plan_state:8} "
                  f"{model.plan_detail}")
        return 0
    if args.command == "resolve":
        try:
            model = registry.resolve(args.model)
        except LookupError as error:
            print(error, file=sys.stderr)
            return 1
        print(json.dumps({"id": model.id, "paths": registry.container_paths(model.id, model.legacy)}))
        return 0
    action = {"export": registry.export, "build-plan": registry.build_plan,
              "prepare": registry.prepare}[args.command]
    return action(args.model)


if __name__ == "__main__":
    sys.exit(main())
