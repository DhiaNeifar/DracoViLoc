"""Fetch trained visual models (YOLO, D-FINE) from Google Drive and export them to ONNX.

The Drive folder holds one folder per training run, named
``<YYYY_MM_DD_HH_MM_SS>_<arch>_<name>/`` with a ``done.json``:

* YOLO     ``2026_10_24_03_31_54_yolo11n_par/weights/best.pt``
           -> ``models/yolo/par_yolo11n_20261024_best.pt``
* D-FINE   ``2026_10_24_04_17_49_dfine_s_full/best_stg2.pth`` (or ``best_stg1.pth``
           when stage 2 never beat stage 1) -> ``models/dfine/full_dfine_s_20261024_best.pth``

A run is fetched once it is complete (``done.json`` says so) under the name
the model lists show. Runs already fetched are recorded in
``models/<family>/drive_models.json`` (run name -> model id, checksum,
metrics) and are not downloaded again; a download identical to a checkpoint
already here is linked to it instead of being added twice. Every checkpoint
without an up-to-date ONNX export is then exported with the registry (pinned
environment in the project image, no network). Full output goes to
``logs/jobs/<time>_fetch_yolo_models.log``.

Usage:  scripts/fetch_yolo_models.sh [--no-export] [--folder ID]

With ``--gui`` progress is printed as ``PROGRESS {json}`` lines, which the
Setup & builds page turns into progress bars, instead of tqdm bars.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from .session_log import RUN_NAME_FORMAT
from .yolo_models import FAMILIES, OK, Registry, _cached_sha256, family_of, sha256

DRIVE_FOLDER = "1ptxQRwCKWE8-DYbIK-FYwNptAi5xLl7g"
MANIFEST = "drive_models.json"
RUN_NAME = re.compile(r"^(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_([a-z0-9]+)_(.+)$")
# Checkpoint to take from a run folder, by family, in order of preference.
CHECKPOINTS = {"yolo": ("weights/best.pt",), "dfine": ("best_stg2.pth", "best_stg1.pth")}


@dataclass
class DriveRun:
    name: str           # Drive folder name, e.g. 2026_10_24_03_31_54_yolo11n_par
    checkpoint_id: str  # Drive file id of the checkpoint
    done_id: str        # Drive file id of done.json
    family: str = "yolo"

    @property
    def model_id(self) -> str:
        match = RUN_NAME.match(self.name)
        date = "".join(match.group(1, 2, 3))
        arch, rest = match.group(7), match.group(8)
        if self.family == "dfine":  # dfine_<size>_<name> -> <name>_dfine_<size>
            size, _, name = rest.partition("_")
            return f"{name or 'run'}_dfine_{size}_{date}_best"
        return f"{rest}_{arch}_{date}_best"

    @property
    def time_suffix(self) -> str:
        return "".join(RUN_NAME.match(self.name).group(4, 5, 6))


def family_of_run(name: str) -> str:
    return "dfine" if RUN_NAME.match(name).group(7) == "dfine" else "yolo"


def list_drive_runs(folder_id: str) -> list[DriveRun]:
    """Training runs in the Drive folder that have a done.json and a checkpoint."""
    import gdown
    files = gdown.download_folder(id=folder_id, skip_download=True, quiet=True,
                                  remaining_ok=True) or []
    by_run: dict[str, dict[str, str]] = {}
    for entry in files:
        run, _, inner = entry.path.partition("/")
        by_run.setdefault(run, {})[inner] = entry.id
    return runs_from_listing(by_run)


def runs_from_listing(by_run: dict[str, dict[str, str]]) -> list[DriveRun]:
    runs = []
    for name, contents in sorted(by_run.items()):
        if not RUN_NAME.match(name) or "done.json" not in contents:
            continue
        family = family_of_run(name)
        checkpoint = next((path for path in CHECKPOINTS[family] if path in contents), None)
        if checkpoint:
            runs.append(DriveRun(name, contents[checkpoint], contents["done.json"], family))
    return runs


def drive_download(file_id: str, target: Path) -> None:
    import gdown
    if not gdown.download(id=file_id, output=str(target), quiet=True):
        raise RuntimeError(f"download of Drive file {file_id} failed")


class Fetcher:
    def __init__(self, project_root: Path, log, download: Callable[[str, Path], None] = drive_download,
                 **registry_options) -> None:
        self.registries = {family: Registry(project_root, family=family, **registry_options)
                           for family in FAMILIES}
        self.manifests = {family: self._load_manifest(registry.models / MANIFEST)
                          for family, registry in self.registries.items()}
        self.download = download
        self.log = log

    @staticmethod
    def _load_manifest(path: Path) -> dict[str, dict]:
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _save_manifest(self, family: str) -> None:
        path = self.registries[family].models / MANIFEST
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.manifests[family], indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
        temporary.replace(path)

    def pending(self, runs: list[DriveRun]) -> list[DriveRun]:
        """Runs not fetched yet (or whose checkpoint has since been removed)."""
        def fetched(run: DriveRun) -> bool:
            model_id = self.manifests[run.family].get(run.name, {}).get("model_id", "")
            return bool(model_id) and self.registries[run.family].checkpoint(model_id).is_file()
        return [run for run in runs if not fetched(run)]

    def fetch(self, run: DriveRun) -> tuple[str, str]:
        """Download one run. Returns (model_id, 'added' | 'already here' | 'not complete')."""
        registry = self.registries[run.family]
        models = registry.models
        models.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=models, prefix=".download-") as scratch:
            done_path = Path(scratch) / "done.json"
            self.download(run.done_id, done_path)
            done = json.loads(done_path.read_text(encoding="utf-8"))
            if done.get("state") != "complete" or done.get("model", run.family) != run.family:
                print(f"{run.name}: skipped, done.json says state={done.get('state')} "
                      f"model={done.get('model')}", file=self.log, flush=True)
                return "", "not complete"
            partial = Path(scratch) / f"checkpoint{registry.suffix}"
            self.download(run.checkpoint_id, partial)
            digest = sha256(partial)
            existing = next((path.stem for path in sorted(models.glob(f"*{registry.suffix}"))
                             if _cached_sha256(path) == digest), None)
            if existing:
                model_id, outcome = existing, "already here"
            else:
                model_id = run.model_id
                if registry.checkpoint(model_id).exists():  # same name and day, other run
                    model_id = model_id.replace("_best", f"_{run.time_suffix}_best")
                os.replace(partial, registry.checkpoint(model_id))
                outcome = "added"
        self.manifests[run.family][run.name] = {
            "model_id": model_id, "sha256": digest, "drive_file_id": run.checkpoint_id,
            "finished_at": done.get("finished_at"), "metrics": done.get("metrics", {}),
            "fetched_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
        self._save_manifest(run.family)
        print(f"{run.name}: {outcome} as {model_id} ({digest[:12]})", file=self.log, flush=True)
        return model_id, outcome

    def needs_export(self) -> list[str]:
        return [model.id for registry in self.registries.values()
                for model in registry.list(check_tensorrt=False)
                if model.has_checkpoint and not model.legacy and model.onnx_state != OK]

    def export(self, model_id: str) -> int:
        print(f"\n===== export {model_id} =====", file=self.log, flush=True)
        with contextlib.redirect_stdout(self.log), contextlib.redirect_stderr(self.log):
            return self.registry_for(model_id).export(model_id)

    def registry_for(self, model_id: str) -> Registry:
        return self.registries[family_of(model_id)]


class Progress:
    """Two-phase progress (download, export): tqdm bars in a terminal, or
    ``PROGRESS {json}`` lines for the GUI."""

    def __init__(self, gui: bool) -> None:
        self.gui = gui
        self.state = {"phase": "listing", "download": [0, 0], "export": [0, 0], "current": "",
                      "added": 0, "exported": 0, "failed": 0, "finished": False}
        self._bar = None
        self._emit()

    def _emit(self) -> None:
        if self.gui:
            print("PROGRESS " + json.dumps(self.state), flush=True)

    def note(self, text: str) -> None:
        if self._bar is not None:
            self._bar.write(text)
        else:
            print(text, flush=True)

    def begin(self, phase: str, total: int, unit_label: str) -> None:
        self.state.update(phase=phase, current="")
        self.state[phase] = [0, total]
        if not self.gui:
            from tqdm import tqdm
            self._bar = tqdm(total=total, desc=unit_label, unit="model", dynamic_ncols=True,
                             disable=total == 0)
        self._emit()

    def working_on(self, name: str) -> None:
        self.state["current"] = name
        if self._bar is not None:
            self._bar.set_postfix_str(name)
        else:
            print(f"[{self.state['phase']} {self.state[self.state['phase']][0] + 1}/"
                  f"{self.state[self.state['phase']][1]}] {name}", flush=True)
        self._emit()

    def step(self, **counts: int) -> None:
        for key, value in counts.items():
            self.state[key] += value
        self.state[self.state["phase"]][0] += 1
        if self._bar is not None:
            self._bar.update(1)
        self._emit()

    def end_phase(self) -> None:
        if self._bar is not None:
            self._bar.close()
            self._bar = None

    def finish(self) -> None:
        self.end_phase()
        self.state.update(phase="done", current="", finished=True)
        self._emit()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="fetch_yolo_models", description=__doc__.split("\n\n")[0])
    parser.add_argument("--project-root", type=Path,
                        default=Path(os.environ.get("DRACOVILOC_ROOT", Path(__file__).resolve().parents[2])))
    parser.add_argument("--folder", default=DRIVE_FOLDER, help="Google Drive folder id")
    parser.add_argument("--no-export", action="store_true", help="only download")
    parser.add_argument("--gui", action="store_true", help="PROGRESS lines instead of tqdm bars")
    args = parser.parse_args(argv)

    log_path = args.project_root / "logs/jobs" / f"{datetime.now():{RUN_NAME_FORMAT}}_fetch_yolo_models.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    failures: list[str] = []
    progress = Progress(args.gui)
    with open(log_path, "a", encoding="utf-8") as log:
        fetcher = Fetcher(args.project_root, log)
        print("Listing the Google Drive folder ...", flush=True)
        try:
            runs = list_drive_runs(args.folder)
        except Exception as error:  # gdown raises several kinds for network/permission problems
            print(f"Cannot list Drive folder {args.folder}: {error}", file=sys.stderr)
            progress.state["failed"] = 1
            progress.finish()
            return 1
        pending = fetcher.pending(runs)
        print(f"{len(runs)} training runs on Drive, {len(runs) - len(pending)} already fetched, "
              f"{len(pending)} to download", flush=True)

        added = []
        progress.begin("download", len(pending), "Download")
        for run in pending:
            progress.working_on(run.name)
            try:
                model_id, outcome = fetcher.fetch(run)
                if outcome == "added":
                    added.append(model_id)
                    progress.note(f"  + {model_id}  (from {run.name})")
                progress.step(added=int(outcome == "added"))
            except Exception as error:
                failures.append(f"download {run.name}: {error}")
                progress.note(f"  ! {run.name}: {error}")
                print(f"{run.name}: download failed: {error}", file=log, flush=True)
                progress.step(failed=1)
        progress.end_phase()

        exported = []
        to_export = [] if args.no_export else fetcher.needs_export()
        if to_export:
            print(f"{len(to_export)} model(s) need an ONNX export "
                  f"(full output in {log_path})", flush=True)
        progress.begin("export", len(to_export), "Export  ")
        for model_id in to_export:
            progress.working_on(model_id)
            code = fetcher.export(model_id)
            if code == 0 and fetcher.registry_for(model_id).onnx_status(model_id)[0] == OK:
                exported.append(model_id)
                progress.note(f"  ✓ {model_id}.onnx")
                progress.step(exported=1)
            else:
                failures.append(f"export {model_id}: exit code {code}")
                progress.note(f"  ! export {model_id} failed (exit code {code})")
                progress.step(failed=1)
        progress.finish()

    print(f"\nDone: {len(added)} model(s) added, {len(exported)} exported, {len(failures)} failed.")
    for failure in failures:
        print(f"  - {failure}")
    if exported:
        print("Build their TensorRT plan in Setup & builds > Visual models; they then "
              "appear in Run setup > Visual.")
    print(f"Log: {log_path}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
