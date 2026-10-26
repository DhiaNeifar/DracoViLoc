import io
import json
import sys
from types import SimpleNamespace

from dracoviloc_gui.fetch_models import DriveRun, Fetcher, list_drive_runs, runs_from_listing


def make_fetcher(tmp_path, files):
    """Fetcher whose 'Drive' is the dict files: file id -> bytes."""
    (tmp_path / "models/yolo").mkdir(parents=True, exist_ok=True)
    downloads = []

    def download(file_id, target):
        downloads.append(file_id)
        target.write_bytes(files[file_id])

    fetcher = Fetcher(tmp_path, io.StringIO(), download,
                      workspace=tmp_path / "isaac", gpu="", image="none")
    return fetcher, downloads


def done(state="complete", model="yolo"):
    return json.dumps({"state": state, "model": model, "finished_at": "2026-10-24T15:47:07",
                       "metrics": {"val_map50": 0.97}}).encode()


def test_model_id_from_run_name():
    run = DriveRun("2026_10_24_03_31_54_yolo11n_par", "c", "d")
    assert run.model_id == "par_yolo11n_20261024_best"
    assert DriveRun("2026_09_17_19_13_36_yolo11n_drone", "c", "d").model_id == "drone_yolo11n_20260917_best"


def test_fetch_adds_then_skips(tmp_path):
    fetcher, downloads = make_fetcher(tmp_path, {"d1": done(), "c1": b"weights-1"})
    run = DriveRun("2026_10_24_03_31_54_yolo11n_par", "c1", "d1")
    assert fetcher.pending([run]) == [run]
    assert fetcher.fetch(run) == ("par_yolo11n_20261024_best", "added")
    assert (tmp_path / "models/yolo/par_yolo11n_20261024_best.pt").read_bytes() == b"weights-1"
    manifest = json.loads((tmp_path / "models/yolo/drive_models.json").read_text())
    assert manifest[run.name]["metrics"] == {"val_map50": 0.97}
    # A new Fetcher reads the manifest: nothing left to download.
    fetcher2, _ = make_fetcher(tmp_path, {})
    assert fetcher2.pending([run]) == []
    assert not list((tmp_path / "models/yolo").glob(".download-*"))


def test_identical_checkpoint_is_linked_not_duplicated(tmp_path):
    (tmp_path / "models/yolo").mkdir(parents=True)
    (tmp_path / "models/yolo/drone_yolo11n_deployed_20260803.pt").write_bytes(b"same")
    fetcher, _ = make_fetcher(tmp_path, {"d": done(), "c": b"same"})
    run = DriveRun("2026_08_03_23_04_37_yolo11n_exp001", "c", "d")
    assert fetcher.fetch(run) == ("drone_yolo11n_deployed_20260803", "already here")
    assert sorted(p.name for p in (tmp_path / "models/yolo").glob("*.pt")) == [
        "drone_yolo11n_deployed_20260803.pt"]


def test_same_day_name_clash_gets_time_suffix(tmp_path):
    (tmp_path / "models/yolo").mkdir(parents=True)
    (tmp_path / "models/yolo/par_yolo11n_20261024_best.pt").write_bytes(b"older run")
    fetcher, _ = make_fetcher(tmp_path, {"d": done(), "c": b"newer run"})
    run = DriveRun("2026_10_24_18_00_00_yolo11n_par", "c", "d")
    assert fetcher.fetch(run) == ("par_yolo11n_20261024_180000_best", "added")


def test_incomplete_run_is_not_downloaded(tmp_path):
    fetcher, downloads = make_fetcher(tmp_path, {"d": done(state="running")})
    run = DriveRun("2026_10_24_03_31_54_yolo11n_par", "c", "d")
    assert fetcher.fetch(run) == ("", "not complete")
    assert downloads == ["d"]
    assert fetcher.pending([run]) == [run]


def test_needs_export_lists_unexported_checkpoints(tmp_path):
    fetcher, _ = make_fetcher(tmp_path, {})
    models = tmp_path / "models/yolo"
    (models / "a_yolo11n_20261001_best.pt").write_bytes(b"a")
    (models / "b_yolo11n_20261002_best.pt").write_bytes(b"b")
    (models / "b_yolo11n_20261002_best.onnx").write_bytes(b"onnx")
    assert fetcher.needs_export() == ["a_yolo11n_20261001_best"]


def test_gui_progress_lines(capsys):
    from dracoviloc_gui.fetch_models import Progress
    progress = Progress(gui=True)
    progress.begin("download", 2, "Download")
    progress.working_on("run_a")
    progress.step(added=1)
    progress.finish()
    states = [json.loads(line[len("PROGRESS "):]) for line in capsys.readouterr().out.splitlines()
              if line.startswith("PROGRESS ")]
    assert states[0]["phase"] == "listing"
    assert states[-1] == {"phase": "done", "download": [1, 2], "export": [0, 0], "current": "",
                          "added": 1, "exported": 0, "failed": 0, "finished": True}


def test_drive_listing_picks_checkpoint_per_family():
    runs = runs_from_listing({
        "2026_10_24_03_31_54_yolo11n_par": {"weights/best.pt": "y", "weights/last.pt": "l", "done.json": "d1"},
        "2026_10_24_04_17_49_dfine_s_full": {"best_stg1.pth": "s1", "best_stg2.pth": "s2", "done.json": "d2"},
        "2026_10_25_00_45_17_dfine_n_full": {"best_stg1.pth": "n1", "done.json": "d3"},
        "2026_10_26_00_00_00_yolo11n_running": {"weights/last.pt": "x"},  # no done.json yet
        "notes": {"readme.txt": "r"},
    })
    assert [(run.family, run.checkpoint_id, run.model_id) for run in runs] == [
        ("yolo", "y", "par_yolo11n_20261024_best"),
        ("dfine", "s2", "full_dfine_s_20261024_best"),
        ("dfine", "n1", "full_dfine_n_20261025_best"),
    ]


def test_drive_listing_supports_gdown_without_remaining_ok(monkeypatch):
    calls = []

    def download_folder(id, skip_download, quiet):
        calls.append((id, skip_download, quiet))
        return []

    monkeypatch.setitem(sys.modules, "gdown", SimpleNamespace(download_folder=download_folder))
    assert list_drive_runs("folder-id") == []
    assert calls == [("folder-id", True, True)]


def test_dfine_run_goes_to_its_own_folder(tmp_path):
    fetcher, _ = make_fetcher(tmp_path, {"d": done(model="dfine"), "c": b"dfine weights"})
    run = DriveRun("2026_10_25_00_45_17_dfine_n_full", "c", "d", "dfine")
    assert fetcher.fetch(run) == ("full_dfine_n_20261025_best", "added")
    assert (tmp_path / "models/dfine/full_dfine_n_20261025_best.pth").read_bytes() == b"dfine weights"
    assert json.loads((tmp_path / "models/dfine/drive_models.json").read_text())[run.name]["model_id"] \
        == "full_dfine_n_20261025_best"
    assert fetcher.pending([run]) == []
    assert fetcher.needs_export() == ["full_dfine_n_20261025_best"]
