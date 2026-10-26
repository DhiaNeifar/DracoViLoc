import json

import pytest

from dracoviloc_gui import yolo_models
from dracoviloc_gui.yolo_models import Registry, gpu_tag


@pytest.fixture
def registry(tmp_path, monkeypatch):
    monkeypatch.setattr(yolo_models, "CACHE", tmp_path / "cache")
    (tmp_path / "repo/models/yolo").mkdir(parents=True)
    (tmp_path / "ws/models").mkdir(parents=True)
    return Registry(tmp_path / "repo", workspace=tmp_path / "ws",
                    gpu="Orin (nvgpu)", image="test-image")


def write(path, content):
    path.write_bytes(content if isinstance(content, bytes) else content.encode())
    return path


def export(registry, model_id, checkpoint=b"weights", onnx=b"graph"):
    pt = write(registry.models / f"{model_id}.pt", checkpoint)
    write(registry.models / f"{model_id}.onnx", onnx)
    write(registry.models / f"{model_id}.onnx.json", json.dumps({
        "source_sha256": yolo_models.sha256(pt), "exported_at": "2026-10-26T00:00:00",
        "outputs": {"output0": [1, 5, 8400]}}))


def build_plan(registry, model_id, tensorrt="10.3"):
    plan = write(registry.plan_path(model_id), b"plan")
    write(plan.with_name(plan.name + ".json"), json.dumps({
        "onnx_sha256": yolo_models.sha256(registry.models / f"{model_id}.onnx"),
        "tensorrt": tensorrt, "throughput_qps": 131.0}))


def test_gpu_tags_are_short_and_file_safe():
    assert gpu_tag("Orin (nvgpu)") == "orin"
    assert gpu_tag("Quadro T2000 with Max-Q Design") == "quadro-t2000-with-max-q-design"
    assert gpu_tag("NVIDIA GeForce RTX 5050") == "rtx-5050"


def test_model_lifecycle_and_auto_selection(registry):
    write(registry.models / "drone_yolo11n_20260917_best.pt", b"new")
    export(registry, "drone_yolo11n_20260825_best")
    build_plan(registry, "drone_yolo11n_20260825_best")
    models = {m.id: m for m in registry.list(check_tensorrt=False)}
    assert models["drone_yolo11n_20260917_best"].onnx_state == "missing"
    assert models["drone_yolo11n_20260825_best"].ready
    # The newest *ready* model wins; an unexported newer checkpoint does not.
    assert registry.resolve("auto", list(models.values())).id == "drone_yolo11n_20260825_best"
    assert registry.container_paths("drone_yolo11n_20260825_best")[1].endswith(
        "drone_yolo11n_20260825_best.orin.plan")


def test_changed_checkpoint_or_onnx_marks_things_stale(registry):
    export(registry, "m_20260101")
    build_plan(registry, "m_20260101")
    write(registry.models / "m_20260101.onnx", b"re-exported graph")
    model = registry.list(check_tensorrt=False)[0]
    assert model.onnx_state == "ok" and model.plan_state == "stale"
    write(registry.models / "m_20260101.pt", b"retrained")
    assert registry.list(check_tensorrt=False)[0].onnx_state == "stale"


def test_tensorrt_upgrade_marks_plan_stale(registry):
    export(registry, "m_20260101")
    build_plan(registry, "m_20260101", tensorrt="10.3")
    state, detail = registry.plan_status("m_20260101", "ok", trt_version="10.7")
    assert state == "stale" and "10.7" in detail


def test_legacy_models_are_selectable_but_never_automatic(registry):
    write(registry.staged / "drone_yolo11n_best.onnx", b"old")
    write(registry.staged / "drone_yolo11n_best.plan", b"old plan")
    models = registry.list(check_tensorrt=False)
    assert models[0].legacy and models[0].ready
    with pytest.raises(LookupError):
        registry.resolve("auto", models)
    assert registry.resolve("drone_yolo11n_best", models).legacy
    assert registry.container_paths("drone_yolo11n_best", legacy=True)[1].endswith("drone_yolo11n_best.plan")


def test_no_gpu_means_plans_unavailable(tmp_path):
    (tmp_path / "models/yolo").mkdir(parents=True)
    registry = Registry(tmp_path, workspace=tmp_path / "ws", gpu="", image="x")
    export(registry, "m_20260101")
    assert registry.list(check_tensorrt=False)[0].plan_state == "unavailable"


def test_dfine_family_and_detectors(tmp_path, monkeypatch):
    monkeypatch.setattr(yolo_models, "CACHE", tmp_path / "cache")
    root, workspace = tmp_path / "repo", tmp_path / "ws"
    (root / "models/dfine").mkdir(parents=True)
    (workspace / "models").mkdir(parents=True)
    dfine = Registry(root, workspace=workspace, gpu="Orin (nvgpu)", image="x", family="dfine")
    for model_id in ("full_dfine_s_20261024_best", "full_dfine_n_20261025_best"):
        pth = write(dfine.checkpoint(model_id), model_id.encode())
        write(dfine.models / f"{model_id}.onnx", b"graph-" + model_id.encode())
        write(dfine.models / f"{model_id}.onnx.json", json.dumps({
            "source_sha256": yolo_models.sha256(pth), "outputs": {}}))
        build_plan(dfine, model_id)
        write(workspace / "models" / f"{model_id}.onnx", b"staged")  # not a legacy YOLO model
    assert yolo_models.detector_of("full_dfine_s_20261024_best") == "dfine"
    assert yolo_models.detector_of("full_dfine_n_20261025_best") == "dfine_n"
    assert yolo_models.detector_of("drone_yolo11n_20260917_best") == "yolo"
    models = dfine.list(check_tensorrt=False)
    assert [model.id for model in models] == ["full_dfine_n_20261025_best", "full_dfine_s_20261024_best"]
    assert all(model.ready and model.family == "dfine" for model in models)
    assert dfine.resolve("auto", models, detector="dfine").id == "full_dfine_s_20261024_best"
    assert dfine.resolve("auto", models, detector="dfine_n").id == "full_dfine_n_20261025_best"
    yolo = Registry(root, workspace=workspace, gpu="Orin (nvgpu)", image="x")
    assert yolo.list(check_tensorrt=False) == []
    with pytest.raises(LookupError, match="No YOLO model"):
        yolo.resolve("auto", [])
