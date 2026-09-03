# DracoViLoc model artifacts

This directory is the authoritative location for every trained model used by
DracoViLoc. Runtime workspaces contain deployed copies only.

## Directory contract

### `ast/`

- `config.json`: architecture and class mapping.
- `model.safetensors`: trained weights.
- `preprocessor_config.json`: feature extractor configuration.
- `training_report.json`: training/evaluation record.
- `drone_ast.onnx`: static `[1,128,128] -> [1,2]` portable export.

- `drone_ast.engine`: generated TensorRT engine for the current Jetson.

`classification_fusion.launch.py` reads both the feature-extractor files and
the engine directly from this directory. No package-local copy is used.

### `gre/`

- `model_logmel.pt`: trained checkpoint.
- `model_logmel.onnx`: portable TensorRT input.
- `model_logmel_meta.json`: feature/model metadata.
- `model_logmel.engine`: engine for the current Jetson.

`classification_fusion.launch.py` reads the engine and metadata directly from
this directory. No package-local copy is used.

### `yolo/`

- `drone_yolo11n_20260825_best.pt`: current checkpoint.
- `drone_yolo11n_deployed_20260803.pt`: previous deployed checkpoint.
- `drone_yolo11n_deployed_20260803.onnx`: previous matching ONNX.

Copy the current checkpoint to the Isaac workspace, export a same-basename
`.onnx`, and generate a same-basename `.plan` there.

## Updating a model

1. Preserve the old model until the replacement passes standalone tests.
2. Copy the new weights/configuration here first.
3. Use consistent versioned basenames.
4. Export ONNX from these stored weights.
5. Verify shapes, bindings, and classes.
6. Build TensorRT on the target Jetson.
7. Keep the generated engine beside its source model in this directory.
8. Update launch paths and documentation.
9. Test standalone inference before arm motion or EKF.

Never reuse an engine after changing the model, ONNX, TensorRT, CUDA, JetPack,
input dimensions, or GPU. See the root [`README.md`](../README.md) for exact
environment, conversion, deployment, and launch commands.
