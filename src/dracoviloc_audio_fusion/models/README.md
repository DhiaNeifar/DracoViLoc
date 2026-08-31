# TensorRT engines

Place the machine-specific audio-classifier TensorRT engines in this
directory before running their respective classifiers:

- `drone_ast.engine` for AST
- `model_logmel.engine` for GRE

The engine files are intentionally excluded from Git because they depend on
the target Jetson's GPU, JetPack, CUDA, and TensorRT versions. Keeping this
directory in the repository allows `colcon build --symlink-install` to install
the package before either engine has been generated or copied into place.
