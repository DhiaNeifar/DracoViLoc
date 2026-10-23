#!/usr/bin/env python3
"""Build the fixed-shape FP32 MobileNetV2 TensorRT engine from ONNX."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workspace-mib", type=int, default=512)
    args = parser.parse_args()
    if not args.onnx.is_file():
        parser.error(f"ONNX model does not exist: {args.onnx}")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    if args.workspace_mib <= 0:
        parser.error("--workspace-mib must be positive")

    import tensorrt as trt

    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    onnx_parser = trt.OnnxParser(network, logger)
    if not onnx_parser.parse(args.onnx.read_bytes()):
        errors = "\n".join(str(onnx_parser.get_error(index))
                           for index in range(onnx_parser.num_errors))
        raise RuntimeError(f"ONNX parsing failed:\n{errors}")
    if network.num_inputs != 1 or network.get_input(0).name != "waveform":
        raise RuntimeError("expected one input named waveform")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, args.workspace_mib * 1024 * 1024)
    profile = builder.create_optimization_profile()
    shape = (1, 16000)
    # TensorRT 10.3's Python binding returns None from set_shape even when it
    # succeeds, so verify the stored profile instead of testing the return.
    profile.set_shape("waveform", shape, shape, shape)
    if tuple(profile.get_shape("waveform")) != (shape, shape, shape):
        raise RuntimeError("could not set waveform optimization profile")
    config.add_optimization_profile(profile)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT engine build failed")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(args.output.name + f".tmp-{os.getpid()}")
    try:
        temporary.write_bytes(serialized)
        temporary.replace(args.output)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Built TensorRT {trt.__version__} FP32 engine: {args.output}")


if __name__ == "__main__":
    main()
