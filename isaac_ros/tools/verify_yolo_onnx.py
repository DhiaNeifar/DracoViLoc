#!/usr/bin/env python3
"""Verify that a YOLO ONNX model has the expected raw tensor contract."""

from argparse import ArgumentParser
from pathlib import Path

import onnx


def tensor_shape(value: onnx.ValueInfoProto) -> list[int | str]:
    return [
        dimension.dim_value if dimension.HasField("dim_value") else dimension.dim_param
        for dimension in value.type.tensor_type.shape.dim
    ]


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("--model", type=Path, required=True, help="Path to the ONNX model")
    parser.add_argument(
        "--num-classes",
        type=int,
        required=True,
        help="Number of detection classes",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Square input image size")
    args = parser.parse_args()

    if args.num_classes < 1:
        parser.error("--num-classes must be at least 1")
    if args.imgsz < 32 or args.imgsz % 32 != 0:
        parser.error("--imgsz must be at least 32 and divisible by 32")

    model_path = args.model.resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {model_path}")

    model = onnx.load(str(model_path))
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise RuntimeError(
            f"Expected one input and one output, found "
            f"{len(model.graph.input)} input(s) and {len(model.graph.output)} output(s)"
        )

    input_value = model.graph.input[0]
    output_value = model.graph.output[0]
    actual_input = (input_value.name, tensor_shape(input_value))
    actual_output = (output_value.name, tensor_shape(output_value))
    predictions = sum((args.imgsz // stride) ** 2 for stride in (8, 16, 32))
    expected_input = ("images", [1, 3, args.imgsz, args.imgsz])
    expected_output = ("output0", [1, 4 + args.num_classes, predictions])

    print(f"Input:  {actual_input[0]} {actual_input[1]}")
    print(f"Output: {actual_output[0]} {actual_output[1]}")

    if actual_input != expected_input:
        raise RuntimeError(f"Expected input {expected_input}, got {actual_input}")
    if actual_output != expected_output:
        raise RuntimeError(f"Expected output {expected_output}, got {actual_output}")

    onnx.checker.check_model(model)
    print("Verification passed: the model has the expected raw YOLO tensor contract.")


if __name__ == "__main__":
    main()
