#!/usr/bin/env python3
"""Export a local Hugging Face AST classifier to a static ONNX model."""

from argparse import ArgumentParser
from pathlib import Path

import torch
from transformers import ASTForAudioClassification


class LogitsOnly(torch.nn.Module):
    """Expose one TensorRT-friendly output instead of a Hugging Face object."""

    def __init__(self, model: ASTForAudioClassification) -> None:
        super().__init__()
        self.model = model

    def forward(self, input_values: torch.Tensor) -> torch.Tensor:
        return self.model(input_values=input_values).logits


def main() -> None:
    parser = ArgumentParser(
        description="Export model.safetensors AST weights to static ONNX")
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    model_dir = args.model_dir.resolve()
    output = args.output.resolve()
    for name in ("config.json", "model.safetensors"):
        if not (model_dir / name).is_file():
            raise FileNotFoundError(f"Missing {model_dir / name}")

    model = ASTForAudioClassification.from_pretrained(
        model_dir, local_files_only=True).eval().cpu()
    time_bins = int(model.config.max_length)
    mel_bins = int(model.config.num_mel_bins)
    example = torch.zeros((1, time_bins, mel_bins), dtype=torch.float32)

    output.parent.mkdir(parents=True, exist_ok=True)
    with torch.inference_mode():
        torch.onnx.export(
            LogitsOnly(model),
            example,
            str(output),
            input_names=["input_values"],
            output_names=["logits"],
            opset_version=args.opset,
            do_constant_folding=True,
            dynamic_axes=None,
            dynamo=False,
        )

    print(f"Exported: {output}")
    print(f"Input:  input_values float32 [1, {time_bins}, {mel_bins}]")
    print(f"Output: logits float32 [1, {model.config.num_labels}]")


if __name__ == "__main__":
    main()
