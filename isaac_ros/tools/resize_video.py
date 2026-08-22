#!/usr/bin/env python3
"""Resize a video to reduce decoded frame size for ROS transport."""

from argparse import ArgumentParser
from pathlib import Path

import cv2


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--fps', type=float, default=None)
    args = parser.parse_args()

    input_path = args.input.resolve()
    output_path = args.output.resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f'Input video not found: {input_path}')
    if args.width < 1 or args.height < 1:
        parser.error('--width and --height must be positive')

    capture = cv2.VideoCapture(str(input_path))
    if not capture.isOpened():
        raise RuntimeError(f'Could not open input video: {input_path}')

    source_fps = capture.get(cv2.CAP_PROP_FPS)
    output_fps = args.fps or (source_fps if source_fps > 0 else 30.0)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*'mp4v'),
        output_fps,
        (args.width, args.height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f'Could not open output video: {output_path}')

    frame_count = 0
    try:
        while True:
            success, frame = capture.read()
            if not success:
                break
            resized = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_AREA)
            writer.write(resized)
            frame_count += 1
            if frame_count % 300 == 0:
                print(f'Converted {frame_count} frames')
    finally:
        capture.release()
        writer.release()

    print(f'Wrote {frame_count} frames to {output_path}')
    print(f'Output: {args.width}x{args.height} at {output_fps:.2f} FPS')


if __name__ == '__main__':
    main()
