from __future__ import annotations

import argparse


def add_visualization_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument("--dt", type=float, default=0.1, help="Sampling period in seconds.")
    parser.add_argument("--duration", type=float, default=10.0, help="Visualization duration in seconds.")
    parser.add_argument("--tail-seconds", type=float, default=2.0, help="History tail length in seconds.")
    parser.add_argument("--loop", action="store_true", help="Loop playback when it reaches the end.")
    parser.add_argument("--no-show", action="store_true", help="Build the figure without opening a GUI window.")
    parser.add_argument("--save-frame", help="Optional path to save a static frame.")
    parser.add_argument("--frame-time", type=float, default=4.5, help="Frame time for --save-frame.")
    return parser


def save_requested_frame(visualizer, args: argparse.Namespace) -> None:
    if args.save_frame:
        visualizer.save_frame(args.save_frame, t=args.frame_time)
        print(args.save_frame)
