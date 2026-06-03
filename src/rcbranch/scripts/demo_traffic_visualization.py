from __future__ import annotations

import argparse

import numpy as np

from rcbranch.evaluation.traffic_visualization import (
    VehicleTrajectory,
    conflict_graph_from_trajectories,
    visualize_traffic_scene,
)
from rcbranch.geometry import ReferencePath


def build_demo_trajectories(dt: float = 0.1, duration: float = 10.0) -> list[VehicleTrajectory]:
    times = np.arange(0.0, duration + 0.5 * dt, dt)
    east_west = ReferencePath.from_xy([[-25.0, 0.0], [25.0, 0.0]])
    south_north = ReferencePath.from_xy([[0.0, -25.0], [0.0, 25.0]])

    ego_s = np.clip(5.0 + 4.2 * times, 0.0, east_west.length)
    yielding_s = np.clip(3.0 + 3.2 * times - 1.8 * np.exp(-0.5 * (times - 4.5) ** 2), 0.0, south_north.length)

    return [
        VehicleTrajectory(
            obstacle_id=1,
            ref_path=east_west,
            s=ego_s,
            times=times,
            v=np.gradient(ego_s, dt),
            label="ego",
            color="tab:blue",
        ),
        VehicleTrajectory(
            obstacle_id=2,
            ref_path=south_north,
            s=yielding_s,
            times=times,
            v=np.gradient(yielding_s, dt),
            label="yielding car",
            color="tab:orange",
        ),
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch a matplotlib traffic-intersection visualization demo.")
    parser.add_argument("--dt", type=float, default=0.1, help="Demo sampling period in seconds.")
    parser.add_argument("--duration", type=float, default=10.0, help="Demo duration in seconds.")
    parser.add_argument("--tail-seconds", type=float, default=2.0, help="History tail length in seconds.")
    parser.add_argument("--loop", action="store_true", help="Loop playback when it reaches the end.")
    parser.add_argument("--no-show", action="store_true", help="Build the figure without opening a GUI window.")
    parser.add_argument("--save-frame", help="Optional path to save a static frame.")
    parser.add_argument("--frame-time", type=float, default=4.5, help="Frame time for --save-frame.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    trajectories = build_demo_trajectories(dt=args.dt, duration=args.duration)
    conflicts = conflict_graph_from_trajectories(trajectories)
    visualizer = visualize_traffic_scene(
        trajectories,
        conflicts=conflicts,
        show=not args.no_show,
        title="rcbranch traffic visualization demo",
        tail_seconds=args.tail_seconds,
        loop=args.loop,
    )
    if args.save_frame:
        visualizer.save_frame(args.save_frame, t=args.frame_time)
        print(args.save_frame)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
