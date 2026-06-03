from __future__ import annotations

import argparse

from rcbranch.evaluation.traffic_visualization import (
    constant_accel_trajectories_from_vehicles,
    visualize_traffic_scene,
)
from rcbranch.geometry import build_conflict_graph
from rcbranch.scenarios import build_turning_crossroad_scene
from rcbranch.scripts.visualization_cli import add_visualization_arguments, save_requested_frame


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize the synthetic 90-degree turning crossroad scenario.")
    add_visualization_arguments(parser)
    parser.add_argument("--conflict-sample-step", type=float, default=0.05, help="Conflict extraction sample step in meters.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    vehicles = build_turning_crossroad_scene()
    trajectories = constant_accel_trajectories_from_vehicles(vehicles, dt=args.dt, duration=args.duration)
    conflicts = build_conflict_graph(vehicles, sample_step=args.conflict_sample_step)
    visualizer = visualize_traffic_scene(
        trajectories,
        conflicts=conflicts,
        show=not args.no_show,
        title="90-degree turning crossroad scenario",
        tail_seconds=args.tail_seconds,
        loop=args.loop,
    )
    save_requested_frame(visualizer, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
