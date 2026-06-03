from __future__ import annotations

import argparse

import numpy as np

from rcbranch.evaluation.traffic_visualization import (
    VehicleTrajectory,
    conflict_graph_from_trajectories,
    visualize_traffic_scene,
)
from rcbranch.geometry import ReferencePath
from rcbranch.scripts.visualization_cli import add_visualization_arguments, save_requested_frame


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
    return add_visualization_arguments(parser)


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
    save_requested_frame(visualizer, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
