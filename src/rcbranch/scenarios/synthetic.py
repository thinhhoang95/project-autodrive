from __future__ import annotations

import numpy as np

from rcbranch.geometry import ReferencePath, VehicleOnPath


def west_to_north_turn_path(
    *,
    approach_length: float = 24.0,
    exit_length: float = 24.0,
    radius: float = 8.0,
    arc_points: int = 25,
) -> ReferencePath:
    """Return a smooth-enough 90-degree left-turn path from west to north."""

    if approach_length <= 0.0 or exit_length <= 0.0:
        raise ValueError("approach_length and exit_length must be positive.")
    if radius <= 0.0:
        raise ValueError("radius must be positive.")
    if arc_points < 3:
        raise ValueError("arc_points must be at least 3.")

    center = np.array([-radius, radius], dtype=float)
    approach = np.array(
        [
            [-radius - approach_length, 0.0],
            [-radius, 0.0],
        ],
        dtype=float,
    )
    angles = np.linspace(-0.5 * np.pi, 0.0, arc_points)
    arc = center + radius * np.column_stack([np.cos(angles), np.sin(angles)])
    exit_path = np.array(
        [
            [0.0, radius],
            [0.0, radius + exit_length],
        ],
        dtype=float,
    )
    return ReferencePath.from_xy(np.vstack([approach, arc[1:-1], exit_path]))


def build_turning_crossroad_scene() -> list[VehicleOnPath]:
    """Build a two-vehicle crossroad scene with a turning ego route."""

    ego_path = west_to_north_turn_path()
    crossing_path = ReferencePath.from_xy([[-4.0, -22.0], [-4.0, 22.0]])

    ego = VehicleOnPath(
        obstacle_id=1,
        is_ego=True,
        route_id="west_to_north_turn",
        incoming_id="west",
        ref_path=ego_path,
        s0=2.0,
        v0=5.0,
        a_obs=0.0,
        length=4.5,
        width=2.0,
        desired_speed=6.0,
        max_accel=2.0,
        max_brake=6.0,
        comfort_brake=3.0,
    )
    crossing = VehicleOnPath(
        obstacle_id=2,
        is_ego=False,
        route_id="south_to_north",
        incoming_id="south",
        ref_path=crossing_path,
        s0=8.0,
        v0=5.0,
        a_obs=0.0,
        length=4.5,
        width=2.0,
        desired_speed=6.0,
        max_accel=2.0,
        max_brake=6.0,
        comfort_brake=3.0,
    )
    return [ego, crossing]
