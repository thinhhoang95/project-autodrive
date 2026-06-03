from __future__ import annotations

from collections.abc import Iterable


def make_commonroad_trajectory(states: Iterable[object]):
    try:
        from commonroad.scenario.trajectory import Trajectory
    except ImportError as exc:
        raise ImportError("CommonRoad trajectory export requires commonroad-io.") from exc
    state_list = list(states)
    if not state_list:
        raise ValueError("Cannot export an empty trajectory.")
    initial_time_step = int(getattr(state_list[0], "time_step"))
    return Trajectory(initial_time_step, state_list)
