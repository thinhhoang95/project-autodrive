from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class TrajectoryMetrics:
    time_to_goal: float | None
    trajectory_length: float
    comfort_cost: float
    jerk_cost: float
    unnecessary_stops: int


def compute_longitudinal_metrics(
    s: np.ndarray,
    v: np.ndarray,
    a: np.ndarray,
    *,
    dt: float,
    goal_s: float | None = None,
    stop_speed: float = 0.2,
) -> TrajectoryMetrics:
    s = np.asarray(s, dtype=float)
    v = np.asarray(v, dtype=float)
    a = np.asarray(a, dtype=float)
    if len(s) < 2:
        raise ValueError("Need at least two position samples.")
    reached_idx = None
    if goal_s is not None:
        reached = np.flatnonzero(s >= goal_s)
        reached_idx = int(reached[0]) if len(reached) else None
    jerk = np.diff(a) / dt if len(a) > 1 else np.zeros(0)
    return TrajectoryMetrics(
        time_to_goal=None if reached_idx is None else reached_idx * dt,
        trajectory_length=float(max(s[-1] - s[0], 0.0)),
        comfort_cost=float(np.sum(a * a) * dt),
        jerk_cost=float(np.sum(jerk * jerk) * dt),
        unnecessary_stops=int(np.sum(v < stop_speed)),
    )


def conflict_time_margin(t_i_out: float, t_j_in: float) -> float:
    return float(t_j_in - t_i_out)
