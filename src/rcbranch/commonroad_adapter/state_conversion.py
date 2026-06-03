from __future__ import annotations

from typing import Any

import numpy as np

from rcbranch.geometry.projection import ProjectedState, project_commonroad_state
from rcbranch.geometry.reference_path import ReferencePath


def _require_commonroad_state():
    try:
        from commonroad.scenario.state import State
    except ImportError as exc:
        raise ImportError(
            "CommonRoad state conversion requires commonroad-io. Install the `commonroad` extra."
        ) from exc
    return State


def commonroad_state_to_path_state(state: Any, ref_path: ReferencePath) -> ProjectedState:
    return project_commonroad_state(state, ref_path)


def path_state_to_commonroad_state(
    s: float,
    v: float,
    a: float,
    ref_path: ReferencePath,
    *,
    time_step: int,
):
    State = _require_commonroad_state()
    x, y, theta, _ = ref_path.interpolate_xytheta(s)
    return State(
        time_step=time_step,
        position=np.array([x, y], dtype=float),
        orientation=theta,
        velocity=v,
        acceleration=a,
    )
