from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from rcbranch.geometry.reference_path import ReferencePath


@dataclass(frozen=True, slots=True)
class ProjectedState:
    s: float
    v: float
    a: float = 0.0
    path_heading: float = 0.0


def project_xy_to_path(ref_path: ReferencePath, xy: np.ndarray) -> tuple[float, np.ndarray]:
    s = ref_path.project_xy_to_s(xy)
    x, y, _, _ = ref_path.interpolate_xytheta(s)
    return s, np.array([x, y], dtype=float)


def project_commonroad_state(state: Any, ref_path: ReferencePath) -> ProjectedState:
    position = np.asarray(getattr(state, "position"), dtype=float)
    s = ref_path.project_xy_to_s(position)
    _, _, theta, _ = ref_path.interpolate_xytheta(s)
    v = float(getattr(state, "velocity", 0.0))
    a = float(getattr(state, "acceleration", 0.0))
    return ProjectedState(s=s, v=v, a=a, path_heading=float(theta))
