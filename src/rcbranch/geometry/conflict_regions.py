from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Iterable

import numpy as np
from shapely.geometry import LineString, Point

from rcbranch.geometry.reference_path import ReferencePath


@dataclass(frozen=True, slots=True)
class ConflictRegion:
    i: int
    j: int
    s_i_in: float
    s_i_out: float
    s_j_in: float
    s_j_out: float
    polygon: object

    def entry_for(self, vehicle_id: int) -> float | None:
        if vehicle_id == self.i:
            return self.s_i_in
        if vehicle_id == self.j:
            return self.s_j_in
        return None

    def exit_for(self, vehicle_id: int) -> float | None:
        if vehicle_id == self.i:
            return self.s_i_out
        if vehicle_id == self.j:
            return self.s_j_out
        return None


def path_tube(ref_path: ReferencePath, half_width: float, margin: float = 0.0):
    if half_width < 0.0 or margin < 0.0:
        raise ValueError("half_width and margin must be nonnegative.")
    return LineString(ref_path.xy).buffer(half_width + margin, cap_style=2, join_style=2)


def _inside_s_interval(ref_path: ReferencePath, polygon: object, sample_step: float) -> tuple[float, float] | None:
    s_samples, xy_samples = ref_path.sample(step=sample_step)
    expanded = polygon.buffer(1e-9)
    inside = np.array([expanded.covers(Point(xy)) for xy in xy_samples])
    if not np.any(inside):
        return None
    idx = np.flatnonzero(inside)
    return float(s_samples[idx[0]]), float(s_samples[idx[-1]])


def compute_conflict_region(
    ref_i: ReferencePath,
    ref_j: ReferencePath,
    *,
    i: int,
    j: int,
    half_width_i: float = 1.0,
    half_width_j: float = 1.0,
    margin: float = 0.25,
    sample_step: float = 0.1,
) -> ConflictRegion | None:
    """Compute the path-coordinate overlap of two swept path tubes."""

    intersection = path_tube(ref_i, half_width_i, margin).intersection(
        path_tube(ref_j, half_width_j, margin)
    )
    if intersection.is_empty:
        return None

    interval_i = _inside_s_interval(ref_i, intersection, sample_step)
    interval_j = _inside_s_interval(ref_j, intersection, sample_step)
    if interval_i is None or interval_j is None:
        return None

    return ConflictRegion(
        i=i,
        j=j,
        s_i_in=interval_i[0],
        s_i_out=interval_i[1],
        s_j_in=interval_j[0],
        s_j_out=interval_j[1],
        polygon=intersection,
    )


def build_conflict_graph(
    vehicles: Iterable[object],
    *,
    margin: float = 0.25,
    sample_step: float = 0.1,
) -> list[ConflictRegion]:
    """Build pairwise route conflicts for vehicle-like objects with paths and widths."""

    vehicle_list = list(vehicles)
    conflicts: list[ConflictRegion] = []
    for veh_i, veh_j in combinations(vehicle_list, 2):
        width_i = float(getattr(veh_i, "width", 2.0))
        width_j = float(getattr(veh_j, "width", 2.0))
        conflict = compute_conflict_region(
            getattr(veh_i, "ref_path"),
            getattr(veh_j, "ref_path"),
            i=int(getattr(veh_i, "obstacle_id")),
            j=int(getattr(veh_j, "obstacle_id")),
            half_width_i=0.5 * width_i,
            half_width_j=0.5 * width_j,
            margin=margin,
            sample_step=sample_step,
        )
        if conflict is not None:
            conflicts.append(conflict)
    return conflicts
