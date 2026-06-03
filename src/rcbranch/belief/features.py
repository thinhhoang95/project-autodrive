from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rcbranch.geometry.active_set import VehicleOnPath
from rcbranch.geometry.conflict_regions import ConflictRegion


@dataclass(frozen=True, slots=True)
class PairFeatures:
    t_i_in_cv: float
    t_i_out_cv: float
    t_j_in_cv: float
    t_j_out_cv: float
    b_i_req_stop: float
    b_j_req_stop: float
    a_i_obs: float
    a_j_obs: float
    gap_if_i_first: float
    gap_if_j_first: float


def time_to_s_constant_velocity(s0: float, v0: float, target_s: float, min_speed: float = 0.1) -> float:
    distance = target_s - s0
    if distance <= 0.0:
        return 0.0
    if v0 < min_speed:
        return float("inf")
    return float(distance / v0)


def required_stop_deceleration(s0: float, v0: float, stop_s: float, eps: float = 1e-6) -> float:
    distance = max(stop_s - s0, eps)
    return float(v0 * v0 / (2.0 * distance))


def compute_pair_features(
    vehicle_i: VehicleOnPath,
    vehicle_j: VehicleOnPath,
    conflict: ConflictRegion,
    *,
    safe_time: float = 0.7,
) -> PairFeatures:
    if conflict.i == vehicle_i.obstacle_id:
        s_i_in, s_i_out = conflict.s_i_in, conflict.s_i_out
        s_j_in, s_j_out = conflict.s_j_in, conflict.s_j_out
    elif conflict.j == vehicle_i.obstacle_id:
        s_i_in, s_i_out = conflict.s_j_in, conflict.s_j_out
        s_j_in, s_j_out = conflict.s_i_in, conflict.s_i_out
    else:
        raise ValueError("vehicle_i is not part of the conflict.")

    t_i_in = time_to_s_constant_velocity(vehicle_i.s0, vehicle_i.v0, s_i_in)
    t_i_out = time_to_s_constant_velocity(vehicle_i.s0, vehicle_i.v0, s_i_out)
    t_j_in = time_to_s_constant_velocity(vehicle_j.s0, vehicle_j.v0, s_j_in)
    t_j_out = time_to_s_constant_velocity(vehicle_j.s0, vehicle_j.v0, s_j_out)
    gap_if_i_first = t_j_in - t_i_out - safe_time
    gap_if_j_first = t_i_in - t_j_out - safe_time

    return PairFeatures(
        t_i_in_cv=t_i_in,
        t_i_out_cv=t_i_out,
        t_j_in_cv=t_j_in,
        t_j_out_cv=t_j_out,
        b_i_req_stop=required_stop_deceleration(vehicle_i.s0, vehicle_i.v0, s_i_in),
        b_j_req_stop=required_stop_deceleration(vehicle_j.s0, vehicle_j.v0, s_j_in),
        a_i_obs=float(np.nan_to_num(vehicle_i.a_obs)),
        a_j_obs=float(np.nan_to_num(vehicle_j.a_obs)),
        gap_if_i_first=gap_if_i_first,
        gap_if_j_first=gap_if_j_first,
    )
