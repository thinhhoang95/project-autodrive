from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.geometry.reference_path import ReferencePath


@dataclass(slots=True)
class VehicleOnPath:
    obstacle_id: int
    is_ego: bool
    route_id: str
    incoming_id: str
    ref_path: ReferencePath
    s0: float
    v0: float
    a_obs: float
    length: float
    width: float
    desired_speed: float
    max_accel: float
    max_brake: float
    comfort_brake: float


def min_conflict_distance_ahead(vehicle: VehicleOnPath, conflicts: Iterable[ConflictRegion]) -> float | None:
    distances: list[float] = []
    for conflict in conflicts:
        entry = conflict.entry_for(vehicle.obstacle_id)
        if entry is None:
            continue
        distance = entry - vehicle.s0
        if distance >= 0.0:
            distances.append(distance)
    if not distances:
        return None
    return min(distances)


def select_front_vehicles_by_incoming(
    vehicles: Iterable[VehicleOnPath],
    conflicts: Iterable[ConflictRegion],
) -> list[VehicleOnPath]:
    """Return the closest pre-conflict vehicle for each incoming approach."""

    conflict_list = list(conflicts)
    groups: dict[str, tuple[VehicleOnPath, float]] = {}
    for vehicle in vehicles:
        distance = min_conflict_distance_ahead(vehicle, conflict_list)
        if distance is None:
            continue
        key = vehicle.incoming_id
        if key not in groups or distance < groups[key][1]:
            groups[key] = (vehicle, distance)
    return [vehicle for vehicle, _ in groups.values()]
