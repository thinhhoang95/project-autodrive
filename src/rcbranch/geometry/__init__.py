"""Geometry and fixed-path abstractions."""

from rcbranch.geometry.active_set import VehicleOnPath, select_front_vehicles_by_incoming
from rcbranch.geometry.conflict_regions import ConflictRegion, build_conflict_graph
from rcbranch.geometry.reference_path import ReferencePath

__all__ = [
    "ConflictRegion",
    "ReferencePath",
    "VehicleOnPath",
    "build_conflict_graph",
    "select_front_vehicles_by_incoming",
]
