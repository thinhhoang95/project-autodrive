from __future__ import annotations

from typing import Iterable

import numpy as np

from rcbranch.geometry.reference_path import ReferencePath


def reference_path_from_polyline(polyline: Iterable[Iterable[float]]) -> ReferencePath:
    return ReferencePath.from_xy(np.asarray(polyline, dtype=float))


def reference_path_from_lanelet_sequence(lanelets: Iterable[object]) -> ReferencePath:
    points: list[np.ndarray] = []
    for lanelet in lanelets:
        center_vertices = np.asarray(getattr(lanelet, "center_vertices"), dtype=float)
        if center_vertices.ndim != 2 or center_vertices.shape[1] != 2:
            raise ValueError("Lanelet center_vertices must have shape (M, 2).")
        if points and np.allclose(points[-1], center_vertices[0]):
            points.extend(center_vertices[1:])
        else:
            points.extend(center_vertices)
    if len(points) < 2:
        raise ValueError("Lanelet sequence did not produce a usable path.")
    return ReferencePath.from_xy(np.vstack(points))


def lanelets_by_id(scenario: object, lanelet_ids: Iterable[int]) -> list[object]:
    network = scenario.lanelet_network
    lanelets = []
    for lanelet_id in lanelet_ids:
        lanelets.append(network.find_lanelet_by_id(int(lanelet_id)))
    return lanelets
