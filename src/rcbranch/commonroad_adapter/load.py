from __future__ import annotations

from pathlib import Path
from typing import Any


def _require_commonroad_reader():
    try:
        from commonroad.common.file_reader import CommonRoadFileReader
    except ImportError as exc:
        raise ImportError(
            "CommonRoad I/O is required for scenario loading. Install with "
            "`uv sync --extra commonroad` or `pip install commonroad-io`."
        ) from exc
    return CommonRoadFileReader


def load_commonroad_problem(
    xml_path: str | Path,
    *,
    lanelet_assignment: bool = True,
    planning_problem_id: int | None = None,
) -> tuple[Any, Any]:
    CommonRoadFileReader = _require_commonroad_reader()
    scenario, planning_problem_set = CommonRoadFileReader(str(xml_path)).open(
        lanelet_assignment=lanelet_assignment
    )
    planning_problem_dict = planning_problem_set.planning_problem_dict
    if planning_problem_id is None:
        planning_problem = next(iter(planning_problem_dict.values()))
    else:
        planning_problem = planning_problem_dict[planning_problem_id]
    return scenario, planning_problem


def is_uncontrolled_intersection_scenario(scenario: Any) -> bool:
    lanelet_network = scenario.lanelet_network
    intersections = getattr(lanelet_network, "intersections", []) or []
    traffic_lights = getattr(lanelet_network, "traffic_lights", []) or []
    return len(intersections) > 0 and len(traffic_lights) == 0
