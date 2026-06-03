from __future__ import annotations

import argparse

from rcbranch.commonroad_adapter.load import is_uncontrolled_intersection_scenario, load_commonroad_problem


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect one CommonRoad scenario for rcbranch.")
    parser.add_argument("scenario_xml", help="Path to a CommonRoad XML scenario.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    scenario, planning_problem = load_commonroad_problem(args.scenario_xml)
    scenario_id = getattr(scenario, "scenario_id", "unknown")
    planning_problem_id = getattr(planning_problem, "planning_problem_id", "unknown")
    uncontrolled = is_uncontrolled_intersection_scenario(scenario)
    print(f"scenario_id={scenario_id}")
    print(f"planning_problem_id={planning_problem_id}")
    print(f"uncontrolled_intersection={uncontrolled}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
