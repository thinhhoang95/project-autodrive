from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="List benchmark scenarios configured for rcbranch.")
    parser.add_argument(
        "--scenario-config",
        default="configs/scenarios_uncontrolled_intersections.yaml",
        help="YAML file containing a scenarios list.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    with Path(args.scenario_config).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    scenarios = data.get("scenarios", [])
    print(f"configured_scenarios={len(scenarios)}")
    for item in scenarios:
        print(item)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
