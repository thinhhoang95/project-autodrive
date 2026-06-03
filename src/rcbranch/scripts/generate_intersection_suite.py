from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate metadata for a synthetic uncontrolled-intersection suite.")
    parser.add_argument("--output", default="configs/synthetic_suite.yaml")
    args = parser.parse_args(argv)
    suite = {
        "scenarios": [
            {"name": "two_car_crossroad", "approaches": 2, "queue_length": 0},
            {"name": "two_car_turning_crossroad", "approaches": 2, "ego_route": "west_to_north_turn"},
            {"name": "four_approach_one_front", "approaches": 4, "queue_length": 0},
            {"name": "queue_followers", "approaches": 4, "queue_length": 2},
            {"name": "ambiguous_yield", "approaches": 2, "driver_type": "cautious"},
            {"name": "aggressive_other", "approaches": 2, "driver_type": "aggressive"},
        ]
    }
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(suite, f, sort_keys=False)
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
