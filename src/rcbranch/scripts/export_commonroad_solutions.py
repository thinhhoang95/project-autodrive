from __future__ import annotations

import argparse


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Placeholder for CommonRoad solution export.")
    parser.add_argument("--trajectory", required=False, help="Path to a serialized trajectory artifact.")
    parser.parse_args(argv)
    print("Use rcbranch.commonroad_adapter.solution_writer.make_commonroad_trajectory for export wiring.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
