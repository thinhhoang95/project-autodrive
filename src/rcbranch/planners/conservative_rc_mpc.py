from __future__ import annotations

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.active_set import VehicleOnPath
from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.mpc.reciprocal_caution_mpc import MPCConfig, MPCWeights, MpcSolution, solve_reciprocal_caution_mpc


def solve_conservative_rc_mpc(
    active: list[VehicleOnPath],
    conflicts: list[ConflictRegion],
    beliefs: dict[tuple[int, int], PairBelief] | None = None,
    *,
    config: MPCConfig | None = None,
    weights: MPCWeights | None = None,
) -> MpcSolution:
    solution = solve_reciprocal_caution_mpc(active, conflicts, beliefs, config=config, weights=weights)
    solution.mode = "conservative_rc_mpc"
    return solution
