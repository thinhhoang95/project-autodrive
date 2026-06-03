from __future__ import annotations

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.active_set import VehicleOnPath
from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.mpc.branch_mpc import BranchMpcSolution, solve_branch_mpc
from rcbranch.mpc.reciprocal_caution_mpc import MpcSolution


def solve_fixed_time_branch_mpc(
    active: list[VehicleOnPath],
    conflicts: list[ConflictRegion],
    beliefs: dict[tuple[int, int], PairBelief],
    *,
    branch_pair: tuple[int, int],
    branch_step: int,
    warm_start: MpcSolution,
) -> BranchMpcSolution:
    return solve_branch_mpc(
        active,
        conflicts,
        beliefs,
        branch_pair=branch_pair,
        kb=branch_step,
        warm_start=warm_start,
    )
