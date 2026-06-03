from __future__ import annotations

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.mpc.branching import BranchScore, compute_branch_scores


def compute_raw_collision_dual_scores(
    raw_collision_duals: dict[tuple[int, int, int], float],
    beliefs: dict[tuple[int, int], PairBelief],
    conflicts: list[ConflictRegion],
    *,
    dt: float,
) -> dict[tuple[int, int], BranchScore]:
    return compute_branch_scores(raw_collision_duals, beliefs, conflicts, dt=dt)
