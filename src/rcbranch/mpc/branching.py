from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.conflict_regions import ConflictRegion


@dataclass(frozen=True, slots=True)
class BranchScore:
    pair: tuple[int, int]
    ambiguity: float
    psi: float
    psi_by_k: np.ndarray
    lambda_sum_by_k: np.ndarray


def _belief_for_pair(
    beliefs: Mapping[tuple[int, int], PairBelief],
    i: int,
    j: int,
) -> PairBelief:
    if (i, j) in beliefs:
        return beliefs[(i, j)]
    if (j, i) in beliefs:
        b = beliefs[(j, i)]
        return PairBelief(
            p_i_first=b.p_j_first,
            p_j_first=b.p_i_first,
            p_unresolved=b.p_unresolved,
            stable_count_i_first=b.stable_count_j_first,
            stable_count_j_first=b.stable_count_i_first,
        )
    return PairBelief(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)


def compute_branch_scores(
    duals: Mapping[tuple[int, int, int], float],
    beliefs: Mapping[tuple[int, int], PairBelief],
    conflicts: list[ConflictRegion],
    *,
    dt: float,
) -> dict[tuple[int, int], BranchScore]:
    scores: dict[tuple[int, int], BranchScore] = {}
    for conflict in conflicts:
        i, j = conflict.i, conflict.j
        ks = [
            key[2]
            for key in duals
            if (key[0] == i and key[1] == j) or (key[0] == j and key[1] == i)
        ]
        if not ks:
            continue
        max_k = max(ks)
        lambda_sum = np.zeros(max_k + 1, dtype=float)
        for k in range(max_k + 1):
            lambda_sum[k] = float(duals.get((i, j, k), 0.0) + duals.get((j, i, k), 0.0))
        belief = _belief_for_pair(beliefs, i, j)
        psi_by_k = belief.ambiguity * lambda_sum
        scores[(i, j)] = BranchScore(
            pair=(i, j),
            ambiguity=belief.ambiguity,
            psi=float(np.sum(psi_by_k) * dt),
            psi_by_k=psi_by_k,
            lambda_sum_by_k=lambda_sum,
        )
    return scores


def choose_branch_time(score: BranchScore, tau_psi: float, latest_k: int | None = None) -> int:
    for k, value in enumerate(score.psi_by_k):
        if value > tau_psi:
            return k
    if latest_k is not None:
        return int(min(max(latest_k, 0), len(score.psi_by_k) - 1))
    return int(np.argmax(score.psi_by_k))
