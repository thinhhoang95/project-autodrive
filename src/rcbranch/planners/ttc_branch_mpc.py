from __future__ import annotations

from rcbranch.belief.features import PairFeatures


def should_branch_on_ttc(features: PairFeatures, threshold: float = 1.0) -> bool:
    conflict_gap = abs(features.t_i_in_cv - features.t_j_in_cv)
    return conflict_gap <= threshold
