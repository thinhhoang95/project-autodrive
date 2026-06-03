import numpy as np
import pytest

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.mpc.branching import choose_branch_time, compute_branch_scores


def _conflict():
    return ConflictRegion(1, 2, 10.0, 12.0, 8.0, 10.0, polygon=None)


def test_branch_score_requires_ambiguity_and_dual_price():
    duals = {
        (1, 2, 0): 0.5,
        (2, 1, 0): 0.5,
        (1, 2, 1): 1.0,
        (2, 1, 1): 1.0,
    }
    beliefs = {(1, 2): PairBelief(0.5, 0.5, 0.0)}

    scores = compute_branch_scores(duals, beliefs, [_conflict()], dt=0.2)
    score = scores[(1, 2)]

    assert score.ambiguity == pytest.approx(1.0)
    assert score.lambda_sum_by_k.tolist() == pytest.approx([1.0, 2.0])
    assert score.psi == pytest.approx(0.6)
    assert choose_branch_time(score, tau_psi=1.5) == 1


def test_branch_score_low_when_order_is_clear():
    duals = {(1, 2, 0): 10.0, (2, 1, 0): 10.0}
    beliefs = {(1, 2): PairBelief(0.95, 0.03, 0.02)}

    score = compute_branch_scores(duals, beliefs, [_conflict()], dt=0.2)[(1, 2)]

    assert score.ambiguity == pytest.approx(0.08)
    assert np.isclose(score.psi, 0.32)
