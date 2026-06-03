from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rcbranch.belief.features import PairFeatures


@dataclass(frozen=True, slots=True)
class PairBelief:
    p_i_first: float
    p_j_first: float
    p_unresolved: float
    stable_count_i_first: int = 0
    stable_count_j_first: int = 0

    @property
    def ambiguity(self) -> float:
        return float(1.0 - abs(self.p_i_first - self.p_j_first))

    def as_array(self) -> np.ndarray:
        return np.array([self.p_i_first, self.p_j_first, self.p_unresolved], dtype=float)

    def normalized(self) -> "PairBelief":
        probs = self.as_array()
        total = probs.sum()
        if total <= 0.0:
            probs = np.array([1.0 / 3.0] * 3)
        else:
            probs = probs / total
        return PairBelief(
            p_i_first=float(probs[0]),
            p_j_first=float(probs[1]),
            p_unresolved=float(probs[2]),
            stable_count_i_first=self.stable_count_i_first,
            stable_count_j_first=self.stable_count_j_first,
        )


@dataclass(frozen=True, slots=True)
class BeliefFilterConfig:
    softmax_temp: float = 1.0
    transition_persistence: float = 0.85
    p_high: float = 0.85
    w_time_overlap: float = 1.0
    w_accel: float = 0.4
    w_near: float = 0.05
    a_yield: float = -2.0
    a_go: float = 0.5
    a_cautious: float = -0.8
    safe_time: float = 0.7


def softmax_negative_cost(costs: list[float] | np.ndarray, temp: float = 1.0) -> np.ndarray:
    if temp <= 0.0:
        raise ValueError("temp must be positive.")
    arr = np.asarray(costs, dtype=float)
    arr = np.nan_to_num(arr, nan=1e6, posinf=1e6, neginf=-1e6)
    scaled = -arr / temp
    scaled = scaled - np.max(scaled)
    z = np.exp(scaled)
    return z / z.sum()


def _finite_square(value: float, cap: float = 1e3) -> float:
    return float(np.nan_to_num(value, nan=cap, posinf=cap, neginf=-cap) ** 2)


class CrossingOrderBeliefFilter:
    """Interpretable temporal filter over i-first, j-first, unresolved."""

    def __init__(self, config: BeliefFilterConfig | None = None):
        self.config = config or BeliefFilterConfig()
        self._beliefs: dict[tuple[int, int], PairBelief] = {}

    def get(self, i: int, j: int) -> PairBelief:
        return self._beliefs.get((i, j), PairBelief(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0))

    def update(self, i: int, j: int, features: PairFeatures) -> PairBelief:
        cfg = self.config
        previous = self.get(i, j)
        likelihood = softmax_negative_cost(self._prototype_costs(features), cfg.softmax_temp)
        predicted = cfg.transition_persistence * previous.as_array()
        predicted += (1.0 - cfg.transition_persistence) * np.array([1.0 / 3.0] * 3)
        posterior = likelihood * predicted
        posterior = posterior / posterior.sum()

        stable_i = previous.stable_count_i_first + 1 if posterior[0] >= cfg.p_high else 0
        stable_j = previous.stable_count_j_first + 1 if posterior[1] >= cfg.p_high else 0
        belief = PairBelief(
            p_i_first=float(posterior[0]),
            p_j_first=float(posterior[1]),
            p_unresolved=float(posterior[2]),
            stable_count_i_first=stable_i,
            stable_count_j_first=stable_j,
        )
        self._beliefs[(i, j)] = belief
        return belief

    def _prototype_costs(self, f: PairFeatures) -> list[float]:
        cfg = self.config
        cost_i_first = (
            cfg.w_time_overlap * _finite_square(max(0.0, f.t_i_out_cv + cfg.safe_time - f.t_j_in_cv))
            + cfg.w_accel * _finite_square(f.a_j_obs - cfg.a_yield)
            + cfg.w_accel * _finite_square(f.a_i_obs - cfg.a_go)
        )
        cost_j_first = (
            cfg.w_time_overlap * _finite_square(max(0.0, f.t_j_out_cv + cfg.safe_time - f.t_i_in_cv))
            + cfg.w_accel * _finite_square(f.a_i_obs - cfg.a_yield)
            + cfg.w_accel * _finite_square(f.a_j_obs - cfg.a_go)
        )
        arrival_gap = abs(f.t_i_in_cv - f.t_j_in_cv)
        near_penalty = cfg.w_near / max(arrival_gap, 1e-3)
        cost_unresolved = (
            cfg.w_accel * _finite_square(f.a_i_obs - cfg.a_cautious)
            + cfg.w_accel * _finite_square(f.a_j_obs - cfg.a_cautious)
            + near_penalty
        )
        return [cost_i_first, cost_j_first, cost_unresolved]
