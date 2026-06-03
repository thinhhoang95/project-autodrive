from __future__ import annotations

from rcbranch.belief.crossing_order_filter import PairBelief


def most_likely_order(pair: tuple[int, int], belief: PairBelief) -> tuple[int, int]:
    i, j = pair
    return (i, j) if belief.p_i_first >= belief.p_j_first else (j, i)


def deterministic_orders(beliefs: dict[tuple[int, int], PairBelief]) -> list[tuple[int, int]]:
    return [most_likely_order(pair, belief) for pair, belief in beliefs.items()]
