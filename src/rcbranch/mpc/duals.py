from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from rcbranch.mpc.nlp_builder import ConstraintMeta


def extract_caution_duals(
    lam_g: Iterable[float],
    metas: Iterable[ConstraintMeta],
    *,
    magnitude: bool = True,
) -> dict[tuple[int, int, int], float]:
    values: dict[tuple[int, int, int], float] = {}
    for lam, meta in zip(lam_g, metas, strict=True):
        if meta.kind != "reciprocal_caution" or meta.i is None or meta.j is None or meta.k is None:
            continue
        value = abs(float(lam)) if magnitude else float(lam)
        values[(meta.i, meta.j, meta.k)] = value
    return values


def normalize_lambda(raw_lam: float, grad_norm: float = 1.0, j_scale: float = 1.0, eps: float = 1e-6) -> float:
    return float(raw_lam * grad_norm / (j_scale + eps))


def ema(prev: float | None, current: float, alpha: float = 0.7) -> float:
    if prev is None:
        return float(current)
    return float(alpha * prev + (1.0 - alpha) * current)


def smooth_dual_series(
    previous: dict[tuple[int, int, int], float] | None,
    current: dict[tuple[int, int, int], float],
    alpha: float = 0.7,
) -> dict[tuple[int, int, int], float]:
    previous = previous or {}
    return {key: ema(previous.get(key), value, alpha=alpha) for key, value in current.items()}
