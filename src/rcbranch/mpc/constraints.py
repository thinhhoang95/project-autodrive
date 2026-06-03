from __future__ import annotations

import numpy as np


def longitudinal_step(s: float, v: float, a: float, dt: float) -> tuple[float, float]:
    return s + dt * v + 0.5 * dt * dt * a, v + dt * a


def comfortable_stop_margin(s: float, v: float, s_in: float, b_comf: float, d_buf: float) -> float:
    if b_comf <= 0.0:
        raise ValueError("b_comf must be positive.")
    return float(v * v / (2.0 * b_comf) + d_buf - (s_in - s))


def constant_velocity_rollout(s0: float, v0: float, dt: float, horizon_steps: int) -> np.ndarray:
    k = np.arange(horizon_steps + 1, dtype=float)
    return s0 + dt * k * v0


def pre_entry_indices(s_ref: np.ndarray, s_in: float, eps: float = 0.25) -> list[int]:
    return [int(k) for k, s in enumerate(s_ref) if s < s_in - eps]
