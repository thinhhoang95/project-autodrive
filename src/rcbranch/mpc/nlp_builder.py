from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import casadi as ca
import numpy as np


@dataclass(frozen=True, slots=True)
class ConstraintMeta:
    kind: str
    i: int | None = None
    j: int | None = None
    k: int | None = None
    branch: int | None = None


class NLPBuilder:
    """Small CasADi NLP helper that preserves constraint metadata."""

    def __init__(self):
        self.w: list[Any] = []
        self.w0: list[float] = []
        self.lbw: list[float] = []
        self.ubw: list[float] = []
        self.var_slices: dict[str, slice] = {}
        self._nx = 0

        self.g: list[Any] = []
        self.lbg: list[float] = []
        self.ubg: list[float] = []
        self.meta: list[ConstraintMeta] = []

        self.J: Any = 0.0

    def add_var(
        self,
        name: str,
        n: int,
        lb: float = -ca.inf,
        ub: float = ca.inf,
        init: float | np.ndarray = 0.0,
    ):
        if n <= 0:
            raise ValueError("Variable length must be positive.")
        if name in self.var_slices:
            raise ValueError(f"Variable {name!r} already exists.")
        x = ca.MX.sym(name, n)
        self.w.append(x)
        self.lbw.extend(np.full(n, lb, dtype=float))
        self.ubw.extend(np.full(n, ub, dtype=float))
        init_arr = np.asarray(init, dtype=float)
        if init_arr.ndim == 0:
            init_arr = np.full(n, float(init_arr))
        if init_arr.shape != (n,):
            raise ValueError(f"Initial value for {name!r} must have shape ({n},).")
        self.w0.extend(init_arr.tolist())
        self.var_slices[name] = slice(self._nx, self._nx + n)
        self._nx += n
        return x

    def add_con(self, expr: Any, lb: float | np.ndarray, ub: float | np.ndarray, meta: ConstraintMeta):
        expr = ca.reshape(expr, -1, 1)
        lb_arr = np.full(expr.shape[0], lb, dtype=float) if np.isscalar(lb) else np.asarray(lb, dtype=float)
        ub_arr = np.full(expr.shape[0], ub, dtype=float) if np.isscalar(ub) else np.asarray(ub, dtype=float)
        if lb_arr.shape != (expr.shape[0],) or ub_arr.shape != (expr.shape[0],):
            raise ValueError("Constraint bounds must match flattened expression length.")
        for row in range(expr.shape[0]):
            self.g.append(expr[row])
            self.lbg.append(float(lb_arr[row]))
            self.ubg.append(float(ub_arr[row]))
            self.meta.append(meta)

    def build_solver(self, name: str = "solver", opts: dict[str, Any] | None = None):
        if not self.w:
            raise ValueError("Cannot build an NLP without decision variables.")
        w = ca.vertcat(*self.w)
        g = ca.vertcat(*self.g) if self.g else ca.MX.zeros(0, 1)
        nlp = {"x": w, "f": self.J, "g": g}
        solver_opts = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.max_iter": 100,
            "ipopt.tol": 1e-5,
        }
        if opts:
            solver_opts.update(opts)
        return ca.nlpsol(name, "ipopt", nlp, solver_opts), w, g

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            "x0": np.asarray(self.w0, dtype=float),
            "lbx": np.asarray(self.lbw, dtype=float),
            "ubx": np.asarray(self.ubw, dtype=float),
            "lbg": np.asarray(self.lbg, dtype=float),
            "ubg": np.asarray(self.ubg, dtype=float),
        }

    def value(self, name: str, x: np.ndarray) -> np.ndarray:
        return np.asarray(x, dtype=float)[self.var_slices[name]]
