from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Mapping

import casadi as ca
import numpy as np

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.active_set import VehicleOnPath
from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.mpc.nlp_builder import ConstraintMeta, NLPBuilder
from rcbranch.mpc.reciprocal_caution_mpc import MPCConfig, MPCWeights, MpcSolution


@dataclass(slots=True)
class BranchMpcSolution:
    status: str
    objective: float
    branch_s: dict[int, dict[int, np.ndarray]]
    branch_v: dict[int, dict[int, np.ndarray]]
    branch_a: dict[int, dict[int, np.ndarray]]
    ego_id: int
    branch_pair: tuple[int, int]
    branch_time_step: int
    solver_time_s: float
    mode: str = "branch_mpc"

    @property
    def ego_accel(self) -> np.ndarray:
        # Non-anticipativity enforces branch equality at k=0; return branch 0.
        return self.branch_a[0][self.ego_id]


def _find_event_interval(s_values: np.ndarray, target_s: float) -> int | None:
    for k in range(len(s_values) - 1):
        lo = min(s_values[k], s_values[k + 1])
        hi = max(s_values[k], s_values[k + 1])
        if lo <= target_s <= hi:
            return k
    return None


def _pair_belief(
    beliefs: Mapping[tuple[int, int], PairBelief],
    i: int,
    j: int,
) -> tuple[float, float]:
    belief = beliefs.get((i, j))
    if belief is not None:
        return belief.p_i_first, belief.p_j_first
    belief = beliefs.get((j, i))
    if belief is not None:
        return belief.p_j_first, belief.p_i_first
    return 0.5, 0.5


def _add_branch_dynamics_and_cost(
    builder: NLPBuilder,
    branch: int,
    vehicles: list[VehicleOnPath],
    cfg: MPCConfig,
    weights: MPCWeights,
    probability: float,
):
    n = cfg.horizon_steps
    s_vars: dict[int, ca.MX] = {}
    v_vars: dict[int, ca.MX] = {}
    a_vars: dict[int, ca.MX] = {}
    for vehicle in vehicles:
        vid = vehicle.obstacle_id
        s_vars[vid] = builder.add_var(f"b{branch}_s_{vid}", n + 1, init=vehicle.s0)
        v_vars[vid] = builder.add_var(
            f"b{branch}_v_{vid}",
            n + 1,
            lb=0.0,
            ub=cfg.v_max,
            init=max(vehicle.v0, 0.0),
        )
        a_vars[vid] = builder.add_var(
            f"b{branch}_a_{vid}",
            n,
            lb=-vehicle.max_brake,
            ub=vehicle.max_accel,
            init=vehicle.a_obs,
        )
        builder.add_con(s_vars[vid][0] - vehicle.s0, 0.0, 0.0, ConstraintMeta("initial_s", i=vid, k=0, branch=branch))
        builder.add_con(v_vars[vid][0] - vehicle.v0, 0.0, 0.0, ConstraintMeta("initial_v", i=vid, k=0, branch=branch))
        for k in range(n):
            s_next = s_vars[vid][k] + cfg.dt * v_vars[vid][k] + 0.5 * cfg.dt * cfg.dt * a_vars[vid][k]
            v_next = v_vars[vid][k] + cfg.dt * a_vars[vid][k]
            builder.add_con(s_vars[vid][k + 1] - s_next, 0.0, 0.0, ConstraintMeta("dynamics_s", i=vid, k=k, branch=branch))
            builder.add_con(v_vars[vid][k + 1] - v_next, 0.0, 0.0, ConstraintMeta("dynamics_v", i=vid, k=k, branch=branch))
            previous_a = vehicle.a_obs if k == 0 else a_vars[vid][k - 1]
            builder.J += probability * weights.w_v * (v_vars[vid][k] - vehicle.desired_speed) ** 2
            builder.J += probability * weights.w_a * a_vars[vid][k] ** 2
            builder.J += probability * weights.w_jerk * (a_vars[vid][k] - previous_a) ** 2
    return s_vars, v_vars, a_vars


def _add_event_time(
    builder: NLPBuilder,
    *,
    branch: int,
    label: str,
    s_var: ca.MX,
    s_warm: np.ndarray,
    target_s: float,
    cfg: MPCConfig,
):
    interval = _find_event_interval(s_warm, target_s)
    if interval is None:
        raise ValueError(f"Cannot locate fixed event interval for {label} at s={target_s}.")
    t0 = interval * cfg.dt
    event_t = builder.add_var(f"b{branch}_T_{label}", 1, lb=t0, ub=t0 + cfg.dt, init=t0)
    alpha = (event_t[0] - t0) / cfg.dt
    s_interp = s_var[interval] + alpha * (s_var[interval + 1] - s_var[interval])
    builder.add_con(
        s_interp - target_s,
        0.0,
        0.0,
        ConstraintMeta("event_time", i=None, k=interval, branch=branch),
    )
    return event_t[0]


def solve_branch_mpc(
    active: list[VehicleOnPath],
    conflicts: list[ConflictRegion],
    beliefs: Mapping[tuple[int, int], PairBelief],
    *,
    branch_pair: tuple[int, int],
    kb: int,
    warm_start: MpcSolution,
    config: MPCConfig | None = None,
    weights: MPCWeights | None = None,
    delta_safe_time: float = 0.7,
) -> BranchMpcSolution:
    """Solve a two-branch fixed-event crossing-order MPC for one conflict pair."""

    cfg = config or MPCConfig()
    wgt = weights or MPCWeights()
    i, j = branch_pair
    conflict = next((c for c in conflicts if {c.i, c.j} == {i, j}), None)
    if conflict is None:
        raise ValueError(f"No conflict found for branch pair {branch_pair}.")
    p_i_first, p_j_first = _pair_belief(beliefs, i, j)
    total = max(p_i_first + p_j_first, 1e-9)
    probabilities = {0: p_i_first / total, 1: p_j_first / total}

    builder = NLPBuilder()
    branch_vars = {
        branch: _add_branch_dynamics_and_cost(builder, branch, active, cfg, wgt, probabilities[branch])
        for branch in (0, 1)
    }
    for vehicle in active:
        vid = vehicle.obstacle_id
        _, _, a0 = branch_vars[0]
        _, _, a1 = branch_vars[1]
        shared_until = max(1, min(kb, cfg.horizon_steps))
        for k in range(shared_until):
            builder.add_con(
                a0[vid][k] - a1[vid][k],
                0.0,
                0.0,
                ConstraintMeta("nonanticipativity", i=vid, k=k),
            )

    for branch, order in ((0, (i, j)), (1, (j, i))):
        s_vars, _, _ = branch_vars[branch]
        first, second = order
        if first == conflict.i:
            first_in, first_out = conflict.s_i_in, conflict.s_i_out
            second_in = conflict.s_j_in
        else:
            first_in, first_out = conflict.s_j_in, conflict.s_j_out
            second_in = conflict.s_i_in
        t_first_in = _add_event_time(
            builder,
            branch=branch,
            label=f"{first}_in",
            s_var=s_vars[first],
            s_warm=warm_start.s[first],
            target_s=first_in,
            cfg=cfg,
        )
        t_first_out = _add_event_time(
            builder,
            branch=branch,
            label=f"{first}_out",
            s_var=s_vars[first],
            s_warm=warm_start.s[first],
            target_s=first_out,
            cfg=cfg,
        )
        t_second_in = _add_event_time(
            builder,
            branch=branch,
            label=f"{second}_in",
            s_var=s_vars[second],
            s_warm=warm_start.s[second],
            target_s=second_in,
            cfg=cfg,
        )
        order_slack = builder.add_var(
            f"b{branch}_xi_order_{first}_before_{second}",
            1,
            lb=0.0,
            ub=ca.inf,
            init=0.0,
        )
        builder.add_con(
            t_second_in - t_first_out + order_slack[0],
            delta_safe_time,
            ca.inf,
            ConstraintMeta("crossing_order", i=first, j=second, branch=branch),
        )
        builder.J += probabilities[branch] * 1000.0 * order_slack[0] ** 2
        builder.J += 1e-4 * t_first_in

    solver, _, _ = builder.build_solver("branch_mpc", {"ipopt.max_iter": cfg.max_ipopt_iter, "ipopt.tol": cfg.solver_tol})
    arrays = builder.arrays()
    start = perf_counter()
    sol = solver(**arrays)
    elapsed = perf_counter() - start
    x = np.asarray(sol["x"], dtype=float).reshape(-1)
    ego = next((vehicle.obstacle_id for vehicle in active if vehicle.is_ego), active[0].obstacle_id)
    branch_s: dict[int, dict[int, np.ndarray]] = {0: {}, 1: {}}
    branch_v: dict[int, dict[int, np.ndarray]] = {0: {}, 1: {}}
    branch_a: dict[int, dict[int, np.ndarray]] = {0: {}, 1: {}}
    for branch in (0, 1):
        for vehicle in active:
            vid = vehicle.obstacle_id
            branch_s[branch][vid] = builder.value(f"b{branch}_s_{vid}", x)
            branch_v[branch][vid] = builder.value(f"b{branch}_v_{vid}", x)
            branch_a[branch][vid] = builder.value(f"b{branch}_a_{vid}", x)
    return BranchMpcSolution(
        status=str(solver.stats().get("return_status", "unknown")),
        objective=float(sol["f"]),
        branch_s=branch_s,
        branch_v=branch_v,
        branch_a=branch_a,
        ego_id=ego,
        branch_pair=branch_pair,
        branch_time_step=kb,
        solver_time_s=elapsed,
    )
