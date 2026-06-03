from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import Mapping

import casadi as ca
import numpy as np

from rcbranch.belief.crossing_order_filter import PairBelief
from rcbranch.geometry.active_set import VehicleOnPath
from rcbranch.geometry.conflict_regions import ConflictRegion
from rcbranch.mpc.constraints import constant_velocity_rollout, pre_entry_indices
from rcbranch.mpc.duals import extract_caution_duals
from rcbranch.mpc.nlp_builder import ConstraintMeta, NLPBuilder


@dataclass(frozen=True, slots=True)
class MPCConfig:
    dt: float = 0.2
    horizon_steps: int = 25
    max_ipopt_iter: int = 100
    solver_tol: float = 1e-5
    v_max: float = 13.9
    d_buf: float = 3.0
    pre_entry_epsilon: float = 0.25


@dataclass(frozen=True, slots=True)
class MPCWeights:
    w_v: float = 1.0
    w_a: float = 0.2
    w_jerk: float = 1.0
    w_slack_caution: float = 500.0


@dataclass(slots=True)
class MpcSolution:
    status: str
    objective: float
    s: dict[int, np.ndarray]
    v: dict[int, np.ndarray]
    a: dict[int, np.ndarray]
    ego_id: int
    duals: dict[tuple[int, int, int], float] = field(default_factory=dict)
    raw_lam_g: np.ndarray = field(default_factory=lambda: np.zeros(0))
    constraint_meta: list[ConstraintMeta] = field(default_factory=list)
    solver_time_s: float = 0.0
    mode: str = "reciprocal_caution"

    @property
    def ego_accel(self) -> np.ndarray:
        return self.a[self.ego_id]


def _is_conflict_unresolved(
    beliefs: Mapping[tuple[int, int], PairBelief] | None,
    i: int,
    j: int,
    threshold: float = 0.05,
) -> bool:
    if not beliefs:
        return True
    belief = beliefs.get((i, j)) or beliefs.get((j, i))
    if belief is None:
        return True
    return belief.p_unresolved > threshold or belief.ambiguity > threshold


def solve_reciprocal_caution_mpc(
    active: list[VehicleOnPath],
    conflicts: list[ConflictRegion],
    beliefs: Mapping[tuple[int, int], PairBelief] | None = None,
    *,
    config: MPCConfig | None = None,
    weights: MPCWeights | None = None,
) -> MpcSolution:
    if not active:
        raise ValueError("At least one active vehicle is required.")
    cfg = config or MPCConfig()
    wgt = weights or MPCWeights()
    n = cfg.horizon_steps
    dt = cfg.dt
    builder = NLPBuilder()

    s_vars: dict[int, ca.MX] = {}
    v_vars: dict[int, ca.MX] = {}
    a_vars: dict[int, ca.MX] = {}
    vehicles = {vehicle.obstacle_id: vehicle for vehicle in active}
    for vehicle in active:
        vid = vehicle.obstacle_id
        s_init = constant_velocity_rollout(vehicle.s0, vehicle.v0, dt, n)
        v_init = np.full(n + 1, max(vehicle.v0, 0.0))
        s_vars[vid] = builder.add_var(f"s_{vid}", n + 1, init=s_init)
        v_vars[vid] = builder.add_var(f"v_{vid}", n + 1, lb=0.0, ub=cfg.v_max, init=v_init)
        a_vars[vid] = builder.add_var(
            f"a_{vid}",
            n,
            lb=-vehicle.max_brake,
            ub=vehicle.max_accel,
            init=np.full(n, vehicle.a_obs),
        )

        builder.add_con(s_vars[vid][0] - vehicle.s0, 0.0, 0.0, ConstraintMeta("initial_s", i=vid, k=0))
        builder.add_con(v_vars[vid][0] - vehicle.v0, 0.0, 0.0, ConstraintMeta("initial_v", i=vid, k=0))
        for k in range(n):
            s_next = s_vars[vid][k] + dt * v_vars[vid][k] + 0.5 * dt * dt * a_vars[vid][k]
            v_next = v_vars[vid][k] + dt * a_vars[vid][k]
            builder.add_con(s_vars[vid][k + 1] - s_next, 0.0, 0.0, ConstraintMeta("dynamics_s", i=vid, k=k))
            builder.add_con(v_vars[vid][k + 1] - v_next, 0.0, 0.0, ConstraintMeta("dynamics_v", i=vid, k=k))
            builder.J += wgt.w_v * (v_vars[vid][k] - vehicle.desired_speed) ** 2
            builder.J += wgt.w_a * a_vars[vid][k] ** 2
            previous_a = vehicle.a_obs if k == 0 else a_vars[vid][k - 1]
            builder.J += wgt.w_jerk * (a_vars[vid][k] - previous_a) ** 2

    for conflict in conflicts:
        if conflict.i not in vehicles or conflict.j not in vehicles:
            continue
        if not _is_conflict_unresolved(beliefs, conflict.i, conflict.j):
            continue
        for follower_id, leader_id, s_in in (
            (conflict.i, conflict.j, conflict.s_i_in),
            (conflict.j, conflict.i, conflict.s_j_in),
        ):
            vehicle = vehicles[follower_id]
            s_ref = constant_velocity_rollout(vehicle.s0, vehicle.v0, dt, n)
            gated_ks = pre_entry_indices(s_ref, s_in, cfg.pre_entry_epsilon)
            if not gated_ks:
                continue
            xi = builder.add_var(
                f"xi_{follower_id}_yield_{leader_id}",
                n + 1,
                lb=0.0,
                ub=ca.inf,
                init=0.0,
            )
            for k in gated_ks:
                h = (
                    v_vars[follower_id][k] ** 2 / (2.0 * vehicle.comfort_brake)
                    + cfg.d_buf
                    - (s_in - s_vars[follower_id][k])
                )
                builder.add_con(
                    h - xi[k],
                    -ca.inf,
                    0.0,
                    ConstraintMeta("reciprocal_caution", i=follower_id, j=leader_id, k=k),
                )
                builder.J += wgt.w_slack_caution * xi[k] ** 2

    opts = {"ipopt.max_iter": cfg.max_ipopt_iter, "ipopt.tol": cfg.solver_tol}
    solver, _, _ = builder.build_solver("rc_mpc", opts=opts)
    arrays = builder.arrays()
    start = perf_counter()
    sol = solver(**arrays)
    elapsed = perf_counter() - start
    x = np.asarray(sol["x"], dtype=float).reshape(-1)
    lam_g = np.asarray(sol["lam_g"], dtype=float).reshape(-1)
    ego = next((vehicle.obstacle_id for vehicle in active if vehicle.is_ego), active[0].obstacle_id)
    return MpcSolution(
        status=str(solver.stats().get("return_status", "unknown")),
        objective=float(sol["f"]),
        s={vid: builder.value(f"s_{vid}", x) for vid in vehicles},
        v={vid: builder.value(f"v_{vid}", x) for vid in vehicles},
        a={vid: builder.value(f"a_{vid}", x) for vid in vehicles},
        ego_id=ego,
        duals=extract_caution_duals(lam_g, builder.meta),
        raw_lam_g=lam_g,
        constraint_meta=list(builder.meta),
        solver_time_s=elapsed,
    )
