from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from rcbranch.belief.crossing_order_filter import CrossingOrderBeliefFilter, PairBelief
from rcbranch.belief.features import compute_pair_features
from rcbranch.geometry.active_set import VehicleOnPath, select_front_vehicles_by_incoming
from rcbranch.geometry.conflict_regions import ConflictRegion, build_conflict_graph
from rcbranch.mpc.branch_mpc import BranchMpcSolution, solve_branch_mpc
from rcbranch.mpc.branching import BranchScore, choose_branch_time, compute_branch_scores
from rcbranch.mpc.duals import smooth_dual_series
from rcbranch.mpc.reciprocal_caution_mpc import MPCConfig, MPCWeights, MpcSolution, solve_reciprocal_caution_mpc


@dataclass(slots=True)
class PlannerMemory:
    belief_filter: CrossingOrderBeliefFilter = field(default_factory=CrossingOrderBeliefFilter)
    previous_duals: dict[tuple[int, int, int], float] = field(default_factory=dict)
    previous_solution: MpcSolution | BranchMpcSolution | None = None
    beliefs: dict[tuple[int, int], PairBelief] = field(default_factory=dict)
    scores: dict[tuple[int, int], BranchScore] = field(default_factory=dict)
    planner_mode: str = "uninitialized"

    def update(
        self,
        *,
        selected_sol: MpcSolution | BranchMpcSolution,
        beliefs: dict[tuple[int, int], PairBelief],
        scores: dict[tuple[int, int], BranchScore],
        planner_mode: str,
    ) -> None:
        self.previous_solution = selected_sol
        self.beliefs = dict(beliefs)
        self.scores = dict(scores)
        self.planner_mode = planner_mode


def _vehicles_by_id(vehicles: list[VehicleOnPath]) -> dict[int, VehicleOnPath]:
    return {vehicle.obstacle_id: vehicle for vehicle in vehicles}


def _solver_succeeded(status: str) -> bool:
    return status in {"Solve_Succeeded", "Solved_To_Acceptable_Level"}


def update_pair_beliefs(
    active: list[VehicleOnPath],
    conflicts: list[ConflictRegion],
    memory: PlannerMemory,
    *,
    safe_time: float = 0.7,
) -> dict[tuple[int, int], PairBelief]:
    by_id = _vehicles_by_id(active)
    beliefs: dict[tuple[int, int], PairBelief] = {}
    for conflict in conflicts:
        if conflict.i not in by_id or conflict.j not in by_id:
            continue
        features = compute_pair_features(by_id[conflict.i], by_id[conflict.j], conflict, safe_time=safe_time)
        beliefs[(conflict.i, conflict.j)] = memory.belief_filter.update(conflict.i, conflict.j, features)
    return beliefs


def run_mpc_cycle(
    vehicles: list[VehicleOnPath],
    memory: PlannerMemory,
    *,
    known_conflicts: list[ConflictRegion] | None = None,
    mpc_config: MPCConfig | None = None,
    weights: MPCWeights | None = None,
    config: dict[str, Any] | None = None,
) -> tuple[float, MpcSolution | BranchMpcSolution]:
    """Run one fixed-path planner cycle and return ego acceleration."""

    config = config or {}
    branching_cfg = config.get("branching", {})
    belief_cfg = config.get("belief", {})
    dt = (mpc_config or MPCConfig()).dt
    tau_psi_big = float(branching_cfg.get("tau_Psi", 1.0))
    tau_psi_step = float(branching_cfg.get("tau_psi", 0.3))
    ema_alpha = float(branching_cfg.get("lambda_ema_alpha", 0.7))
    safe_time = float(belief_cfg.get("safe_time", branching_cfg.get("delta_safe_time", 0.7)))

    conflicts = known_conflicts or build_conflict_graph(vehicles)
    active = select_front_vehicles_by_incoming(vehicles, conflicts)
    if not active:
        active = vehicles
    conflicts = [c for c in conflicts if c.i in _vehicles_by_id(active) and c.j in _vehicles_by_id(active)]
    beliefs = update_pair_beliefs(active, conflicts, memory, safe_time=safe_time)
    rc_sol = solve_reciprocal_caution_mpc(active, conflicts, beliefs, config=mpc_config, weights=weights)
    smoothed_duals = smooth_dual_series(memory.previous_duals, rc_sol.duals, alpha=ema_alpha)
    memory.previous_duals = smoothed_duals
    scores = compute_branch_scores(smoothed_duals, beliefs, conflicts, dt=dt)

    if not scores:
        memory.update(selected_sol=rc_sol, beliefs=beliefs, scores=scores, planner_mode="reciprocal_caution_only")
        return float(rc_sol.ego_accel[0]), rc_sol

    best_pair, best_score = max(scores.items(), key=lambda kv: kv[1].psi)
    if best_score.psi <= tau_psi_big:
        memory.update(selected_sol=rc_sol, beliefs=beliefs, scores=scores, planner_mode="reciprocal_caution_only")
        return float(rc_sol.ego_accel[0]), rc_sol

    kb = choose_branch_time(best_score, tau_psi_step)
    try:
        branch_sol = solve_branch_mpc(
            active,
            conflicts,
            beliefs,
            branch_pair=best_pair,
            kb=kb,
            warm_start=rc_sol,
            config=mpc_config,
            weights=weights,
            delta_safe_time=float(branching_cfg.get("delta_safe_time", 0.7)),
        )
    except ValueError:
        memory.update(selected_sol=rc_sol, beliefs=beliefs, scores=scores, planner_mode="branch_fallback_rc")
        return float(rc_sol.ego_accel[0]), rc_sol
    if not _solver_succeeded(branch_sol.status):
        memory.update(selected_sol=rc_sol, beliefs=beliefs, scores=scores, planner_mode="branch_fallback_rc")
        return float(rc_sol.ego_accel[0]), rc_sol
    memory.update(selected_sol=branch_sol, beliefs=beliefs, scores=scores, planner_mode="branch_mpc")
    return float(branch_sol.ego_accel[0]), branch_sol
