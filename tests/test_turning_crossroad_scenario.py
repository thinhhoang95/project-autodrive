from rcbranch.geometry import build_conflict_graph
from rcbranch.mpc import MPCConfig
from rcbranch.mpc.branch_mpc import BranchMpcSolution
from rcbranch.planners import PlannerMemory, run_mpc_cycle
from rcbranch.scenarios import build_turning_crossroad_scene


def test_reciprocal_caution_is_visible_on_turning_crossroad_scenario():
    vehicles = build_turning_crossroad_scene()
    conflicts = build_conflict_graph(vehicles, sample_step=0.05)
    memory = PlannerMemory()

    ego_a0, solution = run_mpc_cycle(
        vehicles,
        memory,
        known_conflicts=conflicts,
        mpc_config=MPCConfig(max_ipopt_iter=40),
        config={"branching": {"tau_Psi": 1.0e9}},
    )

    assert solution.ego_id == 1
    assert len(solution.ego_accel) > 0
    assert -vehicles[0].max_brake <= ego_a0 <= vehicles[0].max_accel
    assert memory.planner_mode == "reciprocal_caution_only"
    assert ego_a0 < -0.5
    assert sum(solution.duals.values()) > 1.0


def test_branching_is_visible_on_turning_crossroad_scenario():
    vehicles = build_turning_crossroad_scene()
    conflicts = build_conflict_graph(vehicles, sample_step=0.05)
    memory = PlannerMemory()

    ego_a0, solution = run_mpc_cycle(
        vehicles,
        memory,
        known_conflicts=conflicts,
        mpc_config=MPCConfig(max_ipopt_iter=80),
    )

    score = memory.scores[(1, 2)]
    assert memory.planner_mode == "branch_mpc"
    assert isinstance(solution, BranchMpcSolution)
    assert solution.status == "Solve_Succeeded"
    assert score.ambiguity > 0.8
    assert score.psi > 1.0
    assert ego_a0 < -0.5
