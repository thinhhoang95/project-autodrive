# Framework Walkthrough

This document explains the `rcbranch` framework from first principles to the end-to-end planner loop. It is written as both a tutorial and a codebase map, so you can read it once to understand the system and return to individual sections when extending or debugging the implementation.

The short version:

```text
CommonRoad or synthetic scenario
  -> route/reference paths
  -> path-coordinate vehicle states
  -> conflict regions
  -> active front-vehicle set
  -> pairwise crossing-order beliefs
  -> reciprocal-caution MPC
  -> reciprocal-caution dual prices
  -> branch trigger
  -> optional two-branch MPC
  -> first ego acceleration
  -> updated trajectory / CommonRoad state
```

The implementation is intentionally split into modules so each part can be tested and replaced independently.

## 1. Environment Setup

The project targets Python `>=3.11,<3.14`, because the CommonRoad ecosystem currently fits that range better than Python 3.14.

Install the core planner and tests:

```bash
uv sync --python /opt/homebrew/bin/python3.11 --extra dev
```

Install the full CommonRoad stack when you need real CommonRoad XML loading, validation, route planning, or solution export:

```bash
uv sync --python /opt/homebrew/bin/python3.11 --extra dev --extra commonroad
```

On macOS, `commonroad-drivability-checker` may need OpenMP:

```bash
brew install libomp
```

Run tests:

```bash
uv run --python /opt/homebrew/bin/python3.11 pytest
```

Run a basic package check:

```bash
uv run --python /opt/homebrew/bin/python3.11 python main.py
```

Expected output includes the configured MPC horizon:

```text
rcbranch planner package ready; horizon_steps=25
```

## 2. Repository Map

The implementation lives under `src/rcbranch`.

```text
src/rcbranch/
  commonroad_adapter/   CommonRoad loading, route, state, and solution adapters
  geometry/             fixed-path representation, projection, conflict regions
  belief/               crossing-order feature extraction and temporal filter
  mpc/                  CasADi NLP builder, RC-MPC, branch scoring, branch MPC
  planners/             proposed planner loop and baseline wrappers
  evaluation/           simple closed-loop runner, metrics, validation hooks
  scripts/              installed CLI entry points
```

Other important folders:

```text
configs/                default planner, baseline, scenario, and weight YAMLs
scripts/                root-level wrappers for CLI entry points
tests/                  focused component tests
prompts/                source design notes and implementation plan
```

Start here when navigating:

- `configs/default.yaml`: primary parameters for MPC, vehicle limits, belief, and branching.
- `src/rcbranch/planners/proposed_dual_priced_branching.py`: one-cycle planner orchestration.
- `src/rcbranch/mpc/reciprocal_caution_mpc.py`: single reciprocal-caution MPC.
- `src/rcbranch/mpc/branch_mpc.py`: two-branch contingency MPC.
- `tests/`: examples of how individual components are meant to behave.

## 3. Core Concept

The framework is a fixed-route intersection planner. It does not initially optimize a full Cartesian vehicle model. Instead, every active vehicle is represented along a known reference path:

```text
x_i = (s_i, v_i)
u_i = a_i
```

The longitudinal dynamics are:

```text
s[k+1] = s[k] + dt * v[k] + 0.5 * dt^2 * a[k]
v[k+1] = v[k] + dt * a[k]
```

This is the right abstraction for the current research scope:

- fixed paths,
- no lane changes,
- no overtaking,
- local crossing-order uncertainty at uncontrolled intersections.

The planner only executes the first ego acceleration from each MPC solve. Accelerations optimized for other vehicles are predictions, not commands.

## 4. Configuration

The default configuration is in `configs/default.yaml`.

Important sections:

```yaml
mpc:
  dt: 0.2
  horizon_steps: 25
  max_ipopt_iter: 100
  solver_tol: 1.0e-5

vehicle:
  v_max: 13.9
  a_max: 2.0
  b_max: 6.0
  b_comf: 3.0
  d_buf: 3.0

objective:
  w_v: 1.0
  w_a: 0.2
  w_jerk: 1.0
  w_slack_caution: 500.0

branching:
  tau_Psi: 1.0
  tau_psi: 0.3
  lambda_ema_alpha: 0.7
  delta_safe_time: 0.7
```

Load config with:

```python
from rcbranch.config import load_config

config = load_config("configs/default.yaml")
print(config.mpc["horizon_steps"])
```

The dataclass wrappers used by MPC are narrower than the YAML. When constructing them manually, filter keys:

```python
from rcbranch.mpc import MPCConfig, MPCWeights

mpc_config = MPCConfig(
    **{k: config.mpc[k] for k in MPCConfig.__dataclass_fields__ if k in config.mpc}
)
weights = MPCWeights(
    **{k: config.objective[k] for k in MPCWeights.__dataclass_fields__ if k in config.objective}
)
```

## 5. Geometry Layer

The geometry layer converts global paths and positions into longitudinal path coordinates.

### 5.1 Reference Paths

File: `src/rcbranch/geometry/reference_path.py`

`ReferencePath` stores:

- `xy`: polyline points,
- `s`: cumulative arc length,
- `theta`: path heading,
- `kappa`: approximate curvature.

Create a path:

```python
from rcbranch.geometry import ReferencePath

path = ReferencePath.from_xy([
    [0.0, 0.0],
    [10.0, 0.0],
])
```

Project a point to arc length:

```python
s = path.project_xy_to_s([4.2, 1.5])
```

Interpolate back to Cartesian state:

```python
x, y, theta, kappa = path.interpolate_xytheta(s)
```

This is tested in `tests/test_projection.py`.

### 5.2 Conflict Regions

File: `src/rcbranch/geometry/conflict_regions.py`

For each pair of paths, the framework builds swept path tubes with Shapely, intersects them, and samples each path to determine entry and exit coordinates:

```python
from rcbranch.geometry.conflict_regions import compute_conflict_region

east_west = ReferencePath.from_xy([[-10.0, 0.0], [10.0, 0.0]])
south_north = ReferencePath.from_xy([[0.0, -10.0], [0.0, 10.0]])

conflict = compute_conflict_region(
    east_west,
    south_north,
    i=1,
    j=2,
    half_width_i=1.0,
    half_width_j=1.0,
    margin=0.0,
    sample_step=0.05,
)
```

`ConflictRegion` contains:

```text
i, j
s_i_in, s_i_out
s_j_in, s_j_out
polygon
```

This is tested in `tests/test_conflict_regions.py`.

### 5.3 Vehicles On Paths

File: `src/rcbranch/geometry/active_set.py`

The main vehicle state container is:

```python
from rcbranch.geometry import VehicleOnPath
```

Fields include:

```text
obstacle_id
is_ego
route_id
incoming_id
ref_path
s0, v0, a_obs
length, width
desired_speed
max_accel, max_brake, comfort_brake
```

The planner groups vehicles by `incoming_id` and selects only the closest front vehicle per incoming approach for strategic crossing-order reasoning:

```python
from rcbranch.geometry import select_front_vehicles_by_incoming

active = select_front_vehicles_by_incoming(vehicles, conflicts)
```

Followers can still be modeled later through car-following constraints, but they do not need to create new crossing-order branches in the first implementation.

## 6. CommonRoad Adapter

The CommonRoad adapter is intentionally lazy-imported. Core geometry, belief, and MPC tests can run without CommonRoad installed.

### 6.1 Loading Scenarios

File: `src/rcbranch/commonroad_adapter/load.py`

```python
from rcbranch.commonroad_adapter import load_commonroad_problem

scenario, planning_problem = load_commonroad_problem("path/to/scenario.xml")
```

The loader calls:

```python
CommonRoadFileReader(xml_path).open(lanelet_assignment=True)
```

You can filter for uncontrolled intersection scenarios:

```python
from rcbranch.commonroad_adapter import is_uncontrolled_intersection_scenario

if is_uncontrolled_intersection_scenario(scenario):
    ...
```

### 6.2 Route and State Conversion

Files:

- `src/rcbranch/commonroad_adapter/route_extraction.py`
- `src/rcbranch/commonroad_adapter/state_conversion.py`

CommonRoad lanelet centerlines can be converted into a `ReferencePath`:

```python
from rcbranch.commonroad_adapter.route_extraction import (
    lanelets_by_id,
    reference_path_from_lanelet_sequence,
)

lanelets = lanelets_by_id(scenario, [101, 102, 103])
ref_path = reference_path_from_lanelet_sequence(lanelets)
```

Convert CommonRoad states to path states:

```python
from rcbranch.commonroad_adapter.state_conversion import commonroad_state_to_path_state

path_state = commonroad_state_to_path_state(commonroad_state, ref_path)
```

Convert executed `(s, v, a)` back to a CommonRoad `State`:

```python
from rcbranch.commonroad_adapter.state_conversion import path_state_to_commonroad_state

state = path_state_to_commonroad_state(
    s=s_next,
    v=v_next,
    a=a0,
    ref_path=ref_path,
    time_step=42,
)
```

## 7. Belief Layer

The belief layer tracks local crossing-order uncertainty for each conflict pair.

Files:

- `src/rcbranch/belief/features.py`
- `src/rcbranch/belief/crossing_order_filter.py`

Each pair has a belief:

```text
P(i first)
P(j first)
P(unresolved)
```

The ambiguity score is:

```text
A_ij = 1 - abs(P(i first) - P(j first))
```

High ambiguity means the planner is uncertain between the two crossing orders. Low ambiguity means one order is already clear.

### 7.1 Pair Features

`compute_pair_features(...)` calculates:

- constant-velocity conflict entry and exit times,
- required stopping deceleration,
- observed accelerations,
- time gaps if either vehicle goes first.

Example:

```python
from rcbranch.belief.features import compute_pair_features

features = compute_pair_features(vehicle_i, vehicle_j, conflict, safe_time=0.7)
```

### 7.2 Belief Filter

`CrossingOrderBeliefFilter` applies a prototype-cost likelihood and temporal persistence:

```python
from rcbranch.belief import CrossingOrderBeliefFilter

belief_filter = CrossingOrderBeliefFilter()
belief = belief_filter.update(1, 2, features)

print(belief.p_i_first, belief.p_j_first, belief.p_unresolved)
print(belief.ambiguity)
```

This is tested in `tests/test_belief_filter.py`.

## 8. MPC Infrastructure

The MPC layer is built around a low-level CasADi NLP helper instead of hiding everything inside `Opti`.

### 8.1 NLP Builder

File: `src/rcbranch/mpc/nlp_builder.py`

`NLPBuilder` stores:

- decision variables,
- initial guesses,
- variable bounds,
- constraints,
- constraint bounds,
- objective,
- metadata for each constraint.

The metadata is important because reciprocal-caution multipliers need to be mapped back to conflict pair and horizon step:

```python
from rcbranch.mpc.nlp_builder import ConstraintMeta

ConstraintMeta(kind="reciprocal_caution", i=1, j=2, k=7)
```

The toy multiplier sanity check in `tests/test_casadi_toy_multiplier.py` verifies the interpretation:

```text
min 0.5 * (v - v_des)^2
s.t. v - v_safe <= 0

|lambda| = max(0, v_des - v_safe)
```

### 8.2 Reciprocal-Caution MPC

File: `src/rcbranch/mpc/reciprocal_caution_mpc.py`

The solver creates variables for every active vehicle:

```text
s[0:N]
v[0:N]
a[0:N-1]
```

It enforces:

- initial state constraints,
- longitudinal dynamics,
- velocity bounds,
- acceleration bounds,
- reciprocal-caution constraints when crossing order is unresolved.

The reciprocal-caution inequality is:

```text
v_i[k]^2 / (2 * b_comf_i) + d_buf - (s_in_i - s_i[k]) <= xi_i_yield_j[k]
```

This says: while the crossing order is unresolved and the vehicle is still before the conflict entry, it should preserve a comfortable yield option. Slack is allowed but penalized heavily.

Important detail: the caution constraint is gated by a constant-velocity rollout. It is only imposed for predicted pre-entry indices, so the solver does not keep applying a stopping-distance constraint after the vehicle has entered the conflict region.

Call it directly:

```python
from rcbranch.mpc import solve_reciprocal_caution_mpc

rc_solution = solve_reciprocal_caution_mpc(
    active=active_vehicles,
    conflicts=conflicts,
    beliefs=beliefs,
    config=mpc_config,
    weights=weights,
)

print(rc_solution.ego_accel[0])
print(rc_solution.duals)
```

`rc_solution.duals` has keys:

```text
(yielding_vehicle_id, other_vehicle_id, horizon_step)
```

## 9. Branch Scoring

File: `src/rcbranch/mpc/branching.py`

After solving reciprocal-caution MPC, the framework extracts caution multipliers and computes:

```text
Psi_ij = A_ij * sum_k(lambda_i_yield_j[k] + lambda_j_yield_i[k]) * dt
```

In code:

```python
from rcbranch.mpc.branching import compute_branch_scores, choose_branch_time

scores = compute_branch_scores(duals, beliefs, conflicts, dt=0.2)
best_pair, best_score = max(scores.items(), key=lambda kv: kv[1].psi)
kb = choose_branch_time(best_score, tau_psi=0.3)
```

Interpretation:

- high ambiguity and low dual prices: uncertainty exists but is cheap, so stay with reciprocal caution,
- low ambiguity and high dual prices: one order is already clear, so branching is unnecessary,
- high ambiguity and high dual prices: keeping both yield options alive is expensive, so branch.

This is tested in `tests/test_branch_trigger.py`.

## 10. Branch MPC

File: `src/rcbranch/mpc/branch_mpc.py`

When branch score exceeds `tau_Psi`, the planner builds two branches:

```text
branch 0: i first, j second
branch 1: j first, i second
```

The branch objective is weighted by crossing-order probabilities:

```text
P(i first) * J_i_first + P(j first) * J_j_first
```

Before the branch time, controls are tied by non-anticipativity constraints:

```text
a_branch0[q, k] = a_branch1[q, k], for k < k_b
```

The current ego action is always shared because at execution time the uncertainty has not resolved yet.

Crossing order is encoded with event-time variables:

```text
T_second_in >= T_first_out + delta_safe_time
```

The current implementation uses fixed event-index interpolation based on the reciprocal-caution solution. If the warm-start trajectory does not cross the conflict entry/exit interval, branch MPC raises a `ValueError`; the planner catches this and falls back to reciprocal-caution MPC for that cycle.

## 11. Planner Loop

File: `src/rcbranch/planners/proposed_dual_priced_branching.py`

The main function is:

```python
from rcbranch.planners import PlannerMemory, run_mpc_cycle

memory = PlannerMemory()
ego_a0, solution = run_mpc_cycle(vehicles, memory)
```

One cycle does the following:

1. Build or receive conflict regions.
2. Select front vehicles by incoming approach.
3. Update pairwise crossing-order beliefs.
4. Solve reciprocal-caution MPC.
5. Smooth reciprocal-caution duals with EMA.
6. Compute branch scores.
7. If no score exceeds threshold, execute reciprocal-caution solution.
8. If a score exceeds threshold, solve two-branch MPC.
9. Return the first ego acceleration.
10. Store diagnostics in `PlannerMemory`.

The planner mode is stored in:

```python
memory.planner_mode
```

Possible values include:

```text
reciprocal_caution_only
branch_mpc
branch_fallback_rc
```

## 12. Runnable Toy Example

This example builds a two-car perpendicular crossroad without CommonRoad XML. It exercises the same core pipeline: reference paths, vehicles, conflict graph, planner cycle.

```python
from rcbranch.config import load_config
from rcbranch.geometry import ReferencePath, VehicleOnPath, build_conflict_graph
from rcbranch.mpc import MPCConfig, MPCWeights
from rcbranch.planners import PlannerMemory, run_mpc_cycle


def filtered_dataclass(cls, values):
    return cls(**{k: values[k] for k in cls.__dataclass_fields__ if k in values})


config = load_config("configs/default.yaml")
mpc_config = filtered_dataclass(MPCConfig, config.mpc)
weights = filtered_dataclass(MPCWeights, config.objective)
branching = dict(config.branching)

# For the first tutorial run, keep the cycle in reciprocal-caution mode.
# Lower this later when you want to inspect branch-MPC behavior.
branching["tau_Psi"] = 1.0e9

east_west = ReferencePath.from_xy([[-20.0, 0.0], [20.0, 0.0]])
south_north = ReferencePath.from_xy([[0.0, -20.0], [0.0, 20.0]])

ego = VehicleOnPath(
    obstacle_id=1,
    is_ego=True,
    route_id="east_west",
    incoming_id="west",
    ref_path=east_west,
    s0=2.0,
    v0=7.0,
    a_obs=0.0,
    length=4.5,
    width=2.0,
    desired_speed=8.0,
    max_accel=2.0,
    max_brake=6.0,
    comfort_brake=3.0,
)

other = VehicleOnPath(
    obstacle_id=2,
    is_ego=False,
    route_id="south_north",
    incoming_id="south",
    ref_path=south_north,
    s0=2.0,
    v0=7.0,
    a_obs=-0.5,
    length=4.5,
    width=2.0,
    desired_speed=8.0,
    max_accel=2.0,
    max_brake=6.0,
    comfort_brake=3.0,
)

vehicles = [ego, other]
conflicts = build_conflict_graph(vehicles)

memory = PlannerMemory()
ego_a0, solution = run_mpc_cycle(
    vehicles,
    memory,
    known_conflicts=conflicts,
    mpc_config=mpc_config,
    weights=weights,
    config={
        "belief": config.belief,
        "branching": branching,
    },
)

print("planner mode:", memory.planner_mode)
print("ego first acceleration:", ego_a0)
print("solver status:", solution.status)
```

Save this as a scratch script or paste it into:

```bash
uv run --python /opt/homebrew/bin/python3.11 python
```

## 13. Closed-Loop Simulation

File: `src/rcbranch/evaluation/closed_loop_runner.py`

The simple closed-loop runner repeatedly calls `run_mpc_cycle`, executes only ego's first acceleration, and advances all vehicles with the longitudinal model:

```python
from rcbranch.evaluation.closed_loop_runner import run_simple_closed_loop

trace = run_simple_closed_loop(vehicles, cycles=10, dt=0.2)
print(trace.ego_accel)
print(trace.planner_mode)
```

This runner is intentionally minimal. It is useful for quick synthetic checks, but real CommonRoad evaluation should use proper scenario loading, trajectory export, and validation.

## 14. Metrics and Validation

File: `src/rcbranch/evaluation/metrics.py`

Compute basic longitudinal metrics:

```python
from rcbranch.evaluation.metrics import compute_longitudinal_metrics

metrics = compute_longitudinal_metrics(
    s=solution.s[solution.ego_id],
    v=solution.v[solution.ego_id],
    a=solution.a[solution.ego_id],
    dt=0.2,
    goal_s=40.0,
)
```

Metrics currently include:

- time to goal,
- trajectory length,
- comfort cost,
- jerk cost,
- unnecessary stop count.

File: `src/rcbranch/evaluation/commonroad_validation.py`

CommonRoad drivability validation is represented as a dependency hook for now:

```python
from rcbranch.evaluation.commonroad_validation import validation_available

print(validation_available())
```

The validation module is where collision, road-compliance, and dynamic-feasibility checks should be wired once full CommonRoad trajectories are exported.

## 15. Visualizing Traffic Through The Intersection

File: `src/rcbranch/evaluation/traffic_visualization.py`

The framework includes an interactive matplotlib visualizer for path-coordinate trajectories. It draws:

- reference paths,
- conflict regions,
- vehicle rectangles with heading,
- short trajectory tails,
- labels and speeds when available,
- a time seekbar,
- play/pause and one-step controls.

Launch the built-in demo:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-demo-traffic
```

Save a frame without opening the GUI:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-demo-traffic --no-show --save-frame traffic_demo.png --frame-time 4.5
```

Use it from Python:

```python
import numpy as np

from rcbranch.evaluation.traffic_visualization import (
    VehicleTrajectory,
    conflict_graph_from_trajectories,
    visualize_traffic_scene,
)
from rcbranch.geometry import ReferencePath

times = np.linspace(0.0, 8.0, 81)
east_west = ReferencePath.from_xy([[-20.0, 0.0], [20.0, 0.0]])
south_north = ReferencePath.from_xy([[0.0, -20.0], [0.0, 20.0]])

trajectories = [
    VehicleTrajectory(1, east_west, s=4.0 + 4.0 * times, times=times, label="ego"),
    VehicleTrajectory(2, south_north, s=2.0 + 3.5 * times, times=times, label="other"),
]

conflicts = conflict_graph_from_trajectories(trajectories)
visualize_traffic_scene(trajectories, conflicts=conflicts)
```

To visualize an MPC solution, convert it to trajectories:

```python
from rcbranch.evaluation.traffic_visualization import trajectories_from_mpc_solution

trajectories = trajectories_from_mpc_solution(solution, vehicles, dt=0.2)
visualize_traffic_scene(trajectories, conflicts=conflicts)
```

## 16. Baselines

Baseline wrappers live under `src/rcbranch/planners`.

Current files:

- `conservative_rc_mpc.py`: always use reciprocal-caution MPC.
- `deterministic_mpc.py`: choose the most likely crossing order from belief.
- `fixed_time_branch_mpc.py`: branch at a configured fixed horizon step.
- `ttc_branch_mpc.py`: trigger from time-to-conflict gap.
- `raw_collision_dual_branch_mpc.py`: score branches from externally supplied raw collision duals.
- `proposed_dual_priced_branching.py`: proposed ambiguity times reciprocal-caution dual trigger.

The design rule is that baselines should share:

- same scenario adapter,
- same reference paths,
- same conflict extraction,
- same active-set selector,
- same horizon,
- same solver budget,
- same metrics.

Only the branching or planner decision logic should differ.

## 17. Scripts

Installed entry points are declared in `pyproject.toml`:

```text
rcbranch-run-one
rcbranch-run-benchmark
```

Root-level wrappers are also available under `scripts/`.

Inspect one CommonRoad XML scenario:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-run-one path/to/scenario.xml
```

List configured benchmark scenarios:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-run-benchmark
```

Generate starter synthetic-suite metadata:

```bash
uv run --python /opt/homebrew/bin/python3.11 python scripts/generate_intersection_suite.py
```

Launch the interactive traffic visualization demo:

```bash
uv run --python /opt/homebrew/bin/python3.11 rcbranch-demo-traffic
```

## 18. Tests

The tests are small and component-focused.

```text
tests/test_projection.py
  ReferencePath projection and interpolation.

tests/test_conflict_regions.py
  Shapely conflict extraction on a handmade crossroad.

tests/test_casadi_toy_multiplier.py
  Multiplier sanity check for a one-variable constrained optimization.

tests/test_belief_filter.py
  Belief update reacts to yielding/go-first evidence.

tests/test_branch_trigger.py
  Branch score is high only when ambiguity and dual prices are high.
```

Run all tests:

```bash
uv run --python /opt/homebrew/bin/python3.11 pytest
```

Run one file:

```bash
uv run --python /opt/homebrew/bin/python3.11 pytest tests/test_branch_trigger.py
```

## 19. Debugging Guide

Use this map when a planner result looks wrong.

### Projection Looks Wrong

Check:

- `ReferencePath.from_xy(...)` point order,
- duplicate points,
- `project_xy_to_s(...)`,
- `interpolate_xytheta(...)`.

Relevant test:

```bash
pytest tests/test_projection.py
```

### Conflict Region Missing

Check:

- vehicle widths,
- conflict margin,
- path geometry,
- `sample_step`.

Relevant module:

```text
src/rcbranch/geometry/conflict_regions.py
```

Relevant test:

```bash
pytest tests/test_conflict_regions.py
```

### Belief Does Not Match Behavior

Check:

- constant-velocity entry/exit times,
- observed accelerations `a_obs`,
- belief temperature,
- prototype-cost weights,
- transition persistence.

Relevant modules:

```text
src/rcbranch/belief/features.py
src/rcbranch/belief/crossing_order_filter.py
```

### MPC Solve Is Infeasible Or Slow

Check:

- `horizon_steps`,
- acceleration and velocity bounds,
- comfort braking,
- conflict entry distances,
- slack weight,
- IPOPT iteration limit.

Relevant module:

```text
src/rcbranch/mpc/reciprocal_caution_mpc.py
```

### Branching Does Not Trigger

Check:

- `belief.ambiguity`,
- reciprocal-caution dual magnitudes,
- `tau_Psi`,
- `tau_psi`,
- EMA smoothing alpha.

Relevant modules:

```text
src/rcbranch/mpc/branching.py
src/rcbranch/mpc/duals.py
```

### Branch MPC Falls Back

The current branch MPC needs the reciprocal-caution warm-start trajectory to cross event intervals for conflict entry and exit. If it cannot locate those intervals, it raises `ValueError`, and the planner falls back to reciprocal-caution MPC for that cycle.

Check:

- whether the warm-start reaches conflict entry/exit within the horizon,
- whether the horizon is too short,
- whether the conflict region is too far ahead,
- whether the event interval target `s` is reasonable.

Relevant module:

```text
src/rcbranch/mpc/branch_mpc.py
```

## 20. Extending The Framework

### Add A New Belief Model

Keep the output contract:

```text
PairBelief(p_i_first, p_j_first, p_unresolved)
```

Then update or replace:

```text
src/rcbranch/belief/crossing_order_filter.py
```

Keep `ambiguity` compatible with branch scoring.

### Add A New Branch Trigger

Add a planner or scoring module under:

```text
src/rcbranch/planners/
```

Prefer reusing:

```text
src/rcbranch/mpc/branching.py
src/rcbranch/mpc/duals.py
```

Then add a test that proves the trigger is high and low in the right situations.

### Add Full CommonRoad Closed-Loop Evaluation

Likely modules to extend:

```text
src/rcbranch/commonroad_adapter/route_extraction.py
src/rcbranch/commonroad_adapter/solution_writer.py
src/rcbranch/evaluation/commonroad_validation.py
src/rcbranch/evaluation/closed_loop_runner.py
```

The expected flow is:

1. Load `Scenario` and `PlanningProblem`.
2. Extract ego route and obstacle routes.
3. Convert states to `VehicleOnPath`.
4. Run `run_mpc_cycle`.
5. Convert executed ego state back to CommonRoad `State`.
6. Append trajectory.
7. Validate with drivability checker.
8. Compute metrics.

### Add Synthetic Scenarios

Start from:

```bash
uv run --python /opt/homebrew/bin/python3.11 python scripts/generate_intersection_suite.py
```

Then implement concrete synthetic scenario construction around:

```text
ReferencePath
VehicleOnPath
build_conflict_graph
run_mpc_cycle
```

Synthetic scenarios are the best place to tune `tau_Psi` and belief parameters before running public CommonRoad scenarios.

## 21. Current Limitations

This is a first implementation of the research framework, not a full competition-ready CommonRoad planner.

Current limitations:

- fixed-path longitudinal dynamics only,
- no full single-track steering model,
- no route negotiation,
- no lane changing or overtaking,
- branch MPC uses fixed event-index interpolation,
- CommonRoad validation/export hooks are scaffolded but not yet a full benchmark pipeline,
- follower car-following constraints are not fully implemented yet.

These limitations are deliberate. The current goal is to make the reciprocal-caution dual-price branching idea testable and modular.

## 22. Recommended Reading Order

For a new developer:

1. Read `configs/default.yaml`.
2. Read `src/rcbranch/geometry/reference_path.py`.
3. Read `src/rcbranch/geometry/conflict_regions.py`.
4. Read `src/rcbranch/belief/crossing_order_filter.py`.
5. Read `src/rcbranch/mpc/nlp_builder.py`.
6. Read `src/rcbranch/mpc/reciprocal_caution_mpc.py`.
7. Read `src/rcbranch/mpc/branching.py`.
8. Read `src/rcbranch/mpc/branch_mpc.py`.
9. Read `src/rcbranch/planners/proposed_dual_priced_branching.py`.
10. Run the tests and inspect failures before making changes.

This order follows the runtime pipeline and avoids starting with the highest-level planner before understanding the data it consumes.
