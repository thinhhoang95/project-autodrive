# Implementation Plan for CommonRoad 

Your research idea is well matched to CommonRoad because the first paper version is deliberately narrow: fixed paths, no lane changes or overtaking, active interaction around an unsignalized crossroad, and branching only over local crossing order. The uploaded note defines the core loop as: identify front vehicles, build the conflict graph, update pairwise crossing-order beliefs, solve reciprocal-caution MPC, extract multipliers, compute branch scores, and solve a two-branch MPC only when a score exceeds threshold. 

---

## 1. Overall implementation target

Build a **CommonRoad-compatible planner wrapper** with this structure:

```text
CommonRoad scenario
   ↓
scenario adapter: ego, obstacles, lanelets, routes
   ↓
fixed-path longitudinal abstraction: s, v, a for active vehicles
   ↓
conflict graph at uncontrolled intersection
   ↓
pairwise crossing-order belief update
   ↓
CasADi reciprocal-caution MPC
   ↓
extract reciprocal-caution dual prices
   ↓
branch trigger Ψij = ambiguity × dual price
   ↓
optional CasADi branch MPC with two crossing orders
   ↓
execute first ego action
   ↓
convert ego trajectory back to CommonRoad states
   ↓
validate and compare baselines
```

CommonRoad scenarios consist of a `Scenario` and a `PlanningProblemSet`; the `Scenario` contains the lanelet network and static/dynamic obstacles, while each planning problem gives an ego initial state and goal region. ([GitLab][1]) The `CommonRoadFileReader` can load scenario and planning-problem data, including a `lanelet_assignment` option for obstacle-lanelet assignment. ([GitLab][2]) CasADi is appropriate for this because it is a Python-accessible framework for nonlinear optimization and optimal control rather than a black-box OCP solver, which matches the need to manually build MPC constraints and retrieve multipliers. ([CasADi][3])

---

## 2. Recommended repository layout

```text
rc_branching_commonroad/
  configs/
    default.yaml
    scenarios_uncontrolled_intersections.yaml
    baselines.yaml
    weights.yaml

  rcbranch/
    commonroad_adapter/
      load.py
      route_extraction.py
      state_conversion.py
      solution_writer.py

    geometry/
      reference_path.py
      projection.py
      conflict_regions.py
      active_set.py

    belief/
      crossing_order_filter.py
      features.py

    mpc/
      nlp_builder.py
      reciprocal_caution_mpc.py
      branch_mpc.py
      constraints.py
      duals.py
      warm_start.py

    planners/
      proposed_dual_priced_branching.py
      conservative_rc_mpc.py
      deterministic_mpc.py
      fixed_time_branch_mpc.py
      ttc_branch_mpc.py
      raw_collision_dual_branch_mpc.py

    evaluation/
      closed_loop_runner.py
      metrics.py
      commonroad_validation.py
      plots.py

  scripts/
    run_one_scenario.py
    run_benchmark.py
    generate_intersection_suite.py
    export_commonroad_solutions.py

  tests/
    test_projection.py
    test_conflict_regions.py
    test_casadi_toy_multiplier.py
    test_branch_trigger.py
```

The important design rule is that **all baselines use the same CommonRoad adapter, same fixed-path projection, same active-set selector, same prediction horizon, same solver budget, and same evaluation code**. Only the planner logic changes.

---

## 3. Environment and packages

Use a reproducible Python environment, preferably Python 3.11 or 3.12.

```bash
conda create -n rcbranch python=3.11
conda activate rcbranch

pip install \
  commonroad-io \
  commonroad-route-planner \
  commonroad-drivability-checker \
  commonroad-clcs \
  commonroad-scenario-designer \
  casadi \
  shapely \
  numpy scipy pandas matplotlib \
  pyyaml hydra-core tqdm
```

`commonroad-io` is the base package for reading, writing, and visualizing CommonRoad scenarios and planning problems; the current README states it is tested on Python 3.9 through 3.13. ([GitHub][4]) The route planner is useful for generating route and reference-path information on CommonRoad lanelet networks. ([GitHub][5]) The drivability checker is useful for collision, kinematic-feasibility, and road-compliance validation; note that recent versions moved curvilinear-coordinate functionality into `commonroad-clcs`, so installing it explicitly avoids import problems. ([PyPI][6])

---

## 4. CommonRoad scenario handling

### 4.1 Load scenario and planning problem

Start with the standard CommonRoad I/O path:

```python
from commonroad.common.file_reader import CommonRoadFileReader

def load_commonroad_problem(xml_path: str):
    scenario, planning_problem_set = CommonRoadFileReader(xml_path).open(
        lanelet_assignment=True
    )
    planning_problem = next(iter(planning_problem_set.planning_problem_dict.values()))
    return scenario, planning_problem
```

For the uncontrolled-intersection study, make a scenario filter:

```python
def is_uncontrolled_intersection_scenario(scenario) -> bool:
    ln = scenario.lanelet_network
    has_intersection = len(getattr(ln, "intersections", [])) > 0
    has_traffic_lights = len(getattr(ln, "traffic_lights", [])) > 0
    return has_intersection and not has_traffic_lights
```

In practice, do not rely only on the presence or absence of traffic-light objects. For each selected scenario, store a small metadata file:

```yaml
scenario_id: USA_Example-1_1_T-1
intersection_type: four_way
control_type: uncontrolled
traffic_lights_used: false
stop_signs_used: false
ego_route_known: true
route_uncertainty_used: false
```

This matters because your paper’s first version should **not** claim general route negotiation; it should focus on local crossing-order uncertainty. Your note explicitly frames the first study around fixed lane-level paths and excludes overtaking, lane changing, pass-over maneuvers, and arbitrary route negotiation. 

### 4.2 Scenario sources

Use two scenario pools.

First, use **existing CommonRoad intersection scenarios** for compatibility and public reproducibility. The CommonRoad benchmark format is designed for reproducible motion-planning comparisons, and competition-style evaluation considers safety, efficiency, comfort, and traffic-rule compliance. ([arXiv][7])

Second, create a **controlled synthetic uncontrolled-intersection suite** for the ablation study. Use `commonroad-scenario-designer` or programmatic CommonRoad map/scenario generation. The Scenario Designer supports creating/manipulating CommonRoad maps and converting formats such as Lanelet/Lanelet2, OpenDRIVE, OSM, and SUMO to CommonRoad; it also provides a GUI, CLI, and Python APIs. ([commonroad-scenario-designer.readthedocs.io][8])

Your first synthetic suite should mirror the uploaded experimental plan:

1. two-car crossroad,
2. four-approach crossroad with one front vehicle per approach,
3. queue scenario with followers,
4. ambiguous-yield scenario,
5. aggressive-other scenario. 

---

## 5. Fixed-path abstraction

For this research idea, do **not** optimize in global Cartesian coordinates initially. Convert each relevant vehicle to a path coordinate:

[
x_i = (s_i, v_i), \qquad u_i = a_i.
]

Your note already uses this longitudinal model:

[
s_{i,k+1}=s_{i,k}+\Delta t v_{i,k}+\frac{1}{2}\Delta t^2 a_{i,k},
]

[
v_{i,k+1}=v_{i,k}+\Delta t a_{i,k}.
]

This is justified by the first-study assumptions: fixed path, no overtaking, no lane change. 

Create a reference-path object:

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class ReferencePath:
    xy: np.ndarray          # shape (M, 2)
    s: np.ndarray           # arc length, shape (M,)
    theta: np.ndarray       # tangent heading, shape (M,)
    kappa: np.ndarray       # curvature, shape (M,)

    def project_xy_to_s(self, xy: np.ndarray) -> float:
        # nearest-segment projection; return arc length
        ...

    def interpolate_xytheta(self, s_query: float):
        # return x, y, theta, kappa
        ...
```

For ego, get the route from the planning problem. For other vehicles, infer the most likely route from their current lanelet assignment and short provided trajectory/prediction. In synthetic scenarios, store the route explicitly.

---

## 6. Active-set selection at the intersection

At each MPC cycle, build the active strategic set:

[
\mathcal A_t = {\text{front vehicle of each incoming approach}}.
]

Followers should be modeled by car-following constraints but should not introduce new crossing-order branches. Your note makes exactly this reduction: same-lane followers cannot cross before their leaders, so the strategic crossing-order set is often just the front vehicle of each incoming lane. 

Implementation:

```python
@dataclass
class VehicleOnPath:
    obstacle_id: int
    is_ego: bool
    route_id: str
    incoming_id: str
    ref_path: ReferencePath
    s0: float
    v0: float
    a_obs: float
    length: float
    width: float
    desired_speed: float
    max_accel: float
    max_brake: float
    comfort_brake: float
```

```python
def select_front_vehicles_by_incoming(vehicles, conflicts):
    groups = {}
    for veh in vehicles:
        # distance to first relevant conflict entry
        d_conf = min_conflict_distance_ahead(veh, conflicts)
        if d_conf is None:
            continue
        key = veh.incoming_id
        if key not in groups or d_conf < groups[key][1]:
            groups[key] = (veh, d_conf)
    return [veh for veh, _ in groups.values()]
```

For queues, keep followers in the prediction model only if they can affect the front vehicle through car-following. Otherwise, ignore them for the first implementation.

---

## 7. Conflict-region extraction

For each pair of active paths, compute a route-level conflict region.

A robust implementation uses Shapely:

```python
from shapely.geometry import LineString

def path_tube(ref_path: ReferencePath, half_width: float, margin: float):
    return LineString(ref_path.xy).buffer(half_width + margin, cap_style=2, join_style=2)

@dataclass
class ConflictRegion:
    i: int
    j: int
    s_i_in: float
    s_i_out: float
    s_j_in: float
    s_j_out: float
    polygon: object
```

Algorithm:

1. Build a tube polygon around path (i).
2. Build a tube polygon around path (j).
3. Intersect the two polygons.
4. If empty, no conflict.
5. Sample each path at high resolution and find samples inside the intersection polygon.
6. The first/last included samples give (s_i^\text{in}, s_i^\text{out}, s_j^\text{in}, s_j^\text{out}).

This is simpler and more portable than relying on special intersection metadata because it works for recorded, synthetic, converted, and hand-edited CommonRoad scenarios.

---

## 8. Crossing-order belief

Maintain, for each conflicting pair ((i,j)),

[
P(i \succ j), \quad P(j \succ i), \quad P(\bot).
]

Use the ambiguity score from the note:

[
A_{ij} = 1 - |P(i \succ j)-P(j \succ i)|.
]

This is better than entropy alone because (P(\bot)=1) can have low entropy while the crossing order is still unresolved. 

A practical belief filter:

```python
@dataclass
class PairBelief:
    p_i_first: float
    p_j_first: float
    p_unresolved: float
    stable_count_i_first: int = 0
    stable_count_j_first: int = 0
```

Feature vector for pair ((i,j)):

```python
@dataclass
class PairFeatures:
    t_i_in_cv: float
    t_i_out_cv: float
    t_j_in_cv: float
    t_j_out_cv: float
    b_i_req_stop: float
    b_j_req_stop: float
    a_i_obs: float
    a_j_obs: float
    gap_if_i_first: float
    gap_if_j_first: float
```

Prototype-cost likelihood:

[
C_{i\succ j}
============

w_t , \max(0, T_i^\text{out}+\Delta_\text{safe}-T_j^\text{in})^2
+
w_{a,j}(a_j^\text{obs}-a_j^\text{yield})^2
+
w_{a,i}(a_i^\text{obs}-a_i^\text{go})^2.
]

[
C_{j\succ i}
============

w_t , \max(0, T_j^\text{out}+\Delta_\text{safe}-T_i^\text{in})^2
+
w_{a,i}(a_i^\text{obs}-a_i^\text{yield})^2
+
w_{a,j}(a_j^\text{obs}-a_j^\text{go})^2.
]

[
C_{\bot}
========

w_{a,i}(a_i^\text{obs}-a_i^\text{cautious})^2
+
w_{a,j}(a_j^\text{obs}-a_j^\text{cautious})^2
+
w_\text{near}|T_i^\text{in}-T_j^\text{in}|^{-1}.
]

Then:

```python
def softmax_negative_cost(costs, temp=1.0):
    z = np.exp(-np.array(costs) / temp)
    return z / np.sum(z)
```

Add a temporal filter:

[
P_t(r) \propto P(y_t\mid r)\sum_{r'}P(r\mid r')P_{t-1}(r').
]

For the first version, keep this interpretable; do not train a neural predictor unless you later run a separate prediction paper.

---

## 9. Reciprocal-caution MPC in CasADi

### 9.1 Decision variables

For active vehicles (i\in\mathcal A_t):

[
S_i = [s_{i,0},\ldots,s_{i,N}],
]

[
V_i = [v_{i,0},\ldots,v_{i,N}],
]

[
A_i = [a_{i,0},\ldots,a_{i,N-1}].
]

For every unresolved conflicting ordered pair (i\leftarrow j), add slack:

[
\xi_{i\leftarrow j,k}\ge 0.
]

### 9.2 Objective

Use a social-prediction MPC objective, but only execute ego’s first control:

[
J =
\sum_i
\sum_{k=0}^{N-1}
w_v(v_{i,k}-v_i^\text{des})^2
+
w_a a_{i,k}^2
+
w_j(a_{i,k}-a_{i,k-1})^2
+
\sum \rho_\xi \xi_{i\leftarrow j,k}^2.
]

For non-ego vehicles, this objective is not claiming centralized control. It is a structured prediction model: “what would a reciprocal-cautious vehicle plausibly do?”

### 9.3 Constraints

Dynamics:

[
s_{i,k+1}=s_{i,k}+\Delta t v_{i,k}+\frac12\Delta t^2a_{i,k},
]

[
v_{i,k+1}=v_{i,k}+\Delta t a_{i,k}.
]

Bounds:

[
0\le v_{i,k}\le v_i^\max,
]

[
-b_i^\max\le a_{i,k}\le a_i^{\max,+}.
]

Car-following, for followers (q) behind leader (p):

[
s_{q,k}\le s_{p,k}-d_0-T_hv_{q,k}.
]

Reciprocal-caution constraint:

[
h^\bot_{i\leftarrow j,k}
========================

\frac{v_{i,k}^2}{2b_i^\text{comf}}
+
d_\text{buf}
------------

(s_i^\text{in}(ij)-s_{i,k})
\le \xi_{i\leftarrow j,k}.
]

This is the key constraint from your note: while order is unresolved, vehicle (i) should not lose the comfortable option to yield before conflict entry; the symmetric constraint is also imposed for (j\leftarrow i). 

### 9.4 Practical gating of caution constraints

Do **not** blindly impose the caution constraint after a vehicle has already entered the conflict region. Use one of these practical gates:

**Version A, easiest and robust:** impose caution only for indices that are predicted, under the previous solution or constant-velocity rollout, to satisfy

[
s_{i,k}^{\text{ref}} < s_i^\text{in}-\epsilon.
]

Rebuild this active constraint set every MPC cycle.

**Version B, smoother but more nonlinear:** multiply the constraint by a smooth gate

[
\gamma(s)=\sigma(\beta(s_i^\text{in}-s_i)),
]

and impose

[
\gamma(s_{i,k})h^\bot_{i\leftarrow j,k}\le \xi_{i\leftarrow j,k}.
]

I recommend Version A for the first implementation because it produces cleaner dual-variable interpretation.

---

## 10. CasADi implementation pattern

Use the lower-level `nlpsol` interface rather than relying entirely on `Opti` objects. The reason is auditability: you need a reliable mapping from each reciprocal-caution constraint to its Lagrange multiplier. CasADi’s `Opti` interface exposes `lam_g`, and the docs show `opti.lam_g` as the Lagrange multiplier vector associated with `opti.g`; however, for this research code it is cleaner to track the constraint vector manually. ([CasADi][9])

Minimal pattern:

```python
import casadi as ca
import numpy as np
from dataclasses import dataclass

@dataclass
class ConstraintMeta:
    kind: str
    i: int | None = None
    j: int | None = None
    k: int | None = None

class NLPBuilder:
    def __init__(self):
        self.w = []
        self.w0 = []
        self.lbw = []
        self.ubw = []

        self.g = []
        self.lbg = []
        self.ubg = []
        self.meta = []

        self.J = 0

    def add_var(self, name, n, lb=-ca.inf, ub=ca.inf, init=0.0):
        x = ca.MX.sym(name, n)
        self.w.append(x)
        self.lbw.extend(np.full(n, lb))
        self.ubw.extend(np.full(n, ub))
        self.w0.extend(np.full(n, init))
        return x

    def add_con(self, expr, lb, ub, meta: ConstraintMeta):
        expr = ca.reshape(expr, -1, 1)
        for r in range(expr.shape[0]):
            self.g.append(expr[r])
            self.lbg.append(lb if np.isscalar(lb) else lb[r])
            self.ubg.append(ub if np.isscalar(ub) else ub[r])
            self.meta.append(meta)

    def build_solver(self):
        w = ca.vertcat(*self.w)
        g = ca.vertcat(*self.g)
        nlp = {"x": w, "f": self.J, "g": g}
        opts = {
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.max_iter": 100,
            "ipopt.tol": 1e-5,
        }
        solver = ca.nlpsol("solver", "ipopt", nlp, opts)
        return solver, w, g
```

When adding a reciprocal-caution constraint:

```python
h = v_i[k]**2 / (2.0 * b_comf_i) + d_buf - (s_in_i_j - s_i[k])

# h <= xi  ⇔  h - xi <= 0
builder.add_con(
    h - xi_i_j[k],
    lb=-ca.inf,
    ub=0.0,
    meta=ConstraintMeta(kind="reciprocal_caution", i=i, j=j, k=k),
)
```

After solve:

```python
sol = solver(
    x0=np.array(builder.w0),
    lbx=np.array(builder.lbw),
    ubx=np.array(builder.ubw),
    lbg=np.array(builder.lbg),
    ubg=np.array(builder.ubg),
)

lam_g = np.array(sol["lam_g"]).reshape(-1)

lambda_caution = {}
for idx, meta in enumerate(builder.meta):
    if meta.kind == "reciprocal_caution":
        # Use magnitude unless you have verified the solver sign convention
        # with the toy KKT test.
        lambda_caution[(meta.i, meta.j, meta.k)] = abs(lam_g[idx])
```

Add a unit test based on your toy example:

[
\min_v \frac12(v-10)^2
\quad \text{s.t.}\quad
v-v_\text{safe}\le 0.
]

The expected multiplier magnitude is:

[
\lambda^\star=\max(0,10-v_\text{safe}).
]

Your note uses exactly this example to interpret the multiplier as the price of keeping the cautious yield option alive. 

---

## 11. Branch score

For each unresolved conflict pair:

[
\Psi_{ij}
=========

A_{ij}
\sum_{k=0}^{N}
\left(
\tilde{\lambda}^\bot_{i\leftarrow j,k}
+
\tilde{\lambda}^\bot_{j\leftarrow i,k}
\right)\Delta t.
]

Use multiplier normalization and smoothing:

```python
def normalize_lambda(raw_lam, grad_norm, j_scale, eps=1e-6):
    return raw_lam * grad_norm / (j_scale + eps)

def ema(prev, current, alpha=0.7):
    return alpha * prev + (1.0 - alpha) * current
```

The note explicitly warns that raw nonlinear-MPC multipliers can be noisy and should be normalized and filtered rather than thresholded directly. 

Branch if:

[
\Psi_{ij}>\tau_\Psi.
]

For branch time:

[
\psi_{ij,k}
===========

A_{ij}
(\tilde\lambda^\bot_{i\leftarrow j,k}
+
\tilde\lambda^\bot_{j\leftarrow i,k}),
]

[
k_b = \min{k:\psi_{ij,k}>\tau_\psi}.
]

Also keep a latest-feasible guard so that the planner does not wait until one crossing order is already impossible. 

---

## 12. Branch MPC

When pair ((i,j)) is selected, build two branches:

[
b=1: i\succ j,
]

[
b=2: j\succ i.
]

The objective is expected cost:

[
J_\text{branch}
===============

P(i\succ j)J^{i\succ j}
+
P(j\succ i)J^{j\succ i}.
]

Before the branch time, impose non-anticipativity:

[
a^{i\succ j}_{q,k}
==================

a^{j\succ i}_{q,k},
\qquad
k < k_b.
]

In implementation, always force the **current ego action** to be shared:

[
a^{i\succ j}_{\text{ego},0}
===========================

a^{j\succ i}_{\text{ego},0}.
]

This prevents the planner from executing a branch-specific action before the uncertainty has actually resolved. Your note defines the branch MPC as a shared cautious trunk followed by two crossing-order continuations, with non-anticipativity before (k_b). 

### 12.1 Encoding crossing-order constraints

For branch (i\succ j), impose:

[
T_j^\text{in}
\ge
T_i^\text{out}
+
\Delta_\text{safe}.
]

For branch (j\succ i), impose:

[
T_i^\text{in}
\ge
T_j^\text{out}
+
\Delta_\text{safe}.
]

The cleanest first implementation is to use **fixed event-index interpolation**:

1. Use the reciprocal-caution solution or constant-velocity rollout to estimate the crossing index (m_i^\text{in}), (m_i^\text{out}), (m_j^\text{in}), (m_j^\text{out}).
2. Introduce event-time variables (T_i^\text{in},T_i^\text{out},T_j^\text{in},T_j^\text{out}).
3. Constrain each event to lie in its fixed interval:

[
t_m \le T_i^\text{in}\le t_{m+1}.
]

4. Interpolate progress:

[
s_i(T)
======

s_{i,m}
+
\frac{T-t_m}{\Delta t}
(s_{i,m+1}-s_{i,m}).
]

5. Enforce:

[
s_i(T_i^\text{in})=s_i^\text{in},
\qquad
s_i(T_i^\text{out})=s_i^\text{out}.
]

This is not globally exact if the event index changes, but it is stable enough for a first paper implementation. If the solution violates the assumed event interval, rebuild once with updated event indices.

---

## 13. Closed-loop planner loop

```python
def run_mpc_cycle(world, memory, config):
    # 1. Convert CommonRoad snapshot to fixed-path states
    vehicles = build_vehicle_on_path_list(world)

    # 2. Select active front vehicles
    active = select_front_vehicles_by_incoming(vehicles, world.conflicts)

    # 3. Build conflict graph
    conflicts = build_conflict_graph(active)

    # 4. Update crossing-order beliefs
    beliefs = {}
    for c in conflicts:
        features = compute_pair_features(c, active, memory)
        beliefs[(c.i, c.j)] = memory.belief_filter.update(c.i, c.j, features)

    # 5. Solve reciprocal-caution MPC
    rc_sol = solve_reciprocal_caution_mpc(active, conflicts, beliefs, memory)

    # 6. Extract dual prices and compute branch scores
    scores = compute_branch_scores(rc_sol.duals, beliefs, conflicts, memory)

    # 7. Decide whether to branch
    best_pair, best_score = max(scores.items(), key=lambda kv: kv[1].Psi)

    if best_score.Psi <= config.tau_Psi:
        selected_sol = rc_sol
        planner_mode = "reciprocal_caution_only"
    else:
        kb = choose_branch_time(best_score, config)
        selected_sol = solve_branch_mpc(
            active=active,
            conflicts=conflicts,
            beliefs=beliefs,
            branch_pair=best_pair,
            kb=kb,
            warm_start=rc_sol,
        )
        planner_mode = "branch_mpc"

    # 8. Execute first ego action
    ego_a0 = selected_sol.ego_accel[0]

    # 9. Store diagnostics
    memory.update(
        rc_sol=rc_sol,
        selected_sol=selected_sol,
        beliefs=beliefs,
        scores=scores,
        planner_mode=planner_mode,
    )

    return ego_a0, selected_sol
```

Then update ego state:

[
s_{t+1}=s_t+\Delta t v_t+\frac12\Delta t^2a_t,
]

[
v_{t+1}=v_t+\Delta t a_t.
]

Convert (s,v) back to a CommonRoad `State` by interpolating (x,y,\theta,\kappa) from the reference path.

---

## 14. CommonRoad trajectory output

For each MPC cycle, append the executed ego state:

```python
def sv_to_commonroad_state(s, v, a, ref_path, time_step):
    x, y, theta, kappa = ref_path.interpolate_xytheta(s)
    return State(
        time_step=time_step,
        position=np.array([x, y]),
        orientation=theta,
        velocity=v,
        acceleration=a,
    )
```

For a CommonRoad-style benchmark result, export the full ego trajectory and run validation. CommonRoad’s drivability checker is designed to simplify collision avoidance, kinematic feasibility, and road-compliance checks for planned motions. ([PyPI][6])

For strict CommonRoad competition-style evaluation, you may eventually need a kinematic single-track trajectory with steering state. The 2024 competition report describes feasible trajectories using a kinematic single-track state containing Cartesian position, velocity, heading, and steering angle, and evaluates collision-free behavior, safety, comfort, efficiency, and rule compliance. ([arXiv][7]) For the first research implementation, the longitudinal fixed-path model is acceptable, but the paper should state that this is a fixed-route intersection planner rather than a full vehicle-dynamics submission planner.

---

## 15. Baselines to implement

Use the same CommonRoad wrapper and CasADi infrastructure for every baseline.

### Baseline A: deterministic most-likely-order MPC

Pick:

[
r^\star=\arg\max{P(i\succ j),P(j\succ i)}.
]

Then solve a single MPC with only that crossing-order constraint.

Failure mode expected: unsafe or overconfident when beliefs are wrong.

### Baseline B: conservative reciprocal-caution MPC

Always preserve all unresolved yield options. Never branch.

Failure mode expected: unnecessary slowing/stopping and delay.

### Baseline C: fixed-time branch MPC

Always branch at a fixed predicted time, such as (k_b=3) or (k_b=5), whenever there is a conflict pair.

Failure mode expected: branches too early when ambiguity is cheap, or too late when ambiguity becomes expensive quickly.

### Baseline D: TTC-triggered branch MPC

Branch when time-to-collision or time-to-conflict gap falls below a threshold.

Failure mode expected: branches due to geometric closeness even if the other vehicle is clearly yielding.

### Baseline E: raw collision-dual branch MPC

Use multipliers from collision/conflict-avoidance constraints rather than reciprocal-caution constraints.

This is a key ablation because the paper claim is not “dual variables matter,” but rather “the dual variables of reciprocal-caution constraints price the cost of keeping crossing order unresolved.”

### Baseline F: proposed dual-priced reciprocal-caution branching

[
\Psi_{ij}
=========

A_{ij}
\sum_k
(\tilde\lambda_{i\leftarrow j,k}^\bot
+
\tilde\lambda_{j\leftarrow i,k}^\bot)\Delta t.
]

Your note lists essentially these baselines and metrics, including deterministic MPC, conservative MPC, fixed-time branching, TTC-triggered branching, raw collision-multiplier branching, and the proposed reciprocal-caution multiplier trigger. 

---

## 16. Metrics

Log both **CommonRoad-compatible metrics** and **research-specific branching metrics**.

CommonRoad-compatible:

```text
success / goal reached
collision rate
road-boundary violation
dynamic feasibility
time to goal
trajectory length
comfort cost
jerk cost
solver time
```

Research-specific:

```text
minimum conflict time margin
minimum conflict spatial margin
average delay relative to free-flow
number of unnecessary stops
branch count
branch time relative to conflict entry
false branch rate
missed branch rate
belief accuracy against realized crossing order
mean Ψij at branch time
mean reciprocal-caution slack
mean raw λ and normalized λ
```

Define false/missed branches using ground-truth labels in synthetic scenarios:

```text
false branch:
  planner branches but one order already has high stable belief,
  or both branches are cheap and no safety/efficiency benefit occurs.

missed branch:
  planner does not branch before a conflict where deterministic or conservative
  behavior causes collision, hard braking, excessive delay, or infeasibility.
```

The uploaded note already highlights collision/conflict-region violation, delay, unnecessary stopping, speed loss, comfort/jerk, branch count, solver time, conflict-entry margin, false branch rate, and missed branch rate as core metrics. 

---

## 17. Suggested default parameters

Start with conservative values and calibrate on a validation set.

```yaml
mpc:
  dt: 0.2
  horizon_steps: 25        # 5 s horizon
  max_ipopt_iter: 100
  solver_tol: 1.0e-5

vehicle:
  v_max: 13.9              # 50 km/h
  a_max: 2.0
  b_max: 6.0
  b_comf: 3.0
  d_buf: 3.0
  d0_follow: 5.0
  time_headway: 1.2

objective:
  w_v: 1.0
  w_a: 0.2
  w_jerk: 1.0
  w_progress: 0.5
  w_slack_caution: 500.0
  w_slack_priority: 1000.0

belief:
  p_high: 0.85
  persistence_cycles: 3
  softmax_temp: 1.0

branching:
  tau_Psi: 1.0
  tau_psi: 0.3
  lambda_ema_alpha: 0.7
  max_branch_pairs: 1
  min_shared_controls: 1
  delta_safe_time: 0.7
```

Do not tune (\tau_\Psi) on the test set. Tune it on synthetic validation scenarios and then freeze it.

---

## 18. Experimental protocol

Use three evaluation tiers.

### Tier 1: toy and unit tests

Purpose: prove implementation correctness.

Tests:

```text
projection round-trip: xy → s → xy
conflict-region extraction on hand-made crossroad
toy multiplier equals max(0, v_des - v_safe)
belief update reacts correctly to yielding / going-first / unresolved behavior
branch score high only when A high and λ high
```

### Tier 2: synthetic uncontrolled-intersection suite

Purpose: isolate the research claim.

Generate scenarios with controlled parameters:

```text
arrival-time gap: [-2.0 s, +2.0 s]
initial speed: [3, 12] m/s
other-driver type: yielding / assertive / cautious / aggressive
number of approaches: 2 or 4
queue length: 0, 1, 2
sensor noise: none / mild / moderate
belief model error: none / biased / delayed
```

This tier gives clean ground truth for crossing order and false/missed branch labels.

### Tier 3: existing CommonRoad scenarios

Purpose: benchmark compatibility.

Run all baselines on the same set of public intersection scenarios. For existing non-interactive CommonRoad scenarios, treat other vehicles’ recorded trajectories as ground truth for evaluation, while your planner uses only its local prediction/belief model online. For interactive scenarios, later integrate through SUMO/CommonRoad interactive tooling; CommonRoad competition-style settings include both non-interactive scenarios with provided predictions and interactive scenarios using SUMO. ([arXiv][7])

---

## 19. Key implementation pitfalls

### Pitfall 1: confusing prediction control with actual control

You may optimize accelerations for other vehicles inside the reciprocal-caution MPC. That does **not** mean you control them. It is an internal structured prediction model. Only ego’s first action is executed.

### Pitfall 2: using comfortable stopping as safety proof

The reciprocal-caution constraint prices the cost of preserving a comfortable yield option. It is not a hard safety certificate. Your note explicitly says comfortable stopping is a planning preference, while hard safety should use maximum braking, reachable sets, or a safety filter. 

### Pitfall 3: applying caution constraints after conflict entry

The stopping-distance caution constraint is meaningful before entering the conflict region. Gate it by predicted pre-entry indices or a smooth approach-zone gate.

### Pitfall 4: branching too many pairs

In a four-way intersection, many pairwise conflicts may exist. For the first paper, branch only on the highest-score pair per MPC cycle. Keep all other unresolved pairs under reciprocal caution. This preserves the “small branch tree” claim.

### Pitfall 5: raw multipliers are not comparable across costs and units

Normalize, smooth, and log raw values. Run the same normalization for every baseline. The branch score should be calibrated using validation scenarios, not hand-picked per test case.

### Pitfall 6: CommonRoad feasibility vs longitudinal feasibility

A longitudinal path MPC can produce smooth (s,v,a) but still fail a full kinematic single-track feasibility check if the path curvature is high or steering-rate limits are violated. For the first study, keep intersection speeds modest and routes smooth. Later, add a tracking MPC or include steering dynamics.

---

## 20. Minimum viable implementation sequence

A realistic first implementation order:

1. **Load and visualize one CommonRoad intersection.** Confirm ego, obstacles, lanelets, planning problem, and time step.

2. **Build fixed-path projection.** Convert ego and one obstacle from CommonRoad states to (s,v,a), then back to CommonRoad states.

3. **Extract conflict region.** For two fixed routes, compute (s_i^\text{in/out}), (s_j^\text{in/out}), and visualize the conflict polygon.

4. **Implement the toy CasADi multiplier test.** Verify that the returned multiplier magnitude matches (\max(0,v_\text{des}-v_\text{safe})).

5. **Implement two-car reciprocal-caution MPC.** No branching yet. Log slack and multipliers.

6. **Implement belief update.** Start with softmax likelihood over (i\succ j), (j\succ i), and (\bot).

7. **Compute branch score.** Plot (A_{ij}), (\lambda), and (\Psi_{ij}) over time.

8. **Implement two-branch MPC for one pair.** Use event-time constraints and non-anticipativity.

9. **Run closed-loop simulation.** Execute only ego’s first acceleration and replan.

10. **Add baselines.** Reuse the same `NLPBuilder`, scenario adapter, and evaluation code.

11. **Generate synthetic uncontrolled-intersection suite.** Sweep initial gaps, speeds, and behavior types.

12. **Run public CommonRoad intersection scenarios.** Treat this as benchmark compatibility and external validation.

---

## 21. The paper-ready contribution framing

A precise implementation claim would be:

> We implement a CommonRoad-compatible fixed-path planner for uncontrolled intersections. The planner uses CasADi MPC to maintain reciprocal-caution constraints while crossing order is unresolved. The Lagrange multipliers of these constraints are extracted online and combined with pairwise crossing-order ambiguity to trigger a small two-branch contingency MPC only when keeping both yield options is expensive.

That matches the uploaded note’s intended claim: belief over crossing order + reciprocal-caution MPC + dual-priced branch trigger + small crossing-order tree. 

[1]: https://cps.pages.gitlab.lrz.de/commonroad/commonroad-io/user/getting_started.html "Getting Started — CommonRoad_io 2024.3 documentation"
[2]: https://cps.pages.gitlab.lrz.de/commonroad/commonroad-io/api/common.html "Module Common — CommonRoad_io 2024.3 documentation"
[3]: https://web.casadi.org/docs/ "CasADi - Docs"
[4]: https://github.com/CommonRoad/commonroad-io "GitHub - CommonRoad/commonroad-io: Tool to read, write, and visualize CommonRoad scenarios and base for other tools from the CommonRoad framework. · GitHub"
[5]: https://github.com/CommonRoad/commonroad-route-planner?utm_source=chatgpt.com "CommonRoad Route Planner"
[6]: https://pypi.org/project/commonroad-drivability-checker/ "commonroad-drivability-checker · PyPI"
[7]: https://arxiv.org/html/2512.19564v1 "Results of the 2024 CommonRoad Motion Planning Competition for Autonomous Vehicles"
[8]: https://commonroad-scenario-designer.readthedocs.io/ "CommonRoad Scenario Designer — CommonRoad Scenario Designer 0.5 documentation"
[9]: https://web.casadi.org/api/html/dd/dc6/classcasadi_1_1Opti.html "CasADi: casadi::Opti Class Reference"
