# Dual-Priced Reciprocal-Caution Branching for Unsignalized Crossroad MPC

*A tutorial and research plan*

Prepared for Thinh Hoang  
June 2026

> Source: uploaded tutorial PDF, cited as :contentReference[oaicite:0]{index=0}

## Abstract

This tutorial explains a refined research idea for contingency model predictive control (MPC) at an unsignalized crossroad. The setting is deliberately narrow: fixed lanes, no overtaking, no lane changing, no pass-over behavior, and vehicle interactions concentrated around the intersection.

Vehicles follow their leading vehicles, so the main strategic uncertainty is the crossing order of the front vehicles approaching the crossroad.

The central idea is to model unresolved interactions through **reciprocal caution**: while it is unclear who will cross first, each involved vehicle should preserve a safe yield option. The Lagrange multipliers of these reciprocal-caution constraints measure how costly it is to keep the crossing order unresolved. Multiplying this dual cost by an online ambiguity belief produces a branch score.

The planner branches only when unresolved crossing order is both **ambiguous** and **expensive**.

---

## 1. Executive Summary

The one-sentence version is:

> At an unsignalized crossroad, use a shared cautious trunk while crossing order is unclear; branch into candidate crossing orders only when the optimizer’s dual prices say that keeping the interaction unresolved has become expensive.

The research object is not a general autonomous-driving stack. It is a narrow planning mechanism for a crossroad:

> **Branch when reciprocal caution becomes expensive.**

The method has three ingredients.

1. **Belief over crossing order.**  
   For a conflicting pair $(i,j)$, maintain probabilities such as

   $$
   P(i \succ j), \qquad P(j \succ i),
   $$

   where $i \succ j$ means “vehicle $i$ crosses before vehicle $j$.”

2. **Reciprocal-caution constraints.**  
   While the crossing order is unresolved, vehicle $i$ must keep the option to yield to $j$, and vehicle $j$ must keep the option to yield to $i$. A simple version is a stopping-distance condition.

3. **Dual-priced branch score.**  
   Extract the Lagrange multipliers of the reciprocal-caution constraints. If the crossing order is ambiguous and the multiplier cost of reciprocal caution is high, create a small branch tree.

The branch tree is deliberately small:

$$
\text{shared cautious trunk}
\longrightarrow
\begin{cases}
i \succ j, & \text{vehicle } i \text{ crosses first}, \\
j \succ i, & \text{vehicle } j \text{ crosses first}.
\end{cases}
$$

The car still executes only one control action at the current time. Branching is an internal planning representation that keeps multiple future options alive.

---

## 2. Scenario and Assumptions

We study an unsignalized crossroad.

### 2.1 Included Behavior

The scene contains vehicles approaching and crossing the intersection. Each vehicle follows a fixed lane-level path through the crossroad. The main interaction is deciding who crosses first when paths conflict.

### 2.2 Excluded Behavior

The first study should not include:

- overtaking,
- lane changing,
- pass-over maneuvers,
- arbitrary route negotiation,
- general adversarial driving outside the intersection.

These exclusions are not weaknesses. They are what make the first version mathematically clean.

### 2.3 Car-Following Structure

Vehicles in the same lane follow their leaders. Therefore, a follower is not an independent crossing-order decision-maker. If vehicle $q$ follows vehicle $p$, then $q$ cannot cross before $p$.

A simple car-following constraint is

$$
s_{q,k} \leq s_{p,k} - d_0 - T_h v_{q,k},
$$

where $d_0$ is a standstill gap and $T_h$ is a time-headway parameter.

This means the active strategic set is small. At a four-way intersection, it is often enough to reason over the front vehicle on each incoming approach:

$$
\mathcal{A}_t
=
\{\text{front vehicle of each incoming lane}\}.
$$

---

## 3. Why Branching Is Needed

Consider ego vehicle $E$ and another vehicle $O$ approaching the same conflict region.

A single-trajectory planner must implicitly choose one story. It may assume that $O$ yields, so ego keeps going. This is efficient if true, but unsafe if false. Or it may assume that $O$ goes first, so ego slows or stops. This is safe but conservative if $O$ was actually yielding.

Branching avoids this premature commitment:

$$
\text{common action now}
\longrightarrow
\begin{cases}
E \succ O, & E \text{ proceeds and } O \text{ yields}, \\
O \succ E, & O \text{ proceeds and } E \text{ yields}.
\end{cases}
$$

The car still executes one action at the current time. The branches are possible future continuations inside the optimizer.

Thus:

> **Branching = delayed commitment under interaction uncertainty.**

Branching solves the tension between overconfidence and overconservatism:

- It avoids committing too early to “the other car yields.”
- It avoids always behaving as if “the other car never yields.”
- It preserves both responses until the crossing order becomes clearer or until preserving both responses becomes too costly.

---

## 4. The Key Behavioral Insight: Reciprocal Caution

The ego-centric view says:

> Ego does not know what the other driver will do.

The reciprocal view says:

> The other driver may also not know what ego will do. While the crossing order is unclear, both drivers may slow down or preserve their ability to yield.

This is the main modeling insight.

Unresolved crossroad interactions often produce a negotiation phase. Vehicles may creep, slow, hesitate, or maintain enough margin to stop. This is not generic Gaussian noise. It is structured behavior caused by unresolved priority.

Therefore, the first model should be:

$$
\text{unresolved crossing order}
\Longrightarrow
\text{reciprocal caution}.
$$

Before branching, both vehicles keep their options open. After branching, one crossing order is selected in each branch.

---

## 5. Notation

| Symbol | Meaning |
|---|---|
| $i,j$ | Vehicle indices. Usually $i$ and $j$ are a conflicting pair at the crossroad. |
| $k$ | Discrete future time index in the MPC horizon. |
| $\Delta t$ | Sampling time. |
| $s_{i,k}$ | Longitudinal position of vehicle $i$ along its fixed path. |
| $v_{i,k}$ | Speed of vehicle $i$. |
| $a_{i,k}$ | Acceleration command of vehicle $i$. |
| $x_{i,k}$ | State of vehicle $i$, usually $(s_{i,k}, v_{i,k})$. |
| $i \succ j$ | Vehicle $i$ crosses the conflict region before vehicle $j$. |
| $r_{ij}$ | Local crossing-order outcome for pair $(i,j)$. |
| $\bot$ | Unresolved crossing order. |
| $P(i \succ j)$ | Online belief that $i$ crosses before $j$. |
| $A_{ij}$ | Crossing-order ambiguity score. |
| $h^\bot_{i \leftarrow j,k}$ | Reciprocal-caution constraint: while unresolved, $i$ must preserve a safe yield option to $j$. |
| $\lambda^\bot_{i \leftarrow j,k}$ | Lagrange multiplier of $h^\bot_{i \leftarrow j,k}$. |
| $\Psi_{ij}$ | Branch score for pair $(i,j)$. |

---

## 6. Vehicle Dynamics

Because the first study excludes overtaking and lane changes, each vehicle can be modeled longitudinally along a fixed path.

Let

$$
x_{i,k} = (s_{i,k}, v_{i,k}).
$$

A simple discrete-time model is

$$
s_{i,k+1}
=
s_{i,k}
+
\Delta t\, v_{i,k}
+
\frac{1}{2}\Delta t^2 a_{i,k},
$$

$$
v_{i,k+1}
=
v_{i,k}
+
\Delta t\, a_{i,k}.
$$

Basic physical constraints are

$$
0 \leq v_{i,k} \leq v_i^{\max},
$$

$$
-b_i^{\max}
\leq
a_{i,k}
\leq
a_i^{\max,+},
$$

where $b_i^{\max} > 0$ is the maximum braking magnitude and $a_i^{\max,+} > 0$ is the maximum acceleration.

A comfort objective can penalize acceleration and jerk:

$$
\sum_k
w_a a_{i,k}^2
+
w_j(a_{i,k+1}-a_{i,k})^2.
$$

---

## 7. Conflict Regions and Crossing Order

For each conflicting pair $(i,j)$, define a conflict region where their paths overlap.

Let $s_i^{\mathrm{in}}(ij)$ be vehicle $i$'s path coordinate at which it enters the conflict region shared with $j$.

Let $s_i^{\mathrm{out}}(ij)$ be the coordinate at which it exits.

Vehicle $i$ occupies the conflict region with $j$ when

$$
s_i^{\mathrm{in}}(ij)
\leq
s_{i,k}
\leq
s_i^{\mathrm{out}}(ij).
$$

The crossing-order outcomes are:

$$
i \succ j:
\quad
i \text{ clears the conflict region before } j \text{ enters},
$$

$$
j \succ i:
\quad
j \text{ clears the conflict region before } i \text{ enters}.
$$

In branch $i \succ j$, a time-based priority condition is

$$
T_j^{\mathrm{in}}
\geq
T_i^{\mathrm{out}}
+
\Delta_{\mathrm{safe}},
$$

where $T_i^{\mathrm{out}}$ is the predicted time when $i$ exits the conflict region, $T_j^{\mathrm{in}}$ is when $j$ enters, and $\Delta_{\mathrm{safe}}$ is a time buffer.

In branch $j \succ i$, impose

$$
T_i^{\mathrm{in}}
\geq
T_j^{\mathrm{out}}
+
\Delta_{\mathrm{safe}}.
$$

---

## 8. Belief Over Crossing Order

For each conflicting pair $(i,j)$, maintain a belief over crossing order:

$$
P(i \succ j), \qquad P(j \succ i), \qquad P(\bot).
$$

For the branch score, the important uncertainty is the uncertainty between the two resolved crossing orders. A simple ambiguity score is

$$
A_{ij}
=
1
-
\left|
P(i \succ j)
-
P(j \succ i)
\right|.
$$

If both orders are equally likely, then $A_{ij}$ is high. If one order is clear, then $A_{ij}$ is low.

### 8.1 Why Not Use Entropy Only?

Entropy is often used to measure uncertainty. However, if the model includes an unresolved state $\bot$, entropy can be misleading. For example, $P(\bot)=1$ has zero entropy, but the interaction is completely unresolved.

The score above directly measures ambiguity between the two resolved crossing orders.

---

## 9. How to Detect Yielding

Consider ego $E$ and other vehicle $O$. Saying that $O$ is clearly yielding to $E$ means

$$
P(E \succ O)
\text{ is high and stable}.
$$

This should be inferred from evidence, not hand-labeled from a single observation.

### 9.1 Kinematic Evidence

Let

$$
d_O
=
s_O^{\mathrm{in}}(EO)
-
s_O
$$

be the distance from $O$ to the conflict entry.

The required braking magnitude for $O$ to stop before the conflict region is approximately

$$
b_O^{\mathrm{req}}
=
\frac{v_O^2}{2(d_O-d_{\mathrm{buf}})}.
$$

If $b_O^{\mathrm{req}}$ is comfortable and the observed acceleration $a_O^{\mathrm{obs}}$ is negative, that supports yielding.

Another signal is arrival timing. Let $T_E^{\mathrm{out}}$ be when ego would clear the conflict region and $T_O^{\mathrm{in}}$ be when the other vehicle would enter. If

$$
T_O^{\mathrm{in}}
>
T_E^{\mathrm{out}}
+
\Delta_{\mathrm{safe}},
$$

then the evidence supports $E \succ O$.

### 9.2 Policy-Prototype Evidence

Define simple acceleration prototypes for each hypothesis:

$$
\hat{a}^{E \succ O}_O
=
\text{yielding acceleration},
$$

$$
\hat{a}^{O \succ E}_O
=
\text{going-first acceleration},
$$

$$
\hat{a}^{\bot}_O
=
\text{mild cautious acceleration}.
$$

For each hypothesis $r$, define a cost

$$
C_r
=
\frac{
(a_O^{\mathrm{obs}}-\hat{a}_O^r)^2
}{\sigma_a^2}
+
\text{arrival-time inconsistency penalty}.
$$

Then a simple belief update is the softmax likelihood

$$
P_t(r)
=
\frac{
\exp(-C_r)
}{
\sum_{r'} \exp(-C_{r'})
}.
$$

A more stable version uses a Bayesian filter:

$$
P_t(r)
\propto
P(y_t \mid r)
\sum_{r'}
P(r \mid r')P_{t-1}(r'),
$$

where $y_t$ contains observed speed, acceleration, distance to conflict, and arrival-time features.

### 9.3 Clear Yielding Criterion

A practical rule is:

$$
P(E \succ O) > p_{\mathrm{high}}
$$

for several consecutive MPC cycles, with consistent braking and timing evidence.

For example,

$$
p_{\mathrm{high}} = 0.85
$$

with persistence over three to five cycles.

So:

> **Clearly yielding = high belief + stable belief + physically plausible evidence.**

---

## 10. Reciprocal-Caution Constraint

This is the central formulation.

While crossing order between $i$ and $j$ is unresolved, vehicle $i$ should preserve the ability to yield to $j$. A simple way to encode this is to require that $i$ can still stop before entering the conflict region using comfortable braking.

Let

$$
d_i^{\mathrm{in}}(ij)
=
s_i^{\mathrm{in}}(ij)
-
s_{i,k}
$$

be the distance from $i$ to the conflict entry.

Let $b_i^{\mathrm{comf}}$ be comfortable braking magnitude. The comfortable stopping distance is

$$
d_i^{\mathrm{stop}}
=
\frac{v_{i,k}^2}{2b_i^{\mathrm{comf}}}.
$$

The reciprocal-caution constraint is

$$
h^\bot_{i \leftarrow j,k}
=
\frac{v_{i,k}^2}{2b_i^{\mathrm{comf}}}
+
d_{\mathrm{buf}}
-
\left(
s_i^{\mathrm{in}}(ij)
-
s_{i,k}
\right)
\leq
0.
$$

Read this as:

> While the interaction with $j$ is unresolved, vehicle $i$ must not drive so fast or so close that it loses the option to yield comfortably.

There is also a symmetric constraint:

$$
h^\bot_{j \leftarrow i,k}
\leq
0.
$$

This symmetric pair of constraints is the mathematical expression of reciprocal caution.

### 10.1 Softened Version for Feasibility

In practice, the constraint may be impossible to satisfy in all states. Use a nonnegative slack $\xi_{i \leftarrow j,k}$:

$$
h^\bot_{i \leftarrow j,k}
\leq
\xi_{i \leftarrow j,k},
\qquad
\xi_{i \leftarrow j,k}
\geq
0,
$$

and penalize it strongly:

$$
w_\xi \xi_{i \leftarrow j,k}^2.
$$

This avoids solver failure and records how badly the reciprocal-caution condition is violated.

---

## 11. The Multiplier and Its Meaning

Let

$$
\lambda^\bot_{i \leftarrow j,k}
\geq
0
$$

be the Lagrange multiplier of the reciprocal-caution constraint

$$
h^\bot_{i \leftarrow j,k}
\leq
0.
$$

A Lagrange multiplier is a shadow price. Here it means:

$$
\lambda^\bot_{i \leftarrow j,k}
=
\text{marginal cost of forcing vehicle } i
\text{ to keep the option to yield to } j.
$$

If $i$ is far from the intersection and can easily stop, then

$$
h^\bot_{i \leftarrow j,k} < 0,
\qquad
\lambda^\bot_{i \leftarrow j,k} \approx 0.
$$

The caution requirement is not costly.

If $i$ is close and moving fast, then preserving the yield option may require braking or progress loss:

$$
h^\bot_{i \leftarrow j,k} \approx 0,
\qquad
\lambda^\bot_{i \leftarrow j,k} > 0.
$$

The caution requirement is shaping the plan.

This is the refined use of the original multiplier idea. We are not just measuring generic collision pressure. We are measuring the cost of keeping the crossing order unresolved.

---

## 12. Branch Score

For a conflicting pair $(i,j)$, define the branch score

$$
\Psi_{ij}
=
A_{ij}
\sum_{k=0}^{N}
\left(
\tilde{\lambda}^\bot_{i \leftarrow j,k}
+
\tilde{\lambda}^\bot_{j \leftarrow i,k}
\right)
\Delta t.
$$

Here:

- $A_{ij}$ measures crossing-order ambiguity.
- $\lambda^\bot_{i \leftarrow j,k}$ measures the cost of forcing $i$ to remain able to yield to $j$.
- $\lambda^\bot_{j \leftarrow i,k}$ measures the opposite cost.
- $\tilde{\lambda}$ is a filtered or normalized multiplier.

The meaning is:

$$
\Psi_{ij}
=
\text{cost of keeping the interaction unresolved}.
$$

### 12.1 Multiplier Normalization

Raw multipliers depend on cost weights and units. A simple normalization is

$$
\tilde{\lambda}^\bot_{i \leftarrow j,k}
=
\frac{
\lambda^\bot_{i \leftarrow j,k}
}{
\epsilon + J_{\mathrm{scale}}
}.
$$

A gradient-aware normalization is

$$
\tilde{\lambda}^\bot_{i \leftarrow j,k}
=
\frac{
\lambda^\bot_{i \leftarrow j,k}
\left\|
\nabla h^\bot_{i \leftarrow j,k}
\right\|
}{
\epsilon + J_{\mathrm{scale}}
}.
$$

In an implementation, smooth the normalized multipliers over time:

$$
\bar{\lambda}_t
=
\alpha \bar{\lambda}_{t-1}
+
(1-\alpha)\tilde{\lambda}_t.
$$

---

## 13. When to Branch

Branch when

$$
\Psi_{ij} > \tau_\Psi.
$$

Equivalently, define a time-indexed pressure

$$
\psi_{ij,k}
=
A_{ij}
\left(
\tilde{\lambda}^\bot_{i \leftarrow j,k}
+
\tilde{\lambda}^\bot_{j \leftarrow i,k}
\right).
$$

Then select the first predicted time where reciprocal caution becomes too expensive:

$$
k_{b,ij}
=
\min
\left\{
k : \psi_{ij,k} > \tau_\psi
\right\}.
$$

Also use a latest-feasible-branch guard. Let $k_{ij}^{\max}$ be the latest time at which both crossing-order branches remain feasible. Then

$$
k_{b,ij}
=
\min
\left(
\min\{k : \psi_{ij,k} > \tau_\psi\},
k_{ij}^{\max}
\right).
$$

This prevents the planner from waiting until one branch is no longer feasible.

---

## 14. Toy Example: Seeing $\lambda$ in Action

This section gives a deliberately simple one-step example. Its purpose is not to be a realistic MPC model. Its purpose is to make the multiplier interpretation concrete.

The key message is:

$$
\lambda = 0
\Longrightarrow
\text{reciprocal caution is cheap},
$$

$$
\lambda > 0
\Longrightarrow
\text{reciprocal caution is forcing the planner to give something up}.
$$

In the full MPC, $\lambda$ is a marginal cost. In the toy problem below, it becomes simple enough that we can compute it by hand.

### 14.1 Toy Scenario

Ego $E$ approaches an unsignalized crossroad. Another vehicle $O$ approaches from the side. The crossing order is unclear:

$$
P(E \succ O) = 0.5,
\qquad
P(O \succ E) = 0.5.
$$

So ego does not yet know whether it will cross first or whether the other vehicle will cross first.

While the order is unclear, ego should preserve the ability to yield to $O$. For this hand calculation, we only look at ego’s side of reciprocal caution.

Assume ego wants to drive at

$$
v_{\mathrm{des}} = 10 \ \mathrm{m/s}.
$$

If there were no other vehicle, ego would choose $v=10$. Consider the one-step optimization

$$
\min_v
\frac{1}{2}(v-v_{\mathrm{des}})^2.
$$

This objective simply says: choose a speed close to the desired speed.

### 14.2 Caution as a Speed Limit

Let $d_{\mathrm{in}}$ be ego’s distance to the conflict-region entry, $d_{\mathrm{buf}}$ a safety buffer, and $b_{\mathrm{comf}}$ comfortable braking. Ego’s comfortable stopping distance is

$$
d_{\mathrm{stop}}
=
\frac{v^2}{2b_{\mathrm{comf}}}.
$$

To preserve the ability to yield, require

$$
d_{\mathrm{stop}} + d_{\mathrm{buf}}
\leq
d_{\mathrm{in}}.
$$

Equivalently,

$$
\frac{v^2}{2b_{\mathrm{comf}}}
+
d_{\mathrm{buf}}
-
d_{\mathrm{in}}
\leq
0.
$$

This is the reciprocal-caution constraint

$$
h^\bot_{E \leftarrow O}(v) \leq 0.
$$

For hand calculation, rewrite it as a speed limit:

$$
v \leq v_{\mathrm{safe}},
$$

where

$$
v_{\mathrm{safe}}
=
\sqrt{
2b_{\mathrm{comf}}
(d_{\mathrm{in}}-d_{\mathrm{buf}})
}.
$$

So the toy optimization is

$$
\min_v
\frac{1}{2}(v-10)^2
\quad
\text{subject to}
\quad
v \leq v_{\mathrm{safe}}.
$$

### 14.3 The Lagrangian and KKT Conditions

Write the constraint as

$$
v - v_{\mathrm{safe}} \leq 0.
$$

The Lagrangian is

$$
L(v,\lambda)
=
\frac{1}{2}(v-10)^2
+
\lambda(v-v_{\mathrm{safe}}).
$$

The KKT conditions are

$$
v - 10 + \lambda = 0,
$$

$$
\lambda \geq 0,
$$

$$
v - v_{\mathrm{safe}} \leq 0,
$$

$$
\lambda(v-v_{\mathrm{safe}})=0.
$$

The solution is

$$
v^\star
=
\min(10,v_{\mathrm{safe}}),
$$

and the multiplier is

$$
\lambda^\star
=
\max(0,10-v_{\mathrm{safe}}).
$$

This is the whole point. If the caution constraint does not affect ego, then $v_{\mathrm{safe}} \geq 10$, so

$$
v^\star = 10,
\qquad
\lambda^\star = 0.
$$

If caution forces ego to slow below its desired speed, then $v_{\mathrm{safe}} < 10$, so

$$
v^\star = v_{\mathrm{safe}},
\qquad
\lambda^\star = 10-v_{\mathrm{safe}} > 0.
$$

Thus, in this simplified quadratic example,

$$
\lambda^\star
=
\text{how much speed ego gives up to preserve the yield option}.
$$

In a full MPC, $\lambda$ is not literally speed loss, but the interpretation is the same: it is the price of keeping the cautious option alive.

### 14.4 Case A: Ego Is Far from the Crossroad

Let

$$
b_{\mathrm{comf}} = 3 \ \mathrm{m/s^2},
\qquad
d_{\mathrm{buf}} = 5 \ \mathrm{m},
\qquad
d_{\mathrm{in}} = 30 \ \mathrm{m}.
$$

Then

$$
v_{\mathrm{safe}}
=
\sqrt{2 \cdot 3 \cdot (30-5)}
=
\sqrt{150}
\approx
12.25 \ \mathrm{m/s}.
$$

Since $v_{\mathrm{safe}} > v_{\mathrm{des}} = 10$, ego can drive at its desired speed and still preserve the ability to yield:

$$
v^\star = 10,
\qquad
\lambda^\star = 0.
$$

Interpretation: the crossing order may be ambiguous, but ambiguity is not costly yet. The common cautious trunk is cheap.

### 14.5 Case B: Ego Is Closer to the Crossroad

Now let

$$
d_{\mathrm{in}} = 13 \ \mathrm{m}.
$$

Then

$$
v_{\mathrm{safe}}
=
\sqrt{2 \cdot 3 \cdot (13-5)}
=
\sqrt{48}
\approx
6.93 \ \mathrm{m/s}.
$$

The desired speed $10 \ \mathrm{m/s}$ is too high if ego wants to preserve the ability to yield. The optimizer chooses

$$
v^\star = 6.93 \ \mathrm{m/s},
$$

and the multiplier is

$$
\lambda^\star
=
10-6.93
=
3.07.
$$

Interpretation: to keep the crossing order unresolved, ego must slow from $10$ to $6.93 \ \mathrm{m/s}$. The value $\lambda=3.07$ is the toy-model price of reciprocal caution.

### 14.6 Case C: Ego Is Very Close

Now let

$$
d_{\mathrm{in}} = 8 \ \mathrm{m}.
$$

Then

$$
v_{\mathrm{safe}}
=
\sqrt{2 \cdot 3 \cdot (8-5)}
=
\sqrt{18}
\approx
4.24 \ \mathrm{m/s}.
$$

The optimizer chooses

$$
v^\star = 4.24 \ \mathrm{m/s},
$$

and

$$
\lambda^\star
=
10-4.24
=
5.76.
$$

Interpretation: preserving the ability to yield is now very expensive. Ego has to slow drastically. If the crossing order is still ambiguous, this is the kind of situation where branching becomes useful.

### 14.7 Adding Ambiguity

The multiplier alone should not decide branching. We also need to know whether the crossing order is still ambiguous. Use

$$
A
=
1
-
\left|
P(E \succ O)
-
P(O \succ E)
\right|.
$$

If crossing order is maximally unclear,

$$
P(E \succ O)=0.5,
\qquad
P(O \succ E)=0.5,
$$

then

$$
A
=
1-|0.5-0.5|
=
1.
$$

If ego is clearly going first,

$$
P(E \succ O)=0.9,
\qquad
P(O \succ E)=0.05,
$$

then

$$
A
=
1-|0.9-0.05|
=
0.15.
$$

So $A$ asks:

> Is the crossing order still unclear?

### 14.8 Toy Branch Score

For this one-sided toy example, use

$$
\Psi = A\lambda.
$$

In the full reciprocal version, use both vehicles:

$$
\Psi_{EO}
=
A_{EO}
\left(
\lambda^\bot_{E \leftarrow O}
+
\lambda^\bot_{O \leftarrow E}
\right).
$$

#### Ambiguous and Far Away

From Case A, $\lambda=0$. If ambiguity is high, $A=1$, then

$$
\Psi = A\lambda = 1 \cdot 0 = 0.
$$

Decision: **no branch**.

The crossing order is unclear, but keeping both options open is cheap.

#### Ambiguous and Close

From Case B, $\lambda=3.07$. If ambiguity is high, $A=1$, then

$$
\Psi = 1 \cdot 3.07 = 3.07.
$$

If the branch threshold is

$$
\tau_\Psi = 2.5,
$$

then

$$
\Psi > \tau_\Psi.
$$

Decision: **branch**.

The crossing order is unclear, and maintaining reciprocal caution is now expensive.

#### Close but Clearly Yielding

Again use $\lambda=3.07$, but suppose the other vehicle is clearly yielding:

$$
P(E \succ O)=0.9,
\qquad
P(O \succ E)=0.05.
$$

Then $A=0.15$, so

$$
\Psi
=
A\lambda
=
0.15 \cdot 3.07
=
0.46.
$$

If $\tau_\Psi=2.5$, then

$$
\Psi < \tau_\Psi.
$$

Decision: **no branch**.

Reciprocal caution is expensive, but the crossing order is already mostly resolved.

This is important: the method does not branch just because $\lambda$ is high. It branches when ambiguity is high and caution is expensive.

### 14.9 Full Reciprocal Example

Now include both vehicles. Suppose the reciprocal-caution multipliers are

$$
\lambda^\bot_{E \leftarrow O}
=
3.07,
\qquad
\lambda^\bot_{O \leftarrow E}
=
2.40.
$$

The second multiplier is interpreted as: how costly is it for the other vehicle to preserve the ability to yield to ego? In an ego-only vehicle, we do not directly control the other vehicle, but in the model we predict it using the same reciprocal-caution principle.

If

$$
P(E \succ O)=0.5,
\qquad
P(O \succ E)=0.5,
$$

then $A=1$. The reciprocal branch score is

$$
\Psi_{EO}
=
1(3.07+2.40)
=
5.47.
$$

If

$$
\tau_\Psi = 4.0,
$$

then

$$
\Psi_{EO} > \tau_\Psi,
$$

and the planner branches on the ego-other crossing order:

$$
\text{common trunk}
\longrightarrow
\begin{cases}
E \succ O, \\
O \succ E.
\end{cases}
$$

Before the branch time, the action is shared. For example, both branches may require ego to take a moderate cautious action now, such as

$$
v_0 = 6.93 \ \mathrm{m/s}.
$$

After the branch time, controls differ. In the $E \succ O$ branch, ego proceeds and can accelerate back toward

$$
v_{\mathrm{des}} = 10.
$$

In the $O \succ E$ branch, ego continues slowing or stops before the conflict region.

---

## 15. Reciprocal-Caution MPC Before Branching

Before building a branch tree, solve a reciprocal-caution MPC over the active vehicles $\mathcal{A}_t$. The decision variables are

$$
x_{i,0:N},
\qquad
a_{i,0:N-1},
\qquad
i \in \mathcal{A}_t.
$$

A simple objective is

$$
\min
\sum_{i \in \mathcal{A}_t}
\sum_{k=0}^{N-1}
\left[
w_v(v_{i,k}-v_i^{\mathrm{des}})^2
+
w_a a_{i,k}^2
+
w_j(a_{i,k+1}-a_{i,k})^2
\right]
+
\sum w_\xi \xi^2.
$$

Subject to:

$$
s_{i,k+1}
=
s_{i,k}
+
\Delta t v_{i,k}
+
\frac{1}{2}\Delta t^2 a_{i,k},
$$

$$
v_{i,k+1}
=
v_{i,k}
+
\Delta t a_{i,k},
$$

$$
0 \leq v_{i,k} \leq v_i^{\max},
$$

$$
-b_i^{\max}
\leq
a_{i,k}
\leq
a_i^{\max,+},
$$

plus car-following constraints and reciprocal-caution constraints for unresolved conflicting pairs.

After solving this MPC, extract the reciprocal-caution multipliers and compute $\Psi_{ij}$.

---

## 16. Branch MPC After Selecting a Pair

Suppose pair $(i,j)$ has the highest branch score. Build two branches:

$$
b=1: i \succ j,
\qquad
b=2: j \succ i.
$$

The objective is expected branch cost:

$$
\min
\sum_{b \in \{i \succ j,\, j \succ i\}}
P_b J^b,
$$

where

$$
P_{i \succ j} = P(i \succ j),
\qquad
P_{j \succ i} = P(j \succ i).
$$

Before the branch time, controls are shared:

$$
a^{i \succ j}_{q,k}
=
a^{j \succ i}_{q,k},
\qquad
k < k_b,
\qquad
q \in \mathcal{A}_t.
$$

This is the **non-anticipativity constraint**. It prevents the planner from using different actions before the crossing order has been resolved.

After the branch time, controls can differ.

In branch $i \succ j$, impose

$$
T_j^{\mathrm{in}}
\geq
T_i^{\mathrm{out}}
+
\Delta_{\mathrm{safe}}.
$$

In branch $j \succ i$, impose

$$
T_i^{\mathrm{in}}
\geq
T_j^{\mathrm{out}}
+
\Delta_{\mathrm{safe}}.
$$

The first control is executed, and the whole process repeats at the next MPC cycle.

---

## 17. Algorithm

| Step | Operation |
|---:|---|
| 1 | Identify the front vehicle on each incoming approach. |
| 2 | Build the conflict graph among front vehicles whose paths intersect. |
| 3 | For each conflicting pair $(i,j)$, update beliefs $P(i \succ j)$, $P(j \succ i)$, and $P(\bot)$. |
| 4 | Compute crossing-order ambiguity $A_{ij}=1-\left|P(i \succ j)-P(j \succ i)\right|$. |
| 5 | Solve the reciprocal-caution MPC. |
| 6 | Extract multipliers $\lambda^\bot_{i \leftarrow j,k}$ and $\lambda^\bot_{j \leftarrow i,k}$. |
| 7 | Compute branch scores $\Psi_{ij}$. |
| 8 | If no score exceeds threshold, execute the first action of the reciprocal-caution MPC. |
| 9 | If one pair exceeds threshold, build branches $i \succ j$ and $j \succ i$. |
| 10 | Solve branch MPC with shared trunk before $k_b$. Execute the first control and replan. |

---

## 18. Three Important Cases

### 18.1 Ambiguous but Far Away

The crossing order is unclear:

$$
A_{ij} \text{ high}.
$$

But both vehicles can easily stop or yield:

$$
\lambda^\bot_{i \leftarrow j,k}
\approx
0,
\qquad
\lambda^\bot_{j \leftarrow i,k}
\approx
0.
$$

So

$$
\Psi_{ij} \approx 0.
$$

No branch is needed.

Interpretation: the situation is ambiguous, but ambiguity is not costly yet.

### 18.2 Close but Clearly Resolved

Suppose $i$ is clearly going first:

$$
P(i \succ j) \approx 1,
\qquad
P(j \succ i) \approx 0.
$$

Then

$$
A_{ij} \approx 0.
$$

Even if safety is important, branching is not needed.

Interpretation: the situation matters, but the crossing order is already clear.

### 18.3 Close and Unresolved

The crossing order is unclear:

$$
A_{ij} \text{ high}.
$$

Keeping both yield options is becoming costly:

$$
\lambda^\bot_{i \leftarrow j,k}
+
\lambda^\bot_{j \leftarrow i,k}
\text{ high}.
$$

Then

$$
\Psi_{ij} \text{ high}.
$$

Branch.

Interpretation: this is the moment when one shared cautious plan is no longer enough.

---

## 19. Why This Is Not Just Time-to-Collision

A geometric risk metric such as time-to-collision asks:

> Are the vehicles geometrically close?

The reciprocal-caution multiplier asks:

> How costly is it, under the actual planning objective, to keep the crossing order unresolved?

This difference matters. A vehicle can be geometrically close but clearly yielding, in which case branching may be unnecessary. Or a vehicle can be not yet close, but the planner may already be losing the ability to preserve both crossing-order options comfortably.

The dual price is optimizer-native. It reflects dynamics, comfort, progress cost, control limits, car-following constraints, and option preservation.

---

## 20. Experimental Plan

### 20.1 Scenarios

Start with the simplest possible scenarios:

1. Two-car crossroad: one ego and one conflicting vehicle.
2. Four-approach crossroad with one front vehicle per approach.
3. Queue scenario: followers obey car-following constraints behind front vehicles.
4. Ambiguous-yield scenario: both front vehicles decelerate mildly before one proceeds.
5. Aggressive-other scenario: one vehicle does not exhibit reciprocal caution.

### 20.2 Baselines

Compare against:

1. deterministic MPC assuming the most likely crossing order;
2. conservative MPC that always preserves all yield options;
3. fixed-time branch MPC;
4. time-to-collision triggered branching;
5. raw collision-multiplier triggered branching;
6. proposed reciprocal-caution multiplier branching.

### 20.3 Metrics

Measure:

- collision or conflict-region violation rate;
- average delay;
- unnecessary stopping;
- speed loss;
- comfort and jerk;
- branch count;
- solver time;
- time margin at conflict entry;
- false branch rate and missed branch rate.

### 20.4 Critical Ablations

The key ablations are:

$$
\text{raw collision multiplier}
\quad
\text{versus}
\quad
\text{reciprocal-caution multiplier},
$$

and

$$
\lambda
\quad
\text{versus}
\quad
A\lambda.
$$

If reciprocal-caution multipliers trigger useful branch times earlier and with fewer false positives, the idea has evidence.

---

## 21. Known Limitations

### 21.1 The Belief May Be Wrong

The probabilities $P(i \succ j)$ and $P(j \succ i)$ are beliefs, not ground truth. Poor perception or poor behavioral models can mislead the planner. The research should evaluate sensitivity to belief error.

### 21.2 Reciprocal Caution May Fail for Aggressive Drivers

Some drivers may not preserve a yield option. This planner should be paired with a robust safety fallback that assumes emergency braking or worst-case entry behavior.

### 21.3 Comfortable Stopping Is Not Enough for Safety

The reciprocal-caution constraint uses comfortable braking to price caution. Hard safety should use maximum braking, reachable sets, or a separate safety filter. Comfortable braking is a planning preference, not a final safety proof.

### 21.4 Multipliers Can Be Noisy

Nonlinear MPC multipliers can vary due to scaling, active-set changes, and solver tolerances.

Normalize and filter them. Do not use raw multiplier thresholds without calibration.

### 21.5 Crossing-Order Branches May Not Cover Route Uncertainty

This first study assumes route/path options are known or externally inferred. If route intent is unknown, high-level multimodal prediction is still needed. The proposed method addresses local crossing-order resolution, not all intent uncertainty.

---

## 22. Recommended First Paper Claim

A precise contribution statement is:

> We study fixed-path unsignalized crossroad planning under crossing-order uncertainty. We propose reciprocal-caution MPC, in which unresolved conflicting vehicles preserve safe yield options. The Lagrange multipliers of these reciprocal-caution constraints quantify the optimizer-native cost of keeping the crossing order unresolved. Combining this dual cost with an online crossing-order ambiguity belief yields a branch score. The planner creates a small two-branch contingency tree only when unresolved reciprocal caution is both ambiguous and expensive.

The shorter slogan is:

> **Branch when reciprocal caution becomes expensive.**

---

## 23. What Not to Claim

Do not claim:

- that this solves general human intent prediction;
- that the belief probabilities are objectively correct;
- that reciprocal caution holds for all drivers;
- that dual variables for interaction relevance are new;
- that this is a complete safety guarantee without a fallback safety layer.

Do claim:

- that the method gives an optimizer-native branch trigger for unresolved crossing-order interactions;
- that the branch tree is small because it branches on local crossing order, not arbitrary trajectory modes;
- that reciprocal caution captures the symmetric hesitation/preservation behavior common at unsignalized intersections;
- that the multipliers price the cost of maintaining this unresolved cautious state.

---

## 24. Summary

The refined idea is simple but rigorous.

At an unsignalized crossroad, the central uncertainty is often the crossing order. While crossing order is unclear, both vehicles may behave cautiously. We encode this through reciprocal-caution constraints: each unresolved vehicle must preserve the ability to yield.

The MPC multipliers of these constraints tell us how expensive it is to keep the interaction unresolved. When this dual cost is high and the crossing order belief is still ambiguous, the planner branches into two possible crossing orders.

The final method is:

$$
\text{belief over crossing order}
+
\text{reciprocal-caution MPC}
+
\text{dual-priced branch trigger}
+
\text{small crossing-order tree}.
$$

This is narrower and more defensible than a general multi-agent branch planner, and it is directly matched to the crossroad scenario.

---

## References

1. T. Li, L. Zhang, S. Liu, and S. Shen, “MARC: Multipolicy and Risk-aware Contingency Planning for Autonomous Driving,” arXiv:2308.12021, 2023.

2. L. Peters, A. Bajcsy, C.-Y. Chiu, D. Fridovich-Keil, F. Laine, L. Ferranti, and J. Alonso-Mora, “Contingency Games for Multi-Agent Interaction,” arXiv:2304.05483, 2023.

3. H. Kim, S. H. Nair, and F. Borrelli, “Scalable Multi-modal Model Predictive Control via Duality-based Interaction Predictions,” arXiv:2402.01116, 2024.

4. Z. Xing, R. Chaudhari, M. Leibold, D. Wollherr, and M. Buss, “Branch-Stochastic Model Predictive Control for Motion Planning under Multi-Modal Uncertainty with Scenario Clustering,” arXiv:2605.22600, 2026.

5. Y. Jeong and collaborators, “Stochastic Model-Predictive Control with Uncertainty Estimation for Uncontrolled Intersection Passing,” *Applied Sciences*, 2021.