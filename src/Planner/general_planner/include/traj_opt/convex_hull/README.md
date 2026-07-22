# Convex-hull trajectory operator

This module converts each local ascending-power polynomial used by MINCO into
Bezier or MINVO control points. It supports physical-time derivatives and
uniform binary subdivision without sampling the curve.

## Complete operator

For one source segment

```text
p(t) = sum(k=0..n) c[k] t^k,  0 <= t <= T,
```

the order-`r` physical derivative, expressed on normalized time `u=t/T`, is

```text
p^(r)(Tu) =
  sum(j=0..n-r) (j+r)!/j! * T^j * c[j+r] * u^j.
```

For subdivision leaf `l`, let

```text
u = a_l + h v,  a_l = l / 2^depth,  h = 1 / 2^depth.
```

The exact power-basis restriction matrix is

```text
H_l(j,k) = choose(k,j) * a_l^(k-j) * h^j,  j <= k.
```

If `B` is the power-to-Bezier or power-to-MINVO matrix, the stacked complete
operator for all leaves of one source segment is

```text
Q = A_r(T) C,
A_r(T) = stack_l(B H_l) D_r(T) S_r.
```

`S_r` selects source columns `c[r] ... c[n]`, and
`D_r(T)[j,j] = (j+r)!/j! * T^j`. `Representation::update()` explicitly
assembles this `A_r(T)` once per segment and optimization evaluation. The
topology-only part (`B`, all `H_l`, derivative factors) is immutable and shared
through a cache.

## Reverse mode

Given a cost gradient `G_Q = dJ/dQ`,

```text
dJ/dC = A_r(T)^T G_Q,
dJ/dT|C = <G_Q, (dA_r(T)/dT) C>_F.
```

`backwardAdd()` adds these two partials to the MINCO coefficient and direct
duration gradients. `MINCOOptimizer` then calls its existing
`propagateGradFull()` exactly once, so the banded-system adjoint maps the
coefficient gradient to inner waypoints, boundary states, and durations.

If a cost also depends explicitly on subdivided leaf start times or durations,
`backwardPieceTimesAdd()` maps those partials to source durations and the common
trajectory start time.

## Optimization use

The generic API permits one `Representation<DIM>` for any requested derivative
order. The optimized state2state cost path instead creates only the position
Bezier representation and obtains all derivatives using the hodograph
difference operator. In a cost manager, implement the optional callback

```cpp
double evaluateCoefficient(const Trajectory &trajectory,
                           CoeffMat &grad_coefficients,
                           Eigen::VectorXd &grad_durations) const;
```

Within it:

1. update the cached position representation once with
   `trajectory.updateConvexHull(hull)`;
2. generate derivative controls by repeated Bezier differences;
3. evaluate corridor and derivative-bound penalties on the controls;
4. reverse the difference chain into the position-control gradient;
5. call `hull.backwardAdd()` once.

For a convex safe corridor, requiring every position control point of a leaf to
lie in that leaf's half-space polytope certifies the whole continuous leaf. For
a convex velocity or acceleration bound (for example, a Euclidean ball),
requiring every derivative control point to satisfy the bound likewise
certifies the continuous derivative. Increasing subdivision depth tightens each
local hull while preserving this guarantee.

Bezier and MINVO conversion data and licensing are documented in
`THIRD_PARTY_NOTICES.md`.

## State2state corridor integration

The planner keeps the ordinary state2state and high-speed exploration paths
separate:

- `ExpTrajOpt` uses `ExpIntegralCostManager` by default and optionally
  `ExpConvexCostManager` for the state2state corridor backend.
- `ExplorationTrajOpt` continues to use `ExplorationCostManager`; the
  exploration path does not read or use the convex-hull switch.

The state2state mode is selected under the existing `exp_traj` namespace:

```yaml
traj_opt:
  exp_traj:
    convex_hull_en: false              # false: original dense integral
    convex_hull_basis: 0               # 0: Bezier, 1: MINVO
    convex_hull_subdivision_depth: 0   # 2^depth leaves per MINCO piece
    convex_hull_cost_version: 1        # 1: reference, 2: Bezier equivalent dedup
```

In convex mode, position, velocity, acceleration and jerk polynomial bounds
use continuous control-point hulls. Attractor, guide-path, swarm, angular-rate
and thrust costs retain the original dense integral because they are not all
polynomial convex functions of the flat output.

For a seventh-degree MINCO piece, depth zero converts eight position controls
once. Its velocity, acceleration and jerk hodographs contain seven, six and
five derived controls. Uniform subdivision multiplies every count by
`2^depth`, so it should not be enabled globally merely to reduce occasional
conservatism; selective/adaptive leaf subdivision is the intended follow-up.

When no residual dense term is active, `ExpConvexCostManager::usesDenseSampling()`
allows `MINCOOptimizer` to bypass the integral nodes and basis construction
entirely. Timing reports separate the dense residual and control-point
functional so hybrid configurations remain visible.

`MINCOOptimizer` can collect timing only when explicitly enabled. `ExpTrajOpt`
enables it and reports the accumulated dense-integral time, total MINCO cost
evaluation time, LBFGS wall time, and both corresponding percentages when
`print_optimizer_log` is enabled. Other optimizers leave timing disabled.

## Control-point cost functional

Let `L=2^depth`, let a degree-`d` quantity have `m=d+1` controls per leaf,
and let `q[l,i]` be either a corridor half-space residual or a squared-norm
bound residual. The reference functional is

```text
J_v1 = w T/(L m) * sum(l=0..L-1) sum(i=0..m-1) phi(q[l,i]),
```

where `phi` is the existing smooth one-sided L1 penalty. Applying this formula
to all corridor planes certifies position containment; applying it to the
velocity, acceleration and jerk hodographs enforces their convex magnitude
bounds. Division by `L m` is important: subdivision tightens the hull without
silently multiplying the configured penalty weight.

Adjacent Bezier leaves share an exact endpoint,

```text
Q[l,m-1] = Q[l+1,0].
```

For Bezier output, cost version 2 folds the two identical evaluations into one
unique control:

```text
J_v2 = w T/(L m) * sum(j in unique controls) mu[j] phi(q[j]),
mu[j] = 2 at an internal leaf boundary, otherwise 1.
```

Thus `J_v2 == J_v1` as a function of the source polynomial and duration. Its
gradient is also identical: the selected shared row receives
`mu[j] w T/(L m) phi'(q[j]) dq/dQ`, then the existing hodograph adjoint and the
single `backwardAdd()` map it to MINCO coefficients and durations. The explicit
duration derivative of each weighted term is `local_cost/T`; derivative-control
dependence on `T` is added separately by the reverse hodograph chain. No
stop-gradient or approximate control-to-waypoint mapping is used.

For seventh-degree MINCO at depth two, V1 evaluates

```text
4 * (8 + 7 + 6 + 5) = 104
```

position/velocity/acceleration/jerk controls per source segment. V2 evaluates

```text
(4*7+1) + (4*6+1) + (4*5+1) + (4*4+1) = 92,
```

an 11.5% reduction with the same objective and convex-hull certificate.
MINVO controls do not generally coincide with leaf endpoints, so selecting
MINVO automatically uses cost V1; endpoint folding is not applied there.

## LBFGS convergence counters

`EXP_EVALUATIONS` counts all objective/gradient calls, including rejected line
search trials. The progress callback additionally records accepted LBFGS
iterations, the sum and maximum of line-search evaluations, and accepted step
sizes. For one optimizer invocation the accounting identity is approximately

```text
objective evaluations = 1 initial evaluation + line-search evaluations.
```

These counters are written to the state2state timing CSV. They are not added to
the normal per-replan console output; the final planner line remains the compact
total optimization time and dense/control-point shares.

## Depth-two same-goal result

The following paired runs used `click_real_highspeed.yaml`, goal
`[69.032, 1.901, 1.500]`, Bezier depth two and the same 25-second launch window.
Both runs made 145 optimizer calls, so the cost-kernel comparison is unusually
well controlled.

| metric | V1 reference | V2 equivalent dedup | change |
|---|---:|---:|---:|
| objective/gradient evaluations | 80,324 | 80,395 | +0.09% |
| accepted LBFGS iterations | 47,598 | 47,680 | +0.17% |
| line-search eval / accepted iter | 1.685 | 1.683 | -0.1% |
| control-point functional | 554.063 ms | 523.929 ms | -5.44% |
| complete MINCO evaluations | 1730.229 ms | 1693.360 ms | -2.13% |
| LBFGS wall time | 1899.682 ms | 1860.312 ms | -2.07% |
| planner total optimization time | 1905.633 ms | 1869.104 ms | -1.92% |
| path length | 102.250 m | 103.021 m | +0.75% |

The nearly unchanged accepted-iteration and line-search counts show that the
large evaluation count is caused by the optimizer's many accepted iterations
plus line-search trials, not by an accidental duplicate gradient propagation.
The V2 speedup comes from removing algebraically duplicate control penalties;
it does not change the optimization landscape. Closed-loop totals still have
run-to-run variance because real-time replanning can produce a different number
of polynomial pieces, so kernel timings and convergence counters should always
be inspected alongside the final total.
