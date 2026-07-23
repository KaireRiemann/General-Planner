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

## Adaptive constrained optimization with PHR-ALM

The optional constrained state2state path replaces the polynomial penalty
terms by explicit inequalities `g_i(x) <= 0`. Position control points use

```text
g_pos(Q) = (a^T Q + b) / position_scale,
```

for every active corridor plane. Velocity, acceleration and jerk hodograph
controls use

```text
g_r(Q_r) = ||Q_r||^2 / bound_r^2 - 1.
```

The generic Powell-Hestenes-Rockafellar outer solver is implemented in
`utils/optimization/phr_alm.hpp`, next to LBFGS, SDLP and SDQP. For fixed
multipliers and penalty it asks the existing LBFGS solver to minimize

```text
L_rho(x, lambda) = f(x)
  + sum_i [max(0, lambda_i + rho g_i(x))^2 - lambda_i^2] / (2 rho).
```

The `-lambda_i^2/(2 rho)` term is constant during one inner solve, so it does
not change its gradient, but it is required for the standard PHR merit value
used by outer-loop bookkeeping and LBFGS stopping tests.

After the LBFGS inner solve it evaluates all constraints, updates
`lambda_i <- max(0, lambda_i + rho g_i)`, and grows `rho` only when the maximum
violation has not decreased sufficiently. Thus LBFGS remains the only
unconstrained inner optimizer; PHR-ALM supplies the constrained outer loop.

`ExpConvexAlmCostManager` owns the trajectory-specific part: the constraint
layout, values and reverse-mode Jacobian product. For every active constraint,
the PHR derivative with respect to its control point is

```text
dL/dQ_i = max(0, lambda_i + rho g_i) * dg_i/dQ_i.
```

These gradients are accumulated on the selected hodograph controls, reversed
through the Bezier difference chain, passed once through
`Representation::backwardAdd()`, and then propagated by MINCO's existing
banded-system adjoint to waypoint and time variables. There is no finite
difference or sampled approximation in this chain.

Adaptive subdivision selects depth zero for a source MINCO segment while its
coarse position hull has sufficient corridor margin, and selects the configured
maximum depth near a corridor plane. Proactive derivative-margin refinement is
disabled by default because it made nearly every segment fine in the high-speed
test; an actually violated derivative hull still triggers refinement. The
selected topology is frozen for an entire LBFGS inner solve. It may be refined
only between inner solves. Replacing a subdivision layout resets its PHR
multipliers and does not consume a formal outer iteration. Appending constraints
in the experimental active-set mode preserves the old multiplier prefix.

Before PHR starts, the current implementation optionally runs one loose fixed
depth-two V2 penalty solve. This supplies a near-feasible trajectory cheaply
and leaves PHR to perform the stationarity and certificate correction. PHR
always performs at least one inner solve: penalty feasibility alone is not a
constrained KKT condition. An experimental constraint active set is available,
but is disabled by default because repeated constraint insertion increased the
number of LBFGS solves on the full-flight benchmark.

The state2state switches are:

```yaml
traj_opt:
  exp_traj:
    convex_hull_en: true
    convex_hull_basis: 0
    convex_hull_subdivision_depth: 2
    convex_hull_alm_en: true
    convex_hull_alm_warm_start_en: true
    convex_hull_alm_warm_start_accuracy: 1.0e-3
    convex_hull_alm_active_set_en: false
    convex_hull_alm_active_set_margin: 0.05
    convex_hull_adaptive_en: true
    convex_hull_refine_derivative_constraints: false
    convex_hull_alm_max_outer_iterations: 8
    convex_hull_refine_margin: 0.05
    convex_hull_alm_position_scale: 0.25
    convex_hull_alm_initial_penalty: 3.0e+5
    convex_hull_alm_penalty_growth: 5.0
    convex_hull_alm_progress_ratio: 0.5
    convex_hull_alm_constraint_tolerance: 1.0e-2
    convex_hull_alm_require_certification: true
```

Setting `convex_hull_alm_en: false` retains the fixed-depth smooth-penalty
baseline. Setting `convex_hull_en: false` retains the original dense numerical
integral. Exploration and SE3 optimizers do not read these state2state options.

## Short plan-only comparison

A same-command, one-second click benchmark was run in `ros1_noetic`. The
requested goal was `[69.032, 1.901, 1.500]`; the running local-map guard
projected it to the same accepted goal near `[7.172, 1.901, 1.500]` in all
three runs. Each timing CSV contained 14 optimizer records, so this comparison
measures the short four-piece regime rather than the earlier full 100 m flight.

| mode | objective evaluations | LBFGS iterations | total optimization | certificate |
|---|---:|---:|---:|---:|
| dense integral | 6,125 | 3,337 | 116.111 ms | sampled only |
| fixed Bezier V2 depth2 penalty | 5,816 | 3,218 | 120.209 ms | penalty, no strict acceptance test |
| adaptive Bezier depth0/depth2 PHR-ALM | 7,433 | 3,711 | 142.854 ms | 14/14 accepted |

The final ALM run used an average of 3.86 outer iterations and its adaptive
layouts averaged 2.29 coarse and 1.71 fine source segments. Updating and
back-propagating only the selected coarse/fine representations reduced the
control functional from about 6.20 to 4.35 microseconds per objective
evaluation. Closed-loop convergence varied, however: this run made more LBFGS
evaluations and was about 18.8% slower than fixed depth2 and 23.0% slower than
dense sampling in this short-trajectory case. This is the current honest
performance boundary: the constrained formulation is functional and
certifying, while the next optimization target is batching active half-space
constraints and using an inexact-to-accurate inner LBFGS tolerance schedule.

## Full-goal PHR optimization experiment

The full closed-loop goal `[69.032, 1.901, 1.500]` was used to isolate why the
first PHR version was slow and to test the corrections. These runs are not
cycle-identical: real-time replanning changes the number and geometry of later
subproblems, so both total time and kernel counters are shown.

| variant | optimizer calls | evaluations | mean inner solves | total optimization |
|---|---:|---:|---:|---:|
| initial adaptive PHR | 195 | 146,532 | n/a | 3180.546 ms |
| corrected PHR + loose V2 warm start (best observed run) | 157 | 101,522 | 3.268 | 2259.901 ms |
| experimental constraint active set | 145 | 181,070 | 4.628 | 3870.718 ms |
| warm-start accuracy `1e-4` | 153 | 114,145 | n/a | 2883.486 ms |
| initial penalty `1e6` | 155 | 112,578 | 2.865 | 2901.369 ms |
| final configuration confirmation | 161 | 102,208 | 2.988 | 2655.911 ms |

The final confirmation certified 138 of 161 attempts, used on average 1.075
coarse and 2.938 fine source pieces, and spent 511.360 ms in the penalty warm
start, 973.868 ms in the control-point functional, 2511.680 ms in complete
MINCO evaluations, and 2644.468 ms in LBFGS. Relative to the initial PHR run,
the final total is 16.5% lower; the best observed run was 28.9% lower. It is
still slower than the historical fixed-depth V2 penalty result of 1869.104 ms,
which does not apply the same strict post-solve certificate.

The experiment rules out convex conversion as the primary bottleneck. The
active-set version reduced constraints per subproblem but invalidated the
LBFGS model whenever constraints were appended, nearly doubling evaluations.
Likewise, a tighter warm solve moved work into the non-certifying stage, and a
larger initial penalty worsened conditioning. The retained configuration is
therefore: V2 warm-start accuracy `1e-3`, initial PHR penalty `3e5`, violation-
driven adaptive depth zero/two, active set off, and at least one PHR inner
solve. Further speedups should target inner-solve continuation or multiplier
reuse across replans while preserving a fixed constraint topology, rather than
adding more control-point pruning.

## Paired convergence benchmark

`convex_hull_convergence_benchmark` removes closed-loop replanning variance.
Six deterministic seventh-degree MINCO problems use exactly the same boundary
states, initial durations, inner waypoints, corridor and LBFGS parameters for
the dense, depth-two V1 and depth-two V2 objectives. Every accepted iteration
is checked by 4096 time samples and by a depth-six Bezier certificate. The
certificate monitor time is excluded from the reported solver time.

| mode | evaluations / problem | iterations / problem | line searches / iteration | evaluation | solver / problem |
|---|---:|---:|---:|---:|---:|
| dense | 30.167 | 28.167 | 1.036 | 8.709 us | 0.271 ms |
| depth-two V1 | 33.333 | 29.667 | 1.090 | 8.732 us | 0.301 ms |
| depth-two V2 | 33.333 | 29.667 | 1.090 | 8.380 us | 0.289 ms |

V1 and V2 have identical evaluations, accepted iterations and line-search
counts on every paired problem, as required by their algebraically identical
objective and gradient. V2 reduces evaluation cost but cannot improve
convergence. In these problems depth-two V2 evaluates about 3.8% faster than
dense, but needs about 10.5% more evaluations, leaving the complete solve about
6.7% slower. All three modes reach the common sampled and certified tolerance;
there is no measured convergence advantage for the hull penalty in this set.

The executable also includes a deterministic continuous-safety probe. For
`y(u)=4u(1-u)`, 16 uniform nodes have maximum `0.995556`, while the true peak
between nodes is `1.0`. With a bound of `0.998`, dense sampling reports no
violation, whereas the depth-two Bezier hull fails the safety certificate and,
for this exact quadratic probe, exposes the actual continuous violation.
Generally, all hull controls satisfying a convex constraint is a sufficient
continuous-time safety certificate; a control outside the constraint only means
that certification failed and can still be a conservative false positive. This
captures the actual benefit of the hull representation independently of whether
its penalty objective converges faster.

## Shared LBFGS fast path

The ordinary state2state `ExpTrajOpt` now has a representation-independent fast
path. It applies unchanged to dense and fixed-depth convex-hull objectives and
does not alter exploration or SE3 optimization. The implementation:

- skips the post-integral sample buffer and zero-gradient backpropagation when
  the cost manager has no discrete sample term;
- skips flatness reverse propagation when both angular-rate and thrust
  penalties are inactive at a dense node;
- reuses integral work buffers and precomputes normalized sample locations and
  trapezoid weights;
- starts from the guide trajectory timing instead of applying the old fixed
  `0.8` duration compression;
- stops LBFGS only after accepted objective, decision and penalty-log changes
  are jointly stable; and
- restarts the original LBFGS configuration from the initial decision if the
  optimistic fast solve encounters a numerical error.

A 25-second closed-loop comparison used the historical goal
`[69.032, 1.901, 1.500]` and dense sampling in both runs:

| metric | original LBFGS dense | fast LBFGS dense | change |
|---|---:|---:|---:|
| optimizer calls | 134 | 144 | +7.46% |
| objective evaluations | 62,002 | 18,317 | -70.46% |
| evaluations / call | 462.701 | 127.201 | -72.51% |
| accepted iterations / call | 269.687 | 55.021 | -79.60% |
| line-search evaluations / iteration | 1.712 | 2.294 | +34.00% |
| accumulated LBFGS time | 1130.671 ms | 303.145 ms | **-73.19%** |
| LBFGS time / call | 8.438 ms | 2.105 ms | -75.05% |
| planner path length | 103.021 m | 103.360 m | +0.33% |
| `ExpTrajOpt` failures | 0 | 0 | unchanged |

The measured end-to-end optimization speedup is `3.73x`. All 144 optimized
calls satisfied the fast-stop stability test, and none needed the numerical
fallback. A time-step line-search bound was tested but deliberately disabled:
it increased line-search trials enough to offset its protection. The speedup is
therefore dominated by avoiding over-solving after the trajectory has
stabilized, with a smaller contribution from the equivalent dense-kernel
changes.

### Fast depth-two V2 comparison

The same 25-second closed-loop goal and fast-LBFGS parameters were then used
with the fixed Bezier depth-two V2 functional. The original dense row is kept
as the common baseline:

| metric | original dense | fast dense | fast depth2 V2 |
|---|---:|---:|---:|
| optimizer calls | 134 | 144 | 138 |
| objective evaluations | 62,002 | 18,317 | 19,556 |
| evaluations / call | 462.701 | 127.201 | 141.710 |
| accepted iterations / call | 269.687 | 55.021 | 62.536 |
| line-search evaluations / iteration | 1.712 | 2.294 | 2.250 |
| accumulated LBFGS time | 1130.671 ms | 303.145 ms | 364.646 ms |
| LBFGS time / call | 8.438 ms | 2.105 ms | 2.642 ms |
| complete MINCO evaluation time | 1016.101 ms | 298.387 ms | 358.993 ms |
| residual dense functional time | 837.131 ms | 246.951 ms | 170.447 ms |
| control-point functional time | 0 ms | 0 ms | 130.004 ms |
| planner path length | 103.021 m | 103.360 m | 102.446 m |
| `ExpTrajOpt` failures | 0 | 0 | 0 |

Fast depth-two V2 is `3.10x` faster than the original dense solve in
accumulated optimization time, demonstrating that the shared fast path also
works for the hull objective. It is nevertheless 20.3% slower in total and
25.5% slower per optimizer call than fast dense. Its evaluations cost about
12.7% more and it performs 11.4% more evaluations per call. The
control-point functional accounts for 35.65% of its optimization time, while
the retained dense terms account for another 46.74%. Thus the remaining gap is
not Bezier basis conversion: it is the extra control-point penalty work and the
different objective geometry, which requires more accepted iterations.

The different optimizer-call counts are normal closed-loop timing variance, so
the per-call and per-evaluation columns are the strict kernel comparison. The
hull construction still has a distinct mathematical purpose: when all Bezier
controls satisfy a convex corridor, it supplies a sufficient continuous-time
certificate, whereas dense nodes only certify the sampled instants. The
current V2 path is a smooth penalty formulation and does not yet impose a
strict final certificate acceptance test.
