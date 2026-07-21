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
