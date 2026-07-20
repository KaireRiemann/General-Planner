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

Create one `Representation<DIM>` for each required derivative order, and call
`resetTopology()` only when the number of segments, polynomial order, basis, or
subdivision depth changes. In a cost manager, implement the optional callback

```cpp
double evaluateCoefficient(const Trajectory &trajectory,
                           CoeffMat &grad_coefficients,
                           Eigen::VectorXd &grad_durations) const;
```

Within it:

1. update the cached representation with `trajectory.updateConvexHull(hull)`;
2. evaluate corridor or derivative-bound penalties on `hull.controls()`;
3. accumulate `dJ/dQ`;
4. call `hull.backwardAdd(dJ_dQ, grad_coefficients, grad_durations)`.

For a convex safe corridor, requiring every position control point of a leaf to
lie in that leaf's half-space polytope certifies the whole continuous leaf. For
a convex velocity or acceleration bound (for example, a Euclidean ball),
requiring every derivative control point to satisfy the bound likewise
certifies the continuous derivative. Increasing subdivision depth tightens each
local hull while preserving this guarantee.

Bezier and MINVO conversion data and licensing are documented in
`THIRD_PARTY_NOTICES.md`.
