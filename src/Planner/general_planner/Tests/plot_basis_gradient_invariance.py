#!/usr/bin/env python3
"""Plot trajectory-tangent comparison from the basis-invariance self-test CSVs."""

from __future__ import annotations

import csv
import os
import sys

import matplotlib.pyplot as plt


def load_csv(path: str):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"empty csv: {path}")
    cols = reader.fieldnames
    data = {c: [float(r[c]) for r in rows] for c in cols}
    return data


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    quad = os.path.join(here, "basis_invariance_quadratic_tangents.csv")
    minco = os.path.join(here, "basis_invariance_minco_tangents.csv")
    if not os.path.isfile(quad) or not os.path.isfile(minco):
        print("CSV files not found. Run basis_gradient_invariance_self_test first.",
              file=sys.stderr)
        return 1

    q = load_csv(quad)
    m = load_csv(minco)

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6), constrained_layout=True)

    ax = axes[0]
    ax.plot(q["t"], q["delta_p_power_E"], color="#1f4e79", lw=2.4,
            label=r"Power Euclidean  $\delta p_{\mathrm{Power}}^{E}(t)$")
    ax.plot(q["t"], q["delta_p_bernstein_N"], color="#70ad47", lw=2.0, ls="--",
            label=r"Bernstein Natural  $\delta p_{\mathrm{Bernstein}}^{N}(t)$")
    ax.plot(q["t"], q["delta_p_bernstein_E"], color="#c00000", lw=2.0,
            label=r"Bernstein Euclidean  $\delta p_{\mathrm{Bernstein}}^{E}(t)$")
    ax.set_title("Level 2A  Quadratic Power ↔ Bernstein")
    ax.set_xlabel(r"$t$")
    ax.set_ylabel(r"trajectory tangent  $\delta p(t)$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    ax = axes[1]
    ax.plot(m["t"], m["delta_p_P_euclidean_x"], color="#1f4e79", lw=2.4,
            label=r"Waypoint Euclidean  $\delta p_{P}^{E}(t)$")
    ax.plot(m["t"], m["delta_p_y_natural_x"], color="#70ad47", lw=2.0, ls="--",
            label=r"Reparam. Natural  $\delta p_{y}^{N}(t)$")
    ax.plot(m["t"], m["delta_p_y_euclidean_x"], color="#c00000", lw=2.0,
            label=r"Reparam. Euclidean  $\delta p_{y}^{E}(t)$")
    ax.set_title("Level 3  MINCO reduced manifold (x-component)")
    ax.set_xlabel(r"$t$")
    ax.set_ylabel(r"trajectory tangent  $\delta p_x(t)$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    out = os.path.join(here, "basis_invariance_tangents.png")
    fig.suptitle("Euclidean gradient is coordinate-dependent; natural gradient is not",
                 fontsize=12)
    fig.savefig(out, dpi=160)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
