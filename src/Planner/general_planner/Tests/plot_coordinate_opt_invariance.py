#!/usr/bin/env python3
"""Plot iter-1 trajectories of Euclidean vs natural L-BFGS in two coordinates."""

from __future__ import annotations

import csv
import os
import sys

import matplotlib.pyplot as plt


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "coordinate_opt_iter1_trajectories.csv")
    if not os.path.isfile(path):
        print("CSV not found. Run coordinate_opt_invariance_self_test first.",
              file=sys.stderr)
        return 1
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    t = [float(r["t"]) for r in rows]
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 4.5), constrained_layout=True)
    ax = axes[0]
    ax.plot(t, [float(r["p_eucl_power"]) for r in rows], color="#1f4e79", lw=2.2,
            label="Power Euclidean")
    ax.plot(t, [float(r["p_eucl_bernstein"]) for r in rows], color="#c00000", lw=2.0,
            label="Bernstein Euclidean")
    ax.set_title("After 1 Euclidean L-BFGS step")
    ax.set_xlabel(r"$t$")
    ax.set_ylabel(r"$p(t)$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[1]
    ax.plot(t, [float(r["p_nat_power"]) for r in rows], color="#1f4e79", lw=2.4,
            label="Power Natural")
    ax.plot(t, [float(r["p_nat_bernstein"]) for r in rows], color="#70ad47", lw=2.0,
            ls="--", label="Bernstein Natural")
    ax.set_title("After 1 L2-whitened L-BFGS step")
    ax.set_xlabel(r"$t$")
    ax.set_ylabel(r"$p(t)$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    fig.suptitle("Same J, same p0: Euclidean L-BFGS splits; natural L-BFGS does not")
    out = os.path.join(here, "coordinate_opt_iter1_trajectories.png")
    fig.savefig(out, dpi=160)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
