#!/usr/bin/env python3
"""Plot J vs iteration for Euclidean vs natural L-BFGS."""

from __future__ import annotations

import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt


def load(path: str):
    series = defaultdict(lambda: ([], []))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            key = row.get("name", row.get("series"))
            series[key][0].append(int(row["iter"]))
            series[key][1].append(float(row["J"]))
    return series


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "coordinate_opt_speed_traces.csv")
    if not os.path.isfile(path):
        print("missing", path)
        return 1
    s = load(path)
    fig, axes = plt.subplots(1, 3, figsize=(13.8, 4.3), constrained_layout=True)

    def plot(ax, keys, title, ylabel=True):
        styles = {
            "poly_eucl_power": ("#1f4e79", "-", "Euclidean Power"),
            "poly_eucl_bernstein": ("#c00000", "-", "Euclidean Bernstein"),
            "poly_nat_power": ("#70ad47", "--", "Natural Power"),
            "poly_nat_bernstein": ("#ed7d31", ":", "Natural Bernstein"),
            "poly_k0_eucl_power": ("#1f4e79", "-", "Euclidean Power"),
            "poly_k0_eucl_bernstein": ("#c00000", "-", "Euclidean Bernstein"),
            "poly_k0_nat_power": ("#70ad47", "--", "Natural Power"),
            "poly_k0_nat_bernstein": ("#ed7d31", ":", "Natural Bernstein"),
            "minco_eucl_P": ("#1f4e79", "-", "Euclidean P"),
            "minco_eucl_y": ("#c00000", "-", "Euclidean y=RP"),
            "minco_nat_P": ("#70ad47", "--", "Natural P"),
            "minco_nat_y": ("#ed7d31", ":", "Natural y=RP"),
        }
        for k in keys:
            if k not in s:
                continue
            xs, ys = s[k]
            c, ls, lab = styles[k]
            ax.plot(xs, ys, color=c, ls=ls, lw=2.0, label=lab)
        ax.set_title(title)
        ax.set_xlabel("L-BFGS iteration")
        if ylabel:
            ax.set_ylabel(r"$J$")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

    plot(axes[0],
         ["poly_k0_eucl_power", "poly_k0_eucl_bernstein",
          "poly_k0_nat_power", "poly_k0_nat_bernstein"],
         r"Quadratic $J=\frac{1}{2}\int p^2$")
    plot(axes[1],
         ["poly_eucl_power", "poly_eucl_bernstein",
          "poly_nat_power", "poly_nat_bernstein"],
         r"Nonconvex poly (Bernstein Euclidean: other basin)",
         ylabel=False)
    plot(axes[2],
         ["minco_eucl_P", "minco_eucl_y", "minco_nat_P", "minco_nat_y"],
         "MINCO, same physical minimizer",
         ylabel=False)

    fig.suptitle("Natural / L2-whitened L-BFGS vs Euclidean L-BFGS")
    out = os.path.join(here, "coordinate_opt_speed_traces.png")
    fig.savefig(out, dpi=160)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
