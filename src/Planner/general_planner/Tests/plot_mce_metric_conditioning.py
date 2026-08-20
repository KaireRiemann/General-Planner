#!/usr/bin/env python3
"""Plots for MCE condition-number validation."""

from __future__ import annotations

import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np


def here() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def load_rows(name: str):
    path = os.path.join(here(), name)
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main() -> int:
    table = load_rows("mce_conditioning_table.csv")
    spec = load_rows("mce_conditioning_spectrum.csv")
    costs = load_rows("mce_conditioning_cost_traces.csv")
    scaling = load_rows("mce_conditioning_scaling.csv")

    fig, axes = plt.subplots(2, 2, figsize=(12.8, 9.0), constrained_layout=True)

    ax = axes[0, 0]
    for metric, color in [("I", "#c00000"), ("MCE", "#1f4e79")]:
        xs, ys = [], []
        for r in spec:
            if r["case"] == "Pure-MCE" and r["metric"] == metric:
                xs.append(int(r["index"]))
                ys.append(abs(float(r["eigenvalue"])) + 1e-18)
        if xs:
            ax.semilogy(xs, ys, "o-", color=color, lw=1.8, label=metric)
    ax.set_title("Fig.1  Pure-MCE Hessian spectrum (M=5)")
    ax.set_xlabel("eigenvalue index")
    ax.set_ylabel(r"$|\lambda|$")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()

    ax = axes[0, 1]
    for metric, color, ls in [("I", "#c00000", "-"), ("MCE", "#1f4e79", "--")]:
        Ms, ks = [], []
        for r in scaling:
            if r["time_pattern"] == "uniform" and r["metric"] == metric:
                Ms.append(int(r["M"]))
                ks.append(float(r["kappa_G"] if metric == "MCE" else r["kappa_E"]))
        order = np.argsort(Ms)
        ax.semilogy(np.array(Ms)[order], np.array(ks)[order], "o-",
                    color=color, ls=ls, lw=2.0, label=metric)
    ax.set_title("Fig.3  Condition number vs piece count")
    ax.set_xlabel("M (pieces)")
    ax.set_ylabel(r"$\kappa$")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()

    ax = axes[1, 0]
    patterns = []
    ke, kg = [], []
    for r in scaling:
        if r["M"] == "5" and r["metric"] == "I":
            patterns.append(r["time_pattern"])
            ke.append(float(r["kappa_E"]))
        if r["M"] == "5" and r["metric"] == "MCE":
            kg.append(float(r["kappa_G"]))
    x = np.arange(len(patterns))
    w = 0.35
    ax.bar(x - w / 2, ke, w, color="#c00000", label=r"$\kappa_E$")
    ax.bar(x + w / 2, kg, w, color="#1f4e79", label=r"$\kappa_{\mathrm{MCE}}$")
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(patterns)
    ax.set_title("Fig.4  Segment-time ratio (M=5)")
    ax.set_ylabel(r"$\kappa$")
    ax.grid(True, axis="y", which="both", alpha=0.3)
    ax.legend()

    ax = axes[1, 1]
    for r in table:
        if int(r["iters"]) <= 0:
            continue
        kappa = float(r["kappa_G"])
        it = int(r["iters"])
        color = "#1f4e79" if r["metric"] != "I" else "#c00000"
        marker = "o" if r["metric"] != "I" else "s"
        ax.scatter(kappa, it, c=color, marker=marker, s=36, alpha=0.85)
    ax.set_xscale("log")
    ax.set_title(r"Fig.2  $\kappa_G$ vs L-BFGS iterations")
    ax.set_xlabel(r"$\kappa_G$")
    ax.set_ylabel(r"$N_{\mathrm{iter}}$")
    ax.grid(True, which="both", alpha=0.3)

    out1 = os.path.join(here(), "mce_conditioning_overview.png")
    fig.savefig(out1, dpi=150)

    fig2, ax = plt.subplots(figsize=(8.2, 4.6), constrained_layout=True)
    styles = {
        ("Planner-like", "I"): ("#c00000", "-", "Euclidean I"),
        ("Planner-like", "L2"): ("#ed7d31", "-", "L2"),
        ("Planner-like", "H2"): ("#7030a0", "-", "H2"),
        ("Planner-like", "MCE"): ("#1f4e79", "--", "MCE"),
        ("Planner-like", "Mix"): ("#70ad47", ":", "Mix L2+MCE"),
        ("Planner-like", "MCE+GN"): ("#00b0f0", "-.", "MCE+GN"),
    }
    series = defaultdict(list)
    for r in costs:
        key = (r["case"], r["metric"])
        series[key].append((int(r["iter"]), float(r["J"])))
    for key, sty in styles.items():
        if key not in series:
            continue
        pts = sorted(series[key])
        ax.semilogy([p[0] for p in pts],
                    [max(p[1] - 39.9, 1e-8) for p in pts],
                    color=sty[0], ls=sty[1], lw=2.0, label=sty[2])
    ax.set_title("Fig.5  Planner-like frozen-metric L-BFGS")
    ax.set_xlabel("iteration")
    ax.set_ylabel(r"$J_k - 39.9$ (offset for log scale)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    out2 = os.path.join(here(), "mce_conditioning_cost_curves.png")
    fig2.savefig(out2, dpi=150)
    print("wrote", out1)
    print("wrote", out2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
