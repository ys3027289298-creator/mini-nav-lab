#!/usr/bin/env python3
"""Analyse the hierarchical pathfinding benchmark.

Reads results/bench.csv produced by src/hier.c and:
  1. checks the acceptance gates (node ratio < 10%, length inflation <= 5%);
  2. fits the analytic node-count model n(c) = (W/c)^2 + k*L*c against the
     measured rows and solves the continuous optimum c* = (2W^2/(kL))^(1/3);
  3. emits results/summary.json and the data tables consumed by chart.html.
Run with -I as required for scripts handling benchmark output.
"""
import csv
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
W = 2048
N = W * W
CS = [8, 16, 32, 64, 128, 256]
TIERS = ["near", "mid", "far"]
NODE_GATE = 0.10
LEN_GATE = 5.0


def load_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 16 and parts[0] in TIERS:
                rows.append({
                    "tier": parts[0],
                    "c": int(parts[1]),
                    "full_len": int(parts[2]),
                    "hier_len": int(parts[3]),
                    "inflation_pct": float(parts[4]),
                    "coarse_nodes": int(parts[5]),
                    "fine_exp": int(parts[6]),
                    "hier_nodes": int(parts[7]),
                    "full_exp": int(parts[8]),
                    "node_ratio": float(parts[9]),
                    "model_ratio": float(parts[10]),
                    "coarse_exp": int(parts[11]),
                    "blocks_on_path": int(parts[12]),
                    "corridor_cells": int(parts[13]),
                    "fallback": int(parts[14]),
                    "full_ms": float(parts[15]),
                    "hier_ms": float(parts[16]),
                })
    if not rows:
        sys.exit("no benchmark rows found")
    return rows


def main():
    rows = load_rows(HERE / "results" / "bench.csv")
    by_tier = {t: [r for r in rows if r["tier"] == t] for t in TIERS}

    # ---- gate check ----
    violations = [
        f"{r['tier']} c={r['c']}: node_ratio={r['node_ratio']:.4f}"
        for r in rows if r["node_ratio"] >= NODE_GATE
    ] + [
        f"{r['tier']} c={r['c']}: inflation={r['inflation_pct']:.3f}%"
        for r in rows if r["inflation_pct"] > LEN_GATE
    ]

    # ---- analytic model per tier ----
    # measured node count minus the coarse graph term (W/c)^2 is the fine part;
    # fit fine = k*L*c through the origin (one k per tier, least squares).
    model = {}
    for t, rs in by_tier.items():
        L = rs[0]["full_len"]
        xs, ys = [], []
        for r in rs:
            xs.append(L * r["c"])
            ys.append(r["hier_nodes"] - r["coarse_nodes"])
        k = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs)
        # continuous optimum: minimise W^2/c^2 + kLc -> c* = (2W^2/(kL))^(1/3)
        c_star = (2 * N / (k * L)) ** (1 / 3)
        # best discrete c actually measured
        best = min(rs, key=lambda r: r["node_ratio"])
        # break-even: model fraction == 1  -> c^-2 term + linear term = 1
        # solve numerically over a fine grid
        grid = [x / 2 for x in range(8, 1024)]
        frac = [1 / (g * g) + k * L * g / N for g in grid]
        breakeven = next((g for g, v in zip(grid, frac) if v <= 1.0), None)
        curve = [
            {
                "c": g,
                "model_nodes": N / (g * g) + k * L * g,
                "model_frac": 1 / (g * g) + k * L * g / N,
                "coarse_frac": 1 / (g * g),
                "fine_frac": k * L * g / N,
            }
            for g in grid
        ]
        model[t] = {
            "L": L,
            "k_fit": k,
            "c_star_continuous": c_star,
            "best_measured_c": best["c"],
            "best_measured_ratio": best["node_ratio"],
            "breakeven_c": breakeven,
            "curve": curve,
        }

    # ---- measured series for the charts ----
    measured = {
        t: [
            {
                "tier": t,
                "c": r["c"],
                "node_frac": r["node_ratio"],
                "inflation_pct": r["inflation_pct"],
                "coarse_nodes": r["coarse_nodes"],
                "fine_nodes": r["hier_nodes"] - r["coarse_nodes"],
                "hier_nodes": r["hier_nodes"],
                "hier_ms": r["hier_ms"],
                "full_ms": r["full_ms"],
                "fallback": r["fallback"],
            }
            for r in by_tier[t]
        ]
        for t in TIERS
    }

    summary = {
        "world": {"W": W, "H": W, "full_graph_nodes": N},
        "gates": {
            "node_ratio_max": max(r["node_ratio"] for r in rows),
            "inflation_pct_max": max(r["inflation_pct"] for r in rows),
            "fallback_rows": sum(r["fallback"] for r in rows),
            "node_ratio_limit": NODE_GATE,
            "inflation_pct_limit": LEN_GATE,
            "pass": not violations,
            "violations": violations,
        },
        "tiers": model,
        "measured": measured,
        "formula": {
            "hier_nodes": "n(c) = (W/c)^2 + k*L*c",
            "full_nodes": "W^2",
            "ratio": "r(c) = 1/c^2 + k*L*c/W^2",
            "optimum": "c* = (2*W^2/(k*L))^(1/3)",
            "k_meaning": "corridor widening factor: per-cell path length L, "
                         "the fine search touches k*c corridor cells on average",
        },
    }
    out = HERE / "results" / "summary.json"
    out.write_text(json.dumps(summary, indent=2))
    # sibling .js include keeps chart.html working from file:// (no fetch)
    (HERE / "results" / "chart_data.js").write_text(
        "window.CHART_DATA = " + json.dumps(summary, indent=2) + ";\n"
    )

    # console report
    print(f"gates pass: {summary['gates']['pass']}")
    print(f"  worst node ratio : {summary['gates']['node_ratio_max']:.4f} "
          f"(< {NODE_GATE})")
    print(f"  worst inflation  : {summary['gates']['inflation_pct_max']:.3f}% "
          f"(<= {LEN_GATE}%)")
    print(f"  fallback rows    : {summary['gates']['fallback_rows']}")
    for t in TIERS:
        m = model[t]
        print(f"[{t}] L={m['L']:5d}  k={m['k_fit']:.3f}  "
              f"c*={m['c_star_continuous']:6.1f}  "
              f"best measured c={m['best_measured_c']:3d} "
              f"({m['best_measured_ratio']*100:.2f}% of full)  "
              f"break-even c={m['breakeven_c']}")
    print(f"\nwrote {out}")
    print(f"wrote {HERE / 'results' / 'chart_data.js'}")


if __name__ == "__main__":
    main()
