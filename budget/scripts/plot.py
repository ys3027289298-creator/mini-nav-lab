#!/usr/bin/env python3
"""Plot the two policy curves (success rate, path error) and the
expansion-vs-distance relation from results/queries.csv.

Run from the budget/ directory:  python3 scripts/plot.py
Pure stdlib + matplotlib; reads only the CSVs the C benchmark writes.
"""
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# --- validated categorical palette (dataviz skill, light mode) ---
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
RED, INK, INK2, MUTED, GRID, SURFACE = "#d03b3b", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"
TIERS = ["sparse_rho0.10", "dense_rho0.25", "maze"]
SHORT = {"sparse_rho0.10": "sparse rho=0.10",
         "dense_rho0.25": "dense rho=0.25", "maze": "maze rho=0.50"}
TCOL = {"sparse_rho0.10": BLUE, "dense_rho0.25": ORANGE, "maze": AQUA}
RADII = [48, 96, 192, 256]
BUCKETS = [(1, 150), (150, 400), (400, 700), (700, 1024)]

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 10,
    "text.color": INK,
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": INK2,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.axisbelow": True,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
})


def load_queries():
    rows = []
    with open("results/queries.csv", newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: (int(v) if k != "tier" else v)
                         for k, v in r.items()})
    return rows


def load_cap():
    with open("results/run.log") as f:
        m = re.search(r"cap_safe=(\d+).*policy_radius=(\d+)", f.read())
    return int(m.group(1)), int(m.group(2))


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def fig1_success(rows):
    """Curve A: pathfinding success rate.
    Left: windowed fine A* vs window radius. Right: coarse block graph."""
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.2), dpi=150)

    for t in TIERS:
        tr = [r for r in rows if r["tier"] == t]
        ys = [sum(r[f"r{rad}_ok"] for r in tr) / len(tr) for rad in RADII]
        a.plot(RADII, ys, marker="o", markersize=7, lw=2, color=TCOL[t],
               label=SHORT[t])
    a.set_xlabel("window radius r (cells, Chebyshev)")
    a.set_ylabel("success rate")
    a.set_title("Windowed fine A*: success vs radius", color=INK, fontsize=11)
    a.set_ylim(-0.03, 1.08)
    a.set_xticks(RADII)
    a.legend(frameon=False, fontsize=9, loc="upper left")
    style(a)

    x = range(len(TIERS))
    w = 0.36
    s8 = [sum(r["c8_ok"] for r in rows if r["tier"] == t)
          / sum(1 for r in rows if r["tier"] == t) for t in TIERS]
    s16 = [sum(r["c16_ok"] for r in rows if r["tier"] == t)
           / sum(1 for r in rows if r["tier"] == t) for t in TIERS]
    b.bar([i - w / 2 for i in x], s8, w, label="B=8 (64x64 graph)", color=BLUE)
    b.bar([i + w / 2 for i in x], s16, w, label="B=16 (32x32 graph)", color=ORANGE)
    for i, (v8, v16) in enumerate(zip(s8, s16)):
        b.text(i - w / 2, v8 + 0.02, f"{v8:.2f}", ha="center", fontsize=8.5, color=INK2)
        b.text(i + w / 2, v16 + 0.02, f"{v16:.2f}", ha="center", fontsize=8.5, color=INK2)
    b.set_xticks(list(x), [SHORT[t].replace(" ", "\n", 1) for t in TIERS], fontsize=9)
    b.set_ylabel("success rate")
    b.set_ylim(0, 1.12)
    b.set_title("Coarse block A*: success by block size", color=INK, fontsize=11)
    b.legend(frameon=False, fontsize=9)
    style(b)
    fig.tight_layout()
    fig.savefig("results/fig1_success.png", bbox_inches="tight")
    plt.close(fig)


def fig2_error(rows):
    """Curve B: path error of the coarse graph vs the full-A* reference.
    Left: deviation in cells. Right: line-of-sight share of coarse segments."""
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.2), dpi=150)
    x = range(len(TIERS))
    w = 0.36

    med8, med16 = [], []
    for t in TIERS:
        d8 = sorted(r["c8_dev"] for r in rows if r["tier"] == t and r["c8_ok"])
        d16 = sorted(r["c16_dev"] for r in rows if r["tier"] == t and r["c16_ok"])
        med8.append(d8[len(d8) // 2]); med16.append(d16[len(d16) // 2])
    a.bar([i - w / 2 for i in x], med8, w, label="B=8 median", color=BLUE)
    a.bar([i + w / 2 for i in x], med16, w, label="B=16 median", color=ORANGE)
    for i in x:
        a.text(i - w / 2, med8[i] + 45, f"{med8[i]}", ha="center",
               fontsize=9, color=INK2)
        a.text(i + w / 2, med16[i] + 45, f"{med16[i]}", ha="center",
               fontsize=9, color=INK2)
    a.set_xticks(list(x), [SHORT[t].replace(" ", "\n", 1) for t in TIERS], fontsize=9)
    a.set_ylabel("median deviation from optimal path (cells)")
    a.set_title("Coarse path error: BFS distance to optimal path\n(means in results/queries.csv)",
                color=INK, fontsize=11)
    a.legend(frameon=False, fontsize=9)
    style(a)

    los8 = [sum(r["c8_los_pct"] for r in rows if r["tier"] == t and r["c8_ok"])
            / max(1, sum(1 for r in rows if r["tier"] == t and r["c8_ok"]))
            for t in TIERS]
    los16 = [sum(r["c16_los_pct"] for r in rows if r["tier"] == t and r["c16_ok"])
             / max(1, sum(1 for r in rows if r["tier"] == t and r["c16_ok"]))
             for t in TIERS]
    b.bar([i - w / 2 for i in x], los8, w, label="B=8 LOS %", color=AQUA)
    b.bar([i + w / 2 for i in x], los16, w, label="B=16 LOS %", color="#6250d6")
    for i in x:
        b.text(i - w / 2, los8[i] + 1.5, f"{los8[i]:.0f}%", ha="center", fontsize=8.5, color=INK2)
        b.text(i + w / 2, los16[i] + 1.5, f"{los16[i]:.0f}%", ha="center", fontsize=8.5, color=INK2)
    b.set_xticks(list(x), [SHORT[t].replace(" ", "\n", 1) for t in TIERS], fontsize=9)
    b.set_ylabel("coarse segments with fine-grid LOS (%)")
    b.set_ylim(0, 108)
    b.set_title("Smoothability: segments clear on the fine map", color=INK, fontsize=11)
    b.legend(frameon=False, fontsize=9)
    style(b)
    fig.tight_layout()
    fig.savefig("results/fig2_error.png", bbox_inches="tight")
    plt.close(fig)


def fig3_expansion(rows, cap, rsel):
    """E(d, rho): mean expansions per distance bucket, with the 1ms cap."""
    fig, ax = plt.subplots(figsize=(9.5, 4.6), dpi=150)
    ends = {}
    for t in TIERS:
        xs, ys = [], []
        for lo, hi in BUCKETS:
            br = [r for r in rows if r["tier"] == t and lo <= r["dist"] < hi]
            if not br:
                continue
            xs.append(sum(r["dist"] for r in br) / len(br))
            ys.append(sum(r["full_exp"] for r in br) / len(br))
        ax.plot(xs, ys, marker="o", markersize=7, lw=2, color=TCOL[t],
                label=SHORT[t])
        ends[t] = (xs[-1], ys[-1])
    # legend already identifies the series; only label the maze end directly
    x0 = max(p[0] for p in ends.values())
    ax.annotate(SHORT["maze"], (x0, ends["maze"][1]), xytext=(10, 0),
                textcoords="offset points", va="center", color=INK2, fontsize=9)
    ax.axhline(cap, color=RED, lw=1.8, ls="--")
    ax.annotate(f"1 ms budget cap = {cap} expansions (window r={rsel})",
                (BUCKETS[0][0] + 30, cap), xytext=(0, 8),
                textcoords="offset points", color=RED, fontsize=9.5)
    ax.annotate("upper bound: whole map = 262144 cells",
                (700, 262144 * 0.92), color=MUTED, fontsize=9, ha="center")
    ax.set_xlabel("Manhattan start-goal distance d (cells)")
    ax.set_ylabel("mean A* expansions E")
    ax.set_title("Expansions vs distance and obstacle density (512x512, 4-neighbour A*)",
                 color=INK, fontsize=11)
    ax.set_ylim(0, 262144 * 1.02)
    ax.set_xlim(40, 950)
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    style(ax)
    fig.tight_layout()
    fig.savefig("results/fig3_expansion.png", bbox_inches="tight")
    plt.close(fig)


def main():
    rows = load_queries()
    cap, rsel = load_cap()
    os.makedirs("results", exist_ok=True)
    fig1_success(rows)
    fig2_error(rows)
    fig3_expansion(rows, cap, rsel)
    print("wrote results/fig1_success.png results/fig2_error.png results/fig3_expansion.png")


if __name__ == "__main__":
    main()
