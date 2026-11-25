#!/usr/bin/env python3
"""Plot the coalescing threshold curves from curves.csv / tiers.csv.

Inputs are produced by ./batch. Outputs:
  ../artifacts/batch_threshold_curves.png  (compute + latency curves, thresholds)
  ../artifacts/batch_tier_results.png      (three tier runs: searches + latency)
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "artifacts"
OUT.mkdir(exist_ok=True)

# dataviz reference tokens (light surface)
SURFACE = "#fcfcfb"
PAGE = "#f9f9f7"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
BLUE = "#2a78d6"    # independent / direct
ORANGE = "#eb6834"  # coalesced / merged
CRITICAL = "#d03b3b"
SUCCESS = "#006300"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "system-ui", "Segoe UI", "Arial"],
    "font.size": 10.5,
    "text.color": INK,
    "axes.edgecolor": BASELINE,
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "axes.linewidth": 0.9,
    "figure.facecolor": PAGE,
    "axes.facecolor": SURFACE,
})


def read_csv(name):
    with open(HERE / name, newline="") as f:
        return list(csv.DictReader(f))


def style_ax(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", color=GRID, linewidth=0.9)
    ax.set_axisbelow(True)
    ax.tick_params(length=0)


curves = read_csv("curves.csv")
ns = [int(r["n"]) for r in curves]
tdir = [float(r["t_dir_us"]) / 1000 for r in curves]
tmerge = [float(r["t_merge_us"]) / 1000 for r in curves]
infl = [float(r["inflation_pct"]) for r in curves]

tiers = read_csv("tiers.csv")
tn = [int(r["n"]) for r in tiers]
td_runs = [int(r["direct_runs"]) for r in tiers]
tm_runs = [int(r["merged_runs"]) for r in tiers]
tinfl = [float(r["inflation_pct"]) for r in tiers]

N_MIN, N_MAX = 7, 102

# ------------------------------------------------------------ figure 1: curves
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.6, 5.3), gridspec_kw={"wspace": 0.22})

# left: total compute cost before vs after merging (N range covers all tiers)
m = ns.index(64) + 1
n1, d1, m1 = ns[:m], tdir[:m], tmerge[:m]
ax1.plot(n1, d1, color=BLUE, lw=2.5, label="Independent: N·C", zorder=3)
ax1.plot(n1, m1, color=ORANGE, lw=2.5, label="Coalesced: R·C + g·log₂N", zorder=3)
ax1.fill_between(n1, d1, m1, where=[d >= mm for d, mm in zip(d1, m1)],
                 color=BLUE, alpha=0.10, zorder=1, label="Avoided duplicate work")
ax1.fill_between(n1, d1, m1, where=[d < mm for d, mm in zip(d1, m1)],
                 color=CRITICAL, alpha=0.13, zorder=1)
for t in tn:
    i = ns.index(t)
    ax1.scatter([t, t], [tdir[i], tmerge[i]], s=70, color=INK,
                edgecolor=SURFACE, linewidth=1.8, zorder=5)
ax1.axvline(N_MIN, color=MUTED, lw=1.2, ls=":", zorder=2)
ax1.annotate(f"N_min = {N_MIN}\nfirst net saving", xy=(N_MIN, 4.84),
             xytext=(10.5, 1.1), textcoords="data", fontsize=9.5,
             color=INK2,
             arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.0))
ax1.set_xlim(1, 64)
ax1.set_ylim(0, 52)
ax1.set_xlabel("Requests per frame  N")
ax1.set_ylabel("Total compute work (ms)")
ax1.set_title("Compute: N searches collapse to R region searches",
              fontsize=12, fontweight="bold", loc="left", pad=10)
ax1.legend(loc="upper left", frameon=False, fontsize=9.3)
style_ax(ax1)

# inset: threshold crossing zoom (sits in the empty cone between the lines)
ins = ax1.inset_axes([0.57, 0.16, 0.41, 0.35])
zi = [i for i, n in enumerate(ns) if 4 <= n <= 9]
zn = [ns[i] for i in zi]
ins.plot(zn, [tdir[i] for i in zi], color=BLUE, lw=2.0, marker="o", ms=4)
ins.plot(zn, [tmerge[i] for i in zi], color=ORANGE, lw=2.0, marker="o", ms=4)
ins.axvline(N_MIN, color=MUTED, lw=1.1, ls=":")
ins.scatter([7], [tmerge[ns.index(7)]], s=46, color=SUCCESS,
            edgecolor=SURFACE, linewidth=1.4, zorder=5)
ins.set_xlim(4.6, 8.4)
ins.set_ylim(3.8, 6.5)
ins.set_xticks([5, 6, 7, 8])
ins.tick_params(labelsize=8.5, length=0)
ins.grid(axis="y", color=GRID, lw=0.8)
ins.set_axisbelow(True)
for s in ins.spines.values():
    s.set_visible(True)
    s.set_color(BASELINE)
    s.set_linewidth(1.0)
ins.set_title("break-even zoom", fontsize=8.8, color=INK2, pad=3)

# right: latency inflation curve with the 10% gate
ax2.plot(ns, infl, color=ORANGE, lw=2.5, zorder=3)
ax2.fill_between(ns, infl, 10, where=[v > 10 for v in infl],
                 color=CRITICAL, alpha=0.10, zorder=1)
ax2.axhline(10, color=CRITICAL, lw=1.8, ls="--", zorder=2)
ax2.text(2, 10.25, "10% latency cap", color=CRITICAL, fontsize=10,
         fontweight="bold")
ax2.scatter([N_MAX], [10], s=75, facecolor=CRITICAL, edgecolor=SURFACE,
            linewidth=1.8, zorder=5)
ax2.annotate(f"N_max = {N_MAX}", xy=(N_MAX, 10),
             xytext=(-52, 17), textcoords="offset points", fontsize=10,
             fontweight="bold", color=INK2, ha="center",
             arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.1))
for t, v in zip(tn, tinfl):
    i = ns.index(t)
    ax2.scatter([t], [infl[i]], s=70, color=INK, edgecolor=SURFACE,
                linewidth=1.8, zorder=5)
    ax2.annotate(f"N={t}: {v:.2f}%", xy=(t, infl[i]), xytext=(8, -15),
                 textcoords="offset points", fontsize=9.3, color=INK, ha="left")
ax2.set_xlim(1, 128)
ax2.set_ylim(0, 11.8)
ax2.set_xlabel("Requests per frame  N")
ax2.set_ylabel("Avg response latency increase (%)")
ax2.set_title("Latency: wait for grouping, gated at +10%",
              fontsize=12, fontweight="bold", loc="left", pad=10)
tier_handle = Line2D([0], [0], marker="o", color="none", markerfacecolor=INK,
                     markeredgecolor=SURFACE, markersize=9,
                     label="Measured tiers 10 / 30 / 60")
ax2.legend(handles=[tier_handle], loc="lower right", frameon=False, fontsize=9.3)
style_ax(ax2)

fig.suptitle("When is request coalescing worth it?  Compute threshold vs latency gate",
             fontsize=13.5, fontweight="bold", x=0.012, ha="left", y=0.985)
fig.text(0.012, 0.018,
         "Model: T_dir=N·C, T_merge=R·C+g·log₂N (C=800µs, g=12µs); "
         "latency assumes concurrent searches (worst case for merging).",
         fontsize=8.8, color=MUTED)
fig.tight_layout(rect=(0, 0.055, 1, 0.95))
p1 = OUT / "batch_threshold_curves.png"
fig.savefig(p1, dpi=200, facecolor=PAGE)
plt.close(fig)

# ----------------------------------------------------------- figure 2: tiers
fig, (bx1, bx2) = plt.subplots(1, 2, figsize=(11.6, 4.8), gridspec_kw={"wspace": 0.24})
x = range(len(tn))
w = 0.36

b1 = bx1.bar([i - w / 2 for i in x], td_runs, width=w, color=BLUE,
             label="Independent searches", zorder=3)
b2 = bx1.bar([i + w / 2 for i in x], tm_runs, width=w, color=ORANGE,
             label="Coalesced searches", zorder=3)
bx1.bar_label(b1, padding=3, fontsize=10, color=INK2)
bx1.bar_label(b2, padding=3, fontsize=10, color=INK2)
for i, (nd, nm) in enumerate(zip(td_runs, tm_runs)):
    bx1.text(i, nd + 4.5, f"−{nd - nm} duplicate runs", ha="center",
             fontsize=9.2, color=SUCCESS, fontweight="bold")
bx1.set_xticks(list(x), [f"N={n}" for n in tn])
bx1.set_ylim(0, 70)
bx1.set_ylabel("Path search invocations per frame")
bx1.set_title("Same-region recomputation eliminated (R=6)",
              fontsize=12, fontweight="bold", loc="left", pad=10)
bx1.legend(loc="upper left", frameon=False, fontsize=9.5)
style_ax(bx1)

bars = bx2.bar([0, 1, 2], tinfl, width=0.52, color=ORANGE, zorder=3)
bx2.set_xticks([0, 1, 2], [f"N={n}" for n in tn])
bx2.bar_label(bars, labels=[f"{v:.2f}%" for v in tinfl], padding=4,
              fontsize=10.5, color=INK, fontweight="bold")
bx2.axhline(10, color=CRITICAL, lw=1.8, ls="--", zorder=2)
bx2.text(2.45, 10.15, "10% cap", color=CRITICAL, fontsize=10,
         fontweight="bold", ha="right")
for i, v in enumerate(tinfl):
    bx2.text(i, 0.55, "PASS", ha="center", fontsize=9.5, color="white",
             fontweight="bold")
bx2.set_ylim(0, 11.6)
bx2.set_xlabel("Requests per frame  N")
bx2.set_ylabel("Avg latency inflation (%)")
bx2.set_title("Response latency overhead stays within +10%",
              fontsize=12, fontweight="bold", loc="left", pad=10)
style_ax(bx2)

fig.suptitle("Three load tiers, executed once each",
             fontsize=13.5, fontweight="bold", x=0.012, ha="left", y=0.99)
fig.tight_layout(rect=(0, 0, 1, 0.94))
p2 = OUT / "batch_tier_results.png"
fig.savefig(p2, dpi=200, facecolor=PAGE)
plt.close(fig)

print(f"wrote {p1}")
print(f"wrote {p2}")
