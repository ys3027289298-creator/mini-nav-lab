#!/usr/bin/env python3
"""Plot the two trade-off curves from the hotupdate sweep.

Runs ../hotupdate, parses the CSVKSWEEP block and renders two figures:
  dangling_vs_k.png  - dangling accesses vs retained generations k
  peakmem_vs_k.png   - peak live chunks vs k
One measure per axis (never dual-axis); three categorical series (tiers).
"""
import os
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "..", "hotupdate")
OUT = os.path.join(HERE, "..", "artifacts")

# validated reference categorical slots 1-3 (light mode)
COLORS = {"slow": "#2a78d6", "medium": "#eb6834", "fast": "#1baf7a"}
TIERS = ["slow", "medium", "fast"]
INK = "#0b0b0b"
MUTED = "#52514e"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "text.color": INK,
    "axes.edgecolor": BASELINE,
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
})


def load_rows():
    out = subprocess.run([BIN], check=True, capture_output=True, text=True).stdout
    rows = []
    take = False
    for line in out.splitlines():
        if line.startswith("CSVKSWEEP"):
            take = True
            continue
        if take and line.startswith("CSV,"):
            _, tier, T, k, torn, dang, peak, qmax, sw = line.split(",")
            rows.append(dict(tier=tier, T=int(T), k=int(k), torn=int(torn),
                             dang=int(dang), peak=int(peak), qmax=int(qmax),
                             sw=int(sw)))
    if not rows:
        sys.exit("no CSV rows found - run the C binary first")
    return rows


def style_ax(ax):
    ax.axhline(0, color=BASELINE, linewidth=1.0, zorder=1)
    ax.set_xlabel("retained old generations  k")
    ax.set_xticks(range(0, 5))
    ax.legend(frameon=False, title="update tier (period T)", title_fontsize=9)


def plot(rows, metric, ylabel, title, fname, annotate_safe):
    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    kstars = {}
    for tier in TIERS:
        rs = sorted((r for r in rows if r["tier"] == tier), key=lambda r: r["k"])
        xs = [r["k"] for r in rs]
        ys = [r[metric] for r in rs]
        ax.plot(xs, ys, marker="o", markersize=7, linewidth=2,
                color=COLORS[tier], label=f"{tier} (T={rs[0]['T']})")
        safe = next((r["k"] for r in rs if r["dang"] == 0 and r["torn"] == 0), None)
        kstars[tier] = safe
        if annotate_safe and safe is not None:
            ax.annotate(f"k*={safe}", xy=(safe, 0),
                        xytext=(safe, max(ys) * 0.18 + 1),
                        ha="center", fontsize=9, color=COLORS[tier],
                        arrowprops=dict(arrowstyle="-", color=COLORS[tier], lw=1))
    style_ax(ax)
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=12, pad=12)
    fig.tight_layout()
    path = os.path.join(OUT, fname)
    fig.savefig(path, facecolor="#fcfcfb")
    plt.close(fig)
    return path, kstars


def plot_peak(rows, kstars):
    """Peak memory depends on k alone: 16 live chunks per retained
    generation plus the shadow gen being loaded, so all tiers coincide."""
    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    rs = sorted((r for r in rows if r["tier"] == "slow"), key=lambda r: r["k"])
    xs = [r["k"] for r in rs]
    ys = [r["peak"] for r in rs]
    ax.plot(xs, ys, marker="o", markersize=8, linewidth=2,
            color=COLORS["slow"], label="peak live chunks (all tiers coincide)")
    for k, y in zip(xs, ys):
        ax.annotate(str(y), xy=(k, y), xytext=(0, 9),
                    textcoords="offset points", ha="center",
                    fontsize=9, color=MUTED)
    for tier in TIERS:
        k = kstars[tier]
        y = next(r["peak"] for r in rs if r["k"] == k)
        ax.scatter([k], [y], s=170, facecolors="none",
                   edgecolors=COLORS[tier], linewidths=2.2, zorder=5)
        ax.annotate(f"{tier} k*={k}", xy=(k, y), xytext=(k - 0.35, y - 13),
                    fontsize=9, color=COLORS[tier])
    ax.axhline(0, color=BASELINE, linewidth=1.0, zorder=1)
    ax.set_xlabel("retained old generations  k")
    ax.set_xticks(range(0, 5))
    ax.set_ylabel("peak live chunks")
    ax.set_ylim(0, 105)
    ax.set_title("Cost curve: memory peak vs retained generations\n"
                 "rings mark the k* each update tier settles for",
                 fontsize=12, pad=12)
    ax.grid(True, color=GRID, linewidth=0.8)
    fig.tight_layout()
    path = os.path.join(OUT, "peakmem_vs_k.png")
    fig.savefig(path, facecolor="#fcfcfb")
    plt.close(fig)
    return path


def main():
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        subprocess.run(["make", "-C", os.path.join(HERE, "..")], check=True)
    rows = load_rows()
    p1, kstars = plot(rows, "dang", "dangling accesses (whole run)",
                      "Safety curve: dangling accesses vs delay before reclaim",
                      "dangling_vs_k.png", True)
    p2 = plot_peak(rows, kstars)

    # table view (relief for the sub-3:1 aqua contrast warning)
    tbl = os.path.join(OUT, "sweep_table.csv")
    with open(tbl, "w") as f:
        f.write("tier,T,k,torn,dangling,peak_chunks,qmax,switches\n")
        for r in rows:
            f.write(f"{r['tier']},{r['T']},{r['k']},{r['torn']},{r['dang']},"
                    f"{r['peak']},{r['qmax']},{r['sw']}\n")
    print("wrote", p1)
    print("wrote", p2)
    print("wrote", tbl)
    print("measured k*:", kstars)


if __name__ == "__main__":
    main()
