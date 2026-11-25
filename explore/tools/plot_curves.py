#!/usr/bin/env python3
"""Plot oscillation and path-overhead curves from calibrate.py output."""

from __future__ import annotations

import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COLORS = {3: "#2a78d6", 5: "#eb6834", 7: "#1baf7a"}
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
CRITICAL = "#d03b3b"


def main() -> int:
    csv_path = Path(sys.argv[1])
    json_path = Path(sys.argv[2])
    out_path = Path(sys.argv[3])

    with json_path.open(encoding="utf-8") as f:
        summary = json.load(f)

    series: dict[int, dict[float, dict[str, float]]] = defaultdict(dict)
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            radius = int(row["radius"])
            theta = float(row["theta"])
            series[radius][theta] = row

    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [
                "Noto Sans CJK SC",
                "WenQuanYi Zen Hei",
                "system-ui",
                "-apple-system",
                "Segoe UI",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "axes.edgecolor": "#c3c2b7",
            "axes.labelcolor": INK,
            "text.color": INK,
            "xtick.color": SECONDARY,
            "ytick.color": SECONDARY,
        }
    )
    fig, (ax0, ax1) = plt.subplots(
        2, 1, figsize=(9.2, 6.2), sharex=True, facecolor=SURFACE
    )
    for ax in (ax0, ax1):
        ax.set_facecolor(SURFACE)
        ax.grid(True, axis="y", color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.spines["left"].set_color("#c3c2b7")
        ax.spines["bottom"].set_color("#c3c2b7")

    selected = float(summary["committed_theta_cells"])
    first_zero = float(summary["curve_first_all_radii_zero_theta"])

    for radius in sorted(series):
        xs = sorted(series[radius])
        osc = [int(series[radius][x]["oscillations"]) for x in xs]
        overhead = [float(series[radius][x]["path_overhead_pct"]) for x in xs]
        color = COLORS[radius]

        ax0.plot(
            xs, osc, color=color, linewidth=2.2, marker="o", markersize=4.5,
            label=f"视野半径 r={radius}",
        )
        ax1.plot(
            xs, overhead, color=color, linewidth=2.2, marker="o", markersize=4.5,
            label=f"r={radius}",
        )

    for ax in (ax0, ax1):
        ax.axvline(first_zero, color=MUTED, linestyle=":", linewidth=1.6)
        ax.axvline(selected, color=INK, linestyle="--", linewidth=1.6)

    ax0.set_ylim(0, 17.5)
    ax0.set_ylabel("横跳次数（次）")
    ax0.set_title("阈值扫描：先压横跳，再卡路径增幅", fontsize=14, fontweight="bold", loc="left")
    ax0.legend(loc="upper right", frameon=False, fontsize=10)
    ax0.annotate(
        f"曲线首次全零 θ={first_zero:g}",
        xy=(first_zero, 1.4), xytext=(8, 0), textcoords="offset points",
        ha="left", va="center", fontsize=9, color=SECONDARY,
        bbox={"boxstyle": "round,pad=0.2", "facecolor": SURFACE, "edgecolor": "none", "alpha": 0.9},
    )
    ax0.annotate(
        f"采用 θ={selected:g}",
        xy=(selected, 16.2), xytext=(8, 0), textcoords="offset points",
        ha="left", va="center", fontsize=10, fontweight="bold", color=INK,
    )

    ax1.axhline(10.0, color=CRITICAL, linestyle="-.", linewidth=1.8)
    ax1.annotate(
        "路径增幅上限 10%", xy=(0.15, 10.4), color=CRITICAL,
        fontsize=10, fontweight="bold",
    )
    ax1.set_xlabel("重选迟滞阈值 θ（格；单帧位移 = 1 格）")
    ax1.set_ylabel("路径长度增幅（%）")
    ax1.set_ylim(bottom=-0.5)
    ax1.set_xlim(left=0, right=6.2)

    # A compact table-like note for the accepted verification point.
    runs = summary["final_runs"]
    note = (
        "θ=1.5 验收：三档横跳均为 0；"
        f"路径增幅 {runs[0]['path_overhead_pct']:.1f}% / "
        f"{runs[1]['path_overhead_pct']:.1f}% / "
        f"{runs[2]['path_overhead_pct']:.1f}%"
    )
    fig.text(0.08, 0.012, note, fontsize=9.5, color=SECONDARY)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, facecolor=SURFACE)
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
