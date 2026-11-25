#!/usr/bin/env python3
"""Plot max switch jump and implementation complexity versus origin position.

Reads the CSV emitted by `./switch --csv` so the chart and self-test share the
same numerical model.
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main(csv_path: str, png_path: str) -> int:
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    size_order = []
    for row in rows:
        if row["size"] not in size_order:
            size_order.append(row["size"])

    colors = {
        "small_48x36m": "#2a78d6",
        "medium_160x120m": "#eb6834",
        "large_400x300m": "#1baf7a",
    }
    short_names = {
        "small_48x36m": "small 48×36 m",
        "medium_160x120m": "medium 160×120 m",
        "large_400x300m": "large 400×300 m",
    }

    surface = "#fcfcfb"
    grid = "#e1e0d9"
    axis = "#c3c2b7"
    ink = "#0b0b0b"
    secondary = "#52514e"
    muted = "#898781"
    critical = "#d03b3b"

    fig, (ax_jump, ax_cost) = plt.subplots(
        2,
        1,
        figsize=(9.2, 7.0),
        sharex=True,
        gridspec_kw={"height_ratios": [2.15, 1.0]},
        constrained_layout=True,
    )
    fig.patch.set_facecolor(surface)

    for ax in (ax_jump, ax_cost):
        ax.set_facecolor(surface)
        ax.grid(True, axis="y", color=grid, linewidth=1.0)
        ax.set_axisbelow(True)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["bottom"].set_color(axis)
        ax.spines["left"].set_color(axis)
        ax.tick_params(colors=secondary, labelsize=9)
        ax.yaxis.label.set_color(secondary)
        ax.xaxis.label.set_color(secondary)

    radius = float(rows[0]["radius_m"])
    budget = float(rows[0]["safety_budget_m"])

    ax_jump.axhspan(radius, 0.75, color=critical, alpha=0.08, linewidth=0)
    ax_jump.axhline(radius, color=critical, linewidth=2.0)
    ax_jump.axhline(budget, color=secondary, linewidth=1.4, linestyle=(0, (5, 3)))
    ax_jump.text(0.015, radius + 0.012, "character radius R = 0.40 m",
                 color=ink, fontsize=9, va="bottom")
    ax_jump.text(0.015, budget - 0.012, "90% safety budget = 0.36 m",
                 color=secondary, fontsize=9, va="top")
    ax_jump.text(0.02, 0.715, "infeasible band", color=secondary,
                 fontsize=9, style="italic")

    for size in size_order:
        data = [r for r in rows if r["size"] == size]
        z = [float(r["centralization"]) for r in data]
        jump = [float(r["jmax_m"]) for r in data]
        color = colors[size]
        ax_jump.plot(
            z, jump, color=color, linewidth=2.2, solid_capstyle="round",
            label=short_names[size], zorder=3,
        )
        ax_jump.plot(
            z[-1], jump[-1], marker="o", markersize=7.5,
            markerfacecolor=color, markeredgecolor=surface,
            markeredgewidth=2.0, zorder=4, clip_on=False,
        )

    ax_jump.annotate(
        "large centre: 0.350 m",
        xy=(1.0, 0.35), xytext=(0.60, 0.255),
        fontsize=9, color=ink,
        arrowprops={"arrowstyle": "->", "color": muted, "linewidth": 1.2},
    )

    z_strict = 1.0 - radius / 0.70  # large corner Jmax is 0.70 m
    z_strict = 2.0 * z_strict
    for ax in (ax_jump, ax_cost):
        ax.axvline(z_strict, color=muted, linewidth=1.2, linestyle=(0, (2, 3)))

    ax_jump.annotate(
        "large-scene strict threshold\nz > 0.857",
        xy=(z_strict, radius), xytext=(0.48, 0.57),
        fontsize=9, color=ink,
        arrowprops={"arrowstyle": "->", "color": muted, "linewidth": 1.2},
    )
    ax_jump.set_ylim(0.0, 0.75)
    ax_jump.set_xlim(0.0, 1.02)
    ax_jump.set_ylabel("worst jump  $J_{max}$ (m)", fontsize=10)
    ax_jump.set_title(
        "Origin choice: scale-error jump and implementation cost",
        loc="left", color=ink, fontsize=14, fontweight="semibold", pad=12,
    )
    legend = ax_jump.legend(
        loc="upper right", frameon=True, facecolor=surface,
        edgecolor=grid, fontsize=9,
    )
    for text in legend.get_texts():
        text.set_color(secondary)

    z_cost = [float(r["centralization"]) for r in rows if r["size"] == size_order[0]]
    cost = [float(r["complexity_index"]) for r in rows if r["size"] == size_order[0]]
    ax_cost.fill_between(z_cost, 1.0, cost, color=secondary, alpha=0.10, linewidth=0)
    ax_cost.plot(z_cost, cost, color=secondary, linewidth=2.2, solid_capstyle="round")
    labels = (
        (0.0, 1.0, "corner: lowest cost", 0.03, 1.10),
        (1.0, 3.0, "centre: selected", 0.58, 3.10),
    )
    for x, y, label, tx, ty in labels:
        ax_cost.plot(
            x, y, marker="o", markersize=7.5, markerfacecolor=secondary,
            markeredgecolor=surface, markeredgewidth=2.0, zorder=4, clip_on=False,
        )
        ax_cost.text(tx, ty, label, color=ink, fontsize=9)

    ax_cost.annotate(
        "minimum strict-safe point\nz ≈ 0.86; 90% budget forces centre",
        xy=(z_strict, 1.0 + 2.0 * z_strict), xytext=(0.14, 2.55),
        fontsize=9, color=ink,
        arrowprops={"arrowstyle": "->", "color": muted, "linewidth": 1.2},
    )
    ax_cost.set_ylim(0.8, 3.35)
    ax_cost.set_yticks([1, 2, 3])
    ax_cost.set_ylabel("complexity index", fontsize=10)
    ax_cost.set_xlabel("origin centralization z  (0 = bbox corner, 1 = bbox centre)", fontsize=10)

    fig.savefig(png_path, dpi=170, facecolor=surface, bbox_inches="tight", pad_inches=0.18)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} origin_curve.csv output.png", file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1], sys.argv[2]))
