#!/usr/bin/env python3
"""Plot the migration decision curves from curves.csv / results.csv.

Two rendered outputs, same data, one per theme (the dark palette is re-stepped
for the dark surface rather than flipped): curves_light.png, curves_dark.png.
The CSVs are the table-view twins of every value drawn here.
Run with -I; all inputs are paths, nothing is imported from the data dirs.
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Validated categorical slots (dataviz validate_palette.js, all checks PASS).
THEME = {
    "light": {
        "surface": "#fcfcfb", "page": "#f9f9f7",
        "ink": "#0b0b0b", "ink2": "#52514e", "muted": "#898781",
        "grid": "#e1e0d9", "axis": "#c3c2b7",
        "blue": "#2a78d6", "orange": "#eb6834",
        "good": "#006300", "crit": "#d03b3b", "fill": "#2a78d6",
    },
    "dark": {
        "surface": "#1a1a19", "page": "#0d0d0d",
        "ink": "#f0efec", "ink2": "#c3c2b7", "muted": "#898781",
        "grid": "#2c2c2a", "axis": "#383835",
        "blue": "#3987e5", "orange": "#d95926",
        "good": "#0ca30c", "crit": "#e66767", "fill": "#3987e5",
    },
}
TOL = 1.0


def load(path):
    series = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            series.setdefault(row["kind"], []).append((float(row["x"]), float(row["value"])))
    return series


def style(ax, t):
    ax.set_facecolor(t["surface"])
    for spine in ax.spines.values():
        spine.set_color(t["axis"])
        spine.set_linewidth(1.0)
    ax.tick_params(colors=t["muted"], labelsize=9)
    ax.yaxis.label.set_color(t["ink2"])
    ax.xaxis.label.set_color(t["ink2"])
    ax.title.set_color(t["ink"])
    ax.grid(True, color=t["grid"], linewidth=1.0, alpha=0.9)
    ax.set_axisbelow(True)


def make(theme_name, out, curves, tiers):
    t = THEME[theme_name]
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Noto Sans CJK SC", "WenQuanYi Zen Hei",
                            "system-ui", "DejaVu Sans", "Segoe UI"],
        "axes.unicode_minus": False,
        "figure.facecolor": t["page"],
        "axes.facecolor": t["surface"],
        "savefig.facecolor": t["page"],
        "text.color": t["ink"],
        "axes.labelcolor": t["ink2"],
        "xtick.color": t["muted"],
        "ytick.color": t["muted"],
    })
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4.6), dpi=150)

    # --- panel 1: worst-case deviation bounds vs segment count, 1% gate ------
    xb = [p[0] for p in curves["bound_budget"]]
    y_budget = [p[1] for p in curves["bound_budget"]]
    y_tight = [p[1] for p in curves["bound_tight"]]
    style(ax1, t)
    ax1.plot(xb, y_budget, color=t["orange"], lw=2,
             label="题给模型: 每段累积半格 → 恒为 50%")
    ax1.plot(xb, y_tight, color=t["blue"], lw=2,
             label="紧上界 √2·s/L: 仅两个错锚端点")
    ax1.axhline(TOL, color=t["crit"], lw=1.6)
    ax1.text(59.5, TOL + 1.4, "1% 长度预算", color=t["crit"], fontsize=9, ha="right")
    ax1.annotate("题给模型与 N 无关地恒为 50%:\n半格/段的上界只在 L 远大于 N·s 时可过闸",
                 xy=(28, 50), xytext=(1.5, 33), fontsize=8.5, color=t["ink2"],
                 arrowprops=dict(arrowstyle="->", color=t["muted"], lw=1))
    for tid, mark in (("T1", "o"), ("T2", "s"), ("T3", "^")):
        for x, y in curves[f"measured_{tid}"]:
            gate = tiers[tid]["gate_mixed"] == "PASS"
            ax1.scatter([x], [y], marker=mark, s=70, zorder=5,
                        color=t["good"] if gate else t["crit"],
                        edgecolor=t["surface"], linewidth=1.5,
                        label=f"{tid} 实测混锚 ({'PASS' if gate else 'FAIL'})")
    ax1.set_xlim(0, 60)
    ax1.set_ylim(0, 55)
    ax1.set_xlabel("路径段数 N")
    ax1.set_ylabel("长度相对偏差 (%)")
    ax1.set_title("偏差曲线: 1% 预算下的段数取舍")
    ax1.legend(fontsize=8, frameon=False, loc="upper right")

    # --- panel 2: measured accumulation as the rollout lags ------------------
    xr = [int(p[0]) for p in curves["measured_rollout"]]
    yr = [p[1] for p in curves["measured_rollout"]]
    style(ax2, t)
    ax2.plot(xr, yr, color=t["blue"], lw=2, marker="o", ms=8,
             markerfacecolor=t["blue"], markeredgecolor=t["surface"],
             markeredgewidth=1.5, label="实测偏差 (之字形路径)")
    ax2.axhline(TOL, color=t["crit"], lw=1.6)
    ax2.text(2.95, TOL - 0.09, "1% 预算", color=t["crit"], fontsize=9, ha="right")
    for x, y in zip(xr, yr):
        ax2.annotate(f"{y:.2f}%", (x, y), textcoords="offset points",
                     xytext=(0, 9), ha="center", fontsize=8, color=t["ink2"])
    ax2.set_xticks(xr)
    ax2.set_ylim(0, 1.25)
    ax2.set_xlabel("尚未迁移的错锚转弯点数 q")
    ax2.set_ylabel("长度相对偏差 (%)")
    ax2.set_title("部分迁移实测: 错锚转弯点越多偏差越大")
    ax2.legend(fontsize=8, frameon=False, loc="upper left")

    # --- panel 3: migration cost of the two conventions ----------------------
    xc = [p[0] for p in curves["cost_center"]]
    y_center = [p[1] for p in curves["cost_center"]]
    y_corner = [p[1] for p in curves["cost_corner"]]
    style(ax3, t)
    ax3.plot(xc, y_corner, color=t["blue"], lw=2, label="按格角: 零重算成本")
    ax3.plot(xc, y_center, color=t["orange"], lw=2, label="按格心: N+1 个航点全量重算")
    ax3.fill_between(xc, y_corner, y_center, color=t["fill"], alpha=0.10)
    ax3.annotate("重算工作量 = N+1 个航点", xy=(44, 45), color=t["ink2"], fontsize=9)
    ax3.set_xlim(0, 60)
    ax3.set_ylim(0, 62)
    ax3.set_xlabel("路径段数 N")
    ax3.set_ylabel("迁移成本 (需重算的航点数)")
    ax3.set_title("成本曲线: 省事但长路径超差 vs 准确但全量重算")
    ax3.legend(fontsize=8, frameon=False, loc="upper left")

    fig.suptitle("格子坐标 → 浮点世界坐标迁移: 长度偏差与迁移成本取舍 (s=1, 预算 1%)",
                 fontsize=12, color=t["ink"], y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out)
    plt.close(fig)


def main():
    curves = load(sys.argv[1])
    tiers = {}
    with open(sys.argv[2], newline="") as f:
        for row in csv.DictReader(f):
            tiers[row["tier"]] = row
    make("light", "curves_light.png", curves, tiers)
    make("dark", "curves_dark.png", curves, tiers)
    print("wrote curves_light.png curves_dark.png")


if __name__ == "__main__":
    main()
