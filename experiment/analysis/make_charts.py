#!/usr/bin/env python3
"""Render the gray-rollout decision charts from analysis.json.

Three figures, each emitted in light and dark mode against the validated
reference palette (blue #2a78d6 / orange #eb6834; dark steps swapped):
  fig1 canary_tradeoff : detection speed (required N) vs impact surface (exposure)
  fig2 latency_cdf     : empirical latency distributions behind the model
  fig3 power_curve     : rejection rate vs observation-window size at g = 10%
Every figure also gets a table-twin CSV so no value is only readable from a mark.
"""
import csv
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.ticker import FuncFormatter

ROOT = os.path.join(os.path.dirname(__file__), "..")
RESULTS = os.path.join(ROOT, "results")
FIGDIR = os.path.join(RESULTS, "figures")
BENCH = os.path.join(ROOT, "bench", "results")

# register the local CJK font explicitly
for fp in ("/home/ys/.fonts/NotoSansCJKsc-Regular.otf",
           "/home/ys/.fonts/NotoSansCJKsc-Bold.otf"):
    if os.path.exists(fp):
        font_manager.fontManager.addfont(fp)
plt.rcParams["font.family"] = ["Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

THEME = {
    "light": {
        "surface": "#fcfcfb", "page": "#f9f9f7",
        "ink": "#0b0b0b", "ink2": "#52514e", "muted": "#898781",
        "grid": "#e1e0d9", "axis": "#c3c2b7",
        "blue": "#2a78d6", "orange": "#eb6834", "gray": "#898781",
        "good": "#006300",
    },
    "dark": {
        "surface": "#1a1a19", "page": "#0d0d0d",
        "ink": "#f0efec", "ink2": "#c3c2b7", "muted": "#898781",
        "grid": "#2c2c2a", "axis": "#383835",
        "blue": "#3987e5", "orange": "#d95926", "gray": "#898781",
        "good": "#0ca30c",
    },
}


def style_ax(ax, t):
    ax.set_facecolor(t["surface"])
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(t["axis"])
        ax.spines[spine].set_linewidth(1)
    ax.grid(True, color=t["grid"], linewidth=1, axis="both", alpha=0.9)
    ax.set_axisbelow(True)
    ax.tick_params(colors=t["ink2"], labelsize=10)
    ax.title.set_color(t["ink"])
    ax.xaxis.label.set_color(t["ink2"])
    ax.yaxis.label.set_color(t["ink2"])


def save(fig, mode, name):
    path = os.path.join(FIGDIR, f"{name}_{mode}.png")
    fig.savefig(path, dpi=150, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    return path


def kfmt(v, _):
    return f"{v/1000:.1f}K" if v >= 1000 else f"{v:.0f}"


def main():
    os.makedirs(FIGDIR, exist_ok=True)
    with open(os.path.join(RESULTS, "analysis.json")) as fh:
        r = json.load(fh)
    sweep = r["sweep"]
    g = [row["g"] * 100 for row in sweep]
    n_star = [row["n_total"] for row in sweep]
    n_80 = [row["n_total_power80"] for row in sweep]
    g0 = r["inputs"]["canary_fraction"] * 100

    # ---- table twins -----------------------------------------------------
    with open(os.path.join(FIGDIR, "table1_canary_tradeoff.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["canary_pct", "exposure_pct", "n_star_2se", "n_power80",
                    "treatment_n_at_n_star", "control_n_at_n_star"])
        for row in sweep:
            w.writerow([row["g"] * 100, row["exposure_pct"],
                        round(row["n_total"], 1), round(row["n_total_power80"], 1),
                        round(row["n_treatment"], 1), round(row["n_control"], 1)])

    # latency CDF data
    pools = {}
    for key, label in (("full", "对照组：完整 A*"),
                       ("miss", "冷却组首次请求（未命中+A*+写入）"),
                       ("cache", "冷却组重复请求（缓存命中）")):
        with open(os.path.join(BENCH, f"times_{key}.csv")) as fh:
            pools[label] = sorted(int(row[0]) for row in csv.reader(fh) if row)
    with open(os.path.join(FIGDIR, "table2_latency_cdf.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        grid_ns = [0, 100, 250, 500, 1000, 2000, 4000, 8000, 16000]
        w.writerow(["latency_ns"] + list(pools.keys()))
        for x in grid_ns:
            w.writerow([x] + [round(sum(1 for v in a if v <= x) / len(a), 4)
                              for a in pools.values()])

    pc = r["power_curve"]
    with open(os.path.join(FIGDIR, "table3_power_curve.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["fraction_of_N80", "total_n", "mc_rejection_rate"])
        for row in pc:
            w.writerow([row["fraction_of_N80"], row["total_n"],
                        round(row["mc_rejection_rate"], 4)])

    paths = []
    for mode, t in THEME.items():
        # ---------- fig 1: canary decision, two stacked facets, shared x ---
        fig, (ax_n, ax_e) = plt.subplots(
            2, 1, figsize=(8.6, 6.4), sharex=True,
            gridspec_kw={"height_ratios": [2.2, 1], "hspace": 0.12})
        fig.set_facecolor(t["surface"])
        style_ax(ax_n, t)
        style_ax(ax_e, t)
        # upper facet: detection speed (required sample size), one axis only
        ax_n.plot(g, n_star, color=t["blue"], linewidth=2, marker="o",
                  markersize=7, markeredgecolor=t["surface"], markeredgewidth=2,
                  label="所需样本量 N*（2 倍标准误边界）")
        ax_n.plot(g, n_80, color=t["blue"], linewidth=2, linestyle="--",
                  marker="s", markersize=6, alpha=0.7,
                  markeredgecolor=t["surface"], markeredgewidth=2,
                  label="所需样本量 N80（80% 功效）")
        ax_n.axvline(g0, color=t["orange"], linewidth=2, alpha=0.9)
        ax_n.annotate(f"当前灰度 {g0:.0f}%：N*≈{n_star[1]:,.0f}，N80≈{n_80[1]:,.0f}",
                      xy=(g0, n_80[1]), xytext=(g0 + 2.2, n_80[0] * 0.95),
                      color=t["orange"], fontsize=10, fontweight="bold",
                      arrowprops=dict(arrowstyle="->", color=t["orange"], lw=1.5))
        ax_n.set_ylabel("结论速度：所需总请求量 N（条）")
        ax_n.set_title("灰度比例的取舍：结论速度（上） vs 影响面（下），共用横轴",
                       fontsize=13, fontweight="bold", pad=12)
        ax_n.yaxis.set_major_formatter(FuncFormatter(kfmt))
        ax_n.legend(loc="upper right", frameon=False, fontsize=9.5,
                    labelcolor=t["ink2"])
        # lower facet: impact surface, its own scale, no second axis on top
        ax_e.plot(g, g, color=t["gray"], linewidth=2, linestyle=":",
                  marker="D", markersize=6,
                  markeredgecolor=t["surface"], markeredgewidth=1.5,
                  label="影响面：开启冷却的服务器占比")
        ax_e.axvline(g0, color=t["orange"], linewidth=2, alpha=0.9)
        ax_e.set_ylim(0, 55)
        ax_e.set_ylabel("影响面（%）")
        ax_e.set_xlabel("灰度比例 g（% 服务器开启传送门冷却）")
        ax_e.set_xlim(4, 51)
        ax_e.legend(loc="upper left", frameon=False, fontsize=9.5,
                    labelcolor=t["ink2"])
        paths.append(save(fig, mode, "fig1_canary_tradeoff"))

        # ---------- fig 2: latency CDF ----------------------------------
        fig, ax = plt.subplots(figsize=(8.6, 4.8))
        fig.set_facecolor(t["surface"])
        style_ax(ax, t)
        styles = {
            "对照组：完整 A*": (t["orange"], "-", "o"),
            "冷却组首次请求（未命中+A*+写入）": (t["gray"], "--", None),
            "冷却组重复请求（缓存命中）": (t["blue"], "-", "^"),
        }
        for label, vals in pools.items():
            color, ls, mk = styles[label]
            xs = list(range(0, 9001, 50))
            ys = [sum(1 for v in vals if v <= x) / len(vals) for x in xs]
            ax.plot(xs, ys, color=color, linewidth=2, linestyle=ls,
                    marker=mk, markevery=30, markersize=7,
                    markeredgecolor=t["surface"], markeredgewidth=1.5,
                    label=label)
        ax.set_xlim(0, 9000)
        ax.set_ylim(0, 1.02)
        ax.set_xlabel("单次寻路耗时（纳秒）")
        ax.set_ylabel("经验累积分布 F(x) = P(耗时 ≤ x)")
        n_pooled = len(next(iter(pools.values())))
        ax.set_title(f"三类请求的实测耗时分布（各 {n_pooled:,} 次，6 轮随机交错测量池化）",
                     fontsize=13, fontweight="bold", pad=12)
        ax.legend(loc="lower right", frameon=False, fontsize=9.5,
                  labelcolor=t["ink2"])
        ax.annotate("缓存命中 99% 在 500ns 内：冷却只把重复请求压到这一支",
                    xy=(300, 0.86), xytext=(1800, 0.42), color=t["ink2"],
                    fontsize=9.5,
                    arrowprops=dict(arrowstyle="->", color=t["muted"], lw=1.2))
        paths.append(save(fig, mode, "fig2_latency_cdf"))

        # ---------- fig 3: power vs window size -------------------------
        fig, ax = plt.subplots(figsize=(8.6, 4.8))
        fig.set_facecolor(t["surface"])
        style_ax(ax, t)
        xs = [row["total_n"] for row in pc]
        ys = [row["mc_rejection_rate"] for row in pc]
        ax.plot(xs, ys, color=t["blue"], linewidth=2, marker="o", markersize=8,
                markeredgecolor=t["surface"], markeredgewidth=2,
                label="蒙特卡洛拒绝率（每点 4000 次重复实验）")
        ax.axhline(0.8, color=t["good"], linewidth=1.6, linestyle="--")
        ax.annotate("80% 功效线", xy=(xs[0], 0.8), xytext=(xs[0], 0.84),
                    color=t["good"], fontsize=10)
        n80 = r["design_at_10pct"]["window_used_total_n"]
        ax.axvline(n80, color=t["orange"], linewidth=1.6, linestyle="--")
        ax.annotate(f"N80 ≈ {n80:,} 条（灰度 10%）", xy=(n80, 0.25),
                    xytext=(n80 * 1.05, 0.18), color=t["orange"],
                    fontsize=10, fontweight="bold")
        ax.set_ylim(0, 1.02)
        ax.set_xlabel("观测窗口总请求量 N（条，g = 10% 时处理组约占 10%）")
        ax.set_ylabel("正确判定「显著」的概率（检验功效）")
        ax.set_title("等多久才能把 20% 的差值与噪声分开：功效随样本量上升",
                     fontsize=13, fontweight="bold", pad=12)
        ax.xaxis.set_major_formatter(FuncFormatter(kfmt))
        ax.legend(loc="lower right", frameon=False, fontsize=9.5,
                  labelcolor=t["ink2"])
        paths.append(save(fig, mode, "fig3_power_curve"))

    print("\n".join(paths))


if __name__ == "__main__":
    main()
