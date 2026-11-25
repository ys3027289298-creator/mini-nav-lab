#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 results/sim_results.json，生成 figures/ 下的图与诊断 CSV。

配色来自 dataviz 参考调色板（已用 validate_palette.py 校验）：
  蓝 #2a78d6 / 橙 #eb6834 为身份系列（全配对安全）；
  红 #e34948 / 绿 #1baf7a 仅作状态语义（拒绝尖峰、零误伤），并配直接标签。
"""
import csv
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
RESULTS = os.path.join(ROOT, "results")
FIG = os.path.join(ROOT, "figures")

BLUE, ORANGE, RED, GREEN = "#2a78d6", "#eb6834", "#e34948", "#1baf7a"
INK, MUTED, GRID, SURFACE = "#0b0b0b", "#898781", "#e1e0d9", "#fcfcfb"
BASELINE_GRAY = "#9a988f"

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Noto Sans CJK SC", "WenQuanYi Zen Hei",
                        "system-ui", "DejaVu Sans", "Segoe UI", "sans-serif"],
    "axes.unicode_minus": False,
    "font.size": 10,
    "text.color": INK,
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "xtick.color": "#52514e",
    "ytick.color": "#52514e",
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.axisbelow": True,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

SWITCH_T = 300
T0, T_LAG, T_ROLLBACK = 270, 324, 360


def clock(t):
    base = 10 * 60 + 5
    total = base + (t - SWITCH_T)
    return f"{total // 60:02d}:{total % 60:02d}"


def event_lines(ax, label_alt=False):
    """在时间轴上标注 10:02/10:05/10:09/10:15 四个事故事件。"""
    events = [(T0, "10:02 报告"), (SWITCH_T, "10:05 换图"),
              (T_LAG, "10:09 卡顿"), (T_ROLLBACK, "10:15 回滚")]
    styles = [("--", MUTED), ("-", RED), ("--", MUTED), ("--", MUTED)]
    for (t, label), (ls, col) in zip(events, styles):
        ax.axvline(t, color=col, linestyle=ls, linewidth=1.3, alpha=0.85)
    ymax = ax.get_ylim()[1]
    for i, (t, label) in enumerate(events):
        # 奇偶交替两个高度，避免相邻事件文字重叠
        y_text = ymax * (1.045 if i % 2 == 0 else 1.105)
        ax.annotate(label, xy=(t, y_text), ha="center", va="bottom",
                    fontsize=8, color="#52514e")
    ax.set_ylim(top=ymax * 1.17)


def load():
    with open(os.path.join(RESULTS, "sim_results.json"), encoding="utf-8") as f:
        return json.load(f)


# --------------------------------------------------------------------------
# 图1：三档延迟 × 三策略的拒绝率时间线（3x1 小多相）
# --------------------------------------------------------------------------
def plot_timelines(data):
    tiers = data["tiers"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9.5), sharex=True)
    pol_style = {
        "buggy": (RED, "-", "事故：版本号未更新"),
        "fixed": (BLUE, "-", "修复：提前 D_max bump"),
        "latebump": (ORANGE, "--", "bump 提前量不足"),
    }
    for ax, tier in zip(axes, tiers):
        d = tier["d_max"]
        for pol, (col, ls, label) in pol_style.items():
            pr = tier["policies"][pol]
            xs = [r["t"] for r in pr["series"]]
            ys = [r["reject_rate"] for r in pr["series"]]
            ax.plot(xs, ys, color=col, linestyle=ls, linewidth=1.8, label=label)
        event_lines(ax)
        ax.set_ylabel("拒绝率")
        ax.set_title(f"最大同步延迟 D_max = {d}s", loc="left", fontsize=11)
        ax.set_ylim(bottom=0)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", frameon=False,
               fontsize=9.5, ncol=3, bbox_to_anchor=(0.5, 0.965))
    axes[-1].set_xlabel("事故时钟（秒，300s = 10:05）")
    fig.suptitle("地图热更新期间服务端路径拒绝率：事故策略 vs 版本化修复",
                 fontsize=13, x=0.02, ha="left", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = os.path.join(FIG, "fig1_reject_timelines.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


# --------------------------------------------------------------------------
# 图2：D_max=60 事故策略的拒绝率分解（总拒绝 / 切换致因误伤 / 边缘噪声）
# --------------------------------------------------------------------------
def plot_decomposition(data):
    tier = next(t for t in data["tiers"] if t["d_max"] == 60)
    pr = tier["policies"]["buggy"]
    ser = pr["series"]
    xs = [r["t"] for r in ser]
    total = [r["reject_rate"] for r in ser]
    # series 存的是每 tick 计数，样本数 = 4000 客户端 * 8% = 320
    per = 320
    si_rate = [r["switch_induced"] / per for r in ser]
    en_rate = [r["edge_noise"] / per for r in ser]

    fig, ax = plt.subplots(figsize=(10, 5.2))
    ax.fill_between(xs, 0, si_rate, color=RED, alpha=0.28,
                    label="切换致因误伤（版本错位）")
    ax.fill_between(xs, si_rate, [s + e for s, e in zip(si_rate, en_rate)],
                    color=ORANGE, alpha=0.28, label="贴边噪声（与切换无关）")
    ax.plot(xs, total, color=BLUE, linewidth=2.0, label="总拒绝率")
    event_lines(ax)
    ax.set_xlim(240, 420)
    ax.set_ylabel("拒绝率")
    ax.set_xlabel("事故时钟（秒）")
    ax.set_title("D_max=60 事故策略：拒绝率尖峰的因果分解——尖峰全部来自版本错位",
                 loc="left", fontsize=12)
    ax.legend(loc="upper right", frameon=False, fontsize=9)
    fig.tight_layout()
    out = os.path.join(FIG, "fig2_spike_decomposition.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


# --------------------------------------------------------------------------
# 图3：严格程度两条曲线（FPR 与 CIR 随 k 变化）
# --------------------------------------------------------------------------
def plot_strictness(data):
    curve = data["strictness"]["curve"]
    chosen = data["strictness"]["chosen"]
    ks = [c["k"] for c in curve]
    fpr = [c["fpr"] * 100 for c in curve]
    cir = [c["cir"] * 100 for c in curve]

    fig, ax = plt.subplots(figsize=(9.5, 5.4))
    ax.plot(ks, cir, color=BLUE, marker="o", markersize=7, linewidth=2,
            label="作弊拦截率 CIR（越高越好）")
    ax.plot(ks, fpr, color=ORANGE, marker="s", markersize=7, linewidth=2,
            label="诚实路径误判率 FPR（越低越好）")
    ax.axhline(1.0, color=MUTED, linestyle=":", linewidth=1.3)
    ax.annotate("FPR 预算 1%", xy=(ks[-1], 1.0), xytext=(-4, 6),
                textcoords="offset points", ha="right", fontsize=8.5,
                color="#52514e")
    # 选中工作点
    ck = chosen["k"]
    ax.scatter([ck], [chosen["cir"] * 100], s=170, facecolor="none",
               edgecolor=GREEN, linewidth=2.2, zorder=5)
    ax.scatter([ck], [chosen["fpr"] * 100], s=170, facecolor="none",
               edgecolor=GREEN, linewidth=2.2, zorder=5)
    ax.annotate(f"选定工作点 k={ck}\nFPR {chosen['fpr']*100:.2f}% · "
                f"CIR {chosen['cir']*100:.1f}%",
                xy=(ck, chosen["cir"] * 100), xytext=(14, -28),
                textcoords="offset points", fontsize=9, color=GREEN)
    # 两端直接标签
    ax.annotate(f"{cir[0]:.0f}%", xy=(ks[0], cir[0]), xytext=(6, -4),
                textcoords="offset points", fontsize=9, color=BLUE)
    ax.annotate(f"{fpr[0]:.1f}%", xy=(ks[0], fpr[0]), xytext=(6, 2),
                textcoords="offset points", fontsize=9, color=ORANGE)
    ax.set_xticks(ks)
    ax.set_xlabel("严格程度 k：允许的不可走点个数（k=0 为原始逐点严格判据）")
    ax.set_ylabel("百分比 %")
    ax.set_ylim(-3, 108)
    ax.set_title("严格程度取舍：两条曲线的交点决定 k（D_max=60，cutover 前稳态窗口）",
                 loc="left", fontsize=12)
    ax.legend(loc="center right", frameon=False, fontsize=9)
    fig.tight_layout()
    out = os.path.join(FIG, "fig3_strictness_curves.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


# --------------------------------------------------------------------------
# 图4：三档延迟的 bump 时机时间条（含反解点与截断标记）
# --------------------------------------------------------------------------
def plot_bump_timing(data):
    fig, ax = plt.subplots(figsize=(10.5, 6.2))
    rows = []
    for tier in data["tiers"]:
        d = tier["d_max"]
        for pol in ("buggy", "latebump", "fixed"):
            rows.append((d, pol, tier["policies"][pol]))
    y = 0
    yticks, ylabels = [], []
    colmap = {"buggy": RED, "latebump": ORANGE, "fixed": BLUE}
    namemap = {"buggy": "事故(无bump)", "latebump": "bump不足", "fixed": "修复"}
    for d, pol, pr in rows:
        yticks.append(y)
        ylabels.append(f"D={d}s · {namemap[pol]}")
        # cutover 固定在 300
        ax.scatter([SWITCH_T], [y], marker="v", color=INK, s=70, zorder=5)
        if pr["bump_t"] is not None:
            ax.plot([pr["bump_t"], SWITCH_T], [y, y], color=colmap[pol],
                    linewidth=7, solid_capstyle="round", alpha=0.85)
            ax.scatter([pr["bump_t"]], [y], color=colmap[pol], s=55, zorder=6)
            # 拒绝率反解点只对“应当 bump”的策略有意义
            inf = SWITCH_T - pr["inferred_d_max_full_s"]
            if pr["inferred_d_max_full_s"] > 0:
                ax.scatter([inf], [y], marker="x", color=GREEN, s=85,
                           linewidth=2.2, zorder=7)
                if pr["censored_by_rollback"]:
                    ax.annotate("回滚截断\n(全窗口还原)",
                                xy=(inf, y), xytext=(-6, 14),
                                textcoords="offset points", ha="right",
                                fontsize=8, color=RED)
        else:
            ax.annotate("版本号从未 bump", xy=(SWITCH_T, y), xytext=(8, 0),
                        textcoords="offset points", va="center",
                        fontsize=8.5, color=RED)
        y += 1
    for t, lab in [(T0, "10:02"), (SWITCH_T, "10:05"), (T_LAG, "10:09"),
                   (T_ROLLBACK, "10:15")]:
        ax.axvline(t, color=MUTED if t != SWITCH_T else RED,
                   linestyle="-" if t == SWITCH_T else "--",
                   linewidth=1.2, alpha=0.8)
        ax.annotate(lab, xy=(t, y - 0.4), fontsize=8.5, color="#52514e",
                    ha="center")
    ax.set_yticks(yticks)
    ax.set_yticklabels(ylabels)
    ax.set_xlabel("事故时钟（秒）")
    ax.set_xlim(100, 380)
    ax.set_ylim(-0.6, y - 0.1)
    ax.set_title("版本号 bump 时机：彩条=实际提前量，绿叉=拒绝率反解点，须早于 10:05 换图",
                 loc="left", fontsize=12)
    legend_items = [
        Line2D([0], [0], color=BLUE, lw=7, label="修复：提前量 = D_max"),
        Line2D([0], [0], color=ORANGE, lw=7, label="bump 提前量不足"),
        Line2D([0], [0], marker="v", color=INK, lw=0, label="服务端换图 cutover"),
        Line2D([0], [0], marker="x", color=GREEN, lw=0, label="拒绝率反解的应 bump 点"),
    ]
    ax.legend(handles=legend_items, loc="upper right", frameon=False,
              fontsize=9, bbox_to_anchor=(1.0, -0.09), ncol=2)
    ax.grid(axis="y", alpha=0)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    out = os.path.join(FIG, "fig4_bump_timing.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


# --------------------------------------------------------------------------
# 诊断 CSV：出现误判的档位给出被拒路径版本号与当前地图版本
# --------------------------------------------------------------------------
def write_diagnostics_csv(data):
    out = os.path.join(RESULTS, "misjudgement_diagnostics.csv")
    with open(out, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["d_max_s", "policy", "tick", "clock", "client",
                    "path_label_version", "path_gen_version",
                    "server_map_version", "bad_on_eval_map", "bad_on_own_map",
                    "reason"])
        for tier in data["tiers"]:
            d = tier["d_max"]
            for pol, pr in tier["policies"].items():
                for rec in pr["diagnostics_sample"]:
                    w.writerow([d, pol, rec["t"], rec["clock"], rec["client"],
                                rec["label_version"], rec["gen_version"],
                                rec["server_map_version"],
                                rec["bad_on_eval_map"], rec["bad_on_own_map"],
                                rec["reason"]])
                if pr["diagnostics_total"] == 0:
                    w.writerow([d, pol, "", "", "", "", "", "", "", "",
                                "ZERO switch-induced misjudgement"])
    return out


def main():
    os.makedirs(FIG, exist_ok=True)
    data = load()
    outs = [
        plot_timelines(data),
        plot_decomposition(data),
        plot_strictness(data),
        plot_bump_timing(data),
        write_diagnostics_csv(data),
    ]
    for o in outs:
        print("wrote", o)


if __name__ == "__main__":
    main()
