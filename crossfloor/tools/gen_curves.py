#!/usr/bin/env python3
"""Plot the coefficient calibration curves from the C `sweep` CSV.

Two stacked panels sharing the lambda axis (never a dual axis):
  top    - stair/cross-floor selection rate vs conversion coefficient lambda
  bottom - resulting path length vs lambda

lambda* is reverse-solved from data/survey.txt (actual up/down vs flat time),
never picked to make the chart look right. The shaded band is the interval in
which all three sketch cases match the designer's selection intent.
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
CRITICAL = "#d03b3b"
MUTED = "#898781"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e1e0d9"
SURFACE = "#fcfcfb"

plt.rcParams["font.family"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def reverse_solve_lambda(survey_path):
    cell_m, speed = 1.0, 1.4
    ups, downs = [], []
    with open(survey_path, encoding="utf-8") as fh:
        for line in fh:
            t = line.split()
            if len(t) == 2 and t[0] == "cell_m":
                cell_m = float(t[1])
            elif len(t) == 2 and t[0] == "flat_speed_mps":
                speed = float(t[1])
            elif t and t[0] == "person" and len(t) == 3:
                ups.append(float(t[1]))
                downs.append(float(t[2]))
    tau = cell_m / speed
    link_mean = sum(0.5 * (u + d) for u, d in zip(ups, downs)) / len(ups)
    return link_mean / tau


def main(csv_path, survey_path, out_png):
    rows = load_rows(csv_path)
    lam = [float(r["lambda_cells"]) for r in rows]
    rate = [float(r["corpus_select_rate"]) for r in rows]
    mlen = [float(r["corpus_mean_len"]) for r in rows]
    agg = [(int(r["c1_sel"]) + int(r["c2_sel"]) + int(r["c3_sel"])) / 3.0 for r in rows]
    c1len = [int(r["c1_len"]) for r in rows]
    c2len = [int(r["c2_len"]) for r in rows]
    c3len = [int(r["c3_len"]) for r in rows]
    lstar = reverse_solve_lambda(survey_path)

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10.6, 8.2), sharex=True,
        gridspec_kw={"height_ratios": [1.05, 1.0], "hspace": 0.10})
    fig.patch.set_facecolor("white")
    for ax in (ax1, ax2):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            ax.spines[spine].set_color("#c9c7bf")
        ax.tick_params(colors=INK2, labelsize=10)

    # feasible band: C2 stays flat only for lambda > 8, C3 takes bridge only lambda < 15
    for ax in (ax1, ax2):
        ax.axvspan(8.0, 15.0, color=AQUA, alpha=0.08, zorder=1)
        ax.axvline(lstar, color=AQUA, linewidth=2.0, linestyle="-", zorder=3)

    # ---- top: selection rate ----
    ax1.fill_between(lam, rate, 0, color=BLUE, alpha=0.10, zorder=1)
    ax1.plot(lam, rate, color=BLUE, linewidth=2.6, zorder=4,
             label="标定 OD 集楼梯选中率（120 对同层起终点）")
    ax1.step(lam, agg, where="post", color=ORANGE, linewidth=2.3, zorder=5,
             label="三组案例选中楼梯比例 k/3")
    ax1.axhline(2 / 3, color=MUTED, linewidth=1.4, linestyle="--", zorder=2,
                label="策划草图预期 2/3")
    ax1.set_ylim(-0.03, 1.14)
    ax1.set_yticks([0, 1 / 3, 2 / 3, 1.0])
    ax1.set_yticklabels(["0", "1/3", "2/3", "1.0"])
    ax1.set_ylabel("跨层（楼梯）路径选中率", fontsize=12, color=INK)
    ax1.text(8.35, 1.105, f"λ* = {lstar:.2f} 格/层（上下楼实测耗时反解）",
             color=INK, fontsize=10.5, va="top", ha="left",
             bbox=dict(boxstyle="round,pad=0.35", fc="white", ec=AQUA, lw=1.3))
    ax1.annotate("C2 转折点 λ≈8：再小就放着\n20 格平路不走、去爬楼梯",
                 xy=(8.0, 1 / 3), xytext=(1.4, 0.86), fontsize=9.5, color=INK2,
                 va="center", zorder=6,
                 bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="none", alpha=0.85),
                 arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.2))
    ax1.annotate("C3 转折点 λ=15：\n再大连廊被放弃、绕远 48 格",
                 xy=(15.0, 2 / 3), xytext=(16.0, 0.86), fontsize=9.5, color=INK2,
                 va="center",
                 arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.2))
    ax1.legend(loc="lower left", bbox_to_anchor=(0.012, 0.03), fontsize=9.3,
               frameon=True, facecolor="white", edgecolor=GRID)

    # ---- bottom: path length ----
    ax2.plot(lam, mlen, color=BLUE, linewidth=2.6, zorder=4,
             label="标定 OD 集平均路径长度")
    ax2.step(lam, c3len, where="post", color=CRITICAL, linewidth=2.1, zorder=5,
             label="C3 长度（λ>15 由 20 跳成 48，绕远）")
    ax2.step(lam, c2len, where="post", color=ORANGE, linewidth=2.0, zorder=5,
             label="C2 长度（λ>8 回到 20 格平路）")
    ax2.plot(lam, c1len, color=MUTED, linewidth=1.8, linestyle=":", zorder=3,
             label="C1 长度（异层，恒走楼梯 = 18）")
    ax2.annotate("20 → 48 格", xy=(15.1, 48), xytext=(16.0, 44), fontsize=10,
                 color=CRITICAL,
                 arrowprops=dict(arrowstyle="->", color=CRITICAL, lw=1.3))
    ax2.set_xlabel("跨层折算系数 λ（楼梯一层折算成多少格平地距离）", fontsize=12, color=INK)
    ax2.set_ylabel("路径长度（边/格，含竖向连接）", fontsize=12, color=INK)
    ax2.set_xlim(0, 20)
    ax2.set_ylim(0, 54)
    ax2.legend(loc="upper left", fontsize=9.3, frameon=True,
               facecolor="white", edgecolor=GRID)

    fig.suptitle("跨层代价系数标定：选中率与路径长度两条曲线共同定 λ",
                 fontsize=14.5, color=INK, y=0.975)
    fig.text(0.5, 0.006,
             "绿带 8<λ<15：三组案例同时符合草图预期的系数区间；绿线为实测耗时反解 λ*=13.81。\n"
             "λ* 贴近 15 边缘，故 C3 人群实际选中率 17/20=0.85，偏差 0.15>0.10，已报实际值。",
             ha="center", va="bottom", fontsize=9.6, color=INK2, linespacing=1.7)
    fig.subplots_adjust(top=0.925, bottom=0.13, left=0.085, right=0.975, hspace=0.10)
    fig.savefig(out_png, dpi=150, facecolor="white", bbox_inches="tight", pad_inches=0.18)
    print(f"wrote {out_png}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("usage: gen_curves.py curves.csv survey.txt out.png", file=sys.stderr)
        sys.exit(2)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
