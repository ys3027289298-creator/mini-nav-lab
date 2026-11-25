#!/usr/bin/env python3
"""Plot the slice-size trade-off curves from request --csv output.

Two stacked panels (never a dual axis):
  top    - worst response time R_wc(B) in frames
  bottom - single-frame hitch H(B) in weighted work units
Reads the CSV emitted by the C self-check so the figure is never hand-faked.
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

# Reference validated palette (light surface).
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
CRITICAL = "#d03b3b"
MUTED = "#898781"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e1e0d9"
SURFACE = "#fcfcfb"
PAGE = "#f9f9f7"

plt.rcParams["font.family"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False


def load(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            rows.append({k: (int(v) if k != "chosen" else v == "1")
                         for k, v in row.items()})
    return rows


def kfmt(v, _pos):
    return f"{v/1000:g}k" if v >= 1000 else f"{v:g}"


def main(src, dst):
    rows = load(src)
    b = [r["B"] for r in rows]
    bound = [r["bound_frames"] for r in rows]
    mframes = [r["meas_frames"] for r in rows]
    hwc = [r["wc_hitch_units"] for r in rows]
    mhitch = [r["meas_hitch_units"] for r in rows]
    cap = 16384
    bstar = next(r["B"] for r in rows if r["chosen"])
    ustar = next(r["bound_frames"] for r in rows if r["chosen"])
    hstar = next(r["wc_hitch_units"] for r in rows if r["chosen"])

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(10.2, 7.6), sharex=True,
        gridspec_kw={"height_ratios": [1, 1.15], "hspace": 0.12})
    fig.patch.set_facecolor(PAGE)
    for ax in (ax1, ax2):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for spine in ("left", "bottom"):
            ax.spines[spine].set_color("#c3c2b7")
        ax.tick_params(colors=INK2, labelsize=9)

    # feasible-band shading lives in both panels
    for ax in (ax1, ax2):
        ax.axvspan(1, bstar, color=BLUE, alpha=0.06, zorder=0)
        ax.axvline(bstar, color=MUTED, linewidth=1.2, linestyle="--", zorder=2)

    # ---- top: worst response time ----
    ax1.plot(b, bound, color=BLUE, linewidth=2.2, zorder=3,
             label="解析上界 R_wc(B)=1+⌈192/B⌉")
    ax1.plot(b, mframes, color=AQUA, linewidth=1.8, zorder=3,
             label="最坏地图实测帧数（189 pops）")
    ax1.scatter([bstar], [ustar], s=70, color=BLUE, edgecolor="white",
                linewidth=1.6, zorder=5)
    ax1.annotate(f"B*={bstar}\nR_wc≤{ustar} 帧",
                 xy=(bstar, ustar), xytext=(bstar + 9, 60),
                 fontsize=10, color=INK,
                 arrowprops=dict(arrowstyle="->", color=INK2, lw=1.1))
    ax1.annotate("B=1：≤193 帧（平滑但慢）", xy=(1, 193), xytext=(6, 168),
                 fontsize=9.5, color=INK2)
    ax1.annotate("一次性 B=192：2 帧", xy=(192, 2), xytext=(96, 30),
                 fontsize=9.5, color=INK2,
                 arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.0))
    ax1.set_ylabel("出结果所需帧数（60Hz 下 1 帧≈16.7ms）",
                   fontsize=10.5, color=INK)
    ax1.set_ylim(0, 205)
    ax1.legend(loc="upper right", frameon=False, fontsize=9.5)

    # ---- bottom: single-frame hitch ----
    ax2.plot(b, hwc, color=ORANGE, linewidth=2.2, zorder=3,
             label="解析卡顿上界 H_wc(B)=1024B+448")
    ax2.plot(b, mhitch, color=AQUA, linewidth=1.8, zorder=3,
             label="最坏地图实测单帧工作峰值")
    ax2.axhline(cap, color=CRITICAL, linewidth=1.6, linestyle="--", zorder=2,
                label=f"单帧预算 CAP={cap} 单元")
    ax2.scatter([bstar], [hstar], s=70, color=ORANGE, edgecolor="white",
                linewidth=1.6, zorder=5)
    ax2.annotate(f"B*={bstar}：H_wc={hstar}≤{cap}\nB*+1 即 {1024*(bstar+1)+448} > CAP",
                 xy=(bstar, hstar), xytext=(bstar + 10, 52000),
                 fontsize=10, color=INK,
                 arrowprops=dict(arrowstyle="->", color=INK2, lw=1.1))
    h192 = 1024 * 192 + 448
    ax2.scatter([192], [h192], s=55, color=CRITICAL, edgecolor="white",
                linewidth=1.4, zorder=5)
    ax2.annotate(f"不分片 B=192：{h192//1000}k 单元\n≈{h192/cap:.0f}× 预算 → 卡帧",
                 xy=(192, h192), xytext=(58, 158000),
                 fontsize=10, color=CRITICAL,
                 bbox=dict(boxstyle="round,pad=0.3", fc=SURFACE,
                           ec="none", alpha=0.9),
                 arrowprops=dict(arrowstyle="->", color=CRITICAL, lw=1.1))
    ax2.set_ylabel("单帧卡顿（加权工作单元/帧）", fontsize=10.5, color=INK)
    ax2.set_xlabel("每帧弹出节点预算 B（个 A* 节点/帧，对数刻度）",
                   fontsize=10.5, color=INK)
    ax2.set_ylim(0, 205000)
    ax2.yaxis.set_major_formatter(FuncFormatter(kfmt))
    ax2.legend(loc="upper left", frameon=False, fontsize=9.5)

    ax2.set_xscale("log", base=2)
    ax2.set_xticks([1, 2, 4, 8, 16, 32, 64, 128, 192])
    ax2.set_xticklabels(["1", "2", "4", "8", "16", "32", "64", "128", "192"])
    ax2.set_xlim(0.9, 210)

    fig.suptitle("分片预算的取舍：最坏响应时间 vs 单帧卡顿（16×12，N=192）",
                 fontsize=13.5, color=INK, fontweight="bold", y=0.975)
    fig.text(0.5, 0.925,
             "阴影为满足单帧预算的可行区间；取最大可行 B*=15，把最坏响应压到 14 帧，且任何一帧都不超过 CAP。",
             ha="center", fontsize=10, color=INK2)

    fig.tight_layout(rect=(0.02, 0.02, 0.98, 0.90))
    fig.savefig(dst, dpi=160, facecolor=fig.get_facecolor())
    print(f"wrote {dst}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: gen_curves.py CURVES.csv OUT.png", file=sys.stderr)
        sys.exit(2)
    main(sys.argv[1], sys.argv[2])
