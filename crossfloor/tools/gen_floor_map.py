#!/usr/bin/env python3
"""Render the three transcribed floor plans and the one-run paths.

Reads data/floors.txt (sketch transcription) and docs/routes.txt (emitted by
the real C planner in `routes` mode), so the picture can never diverge from
what crossfloor actually computed. 3 rows = 1F/2F/3F, 3 columns = the cases.
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"
INK = "#26241f"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e7e5dd"
WALL = "#3c3a35"
CASE_COLORS = [BLUE, ORANGE, AQUA]

plt.rcParams["font.family"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

FLOOR_NAME = {0: "一楼 1F", 1: "二楼 2F", 2: "三楼 3F"}


def parse_floors(path):
    w = h = None
    floors = {}
    cur = None
    rows = []
    inrows = False
    with open(path, encoding="utf-8") as fh:
        for raw in fh:
            s = raw.strip()
            if s.startswith("width "):
                w = int(s.split()[1])
            elif s.startswith("height "):
                h = int(s.split()[1])
            elif s.startswith("floor "):
                cur = int(s.split()[1])
            elif s == "rows":
                inrows = True
                rows = []
            elif s == "endrows":
                floors[cur] = rows
                inrows = False
            elif inrows:
                rows.append(raw.rstrip("\r\n"))
    return w, h, floors


def parse_routes(path):
    cases = []
    cur = None
    with open(path, encoding="utf-8") as fh:
        for raw in fh:
            t = raw.split()
            if not t:
                continue
            if t[0] == "case":
                def val(key):
                    return int(t[t.index(key) + 1])
                cur = {"name": t[1], "selected": val("selected"),
                       "flat": val("flat"), "vert": val("vert"), "nodes": []}
                cases.append(cur)
            elif t[0] == "node":
                cur["nodes"].append((int(t[1]), int(t[2]), int(t[3])))
    return cases


def short_name(n):
    return {"C1_2F_to_1F_opposite": "案例一  2F→1F 对面",
            "C2_3F_wall_popover": "案例二  3F 同层近路",
            "C3_1F_switchback_bridge": "案例三  1F 绕墙 vs 楼梯连廊"}.get(n, n)


def main(floors_path, routes_path, out_png):
    W, H, floors = parse_floors(floors_path)
    cases = parse_routes(routes_path)

    fig, axes = plt.subplots(3, len(cases), figsize=(13.6, 12.4))
    fig.patch.set_facecolor("white")

    for ci, case in enumerate(cases):
        color = CASE_COLORS[ci % 3]
        nodes = case["nodes"]
        for f in range(3):
            ax = axes[f][ci]
            ax.set_facecolor("#fcfcfb")
            plan = floors[f]
            # grid
            ax.set_xticks([x + 0.5 for x in range(W)], minor=True)
            ax.set_yticks([y + 0.5 for y in range(H)], minor=True)
            ax.grid(which="minor", color=GRID, linewidth=0.5)
            ax.tick_params(which="both", length=0, labelbottom=False, labelleft=False)
            for sp in ax.spines.values():
                sp.set_color("#c9c7bf")
            # cells
            for y, row in enumerate(plan):
                for x, ch in enumerate(row):
                    if ch == "#":
                        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, color=WALL, zorder=2))
                    elif ch == "O":
                        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1,
                                               facecolor=AQUA, alpha=0.16, zorder=1))
                        ax.scatter([x], [y], s=150, marker="o", facecolors="white",
                                  edgecolors=AQUA, linewidths=2.2, zorder=4)
            # flat segments (only between same-floor consecutive nodes)
            for a, b in zip(nodes, nodes[1:]):
                if a[0] == b[0] == f:
                    ax.plot([a[1], b[1]], [a[2], b[2]], color=color, linewidth=3.0,
                            solid_capstyle="round", zorder=5, alpha=0.95)
            # vertical links: diamond on both endpoint floors; label the
            # actual traversal direction only at the source node
            for a, b in zip(nodes, nodes[1:]):
                if a[0] != b[0] and f in (a[0], b[0]):
                    for (vf, vx, vy) in (a, b):
                        if vf == f:
                            ax.scatter([vx], [vy], s=110, marker="D", color=INK,
                                       edgecolors="white", linewidths=1.2, zorder=6)
                    if f == a[0]:
                        word = "下楼" if b[0] < a[0] else "上楼"
                        ax.annotate(word, (a[1], a[2]), xytext=(7, 7),
                                    textcoords="offset points", fontsize=9.5,
                                    color=INK, zorder=7,
                                    bbox=dict(boxstyle="round,pad=0.2", fc="white",
                                              ec=AQUA, lw=1.0))
            # start / goal
            if nodes:
                s0, g0 = nodes[0], nodes[-1]
                if s0[0] == f:
                    ax.scatter([s0[1]], [s0[2]], s=320, marker="o", color=AQUA,
                               edgecolors="white", linewidths=1.6, zorder=8)
                    ax.text(s0[1], s0[2], "起", color="white", ha="center", va="center",
                            fontsize=10, fontweight="bold", zorder=9)
                if g0[0] == f:
                    ax.scatter([g0[1]], [g0[2]], s=360, marker="*", color="#d03b3b",
                               edgecolors="white", linewidths=1.0, zorder=8)
                    ax.text(g0[1] + 0.9, g0[2] - 0.9, "终", color="#d03b3b",
                            fontsize=12, fontweight="bold", zorder=9)
            ax.set_xlim(-0.6, W - 0.4)
            ax.set_ylim(H - 0.4, -0.6)
            ax.set_aspect("equal")
            if f == 0:
                verdict = "选楼梯 ✓" if case["selected"] else "走平地 ✓"
                title = f"{short_name(case['name'])}\n{verdict}  平路{case['flat']}格 + 跨层{case['vert']}连接"
                ax.set_title(title, fontsize=11.5, color=INK, pad=8)
            if ci == 0:
                ax.set_ylabel(FLOOR_NAME[f], fontsize=12.5, color=INK, labelpad=6)

    legend_handles = [
        Rectangle((0, 0), 1, 1, color=WALL, label="墙体/阻挡"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="white",
               markeredgecolor=AQUA, markeredgewidth=2.2, markersize=11, label="草图圆圈→楼梯连接点"),
        Line2D([0], [0], color=MUTED, lw=3, marker="D", markerfacecolor=INK,
               markeredgecolor="white", markersize=9, label="竖向楼梯边（折算 λ*=13.81 格/层）"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=AQUA, markersize=12, label="起点"),
        Line2D([0], [0], marker="*", color="w", markerfacecolor="#d03b3b", markersize=15, label="终点"),
    ]
    fig.legend(handles=legend_handles, loc="lower center", ncol=5, fontsize=10.5,
               frameon=True, facecolor="white", edgecolor="#c9c7bf",
               bbox_to_anchor=(0.5, 0.015))
    fig.suptitle("三层平面图与三组起终点的实际寻路路径（λ*=13.81，C 规划器实跑输出）",
                 fontsize=15, color=INK, y=0.985)
    fig.subplots_adjust(left=0.055, right=0.985, top=0.93, bottom=0.075,
                        hspace=0.12, wspace=0.06)
    fig.savefig(out_png, dpi=140, facecolor="white", bbox_inches="tight", pad_inches=0.2)
    print(f"wrote {out_png}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("usage: gen_floor_map.py floors.txt routes.txt out.png", file=sys.stderr)
        sys.exit(2)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
