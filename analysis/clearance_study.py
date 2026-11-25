#!/usr/bin/env python3
"""Quantify the wall-sticking / detour / blocked-view trade-off.

The planner is a small continuous-geometry counterpart to grid/src/grid.c and
smooth/src/smooth.c: 8-neighbour A* on a metre grid, followed by greedy
line-of-sight shortcuts.  A stand-off margin b inflates every rectangular
obstacle for planning and shortcut acceptance.
"""
from __future__ import annotations

import csv
import heapq
import json
import math
from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

H = 0.10
WIDTH = 16.0
HEIGHT = 12.0
NX = int(WIDTH / H)
NY = int(HEIGHT / H)
DS = 0.05
PREVIEW = 2.7
CONTACT = 0.30

# Acceptance intervals.  These are explicit lab targets, not facts inferred
# from the feedback words; change them here if the robot contract changes.
CLEARANCE_TARGET = (0.35, 0.90)
RATIO_TARGET = (1.00, 1.35)
OCCLUSION_TARGET = (0.00, 0.20)


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass(frozen=True)
class Scenario:
    key: str
    name: str
    start: tuple[float, float]
    goal: tuple[float, float]
    obstacles: tuple[Rect, ...]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def point_rect_dist(p: tuple[float, float], r: Rect) -> float:
    x, y = p
    dx = max(r.x0 - x, 0.0, x - r.x1)
    dy = max(r.y0 - y, 0.0, y - r.y1)
    return math.hypot(dx, dy)


def point_segment_dist(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denom = vx * vx + vy * vy
    t = 0.0 if denom == 0.0 else clamp((wx * vx + wy * vy) / denom, 0.0, 1.0)
    qx, qy = ax + t * vx, ay + t * vy
    return math.hypot(px - qx, py - qy)


def segments_intersect(
    a: tuple[float, float], b: tuple[float, float],
    c: tuple[float, float], d: tuple[float, float],
) -> bool:
    """Inclusive orientation test; touching a corner counts as blocked."""
    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p, q, r):
        return (min(p[0], r[0]) - 1e-10 <= q[0] <= max(p[0], r[0]) + 1e-10 and
                min(p[1], r[1]) - 1e-10 <= q[1] <= max(p[1], r[1]) + 1e-10)

    c1, c2 = cross(a, b, c), cross(a, b, d)
    c3, c4 = cross(c, d, a), cross(c, d, b)
    if abs(c1) < 1e-10 and on_segment(a, c, b):
        return True
    if abs(c2) < 1e-10 and on_segment(a, d, b):
        return True
    if abs(c3) < 1e-10 and on_segment(c, a, d):
        return True
    if abs(c4) < 1e-10 and on_segment(c, b, d):
        return True
    return (c1 > 0) != (c2 > 0) and (c3 > 0) != (c4 > 0)


def segment_rect_hit(a: tuple[float, float], b: tuple[float, float], r: Rect) -> bool:
    if r.x0 <= a[0] <= r.x1 and r.y0 <= a[1] <= r.y1:
        return True
    if r.x0 <= b[0] <= r.x1 and r.y0 <= b[1] <= r.y1:
        return True
    corners = ((r.x0, r.y0), (r.x1, r.y0), (r.x1, r.y1), (r.x0, r.y1))
    edges = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    return any(segments_intersect(a, b, e0, e1) for e0, e1 in edges)


def segment_rect_dist(a: tuple[float, float], b: tuple[float, float], r: Rect) -> float:
    if segment_rect_hit(a, b, r):
        return 0.0
    corners = ((r.x0, r.y0), (r.x1, r.y0), (r.x1, r.y1), (r.x0, r.y1))
    edges = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    ans = min(point_segment_dist(a, e0, e1) for e0, e1 in edges)
    ans = min(ans, min(point_segment_dist(b, e0, e1) for e0, e1 in edges))
    for p in (a, b):
        ans = min(ans, point_rect_dist(p, r))
    # Endpoints of rectangle edges projected onto the query segment.
    for corner in corners:
        ans = min(ans, point_segment_dist(corner, a, b))
    return ans


def cell_center(ix: int, iy: int) -> tuple[float, float]:
    return ((ix + 0.5) * H, (iy + 0.5) * H)


def world_to_cell(p: tuple[float, float]) -> tuple[int, int]:
    return int(p[0] / H), int(p[1] / H)


def astar(scn: Scenario, margin: float) -> list[tuple[float, float]] | None:
    sx, sy = world_to_cell(scn.start)
    gx, gy = world_to_cell(scn.goal)

    def blocked(ix: int, iy: int) -> bool:
        if not (0 <= ix < NX and 0 <= iy < NY):
            return True
        p = cell_center(ix, iy)
        return any(point_rect_dist(p, r) <= margin + 1e-12 for r in scn.obstacles)

    if blocked(sx, sy) or blocked(gx, gy):
        return None

    start = sy * NX + sx
    goal = gy * NX + gx
    dist = {start: 0.0}
    parent = {start: -1}
    pq = [(math.hypot(gx - sx, gy - sy), 0.0, start)]
    directions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
    ]

    while pq:
        _, g, cur = heapq.heappop(pq)
        if g > dist.get(cur, math.inf) + 1e-12:
            continue
        if cur == goal:
            cells = []
            while cur != -1:
                cells.append(cur)
                cur = parent[cur]
            cells.reverse()
            return [cell_center(c % NX, c // NX) for c in cells]

        cx, cy = cur % NX, cur // NX
        for dx, dy, cost in directions:
            nx, ny = cx + dx, cy + dy
            if blocked(nx, ny):
                continue
            if dx and dy and (blocked(cx + dx, cy) or blocked(cx, cy + dy)):
                continue
            ni = ny * NX + nx
            ng = g + cost * H
            if ng < dist.get(ni, math.inf) - 1e-12:
                dist[ni] = ng
                parent[ni] = cur
                hcost = math.hypot(gx - nx, gy - ny) * H
                heapq.heappush(pq, (ng + hcost, ng, ni))
    return None


def shortcut_allowed(a: tuple[float, float], b: tuple[float, float], scn: Scenario, margin: float) -> bool:
    # Keep shortcuts inside the lab.
    if not (0 <= a[0] <= WIDTH and 0 <= a[1] <= HEIGHT and 0 <= b[0] <= WIDTH and 0 <= b[1] <= HEIGHT):
        return False
    # A crossing/touching hit is never a shortcut, even at margin zero.
    for r in scn.obstacles:
        if segment_rect_hit(a, b, r):
            return False
        if segment_rect_dist(a, b, r) < margin - 1e-9:
            return False
    return True


def smooth_path(scn: Scenario, path: list[tuple[float, float]], margin: float) -> list[tuple[float, float]]:
    if len(path) <= 2:
        return path
    out = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not shortcut_allowed(path[i], path[j], scn, margin):
            j -= 1
        out.append(path[j])
        i = j
    return out


def polyline_length(points: Iterable[tuple[float, float]]) -> float:
    pts = list(points)
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def cumulative_lengths(points: list[tuple[float, float]]) -> list[float]:
    cum = [0.0]
    for a, b in zip(points, points[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return cum


def point_at_distance(points: list[tuple[float, float]], cum: list[float], s: float) -> tuple[float, float]:
    s = clamp(s, 0.0, cum[-1])
    j = bisect_right(cum, s) - 1
    j = max(0, min(j, len(points) - 2))
    seg = cum[j + 1] - cum[j]
    t = 0.0 if seg <= 1e-12 else (s - cum[j]) / seg
    a, b = points[j], points[j + 1]
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def densify(points: list[tuple[float, float]], step: float) -> list[tuple[float, float]]:
    cum = cumulative_lengths(points)
    total = cum[-1]
    distances = [i * step for i in range(int(total / step) + 1)]
    if not distances or abs(distances[-1] - total) > 1e-8:
        distances.append(total)
    return [point_at_distance(points, cum, s) for s in distances]


def count_wall_events(samples: list[tuple[float, float]], scn: Scenario) -> tuple[int, float]:
    close = [any(point_rect_dist(p, r) < CONTACT for r in scn.obstacles) for p in samples]
    events = 0
    in_event = False
    for flag in close:
        if flag and not in_event:
            events += 1
            in_event = True
        elif not flag:
            in_event = False
    dmin = min(min(point_rect_dist(p, r) for r in scn.obstacles) for p in samples)
    return events, dmin


def occlusion_fraction(samples: list[tuple[float, float]], scn: Scenario) -> float:
    lengths = cumulative_lengths(samples)
    total = lengths[-1]
    if total == 0:
        return 0.0

    # Classify each short dense interval at its midpoint, weighted by length.
    blocked_len = 0.0
    for i in range(len(samples) - 1):
        s_mid = (lengths[i] + lengths[i + 1]) / 2.0
        target_s = min(total, s_mid + PREVIEW)
        p = point_at_distance(samples, lengths, s_mid)
        q = point_at_distance(samples, lengths, target_s)
        if any(segment_rect_hit(p, q, r) for r in scn.obstacles):
            blocked_len += lengths[i + 1] - lengths[i]
    return blocked_len / total


def evaluate(scn: Scenario, margin: float) -> dict:
    raw = astar(scn, margin)
    if raw is None:
        return {"scenario": scn.key, "margin": margin, "reachable": False}
    planned = smooth_path(scn, raw, margin)
    samples = densify(planned, DS)
    length = polyline_length(planned)
    straight = math.hypot(planned[-1][0] - planned[0][0], planned[-1][1] - planned[0][1])
    events, dmin = count_wall_events(samples, scn)
    occ = occlusion_fraction(samples, scn)
    return {
        "scenario": scn.key,
        "name": scn.name,
        "margin": round(margin, 3),
        "reachable": True,
        "astar_points": len(raw),
        "smooth_points": len(planned),
        "length_m": length,
        "straight_m": straight,
        "detour_ratio": length / straight,
        "min_clearance_m": dmin,
        "wall_events": events,
        "occluded_fraction": occ,
    }


def violations(row: dict) -> list[dict]:
    if not row.get("reachable"):
        return [{"metric": "reachable", "actual": False, "target": True,
                 "direction": "膨胀余量封死通道，b 偏大"}]
    out: list[dict] = []
    c_lo, c_hi = CLEARANCE_TARGET
    r_lo, r_hi = RATIO_TARGET
    o_lo, o_hi = OCCLUSION_TARGET
    c = row["min_clearance_m"]
    if c < c_lo:
        out.append({"metric": "min_clearance_m", "actual": c, "lower": c_lo, "upper": c_hi,
                    "direction": f"低于安全下限 {c_lo:.2f} m（偏小）"})
    elif c > c_hi:
        out.append({"metric": "min_clearance_m", "actual": c, "lower": c_lo, "upper": c_hi,
                    "direction": f"高于上限 {c_hi:.2f} m（偏大）"})
    rho = row["detour_ratio"]
    if rho < r_lo:
        out.append({"metric": "detour_ratio", "actual": rho, "lower": r_lo, "upper": r_hi,
                    "direction": f"低于理论下限 {r_lo:.2f}（异常偏小）"})
    elif rho > r_hi:
        out.append({"metric": "detour_ratio", "actual": rho, "lower": r_lo, "upper": r_hi,
                    "direction": f"高于上限 {r_hi:.2f}（绕远）"})
    eta = row["occluded_fraction"]
    if eta < o_lo:
        out.append({"metric": "occluded_fraction", "actual": eta, "lower": o_lo, "upper": o_hi,
                    "direction": "低于 0（计算异常）"})
    elif eta > o_hi:
        out.append({"metric": "occluded_fraction", "actual": eta, "lower": o_lo, "upper": o_hi,
                    "direction": f"高于上限 {o_hi:.2f}（看不见路占比偏大）"})
    if row["wall_events"] > 0:
        out.append({"metric": "wall_events", "actual": row["wall_events"], "target": 0,
                    "direction": "高于零容忍目标（卡墙次数偏多）"})
    return out


def passes(row: dict) -> dict[str, bool]:
    if not row.get("reachable"):
        return {"clearance": False, "ratio": False, "occlusion": False, "wall": False}
    c_lo, c_hi = CLEARANCE_TARGET
    r_lo, r_hi = RATIO_TARGET
    o_lo, o_hi = OCCLUSION_TARGET
    return {
        "clearance": c_lo <= row["min_clearance_m"] <= c_hi,
        "ratio": r_lo <= row["detour_ratio"] <= r_hi,
        "occlusion": o_lo <= row["occluded_fraction"] <= o_hi,
        "wall": row["wall_events"] == 0,
    }


def scenarios() -> list[Scenario]:
    return [
        Scenario(
            "easy", "一档：单墙转角",
            (1.0, 3.0), (14.5, 3.0),
            (Rect(6.8, 0.0, 7.2, 7.0),),
        ),
        Scenario(
            "medium", "二档：错位门缝",
            (1.0, 4.6), (14.5, 7.4),
            (Rect(7.3, 0.0, 8.2, 5.7), Rect(7.3, 6.9, 8.2, 12.0)),
        ),
        Scenario(
            "hard", "三档：交错双墙",
            (1.0, 2.5), (15.0, 7.0),
            (
                Rect(4.8, 4.0, 5.3, 12.0),
                Rect(9.7, 0.0, 10.2, 7.6),
                Rect(12.7, 6.2, 13.2, 12.0),
            ),
        ),
    ]


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    outdir = root / "artifacts"
    outdir.mkdir(exist_ok=True)
    margins = [i * 0.05 for i in range(0, 17)]  # 0.00 ... 0.80
    scn_list = scenarios()
    rows = [evaluate(s, b) for s in scn_list for b in margins]

    csv_path = outdir / "clearance_sweep.csv"
    fields = ["scenario", "name", "margin", "reachable", "astar_points", "smooth_points",
              "length_m", "straight_m", "detour_ratio", "min_clearance_m",
              "wall_events", "occluded_fraction"]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, "") for k in fields})

    selected = 0.45
    final = [evaluate(s, selected) for s in scn_list]

    feasible_by_scenario: dict[str, list[float]] = {}
    for scn in scn_list:
        feasible_by_scenario[scn.key] = [
            r["margin"] for r in rows
            if r["scenario"] == scn.key and not violations(r)
        ]
    common_feasible = [
        b for b in margins
        if all(not violations(next(
            r for r in rows
            if r["scenario"] == scn.key and abs(r["margin"] - b) < 1e-9
        )) for scn in scenarios())
    ]
    common_feasible = [round(b, 2) for b in common_feasible]
    feasible_range = [min(common_feasible), max(common_feasible)] if common_feasible else None

    boundary_checks = []
    if feasible_range:
        below = round(max(0.0, feasible_range[0] - 0.05), 2)
        above = round(feasible_range[1] + 0.05, 2)
        for b in (below, above):
            for scn in scn_list:
                row = next((r for r in rows if r["scenario"] == scn.key and abs(r["margin"] - b) < 1e-9), None)
                if row is not None and violations(row):
                    boundary_checks.append({"margin": b, "scenario": scn.key,
                                            "name": scn.name, "violations": violations(row),
                                            "metrics": row})

    hard_35 = next(r for r in rows if r["scenario"] == "hard" and abs(r["margin"] - 0.35) < 1e-9)
    hard_40 = next(r for r in rows if r["scenario"] == "hard" and abs(r["margin"] - 0.40) < 1e-9)
    eta_target = OCCLUSION_TARGET[1]
    eta_root = (0.35 + (eta_target - hard_35["occluded_fraction"]) * 0.05
                / (hard_40["occluded_fraction"] - hard_35["occluded_fraction"]))

    medium_scn = next(s for s in scn_list if s.key == "medium")
    fine_door = [evaluate(medium_scn, i / 100.0) for i in range(50, 57)]
    door_reachable = [r["margin"] for r in fine_door if r.get("reachable")]
    door_blocked = [r["margin"] for r in fine_door if not r.get("reachable")]
    boundary_estimates = {
        "hard_visibility_root_linear_m": eta_root,
        "medium_door_last_reachable_1cm_m": max(door_reachable) if door_reachable else None,
        "medium_door_first_unreachable_1cm_m": min(door_blocked) if door_blocked else None,
    }

    report = {
        "constants": {
            "grid_step_m": H, "preview_m": PREVIEW, "contact_threshold_m": CONTACT,
            "wall_events_target": 0,
            "clearance_target_m": CLEARANCE_TARGET,
            "detour_ratio_target": RATIO_TARGET,
            "occluded_fraction_target": OCCLUSION_TARGET,
            "selected_margin_m": selected,
        },
        "feasible_grid_margins_by_scenario": feasible_by_scenario,
        "common_feasible_grid_margins": common_feasible,
        "common_feasible_range_grid": feasible_range,
        "boundary_estimates": boundary_estimates,
        "boundary_checks": boundary_checks,
        "final_runs": [],
    }
    for row in final:
        verdict = passes(row)
        row_violations = violations(row)
        report["final_runs"].append({
            **row,
            "target_pass": verdict,
            "violations": row_violations,
            "all_pass": not row_violations,
        })

    json_path = outdir / "clearance_report.json"
    json_path.write_text(json.dumps({"report": report, "sweep": rows}, ensure_ascii=False, indent=2), encoding="utf-8")

    # Matplotlib is imported only for plotting; the numerical outputs above are
    # already complete if a headless environment lacks a backend.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    colors = {"easy": "#2a78d6", "medium": "#eb6834", "hard": "#1baf7a"}
    labels = {"easy": "一档 单墙转角", "medium": "二档 错位门缝", "hard": "三档 交错双墙"}
    fig, axes = plt.subplots(3, 1, figsize=(8.2, 8.6), sharex=True)
    metric_specs = [
        ("wall_events", "卡墙次数 $N_w$", None),
        ("detour_ratio", "绕远比 $\\rho=L/D$", RATIO_TARGET),
        ("occluded_fraction", "遮挡段占比 $\\eta$", OCCLUSION_TARGET),
    ]
    for ax, (key, ylabel, target) in zip(axes, metric_specs):
        if feasible_range:
            ax.axvspan(feasible_range[0], feasible_range[1], color="#2a78d6", alpha=0.08,
                       linewidth=0, label="_common feasible grid interval")
        for scn in scn_list:
            sr = [r for r in rows if r["scenario"] == scn.key and r.get("reachable")]
            xs = [r["margin"] for r in sr]
            ys = [r[key] for r in sr]
            ax.plot(xs, ys, color=colors[scn.key], linewidth=2, marker="o", markersize=4,
                    label=labels[scn.key])
        if target:
            lo, hi = target
            ax.axhspan(lo, hi, color="#0ca30c", alpha=0.10, linewidth=0)
            ax.axhline(hi, color="#52514e", linewidth=1, linestyle=":")
        if key == "wall_events":
            ax.axhline(0, color="#0ca30c", linewidth=1.5, alpha=0.5)
        ax.axvline(selected, color="#0b0b0b", linewidth=1.5, linestyle="--", alpha=0.75)
        ax.set_ylabel(ylabel)
        ax.grid(True, color="#e1e0d9", linewidth=0.8)
        ax.set_facecolor("#fcfcfb")
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.spines["left"].set_color("#c3c2b7")
        ax.spines["bottom"].set_color("#c3c2b7")
    axes[0].legend(loc="upper right", frameon=False, ncol=1)
    axes[-1].set_xlabel("留边距离 $b$ (m)")
    axes[-1].set_xlim(0, 0.8)
    fig.suptitle("留边距离的三指标权衡（虚线为选定 b=0.45 m）", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    png_path = outdir / "clearance_curves.png"
    svg_path = outdir / "clearance_curves.svg"
    fig.savefig(png_path, dpi=180, facecolor="#fcfcfb")
    fig.savefig(svg_path, facecolor="#fcfcfb")
    plt.close(fig)

    from matplotlib.patches import Rectangle
    map_fig, map_axes = plt.subplots(1, 3, figsize=(15.0, 4.1))
    for ax, scn, row in zip(map_axes, scenarios(), final):
        path = smooth_path(scn, astar(scn, selected) or [], selected)
        for ob in scn.obstacles:
            ax.add_patch(Rectangle((ob.x0, ob.y0), ob.x1 - ob.x0, ob.y1 - ob.y0,
                                   color="#52514e", alpha=0.78, linewidth=0))
        ax.plot([p[0] for p in path], [p[1] for p in path],
                color="#2a78d6", linewidth=2.2, marker="o", markersize=4.5,
                markerfacecolor="#2a78d6", markeredgecolor="#fcfcfb", markeredgewidth=1.2)
        ax.scatter([scn.start[0]], [scn.start[1]], s=95, color="#0ca30c",
                   edgecolor="#fcfcfb", linewidth=2, zorder=5)
        ax.scatter([scn.goal[0]], [scn.goal[1]], s=95, color="#d03b3b",
                   edgecolor="#fcfcfb", linewidth=2, zorder=5)
        ax.set_xlim(0, WIDTH)
        ax.set_ylim(0, HEIGHT)
        ax.set_aspect("equal")
        ax.grid(True, color="#e1e0d9", linewidth=0.8)
        ax.set_facecolor("#fcfcfb")
        ax.set_title(f"{scn.name}\n"
                     f"$d_{{min}}$={row['min_clearance_m']:.3f} m, "
                     f"$\\rho$={row['detour_ratio']:.3f}, "
                     f"$\\eta$={row['occluded_fraction']:.3f}", fontsize=11)
    map_fig.tight_layout()
    map_png = outdir / "scenario_paths.png"
    map_fig.savefig(map_png, dpi=170, facecolor="#fcfcfb", bbox_inches="tight")
    plt.close(map_fig)

    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"wrote {csv_path}")
    print(f"wrote {json_path}")
    print(f"wrote {png_path} and {svg_path}")
    print(f"wrote {map_png}")


if __name__ == "__main__":
    main()
