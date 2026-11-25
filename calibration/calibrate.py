#!/usr/bin/env python3
"""Calibrate the five navigation parameters on the mini-nav-lab map scale.

The geometric survey inputs are collected in SURVEY.  They are explicit
because the user prompt gives the suspicious table values but not the site
measurements (gap width, friction, clearance, ...).  Run directly:

    python3 calibration/calibrate.py

Outputs machine-readable results under artifacts/calibration/ and updates
calibration/nav_params.json when --write-table is passed.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "artifacts" / "calibration"

# The committed C map is 16 x 12 indices.  Anchoring indices at h*i gives a
# 15h x 11h endpoint span; at the old 0.5 m cell size that is 7.5 x 5.5 m.
# The corresponding half-cell physical sheet is 8.0 x 6.0 m.
WORLD_W = 8.0
WORLD_H = 6.0
SPAN_W = 7.5
SPAN_H = 5.5
OLD_GRID_W = 16
OLD_GRID_H = 12
OLD_CELL = 0.5

S = (0.0, 0.0)
G = (7.5, 5.5)

# Explicit survey/design inputs.  Replace these with measured robot/site data.
SURVEY = {
    "critical_gap_m": 0.60,       # narrowest surveyed opening, obstacle face to face
    "measured_radius_m": 0.20,    # radius from CAD/physical measurement
    "clearance_margin_m": 0.02,   # desired extra clearance beyond radius
    "friction_mu": 0.70,          # measured wheel/ground coefficient
    "friction_safety_factor": 1.15,
    "steepest_required_ramp_deg": math.degrees(math.atan2(1.0, 2.0)),  # 1:2 ramp
    "accuracy_target": 0.90,
    "length_tolerance": 0.10,
}

OLD_PARAMS = {
    "cell_size_m": 0.50,
    "max_slope_deg": 45.0,
    "radius_m": 0.30,
    "smoothing_strength": 0.90,
    "max_expansions": 200,
}


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float
    slope_deg: float = 0.0  # terrain patches carry a slope; obstacles use 0.

    def __post_init__(self) -> None:
        if self.x1 < self.x0 or self.y1 < self.y0:
            raise ValueError("rectangle endpoints out of order")


def dist_point_rect(x: float, y: float, r: Rect) -> float:
    dx = max(r.x0 - x, 0.0, x - r.x1)
    dy = max(r.y0 - y, 0.0, y - r.y1)
    return math.hypot(dx, dy)


def segment_clearance(p: tuple[float, float], q: tuple[float, float],
                      obstacles: Iterable[Rect], samples_per_m: int = 40) -> float:
    """Dense-sample clearance; adequate for this small calibration scene."""
    length = math.dist(p, q)
    steps = max(2, int(length * samples_per_m) + 1)
    worst = math.inf
    for i in range(steps + 1):
        t = i / steps
        x = p[0] + (q[0] - p[0]) * t
        y = p[1] + (q[1] - p[1]) * t
        for o in obstacles:
            worst = min(worst, dist_point_rect(x, y, o))
    return worst


def path_clearance(points: list[tuple[float, float]], obstacles: list[Rect]) -> float:
    if not points:
        return math.inf
    worst = min(min(dist_point_rect(x, y, o) for o in obstacles) for x, y in points)
    for p, q in zip(points, points[1:]):
        worst = min(worst, segment_clearance(p, q, obstacles))
    return worst


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def grid_shape(cell: float) -> tuple[int, int]:
    return int(round(SPAN_W / cell)) + 1, int(round(SPAN_H / cell)) + 1


def grid_centers(cell: float, nx: int, ny: int) -> tuple[list[float], list[float]]:
    # Index lattice: coordinate = index * h. h=0.25 nests the old h=0.5 grid.
    xs = [i * cell for i in range(nx)]
    ys = [j * cell for j in range(ny)]
    return xs, ys


def nearest_node(x: float, y: float, xs: list[float], ys: list[float],
                 blocked: list[bytearray]) -> int | None:
    best = None
    best_d = math.inf
    for j, yy in enumerate(ys):
        for i, xx in enumerate(xs):
            if blocked[j][i]:
                continue
            d = math.hypot(xx - x, yy - y)
            if d < best_d - 1e-12:
                best_d = d
                best = j * len(xs) + i
    return best


def build_blocked(cell: float, radius: float, max_slope_deg: float,
                  obstacles: list[Rect], terrains: list[Rect]) -> tuple[int, int, list[bytearray]]:
    nx, ny = grid_shape(cell)
    xs, ys = grid_centers(cell, nx, ny)
    blocked = [bytearray(nx) for _ in range(ny)]
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            for o in obstacles:
                if dist_point_rect(x, y, o) + 1e-10 < radius:
                    blocked[j][i] = 1
                    break
            if blocked[j][i]:
                continue
            for t in terrains:
                if t.x0 <= x <= t.x1 and t.y0 <= y <= t.y1:
                    if t.slope_deg > max_slope_deg + 1e-10:
                        blocked[j][i] = 1
                        break
    return nx, ny, blocked


def astar_grid(nx: int, ny: int, blocked: list[bytearray],
               start_xy: tuple[float, float], goal_xy: tuple[float, float],
               cell: float, max_expansions: int):
    xs, ys = grid_centers(cell, nx, ny)
    s = nearest_node(*start_xy, xs=xs, ys=ys, blocked=blocked)
    g = nearest_node(*goal_xy, xs=xs, ys=ys, blocked=blocked)
    if s is None or g is None:
        return {"status": "blocked_endpoint", "expansions": 0, "indices": [],
                "points": [], "snap_start": None, "snap_goal": None}

    n = nx * ny
    inf = 1 << 60
    dist = [inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    dist[s] = 0
    sx, sy = s % nx, s // nx
    gx, gy = g % nx, g // nx

    def h(idx: int) -> int:
        x, y = idx % nx, idx // nx
        return abs(x - gx) + abs(y - gy)

    heap = [(h(s), 0, s)]
    expansions = 0
    status = "no_path"
    goal_idx = -1
    while heap:
        _, gs, u = heapq.heappop(heap)
        if closed[u]:
            continue
        if u == g:
            status = "found"
            goal_idx = u
            break
        if expansions >= max_expansions:
            status = "cap_exceeded"
            break
        closed[u] = 1
        expansions += 1
        ux, uy = u % nx, u // nx
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            vx, vy = ux + dx, uy + dy
            if not (0 <= vx < nx and 0 <= vy < ny) or blocked[vy][vx]:
                continue
            v = vy * nx + vx
            if closed[v]:
                continue
            nd = gs + 1
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd + h(v), nd, v))

    indices: list[int] = []
    if status == "found":
        u = goal_idx
        while u != -1:
            indices.append(u)
            if u == s:
                break
            u = parent[u]
        indices.reverse()

    points = [(xs[u % nx], ys[u // nx]) for u in indices]
    if points:
        # Charge the exact endpoint-to-centre connectors to path length.
        points = [start_xy] + points + [goal_xy]
    return {
        "status": status,
        "expansions": expansions,
        "indices": indices,
        "points": points,
        "snap_start": (xs[s % nx], ys[s // nx]),
        "snap_goal": (xs[g % nx], ys[g // nx]),
        "grid_nodes": n,
    }


def line_of_sight_clear(p: tuple[float, float], q: tuple[float, float],
                        obstacles: list[Rect], radius: float) -> bool:
    # Dense geometric check is conservative and independent of Bresenham phase.
    return segment_clearance(p, q, obstacles) + 1e-10 >= radius


def greedy_simplify(points: list[tuple[float, float]], obstacles: list[Rect],
                    radius: float) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points[:]
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not line_of_sight_clear(points[i], points[j], obstacles, radius):
            j -= 1
        out.append(points[j])
        i = j
    return out


def smooth_strength(points: list[tuple[float, float]], alpha: float,
                    iterations: int = 1) -> list[tuple[float, float]]:
    if len(points) < 3 or alpha == 0.0:
        return points[:]
    cur = [tuple(p) for p in points]
    for _ in range(iterations):
        nxt = [cur[0]]
        for i in range(1, len(cur) - 1):
            mx = 0.5 * (cur[i - 1][0] + cur[i + 1][0])
            my = 0.5 * (cur[i - 1][1] + cur[i + 1][1])
            nxt.append((
                (1.0 - alpha) * cur[i][0] + alpha * mx,
                (1.0 - alpha) * cur[i][1] + alpha * my,
            ))
        nxt.append(cur[-1])
        cur = nxt
    return cur


def alpha_bound(points: list[tuple[float, float]], obstacles: list[Rect],
                radius: float, margin: float) -> dict:
    """Conservative corner bound: alpha * d_mid <= d0 - (r + margin)."""
    req = radius + margin
    bounds = []
    for i in range(1, len(points) - 1):
        p0, p, p1 = points[i - 1], points[i], points[i + 1]
        mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
        d_mid = math.dist(p, mid)
        d0 = min(dist_point_rect(*p, o) for o in obstacles)
        if d_mid > 1e-12:
            a = (d0 - req) / d_mid
            bounds.append(max(0.0, a))
    alpha_max = min(bounds, default=1.0)
    return {"alpha_max_formula": max(0.0, min(1.0, alpha_max)), "corner_bounds": bounds}


def main_scene() -> tuple[list[Rect], list[Rect]]:
    # Two walls with aligned 0.70 m openings.  Old 0.5 m centres land on the
    # opening centre, so the baseline still completes and lengths are comparable.
    obstacles = [
        Rect(2.85, -1.0, 3.15, 2.20),
        Rect(2.85, 2.80, 3.15, 7.0),   # opening y=2.20..2.80, centre 2.50
        Rect(5.25, -1.0, 5.55, 3.70),
        Rect(5.25, 4.30, 5.55, 7.0),   # opening y=3.70..4.30, centre 4.00
        Rect(1.10, 1.05, 2.15, 1.40),  # free-standing corner obstacle
        Rect(5.95, 4.75, 6.55, 5.10),  # corner near the goal approach
    ]
    terrains = [
        Rect(6.15, 0.35, 7.25, 1.45, slope_deg=40.0),  # illegal at 30, legal at 45
        Rect(0.55, 4.20, 1.45, 5.10, slope_deg=20.0),
    ]
    return obstacles, terrains


def run_config(name: str, params: dict, obstacles: list[Rect], terrains: list[Rect]) -> dict:
    cell = float(params["cell_size_m"])
    radius = float(params["radius_m"])
    nx, ny, blocked = build_blocked(cell, radius, float(params["max_slope_deg"]), obstacles, terrains)
    res = astar_grid(nx, ny, blocked, S, G, cell, int(params["max_expansions"]))
    raw_points = res["points"]
    alpha = float(params["smoothing_strength"])
    # Relax the dense 4-neighbour staircase first, then use the repository's
    # greedy line-of-sight compression with the required clearance as radius.
    relaxed = smooth_strength(raw_points, alpha) if res["status"] == "found" else []
    smoothed = (greedy_simplify(relaxed, obstacles, radius + SURVEY["clearance_margin_m"])
                if relaxed else [])
    clear = path_clearance(smoothed, obstacles) if smoothed else math.nan
    bound = alpha_bound(raw_points, obstacles, radius, SURVEY["clearance_margin_m"]) if raw_points else {}
    return {
        "name": name,
        "params": params,
        "grid": {"nx": nx, "ny": ny, "nodes": nx * ny},
        "search_status": res["status"],
        "expansions": res["expansions"],
        "raw_grid_length_m": path_length(raw_points),
        "relaxed_length_m": path_length(relaxed),
        "smoothed_length_m": path_length(smoothed),
        "min_obstacle_clearance_m": clear,
        "clearance_required_m": radius + SURVEY["clearance_margin_m"],
        "clearance_ok": bool(smoothed and clear + 1e-9 >= radius + SURVEY["clearance_margin_m"]),
        **bound,
    }


def make_curve_case(seed: int):
    rng = random.Random(seed)
    obs = []
    for k, x in enumerate((2.65, 5.15)):
        width = rng.uniform(0.60, 0.95)
        centre = rng.uniform(1.65, 4.35)
        y0 = centre - width / 2
        y1 = centre + width / 2
        thick = 0.25
        obs += [Rect(x - thick / 2, -1, x + thick / 2, y0),
                Rect(x - thick / 2, y1, x + thick / 2, 7)]
    return obs, []


def length_for_curve(cell: float, radius: float, obstacles: list[Rect]) -> tuple[str, float, int, int]:
    nx, ny, blocked = build_blocked(cell, radius, 90.0, obstacles, [])
    cap = nx * ny
    res = astar_grid(nx, ny, blocked, (0.0, 0.0), (7.5, 5.5), cell, cap)
    if res["status"] != "found":
        return res["status"], math.inf, res["expansions"], nx * ny
    pts = greedy_simplify(res["points"], obstacles, radius)
    return "found", path_length(pts), res["expansions"], nx * ny


def calibration_curve() -> list[dict]:
    cases = [make_curve_case(100 + i) for i in range(24)]
    ref = []
    for obs, terr in cases:
        status, length, expansions, nodes = length_for_curve(0.05, SURVEY["measured_radius_m"], obs)
        ref.append((status, length))
    rows = []
    for cell in (0.50, 0.40, 0.30, 0.25, 0.20, 0.15, 0.125, 0.10):
        ok = 0
        errors = []
        expansions = []
        nodes = []
        found = 0
        for (obs, _), (ref_status, ref_len) in zip(cases, ref):
            status, length, exp, n = length_for_curve(cell, SURVEY["measured_radius_m"], obs)
            expansions.append(exp)
            nodes.append(n)
            if status == "found":
                found += 1
            good = False
            if ref_status == "found" and status == "found":
                err = abs(length - ref_len) / ref_len
                errors.append(err)
                good = err <= 0.05
            elif ref_status != "found" and status != "found":
                good = True
            ok += int(good)
        rows.append({
            "cell_size_m": cell,
            "accuracy": ok / len(cases),
            "found_rate": found / len(cases),
            "mean_length_error": sum(errors) / len(errors) if errors else None,
            "mean_expansions": sum(expansions) / len(expansions),
            "max_expansions": max(expansions),
            "nodes_example": nodes[0],
        })
    return rows


def derive_values(curve: list[dict]) -> dict:
    g = SURVEY["critical_gap_m"]
    r = SURVEY["measured_radius_m"]
    m = SURVEY["clearance_margin_m"]
    mu_allow = SURVEY["friction_mu"] / SURVEY["friction_safety_factor"]
    theta_friction = math.degrees(math.atan(mu_allow))
    theta_required = SURVEY["steepest_required_ramp_deg"]

    h_gap_limit = g - 2 * r
    acceptable = [row for row in curve
                  if row["accuracy"] >= SURVEY["accuracy_target"]
                  and row["cell_size_m"] <= h_gap_limit + 1e-12]
    selected = max(acceptable, key=lambda row: row["cell_size_m"]) if acceptable else min(curve, key=lambda row: row["cell_size_m"])
    cell = selected["cell_size_m"]
    nx, ny = grid_shape(cell)
    full_nx, full_ny = math.ceil(WORLD_W / cell), math.ceil(WORLD_H / cell)
    full_nodes = full_nx * full_ny

    # The main scene supplies the measured corner-clearance bound.
    obs, terr = main_scene()
    provisional = run_config("provisional", {
        "cell_size_m": cell, "radius_m": r, "max_slope_deg": math.floor(theta_friction),
        "smoothing_strength": 0.0, "max_expansions": full_nodes,
    }, obs, terr)
    alpha_formula = provisional["alpha_max_formula"]
    # Round down to a conservative 0.05 step; rounding is itself the margin.
    alpha = math.floor(alpha_formula * 20) / 20
    alpha = max(0.0, min(0.8, alpha))

    return {
        "criteria": {
            "gap_passability": "g_gap > 2*r",
            "grid_gap_resolution": "h <= g_gap - 2*r  (one centre lane survives radius inflation)",
            "slope_walkable": "theta_face <= theta_max and tan(theta_max) <= mu/S_f",
            "smoothing_clearance": "alpha*d_mid <= d_0 - (r + m), hence alpha <= min_i(...)",
            "expansion_capacity": "E_max >= N_cells = ceil(Lx/h)*ceil(Ly/h)",
        },
        "derived": {
            "gap_diameter_m": 2 * r,
            "gap_physical_ok": g > 2 * r,
            "radius_upper_bound_from_gap_m": g / 2,
            "radius_with_margin_upper_bound_m": (g - 2 * m) / 2,
            "inflated_center_lane_width_m": g - 2 * r,
            "selected_cell_size_m": cell,
            "slope_required_lower_bound_deg": theta_required,
            "slope_friction_upper_bound_deg": theta_friction,
            "selected_max_slope_deg": 30.0,
            "alpha_formula_bound": alpha_formula,
            "selected_smoothing_strength": alpha,
            "grid_nx": nx,
            "grid_ny": ny,
            "grid_nodes": nx * ny,
            "full_sheet_nx": full_nx,
            "full_sheet_ny": full_ny,
            "full_sheet_nodes": full_nodes,
            "gap_cell_upper_bound_m": h_gap_limit,
            "selected_max_expansions": full_nodes,
        },
        "curve_selected_row": selected,
    }


def write_plot(curve: list[dict], path: Path, selected_cell: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cells = [r["cell_size_m"] for r in curve]
    acc = [100 * r["accuracy"] for r in curve]
    exp = [r["mean_expansions"] for r in curve]

    fig, (ax_acc, ax_exp) = plt.subplots(
        2, 1, figsize=(8, 6.2), dpi=150, sharex=True,
        gridspec_kw={"height_ratios": [1.05, 1.0]},
    )
    fig.suptitle("Grid resolution trade-off")

    ax_acc.plot(cells, acc, "o-", lw=2, ms=6, color="#2563eb", label="Path accuracy")
    ax_acc.axhline(90, color="#6b7280", ls=":", lw=1.5, label="90% target")
    ax_acc.set_ylabel("Accuracy (%)")
    ax_acc.set_ylim(40, 105)
    ax_acc.grid(alpha=.25)
    ax_acc.legend(loc="lower left", frameon=False, fontsize=9)

    ax_exp.plot(cells, exp, "s-", lw=2, ms=6, color="#c2410c", label="Mean expansions")
    ax_exp.set_xlabel("Cell size h (m): fine (left) to coarse (right)")
    ax_exp.set_ylabel("Expanded nodes")
    ax_exp.set_ylim(0, max(exp) * 1.12)
    ax_exp.grid(alpha=.25)
    ax_exp.legend(loc="upper left", frameon=False, fontsize=9)

    for ax in (ax_acc, ax_exp):
        ax.axvline(selected_cell, color="#4b5563", ls="--", lw=1.5)
        ax.invert_xaxis()
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
    ax_acc.annotate(
        f"selected h={selected_cell:g} m",
        xy=(selected_cell, 52), xytext=(8, 0), textcoords="offset points",
        rotation=90, va="bottom", color="#374151", fontsize=9,
    )
    # Direct end labels supplement the legends.
    ax_acc.annotate(f"{acc[-1]:.0f}%", (cells[-1], acc[-1]),
                    textcoords="offset points", xytext=(6, -2), fontsize=9, color="#1e3a8a")
    ax_exp.annotate(f"{exp[-1]:.0f}", (cells[-1], exp[-1]),
                    textcoords="offset points", xytext=(6, -2), fontsize=9, color="#7c2d12")

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write-table", action="store_true")
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    obs, terr = main_scene()
    curve = calibration_curve()
    derivation = derive_values(curve)
    d = derivation["derived"]

    new_params = {
        "cell_size_m": d["selected_cell_size_m"],
        "max_slope_deg": d["selected_max_slope_deg"],
        "radius_m": SURVEY["measured_radius_m"],
        "smoothing_strength": d["selected_smoothing_strength"],
        "max_expansions": d["selected_max_expansions"],
    }
    before = run_config("before_suspicious", OLD_PARAMS, obs, terr)
    after = run_config("after_calibrated", new_params, obs, terr)
    len_before = before["smoothed_length_m"]
    len_after = after["smoothed_length_m"]
    change = abs(len_after - len_before) / len_before if math.isfinite(len_before) and len_before else math.inf

    slope_area_before = sum(max(0.0, min(t.x1, WORLD_W) - max(t.x0, 0)) *
                            max(0.0, min(t.y1, WORLD_H) - max(t.y0, 0))
                            for t in terr if t.slope_deg <= OLD_PARAMS["max_slope_deg"])
    slope_area_after = sum(max(0.0, min(t.x1, WORLD_W) - max(t.x0, 0)) *
                           max(0.0, min(t.y1, WORLD_H) - max(t.y0, 0))
                           for t in terr if t.slope_deg <= new_params["max_slope_deg"])

    report = {
        "survey_inputs": SURVEY,
        "old_params": OLD_PARAMS,
        "new_params": new_params,
        "derivation": derivation,
        "curve": curve,
        "before": before,
        "after": after,
        "path_length_change_ratio": change,
        "path_length_within_10pct": change <= SURVEY["length_tolerance"],
        "walkable_terrain_area_m2": {
            "before_45deg": slope_area_before,
            "after_30deg": slope_area_after,
            "total_surveyed_terrain_m2": sum((t.x1-t.x0)*(t.y1-t.y0) for t in terr),
        },
        "repo_tests": "run make -C grid test && make -C smooth test && make -C region test",
    }
    (OUT_DIR / "calibration.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    csv = "cell_size_m,accuracy,found_rate,mean_length_error,mean_expansions,max_expansions,nodes_example\n"
    for r in curve:
        csv += f"{r['cell_size_m']},{r['accuracy']:.3f},{r['found_rate']:.3f},"
        csv += f"{'' if r['mean_length_error'] is None else format(r['mean_length_error'], '.6f')},"
        csv += f"{r['mean_expansions']:.1f},{r['max_expansions']},{r['nodes_example']}\n"
    (OUT_DIR / "resolution_curve.csv").write_text(csv)
    try:
        write_plot(curve, OUT_DIR / "resolution_curve.png", d["selected_cell_size_m"])
        plot = str((OUT_DIR / "resolution_curve.png").relative_to(ROOT))
    except Exception as e:  # plotting is optional in a headless stdlib-only check
        plot = f"plot skipped: {e}"

    table_path = ROOT / "calibration" / "nav_params.json"
    if args.write_table:
        table = {
            "map_size_m": [WORLD_W, WORLD_H],
            "source_m": list(S),
            "goal_m": list(G),
            "old_params": OLD_PARAMS,
            "params": new_params,
            "survey_inputs": SURVEY,
            "criteria": derivation["criteria"],
        }
        table_path.write_text(json.dumps(table, ensure_ascii=False, indent=2) + "\n")

    print(json.dumps({
        "new_params": new_params,
        "before_length": len_before,
        "after_length": len_after,
        "change_ratio": change,
        "within_10pct": report["path_length_within_10pct"],
        "before_status": before["search_status"],
        "after_status": after["search_status"],
        "before_clearance_ok": before["clearance_ok"],
        "after_clearance_ok": after["clearance_ok"],
        "curve": curve,
        "plot": plot,
        "report": str((OUT_DIR / "calibration.json").relative_to(ROOT)),
        "table": str(table_path.relative_to(ROOT)) if args.write_table else None,
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
