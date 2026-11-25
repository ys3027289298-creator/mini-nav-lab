#!/usr/bin/env python3
"""Minimal multi-goal navigation reproduction and exhaustive/greedy comparison.

Run:
    python3 multigoal/repro_multigoal.py
    python3 multigoal/repro_multigoal.py --budget-ms 100 --tiers 3,6,9

The first demo reproduces the bug "visited the first point and stopped".
It then compares all visit orders with nearest-neighbour greedy planning.
Only the Python standard library is required.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import random
import statistics
import time
from itertools import permutations
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

W, H = 24, 16
SEED = 2
START: Tuple[int, int] = (1, 1)
# Fixed deterministic target set; labels in reports are 1-based positions here.
TARGETS: List[Tuple[int, int]] = [
    (2, 2), (3, 6), (6, 12), (22, 14),
    (20, 4), (20, 1), (19, 11), (6, 7),
    (21, 7), (17, 6), (18, 8), (2, 14),
]

Point = Tuple[int, int]


def build_blocked() -> Set[Point]:
    """A few wall lines with gaps so every benchmark target stays reachable."""
    blocked: Set[Point] = set()

    for y in range(2, 13):
        if y not in (5, 9):
            blocked.add((7, y))

    for x in range(4, 19):
        if x not in (10, 15):
            blocked.add((x, 5))

    for y in range(1, 12):
        if y not in (4, 8):
            blocked.add((16, y))

    for x in range(9, 22):
        if x not in (12, 19):
            blocked.add((x, 10))

    # Deterministic scattered obstacles.
    rng = random.Random(SEED)
    for _ in range(25):
        x = rng.randrange(W)
        y = rng.randrange(H)
        blocked.add((x, y))

    return blocked


class PathFinder:
    """Four-neighbour A*, matching the movement model in grid/src/grid.c."""

    def __init__(self) -> None:
        self.astar_calls = 0
        self.durations_ms: List[float] = []

    def find_path(
        self, blocked: Set[Point], start: Point, goal: Point
    ) -> Optional[List[Point]]:
        self.astar_calls += 1
        t0 = time.perf_counter()

        if start in blocked or goal in blocked:
            return None

        heap: List[Tuple[int, int, Point]] = [(0, 0, start)]
        gscore: Dict[Point, int] = {start: 0}
        parent: Dict[Point, Point] = {}

        while heap:
            _, g, cur = heapq.heappop(heap)
            if g != gscore.get(cur):
                continue
            if cur == goal:
                path = [cur]
                while cur in parent:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                self.durations_ms.append((time.perf_counter() - t0) * 1000.0)
                return path

            x, y = cur
            for nxt in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                nx, ny = nxt
                if not (0 <= nx < W and 0 <= ny < H) or nxt in blocked:
                    continue
                ng = g + 1
                if ng < gscore.get(nxt, 1 << 30):
                    gscore[nxt] = ng
                    parent[nxt] = cur
                    h = abs(nx - goal[0]) + abs(ny - goal[1])
                    heapq.heappush(heap, (ng + h, ng, nxt))

        self.durations_ms.append((time.perf_counter() - t0) * 1000.0)
        return None


def build_cost_matrix(
    pf: PathFinder, blocked: Set[Point], targets: Sequence[Point]
) -> Tuple[List[List[int]], float]:
    """Symmetric pairwise shortest-path costs among START and targets."""
    nodes = [START, *targets]
    m = len(nodes)
    dist = [[0] * m for _ in range(m)]

    t0 = time.perf_counter()
    for i in range(m):
        for j in range(i + 1, m):
            path = pf.find_path(blocked, nodes[i], nodes[j])
            if path is None:
                raise RuntimeError(f"unreachable: {nodes[i]} -> {nodes[j]}")
            dist[i][j] = dist[j][i] = len(path) - 1
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    return dist, elapsed_ms


def exhaustive_best(dist: Sequence[Sequence[int]], n: int) -> Tuple[int, Tuple[int, ...], int, float]:
    """Return optimum cost, order, number of orders, and enumeration time."""
    best_cost = 1 << 60
    best_order: Tuple[int, ...] = ()
    checked = 0

    t0 = time.perf_counter()
    for order in permutations(range(1, n + 1)):
        cost = dist[0][order[0]]
        prev = order[0]
        for cur in order[1:]:
            cost += dist[prev][cur]
            prev = cur
        if cost < best_cost:
            best_cost = cost
            best_order = order
        checked += 1
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    return best_cost, best_order, checked, elapsed_ms


def greedy_nearest_neighbour(
    dist: Sequence[Sequence[int]], n: int
) -> Tuple[int, Tuple[int, ...], float]:
    cost = 0
    current = 0
    remaining = set(range(1, n + 1))
    order: List[int] = []

    t0 = time.perf_counter()
    while remaining:
        nxt = min(remaining, key=lambda j: (dist[current][j], j))
        cost += dist[current][nxt]
        order.append(nxt)
        remaining.remove(nxt)
        current = nxt
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    return cost, tuple(order), elapsed_ms


def execute_order(
    pf: PathFinder,
    blocked: Set[Point],
    targets: Sequence[Point],
    order: Sequence[int],
) -> List[Point]:
    """Actually concatenate grid paths; never return after the first target."""
    current = START
    full_path: List[Point] = []

    for label in order:
        goal = targets[label - 1]
        leg = pf.find_path(blocked, current, goal)
        if leg is None:
            raise RuntimeError(f"cannot execute leg to target {label}")
        if not full_path:
            full_path.extend(leg)
        else:
            full_path.extend(leg[1:])  # shared endpoint, avoid duplicate cell
        current = goal

    return full_path


def buggy_visit_first_only(
    pf: PathFinder, blocked: Set[Point], targets: Sequence[Point]
) -> Tuple[List[Point], List[Point]]:
    """Reproduction of the reported bug: a return/break sits inside the loop."""
    current = START
    reached: List[Point] = []
    walked: List[Point] = []

    for goal in targets:
        leg = pf.find_path(blocked, current, goal)
        if leg is None:
            raise RuntimeError("first target unreachable")
        walked = leg
        reached.append(goal)
        return reached, walked  # BUG: loop exits after target 1; move return after loop.

    return reached, walked


def render_map(
    blocked: Set[Point],
    targets: Sequence[Point],
    path: Sequence[Point],
) -> str:
    path_cells = set(path)
    labels = {START: "S"}
    for i, target in enumerate(targets, 1):
        labels[target] = str(i)

    rows = []
    for y in range(H):
        chars = []
        for x in range(W):
            p = (x, y)
            if p in labels:
                chars.append(labels[p])
            elif p in blocked:
                chars.append("#")
            elif p in path_cells:
                chars.append("*")
            else:
                chars.append(".")
        rows.append("".join(chars))
    return "\n".join(rows)


def gap_percent(greedy_cost: int, optimal_cost: int) -> float:
    return 100.0 * (greedy_cost - optimal_cost) / optimal_cost


def largest_factorial_n(budget_ms: float, ms_per_order: float) -> int:
    n = 1
    while ms_per_order * math.factorial(n + 1) <= budget_ms:
        n += 1
    return n


def write_csv(rows: Sequence[Dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_html_chart(rows: Sequence[Dict[str, float]], path: Path, budget_ms: float) -> None:
    """Write a dependency-free two-panel line chart."""
    ns = [int(r["n"]) for r in rows]
    w, h = 520, 300
    pad_l, pad_r, pad_t, pad_b = 54, 18, 28, 42

    def x(v: float) -> float:
        return pad_l + (v - min(ns)) / (max(ns) - min(ns)) * (w - pad_l - pad_r)

    def y_linear(v: float, lo: float, hi: float) -> float:
        return pad_t + (hi - v) / (hi - lo) * (h - pad_t - pad_b)

    def y_log(v: float, lo: float, hi: float) -> float:
        lv, lo_l, hi_l = math.log10(v), math.log10(lo), math.log10(hi)
        return pad_t + (hi_l - lv) / (hi_l - lo_l) * (h - pad_t - pad_b)

    cost_hi = max(float(r["greedy_cost"]) for r in rows) * 1.12
    cost_lo = min(float(r["opt_cost"]) for r in rows) * 0.94
    time_vals = [max(float(r["exhaustive_total_ms"]), 0.001) for r in rows]
    time_vals += [max(float(r["greedy_ms"]), 0.001) for r in rows]
    t_lo, t_hi = 0.001, max(max(time_vals), budget_ms) * 1.25

    def line(key: str, yfn) -> str:
        pts = " ".join(f"{x(float(r['n'])):.1f},{yfn(float(r[key])):.1f}" for r in rows)
        return f'<polyline points="{pts}" fill="none" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>'

    def dots(key: str, color: str, yfn) -> str:
        return "".join(
            f'<circle cx="{x(float(r["n"])):.1f}" cy="{yfn(float(r[key])):.1f}" r="4.5" fill="{color}" stroke="#fcfcfb" stroke-width="2"/>'
            for r in rows
        )

    cost_opt_line = line("opt_cost", lambda v: y_linear(v, cost_lo, cost_hi)).replace('stroke-width="2.5"', 'stroke="#2a78d6" stroke-width="2.5"')
    cost_greedy_line = line("greedy_cost", lambda v: y_linear(v, cost_lo, cost_hi)).replace('stroke-width="2.5"', 'stroke="#eb6834" stroke-width="2.5" stroke-dasharray="6 4"')
    opt_dots = dots("opt_cost", "#2a78d6", lambda v: y_linear(v, cost_lo, cost_hi))
    greedy_dots = dots("greedy_cost", "#eb6834", lambda v: y_linear(v, cost_lo, cost_hi))

    exh_time_line = line("exhaustive_total_ms", lambda v: y_log(max(v, 0.001), t_lo, t_hi)).replace('stroke-width="2.5"', 'stroke="#2a78d6" stroke-width="2.5"')
    greedy_time_line = line("greedy_ms", lambda v: y_log(max(v, 0.001), t_lo, t_hi)).replace('stroke-width="2.5"', 'stroke="#eb6834" stroke-width="2.5" stroke-dasharray="6 4"')
    exh_dots = dots("exhaustive_total_ms", "#2a78d6", lambda v: y_log(max(v, 0.001), t_lo, t_hi))
    greedy_time_dots = dots("greedy_ms", "#eb6834", lambda v: y_log(max(v, 0.001), t_lo, t_hi))
    budget_y = y_log(budget_ms, t_lo, t_hi)

    xticks = "".join(
        f'<text x="{x(n):.1f}" y="{h-16}" text-anchor="middle" class="tick">{n}</text>'
        for n in ns
    )
    log_ticks = [0.001, 0.01, 0.1, 1, 10, 100, 1000]
    time_yticks = "".join(
        f'<line class="gridline" x1="{pad_l}" y1="{y_log(v, t_lo, t_hi):.1f}" x2="{w-pad_r}" y2="{y_log(v, t_lo, t_hi):.1f}"/>'
        f'<text x="{pad_l-8}" y="{y_log(v, t_lo, t_hi)+4:.1f}" text-anchor="end" class="tick">{v:g}</text>'
        for v in log_ticks if t_lo <= v <= t_hi
    )

    cost_tick_values = list(range(20, 71, 10))
    cost_yticks = "".join(
        f'<line class="gridline" x1="{pad_l}" y1="{y_linear(v, cost_lo, cost_hi):.1f}" x2="{w-pad_r}" y2="{y_linear(v, cost_lo, cost_hi):.1f}"/>'
        f'<text x="{pad_l-8}" y="{y_linear(v, cost_lo, cost_hi)+4:.1f}" text-anchor="end" class="tick">{v}</text>'
        for v in cost_tick_values if cost_lo <= v <= cost_hi
    )

    def hover_layer(kind: str) -> str:
        step = (w - pad_l - pad_r) / (max(ns) - min(ns))
        pieces = []
        for r in rows:
            n = int(r["n"])
            cx = x(float(n))
            tip_w, tip_h = 176, 92
            tx = cx + 12 if cx < w - tip_w - 28 else cx - tip_w - 12
            ty = pad_t + 6
            if kind == "cost":
                body = f"""
                <line x1="{tx+12}" y1="{ty+38}" x2="{tx+30}" y2="{ty+38}" stroke="#2a78d6" stroke-width="2.5"/>
                <text x="{tx+38}" y="{ty+42}">opt <tspan class="strong">{int(r['opt_cost'])}</tspan></text>
                <line x1="{tx+12}" y1="{ty+58}" x2="{tx+30}" y2="{ty+58}" stroke="#eb6834" stroke-width="2.5" stroke-dasharray="5 3"/>
                <text x="{tx+38}" y="{ty+62}">greedy <tspan class="strong">{int(r['greedy_cost'])}</tspan></text>
                <text x="{tx+12}" y="{ty+82}">gap <tspan class="strong">{float(r['gap_pct']):.2f}%</tspan></text>"""
                label = f"n={n}, optimal {int(r['opt_cost'])}, greedy {int(r['greedy_cost'])}"
            else:
                exh_ms = float(r["exhaustive_total_ms"])
                greedy_ms = float(r["greedy_ms"])
                body = f"""
                <line x1="{tx+12}" y1="{ty+38}" x2="{tx+30}" y2="{ty+38}" stroke="#2a78d6" stroke-width="2.5"/>
                <text x="{tx+38}" y="{ty+42}">exh <tspan class="strong">{exh_ms:.3f} ms</tspan></text>
                <line x1="{tx+12}" y1="{ty+58}" x2="{tx+30}" y2="{ty+58}" stroke="#eb6834" stroke-width="2.5" stroke-dasharray="5 3"/>
                <text x="{tx+38}" y="{ty+62}">NN <tspan class="strong">{greedy_ms:.4f} ms</tspan></text>
                <text x="{tx+12}" y="{ty+82}">budget <tspan class="strong">{budget_ms:g} ms</tspan></text>"""
                label = f"n={n}, exhaustive {exh_ms:.3f} ms, greedy {greedy_ms:.4f} ms"

            pieces.append(f"""
            <g class="hover" tabindex="0" aria-label="{label}">
              <rect x="{cx-step/2:.1f}" y="{pad_t}" width="{step:.1f}" height="{h-pad_t-pad_b}" fill="transparent"/>
              <line class="xhair" x1="{cx:.1f}" y1="{pad_t}" x2="{cx:.1f}" y2="{h-pad_b}"/>
              <g class="tip">
                <rect x="{tx:.1f}" y="{ty}" width="{tip_w}" height="{tip_h}" rx="8"/>
                <text x="{tx+12}" y="{ty+20}" class="tip-title">n = {n}</text>
                {body}
              </g>
            </g>""")
        return "".join(pieces)

    cost_hover = hover_layer("cost")
    time_hover = hover_layer("time")

    html = f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>multi-goal curves</title>
<style>
 body {{ margin:0; background:#f9f9f7; color:#0b0b0b; font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif; }}
 main {{ max-width:1120px; margin:24px auto; padding:0 20px; }}
 .grid {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(480px,1fr)); gap:18px; }}
 figure {{ margin:0; background:#fcfcfb; border:1px solid rgba(11,11,11,.10); border-radius:12px; padding:14px 16px 8px; }}
 h2 {{ font-size:16px; margin:0 0 4px; }}
 p.note {{ color:#52514e; margin:0 0 8px; font-size:13px; }}
 svg {{ width:100%; height:auto; display:block; }}
 .tick {{ fill:#898781; font-size:11px; font-variant-numeric:tabular-nums; }}
 .axis {{ stroke:#c3c2b7; stroke-width:1; }}
 .gridline {{ stroke:#e1e0d9; stroke-width:1; }}
 .legend {{ display:flex; gap:16px; align-items:center; color:#52514e; font-size:13px; }}
 .swatch {{ display:inline-block; width:18px; height:3px; border-radius:2px; margin-right:6px; vertical-align:middle; }}
 table {{ border-collapse:collapse; width:100%; margin-top:18px; background:#fcfcfb; border-radius:12px; overflow:hidden; }}
 th,td {{ padding:8px 10px; border-bottom:1px solid #e1e0d9; text-align:right; font-variant-numeric:tabular-nums; }}
 th:first-child,td:first-child {{ text-align:left; }}
 .pass {{ color:#006300; font-weight:700; }} .fail {{ color:#b42318; font-weight:700; }}
 .hover .xhair {{ visibility:hidden; stroke:#898781; stroke-width:1; stroke-dasharray:3 3; }}
 .hover .tip {{ visibility:hidden; }}
 .hover:hover .xhair, .hover:focus .xhair,
 .hover:hover .tip, .hover:focus .tip {{ visibility:visible; }}
 .hover:focus {{ outline:none; }}
 .tip rect {{ fill:#fcfcfb; stroke:#c3c2b7; stroke-width:1; }}
 .tip text {{ fill:#52514e; font-size:12px; }}
 .tip .tip-title {{ fill:#0b0b0b; font-size:13px; font-weight:700; }}
 .tip .strong {{ fill:#0b0b0b; font-weight:700; font-variant-numeric:tabular-nums; }}
</style></head><body><main>
<h1>多目标顺序：总代价与耗时曲线</h1>
<p class="note">蓝线为穷举最优；橙色虚线为最近邻贪心。耗时纵轴为对数毫秒。</p>
<div class="grid">
<figure>
  <h2>路径总代价</h2>
  <div class="legend"><span><i class="swatch" style="background:#2a78d6"></i>穷举最优</span><span><i class="swatch" style="background:#eb6834"></i>贪心最近邻</span></div>
  <svg viewBox="0 0 {w} {h}" role="img" aria-label="total path cost">
    <line class="axis" x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{h-pad_b}"/>
    <line class="axis" x1="{pad_l}" y1="{h-pad_b}" x2="{w-pad_r}" y2="{h-pad_b}"/>
    {cost_yticks}{xticks}{cost_opt_line}{cost_greedy_line}{opt_dots}{greedy_dots}{cost_hover}
  </svg>
</figure>
<figure>
  <h2>规划耗时</h2>
  <div class="legend"><span><i class="swatch" style="background:#2a78d6"></i>穷举（含矩阵）</span><span><i class="swatch" style="background:#eb6834"></i>贪心选择</span><span><i class="swatch" style="background:#898781"></i>{budget_ms:g} ms 上界</span></div>
  <svg viewBox="0 0 {w} {h}" role="img" aria-label="planning time">
    <line class="axis" x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{h-pad_b}"/>
    <line class="axis" x1="{pad_l}" y1="{h-pad_b}" x2="{w-pad_r}" y2="{h-pad_b}"/>
    <line x1="{pad_l}" y1="{budget_y:.1f}" x2="{w-pad_r}" y2="{budget_y:.1f}" stroke="#898781" stroke-width="1.5" stroke-dasharray="3 4"/>
    <text x="{w-pad_r}" y="{budget_y-6:.1f}" text-anchor="end" class="tick">budget {budget_ms:g} ms</text>
    {time_yticks}{xticks}{exh_time_line}{greedy_time_line}{exh_dots}{greedy_time_dots}{time_hover}
  </svg>
</figure>
</div>
<table><thead><tr><th>n</th><th>n!</th><th>A* calls</th><th>opt</th><th>greedy</th><th>gap</th><th>exh ms</th><th>greedy ms</th><th>check</th></tr></thead>
<tbody>
{''.join(f"<tr><td>{int(r['n'])}</td><td>{int(r['permutations'])}</td><td>{int(r['astar_calls'])}</td><td>{int(r['opt_cost'])}</td><td>{int(r['greedy_cost'])}</td><td>{r['gap_pct']:.2f}%</td><td>{r['exhaustive_total_ms']:.3f}</td><td>{r['greedy_ms']:.4f}</td><td class='{'pass' if r['gap_pct'] <= 10 else 'fail'}'>{'PASS' if r['gap_pct'] <= 10 else 'OVER 10%'}</td></tr>" for r in rows)}
</tbody></table>
</main></body></html>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--budget-ms", type=float, default=100.0)
    parser.add_argument("--tiers", type=str, default="3,6,9")
    parser.add_argument("--curve-max", type=int, default=10)
    default_out = Path(__file__).resolve().parents[1] / "artifacts" / "multigoal"
    parser.add_argument("--outdir", type=Path, default=default_out)
    args = parser.parse_args()

    tiers = [int(x) for x in args.tiers.split(",") if x.strip()]
    if any(n < 3 or n > args.curve_max for n in tiers):
        parser.error("all tiers must be between 3 and --curve-max")
    blocked = build_blocked()
    args.outdir.mkdir(parents=True, exist_ok=True)

    print("== 1. minimal bug reproduction ==")
    bug_pf = PathFinder()
    reached, first_path = buggy_visit_first_only(bug_pf, blocked, TARGETS[:3])
    print(f"buggy reached {len(reached)}/3 points: {reached}")
    print(f"buggy_stops_after_first={len(reached) == 1 and reached[0] == TARGETS[0]}")
    print(f"first leg A* cost={len(first_path) - 1}, astar_calls={bug_pf.astar_calls}")

    print("\n== 2. ordered multi-goal cost ==")
    print("C(pi) = d(S, T_pi1) + sum_{k=1}^{n-1} d(T_pik, T_pi(k+1))")
    print("pi* = argmin over all n! permutations; add d(T_pi_n,S) if return is required.")

    fix_pf = PathFinder()
    dist3, matrix_ms3 = build_cost_matrix(fix_pf, blocked, TARGETS[:3])
    opt3, order3, perm3, enum_ms3 = exhaustive_best(dist3, 3)
    fixed_path = execute_order(fix_pf, blocked, TARGETS[:3], order3)
    print(f"fixed order={order3}, total_cost={opt3}, executed_path_cells={len(fixed_path)}")
    print(render_map(blocked, TARGETS[:3], fixed_path))
    print("legend: S=start, 1/2/3=targets, *=concatenated path, #=wall")

    print("\n== 3. factorial scaling and reverse bound ==")
    print("given observations: K(3)=16 pathfindings for 6 orders; K(4)=60 for 24 orders")
    print(f"observed call ratio 60/16={60/16:.3f}; permutation ratio 4!/3!={math.factorial(4)/math.factorial(3):.1f}")
    kappa = max(16 / math.factorial(3), 60 / math.factorial(4))
    print(f"conservative factorial-call fit: K(n) <= {kappa:.3f} * n!  (matches K3=16, gives K4=64 vs observed 60)")

    rows: List[Dict[str, object]] = []
    numeric_rows: List[Dict[str, float]] = []
    all_astar_latency: List[float] = []
    tier_results: List[Tuple[int, int, Tuple[int, ...], int, Tuple[int, ...], float, bool]] = []

    print("\n== 4. curves: one run for each n ==")
    header = f"{'n':>2} {'n!':>9} {'A*calls':>7} {'opt':>4} {'greedy':>6} {'gap%':>6} {'exh_ms':>9} {'greedy_ms':>10}"
    print(header)

    for n in range(3, args.curve_max + 1):
        pf = PathFinder()
        dist, matrix_ms = build_cost_matrix(pf, blocked, TARGETS[:n])
        all_astar_latency.extend(pf.durations_ms)
        opt_cost, opt_order, nperm, enum_ms = exhaustive_best(dist, n)
        greedy_cost, greedy_order, greedy_ms = greedy_nearest_neighbour(dist, n)
        total_exh_ms = matrix_ms + enum_ms
        gap = gap_percent(greedy_cost, opt_cost)

        row: Dict[str, object] = {
            "n": n,
            "permutations": nperm,
            "astar_calls_matrix": pf.astar_calls,
            "astar_calls": pf.astar_calls,
            "matrix_ms": round(matrix_ms, 6),
            "exhaustive_order_ms": round(enum_ms, 6),
            "exhaustive_total_ms": round(total_exh_ms, 6),
            "greedy_ms": round(greedy_ms, 6),
            "opt_cost": opt_cost,
            "opt_order": ">".join(map(str, opt_order)),
            "greedy_cost": greedy_cost,
            "greedy_order": ">".join(map(str, greedy_order)),
            "gap_pct": round(gap, 4),
            "within_10pct": gap <= 10.0,
        }
        rows.append(row)
        numeric_rows.append({
            "n": float(n), "permutations": float(nperm),
            "astar_calls": float(pf.astar_calls),
            "exhaustive_total_ms": total_exh_ms,
            "greedy_ms": greedy_ms, "opt_cost": float(opt_cost),
            "greedy_cost": float(greedy_cost), "gap_pct": gap,
        })
        print(f"{n:>2} {nperm:>9} {pf.astar_calls:>7} {opt_cost:>4} {greedy_cost:>6} {gap:>6.2f} {total_exh_ms:>9.3f} {greedy_ms:>10.4f}")

        if n in tiers:
            ok = gap <= 10.0
            tier_results.append((n, opt_cost, opt_order, greedy_cost, greedy_order, gap, ok))
            # Materialize the selected greedy grid path as an end-to-end check.
            greedy_path = execute_order(PathFinder(), blocked, TARGETS[:n], greedy_order)
            greedy_edges = len(greedy_path) - 1
            if greedy_edges != greedy_cost:
                raise RuntimeError(f"cost/path mismatch n={n}: {greedy_edges} != {greedy_cost}")

    write_csv(rows, args.outdir / "curve.csv")
    write_html_chart(numeric_rows, args.outdir / "curves.html", args.budget_ms)

    # Fit the asymptotic coefficient on the largest three measured sizes; tiny-n
    # timings are dominated by interpreter/scheduling noise.
    fit_rows = rows[-3:]
    ms_per_order = max(
        float(r["exhaustive_order_ms"]) / math.factorial(int(r["n"]))
        for r in fit_rows
    )
    ms_per_segment = max(
        float(r["exhaustive_order_ms"]) / (int(r["n"]) * math.factorial(int(r["n"])))
        for r in fit_rows
    )
    tau_ms = statistics.median(all_astar_latency)

    def predicted_exhaustive_ms(n: int, budget_tau: float = tau_ms) -> float:
        return (
            ms_per_segment * n * math.factorial(n)
            + budget_tau * n * (n + 1) / 2
        )

    def largest_cached_n(budget: float) -> int:
        n = 1
        while predicted_exhaustive_ms(n + 1) <= budget:
            n += 1
        return n

    nmax_time = largest_cached_n(args.budget_ms)
    stated_nmax = largest_factorial_n(args.budget_ms / kappa, tau_ms)  # kappa*n!*tau <= budget

    def largest_n_naive(budget: float, tau: float) -> int:
        n = 1
        while (n + 1) * math.factorial(n + 1) * tau <= budget:
            n += 1
        return n

    naive_nmax = largest_n_naive(args.budget_ms, tau_ms)

    print("\n== 5. reverse solve for maximum enumerable n ==")
    print(f"measured coefficients: a={ms_per_segment * 1e6:.2f} ns/segment, "
          f"c={ms_per_order * 1000:.4f} us/order near the cutoff")
    print("bound: a*n*n! + tau*n(n+1)/2 <= B; the n*n! term is still factorial-scale.")
    print(f"budget B={args.budget_ms:g} ms")
    for b in (16.0, 100.0, 1000.0):
        print(f"  B={b:>6g} ms -> n_max={largest_cached_n(b)} with cached pair-cost matrix")
    print(f"primary bound at B={args.budget_ms:g} ms -> n_max={nmax_time}")
    print(f"median A* latency tau={tau_ms * 1000:.1f} us")
    print(f"stated K(n)~={kappa:.3f}n! path-calls + measured tau -> n_max={stated_nmax}")
    print(f"naive n*n! re-pathing every leg -> n_max={naive_nmax}")

    first_over = next((int(r["n"]) for r in rows if float(r["exhaustive_total_ms"]) > args.budget_ms), None)
    print("\n== 6. three required tiers ==")
    all_ok = True
    for n, opt_cost, opt_order, gcost, gorder, gap, ok in tier_results:
        all_ok &= ok
        status = "PASS <=10%" if ok else "OVER_10PCT"
        print(f"n={n}: optimal_cost={opt_cost} optimal_order={opt_order}")
        print(f"      greedy_cost={gcost} greedy_order={gorder} gap={gap:.2f}% {status}")
        if not ok:
            print(f"      ACTUAL_COST={gcost}; SELECTED_ORDER={gorder}")

    if first_over is not None:
        row = next(r for r in rows if int(r["n"]) == first_over)
        gap = float(row["gap_pct"])
        print("\n== switch decision ==")
        print(f"time curve first exceeds {args.budget_ms:g} ms at n={first_over} "
              f"({float(row['exhaustive_total_ms']):.3f} ms)")
        if gap <= 10.0:
            print(f"cost curve at n={first_over}: greedy gap={gap:.2f}% <=10%; switch exhaustive -> nearest-neighbour greedy here.")
        else:
            print(f"cost curve at n={first_over}: greedy gap={gap:.2f}% >10%; plain NN is not acceptable; "
                  "report actual order/cost and add 2-opt/beam/DP.")
            print(f"ACTUAL_COST={row['greedy_cost']}; SELECTED_ORDER={row['greedy_order']}")
            all_ok = False
    else:
        print("\nswitch decision: no measured n exceeded the time budget; keep exhaustive in the measured range.")

    print(f"\nwrote {args.outdir / 'curve.csv'}")
    print(f"wrote {args.outdir / 'curves.html'}")
    print(f"all_tiers_within_10pct={all_ok}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
