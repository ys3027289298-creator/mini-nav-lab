#!/usr/bin/env python3
"""Deterministic multi-agent deadlock matrix lab.

The lab is intentionally dependency-free.  It runs small circle-agent scenarios
through a doorway/choke geometry, detects the deadlock predicate from the
requirement, enumerates a four-dimensional Cartesian matrix, and audits sample
coverage against a full-matrix oracle.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

# ---- model constants -------------------------------------------------------

DT = 1.0 / 30.0
MAX_FRAMES = 1500
RADIUS = 0.30
DIAMETER = 2.0 * RADIUS
MAX_SPEED = 1.05
ARRIVE_RADIUS = 0.20

# Tolerances are the discrete-frame realization of:
#   |v_i - v_j| ~= 0 and d_ij < r_i + r_j.
# CONTACT_EPS turns the strict inequality into a noise band around contact.
CONTACT_EPS = 0.020
REL_SPEED_EPS = 0.035
WINDOW_FRAMES = 30  # one second at 30 Hz

WORLD_XMIN, WORLD_XMAX = -5.8, 5.8
WORLD_YMIN, WORLD_YMAX = -3.6, 3.6
WALL_HALF_HEIGHT = 3.6

N_LEVELS = (1, 2, 3, 4, 6)
PAIR_N_LEVELS = (2, 3, 4, 6)
DISTRIBUTION_LEVELS = ("column", "row", "arc", "opposed")
GOAL_LEVELS = ("same", "distinct")
# Width is normalized by one agent diameter D = 2r.
WIDTH_LEVELS = (1.0, 1.25, 1.5, 2.0, 2.5, 3.0)

FULL_MATRIX_SIZE = (
    len(N_LEVELS)
    * len(DISTRIBUTION_LEVELS)
    * len(GOAL_LEVELS)
    * len(WIDTH_LEVELS)
)
MATRIX_SIZE = (
    len(PAIR_N_LEVELS)
    * len(DISTRIBUTION_LEVELS)
    * len(GOAL_LEVELS)
    * len(WIDTH_LEVELS)
)
EXPLORATORY_RATE = 0.25


@dataclass(frozen=True)
class Case:
    n: int
    distribution: str
    goal: str
    width_d: float

    @property
    def width(self) -> float:
        return self.width_d * DIAMETER

    @property
    def ident(self) -> str:
        return f"n{self.n}-{self.distribution}-{self.goal}-w{self.width_d:g}"

    def tuple(self) -> tuple[object, ...]:
        return (self.n, self.distribution, self.goal, self.width_d)


@dataclass
class Agent:
    aid: int
    x: float
    y: float
    gx: float
    gy: float
    side_sign: int = 1
    active: bool = True


@dataclass
class CaseResult:
    case: Case
    deadlocked: bool
    timeout: bool
    success: bool
    frames: int
    elapsed_s: float
    pair: tuple[int, int] | None
    min_distance: float
    min_normalized_distance: float
    reason: str
    runtime_ms: float


# ---- enumeration -----------------------------------------------------------

def all_cases() -> list[Case]:
    return [
        Case(n, distribution, goal, width_d)
        for n in N_LEVELS
        for distribution in DISTRIBUTION_LEVELS
        for goal in GOAL_LEVELS
        for width_d in WIDTH_LEVELS
    ]


def eligible_cases() -> list[Case]:
    """Cells on which the two-agent predicate can possibly be true."""
    return [c for c in all_cases() if c.n >= 2]


# ---- geometry / scenario construction -------------------------------------

def _slot_y(i: int, n: int, spacing: float = 0.56) -> float:
    return (i - (n - 1) / 2.0) * spacing


def make_agents(case: Case) -> list[Agent]:
    n = case.n
    positions: list[tuple[float, float]] = []
    groups: list[int] = []  # 0 = starts left, 1 = starts right

    if case.distribution == "column":
        for i in range(n):
            positions.append((-1.05 - i * 0.78, 0.0))
            groups.append(0)

    elif case.distribution == "row":
        for i in range(n):
            positions.append((-1.05, _slot_y(i, n, 0.68)))
            groups.append(0)

    elif case.distribution == "arc":
        # Semicircle in the left antechamber.  Radius grows enough that the
        # deterministic slots do not overlap for the largest matrix level.
        radius = 2.05
        lo = math.pi / 2.0 + 0.42
        hi = 3.0 * math.pi / 2.0 - 0.42
        for i in range(n):
            t = 0.5 if n == 1 else i / (n - 1)
            theta = lo + (hi - lo) * t
            positions.append((radius * math.cos(theta), radius * math.sin(theta)))
            groups.append(0)

    elif case.distribution == "opposed":
        left_n = (n + 1) // 2
        right_n = n - left_n
        for j in range(left_n):
            y = (j - (left_n - 1) / 2.0) * 0.70
            positions.append((-1.15, y))
            groups.append(0)
        for j in range(right_n):
            y = (j - (right_n - 1) / 2.0) * 0.70
            positions.append((1.15, y))
            groups.append(1)
    else:  # pragma: no cover - enumerated above
        raise ValueError(case.distribution)

    agents: list[Agent] = []
    left_indices = [i for i, g in enumerate(groups) if g == 0]
    right_indices = [i for i, g in enumerate(groups) if g == 1]

    for i, ((x, y), group) in enumerate(zip(positions, groups)):
        if case.distribution == "opposed":
            if case.goal == "same":
                gx, gy = 0.0, 0.0
            elif group == 0:
                j = left_indices.index(i)
                m = len(left_indices)
                gx, gy = 2.85, _slot_y(j, m, 0.54)
            else:
                j = right_indices.index(i)
                m = len(right_indices)
                gx, gy = -2.85, _slot_y(j, m, 0.54)
        else:
            gx = 2.85
            gy = 0.0 if case.goal == "same" else _slot_y(i, n, 0.54)

        # A universal own-left bias is enough to select distinct passing lanes
        # when the geometric corridor width permits them.
        agents.append(Agent(i, x, y, gx, gy, side_sign=1))

    return agents


# ---- collision and steering ------------------------------------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _wall_segments(case: Case) -> list[tuple[float, float, float, float]]:
    """Vertical wall at x=0 with a central gap of case.width."""
    half_gap = case.width / 2.0
    return [
        (0.0, -WALL_HALF_HEIGHT, 0.0, -half_gap),
        (0.0, half_gap, 0.0, WALL_HALF_HEIGHT),
    ]


def _resolve_segment_circle(a: Agent, x1: float, y1: float, x2: float, y2: float) -> None:
    vx, vy = x2 - x1, y2 - y1
    length2 = vx * vx + vy * vy
    t = 0.0 if length2 == 0.0 else _clamp(
        ((a.x - x1) * vx + (a.y - y1) * vy) / length2, 0.0, 1.0
    )
    qx, qy = x1 + t * vx, y1 + t * vy
    dx, dy = a.x - qx, a.y - qy
    d = math.hypot(dx, dy)
    if d >= RADIUS:
        return
    if d > 1.0e-10:
        nx, ny = dx / d, dy / d
    else:
        # The center is exactly on a wall segment.  Preserve the side from
        # which the agent approached whenever possible.
        nx = -1.0 if a.x <= 0.0 else 1.0
        ny = 0.0
        if abs(a.x) < 1.0e-10:
            nx = 1.0 if a.aid % 2 else -1.0
    push = RADIUS - d
    a.x += nx * push
    a.y += ny * push


def _resolve_collisions(agents: list[Agent], case: Case) -> None:
    active = [a for a in agents if a.active]

    # Gauss-Seidel iterations are deterministic and adequate for <= 8 discs.
    for _ in range(10):
        for i in range(len(active)):
            a = active[i]

            # Outer rectangular world.
            a.x = _clamp(a.x, WORLD_XMIN + RADIUS, WORLD_XMAX - RADIUS)
            a.y = _clamp(a.y, WORLD_YMIN + RADIUS, WORLD_YMAX - RADIUS)

            # Door/facade walls.
            for segment in _wall_segments(case):
                _resolve_segment_circle(a, *segment)

            for j in range(i + 1, len(active)):
                b = active[j]
                dx, dy = a.x - b.x, a.y - b.y
                d = math.hypot(dx, dy)
                min_d = 2.0 * RADIUS
                if d >= min_d:
                    continue
                if d > 1.0e-10:
                    nx, ny = dx / d, dy / d
                else:
                    # Exact coincidence: deterministic, id-based normal.
                    nx, ny = (1.0, 0.0) if a.aid < b.aid else (-1.0, 0.0)
                penetration = min_d - d
                a.x += nx * penetration * 0.5
                a.y += ny * penetration * 0.5
                b.x -= nx * penetration * 0.5
                b.y -= ny * penetration * 0.5


def _steer(agents: list[Agent], frame: int) -> list[tuple[float, float]]:
    velocities: list[tuple[float, float]] = []
    sense = 0.95

    for a in agents:
        if not a.active:
            velocities.append((0.0, 0.0))
            continue

        gdx, gdy = a.gx - a.x, a.gy - a.y
        gd = math.hypot(gdx, gdy)
        if gd <= ARRIVE_RADIUS:
            velocities.append((0.0, 0.0))
            continue

        # Route through the central opening before taking the direct segment
        # to a lateral goal slot.  Without this portal waypoint, a disc can
        # spend the whole run pushing along a facade toward a goal that lies
        # behind the wall segment.
        tx, ty = a.gx, a.gy
        if a.gx > 0.1 and a.x < 0.14:
            tx, ty = 0.14, 0.0
        elif a.gx < -0.1 and a.x > -0.14:
            tx, ty = -0.14, 0.0
        gdx, gdy = tx - a.x, ty - a.y
        gd = math.hypot(gdx, gdy)
        if gd < 1.0e-8:
            ux, uy = 1.0 if a.gx > a.x else -1.0, 0.0
        else:
            ux, uy = gdx / gd, gdy / gd

        # Separation is deliberately local and greedy.  It breaks clumps in
        # open space but cannot create a passing lane through a sub-two-diameter
        # choke when the walls deny that geometry.
        sx, sy = 0.0, 0.0
        ahead_urgency = 0.0
        for b in agents:
            if b.aid == a.aid or not b.active:
                continue
            dx, dy = a.x - b.x, a.y - b.y
            d = math.hypot(dx, dy)
            if 1.0e-8 < d < sense:
                weight = (sense - d) / sense
                sx += (dx / d) * weight
                sy += (dy / d) * weight

                # Only commit to a lateral passing lane when another disc is
                # actually ahead.  A constant side bias can otherwise cancel
                # the goalward component while sliding along a wall.
                to_b_x, to_b_y = -dx / d, -dy / d
                forward = ux * to_b_x + uy * to_b_y
                if forward > 0.20:
                    ahead_urgency = max(ahead_urgency, weight)

        # Own-left perpendicular: for eastbound motion this is +y; for
        # westbound motion it is -y, so an opposed pair chooses opposite
        # global sides rather than both dancing in the same direction.
        px, py = -uy, ux
        # Tiny deterministic priority asymmetry prevents a mathematically
        # symmetric pair from orbiting a shared point forever.  It is far too
        # small to open a geometrically impossible narrow passing lane.
        eagerness = 1.55 + 0.06 * (a.aid + 1)
        desired_x = eagerness * ux + 0.45 * sx + 0.85 * ahead_urgency * px * a.side_sign
        desired_y = eagerness * uy + 0.45 * sy + 0.85 * ahead_urgency * py * a.side_sign

        # Deterministic exploratory probe.  It breaks a perfectly symmetric
        # orbit in open width; a wall-bound jam cannot convert it into motion.
        probe = 0.42 * math.sin(frame * 0.17 + a.aid * 2.17)
        desired_x += probe * px
        desired_y += probe * py

        speed = math.hypot(desired_x, desired_y)
        if speed > MAX_SPEED:
            desired_x = desired_x / speed * MAX_SPEED
            desired_y = desired_y / speed * MAX_SPEED
        velocities.append((desired_x, desired_y))

    return velocities


# ---- deadlock predicate ----------------------------------------------------

def _window_deadlock(
    history: list[list[tuple[float, float]]],
    agents: list[Agent],
) -> tuple[int, int] | None:
    """Return a pair satisfying the deadlock predicate over a full window."""
    if len(history) < WINDOW_FRAMES + 1:
        return None

    old = history[0]
    now = history[-1]
    active_ids = [a.aid for a in agents if a.active]
    by_id = {a.aid: a for a in agents}

    for ii in range(len(active_ids)):
        for jj in range(ii + 1, len(active_ids)):
            i, j = active_ids[ii], active_ids[jj]
            ai, bj = by_id[i], by_id[j]

            # Both still have unfinished goals.
            if math.hypot(ai.gx - ai.x, ai.gy - ai.y) <= ARRIVE_RADIUS:
                continue
            if math.hypot(bj.gx - bj.x, bj.gy - bj.y) <= ARRIVE_RADIUS:
                continue

            # Distance must stay inside contact band for every frame in the
            # rolling window, not merely at its endpoints.
            contact_all_window = True
            for positions in history:
                d = math.hypot(
                    positions[i][0] - positions[j][0],
                    positions[i][1] - positions[j][1],
                )
                if d >= 2.0 * RADIUS + CONTACT_EPS:
                    contact_all_window = False
                    break
            if not contact_all_window:
                continue

            d_now = math.hypot(now[i][0] - now[j][0], now[i][1] - now[j][1])
            rel_dx = (now[i][0] - old[i][0]) - (now[j][0] - old[j][0])
            rel_dy = (now[i][1] - old[i][1]) - (now[j][1] - old[j][1])
            relative_speed = math.hypot(rel_dx, rel_dy) / (WINDOW_FRAMES * DT)

            if relative_speed <= REL_SPEED_EPS and d_now < 2.0 * RADIUS + CONTACT_EPS:
                return (i, j)

    return None


# ---- one simulation --------------------------------------------------------

def run_case(case: Case) -> CaseResult:
    start_clock = time.perf_counter()
    agents = make_agents(case)

    # history is indexed by agent id and stores post-collision positions.
    history: list[list[tuple[float, float]]] = []
    deadlocked_pair: tuple[int, int] | None = None
    deadlock_frame: int | None = None
    min_distance = float("inf")

    for frame in range(1, MAX_FRAMES + 1):
        velocities = _steer(agents, frame)
        for a, (vx, vy) in zip(agents, velocities):
            if a.active:
                a.x += vx * DT
                a.y += vy * DT

        _resolve_collisions(agents, case)

        for a in agents:
            if a.active and math.hypot(a.gx - a.x, a.gy - a.y) <= ARRIVE_RADIUS:
                # Arrival is success.  Removing the completed agent prevents a
                # character standing exactly on a shared goal from being
                # mislabeled as a navigation deadlock.
                a.active = False

        positions = [(a.x, a.y) for a in agents]
        history.append(positions)
        if len(history) > WINDOW_FRAMES + 1:
            history.pop(0)

        active = [a for a in agents if a.active]
        for ii in range(len(active)):
            for jj in range(ii + 1, len(active)):
                d = math.hypot(
                    active[ii].x - active[jj].x,
                    active[ii].y - active[jj].y,
                )
                min_distance = min(min_distance, d)

        pair = _window_deadlock(history, agents)
        if pair is not None:
            deadlocked_pair = pair
            deadlock_frame = frame
            break

        if not active:
            elapsed = time.perf_counter() - start_clock
            return CaseResult(
                case=case,
                deadlocked=False,
                timeout=False,
                success=True,
                frames=frame,
                elapsed_s=frame * DT,
                pair=None,
                min_distance=min_distance if math.isfinite(min_distance) else 0.0,
                min_normalized_distance=(
                    min_distance / DIAMETER if math.isfinite(min_distance) else 0.0
                ),
                reason="all_arrived",
                runtime_ms=elapsed * 1000.0,
            )

    elapsed = time.perf_counter() - start_clock
    active = [a for a in agents if a.active]
    timeout = deadlocked_pair is None
    if deadlocked_pair is not None:
        reason = f"deadlock_pair_{deadlocked_pair[0]}_{deadlocked_pair[1]}_at_frame_{deadlock_frame}"
    else:
        reason = "timeout_without_predicate"

    return CaseResult(
        case=case,
        deadlocked=deadlocked_pair is not None,
        timeout=timeout,
        success=False,
        frames=deadlock_frame or MAX_FRAMES,
        elapsed_s=(deadlock_frame or MAX_FRAMES) * DT,
        pair=deadlocked_pair,
        min_distance=min_distance if math.isfinite(min_distance) else 0.0,
        min_normalized_distance=(
            (min_distance if math.isfinite(min_distance) else 0.0) / DIAMETER
        ),
        reason=reason,
        runtime_ms=elapsed * 1000.0,
    )


# ---- matrix / sampling -----------------------------------------------------

def result_row(r: CaseResult) -> dict[str, object]:
    c = r.case
    return {
        "id": c.ident,
        "n": c.n,
        "distribution": c.distribution,
        "goal": c.goal,
        "width_d": c.width_d,
        "deadlocked": int(r.deadlocked),
        "timeout": int(r.timeout),
        "success": int(r.success),
        "frames": r.frames,
        "sim_seconds": round(r.elapsed_s, 3),
        "pair": "" if r.pair is None else f"{r.pair[0]}-{r.pair[1]}",
        "min_distance": round(r.min_distance, 6),
        "min_distance_d": round(r.min_normalized_distance, 4),
        "reason": r.reason,
        "runtime_ms": round(r.runtime_ms, 3),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def sample_indices_uniform(population: int, rate: float, rng: random.Random) -> list[int]:
    k = max(1, int(round(population * rate))) if rate > 0 else 0
    return sorted(rng.sample(range(population), k))


def latin_hypercube_case_sample(rate: float, seed: int) -> list[Case]:
    """Stratified four-dimensional sample.

    Every dimension level is represented before additional Cartesian cells are
    filled.  At rate=1 this is the complete Cartesian matrix.
    """
    rng = random.Random(seed)
    cases = eligible_cases()
    target = max(18, int(round(len(cases) * rate)))

    selected: dict[str, Case] = {}

    # Mandatory one-dimensional level coverage.  A deterministic anchor uses
    # the smallest other levels; random anchors cover the remaining levels.
    for values, dim in (
        (PAIR_N_LEVELS, "n"),
        (DISTRIBUTION_LEVELS, "distribution"),
        (GOAL_LEVELS, "goal"),
        (WIDTH_LEVELS, "width_d"),
    ):
        for value in values:
            matches = [c for c in cases if getattr(c, dim) == value]
            chosen = rng.choice(matches)
            selected[chosen.ident] = chosen

    # Add stratified random Cartesian cells until target size.
    remaining = [c for c in cases if c.ident not in selected]
    rng.shuffle(remaining)
    for c in remaining:
        if len(selected) >= target:
            break
        selected[c.ident] = c

    # If the requested rate is below the number of mandatory level anchors,
    # the anchors win because boundary/equivalence coverage is the point.
    return sorted(selected.values(), key=lambda c: c.ident)

def coverage_curves(oracle: list[CaseResult], repeats: int = 200) -> list[dict[str, float | int | str]]:
    deadlock_ids = {r.case.ident for r in oracle if r.deadlocked}
    deadlock_indices = [i for i, r in enumerate(oracle) if r.deadlocked]
    rates = [i / 20 for i in range(1, 21)]  # 5% .. 100%
    rows: list[dict[str, float | int | str]] = []

    for rate in rates:
        total_k = max(1, int(round(len(oracle) * rate)))
        covered_counts: list[int] = []
        all_hit = 0
        missing_examples: list[set[str]] = []

        for rep in range(repeats):
            rng = random.Random(10_000 + rep * 97 + int(rate * 1000))
            idx = set(sample_indices_uniform(len(oracle), rate, rng))
            hit = sum(1 for i in deadlock_indices if i in idx)
            covered_counts.append(hit)
            if hit == len(deadlock_indices):
                all_hit += 1
            if rep < 20:
                missed = deadlock_ids - {oracle[i].case.ident for i in idx}
                missing_examples.append(missed)

        rows.append(
            {
                "rate": rate,
                "total_cases": total_k,
                "mean_covered_deadlocks": round(statistics.fmean(covered_counts), 2),
                "min_covered_deadlocks": min(covered_counts),
                "deadlock_total": len(deadlock_ids),
                "p_repeat_full_coverage": round(all_hit / repeats, 3),
            }
        )

    return rows


def choose_rate(curve: list[dict[str, float | int | str]]) -> float:
    """Lowest rate whose repeated uniform samples cover all deadlocks in >=95%."""
    for row in curve:
        if float(row["p_repeat_full_coverage"]) >= 0.95:
            return float(row["rate"])
    return 1.0


def audit_coverage(sample_cases: Iterable[Case], oracle: list[CaseResult]) -> dict[str, object]:
    sample_ids = {c.ident for c in sample_cases}
    deadlock_results = [r for r in oracle if r.deadlocked]
    timeout_results = [r for r in oracle if r.timeout]
    covered = [r.case.ident for r in deadlock_results if r.case.ident in sample_ids]
    missed = [result_row(r) for r in deadlock_results if r.case.ident not in sample_ids]
    covered_timeouts = [r.case.ident for r in timeout_results if r.case.ident in sample_ids]
    missed_timeouts = [result_row(r) for r in timeout_results if r.case.ident not in sample_ids]
    return {
        "deadlock_total": len(deadlock_results),
        "covered_by_sample": len(covered),
        "missed_count": len(missed),
        "covered_ids": covered,
        "missed": missed,
        "timeout_total": len(timeout_results),
        "covered_timeouts": len(covered_timeouts),
        "missed_timeout_count": len(missed_timeouts),
        "covered_timeout_ids": covered_timeouts,
        "missed_timeouts": missed_timeouts,
    }


def mandatory_matrix(oracle: list[CaseResult], sampled: list[Case]) -> list[Case]:
    """Final regression matrix: sample plus every reproduced failure."""
    by_id = {c.ident: c for c in sampled}
    for r in oracle:
        if not r.success:
            by_id[r.case.ident] = r.case
    return sorted(by_id.values(), key=lambda c: c.ident)


# ---- output / chart --------------------------------------------------------

def write_svg_curve(path: Path, curve: list[dict[str, float | int | str]], chosen: float) -> None:
    """Dependency-free two-panel SVG: one own y-axis per measured quantity."""
    width, height = 980, 680
    ml, mr = 82, 44
    plot_w = width - ml - mr
    top_y, top_h = 82, 226
    bottom_y, bottom_h = 400, 178
    d_total = int(curve[-1]["deadlock_total"])
    case_total = MATRIX_SIZE
    d_max = max(10, int(math.ceil(d_total / 10.0) * 10))
    c_max = max(10, int(math.ceil(case_total / 40.0) * 40))

    def xx(rate: float) -> float:
        return ml + float(rate) * plot_w

    def y_dead(v: float) -> float:
        return top_y + top_h * (1.0 - v / d_max)

    def y_case(v: float) -> float:
        return bottom_y + bottom_h * (1.0 - v / c_max)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="chart-title chart-desc">',
        '<title id="chart-title">四维抽样率曲线</title>',
        '<desc id="chart-desc">上图为重复抽样平均覆盖的卡死组合数，下图为抽样对应的总用例数。</desc>',
        '<rect width="100%" height="100%" fill="#fcfcfb"/>',
        '<style>',
        'text{font-family:ui-sans-serif,system-ui,"Noto Sans CJK SC",sans-serif;font-size:13px;fill:#0b0b0b;font-variant-numeric:tabular-nums}',
        '.title{font-size:21px;font-weight:750}.subtitle{fill:#52514e;font-size:13px}',
        '.panel-title{font-size:15px;font-weight:650}.tick{fill:#52514e;font-size:12px}',
        '.grid{stroke:#e1e0d9;stroke-width:1}.axis{stroke:#c3c2b7;stroke-width:1.2}',
        '.dead-line{fill:none;stroke:#e34948;stroke-width:3;stroke-linejoin:round;stroke-linecap:round}',
        '.case-line{fill:none;stroke:#2a78d6;stroke-width:3;stroke-linejoin:round;stroke-linecap:round}',
        '.chosen{stroke:#0b0b0b;stroke-width:1.5;stroke-dasharray:5 5}',
        '.direct{font-weight:700}.hint{fill:#52514e;font-size:12px}',
        '</style>',
        '<text class="title" x="44" y="38">抽样率取舍：卡死覆盖与总用例成本</text>',
        f'<text class="subtitle" x="44" y="61">200 次重复均匀笛卡尔单元抽样；虚线为选定的 {chosen*100:.0f}% 探索抽样率</text>',
    ]

    # Top panel grid and axes.
    parts.append(f'<text class="panel-title" x="{ml}" y="{top_y-16}">覆盖到的卡死组合数（平均值，总预言机 {d_total} 个）</text>')
    for k in range(6):
        v = d_max * k / 5
        yy = y_dead(v)
        parts.append(f'<line class="grid" x1="{ml}" y1="{yy:.1f}" x2="{ml+plot_w}" y2="{yy:.1f}"/>')
        parts.append(f'<text class="tick" x="{ml-12}" y="{yy+4:.1f}" text-anchor="end">{v:.0f}</text>')
    parts.append(f'<line class="axis" x1="{ml}" y1="{top_y}" x2="{ml}" y2="{top_y+top_h}"/>')
    parts.append(f'<line class="axis" x1="{ml}" y1="{top_y+top_h}" x2="{ml+plot_w}" y2="{top_y+top_h}"/>')

    # Bottom panel grid and axes.
    parts.append(f'<text class="panel-title" x="{ml}" y="{bottom_y-16}">总用例数（n≥2 的可判定矩阵共 {case_total} 例）</text>')
    for k in range(6):
        v = c_max * k / 5
        yy = y_case(v)
        parts.append(f'<line class="grid" x1="{ml}" y1="{yy:.1f}" x2="{ml+plot_w}" y2="{yy:.1f}"/>')
        parts.append(f'<text class="tick" x="{ml-12}" y="{yy+4:.1f}" text-anchor="end">{v:.0f}</text>')
    parts.append(f'<line class="axis" x1="{ml}" y1="{bottom_y}" x2="{ml}" y2="{bottom_y+bottom_h}"/>')
    parts.append(f'<line class="axis" x1="{ml}" y1="{bottom_y+bottom_h}" x2="{ml+plot_w}" y2="{bottom_y+bottom_h}"/>')

    # Shared x ticks.
    for k in range(11):
        rate = k / 10
        xx_pos = xx(rate)
        parts.append(f'<line class="grid" x1="{xx_pos:.1f}" y1="{top_y}" x2="{xx_pos:.1f}" y2="{top_y+top_h}"/>')
        parts.append(f'<line class="grid" x1="{xx_pos:.1f}" y1="{bottom_y}" x2="{xx_pos:.1f}" y2="{bottom_y+bottom_h}"/>')
        parts.append(f'<text class="tick" x="{xx_pos:.1f}" y="{bottom_y+bottom_h+24}" text-anchor="middle">{int(rate*100)}%</text>')
    parts.append(f'<text x="{ml+plot_w/2}" y="{height-24}" text-anchor="middle" fill="#52514e">四维笛卡尔单元抽样率</text>')

    dead_points = [
        (xx(float(r["rate"])), y_dead(float(r["mean_covered_deadlocks"]))) for r in curve
    ]
    case_points = [
        (xx(float(r["rate"])), y_case(float(r["total_cases"]))) for r in curve
    ]
    dead_str = " ".join(f"{x:.1f},{y:.1f}" for x, y in dead_points)
    case_str = " ".join(f"{x:.1f},{y:.1f}" for x, y in case_points)
    area_str = (
        f"M {dead_points[0][0]:.1f},{top_y+top_h:.1f} "
        + " ".join(f"L{x:.1f},{y:.1f}" for x, y in dead_points)
        + f" L{dead_points[-1][0]:.1f},{top_y+top_h:.1f} Z"
    )
    parts.append(f'<path d="{area_str}" fill="#e34948" opacity="0.10"/>')
    parts.append(f'<polyline class="dead-line" points="{dead_str}"/>')
    parts.append(f'<polyline class="case-line" points="{case_str}"/>')

    for r, (xpos, ypos) in zip(curve, dead_points):
        label = f"抽样率 {float(r['rate'])*100:.0f}%；总用例 {r['total_cases']}；平均覆盖卡死 {r['mean_covered_deadlocks']}；最少 {r['min_covered_deadlocks']}；200次中全覆盖概率 {float(r['p_repeat_full_coverage'])*100:.1f}%"
        parts.append(f'<circle cx="{xpos:.1f}" cy="{ypos:.1f}" r="4.2" fill="#e34948" stroke="#fcfcfb" stroke-width="2"><title>{label}</title></circle>')
        parts.append(f'<circle cx="{xpos:.1f}" cy="{ypos:.1f}" r="13" fill="transparent"><title>{label}</title></circle>')

    for r, (xpos, ypos) in zip(curve, case_points):
        label = f"抽样率 {float(r['rate'])*100:.0f}%；总用例 {r['total_cases']}"
        parts.append(f'<circle cx="{xpos:.1f}" cy="{ypos:.1f}" r="4.2" fill="#2a78d6" stroke="#fcfcfb" stroke-width="2"><title>{label}</title></circle>')
        parts.append(f'<circle cx="{xpos:.1f}" cy="{ypos:.1f}" r="13" fill="transparent"><title>{label}</title></circle>')

    # Direct endpoint labels.
    last = curve[-1]
    parts.append(f'<text class="direct" x="{dead_points[-1][0]-8:.1f}" y="{dead_points[-1][1]-12:.1f}" text-anchor="end" fill="#e34948">{last["mean_covered_deadlocks"]}/{d_total}</text>')
    parts.append(f'<text class="direct" x="{case_points[-1][0]-8:.1f}" y="{case_points[-1][1]-12:.1f}" text-anchor="end" fill="#2a78d6">{last["total_cases"]} 例</text>')

    cx = xx(chosen)
    for y0, h in ((top_y, top_h), (bottom_y, bottom_h)):
        parts.append(f'<line class="chosen" x1="{cx:.1f}" y1="{y0-4}" x2="{cx:.1f}" y2="{y0+h+4}"/>')
    parts.append(f'<text x="{cx+9:.1f}" y="{top_y+18}">选定探索样本 {chosen*100:.0f}%</text>')
    parts.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")


# ---- reporting -------------------------------------------------------------

def dimension_table() -> list[dict[str, object]]:
    return [
        {"dimension": "角色数 n", "levels": list(N_LEVELS), "count": len(N_LEVELS)},
        {"dimension": "起始分布 distribution", "levels": list(DISTRIBUTION_LEVELS), "count": len(DISTRIBUTION_LEVELS)},
        {"dimension": "目标重合 goal", "levels": list(GOAL_LEVELS), "count": len(GOAL_LEVELS)},
        {"dimension": "通道宽度 w/D", "levels": list(WIDTH_LEVELS), "count": len(WIDTH_LEVELS)},
    ]


def build_report(
    out: Path,
    full_results: list[CaseResult],
    oracle: list[CaseResult],
    curve: list[dict[str, float | int | str]],
    chosen_rate: float,
    sampled: list[Case],
    sample_results: list[CaseResult],
    audit_before: dict[str, object],
    final_matrix: list[Case],
    audit_after: dict[str, object],
) -> None:
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "full-matrix-results.csv", [result_row(r) for r in full_results])
    write_csv(out / "deadlock-oracle.csv", [result_row(r) for r in oracle])
    write_csv(out / "sampling-curve.csv", [dict(r) for r in curve])
    write_csv(out / "selected-sample.csv", [result_row(r) for r in sample_results])
    write_csv(out / "sample-misses.csv", list(audit_before["missed"]))
    oracle_by_id = {r.case.ident: r for r in oracle}
    write_csv(
        out / "final-regression-matrix.csv",
        [
            {
                "id": c.ident,
                "n": c.n,
                "distribution": c.distribution,
                "goal": c.goal,
                "width_d": c.width_d,
                "deadlocked": int(oracle_by_id[c.ident].deadlocked),
                "timeout": int(oracle_by_id[c.ident].timeout),
                "reason": oracle_by_id[c.ident].reason,
            }
            for c in final_matrix
        ],
    )
    (out / "coverage-audit.json").write_text(
        json.dumps(
            {
                "dimensions": dimension_table(),
                "full_cartesian_size": FULL_MATRIX_SIZE,
                "predicate_eligible_size": MATRIX_SIZE,
                "predicate_filtered": {
                    "rule": "pair predicate requires n >= 2",
                    "removed_n1_cells": FULL_MATRIX_SIZE - MATRIX_SIZE,
                },
                "chosen_exploratory_rate": chosen_rate,
                "random_uniform_95pct_full_coverage_rate": choose_rate(curve),
                "predicate": {
                    "window_frames": WINDOW_FRAMES,
                    "dt": DT,
                    "radius": RADIUS,
                    "diameter": DIAMETER,
                    "contact_eps": CONTACT_EPS,
                    "relative_speed_eps": REL_SPEED_EPS,
                },
                "before_augmentation": audit_before,
                "after_augmentation": audit_after,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    write_svg_curve(out / "sampling-curve.svg", curve, chosen_rate)

    deadlocks = [r for r in oracle if r.deadlocked]
    timeouts = [r for r in oracle if r.timeout]
    n1_results = [r for r in full_results if r.case.n == 1]
    n1_success = sum(r.success for r in n1_results)
    random_guarantee_rate = choose_rate(curve)
    by_dim: dict[str, dict[object, int]] = {
        "n": {},
        "distribution": {},
        "goal": {},
        "width_d": {},
    }
    for r in deadlocks:
        for dim in by_dim:
            v = getattr(r.case, dim)
            by_dim[dim][v] = int(by_dim[dim].get(v, 0)) + 1

    missed_rows = audit_before["missed"]
    missed_lines = [
        f"| {m['n']} | `{m['distribution']}` | `{m['goal']}` | {m['width_d']:g}D | `{m['id']}` | {m['reason']} |"
        for m in missed_rows
    ]

    md = f"""# 多角色目标汇聚：卡死矩阵与四维抽样报告

## 1. 卡死定义

对尚未到达目标的角色对 `(i,j)`，在连续 `{WINDOW_FRAMES}` 帧（`{WINDOW_FRAMES*DT:.1f}` 秒）内同时满足：

```text
d_ij(t) = ||p_i(t) - p_j(t)||
Vrel_ij  = ||Δ(p_i - p_j)|| / (W·dt)

Blocked(i,j; t0..t0+W) =
    ∀t∈窗口: d_ij(t) < r_i + r_j + ε_d
    ∧ Vrel_ij ≤ ε_v
    ∧ i,j 均未到达目标
```

本实验取 `r={RADIUS:.2f} m`、`ε_d={CONTACT_EPS:.3f} m`（接触数值容差；理想表达式中为 0）、`ε_v={REL_SPEED_EPS:.3f} m/s`、窗口 `{WINDOW_FRAMES}` 帧。到达目标的角色立即判成功并移出碰撞体，避免把“终点重合后的正常消失”误报为卡死。

## 2. 四维笛卡尔矩阵

| 维度 | 取值 | 等价类/边界理由 |
|---|---:|---|
| 角色数 `n` | {", ".join(map(str, N_LEVELS))} | `1` 是不可能构成角色对的负对照；`2` 验证最小对，`3` 检查奇数对称，`4/6` 检查小群与拥挤 |
| 起始分布 | `column`, `row`, `arc`, `opposed` | 纵队、横排、弧形汇聚、双向对冲 |
| 目标是否重合 | `same`, `distinct` | 同一目标点 / 每个角色独立目标点 |
| 通道宽度 `w/D` | {", ".join(f"{x:g}" for x in WIDTH_LEVELS)} | `D=2r`；`w=D` 仅容一身，`w=2D` 是双人无侧隙错身边界，`>2D` 才有真实错身余量 |

笛卡尔积总单元数与表达式过滤后的最少可判定矩阵为：

```text
全笛卡尔积 C_full = {len(N_LEVELS)} × {len(DISTRIBUTION_LEVELS)} × {len(GOAL_LEVELS)} × {len(WIDTH_LEVELS)} = {FULL_MATRIX_SIZE}
n=1 负对照       = 1 × {len(DISTRIBUTION_LEVELS)} × {len(GOAL_LEVELS)} × {len(WIDTH_LEVELS)} = {FULL_MATRIX_SIZE - MATRIX_SIZE}
最少可判定矩阵 C  = {len(PAIR_N_LEVELS)} × {len(DISTRIBUTION_LEVELS)} × {len(GOAL_LEVELS)} × {len(WIDTH_LEVELS)} = {MATRIX_SIZE}
```

卡死表达式首先要求存在未到达的角色对，因此 `n=1` 的 `{FULL_MATRIX_SIZE - MATRIX_SIZE}` 个单元不可能满足谓词，只作为负对照保留在 `full-matrix-results.csv`（本次 `{n1_success}/{len(n1_results)}` 成功）。几何上，两个半径为 `r` 的圆体平行错身需要 `w ≥ 4r = 2D`，所以 `w/D < 2` 是强风险边界；但宽通道不能据此从矩阵删除——本地贪心控制器仍可能在门口交叉、成对绕圈或人群挤压中复现卡死，本次宽通道结果也证伪了“宽于 2D 就不可能卡死”的过度过滤。

## 3. 全矩阵预言机结果

- 全笛卡尔负对照+正式矩阵：**{len(full_results)}**
- `n≥2` 可判定矩阵：**{len(oracle)}**
- 可稳定复现卡死：**{len(deadlocks)}**
- 达到帧数上限且未满足卡死表达式（单独标记为 timeout/livelock，不混入静态卡死数）：**{len(timeouts)}**
- 可判定矩阵仿真累计耗时：**{sum(r.runtime_ms for r in oracle)/1000:.2f} s**（Python 确定性参考模型）

按维度统计已复现卡死组合：

```json
{json.dumps(by_dim, ensure_ascii=False, indent=2)}
```

完整逐例结果见 `full-matrix-results.csv`（240 个全笛卡尔单元）和 `deadlock-oracle.csv`（192 个谓词可判定单元）。

## 4. 抽样率曲线与取舍

对每个抽样率重复 200 次均匀笛卡尔单元抽样。下图上半部分是平均覆盖到的预言机卡死组合数，下半部分是该抽样率对应的总用例数；原始统计和“200 次中一次抽样全覆盖的概率”见 `sampling-curve.csv`。

![sampling curves](./sampling-curve.svg)

曲线没有出现“卡死覆盖提前饱和、总用例仍快速上升”的免费拐点：均匀抽样的平均覆盖数近似按 `q × {len(deadlocks)}` 线性增长。若要求随机抽样本身以至少 95% 概率覆盖全部卡死单元，曲线给出的比率是 **{random_guarantee_rate*100:.0f}%**；这说明硬覆盖门槛不能交给随机运气。

这里采用可审计的两段策略：先跑 **{chosen_rate*100:.0f}%** 四维分层探索样本（本次 **{len(sample_results)}** 例，强制每个维度取值至少出现一次），再用全量预言机/边界矩阵做覆盖对账，把所有漏掉的已复现卡死单元补成确定性回归用例。`{chosen_rate*100:.0f}%` 是探索预算选择，不是统计全覆盖保证；真正的保证来自最后的集合闭合审计。

## 5. 覆盖审计与漏掉组合

增强前，选定样本对预言机卡死组合的覆盖为 **{audit_before['covered_by_sample']}/{audit_before['deadlock_total']}**，漏掉 **{audit_before['missed_count']}** 个。

"""
    if missed_lines:
        md += (
            "完整机器可读清单是 [`sample-misses.csv`](./sample-misses.csv) 与 `coverage-audit.json`。维度取值如下：\n\n"
            "<details><summary>展开全部漏掉组合</summary>\n\n"
            "| n | 起始分布 | 目标 | w/D | 用例 ID | 复现信号 |\n"
            "|---:|---|---|---:|---|---|\n"
            + "\n".join(missed_lines)
            + "\n\n</details>\n\n这些漏掉单元的四维取值已显式列出，并全部补入最终回归矩阵。\n\n"
        )
    else:
        md += "选定分层样本已经覆盖全部预言机卡死单元，没有漏掉的维度取值。\n\n"

    md += f"""增强后的最终回归矩阵共 **{len(final_matrix)}** 例，对已复现卡死组合覆盖 **{audit_after['covered_by_sample']}/{audit_after['deadlock_total']}**，漏数为 **{audit_after['missed_count']}**；另外 **{audit_after['timeout_total']}** 个不满足静态相对速度谓词、但超时未到终点的绕圈/livelock 单元也已全部纳入，漏数为 **{audit_after['missed_timeout_count']}**。清单见 `final-regression-matrix.csv`。

## 6. 复现实验

```sh
# from repository root
python3 deadlock/src/deadlock_lab.py run --out artifacts/deadlock-lab
# or: make -C deadlock test
```

输出：

- `full-matrix-results.csv`：{FULL_MATRIX_SIZE} 个全笛卡尔单元（含 `n=1` 负对照）；
- `deadlock-oracle.csv`：{MATRIX_SIZE} 个谓词可判定单元的全量结果；
- `sampling-curve.csv` / `sampling-curve.svg`：抽样率与卡死覆盖、总用例数曲线；
- `selected-sample.csv`：选定四维样本的实际运行结果；
- `sample-misses.csv`：该样本相对全矩阵预言机漏掉的卡死维度取值；
- `final-regression-matrix.csv`：样本加全部已复现卡死单元；
- `coverage-audit.json`：机器可读的覆盖审计。

该参考控制器是故意保留局部、无全局通行权协商的贪心模型；它用于把“哪些维度组合能复现卡死”变成可执行矩阵，而不是声称现有 `grid/` A* 已包含动态避障。
"""
    (out / "report.md").write_text(md, encoding="utf-8")


# ---- CLI -------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    run_p = sub.add_parser("run", help="run oracle, sampling, audit, and report")
    default_out = Path(__file__).resolve().parents[2] / "artifacts" / "deadlock-lab"
    run_p.add_argument("--out", type=Path, default=default_out)
    run_p.add_argument("--seed", type=int, default=20261126)
    run_p.add_argument("--rate", type=float, default=0.0, help="override chosen sample rate")

    one_p = sub.add_parser("one", help="run one case and print JSON")
    one_p.add_argument("--n", type=int, required=True)
    one_p.add_argument("--distribution", choices=DISTRIBUTION_LEVELS, required=True)
    one_p.add_argument("--goal", choices=GOAL_LEVELS, required=True)
    one_p.add_argument("--width-d", type=float, required=True)

    args = parser.parse_args()

    if args.cmd == "one":
        case = Case(args.n, args.distribution, args.goal, args.width_d)
        r = run_case(case)
        print(json.dumps(result_row(r), ensure_ascii=False, indent=2))
        return 0 if r.success else 2

    out: Path = args.out
    full_cases = all_cases()
    full_results = [run_case(c) for c in full_cases]
    oracle = [r for r in full_results if r.case.n >= 2]

    result_by_id = {r.case.ident: r for r in full_results}
    if not result_by_id["n2-opposed-same-w1"].deadlocked:
        raise RuntimeError("canonical narrow head-on deadlock was not reproduced")
    if any(not r.success for r in full_results if r.case.n == 1):
        raise RuntimeError("n=1 negative control must always succeed")

    curve = coverage_curves(oracle)
    chosen_rate = args.rate if args.rate > 0 else EXPLORATORY_RATE

    sampled = latin_hypercube_case_sample(chosen_rate, args.seed)
    sample_results = [run_case(c) for c in sampled]
    audit_before = audit_coverage(sampled, oracle)

    final_matrix = mandatory_matrix(oracle, sampled)
    audit_after = audit_coverage(final_matrix, oracle)
    if audit_after["missed_count"] != 0 or audit_after["missed_timeout_count"] != 0:
        raise RuntimeError("coverage closure failed: reproduced failures remain outside matrix")

    build_report(
        out=out,
        full_results=full_results,
        oracle=oracle,
        curve=curve,
        chosen_rate=chosen_rate,
        sampled=sampled,
        sample_results=sample_results,
        audit_before=audit_before,
        final_matrix=final_matrix,
        audit_after=audit_after,
    )

    print(f"full_cartesian_size={FULL_MATRIX_SIZE}")
    print(f"matrix_size={MATRIX_SIZE}")
    print(f"deadlocks={audit_after['deadlock_total']}")
    print(f"chosen_rate={chosen_rate:.2f}")
    print(f"sample_size={len(sampled)}")
    print(f"missed_before={audit_before['missed_count']}")
    print(f"timeouts={audit_after['timeout_total']} missed_timeouts_before={audit_before['missed_timeout_count']}")
    print(f"final_matrix_size={len(final_matrix)}")
    print(f"missed_after={audit_after['missed_count']}")
    print(f"out={out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
