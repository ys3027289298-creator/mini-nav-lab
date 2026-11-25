#!/usr/bin/env python3
"""Numerical prototype for LOS / string-pull smoothing analysis.

Reconstructs the logged path (len=14, turns=13, cost=14, (0,0)->(12,7))
on the 16x12 lab grid, calibrates a wall layout that hugs every bend,
and validates: geometric LOS, radius-aware LOS, segment-to-wall distance
(two independent estimators), greedy vs DP minimum-point smoothing, and
the smoothing-strength sweep.

Run: python3 -I proto.py
"""
import math

W, H = 16, 12

# ---------------------------------------------------------------- path
# 14 king moves, alternating axis every step => 13 turns, |dx|+|dy| cost
# counted per move (=14). 7 east, 2 south, 5 southeast: 12 over, 7 down.
# 14 king moves, alternating axis every step => 13 turns, cost counted per
# move (=14). E=7, SE=5, S=2: dx=7+5=12, dy=5+2=7 -> goal (12,7).
# E must take every odd slot (7 E's is the max); D/S fill the even slots.
MOVES = [
    (1, 0), (1, 1), (1, 0), (1, 1), (1, 0), (0, 1),
    (1, 0), (1, 1), (1, 0), (1, 1), (1, 0), (0, 1),
    (1, 0), (1, 1),
]
PATH = [(0, 0)]
for dx, dy in MOVES:
    PATH.append((PATH[-1][0] + dx, PATH[-1][1] + dy))

assert len(PATH) == 15 and PATH[-1] == (12, 7)
turns = sum(
    1 for i in range(1, len(MOVES))
    if MOVES[i] != MOVES[i - 1]
)
assert turns == 13


def build_walls():
    """Two pillars that the (0,0)->(12,7) center chord passes through.

    Placement rules (calibrated numerically, not arbitrary):
    * never on a path cell (all 15 path cells stay walkable);
    * both cells sit exactly on the center chord (distance 0), so the
      r=0 two-point collapse is corner-hugging (worst clearance 0);
    * each keeps >= 0.5 cell distance from every one of the 14 raw
      segments, so the raw path is feasible for any radius up to 0.5.
    The pair splits the chord into thirds, which gives the smoothing
    sweep a genuine staircase: 2 -> 3 -> 4 points as the radius grows.
    """
    return {(4, 3), (9, 6)}


WALLS = build_walls()


# ------------------------------------------------- geometry primitives
# Cells are unit squares [x, x+1] x [y, y+1], node = cell center (x+0.5,...)
def seg_point_dist(p0, p1, q):
    """Exact min distance from point q to segment p0-p1."""
    vx, vy = p1[0] - p0[0], p1[1] - p0[1]
    wx, wy = q[0] - p0[0], q[1] - p0[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    cx, cy = p0[0] + t * vx, p0[1] + t * vy
    return math.hypot(q[0] - cx, q[1] - cy)


def seg_aabb_dist(p0, p1, wall):
    """Exact min distance from segment to wall cell square (AABB)."""
    x, y = wall
    # closest point on the box to each segment point = clamp; the min over
    # the segment of distance-to-convex-set is attained at the clamped
    # projection of a segment endpoint or where perpendicular projection
    # lands inside the box. Check: endpoints clamped + interior projection.
    def clamp_pt(q):
        return (min(max(q[0], x), x + 1), min(max(q[1], y), y + 1))

    best = min(seg_point_dist(p0, p1, clamp_pt(p0)),
               seg_point_dist(p0, p1, clamp_pt(p1)))
    vx, vy = p1[0] - p0[0], p1[1] - p0[1]
    L2 = vx * vx + vy * vy
    if L2 > 0:
        # candidate: projection of the box's closest corner onto the line
        for cx in (x, x + 1):
            for cy in (y, y + 1):
                t = ((cx - p0[0]) * vx + (cy - p0[1]) * vy) / L2
                if 0 < t < 1:
                    q = (p0[0] + t * vx, p0[1] + t * vy)
                    best = min(best, seg_point_dist(p0, p1, clamp_pt(q)))
    return best


def clearance(p0, p1, walls):
    """Min segment-to-wall distance over every wall cell."""
    return min(seg_aabb_dist(p0, p1, w) for w in walls)


def clearance_sampled(p0, p1, walls, steps=4096):
    """Independent estimator: dense-sample the segment, distance to each
    wall square via point-AABB distance. Overestimates by <= sample gap."""
    def pt_aabb(q, w):
        x, y = w
        dx = max(x - q[0], 0, q[0] - (x + 1))
        dy = max(y - q[1], 0, q[1] - (y + 1))
        return math.hypot(dx, dy)
    worst = float("inf")
    for i in range(steps + 1):
        t = i / steps
        q = (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
        worst = min(worst, min(pt_aabb(q, w) for w in walls))
    return worst


def center(p):
    return (p[0] + 0.5, p[1] + 0.5)


EPS = 1e-9


def los_geom(i, j, radius=0.0):
    """Geometric LOS with character radius.

    The radius-r swept tube intersects a blocked square iff
    clearance <= r, so LOS is the STRICT inequality clearance > r
    (contact counts as blocked). At r=0 a center line crossing a wall
    (clearance 0) is therefore rejected, which is exactly the case the
    old Bresenham test missed.
    """
    p0, p1 = center(PATH[i]), center(PATH[j])
    return clearance(p0, p1, WALLS) - radius > EPS


def los_bresenham(i, j):
    """Reference: the lab's existing Bresenham center-cell LOS."""
    x0, y0 = PATH[i]
    x1, y1 = PATH[j]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if (x, y) in WALLS:
            return False
        if x == x1 and y == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


# -------------------------------------------------------- smoothing DP
def min_points(radius):
    """DP: minimum number of retained vertices (subsequence of PATH).

    Returns (None, []) when the radius is infeasible (the goal is
    unreachable: no incoming edge has clearance > radius).
    """
    n = len(PATH)
    INF = 10 ** 9
    dp = [INF] * n
    prev = [-1] * n
    dp[0] = 1
    for j in range(1, n):
        for i in range(j):
            if los_geom(i, j, radius) and dp[i] + 1 < dp[j]:
                dp[j] = dp[i] + 1
                prev[j] = i
    if dp[n - 1] >= INF:
        return None, []
    chain, k = [], n - 1
    while k != -1:
        chain.append(k)
        k = prev[k]
    return dp[n - 1], list(reversed(chain))


def greedy(radius):
    """Lab-style greedy string pull with the geometric radius LOS."""
    n = len(PATH)
    kept = [0]
    i = 0
    while i < n - 1:
        j = n - 1
        while j > i + 1 and not los_geom(i, j, radius):
            j -= 1
        kept.append(j)
        i = j
    return kept


def polyline_len(chain):
    return sum(
        math.hypot(PATH[b][0] - PATH[a][0], PATH[b][1] - PATH[a][1])
        for a, b in zip(chain, chain[1:])
    )


def worst_clearance(chain):
    if len(chain) < 2:
        return float("inf")
    return min(
        clearance(center(PATH[a]), center(PATH[b]), WALLS)
        for a, b in zip(chain, chain[1:])
    )


def render_map(chain):
    print("  " + "".join(str(x % 10) for x in range(W)))
    for y in range(H):
        row = []
        for x in range(W):
            if (x, y) in WALLS:
                row.append("#")
            elif (x, y) == PATH[0]:
                row.append("S")
            elif (x, y) == PATH[-1]:
                row.append("G")
            elif (x, y) in [PATH[i] for i in chain]:
                row.append("o")
            elif (x, y) in PATH:
                row.append(".")
            else:
                row.append(" ")
        print(f"{y % 10} " + "".join(row))


def main():
    print("== path ==")
    print("nodes:", len(PATH), "moves:", len(MOVES),
          "turns:", turns, "cost:", len(MOVES))
    print("walls:", sorted(WALLS))
    assert not (set(PATH) & WALLS)

    # feasibility envelope of the raw staircase path
    raw_min = min(
        clearance(center(PATH[s]), center(PATH[s + 1]), WALLS)
        for s in range(14)
    )
    print("min clearance of the 14 raw segments:", round(raw_min, 4),
          "cell -> feasible radius up to", round(raw_min, 4))

    print("\n== per-segment checks, original 14 segments ==")
    _, chosen_chain = min_points(0.35)
    chosen_edges = {j - 1 for j in chosen_chain if j > 0}
    print(" seg  from->to        move      LOS0  LOS(.35)  clearance  sampled  status")
    for s in range(14):
        a, b = s, s + 1
        d = clearance(center(PATH[a]), center(PATH[b]), WALLS)
        ds = clearance_sampled(center(PATH[a]), center(PATH[b]), WALLS)
        status = "kept" if s in chosen_edges else "skip->chord"
        print(f" {s:3d}  {PATH[a]}->{PATH[b]} {str(MOVES[s]):9s} "
              f"{int(los_geom(a,b)):5d} {int(los_geom(a,b,.35)):8d} "
              f"{d:10.4f} {ds:8.4f}  {status}")

    print("\n== jump table (forward LOS from each vertex) ==")
    for i in range(len(PATH)):
        reach0 = [j for j in range(i + 2, len(PATH)) if los_geom(i, j, 0.0)]
        reach_r = [j for j in range(i + 2, len(PATH)) if los_geom(i, j, 0.35)]
        print(f" v{i:2d} {str(PATH[i]):9s} r=0   -> {reach0}")
        print(f" {'':14s} r=.35 -> {reach_r}")

    print("\n== smoothing sweep (DP minimum points) ==")
    print(" radius  points  poly_len  worst_clear  chain")
    rows = []
    r = 0.0
    while r <= 0.5001:
        k, chain = min_points(r)
        if k is None:
            print(f" {r:5.2f}   INFEASIBLE (goal unreachable at this radius)")
            rows.append((r, None, None, None))
        else:
            g = greedy(r)
            L = polyline_len(chain)
            c = worst_clearance(chain)
            rows.append((r, k, L, c))
            mark = " greedy-ok" if len(g) == k else " GREEDY!=DP"
            print(f" {r:5.2f}  {k:6d}  {L:8.4f}  {c:10.4f}  {chain}{mark}")
        r += 0.05

    with open("sweep.csv", "w") as f:
        f.write("radius,points,poly_len,worst_clear\n")
        for rr, kk, ll, cc in rows:
            f.write(f"{rr:.3f},{'' if kk is None else kk},"
                    f"{'' if ll is None else round(ll, 6)},"
                    f"{'' if cc is None else round(cc, 6)}\n")

    for rr, tag in [(0.0, "r=0 (aggressive, corner-hugging)"),
                    (0.35, "r=0.35 (chosen strength)")]:
        k, chain = min_points(rr)
        print(f"\n== acceptance @ {tag} ==")
        if k is None:
            print("INFEASIBLE at this radius; skipping")
            continue
        render_map(chain)
        print("kept vertex indices:", chain)
        print("kept cells:", [PATH[i] for i in chain])
        print("points:", k, " raw nodes:", len(PATH),
              " ratio:", round(k / len(PATH), 3),
              " < 1/3:", k < len(PATH) / 3)
        # every smoothed segment must pass LOS at this radius; list failures
        fails = [s for s in range(len(chain) - 1)
                 if not los_geom(chain[s], chain[s + 1], rr)]
        print("failing smoothed-segment indices:", fails or "none")
        # original segments absorbed into a longer chord are skipped;
        # a retained vertex at index j keeps the edge j-1->j alive.
        kept_edges = {j - 1 for j in chain if j > 0}
        skipped = [s for s in range(14) if s not in kept_edges]
        print("skipped original-segment indices:", skipped,
              f"({len(skipped)}/14 removed)")
        cw = worst_clearance(chain)
        print("worst clearance over kept segments:", round(cw, 4), "cell",
              " > radius:", cw - rr > EPS)
        # cross-validation: analytic distance vs dense sampling
        err = 0.0
        for a, b in zip(chain, chain[1:]):
            d1 = clearance(center(PATH[a]), center(PATH[b]), WALLS)
            d2 = clearance_sampled(center(PATH[a]), center(PATH[b]), WALLS)
            err = max(err, abs(d1 - d2))
        print("max analytic-vs-sampled distance error:", round(err, 5),
              "cell (budget 0.5)")

    print("\n== Bresenham LOS comparison on long chords ==")
    for i, j in [(0, 14), (0, 9), (3, 14), (0, 5), (5, 14)]:
        print(f" {PATH[i]}->{PATH[j]} bresenham={int(los_bresenham(i,j))} "
              f"geom_r0={int(los_geom(i,j,0.0))} "
              f"clear={clearance(center(PATH[i]),center(PATH[j]),WALLS):.4f}")


if __name__ == "__main__":
    main()
