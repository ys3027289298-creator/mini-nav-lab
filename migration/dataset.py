"""Deterministic path-record datasets at three sizes.

Records are produced by a Python port of grid/src/grid.c (4-neighbour A* on a
16x12 map) so the payloads obey the same invariants as the C module: indices
are in-range, path_len <= 48, paths are contiguous and start/end at the
requested cells.  A configurable minority is deliberately made non-canonical
(padding garbage, stray flags, over-long path_len, out-of-grid indices) so the
"incremental snapshot silently loses data" failure mode actually occurs.
"""

from __future__ import annotations

import random

from . import schema

SIZES = {"small": 5_000, "medium": 50_000, "large": 500_000}
SEED = 20261126
MIGRATION_EPOCH_US = 1_800_000_000_000_000

# Fractions of records given each kind of non-canonical damage.
DAMAGE_PADDING = 0.06
DAMAGE_FLAGS = 0.02
DAMAGE_PATH_LEN = 0.01
DAMAGE_RANGE = 0.01


def _astar(blocked: bytearray, sx: int, sy: int, gx: int, gy: int) -> list[int] | None:
    W, H = schema.W, schema.H

    def walk(x: int, y: int) -> bool:
        return 0 <= x < W and 0 <= y < H and not blocked[y * W + x]

    if not walk(sx, sy) or not walk(gx, gy):
        return None
    start, goal = sy * W + sx, gy * W + gx
    INF = 1 << 29
    g = [INF] * (W * H)
    f = [INF] * (W * H)
    closed = [False] * (W * H)
    parent = [-1] * (W * H)
    g[start] = 0
    f[start] = abs(gx - sx) + abs(gy - sy)
    open_ = [start]
    while open_:
        best = 0
        for i in range(1, len(open_)):
            if f[open_[i]] < f[open_[best]]:
                best = i
        cur = open_[best]
        # Mirror grid.c exactly: move the last entry into the hole rather than
        # shifting, because that changes which of several equal-f nodes is
        # expanded next, and therefore which optimal path is returned.
        open_[best] = open_[-1]
        open_.pop()
        if cur == goal:
            out, c = [], cur
            while c != -1:
                out.append(c)
                c = parent[c]
            return out[::-1]
        closed[cur] = True
        cx, cy = cur % W, cur // W
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if not walk(nx, ny):
                continue
            ni = ny * W + nx
            if closed[ni]:
                continue
            ng = g[cur] + 1
            if ng < g[ni]:
                g[ni] = ng
                f[ni] = ng + abs(gx - nx) + abs(gy - ny)
                parent[ni] = cur
                if ni not in open_:
                    open_.append(ni)
    return None


def _make_record(rng: random.Random, index: int) -> schema.OldRecord:
    W, H = schema.W, schema.H
    while True:
        blocked = bytearray(W * H)
        density = rng.choice((0.0, 0.08, 0.15, 0.25))
        for i in range(W * H):
            if rng.random() < density:
                blocked[i] = 1
        sx, sy = rng.randrange(W), rng.randrange(H)
        gx, gy = rng.randrange(W), rng.randrange(H)
        path = _astar(blocked, sx, sy, gx, gy)
        if path is None or len(path) > schema.MAX_POINTS:
            continue
        break

    rec = schema.OldRecord(
        flags=0,
        path_len=len(path),
        path=list(path) + [0] * (schema.MAX_POINTS - len(path)),
        start_idx=sy * W + sx,
        goal_idx=gy * W + gx,
        cost=len(path) - 1,
        created_at=MIGRATION_EPOCH_US - 86_400_000_000 + index * 1_000_003,
    )
    damage = rng.random()
    if damage < DAMAGE_PADDING:
        # Garbage in a slot beyond path_len: invisible to the new layout.
        slot = rng.randrange(rec.path_len, schema.MAX_POINTS)
        rec.path[slot] = rng.randrange(1, 1 << 32)
    elif damage < DAMAGE_PADDING + DAMAGE_FLAGS:
        rec.flags |= schema.FLAG_COORD
    elif damage < DAMAGE_PADDING + DAMAGE_FLAGS + DAMAGE_PATH_LEN:
        rec.path_len = rng.randrange(schema.MAX_POINTS + 1, 1 << 20)
    elif damage < DAMAGE_PADDING + DAMAGE_FLAGS + DAMAGE_PATH_LEN + DAMAGE_RANGE:
        which = rng.choice(("path", "start", "goal"))
        if which == "path":
            rec.path[rng.randrange(rec.path_len)] = rng.randrange(schema.CELLS, 1 << 32)
        elif which == "start":
            rec.start_idx = rng.randrange(schema.CELLS, 1 << 32)
        else:
            rec.goal_idx = rng.randrange(schema.CELLS, 1 << 32)
    return rec


def generate(size: str, limit: int | None = None) -> list[schema.OldRecord]:
    """Records are generated on demand; ``limit`` truncates for smoke tests."""
    n = SIZES[size]
    if limit is not None:
        n = min(n, limit)
    rng = random.Random(f"{SEED}:{size}")
    return [_make_record(rng, i) for i in range(n)]


def iter_records(size: str):
    """Streaming variant so the 500k tier does not hold every record twice."""
    n = SIZES[size]
    rng = random.Random(f"{SEED}:{size}")
    for i in range(n):
        yield _make_record(rng, i)
