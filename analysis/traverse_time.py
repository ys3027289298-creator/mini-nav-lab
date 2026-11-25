#!/usr/bin/env python3
"""Traverse-time estimation experiment on mini-nav-lab style grids.

Three estimators of traversal time vs path length, on three terrain tiers:
  level 0  T_dist  = L / v0                      (distance only, linear)
  level 1  T_speed = sum_i l_i / v(terrain_i)    (piecewise by terrain)
  level 2  T_corr  = T_speed + n_turns*c_turn + c_ss  (corner-corrected)
Measured ground truth adds per-turn deceleration and start/stop overhead
plus small measurement noise.

Outputs:
  artifacts/traverse_time_curves.png  3 tiers x 3 curves, max-deviation marked
  artifacts/precision_tradeoff.png    error% and compute time vs precision level
  artifacts/traverse_report.md        numbers: divergence, equivalent corner
                                      cost, 5% check, worst path if violated
"""
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

W, H = 16, 12
CELL = 1.0  # m per cell
V0 = 3.0    # nominal speed for distance-only estimate (m/s)
TAU_TURN = 0.35  # s, true corner deceleration cost per turn (ground truth)
TAU_SS = 0.8     # s, true start/stop overhead (ground truth)
NOISE = 0.015    # measurement noise per repetition, fraction of measured time
MEAS_REPS = 5    # measured time = mean over this many repetitions
ERR_LIMIT = 5.0  # %, required accuracy vs measured time

SPEEDS = {"road": 3.0, "grass": 1.5, "sand": 0.6}

# terrain tiers: column-band widths (road, grass, sand) across W=16
TIERS = {
    "A road-dominant": (11, 3, 2),
    "B mixed": (6, 6, 4),
    "C sand-dominant": (2, 4, 10),
}

OUT = Path(__file__).resolve().parent.parent / "artifacts"


def idx(x, y):
    return y * W + x


def build_map(bands):
    """Terrain field from column bands + fixed walls with staggered gaps."""
    terr = np.empty(W * H, dtype=object)
    wr, wg, ws = bands
    for y in range(H):
        for x in range(W):
            t = "road" if x < wr else ("grass" if x < wr + wg else "sand")
            terr[idx(x, y)] = t
    blocked = np.zeros(W * H, dtype=bool)
    # vertical walls with one gap each, staggered to force zigzag turns
    for wx, gap in ((4, 2), (8, H - 3), (12, 2)):
        for y in range(1, H - 1):
            if y != gap:
                blocked[idx(wx, y)] = True
    return terr, blocked


def astar(blocked, sx, sy, gx, gy):
    """4-neighbour A*, unit step cost. Returns list of cells or None."""
    if blocked[idx(sx, sy)] or blocked[idx(gx, gy)]:
        return None
    g = {idx(sx, sy): 0}
    parent = {idx(sx, sy): -1}
    open_set = [idx(sx, sy)]
    closed = set()
    while open_set:
        cur = min(open_set, key=lambda c: g[c] + abs(gx - c % W) + abs(gy - c // W))
        open_set.remove(cur)
        if cur == idx(gx, gy):
            path = []
            while cur != -1:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        cx, cy = cur % W, cur // W
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            ni = idx(nx, ny)
            if blocked[ni] or ni in closed:
                continue
            ng = g[cur] + 1
            if ng < g.get(ni, 1 << 29):
                g[ni] = ng
                parent[ni] = cur
                if ni not in open_set:
                    open_set.append(ni)
    return None


def count_turns(path):
    xy = [(c % W, c // W) for c in path]
    dirs = [(xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1])
            for i in range(len(xy) - 1)]
    return sum(1 for i in range(1, len(dirs)) if dirs[i] != dirs[i - 1])


def t_speed(path, terr):
    return sum(CELL / SPEEDS[terr[c]] for c in path[1:])


def gen_paths(terr, blocked, n_pairs=400, seed=7):
    rng = np.random.default_rng(seed)
    free = [i for i in range(W * H) if not blocked[i]]
    paths, seen = [], set()
    for _ in range(n_pairs):
        s, gcell = rng.choice(free, size=2, replace=False)
        key = (s, gcell)
        if key in seen:
            continue
        seen.add(key)
        p = astar(blocked, s % W, s // W, gcell % W, gcell // W)
        if p and len(p) >= 3:
            paths.append(p)
    paths.sort(key=len)
    return paths


def measure(paths, terr, rng):
    """Ground truth: speed term + turn decel + start/stop, mean of noisy reps."""
    out = []
    for p in paths:
        nt = count_turns(p)
        t = t_speed(p, terr) + nt * TAU_TURN + TAU_SS
        t *= 1.0 + rng.normal(0.0, NOISE, size=MEAS_REPS).mean()
        out.append((p, nt, t))
    return out


def fit_corner_cost(turns, residual, t_meas):
    """Weighted least squares: residual = n_turns*c_turn + c_ss.

    Measurement noise is proportional to t_meas, so fit in relative
    (per-second-of-measured-time) space to keep short slow paths from
    being dominated by long ones.
    """
    A = np.vstack([turns / t_meas, 1.0 / t_meas]).T
    b = residual / t_meas
    (c_turn, c_ss), *_ = np.linalg.lstsq(A, b, rcond=None)
    return c_turn, c_ss


def bench_estimators(paths, terr, c_turn, c_ss, reps=300):
    """Mean per-path compute time (us) for the three precision levels."""
    ts = []
    for level in (0, 1, 2):
        t0 = time.perf_counter()
        for _ in range(reps):
            for p in paths:
                if level == 0:
                    _ = (len(p) - 1) * CELL / V0
                elif level == 1:
                    _ = t_speed(p, terr)
                else:
                    _ = t_speed(p, terr) + count_turns(p) * c_turn + c_ss
        dt = (time.perf_counter() - t0) / reps / len(paths)
        ts.append(dt * 1e6)
    return ts


def run_tier(name, bands, rng):
    terr, blocked = build_map(bands)
    paths = gen_paths(terr, blocked)
    samples = measure(paths, terr, rng)

    L = np.array([(len(p) - 1) * CELL for p, _, _ in samples])
    nt = np.array([n for _, n, _ in samples], dtype=float)
    t_meas = np.array([t for _, _, t in samples])
    t_dist = L / V0
    t_sp = np.array([t_speed(p, terr) for p, _, _ in samples])

    # divergence: max gap between measured and speed estimate
    dev = t_meas - t_sp
    k = int(np.argmax(dev))
    c_eq = dev[k] / nt[k] if nt[k] > 0 else float("nan")
    # onset: first path where relative deviation exceeds 2%
    onset = next((i for i in np.argsort(L)
                  if dev[i] / t_sp[i] > 0.02), None)

    # level-2 corrected estimate from fitted equivalent corner cost
    c_turn, c_ss = fit_corner_cost(nt, dev, t_meas)
    t_corr = t_sp + nt * c_turn + c_ss

    ests = [t_dist, t_sp, t_corr]
    err = [np.abs(e - t_meas) / t_meas * 100.0 for e in ests]
    max_err = [float(e.max()) for e in err]
    worst = [int(e.argmax()) for e in err]
    compute_us = bench_estimators(paths, terr, c_turn, c_ss)

    return dict(name=name, L=L, nt=nt, t_meas=t_meas, t_dist=t_dist,
                t_sp=t_sp, t_corr=t_corr, dev=dev, k=k, c_eq=c_eq,
                onset=onset, c_turn=c_turn, c_ss=c_ss, err=err,
                max_err=max_err, worst=worst, compute_us=compute_us,
                samples=samples)


def plot_curves(results, path):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), sharey=False)
    for ax, r in zip(axes, results):
        L = r["L"]
        # raw samples as light scatter, binned means as the three lines
        ax.scatter(L, r["t_meas"], s=6, color="#d62728", alpha=0.15)
        bins = np.arange(L.min(), L.max() + 1)
        bi = np.clip(np.digitize(L, bins) - 1, 0, len(bins) - 1)

        def binned(y):
            cnt = np.bincount(bi, minlength=len(bins))
            s = np.bincount(bi, weights=y, minlength=len(bins))
            out = np.where(cnt > 0, s / np.maximum(cnt, 1), np.nan)
            return bins, out

        for y, style, color, lab in (
                (r["t_dist"], "--", "#888", "dist-only  T=L/v0"),
                (r["t_sp"], "o-", "#1f77b4", "speed est  T=sum(li/vi)"),
                (r["t_meas"], "s-", "#d62728",
                 "measured  +turns+start/stop")):
            bx, by = binned(y)
            ax.plot(bx, by, style, ms=4, color=color, label=lab)
        k = r["k"]
        ax.annotate(
            f"max dev\nL={r['L'][k]:.0f} m\n"
            f"T_meas={r['t_meas'][k]:.2f} s\n"
            f"dev={r['dev'][k]:.2f} s\nc_eq={r['c_eq']:.3f} s/turn",
            xy=(r["L"][k], r["t_meas"][k]),
            xytext=(0.03, 0.97), textcoords="axes fraction",
            va="top", fontsize=8,
            arrowprops=dict(arrowstyle="->", color="black"))
        ax.plot([r["L"][k]], [r["t_meas"][k]], "*", ms=14, color="black")
        ax.set_title(r["name"])
        ax.set_xlabel("path length L (m)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("traversal time (s)")
    axes[0].legend(fontsize=8, loc="lower right")
    fig.suptitle("Traversal time vs path length - three estimators, three terrain tiers")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_tradeoff(results, path):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    labels = ["L0 dist-only", "L1 dist/speed", "L2 +corner"]
    for ax, r in zip(axes, results):
        x = [0, 1, 2]
        ax.plot(x, r["max_err"], "o-", color="#d62728", label="max error (%)")
        ax.axhline(ERR_LIMIT, color="#d62728", ls=":", lw=1)
        ax.text(2.05, ERR_LIMIT, "5%", color="#d62728", va="center", fontsize=8)
        ax.set_ylabel("max estimation error (%)", color="#d62728")
        ax.set_ylim(bottom=0)
        ax2 = ax.twinx()
        ax2.plot(x, r["compute_us"], "s--", color="#1f77b4",
                 label="compute time (us/path)")
        ax2.set_ylabel("compute time (us/path)", color="#1f77b4")
        ax.set_xticks(x, labels)
        ax.set_title(r["name"])
        ax.grid(alpha=0.3)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="center left")
    fig.suptitle("Precision vs cost: pick the cheapest level under the 5% bar")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    rng = np.random.default_rng(42)
    results = [run_tier(name, bands, rng) for name, bands in TIERS.items()]

    OUT.mkdir(exist_ok=True)
    plot_curves(results, OUT / "traverse_time_curves.png")
    plot_tradeoff(results, OUT / "precision_tradeoff.png")

    lines = ["# Traverse-time estimation report", ""]
    for r in results:
        k = r["k"]
        p, nturns, t_meas = r["samples"][k]
        s, g = p[0], p[-1]
        lines += [
            f"## {r['name']}",
            "",
            f"- paths sampled: {len(r['samples'])}, "
            f"L in [{r['L'].min():.0f}, {r['L'].max():.0f}] m",
            f"- divergence onset (dev>2% of speed est): "
            + (f"L={r['L'][r['onset']]:.0f} m" if r["onset"] is not None else "n/a"),
            f"- max deviation at L={r['L'][k]:.0f} m: "
            f"T_meas={t_meas:.3f} s, T_speed={r['t_sp'][k]:.3f} s, "
            f"dev={r['dev'][k]:.3f} s, turns={nturns}",
            f"- equivalent corner cost at that point: "
            f"c_eq = dev/turns = {r['c_eq']:.4f} s/turn",
            f"- fitted model: c_turn={r['c_turn']:.4f} s/turn, "
            f"c_ss={r['c_ss']:.4f} s (start/stop)",
            f"- max error by level: L0={r['max_err'][0]:.1f}%, "
            f"L1={r['max_err'][1]:.1f}%, L2={r['max_err'][2]:.2f}%",
            f"- compute by level: "
            + ", ".join(f"L{i}={t:.2f} us/path"
                        for i, t in enumerate(r["compute_us"])),
        ]
        w = r["worst"][2]
        wp, wn, wt = r["samples"][w]
        ok = r["max_err"][2] <= ERR_LIMIT
        lines.append(
            f"- 5% check on L2 corrected estimate: "
            f"{'PASS' if ok else 'FAIL'} (max {r['max_err'][2]:.2f}%)")
        if not ok:
            lines.append(
                f"  - worst path: ({wp[0]%W},{wp[0]//W})->({wp[-1]%W},{wp[-1]//W}), "
                f"L={(len(wp)-1)*CELL:.0f} m, turns={wn}, T_meas={wt:.3f} s, "
                f"err={r['err'][2][w]:.2f}%")
        lines.append("")
    report = "\n".join(lines)
    (OUT / "traverse_report.md").write_text(report)
    print(report)


if __name__ == "__main__":
    main()
