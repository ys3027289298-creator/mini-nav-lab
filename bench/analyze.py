#!/usr/bin/env python3
"""analyze.py: tell a straight-line leak from a bounded/log curve, recover the
per-round residual, and convert it into a leaked-entry count.

Companion to bench/navbench.c (real modules) and bench/control.c (known truth).

Method
------
For each run we fit four models to the memory-vs-round curve by least squares
and rank them by AICc:

    flat        M(n) = a
    linear      M(n) = a + b*n               <- constant residual each round
    log         M(n) = a + b*ln(n)           <- residual shrinking per round
    saturating  M(n) = a + b*(1-exp(-n/tau)) <- bounded store filling up

A straight line means every round leaves the same amount behind: the slope b
is the per-round residual in bytes. The slope of the live-block count is the
same residual in entries, with no size assumption; the two together give
bytes-per-entry, and bytes-per-entry is what turns a byte slope into an entry
count when the block counter is unavailable.

Round 0 is the pre-run baseline and is excluded from every fit: it measures
process cold-start, not per-round behaviour.

The 10% gate compares the first measured round against round `--eval-round`,
both read from the same run.

Usage:
  analyze.py --csv measurements/repo.csv --module grid
  analyze.py --csv measurements/controls.csv --module leak --entry-bytes 72
"""
import argparse
import csv
import math
import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit("numpy required")


# --------------------------------------------------------------------------
# data


SCALE = {"live_bytes": 1.0, "rss_kb": 1024.0}


def load(csv_path, module, bytes_col="live_bytes"):
    """Return {(run_id, rounds_planned): [(n, bytes, blocks, rss), ...]}
    with the chosen memory column normalised to bytes."""
    scale = SCALE.get(bytes_col, 1.0)
    runs = {}
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            if module and r.get("module") != module:
                continue
            try:
                n = int(r["round"])
                b = float(r[bytes_col]) * scale
                k = float(r["live_blocks"])
                rss = float(r["rss_kb"])
            except (KeyError, ValueError):
                continue
            key = (r.get("run", "0"), int(r.get("rounds_planned", 0) or 0))
            runs.setdefault(key, []).append((n, b, k, rss))
    out = {}
    for key, rows in runs.items():
        rows = [row for row in rows if row[0] > 0]  # drop cold-start baseline
        rows.sort()
        if rows:
            out[key] = rows
    if not out:
        sys.exit(f"no rows for module={module!r} in {csv_path}")
    return out


# --------------------------------------------------------------------------
# fits


def _sse(resid):
    return float(np.sum(resid * resid))


def _aicc(n, k, s):
    if n <= k + 1:
        return float("inf")
    return n * math.log(max(s, 1e-12) / n) + 2 * k + 2 * k * (k + 1) / (n - k - 1)


def fit_flat(n, y):
    a = float(np.mean(y))
    return {"name": "flat", "k": 1, "sse": _sse(y - a), "pred": np.full_like(y, a)}


def fit_linear(n, y):
    A = np.column_stack([np.ones_like(n), n])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    return {"name": "linear", "k": 2, "sse": _sse(y - pred), "pred": pred,
            "coef": coef}


def fit_log(n, y):
    A = np.column_stack([np.ones_like(n), np.log(n)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    pred = A @ coef
    return {"name": "log", "k": 2, "sse": _sse(y - pred), "pred": pred,
            "coef": coef}


def fit_saturating(n, y):
    best = None
    span = float(n.max() - n.min()) or 1.0
    for tau in np.geomspace(span / 100.0, span * 50.0, 600):
        z = 1.0 - np.exp(-(n - n.min()) / tau)
        A = np.column_stack([np.ones_like(n), z])
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        pred = A @ coef
        s = _sse(y - pred)
        if best is None or s < best[0]:
            best = (s, coef, tau, pred)
    s, coef, tau, pred = best
    return {"name": "saturating", "k": 3, "sse": s, "pred": pred,
            "coef": coef, "tau": tau}


def fit_all(n, y):
    fits = [fit_flat(n, y), fit_linear(n, y), fit_log(n, y), fit_saturating(n, y)]
    for f in fits:
        f["aicc"] = _aicc(len(n), f["k"], f["sse"])
    fits.sort(key=lambda f: f["aicc"])
    return fits


def slope_ci(n, y):
    """Linear slope with a 95% interval; the interval is what separates a real
    residual from noise when the series looks flat."""
    A = np.column_stack([np.ones_like(n), n])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ coef
    dof = len(n) - 2
    if dof <= 0:
        return float(coef[1]), float("nan")
    s2 = float(np.sum(resid ** 2)) / dof
    try:
        se = math.sqrt(max(s2 * np.linalg.inv(A.T @ A)[1, 1], 0.0))
    except np.linalg.LinAlgError:
        return float(coef[1]), float("nan")
    return float(coef[1]), 1.96 * se


def half_slopes(n, y):
    """Slope of the first and second half separately.

    A straight line has the same slope in both halves; any curve that bends
    over (log, saturating, partial reclaim) has a smaller late slope. This is
    a shape check that does not depend on choosing between nonlinear models,
    so it stays informative when AICc cannot separate them.
    """
    h = len(n) // 2
    if h < 2:
        return None
    early = slope_ci(n[:h], y[:h])
    late = slope_ci(n[h:], y[h:])
    return early[0], late[0], early[1], late[1]


# --------------------------------------------------------------------------
# interpretation


def classify(best, slope, half):
    if best["name"] == "flat":
        return "flat / no residual"
    if best["name"] == "linear":
        if not math.isnan(half) and slope - half <= 0:
            return "flat (slope not distinguishable from 0 at 95%)"
        return "linear (straight line: equal residual every round = real leak)"
    if best["name"] == "saturating":
        return "saturating (bounded store filling: reclaimed/capped, not a runaway leak)"
    if best["name"] == "log":
        return "logarithmic (residual shrinks per round: partial reclaim)"
    return "indeterminate"


def shape_verdict(fits, n, y):
    """Combine the model ranking with the early/late slope check.

    AICc alone can be a coin flip between `linear` and `saturating` over a
    short, noisy window: a straight line whose later half happens to fit a
    flat curve slightly better wins on AICc with no real support. The
    half-slope test settles it -- a genuine curve has a late slope clearly
    below its early slope, by more than the slope uncertainty allows for noise.
    """
    best = fits[0]
    slope, half = slope_ci(n, y)
    margin = fits[1]["aicc"] - best["aicc"] if len(fits) > 1 else float("inf")
    hs = half_slopes(n, y)
    early = late = None
    curved = False
    if hs:
        early, late, se_e, se_l = hs
        # curvature must be visible above the noise on the split halves
        drift = early - late
        drift_sig = math.hypot(se_e, se_l)
        if early > 0 and late < 0.5 * early and drift > 2.0 * drift_sig:
            curved = True

    no_residual = (best["name"] == "flat" or
                   (best["name"] == "linear" and not math.isnan(half)
                    and slope - half <= 0))
    if no_residual and not curved:
        return "flat / no residual", slope, half, early, late

    if curved or (best["name"] in ("log", "saturating") and margin >= 2.0):
        if curved and late <= max(0.1 * early, 2.0 * (hs[3] if hs else 0.0)):
            return ("saturating (bounded store filling: reclaimed/capped, "
                    "not a runaway leak)"), slope, half, early, late
        if best["name"] == "saturating":
            return ("saturating (bounded store filling: reclaimed/capped, "
                    "not a runaway leak)"), slope, half, early, late
        return ("logarithmic / decelerating (residual shrinks per round: "
                "partial reclaim)"), slope, half, early, late

    if best["name"] == "linear" and margin >= 2.0:
        return ("linear (straight line: equal residual every round = real leak)"), \
            slope, half, early, late
    if best["name"] == "linear" and not curved:
        # a straight line the half-slope test does not contradict; the AICc
        # runner-up is close but has no shape evidence behind it
        return ("linear (straight line, AICc margin thin but slope does not "
                "curve: equal residual every round = real leak)"), \
            slope, half, early, late
    # a nonlinear model won on AICc, but the half-slope test found no real
    # curvature -- the ranking is not trustworthy, and neither is a slope read
    return "straight-ish but ambiguous (extend the run)", slope, half, early, late


def fmt_int(x):
    return f"{x:.0f}" if abs(x - round(x)) < 1e-9 else f"{x:.1f}"


def analyze_run(rows, label, entry_bytes):
    n = np.array([r[0] for r in rows], dtype=float)
    y = np.array([r[1] for r in rows], dtype=float)
    k = np.array([r[2] for r in rows], dtype=float)
    rss = np.array([r[3] for r in rows], dtype=float)

    fits = fit_all(n, y)
    best = fits[0]
    verdict, slope, half, early, late = shape_verdict(fits, n, y)

    kfits = fit_all(n, k)
    kbest = kfits[0]
    kslope, khalf = slope_ci(n, k)

    print(f"run {label}: {len(rows)} points, rounds {int(n.min())}..{int(n.max())}")
    print("  model ranking (AICc, lower is better):")
    for f in fits:
        print(f"    {f['name']:<11} k={f['k']} SSE={f['sse']:>12.1f} AICc={f['aicc']:>8.2f}")
    margin = (fits[1]["aicc"] - best["aicc"]) if len(fits) > 1 else float("inf")
    print(f"  best model: {best['name']}  (runner-up {margin:.1f} AICc behind)")

    print(f"  bytes shape : {verdict}")
    if early is not None and late is not None:
        meaningful = max(abs(early), abs(late), 1e-9) > 1e-6
        label = "curving" if (meaningful and late < 0.5 * early) else "straight"
        print(f"  half-slopes : early {early:+.1f}, late {late:+.1f} B/round "
              f"({label})")
    print(f"  bytes slope : {slope:+.1f} +/- {half:.1f} B/round (95%)")
    print(f"  blocks shape: {classify(kbest, kslope, khalf)}")
    print(f"  blocks slope: {kslope:+.2f} +/- {khalf:.2f} entries/round (95%)")

    residual = None
    linear_shape = verdict.startswith("linear")
    if linear_shape and not math.isnan(half) and slope - half > 0:
        residual = slope
        per_entry = None
        if entry_bytes and entry_bytes > 0:
            per_entry = slope / entry_bytes
        elif not math.isnan(khalf) and abs(slope) > 0 and abs(kslope) > 1e-9:
            per_entry = entry_bytes = slope / kslope
            print(f"  implied entry size: {entry_bytes:.1f} B/entry (slope/block-slope)")
        if per_entry is not None:
            print(f"  residual   : {fmt_int(slope)} B/round = "
                  f"{fmt_int(per_entry)} path entries/round")
        else:
            print(f"  residual   : {fmt_int(slope)} B/round "
                  f"(pass --entry-bytes to convert to entries)")

    # 10% gate over the observed window, using this run's own endpoints
    base, last = y[0], y[-1]
    if base > 0:
        growth = (last - base) / base
        print(f"  growth r{int(n[0])}->r{int(n[-1])}: {growth * 100:+.2f}% "
              f"({last - base:+.0f} B)  gate=10%  "
              f"{'PASS' if growth <= 0.10 else 'FAIL'}")
        if residual is not None:
            proj = residual * (n[-1] - n[0]) / base * 100
            print(f"  slope-projected growth over that window: {proj:+.2f}%")
    print()
    return {"verdict": verdict, "best": best["name"], "slope": slope,
            "half": half, "kslope": kslope, "residual": residual}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--csv", required=True)
    ap.add_argument("--module")
    ap.add_argument("--metric", default="live_bytes",
                    help="memory column: live_bytes (heap, default) or rss_kb")
    ap.add_argument("--entry-bytes", type=float, default=None,
                    help="bytes per path entry; recovered from block slope if omitted")
    a = ap.parse_args()

    runs = load(a.csv, a.module, a.metric)
    results = []
    for (run_id, planned), rows in sorted(runs.items(), key=lambda kv: kv[0][1]):
        label = f"run={run_id} rounds={planned or '?'}"
        results.append(analyze_run(rows, label, a.entry_bytes))

    if len(results) > 1:
        names = [r["verdict"] for r in results]
        print("cross-run verdict:")
        if all(x.startswith("flat") for x in names):
            print("  all brackets flat -> no measurable residual at any round count")
        elif all(x.startswith("linear") for x in names):
            slopes = [r["slope"] for r in results]
            print(f"  all brackets linear -> leak confirmed; slopes "
                  f"{', '.join(f'{s:+.1f}' for s in slopes)} B/round")
        else:
            print(f"  brackets disagree ({', '.join(names)}) -> shape depends on "
                  f"how long the process runs; treat as saturating until a longer "
                  f"run confirms a straight line")


if __name__ == "__main__":
    main()
