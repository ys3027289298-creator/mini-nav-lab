#!/usr/bin/env python3
"""Analyse bench_raw.csv (two map families x three open-list structures).

Knee methodology
----------------
The two competing costs inside one array-open-list search are measured
directly per query:
  expansion cost  ce * exp     (linear in explored nodes, hence in area)
  scan cost       cs * scan    (linear scans over the open set)
The knee is the area where the two are equal; it is located two independent
ways:
  1. pinned piecewise fit of T vs A on log-log (slopes 1 and 1.5 for 2D
     embedded grids, whose open set is a perimeter O(sqrt A); 1 and 2 for
     abstract graphs with a space-filling frontier O(A));
  2. the measured ce, cs constants and per-row exp/scan counters: the rows
     bracket the area where cs*scan / (ce*exp) crosses 1.
The capacity formula c* = k^2*alpha*A*/4 is then evaluated against measured
peak open size to show which geometric regime the formula's O(A) assumption
matches and where it breaks.
"""
import csv
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RAW = os.path.join(ROOT, "bench_raw.csv")
RUN_GLOB = "bench_run_*.csv"

C_ARRAY, C_BITMAP, C_HEAP = "#eb6834", "#1baf7a", "#2a78d6"
INK, INK2, MUTED, GRID, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"

# tier side/area use the actual swept grid sizes (odd sides)
TIERS = [("small", 17, 289), ("medium", 65, 4225), ("large", 257, 66049)]


def _read_one(path):
    out = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            for key in ("side", "area", "reps", "exp", "ins", "scan",
                        "omax", "pathlen"):
                r[key] = int(r[key])
            for key in ("ns_median", "ns_min", "k", "alpha"):
                r[key] = float(r[key])
            out[(r["family"], r["variant"], r["area"])] = r
    return out


def load():
    """Aggregate every bench_run_*.csv. Search counters are deterministic
    (same map + same tie rules), so they come from the first run; the timing
    columns are pooled across independent runs and replaced by their median,
    which tames microsecond-level scheduler/cache noise on small maps."""
    import glob
    paths = sorted(glob.glob(os.path.join(ROOT, RUN_GLOB)))
    if not paths and os.path.exists(RAW):
        paths = [RAW]
    if not paths:
        raise SystemExit("no benchmark csv: run 'make run' or 'make runs' first")
    runs = [_read_one(p) for p in paths]
    keys = runs[0].keys()
    data = {}
    for key in keys:
        base = dict(runs[0][key])
        med = np.median([r[key]["ns_median"] for r in runs])
        mn = np.median([r[key]["ns_min"] for r in runs])
        base["ns_median"] = float(med)
        base["ns_min"] = float(mn)
        base["n_runs"] = len(runs)
        data.setdefault((key[0], key[1]), []).append(base)
    for key in data:
        data[key].sort(key=lambda r: r["area"])
    return data


def piecewise_fit(areas, t, hi_slope, min_area=200.0):
    """Pinned two-segment log-log fit; low slope = 1, high slope given.
    Points below min_area are micro-map noise (a handful of ns per query) and
    excluded so they cannot pull the linear branch off its asymptotic line."""
    keep = areas >= min_area
    areas, t = areas[keep], t[keep]
    x = np.log(areas)
    y = np.log(t)
    n = len(x)
    best = None
    for j in range(2, n - 1):
        b1 = float(np.mean(y[:j] - x[:j]))
        b2 = float(np.mean(y[j:] - hi_slope * x[j:]))
        err = np.concatenate([(x[:j] + b1 - y[:j]),
                              (hi_slope * x[j:] + b2 - y[j:])])
        sse = float(err @ err)
        if best is None or sse < best[0]:
            xk = (b1 - b2) / (hi_slope - 1.0)
            best = (sse, j, 1.0, hi_slope, b1, b2, float(np.exp(xk)))
    return best


def free_fit(areas, t, min_area=200.0):
    """Unpinned fit for reporting the slopes the data actually wants."""
    keep = areas >= min_area
    areas, t = areas[keep], t[keep]
    x = np.log(areas)
    y = np.log(t)
    n = len(x)
    best = None
    for j in range(2, n - 1):
        s1, b1 = np.linalg.lstsq(np.vstack([x[:j], np.ones(j)]).T,
                                 y[:j], rcond=None)[0]
        s2, b2 = np.linalg.lstsq(np.vstack([x[j:], np.ones(n-j)]).T,
                                 y[j:], rcond=None)[0]
        err = np.concatenate([(s1*x[:j]+b1-y[:j]), (s2*x[j:]+b2-y[j:])])
        sse = float(err @ err)
        if best is None or sse < best[0]:
            best = (sse, j, float(s1), float(s2))
    return best


def local_exponents(rows, smooth=3):
    a = np.array([r["area"] for r in rows], float)
    t = np.array([r["ns_median"] for r in rows], float)
    e = np.diff(np.log(t)) / np.diff(np.log(a))
    am = np.sqrt(a[1:] * a[:-1])
    if smooth and len(e) >= smooth:
        kernel = np.ones(smooth) / smooth
        e = np.convolve(e, kernel, mode="valid")
        am = am[(smooth - 1) // 2: (smooth - 1) // 2 + len(e)]
    return am, e


def cost_constants(rows, heap=False, heap_rows=None):
    """Local, physically anchored calibration of T = ce*exp + cs*scan.

    The two regressors are collinear and, on maze maps, the per-expansion
    price drifts as the working set grows, so no single global pair of
    constants fits perfectly. Constants are therefore anchored locally:
      cs - T/scan on the three largest points, where each linear-scan step is
           a cache-hot integer compare and the price is nearly pure;
      ce - the heap variant's T/exp (identical expansion sequence, verified by
           the BFS oracle, with no linear scan), averaged over stable points.
    The returned r2 reports honestly how constant that local model is on the
    family (maze geometry makes it drift; scatter maps stay within ~1%).
    """
    stable = [r for r in rows if r["ns_median"] >= 3000]
    if len(stable) < 4:
        stable = rows
    if heap:
        X = np.array([[r["exp"], r["exp"] * np.log2(max(r["omax"], 2))]
                      for r in stable], float)
        beta = np.maximum(np.linalg.lstsq(
            X, np.array([r["ns_median"] for r in stable]), rcond=None)[0], 0.0)
        ce, cs = float(beta[0]), float(beta[1])
    else:
        big = sorted(stable, key=lambda r: r["area"])[-3:]
        cs = float(np.mean([r["ns_median"] / r["scan"] for r in big]))
        assert heap_rows is not None
        href = {r["area"]: r for r in heap_rows}
        ce = float(np.mean([href[r["area"]]["ns_median"] / r["exp"]
                            for r in stable if r["area"] in href]))
        beta = np.array([ce, cs])
    Xall = np.array([[r["exp"], r["scan"]] if not heap else
                     [r["exp"], r["exp"] * np.log2(max(r["omax"], 2))]
                     for r in rows], float)
    y = np.array([r["ns_median"] for r in rows], float)
    r2 = 1 - float(((y - Xall @ beta) ** 2).sum() /
                   ((y - y.mean()) @ (y - y.mean())))
    return ce, cs, r2


def main():
    data = load()
    families = sorted({k[0] for k in data})
    knee_out, tier_out = [], []
    fits, consts, frees = {}, {}, {}

    for fam in families:
        hi_slope = 2.0 if fam == "abstract" else 1.5
        for v in ("array", "bitmap"):
            rows = data[(fam, v)]
            a = np.array([r["area"] for r in rows], float)
            t = np.array([r["ns_median"] for r in rows], float)
            pin = piecewise_fit(a, t, hi_slope)
            fr = free_fit(a, t)
            ce, cs, r2 = cost_constants(rows, heap_rows=data[(fam, "heap")])
            fits[(fam, v)] = pin
            frees[(fam, v)] = fr
            consts[(fam, v)] = (ce, cs, r2)

            # knee via measured cost crossing cs*scan = ce*exp
            ratio = [(r["area"], cs * r["scan"] / max(ce * r["exp"], 1e-9))
                     for r in rows]
            cross_area = float("inf")       # inf: never crossed inside the sweep
            for (a0, q0), (a1, q1) in zip(ratio, ratio[1:]):
                if q0 <= 1.0 <= q1:
                    cross_area = float(np.interp(1.0, [q0, q1], [a0, a1]))
                    break
            amax = max(r["area"] for r in rows)
            # capacity-formula check around the fitted knee
            near = [r for r in rows if 0.4 * pin[6] <= r["area"] <= 2.5 * pin[6]]
            k = float(np.mean([r["k"] for r in near]))
            alpha = float(np.mean([r["alpha"] for r in near]))
            omax = float(np.mean([r["omax"] for r in near]))
            c_pred = k * k * alpha * pin[6] / 4.0
            omax_growth = free_fit(
                a, np.array([r["omax"] for r in rows], float) + 1e-9)[2]
            knee_out.append({
                "family": fam, "variant": v,
                "knee_area_cost_crossing":
                    round(cross_area, 1) if np.isfinite(cross_area)
                    else f">{amax}",
                "knee_side": round(np.sqrt(cross_area), 1)
                    if np.isfinite(cross_area) else f">{int(np.sqrt(amax))}",
                "knee_area_pinned_fit": round(pin[6], 1),
                "free_slope_lo": round(fr[2], 3),
                "free_slope_hi": round(fr[3], 3),
                "ce_ns_per_exp": round(ce, 3),
                "cs_ns_per_scan_entry": round(cs, 3),
                "cost_model_r2": round(r2, 4),
                "omax_growth_exponent": round(omax_growth, 3),
                "omax_measured_near_knee": round(omax, 1),
                "capacity_formula_predicts": round(c_pred, 1),
                "formula_over_measured": round(c_pred / omax, 2),
            })

        # heap: fit a single log-log slope (no knee expected in range)
        rows = data[(fam, "heap")]
        ce, ch, r2 = cost_constants(rows, heap=True)
        consts[(fam, "heap")] = (ce, ch, r2)
        ah = np.array([r["area"] for r in rows], float)
        th = np.array([r["ns_median"] for r in rows], float)
        heap_slope = float(np.polyfit(np.log(ah), np.log(th), 1)[0])
        frees[(fam, "heap_slope")] = heap_slope

    # ---- tier decision (scatter family is the nominal operating regime) ----
    def t_at(fam, v, area):
        hit = next((r for r in data[(fam, v)] if r["area"] == area), None)
        return hit["ns_median"] if hit else None

    # measured knees (cost crossing) for the scatter family, recomputed here
    knee_area = {}
    for v in ("array", "bitmap"):
        rows = data[("scatter", v)]
        ce, cs, _ = consts[("scatter", v)]
        q = [(r["area"], cs * r["scan"] / max(ce * r["exp"], 1e-9)) for r in rows]
        knee_area[v] = float("inf")
        for (a0, q0), (a1, q1) in zip(q, q[1:]):
            if q0 <= 1.0 <= q1:
                knee_area[v] = float(np.interp(1.0, [q0, q1], [a0, a1]))
                break
    knee_area["heap"] = float(max(r["area"] for r in data[("scatter", "heap")]))

    for name, side, area in TIERS:
        times = {v: t_at("scatter", v, area) for v in ("array", "bitmap", "heap")}
        # hard constraint: knee must lie above the target area; among the
        # feasible structures take the fastest one (no small-map regression)
        feasible = [v for v in times if knee_area[v] > area]
        fastest = min(feasible, key=lambda v: times[v])
        # small maps sit in the measurement-noise band: treat anything within
        # 5% of the fastest as a statistical tie and prefer the cache-friendly
        # bitmap there (zero regression risk and strictly faster on even tinier
        # maps), rather than letting sub-noise differences flip the decision
        tied = [v for v in feasible
                if times[v] <= times[fastest] * 1.05]
        if name == "small" and "bitmap" in tied:
            pick = "bitmap"
            tie_note = "tie<=5pct -> cache-friendly bitmap"
        else:
            pick = fastest
            tie_note = ""
        best_overall = min(times, key=lambda v: times[v])
        tier_out.append({
            "tier": name, "side": side, "area": area,
            "array_ns": times["array"], "bitmap_ns": times["bitmap"],
            "heap_ns": times["heap"], "chosen": pick,
            "decision_note": tie_note,
            "chosen_ns": times[pick],
            "chosen_knee_area": (round(knee_area[pick], 1)
                                 if np.isfinite(knee_area[pick])
                                 else f">{knee_area['heap']:.0f}"),
            "knee_above_target": bool(knee_area[pick] > area),
            "speedup_vs_plain_array": round(times["array"] / times[pick], 2),
            "is_fastest_of_three": bool(pick == best_overall),
            "overhead_vs_fastest_pct": round(
                100.0 * (times[pick] / times[best_overall] - 1.0), 2),
        })

    with open(os.path.join(ROOT, "bench_knees.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(knee_out[0].keys()))
        w.writeheader(); w.writerows(knee_out)
    with open(os.path.join(ROOT, "bench_tiers.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(tier_out[0].keys()))
        w.writeheader(); w.writerows(tier_out)

    # ---- figure 1: latency curves, one panel per family ----
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.6), dpi=150, sharey=True)
    fig.patch.set_facecolor(SURF)
    style = {"array": (C_ARRAY, "o", "array: O(n) find-min + O(n) lookup"),
             "bitmap": (C_BITMAP, "s", "array + in-open byte: O(n) find-min, O(1) lookup"),
             "heap": (C_HEAP, "^", "binary heap + pos[]: O(log n) both")}
    for ax, fam in zip(axes, families):
        ax.set_facecolor(SURF)
        for v in ("array", "bitmap", "heap"):
            rows = data[(fam, v)]
            col, mk, lab = style[v]
            ax.plot([r["area"] for r in rows], [r["ns_median"] for r in rows],
                    color=col, marker=mk, markersize=6, linewidth=2,
                    markerfacecolor=SURF, markeredgewidth=1.8, label=lab)
        knee_for_fam = {z["variant"]: z for z in knee_out if z["family"] == fam}
        for v, col in (("array", C_ARRAY), ("bitmap", C_BITMAP)):
            rows = data[(fam, v)]
            xk = knee_for_fam[v]["knee_area_cost_crossing"]
            if isinstance(xk, str):          # knee beyond the measured range
                last = rows[-1]
                ax.annotate(f"{v}: no knee in range\n(scan cost still < expansion)",
                            xy=(last["area"], last["ns_median"]),
                            xytext=(rows[0]["area"] * 1.4,
                                    last["ns_median"] * 0.18),
                            color=INK2, fontsize=8.2,
                            arrowprops=dict(arrowstyle="->", color=col, lw=1.2))
                continue
            pin = fits[(fam, v)]
            yk = np.exp(pin[4]) * xk
            ax.scatter([xk], [yk], s=170, facecolor="none", edgecolor=col,
                       linewidth=2.2, zorder=5)
            # offset direction depends on which variant so nearby knees
            # never overlap: array label drops below, bitmap rises above
            ytext = yk * (0.38 if v == "array" else 2.3)
            xtext = xk * (1.7 if v == "array" else 0.55)
            ax.annotate(f"{v} knee\n{xk:,.0f}", xy=(xk, yk),
                        xytext=(xtext, ytext),
                        color=INK2, fontsize=8.5,
                        arrowprops=dict(arrowstyle="->", color=col, lw=1.3))
        for name, side, area in TIERS:
            ax.axvline(area, color=MUTED, lw=0.9, ls=(0, (4, 4)), alpha=0.6)
        ax.set_xscale("log", base=2)
        ax.set_yscale("log")
        ax.set_title(f"{fam} family", color=INK, fontsize=11, loc="left")
        ax.grid(True, which="major", color=GRID, lw=0.8)
        ax.grid(True, which="minor", color=GRID, lw=0.4, alpha=0.5)
        for sp in ax.spines.values():
            sp.set_color("#c3c2b7")
        ax.tick_params(colors=INK2)
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v):,}"))
        ax.set_xlabel("map area A (grid cells)", color=INK)
    axes[0].set_ylabel("A* search time per query (ns, median)", color=INK)
    axes[0].legend(loc="upper left", frameon=False, fontsize=8.5)
    for t in axes[0].get_legend().get_texts():
        t.set_color(INK)
    fig.suptitle("A* open-list latency vs area: knee = expansion/scan cost crossing",
                 color=INK, fontsize=12.5, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(ROOT, "fig_latency.png"), facecolor=SURF)
    plt.close(fig)

    # ---- figure 2: local exponents per family ----
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.8), dpi=150, sharey=True)
    fig.patch.set_facecolor(SURF)
    for ax, fam in zip(axes, families):
        ax.set_facecolor(SURF)
        for v, col, mk in (("array", C_ARRAY, "o"), ("bitmap", C_BITMAP, "s"),
                           ("heap", C_HEAP, "^")):
            am, e = local_exponents(data[(fam, v)])
            ax.plot(am, e, color=col, marker=mk, markersize=6, linewidth=2,
                    markerfacecolor=SURF, markeredgewidth=1.8, label=v)
        ax.axhline(1.0, color=MUTED, lw=1, ls="--")
        ax.axhline(1.5, color=MUTED, lw=1, ls="--")
        ax.axhline(2.0, color=MUTED, lw=1, ls=":")
        ax.text(90, 1.03, "slope 1: expansion-bound", color=INK2, fontsize=8.5)
        ax.text(90, 1.53, "slope 1.5: perimeter-frontier scans", color=INK2, fontsize=8.5)
        ax.text(90, 1.92, "slope 2: space-filling frontier (abstract graph)",
                color=INK2, fontsize=8.5)
        knee_for_fam = {z["variant"]: z for z in knee_out if z["family"] == fam}
        for v, col in (("array", C_ARRAY), ("bitmap", C_BITMAP)):
            xk = knee_for_fam[v]["knee_area_cost_crossing"]
            if not isinstance(xk, str):
                ax.axvline(xk, color=col, lw=1.1, ls=(0, (2, 3)), alpha=0.8)
        ax.set_xscale("log", base=2)
        ax.set_title(f"{fam} family", color=INK, fontsize=11, loc="left")
        ax.set_ylim(0.6, 2.25)
        ax.set_xlim(80, 3_000_000)
        ax.grid(True, color=GRID, lw=0.8)
        for sp in ax.spines.values():
            sp.set_color("#c3c2b7")
        ax.tick_params(colors=INK2)
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v):,}"))
        ax.set_xlabel("map area A (grid cells)", color=INK)
    axes[0].set_ylabel(r"local slope $\Delta\log T / \Delta\log A$", color=INK)
    axes[1].legend(loc="lower right", frameon=False, fontsize=9)
    for t in axes[1].get_legend().get_texts():
        t.set_color(INK)
    fig.suptitle("Local scaling exponent per map family",
                 color=INK, fontsize=12.5, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(ROOT, "fig_exponent.png"), facecolor=SURF)
    plt.close(fig)

    print(json.dumps({"knees": knee_out, "tiers": tier_out,
                      "constants": {f"{k[0]}/{k[1]}": [round(x, 4) for x in v]
                                    for k, v in consts.items()},
                      "heap_loglog_slopes": {f: round(frees[(f, "heap_slope")], 3)
                                             for f in families}},
                     indent=2))


if __name__ == "__main__":
    main()
