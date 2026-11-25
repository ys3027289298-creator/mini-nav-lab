#!/usr/bin/env python3
"""Aggregate sweep.json into the SLA decision tables and trade-off curves.

Outputs (under sched/results/):
  summary.json            machine-readable aggregated metrics
  tradeoff_<mix>.csv      per-k curves for each mix
  conflict_<mix>.csv      SLA conflict / sacrificed-request counts
  closed_form.json        analytic inversion for the concurrency cap
"""
import csv
import json
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

import sim

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")
os.makedirs(RESULTS, exist_ok=True)

with open(os.path.join(HERE, "sweep.json")) as fh:
    rows = json.load(fh)


def agg(filters):
    """Average metrics across seeds for runs matching filters."""
    sel = [r for r in rows if all(r.get(k) == v for k, v in filters.items())]
    keys = [
        ("P0", "one_frame_hit_rate"), ("P0", "miss_rate"),
        ("P1", "starvation_rate"), ("P2", "starvation_rate"),
        ("P1", "max_sojourn_frames"), ("P2", "max_sojourn_frames"),
    ]
    metric_names = [f"{c}_{m}" for c, m in keys]
    out = defaultdict(list)
    for r in sel:
        for c, m in keys:
            out[f"{c}_{m}"].append(r[c][m])
        out["preempt_events"].append(r["preempt_events"])
        out["mean_preempt_loss_ratio"].append(r["mean_preempt_loss_ratio"])
        out["waste_share"].append(r["waste_share"])
        out["P1_arrivals"].append(r["P1"]["arrivals"])
        out["P2_arrivals"].append(r["P2"]["arrivals"])
        out["P1_starved"].append(r["P1"]["starved"])
        out["P2_starved"].append(r["P2"]["starved"])
    result = {name: 0.0 for name in metric_names}
    result.update({k: float(np.mean(v)) if v else 0.0
                   for k, v in out.items()})
    return result, len(sel)


summary = {"regimes": {}}
for regime in ("calibrated", "congestion", "congestion_ps"):
    summary["regimes"][regime] = {}
    mixes = ("balanced",) if regime == "congestion_ps" else tuple(sim.MIXES)
    for mix in mixes:
        table = []
        for k in sim.K_GRID:
            preempt_pol = "ps" if regime == "congestion_ps" else "reserve"
            a, n = agg({"regime": regime, "mix": mix, "k": k,
                        "policy": preempt_pol, "preempt": True})
            a["k"] = k
            a["n_runs"] = n
            table.append(a)
        summary["regimes"][regime][mix] = table

        # CSV trade-off curve
        path = os.path.join(RESULTS, f"tradeoff_{regime}_{mix}.csv")
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["k", "P0_one_frame_hit", "P0_miss",
                        "P1_starvation", "P2_starvation",
                        "P1_max_sojourn_frames", "P2_max_sojourn_frames",
                        "preemptions", "mean_preempt_loss", "waste_share"])
            for a in table:
                w.writerow([a["k"], f"{a['P0_one_frame_hit_rate']:.4f}",
                            f"{a['P0_miss_rate']:.4f}",
                            f"{a['P1_starvation_rate']:.4f}",
                            f"{a['P2_starvation_rate']:.4f}",
                            f"{a['P1_max_sojourn_frames']:.1f}",
                            f"{a['P2_max_sojourn_frames']:.1f}",
                            f"{a['preempt_events']:.1f}",
                            f"{a['mean_preempt_loss_ratio']:.4f}",
                            f"{a['waste_share']:.4f}"])

# ---- conflict analysis: how many requests are sacrificed at each k ----
conflict = {}
for mix in sim.MIXES:
    lines = []
    for k in sim.K_GRID:
        a, _ = agg({"regime": "congestion", "mix": mix, "k": k,
                    "policy": "reserve", "preempt": True})
        p0_ok = a["P0_one_frame_hit_rate"] >= sim.SLA_HIT
        # starvation count is a mean over seeds; scale to expected per-run count
        p1_sac = a["P1_starved"]
        p2_sac = a["P2_starved"]
        wait_violated = (a["P1_max_sojourn_frames"] > sim.STARV_FRAMES or
                         a["P2_max_sojourn_frames"] > sim.STARV_FRAMES)
        lines.append({
            "k": k,
            "P0_hit": round(a["P0_one_frame_hit_rate"], 4),
            "P0_SLA_met": p0_ok,
            "wait_5s_violated": wait_violated,
            "expected_P1_starved_per_run": round(p1_sac, 2),
            "expected_P2_starved_per_run": round(p2_sac, 2),
            "total_sacrificed_low": round(p1_sac + p2_sac, 2),
        })
    conflict[mix] = lines
    with open(os.path.join(RESULTS, f"conflict_congestion_{mix}.csv"),
              "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(lines[0].keys()))
        w.writeheader()
        w.writerows(lines)

# ---- analytic (closed-form) inversion of the concurrency cap ----
# Frame budget F expansion slots. Under the reserve policy each of the k
# in-flight low-priority jobs occupies one worker, which (in the worst frame)
# consumes one slot before a P0 request can be served. A P0 request of size W0
# therefore meets its one-frame SLA without preemption iff
#
#     W0 + k <= F                                          (Eq. reservation)
#
# To guarantee the 99% one-frame hit target, size the frame for the 99th
# percentile of the P0 work distribution, W0 = W_{0.99}. Inverting:
#
#     k* = max { k in N : k <= F - W_{0.99} }
#        = floor(F - W_{0.99})
#
# Above k*, a P0 at the 99th-percentile size no longer fits alongside the
# reserved workers; the scheduler must preempt. Preemption reclaims a worker's
# slot but only from jobs with remaining work > F (the spec criterion), so the
# guarantee degrades exactly when no such long job is available to interrupt.
#
# The fair-share view gives the same boundary from the throughput side:
# each active job draws floor(F/k) expansions per frame, so the expected
# low-priority service rate is F/k per job and P0 sees a residual budget of
# F - k reservations on every frame it needs the whole frame.
work = sim.load_work()
p99_p0 = float(np.percentile(work["P0"], 99))
p95_p0 = float(np.percentile(work["P0"], 95))
mean_p0 = float(work["P0"].mean())
k_star_reserve = int(sim.F - p99_p0)        # floor, sizes are integer slots
k_star_p95 = int(sim.F - p95_p0)
closed = {
    "F_slots": sim.F, "frame_ms": sim.FRAME_MS,
    "starvation_frames": sim.STARV_FRAMES,
    "P0_mean_expansions": mean_p0,
    "P0_p95_expansions": p95_p0,
    "P0_p99_expansions": p99_p0,
    "SLA_hit_target": sim.SLA_HIT,
    "derivation":
        "k* = floor(F - W_0.99); P0 of 99th-pct size fits with k reserved "
        "low-priority workers iff W_0.99 + k <= F",
    "k_star_reserve_p99": k_star_reserve,
    "k_star_reserve_p95": k_star_p95,
    "by_k": [],
}
for k in sim.K_GRID:
    fair = sim.F // k
    residual = sim.F - k                     # slots left if every worker reserves 1
    closed["by_k"].append({
        "k": k,
        "fair_share_per_job": fair,
        "residual_for_P0_worst_frame": residual,
        "p99_P0_fits_without_preemption": bool(p99_p0 <= residual),
        "p95_P0_fits_without_preemption": bool(p95_p0 <= residual),
    })
# k* measured from the congestion simulation (largest k clearing 99%)
closed["simulated_k_star"] = {}
for mix in sim.MIXES:
    table = summary["regimes"]["congestion"][mix]
    feas = [a["k"] for a in table
            if a["P0_one_frame_hit_rate"] >= sim.SLA_HIT]
    closed["simulated_k_star"][mix] = max(feas) if feas else None

with open(os.path.join(RESULTS, "closed_form.json"), "w") as fh:
    json.dump(closed, fh, indent=2)
with open(os.path.join(RESULTS, "summary.json"), "w") as fh:
    json.dump({"summary": summary, "conflict": conflict,
               "closed_form": closed}, fh, indent=2)

print("=== closed-form inversion ===")
print(f"F={sim.F} slots/frame, P0 mean={mean_p0:.2f}, "
      f"p95={p95_p0:.1f}, p99={p99_p0:.1f}")
print(f"analytic k* = floor(F - W_0.99) = floor({sim.F} - {p99_p0:.0f}) "
      f"= {k_star_reserve}")
print(f"analytic k* (p95 sizing) = {k_star_p95}")
for row in closed["by_k"]:
    print(f"k={row['k']} fair={row['fair_share_per_job']:2d} "
          f"residual_P0={row['residual_for_P0_worst_frame']:2d} "
          f"p99_fits={row['p99_P0_fits_without_preemption']} "
          f"p95_fits={row['p95_P0_fits_without_preemption']}")
print("\nsimulated k* (largest k meeting 99% in congestion):",
      closed["simulated_k_star"])
print("\nwrote results to", RESULTS)
