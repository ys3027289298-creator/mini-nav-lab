#!/usr/bin/env python3
"""Emit chart_data.json: the exact series the HTML report plots."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
rows = json.load(open(os.path.join(HERE, "sweep.json")))


def series(regime, mix, policy, preempt):
    out = defaultdict(lambda: defaultdict(list))
    for r in rows:
        if (r["regime"] == regime and r["mix"] == mix
                and r["policy"] == policy and r["preempt"] == preempt):
            k = r["k"]
            out[k]["P0_hit"].append(r["P0"]["one_frame_hit_rate"])
            out[k]["P2_starv"].append(r["P2"]["starvation_rate"])
            out[k]["P1_starv"].append(r["P1"]["starvation_rate"])
            out[k]["P2_max"].append(r["P2"]["max_sojourn_frames"])
            out[k]["P1_max"].append(r["P1"]["max_sojourn_frames"])
            out[k]["P1_starved_n"].append(r["P1"]["starved"])
            out[k]["P2_starved_n"].append(r["P2"]["starved"])
            out[k]["P1_arr_n"].append(r["P1"]["arrivals"])
            out[k]["P2_arr_n"].append(r["P2"]["arrivals"])
            out[k]["preempt"].append(r["preempt_events"])
            out[k]["loss"].append(r["mean_preempt_loss_ratio"])
            out[k]["waste"].append(r["waste_share"])
    ks = sorted(out)
    def m(k, name):
        return round(float(np.mean(out[k][name])), 4)
    return [{
        "k": k,
        "P0_hit": m(k, "P0_hit"),
        "P1_starvation": m(k, "P1_starv"),
        "P2_starvation": m(k, "P2_starv"),
        "P1_starved_count": m(k, "P1_starved_n"),
        "P2_starved_count": m(k, "P2_starved_n"),
        "P1_arrivals": m(k, "P1_arr_n"),
        "P2_arrivals": m(k, "P2_arr_n"),
        "P1_max_sojourn": m(k, "P1_max"),
        "P2_max_sojourn": m(k, "P2_max"),
        "preemptions": m(k, "preempt"),
        "preempt_loss": m(k, "loss"),
        "waste_share": m(k, "waste"),
    } for k in ks]


data = {"meta": {
            "F": sim.F, "frame_ms": sim.FRAME_MS,
            "starvation_frames": sim.STARV_FRAMES,
            "sla_target": sim.SLA_HIT,
            "congestion_work": sim.CONGESTION_WORK,
            "congestion_util": sim.CONGESTION_UTIL,
        },
        "congestion": {}, "calibrated": {}, "fcfs": {}, "ps": {}}

for mix in sim.MIXES:
    data["congestion"][mix] = series("congestion", mix, "reserve", True)
    data["calibrated"][mix] = series("calibrated", mix, "reserve", True)
    data["fcfs"][mix] = series("congestion", mix, "reserve", False)
data["ps"]["balanced"] = series("congestion_ps", "balanced", "ps", True)

# calibration distribution summary for the report
calib_rows = open(os.path.join(HERE, "calib", "calib.csv")).readlines()[1:]
cal = defaultdict(list)
for line in calib_rows:
    c, e, reachable, dist = line.strip().split(",")
    cal[c].append(int(e))
data["calibration"] = {
    c: {
        "mean": round(float(np.mean(v)), 2),
        "p50": float(np.percentile(v, 50)),
        "p95": float(np.percentile(v, 95)),
        "p99": float(np.percentile(v, 99)),
        "max": int(np.max(v)),
    } for c, v in cal.items()}

with open(os.path.join(HERE, "results", "chart_data.json"), "w") as fh:
    json.dump(data, fh, indent=2)
print("wrote chart_data.json")
