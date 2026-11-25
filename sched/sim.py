#!/usr/bin/env python3
"""Preemptive priority scheduler simulator for the nav service SLA.

Model
-----
Time is frame-stepped. One frame holds F node-expansion slots (F=25,
calibrated so solo P0 work <= F for >=99% of measured A* instances;
1 frame = 20 ms -> 5 s = 250 frames).

Request classes (work sizes drawn from the instrumented A* calibration):
  P0 main-quest  : strict SLA, must finish within one frame of arrival
  P1 normal      : preemptible, queueable
  P2 background  : preemptible, queueable

Two capacity policies for the k low-priority slots, selectable via --policy:

  reserve  (default): every in-flight low job is a worker that reserves one
                      expansion slot per frame. P0 may preempt it, but the
                      reservation exists whenever P0 does not need the slot.
                      -> more in-flight jobs => fewer slots left for P0,
                         which is the "mutual drag" the SLA analysis inverts.
  ps                 : pure processor sharing; low jobs only consume slots
                      they actually use (work-conserving, no reservation).

Preemption rule (per spec): when P0 arrives, a running low job j is
interrupted iff remaining_j > F; interrupted progress is discarded and the
job requeues with its full original work size. Loss ratio of an interruption
= spent_j / W_j (expanded nodes / total expansions of the instance).

Fair share: in one frame each active low job receives at most
floor(budget / n_active) expansions, so long jobs stay in-flight across
frames and remain preemptible instead of draining inside a single frame.
"""
import argparse
import csv
import collections
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, "calib", "calib.csv")

F = 25                 # expansion slots per frame
FRAME_MS = 20.0
STARV_FRAMES = 250     # 5 s / 20 ms
MIXES = {
    # name: (P0, P1, P2) arrival fractions
    "main_heavy": (0.35, 0.45, 0.20),
    "balanced":   (0.20, 0.50, 0.30),
    "bg_heavy":   (0.10, 0.40, 0.50),
}
CLASSES = ("P0", "P1", "P2")
LOW = ("P1", "P2")


def load_work():
    work = {"P0": [], "P1": [], "P2": []}
    with open(CALIB) as fh:
        for row in csv.DictReader(fh):
            work[row["class"]].append(int(row["expansions"]))
    return {k: np.asarray(v, dtype=np.int64) for k, v in work.items()}


def simulate(k, mix, offered_util, n_frames, warmup, seed, policy="reserve",
             preempt=True, work=None):
    """One simulation run.

    k            max simultaneously running low-priority jobs
    mix          key into MIXES
    offered_util total offered expansion work per frame (fraction of F)
    preempt      False -> FCFS baseline (no preemption)
    """
    rng = np.random.default_rng(seed)
    means = {c: work[c].mean() for c in CLASSES}
    frac = dict(zip(CLASSES, MIXES[mix]))
    rates = {c: offered_util * frac[c] / means[c] for c in CLASSES}

    counts = {c: rng.poisson(rates[c], size=n_frames) for c in CLASSES}

    def draw(c):
        return int(work[c][rng.integers(0, len(work[c]))])

    # job: id -> [class, total W, remaining, spent, arrival, first_run]
    jobs = {}
    next_id = 0
    p0_queue = collections.deque()
    low_wait = {"P1": collections.deque(), "P2": collections.deque()}
    active = collections.deque()
    active_ids = set()

    stats = {c: collections.Counter() for c in CLASSES}
    first_waits = {c: [] for c in LOW}
    sojourns = {c: [] for c in CLASSES}
    preempt_events = []

    def new_job(c, frame):
        nonlocal next_id
        jid = next_id
        next_id += 1
        w = draw(c)
        jobs[jid] = [c, w, w, 0, frame, -1]
        return jid

    def admit(frame):
        while len(active) < k:
            picked = None
            for c in LOW:                     # P1 strict priority over P2
                if low_wait[c]:
                    picked = low_wait[c].popleft()
                    break
            if picked is None:
                break
            if jobs[picked][5] < 0:
                jobs[picked][5] = frame
            active.append(picked)
            active_ids.add(picked)

    def serve_low(budget, frame, measure):
        """Fair-share RR. Returns (leftover budget, {jid: slots given})."""
        given = {}
        if not active or budget <= 0:
            return budget, given
        quota = max(1, budget // len(active))
        for jid in list(active):
            if budget <= 0:
                break
            j = jobs.get(jid)
            if j is None:
                continue
            give = min(j[2], quota, budget)
            j[2] -= give
            j[3] += give
            budget -= give
            given[jid] = give
            if j[2] == 0:
                active.remove(jid)
                active_ids.discard(jid)
                if measure and j[4] >= warmup:
                    stats[j[0]]["done"] += 1
                    stats[j[0]]["useful_expansions"] += j[1]
                    sojourns[j[0]].append(frame - j[4] + 1)
                    first_waits[j[0]].append(j[5] - j[4])
                del jobs[jid]
        return budget, given

    for f in range(n_frames):
        measure = f >= warmup

        # 1. arrivals
        for c in CLASSES:
            for _ in range(int(counts[c][f])):
                jid = new_job(c, f)
                if c == "P0":
                    p0_queue.append(jid)
                else:
                    low_wait[c].append(jid)

        # 2. admit up to k low jobs
        admit(f)

        # 3. low-priority work runs first at its fair per-frame share.
        #    These expansions are already "in flight" when a P0 request
        #    arrives later in the frame; only leftover slots, plus slots
        #    reclaimed by preemption, are available to P0.
        cap = F
        given = {}
        if active:
            cap, given = serve_low(cap, f, measure)

        # 4. P0 demand (new arrivals + any carried over)
        p0_demand = sum(jobs[j][2] for j in p0_queue)

        # 5. preemption at the P0 arrival point.
        #    Criterion per running low job j (spec):
        #      P0 waiting AND remaining_j > F  -> interrupt j.
        #    Interrupting reclaims the slot j already consumed this frame;
        #    its spent expansions are discarded and it requeues at full W.
        #    Victim order: P2 before P1, largest total work first; stop once
        #    the P0 demand fits in the frame.
        if preempt and p0_queue and p0_demand > cap:
            ordered = sorted(
                active,
                key=lambda j: (0 if jobs[j][0] == "P2" else 1,
                               -jobs[j][1]))
            victims = []
            for jid in ordered:
                if p0_demand <= cap:
                    break
                j = jobs[jid]
                if j[2] <= F:
                    continue                 # finish-in-frame is cheaper
                victims.append(jid)
                cap += given.get(jid, 0)     # reclaim this frame's slot
                cap = min(cap, F)
            for jid in victims:
                j = jobs[jid]
                loss = j[3] / j[1] if j[1] else 0.0
                if measure:
                    preempt_events.append((f, j[0], loss))
                    stats[j[0]]["preemptions"] += 1
                    stats[j[0]]["wasted_expansions"] += j[3]
                c, arr, w, first = j[0], j[4], j[1], j[5]
                jobs[jid] = [c, w, w, 0, arr, first]   # progress discarded
                active.remove(jid)
                active_ids.discard(jid)
                low_wait[c].appendleft(jid)
            if victims:
                admit(f)                      # backfill freed worker slots

        # 6. P0 executes with whatever capacity is available
        p0_done = []
        for jid in p0_queue:
            j = jobs[jid]
            if cap == 0:
                break
            serve = min(j[2], cap)
            j[2] -= serve
            j[3] += serve
            cap -= serve
            if j[2] == 0:
                p0_done.append(jid)
        for jid in p0_done:
            p0_queue.remove(jid)
            j = jobs[jid]
            if measure and j[4] >= warmup:
                stats["P0"]["done"] += 1
                sojourns["P0"].append(f - j[4] + 1)
            del jobs[jid]
        for jid in list(p0_queue):           # unfinished on arrival frame
            if measure and jobs[jid][4] == f:
                stats["P0"]["miss"] += 1
        if measure:
            stats["P0"]["arrivals"] += int(counts["P0"][f])

        # 7. any leftover slots go back to low jobs
        if preempt:
            cap, _ = serve_low(cap, f, measure)

        if measure:
            for c in LOW:
                stats[c]["arrivals"] += int(counts[c][f])

    # jobs still present at horizon; only count jobs that arrived inside the
    # measurement window so starved counts share arrivals' denominator
    for jid, j in list(jobs.items()):
        c = j[0]
        if c == "P0" or j[4] < warmup:
            continue
        sojourns[c].append(n_frames - j[4])
        if j[5] >= 0:
            first_waits[c].append(j[5] - j[4])

    out = {"k": k, "mix": mix, "policy": policy, "preempt": preempt,
           "rates": rates, "seed": seed}
    for c in CLASSES:
        sj = np.asarray(sojourns[c], dtype=float)
        d = dict(stats[c])
        d.update({
            "arrivals": stats[c]["arrivals"],
            "done": stats[c]["done"],
            "max_sojourn_frames": float(sj.max()) if len(sj) else 0.0,
            "p95_sojourn_frames": float(np.percentile(sj, 95)) if len(sj) else 0.0,
            "starved": int((sj > STARV_FRAMES).sum()),
            "starvation_rate": float((sj > STARV_FRAMES).mean()) if len(sj) else 0.0,
        })
        if c in first_waits:
            fw = np.asarray(first_waits[c], dtype=float)
            d["max_first_wait_frames"] = float(fw.max()) if len(fw) else 0.0
            d["p95_first_wait_frames"] = float(np.percentile(fw, 95)) if len(fw) else 0.0
        out[c] = d

    p0_arr = out["P0"]["arrivals"]
    p0_miss = out["P0"].get("miss", 0)
    out["P0"]["one_frame_hit_rate"] = 1.0 - p0_miss / p0_arr if p0_arr else 1.0
    out["P0"]["miss_rate"] = p0_miss / p0_arr if p0_arr else 0.0
    out["preempt_events"] = len(preempt_events)
    out["mean_preempt_loss_ratio"] = (
        float(np.mean([e[2] for e in preempt_events])) if preempt_events else 0.0)
    total_waste = sum(stats[c]["wasted_expansions"] for c in LOW)
    total_useful = sum(stats[c]["useful_expansions"] for c in LOW)
    out["wasted_expansions"] = total_waste
    out["waste_share"] = (total_waste / (total_waste + total_useful)
                          if total_waste + total_useful else 0.0)
    return out


# Congestion regime: fixed work sizes chosen so low-priority queues stay
# backlogged and the 5 s (250-frame) starvation line is reachable. P0 stays
# small (10 expansions, inside the calibrated p99=20).
CONGESTION_WORK = {
    "P0": 10,
    "P1": 100,
    "P2": 600,
}
CONGESTION_UTIL = 7.0
K_GRID = (1, 2, 3, 4, 5, 6)
SLA_HIT = 0.99


def congestion_work(n=4000):
    return {c: np.full(n, w, dtype=np.int64)
            for c, w in CONGESTION_WORK.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=30000)
    ap.add_argument("--warmup", type=int, default=6000)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--out", default=os.path.join(HERE, "sweep.json"))
    args = ap.parse_args()

    real_work = load_work()
    cong_work = congestion_work()
    rows = []

    def run(regime, work, util, k, mix, seed, policy, preempt):
        r = simulate(k, mix, util, args.frames, args.warmup, seed,
                     policy, preempt, work)
        r["regime"] = regime
        rows.append(r)

    for mix in MIXES:
        # Real calibrated workload: realistic arrival rates, light load
        for k in K_GRID:
            for seed in range(args.seeds):
                run("calibrated", real_work, 0.85, k, mix, seed,
                    "reserve", True)
                run("calibrated", real_work, 0.85, k, mix, seed,
                    "reserve", False)
        # Congestion regime: backlogged low queues, both thresholds crossed
        for k in K_GRID:
            for seed in range(args.seeds):
                run("congestion", cong_work, CONGESTION_UTIL, k, mix, seed,
                    "reserve", True)
                run("congestion", cong_work, CONGESTION_UTIL, k, mix, seed,
                    "reserve", False)
        # processor-sharing cross-check, balanced mix only
        if mix == "balanced":
            for k in K_GRID:
                for seed in range(args.seeds):
                    run("congestion_ps", cong_work, CONGESTION_UTIL, k, mix,
                        seed, "ps", True)

    with open(args.out, "w") as fh:
        json.dump(rows, fh)
    print(f"wrote {len(rows)} runs -> {args.out}")


if __name__ == "__main__":
    main()
