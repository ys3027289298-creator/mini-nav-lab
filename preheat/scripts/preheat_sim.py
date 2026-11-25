#!/usr/bin/env python3
"""preheat_sim.py — cache preheat batch sizing and conflict adjudication.

Model (all symbols defined in README.md):
  benefit per block b_i   = (t_q(n_i) - t_hit(n_i)) * p_i
  preheat cost per block  = c_i = Q_i * t_q(n_i),  Q_i = n_i*(n_i-1)
  window capacity bound   = B_cap = floor(W_idle / c_ref)
  first-arrival wait      = t_hit on a preheated block, else t_q(n)
  hard constraints        = wait <= SLO (100 ms) AND total preheat <= W_idle

Calibration: bench_open.csv / bench_choke.csv / bench_hit.csv produced by
`make measure`. t_q is fitted as a power law t = a*n^k per profile and per
statistic (mean drives throughput cost; p95/max drive the SLO check).
Fits used *inside* the measured n range are interpolation; anything above
n=49152 is extrapolation and labelled as such in every output.

Run: python3 -I scripts/preheat_sim.py --tier S|M|L|all [--window 30000]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import site
import sys
from dataclasses import dataclass, field

# -I (isolated mode) drops user site-packages; our only third-party need is
# matplotlib, installed there. Re-add that single trusted local directory.
for _p in site.getusersitepackages(), *site.getsitepackages():
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.append(_p)

# ---------------------------------------------------------------- constants
SLO_MS = 100.0                 # hard first-arrival latency budget, ms
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "output")
N_MEASURED_MAX = 49152         # largest n actually benchmarked

# Three tiers: same total cell budget (~12k cells), varying block granularity.
# Every (n, profile) pair here is inside the measured range -> no extrapolation.
TIERS = {
    # name: (blocks M, block cells n, terrain profile)
    "S": (64, 192,  "choke"),   # 64 small blocks  (16x12, the grid.c lab size)
    "M": (16, 768,  "choke"),   # 16 medium blocks (32x24)
    "L": (7,  1728, "choke"),   # 7 large blocks   (48x36)
}

# Stress tier kept strictly separate: million-cell blocks whose cold latency
# is *extrapolated*; exists to exercise the SLO-conflict branch honestly.
STRESS = {"X": (4, 1_048_576, "choke")}  # 4 blocks of 1024x1024


# ------------------------------------------------------------- calibration
@dataclass
class PowerFit:
    profile: str
    stat: str
    a: float
    k: float
    r2: float
    n_max_measured: int

    def predict(self, n: int) -> float:
        return self.a * (n ** self.k)

    def extrapolates(self, n: int) -> bool:
        return n > self.n_max_measured


def load_calib(path: str) -> dict[str, list[dict]]:
    rows: dict[str, list[dict]] = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows.setdefault(r["profile"], []).append(r)
    return rows


def fit_power(rows: list[dict], stat: str) -> PowerFit:
    xs = [math.log(int(r["n"])) for r in rows]
    ys = [math.log(float(r[stat])) for r in rows]
    xbar, ybar = sum(xs) / len(xs), sum(ys) / len(ys)
    sxx = sum((x - xbar) ** 2 for x in xs)
    sxy = sum((x - xbar) * (y - ybar) for x, y in zip(xs, ys))
    k = sxy / sxx
    a = math.exp(ybar - k * xbar)
    # R^2 in log-log space
    yhat = [a * (int(r["n"]) ** k) for r in rows]
    ylog = [math.log(v) for v in yhat]
    ss_res = sum((y - yh) ** 2 for y, yh in zip(ys, ylog))
    ss_tot = sum((y - ybar) ** 2 for y in ys)
    profile = rows[0]["profile"]
    n_max = max(int(r["n"]) for r in rows)
    return PowerFit(profile, stat, a, k, 1 - ss_res / ss_tot, n_max)


@dataclass
class Calibration:
    fits: dict[tuple[str, str], PowerFit] = field(default_factory=dict)

    @classmethod
    def load(cls) -> "Calibration":
        c = cls()
        for fname in ("bench_open.csv", "bench_choke.csv"):
            rows = load_calib(os.path.join(DATA, fname))
            for profile, prows in rows.items():
                for stat in ("mean_ms", "p95_ms", "max_ms"):
                    c.fits[(profile, stat)] = fit_power(prows, stat)
        hit_rows = load_calib(os.path.join(DATA, "bench_hit.csv"))["hit"]
        # measured warm lookup is effectively n-independent (a keyed table
        # read); the power-law fit there has R^2 ~ 0.46 and is meaningless,
        # so use the flat sample mean instead
        hit_mean = sum(float(r["mean_ms"]) for r in hit_rows) / len(hit_rows)
        flat = PowerFit("hit", "mean_ms", hit_mean, 0.0,
                        float("nan"), max(int(r["n"]) for r in hit_rows))
        c.fits[("hit", "mean_ms")] = flat
        return c

    def tq(self, profile: str, n: int, stat: str = "mean_ms") -> float:
        return self.fits[(profile, stat)].predict(n)

    def thit(self, n: int) -> float:
        return self.fits[("hit", "mean_ms")].predict(n)


# ---------------------------------------------------------------- scheduling
@dataclass
class Block:
    bid: str
    n: int
    profile: str
    p: float                 # visit probability
    c_ms: float              # full preheat cost, ms
    cold_ms: float           # p95 cold first-query wait, ms
    cold_max_ms: float
    warm_ms: float
    benefit_ms: float        # expected saved wait per arrival, ms

    @property
    def window_feasible(self) -> bool:
        return self.c_ms <= WINDOW_MS[0]


# module-level window so Block.window_feasible reads the active scenario
WINDOW_MS = [30_000.0]


def build_blocks(tier: str, calib: Calibration, m: int, n: int, profile: str,
                 bid_prefix: str = "B") -> list[Block]:
    # uniform visit probability across blocks; p_i sum to 1 over the tier
    p = 1.0 / m
    blocks = []
    for i in range(m):
        tq_mean = calib.tq(profile, n, "mean_ms")
        tq_p95 = calib.tq(profile, n, "p95_ms")
        tq_max = calib.tq(profile, n, "max_ms")
        thit = calib.thit(n)
        q = n * (n - 1)
        c = q * tq_mean
        blocks.append(Block(
            bid=f"{tier}-{bid_prefix}{i:02d}", n=n, profile=profile, p=p,
            c_ms=c, cold_ms=tq_p95, cold_max_ms=tq_max, warm_ms=thit,
            benefit_ms=(tq_mean - thit) * p,
        ))
    return blocks


def schedule(blocks: list[Block], window_ms: float) -> dict:
    """Greedy next-fit-decreasing bin packing into idle windows.

    A block whose own full preheat exceeds the window can never be heated and
    is dropped up front. Everything else is packed, highest cost first, into
    the minimum number of windows; the first window's population is the
    batch size. Returns per-block verdicts and window loadings.
    """
    dropped, packed = [], []
    for b in sorted(blocks, key=lambda b: b.c_ms, reverse=True):
        if b.c_ms > window_ms:
            dropped.append(b)
        else:
            packed.append(b)

    windows: list[list[Block]] = []
    loads: list[float] = []
    for b in packed:  # already cost-sorted
        for wi, load in enumerate(loads):
            if load + b.c_ms <= window_ms:
                windows[wi].append(b)
                loads[wi] += b.c_ms
                break
        else:
            windows.append([b])
            loads.append(b.c_ms)

    verdicts = {}
    for b in dropped:
        if b.cold_ms > SLO_MS:
            reason = "SLO_breach_and_window_conflict"
        elif b.cold_ms > 0.8 * SLO_MS:
            # within 20% of the SLO: dropping leaves no latency headroom
            reason = "window_conflict_SLO_headroom_at_risk"
        else:
            reason = "preheat_cost_exceeds_window_cold_still_under_SLO"
        verdicts[b.bid] = {"action": "drop", "reason": reason,
                           "cost_ms": b.c_ms, "cold_p95_ms": b.cold_ms,
                           "cold_max_ms": b.cold_max_ms}
    for wi, win in enumerate(windows):
        for b in win:
            verdicts[b.bid] = {"action": "preheat", "window": wi,
                               "cost_ms": b.c_ms, "cold_p95_ms": b.cold_ms}
    return {"windows": windows, "loads": loads, "dropped": dropped,
            "verdicts": verdicts}


def batch_sweep(blocks: list[Block], window_ms: float) -> list[dict]:
    """Vary batch size b = 1..M; record window occupancy and worst wait.

    Pre-heating target is the b highest-probability blocks (here p is
    uniform, so cost-ascending order maximises count that fits).
    """
    ordered = sorted(blocks, key=lambda b: b.c_ms)
    rows = []
    for b in range(1, len(blocks) + 1):
        chosen = ordered[:b]
        load = sum(x.c_ms for x in chosen)
        heated = {x.bid for x in chosen}
        waits = [(x.warm_ms if x.bid in heated else x.cold_ms) for x in blocks]
        rows.append({
            "batch": b,
            "window_load_ms": load,
            "window_load_frac": load / window_ms,
            "worst_wait_p95_ms": max(waits),
            "slo_ok": max(waits) <= SLO_MS,
            "window_ok": load <= window_ms,
            "feasible": max(waits) <= SLO_MS and load <= window_ms,
            "expected_wait_ms": sum(
                (x.warm_ms if x.bid in heated else x.cold_ms) * x.p
                for x in blocks),
        })
    return rows


def cap_from_formula(blocks: list[Block], window_ms: float) -> int:
    """B_cap = floor(W_idle / c_ref), c_ref = mean per-block cost."""
    c_ref = sum(b.c_ms for b in blocks) / len(blocks)
    return math.floor(window_ms / c_ref), c_ref


# --------------------------------------------------------------------- run
def run_tier(name: str, m: int, n: int, profile: str, calib: Calibration,
             window_ms: float, stress: bool = False) -> dict:
    WINDOW_MS[0] = window_ms
    blocks = build_blocks(name, calib, m, n, profile)
    sched = schedule(blocks, window_ms)
    sweep = batch_sweep(blocks, window_ms)
    b_cap, c_ref = cap_from_formula(blocks, window_ms)
    feasible = [r for r in sweep if r["feasible"]]
    chosen = max(feasible, key=lambda r: r["batch"]) if feasible else None

    heated_ids = {bid for bid, v in sched["verdicts"].items()
                  if v["action"] == "preheat"}
    total_benefit = sum(b.benefit_ms for b in blocks
                        if b.bid in heated_ids)
    extrap = calib.fits[(profile, "mean_ms")].extrapolates(n)
    worst_cold_p95 = max(b.cold_ms for b in blocks)
    worst_cold_max = max(b.cold_max_ms for b in blocks)
    result = {
        "tier": name,
        "stress_extrapolation": extrap,
        "blocks": m,
        "block_cells": n,
        "pairs_per_block_Q": n * (n - 1),
        "profile": profile,
        "window_ms": window_ms,
        "tq_mean_ms": calib.tq(profile, n, "mean_ms"),
        "tq_p95_ms": calib.tq(profile, n, "p95_ms"),
        "tq_max_ms": calib.tq(profile, n, "max_ms"),
        "thit_ms": calib.thit(n),
        "per_block_cost_ms": blocks[0].c_ms,
        "B_cap_formula": b_cap,
        "c_ref_ms": c_ref,
        "scheduled_windows": len(sched["windows"]),
        "first_window_batch": len(sched["windows"][0]) if sched["windows"] else 0,
        "chosen_batch_size": chosen["batch"] if chosen else 0,
        "chosen_window_load_ms": chosen["window_load_ms"] if chosen else None,
        "chosen_worst_wait_p95_ms":
            chosen["worst_wait_p95_ms"] if chosen else None,
        "slo_pass_all_cold": worst_cold_p95 <= SLO_MS,
        "slo_margin_p95_ms": SLO_MS - worst_cold_p95,
        "slo_margin_max_ms": SLO_MS - worst_cold_max,
        "extrapolation_warning":
            "cold latency is EXTRAPOLATED, not measured; linear open-list "
            "A* has O(n^2) worst case, true max likely higher"
            if extrap else None,
        "dropped": [
            {"bid": b.bid, **{k: v for k, v in sched["verdicts"][b.bid].items()
                              if k != "action"}}
            for b in sched["dropped"]
        ],
        "total_expected_benefit_ms_per_arrival": total_benefit,
        "sweep": sweep,
    }
    return result


def write_csv(path: str, rows: list[dict]) -> None:
    keys = list(rows[0].keys())
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def plot_curves(name: str, result: dict, path: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(f"  [warn] matplotlib unavailable, skipping plot for tier {name}")
        return False

    sweep = result["sweep"]
    b = [r["batch"] for r in sweep]
    wait = [r["worst_wait_p95_ms"] for r in sweep]
    occ = [r["window_load_frac"] * 100 for r in sweep]
    chosen = result["chosen_batch_size"]

    fig, ax1 = plt.subplots(figsize=(8.5, 5.2))

    # shade infeasible window region (occupancy above 100%), but only when
    # at least one feasible batch exists — otherwise the whole plot would be
    # shaded and the annotation carries the message instead
    first_overflow = next((r["batch"] for r in sweep if not r["window_ok"]),
                          None)
    any_feasible = any(r["window_ok"] for r in sweep)
    if first_overflow is not None and any_feasible:
        ax1.axvspan(first_overflow - 0.5, max(b) + 0.5,
                    color="#d62728", alpha=0.07, zorder=0)
        ax1.text(first_overflow, 0.92, " window\n overrun",
                 transform=ax1.get_xaxis_transform(),
                 color="#d62728", fontsize=8, va="top")

    l1, = ax1.plot(b, wait, "o-", color="#1f77b4",
                   label="worst first-arrival wait (p95)")
    l2 = ax1.axhline(SLO_MS, color="#1f77b4", ls="--", lw=1,
                     label=f"SLO = {SLO_MS:.0f} ms")
    ax1.set_xlabel("preheat batch size b (blocks per idle window)")
    ax1.set_ylabel("first-arrival wait (ms)", color="#1f77b4")
    ax1.tick_params(axis="y", labelcolor="#1f77b4")
    ax1.set_yscale("log")
    ax1.set_ylim(min(min(wait) * 0.5, 1e-5),
                 max(SLO_MS * 3, max(wait) * 3))
    # thin out x ticks when there are many batches
    step = max(1, math.ceil(len(b) / 16))
    ax1.set_xticks(b[::step])

    ax2 = ax1.twinx()
    l3, = ax2.plot(b, occ, "s-", color="#d62728",
                   label="idle-window occupancy")
    l4 = ax2.axhline(100, color="#d62728", ls="--", lw=1,
                     label="window capacity = 100%")
    ax2.set_ylabel("idle-window occupancy (%)", color="#d62728")
    ax2.tick_params(axis="y", labelcolor="#d62728")
    ax2.set_ylim(0, max(120, max(occ) * 1.05))

    if chosen:
        ax1.axvline(chosen, color="green", ls=":", lw=1.5)
        # keep the label inside the axes
        tx = chosen - (max(b) - min(b)) * 0.22 if chosen > 0.8 * max(b) \
            else chosen + 0.3
        ax1.annotate(f"chosen b*={chosen}", xy=(chosen, min(wait)),
                     xytext=(tx, min(wait) * 0.3),
                     color="green", fontsize=9)
    else:
        ax1.annotate("no feasible batch:\nevery block exceeds the window",
                     xy=(b[0], max(wait)), xytext=(b[0], max(wait) * 1.2),
                     color="#d62728", fontsize=9)

    tag = " [EXTRAPOLATED STRESS]" if result["stress_extrapolation"] else ""
    plt.title(f"Tier {name}: M={result['blocks']} blocks, "
              f"n={result['block_cells']} cells, "
              f"W_idle={result['window_ms']:.0f} ms{tag}")
    handles = [l1, l2, l3, l4]
    ax1.legend(handles, [h.get_label() for h in handles], fontsize=8,
               loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return True


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tier", default="all",
                    choices=["S", "M", "L", "X", "all"])
    ap.add_argument("--window", type=float, default=30_000.0,
                    help="idle window W_idle in ms (default 30000)")
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    calib = Calibration.load()
    fit_report = {
        f"{prof}/{stat}": {"a": f.a, "k": f.k,
                           "R2": None if math.isnan(f.r2) else round(f.r2, 4),
                           "measured_up_to_n": f.n_max_measured}
        for (prof, stat), f in sorted(calib.fits.items())
    }

    scenarios = {}
    if args.tier == "all":
        scenarios = {**TIERS, **STRESS}
    elif args.tier == "X":
        scenarios = dict(STRESS)
    else:
        scenarios = {args.tier: TIERS[args.tier]}

    summary = []
    for name, (m, n, profile) in scenarios.items():
        res = run_tier(name, m, n, profile, calib, args.window,
                       stress=(name == "X"))
        write_csv(os.path.join(OUT, f"sweep_{name}.csv"), res["sweep"])
        plot_curves(name, res, os.path.join(OUT, f"curves_{name}.png"))
        summary.append(res)

    with open(os.path.join(OUT, "results.json"), "w") as fh:
        json.dump({"window_ms": args.window, "slo_ms": SLO_MS,
                   "fits": fit_report, "tiers": summary}, fh, indent=2)

    # console report
    print("== power-law fits t=a*n^k (log-log R2) ==")
    for key, v in fit_report.items():
        r2 = "flat(measured)" if isinstance(v["R2"], float) and math.isnan(v["R2"]) else f"{v['R2']}"
        print(f"  {key:18s} a={v['a']:.3e} k={v['k']:.3f} R2={r2}")
    print()
    for r in summary:
        tag = " [EXTRAPOLATED]" if r["stress_extrapolation"] else ""
        print(f"== Tier {r['tier']}{tag}: M={r['blocks']} n={r['block_cells']} "
              f"Q={r['pairs_per_block_Q']} profile={r['profile']} ==")
        print(f"  t_q mean/p95/max = {r['tq_mean_ms']:.4f} / "
              f"{r['tq_p95_ms']:.4f} / {r['tq_max_ms']:.4f} ms ; "
              f"t_hit={r['thit_ms']:.6f} ms")
        print(f"  per-block preheat c_i = {r['per_block_cost_ms']:.1f} ms ; "
              f"B_cap=floor(W/c)={r['B_cap_formula']} ; "
              f"windows needed={r['scheduled_windows']} ; "
              f"first-window batch={r['first_window_batch']}")
        print(f"  chosen batch b*={r['chosen_batch_size']} "
              f"load={r['chosen_window_load_ms']} ms "
              f"worst wait p95={r['chosen_worst_wait_p95_ms']} ms ; "
              f"all-cold SLO pass={r['slo_pass_all_cold']} "
              f"(p95 margin {r['slo_margin_p95_ms']:.2f} ms, "
              f"max margin {r['slo_margin_max_ms']:.2f} ms)")
        if r["extrapolation_warning"]:
            print(f"  !! {r['extrapolation_warning']}")
        if r["dropped"]:
            for d in r["dropped"]:
                print(f"  DROP {d['bid']}: {d['reason']} "
                      f"(cost={d['cost_ms']:.1f} ms, "
                      f"cold p95={d['cold_p95_ms']:.2f} ms, "
                      f"cold max={d['cold_max_ms']:.2f} ms)")
        else:
            print("  dropped blocks: none")
        print()


if __name__ == "__main__":
    main()
