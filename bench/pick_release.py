#!/usr/bin/env python3
"""pick_release.py: choose a release policy from the peak-memory and hit-rate
curves together.

The two curves pull in opposite directions. Holding longer buys hits; it costs
resident bytes. A policy is worth choosing when the memory it saves is cheap in
hit rate, i.e. when the point sits on the knee of the trade-off curve.

Reported per policy:
  peak_bytes   high-water mark of cache payload
  hit_rate     share of queries served from cache
  evict_ratio  bytes saved per point of hit rate given up, against the
               unbounded baseline -- the price of releasing early

Usage: pick_release.py --csv measurements/cache.csv
"""
import argparse
import csv
import sys
from collections import defaultdict


def load(path):
    groups = defaultdict(list)
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                groups[r["hot_bias"]].append({
                    "policy": r["policy"], "cap": int(r["cap"]), "ttl": int(r["ttl"]),
                    "peak": int(r["peak_bytes"]), "hit": float(r["hit_rate"]),
                    "hits": int(r["hits"]), "misses": int(r["misses"]),
                })
            except (KeyError, ValueError):
                continue
    if not groups:
        sys.exit(f"no rows in {path}")
    return groups


def analyze(group, bias):
    base = max(group, key=lambda p: (p["peak"], p["hit"]))
    print(f"hot_bias={bias}%  ({group[0]['hits'] + group[0]['misses']} queries)")
    print(f"  baseline (keep everything): peak={base['peak']:,} B  "
          f"hit={base['hit']:.3f}")
    print(f"  {'policy':<17} {'cap':>5} {'ttl':>4} {'peak':>10} {'hit':>7} "
          f"{'d_peak':>9} {'d_hit':>8} {'B/hit-pt':>10}")
    rows = []
    for p in sorted(group, key=lambda p: p["peak"]):
        d_peak = base["peak"] - p["peak"]
        d_hit = base["hit"] - p["hit"]
        price = d_peak / (d_hit * 100) if d_hit > 0.0005 else float("inf")
        rows.append({**p, "d_peak": d_peak, "d_hit": d_hit, "price": price})
        price_s = "  inf" if price == float("inf") else f"{price:>10,.0f}"
        print(f"  {p['policy']:<17} {p['cap']:>5} {p['ttl']:>4} "
              f"{p['peak']:>10,} {p['hit']:>7.3f} {d_peak:>9,} "
              f"{d_hit:>8.3f} {price_s}")

    # knee: the point that gives up the most bytes for the least hit rate.
    # Restricted to policies that keep at least half the baseline hit rate --
    # below that a release policy is not "saving memory", it is disabling cache.
    viable = [r for r in rows if r["hit"] >= 0.5 * base["hit"] and r["d_peak"] > 0]
    if not viable:
        print("  -> no policy keeps >=50% of baseline hits while releasing "
              "memory; caching does not pay for itself at this locality\n")
        return
    knee = max(viable, key=lambda r: r["d_peak"] / max(r["d_hit"] * 100, 1e-9))
    print(f"  -> knee: {knee['policy']}  frees {knee['d_peak']:,} B "
          f"for -{knee['d_hit'] * 100:.1f} hit points")
    print(f"     ({knee['price']:,.0f} bytes freed per hit point given up)")
    print()

    # TTL half-life reading: with reuse inter-arrival of about 1/(1-hit) rounds,
    # a TTL shorter than that is guaranteed to miss the reuses it paid to keep.
    if base["hit"] > 0:
        mean_gap = 1.0 / max(base["hit"], 1e-6)
        print(f"  reuse gap implied by baseline hit rate: ~{mean_gap:.1f} "
              f"queries; a TTL under that cannot serve them")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--csv", default="measurements/cache.csv")
    a = ap.parse_args()
    for bias, group in sorted(load(a.csv).items()):
        analyze(group, bias)


if __name__ == "__main__":
    main()
