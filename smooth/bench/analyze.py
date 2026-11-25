#!/usr/bin/env python3
"""Analyze smooth_bench / smooth_sweep CSVs: identity checks, inverse
solving of probe counts from reduction, and horizon trade-off curves."""
import csv
import sys
import os

ART = "/home/ys/feedback/uidreczz28dp8PAzerL/artifacts"


def load_bench(tag):
    rows = []
    with open(os.path.join(ART, f"smooth_bench_{tag}.csv")) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def load_sweep(tag):
    rows = []
    with open(os.path.join(ART, f"smooth_sweep_{tag}.csv")) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def main(tag):
    rows = load_bench(tag)
    print(f"################ bench {tag} ################")
    by_size = {}
    for r in rows:
        by_size.setdefault(r["size"], {})[r["version"]] = r
    for size, v in by_size.items():
        a, b = v["A"], v["B"]
        n = int(a["raw_n"])
        ma, mb = int(a["out_m"]), int(b["out_m"])
        pa, pb = int(a["los_calls"]), int(b["los_calls"])
        ra, rb = float(a["reduction"]), float(b["reduction"])
        print(f"\n[{size}] n={n}")
        print(f"  A: m={ma} r={ra:.4f} probes={pa}  identity n+mA-3={n+ma-3} match={pa==n+ma-3}")
        print(f"  B: m={mb} r={rb:.4f} probes={pb}")
        # inverse solve: required probes if reduction r is the only known number
        m_from_r_a = round(n * (1 - ra))
        implied_p_a = n + m_from_r_a - 3
        print(f"  inverse from r_A: m=n(1-r)={m_from_r_a}, P_A=n+m-3={implied_p_a} (actual {pa})")
        # A's probes as fraction of all pairs n(n-1)/2
        pairs = n * (n - 1) / 2
        print(f"  probe share of all pairs: A={pa/pairs:.4f} B={pb/pairs:.4f}")
        print(f"  avg reach: A={float(a['avg_reach']):.3f} B={float(b['avg_reach']):.3f}")
        print(f"  min clearance: A={float(a['min_clearance']):.3f} B={float(b['min_clearance']):.3f}")

    sw = load_sweep(tag)
    print(f"\n################ horizon sweep {tag} ################")
    cur = None
    for r in sw:
        if r["size"] != cur:
            cur = r["size"]
            print(f"\n[{cur}] n={by_size[cur]['A']['raw_n']}  "
                  f"full-B r={by_size[cur]['B']['reduction']} "
                  f"clear={by_size[cur]['B']['min_clearance']}")
            print("  S      m      r       probes   clearance")
        print(f"  {int(r['horizon_S']):>5} {int(r['out_m']):>5}  {float(r['reduction']):.4f}  "
              f"{int(r['los_calls']):>7}  {float(r['min_clearance']):.3f}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "random40_strict")
