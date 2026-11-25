#!/usr/bin/env python3
"""Probe: which float32 terrain-cost scheme makes accumulation order matter?

Pure stdlib. Every addition is rounded to IEEE-754 binary32 via struct so the
result matches C `float` (round-to-nearest-even). We measure how often two
routes with equal exact (integer) cost get different float32 sums purely
because of summation order or cost multiset.
"""
import random
import struct

def f32(x):
    """round a python float to binary32"""
    return struct.unpack("f", struct.pack("f", x))[0]

def f32_add(a, b):
    return f32(a + b)

def ulp_gap(a, b):
    """gap between two float32 values in ulps of the larger magnitude"""
    if a == b:
        return 0.0
    ia = struct.unpack("I", struct.pack("f", a))[0]
    ib = struct.unpack("I", struct.pack("f", b))[0]
    return abs(int(ia) - int(ib))  # integer bit distance == ulp distance

def fsum(seq):
    s = f32(0.0)
    for v in seq:
        s = f32_add(s, f32(v))
    return s

SCHEMES = {
    "A_[0.9,1.0,1.1,1.2]": [0.9, 1.0, 1.1, 1.2],
    "B_[0.1,1.0,1.9]":     [0.1, 1.0, 1.9],
    "C_[0.01,1.0]":        [0.01, 1.0],
    "D_[0.1,0.2,1.0]":     [0.1, 0.2, 1.0],
}
LENGTHS = [10, 25, 50, 100, 220]
TRIALS = 4000

rng = random.Random(20261126)
print(f"{'scheme':<22}{'L':>5}{'order_flip%':>13}{'multiset_flip%':>17}{'max_ulp':>9}")
for name, costs in SCHEMES.items():
    int_costs = [round(c * 100) for c in costs]
    f_costs = [f32(c) for c in costs]
    for L in LENGTHS:
        order_flips = multi_flips = 0
        max_ulp = 0.0
        for _ in range(TRIALS):
            idx = [rng.randrange(len(costs)) for _ in range(L)]
            seq = [f_costs[i] for i in idx]
            isum = sum(int_costs[i] for i in idx)

            # kind 1: same multiset, reversed accumulation order
            s1 = fsum(seq)
            s2 = fsum(seq[::-1])
            if s1 != s2:
                order_flips += 1
                max_ulp = max(max_ulp, ulp_gap(s1, s2))

            # kind 2: different multiset, same integer sum
            for _try in range(200):
                alt = [rng.randrange(len(costs)) for _ in range(L)]
                if sum(int_costs[i] for i in alt) == isum:
                    s3 = fsum([f_costs[i] for i in alt])
                    if s3 != s1:
                        multi_flips += 1
                        max_ulp = max(max_ulp, ulp_gap(s1, s3))
                    break

        print(f"{name:<22}{L:>5}{100*order_flips/TRIALS:>12.2f}%"
              f"{100*multi_flips/TRIALS:>16.2f}%{max_ulp:>9.1f}")
    print()
