#!/usr/bin/env python3
"""test_analyze.py: the analyzer must recover the truth it is given.

Each case synthesises a memory curve with a known shape and known per-round
residual, then asserts the analyzer names the shape and the slope correctly.
Run: python3 bench/test_analyze.py
"""
import unittest

import numpy as np

import analyze


def series(kind, base=4000.0, per_round=2880.0, rounds=12, tau=4.0, noise=0.0,
           seed=7):
    n = np.arange(1, rounds + 1, dtype=float)
    if kind == "flat":
        y = np.full_like(n, base)
    elif kind == "linear":
        y = base + per_round * (n - 1)
    elif kind == "log":
        y = base + per_round * np.log1p(n - 1)
    elif kind == "saturating":
        y = base + per_round * (1.0 - np.exp(-(n - 1) / tau))
    else:
        raise ValueError(kind)
    if noise:
        rng = np.random.default_rng(seed)
        y = y + rng.normal(0, noise, size=y.shape)
    return n, y


class ShapeTest(unittest.TestCase):
    def test_flat(self):
        n, y = series("flat")
        fits = analyze.fit_all(n, y)
        v, *_ = analyze.shape_verdict(fits, n, y)
        self.assertTrue(v.startswith("flat"), v)

    def test_linear(self):
        n, y = series("linear")
        fits = analyze.fit_all(n, y)
        v, slope, half, _, _ = analyze.shape_verdict(fits, n, y)
        self.assertTrue(v.startswith("linear"), v)
        self.assertAlmostEqual(slope, 2880.0, delta=1.0)
        self.assertGreater(slope - half, 0)

    def test_saturating(self):
        n, y = series("saturating", tau=2.0, rounds=20)
        fits = analyze.fit_all(n, y)
        v, *_ = analyze.shape_verdict(fits, n, y)
        self.assertTrue(v.startswith("saturating"), v)

    def test_log(self):
        n, y = series("log", rounds=40)
        fits = analyze.fit_all(n, y)
        v, *_ = analyze.shape_verdict(fits, n, y)
        self.assertFalse(v.startswith("linear"), v)

    def test_noisy_linear_is_recovered_or_flagged_ambiguous(self):
        """Noise can make one extra parameter pay for itself; the analyzer must
        then either call it linear or refuse to certify a slope -- never call a
        straight line a leak that slow in a way the half-slope test rejects."""
        n, y = series("linear", noise=300.0, rounds=20)
        fits = analyze.fit_all(n, y)
        v, slope, half, early, late = analyze.shape_verdict(fits, n, y)
        self.assertTrue(v.startswith(("linear", "straight-ish")), v)
        if v.startswith("straight-ish"):
            # ambiguity is only acceptable when the halves agree (no curve)
            self.assertLess(abs(early - late), 0.5 * early)
        else:
            self.assertAlmostEqual(slope, 2880.0, delta=200.0)

    def test_noise_does_not_invent_a_leak(self):
        n, y = series("flat", noise=300.0, rounds=30)
        fits = analyze.fit_all(n, y)
        v, slope, half, _, _ = analyze.shape_verdict(fits, n, y)
        self.assertTrue(v.startswith("flat"), v)
        self.assertLessEqual(slope + half, 300.0)  # cannot certify a residual


class ControlTest(unittest.TestCase):
    """The three injected control shapes, read from generated rows."""

    def rows(self, kind, per_round_bytes, entry_bytes=72, rounds=12, tau=4.0):
        per_round_entries = per_round_bytes / entry_bytes
        out = []
        for r in range(1, rounds + 1):
            if kind == "linear":
                b = per_round_bytes * r
                k = per_round_entries * r
            elif kind == "flat":
                b = per_round_bytes
                k = per_round_entries
            elif kind == "saturating":
                frac = 1.0 - np.exp(-(r - 1) / tau)
                b = per_round_bytes * frac
                k = per_round_entries * frac
            out.append((r, b, k, 0.0))
        return out

    def test_recovers_entries_from_slope(self):
        rows = self.rows("linear", 2880.0, 72.0)
        res = analyze.analyze_run(rows, "synthetic-leak", 72.0)
        self.assertTrue(res["verdict"].startswith("linear"), res["verdict"])
        self.assertAlmostEqual(res["residual"] / 72.0, 40.0, delta=0.5)

    def test_flat_has_no_residual(self):
        rows = self.rows("flat", 2880.0, 72.0)
        res = analyze.analyze_run(rows, "synthetic-cache", 72.0)
        self.assertIsNone(res["residual"])

    def test_saturating_not_called_a_leak(self):
        rows = self.rows("saturating", 2880.0, 72.0, rounds=30, tau=3.0)
        res = analyze.analyze_run(rows, "synthetic-decay", 72.0)
        self.assertFalse(res["verdict"].startswith("linear"), res["verdict"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
