#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tradeoff.py - per-cell marker byte vs count-only comparison.

Two curves decide whether to add the marker:

  localization precision : of the candidate cells a method leaves you with,
                           what fraction are true discrepancy coordinates?
                           P = k / |candidate set|   (recall is 1 for all
                           methods modelled here)
  memory overhead        : extra resident bytes for the localization state.

Methods
  marker-byte : retain a uint8 visited/diff mark per cell -> candidates exact
  block8      : compare 8x8 subtotals -> every cell in a dirty block stays
  scalar-only : compare only W vs V totals -> every cell stays a suspect

Precision for block8 is measured by Monte Carlo injection of k=12 random
discrepancies (fixed RNG seed); the other two are exact by construction.

Outputs CSV + two-panel PNG (precision and memory are separate panels on
purpose: never a dual axis).
"""

import csv
import os
import random

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

K = 12
TRIALS = 400
SEED = 20261126
B = 8

# Validated light-mode palette (dataviz palette slots 1-3 + text gray).
C_BLUE = "#2a78d6"
C_ORANGE = "#eb6834"
C_AQUA = "#1baf7a"
C_GRAY = "#52514e"
INK = "#0b0b0b"
GRID = "#e3e2dc"

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ART = os.path.join(REPO, "artifacts")


def block_candidates(w, h, dirty_blocks):
    total = 0
    nb_x = (w + B - 1) // B
    for bx, by in dirty_blocks:
        x0, y0 = bx * B, by * B
        total += min(B, w - x0) * min(B, h - y0)
    return total, nb_x


def measure_precision(w, h, k, trials, rng):
    n = w * h
    p_scalar = k / n
    cand_marker = k
    # block8 Monte Carlo
    prec_sum = 0.0
    cand_sum = 0.0
    nb_y = (h + B - 1) // B
    for _ in range(trials):
        flat = rng.sample(range(n), k)
        dirty = set()
        for f in flat:
            x, y = f % w, f // w
            dirty.add((x // B, y // B))
        cand, _ = block_candidates(w, h, dirty)
        cand_sum += cand
        prec_sum += k / cand
    return {
        "N": n,
        "p_marker": k / cand_marker,
        "p_block8": prec_sum / trials,
        "cand_block8": cand_sum / trials,
        "p_scalar": p_scalar,
        "cand_scalar": n,
        "mem_marker_u8": n,
        "mem_marker_bits": (n + 7) // 8,
        "mem_comp_int": 4 * n,
        "mem_block8_u32": 4 * ((w + B - 1) // B) * nb_y,
    }


def main():
    os.makedirs(ART, exist_ok=True)
    rng = random.Random(SEED)
    shapes = [(16, 12), (22, 16)]
    for s in (32, 64, 128, 256, 512, 1024):
        shapes.append((s, s))
    rows = [measure_precision(w, h, K, TRIALS, rng) for w, h in shapes]

    csv_path = os.path.join(ART, "reconcile-tradeoff.csv")
    fields = list(rows[0].keys())
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=fields)
        wr.writeheader()
        wr.writerows(rows)

    # ---- console table ----------------------------------------------------
    print("k=%d injected discrepancies, block=%dx%d, trials=%d seed=%d"
          % (K, B, B, TRIALS, SEED))
    hdr = ("grid", "N", "P_marker", "P_block8", "P_scalar",
           "cand_block8", "mem_u8_B", "mem_bit_B", "mem_compint_B")
    print("{:>9} {:>8} {:>8} {:>8} {:>9} {:>11} {:>9} {:>9} {:>12}"
          .format(*hdr))
    for (w, h), r in zip(shapes, rows):
        print("{:>4}x{:<4} {:>8d} {:>8.3f} {:>8.4f} {:>9.2e} {:>11.1f} "
              "{:>9d} {:>9d} {:>12d}".format(
                  w, h, r["N"], r["p_marker"], r["p_block8"], r["p_scalar"],
                  r["cand_block8"], r["mem_marker_u8"],
                  r["mem_marker_bits"], r["mem_comp_int"]))
    print("csv=%s" % csv_path)

    # ---- chart: two panels, shared log-x, never dual axis -----------------
    Ns = [r["N"] for r in rows]
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9.2, 7.6), sharex=True,
        gridspec_kw={"height_ratios": [1.05, 1.0]})
    fig.patch.set_facecolor("#fcfcfb")
    for ax in (ax1, ax2):
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, which="major", color=GRID, linewidth=0.8)
        ax.grid(True, which="minor", color=GRID, linewidth=0.4, alpha=0.6)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.spines["left"].set_color("#c9c8c0")
        ax.spines["bottom"].set_color("#c9c8c0")
        ax.tick_params(colors=INK, labelsize=9)

    # top: precision
    pm = [r["p_marker"] for r in rows]
    pb = [r["p_block8"] for r in rows]
    ps = [r["p_scalar"] for r in rows]
    ax1.plot(Ns, pm, "-o", color=C_BLUE, lw=2, ms=6,
             label="per-cell marker byte (candidate set = exact coords)")
    ax1.plot(Ns, pb, "-s", color=C_AQUA, lw=2, ms=5,
             label="8x8 subtotal comparison (Monte Carlo)")
    ax1.plot(Ns, ps, "-^", color=C_ORANGE, lw=2, ms=6,
             label="scalar totals only (candidate set = every cell)")
    ax1.set_xscale("log")
    ax1.set_yscale("log")
    ax1.set_ylabel("localization precision  k / |candidate set|  (log)",
                   fontsize=10, color=INK)
    ax1.set_ylim(8e-6, 2.0)
    ax1.set_title("Adding one marker byte: localization precision vs memory",
                  fontsize=12.5, color=INK, pad=10, weight="semibold")
    # direct end labels (secondary encoding for the aqua contrast warning)
    x_end = Ns[-1]
    ax1.annotate("marker 1.0", (x_end, 1.0), xytext=(6, -2),
                 textcoords="offset points", color=C_BLUE, fontsize=9,
                 va="center", weight="bold")
    ax1.annotate("block8 ~1/64", (x_end, pb[-1]), xytext=(6, 0),
                 textcoords="offset points", color="#0c7a55", fontsize=9,
                 va="center")
    ax1.annotate("scalar 12/N", (x_end, ps[-1]), xytext=(6, 2),
                 textcoords="offset points", color="#b3471f", fontsize=9,
                 va="center")
    ax1.legend(loc="lower left", fontsize=8.6, frameon=True,
               facecolor="#ffffff", edgecolor=GRID)

    # bottom: memory
    mu8 = [r["mem_marker_u8"] for r in rows]
    mbit = [r["mem_marker_bits"] for r in rows]
    mcomp = [r["mem_comp_int"] for r in rows]
    mblk = [r["mem_block8_u32"] for r in rows]
    ax2.plot(Ns, mu8, "-", color=C_BLUE, lw=2.2,
             label="uint8 marker: N bytes")
    ax2.plot(Ns, mbit, ":", color=C_BLUE, lw=1.8,
             label="bit-packed marker: N/8 bytes")
    ax2.plot(Ns, mblk, "--", color=C_AQUA, lw=1.8,
             label="8x8 block u32 table: N/16 bytes")
    ax2.plot(Ns, mcomp, "-.", color=C_GRAY, lw=1.8,
             label="status quo comp int[] in region.c: 4N bytes")
    ax2.set_xscale("log")
    ax2.set_yscale("log")
    ax2.set_xlabel("grid cells N (log scale)", fontsize=10, color=INK)
    ax2.set_ylabel("extra localization memory, bytes (log)",
                   fontsize=10, color=INK)
    ax2.legend(loc="upper left", fontsize=8.6, frameon=True,
               facecolor="#ffffff", edgecolor=GRID)
    ax2.text(0.985, 0.07,
             "count-only option: 0 extra bytes,\nlocalization precision = 12/N",
             transform=ax2.transAxes, ha="right", va="bottom",
             color="#b3471f", fontsize=9,
             bbox=dict(boxstyle="round,pad=0.35", fc="#fdf1ea",
                       ec=C_ORANGE, lw=0.8, alpha=0.95))

    # project reference points (staggered labels to avoid collision)
    refs = ((192, "repo 16x12 = 192 cells\nmarker 192 B", (12, -34)),
            (352, "case 22x16 = 352 cells\nmarker 352 B", (14, 14)))
    for n_ref, label, off in refs:
        for ax in (ax1, ax2):
            ax.axvline(n_ref, color=C_GRAY, lw=0.9, alpha=0.55,
                       linestyle=(0, (2, 3)))
        ax2.annotate(label, xy=(n_ref, n_ref), xytext=off,
                     textcoords="offset points", fontsize=8.4, color=INK,
                     bbox=dict(boxstyle="round,pad=0.25", fc="#fcfcfb",
                               ec="none", alpha=0.85),
                     arrowprops=dict(arrowstyle="-", color=C_GRAY, lw=0.8))
    ax2.set_ylim(bottom=8)

    fig.tight_layout(rect=(0, 0, 1, 1))
    png_path = os.path.join(ART, "reconcile-tradeoff.png")
    fig.savefig(png_path, dpi=150, facecolor=fig.get_facecolor())
    print("chart=%s" % png_path)

    print("")
    print("decision: for nav-grid scales the uint8 marker costs")
    print("192 B on the repo grid / 352 B on the 340-walkable case /")
    print("1 MiB at N=1,048,576, while scalar-only precision falls to")
    print("12/N. ADD the marker; region.c already retains comp int[] (4N),")
    print("so the marginal cost there is zero - only expose/diff it.")


if __name__ == "__main__":
    main()
