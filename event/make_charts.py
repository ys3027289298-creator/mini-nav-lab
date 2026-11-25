#!/usr/bin/env python3
"""Render the two decision curves from artifacts/ CSVs.

Outputs:
  artifacts/loss_rate_curves.png       loss vs smoothing strength + counts
  artifacts/query_overhead_curves.png  query-cost scaling + E=8 budget gate
Palette/mark choices follow the dataviz skill reference palette (light).
"""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
ART = ROOT / "artifacts"

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
GOOD_TXT = "#006300"
BAD = "#d03b3b"

C_IDX = "#2a78d6"      # blue
C_QMAP = "#eb6834"     # orange
C_CURSOR = "#1baf7a"   # aqua
C_BSEARCH = "#eda100"  # yellow
C_SCAN = "#e87ba4"     # magenta

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "system-ui", "sans-serif"],
    "font.size": 10,
    "axes.facecolor": SURFACE,
    "figure.facecolor": SURFACE,
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": INK,
    "text.color": INK,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.axisbelow": True,
})


def read_csv(name: str) -> list[dict]:
    with (ART / name).open() as fh:
        return list(csv.DictReader(fh))


def loss_figure() -> None:
    law = read_csv("loss_curve.csv")
    levels = read_csv("official_levels.csv")

    sig = np.array([float(r["sigma"]) for r in law])
    rho = np.array([float(r["curated_loss"]) for r in law])
    order = np.argsort(sig)
    sig, rho = sig[order], rho[order]

    lvl = [(r["level"].split("_")[0], float(r["sigma"]),
            float(r["loss_rate"]), int(r["fired"]), int(r["total"]))
           for r in levels]

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12.6, 5.3), dpi=160)

    # ---- left: loss rate vs strength ----
    ax.plot([0, 1], [0, 1], "--", color=MUTED, lw=1.6, zorder=1,
            label="proportional law  ρ = σ  (uniform anchors)")
    ax.plot(sig, rho, "-", color=C_QMAP, lw=1.8, alpha=0.55, zorder=2)
    ax.scatter(sig, rho, s=42, color=C_QMAP, edgecolor=SURFACE,
               linewidth=1.3, zorder=3,
               label="ordinal binding, 8 named events (cap sweep)")

    ls = np.array([x[1] for x in lvl])
    lr = np.array([x[2] for x in lvl])
    ax.scatter(ls, ls, marker="s", s=70, color=C_IDX, edgecolor=SURFACE,
               linewidth=1.4, zorder=4,
               label="dense binding, one event / point  (ρ = σ exactly)")
    ax.scatter(ls, lr, marker="o", s=118, color=C_QMAP, edgecolor=INK,
               linewidth=1.1, zorder=5)

    offsets = {"S1": (12, 2), "S2": (10, -18), "S3": (-10, -20)}
    align = {"S1": ("left", "center"), "S2": ("left", "center"),
             "S3": ("right", "center")}
    for tag, s, r, fired, total in lvl:
        dx, dy = offsets[tag]
        ha, va = align[tag]
        lost = total - fired
        ax.annotate(f"{tag}: {lost}/{total} missed", (s, r),
                    xytext=(dx, dy), textcoords="offset points",
                    fontsize=9.5, fontweight="bold", color=INK,
                    ha=ha, va=va,
                    bbox=dict(boxstyle="round,pad=0.25", fc=SURFACE,
                              ec="#c3c2b7", lw=0.8))

    ax.plot([0, ls[0]], [0, 0], color=C_CURSOR, lw=2.6, zorder=3)
    ax.scatter(np.r_[0.0, ls], np.zeros(4), marker="D", s=64,
               color=C_CURSOR, edgecolor=SURFACE, linewidth=1.4, zorder=6,
               label="position-interval binding (fix)   ρ = 0")
    ax.set_xlim(0, 1.0)
    ax.set_ylim(-0.04, 1.04)
    ax.set_xlabel("smoothing strength   σ = deleted points / original N")
    ax.set_ylabel("event loss rate   ρ = missed events / bound events")
    ax.set_title("Loss rate rises with smoothing strength",
                 fontweight="bold", loc="left")
    ax.legend(loc="upper left", fontsize=8.8, framealpha=0.94,
               edgecolor="#c3c2b7")

    # ---- right: trigger counts ----
    cats = ["Unsmoothed", "S1 light", "S2 normal", "S3 heavy"]
    idx_f = [8, 5, 3, 2]
    iv_f = [8, 8, 8, 8]
    x = np.arange(4)
    w = 0.38
    b1 = bx.bar(x - w / 2, idx_f, w, color=C_QMAP, edgecolor=SURFACE,
                linewidth=1.2, label="ordinal index (buggy)")
    b2 = bx.bar(x + w / 2, iv_f, w, color=C_CURSOR, edgecolor=SURFACE,
                linewidth=1.2, label="position interval / qmap (fix)")
    bx.axhline(8, color=MUTED, ls=":", lw=1.6, zorder=1,
               label="unsmoothed baseline = 8")
    for rect, v in zip(b1, idx_f):
        pct = v / 8 * 100
        pct_txt = "100%" if v == 8 else f"{pct:.1f}%"
        bx.text(rect.get_x() + rect.get_width() / 2, v + 0.12,
                f"{v}/8\n{pct_txt}", ha="center", va="bottom",
                fontsize=8.6, fontweight="bold", color=INK)
    for rect, v in zip(b2, iv_f):
        bx.text(rect.get_x() + rect.get_width() / 2, v + 0.12,
                f"{v}/8\n100%", ha="center", va="bottom",
                fontsize=8.6, fontweight="bold", color=GOOD_TXT)
    bx.set_xticks(x)
    bx.set_xticklabels(cats)
    bx.set_ylim(0, 10.4)
    bx.set_ylabel("animation events triggered (of 8)")
    bx.set_title("Fix matches the unsmoothed count at every strength",
                 fontweight="bold", loc="left")
    bx.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3,
              frameon=False, fontsize=8.8, columnspacing=1.1)

    fig.tight_layout()
    out = ART / "loss_rate_curves.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)


def overhead_figure() -> None:
    rows = read_csv("overhead_curve.csv")
    methods = ["idx", "qmap", "cursor", "bsearch", "scan"]
    labels = {
        "idx": "idx  direct ordinal load",
        "qmap": "qmap  position-interval table (chosen fix)",
        "cursor": "cursor  sorted interval sweep",
        "bsearch": "bsearch  interval lookup",
        "scan": "scan  linear interval test",
    }
    colors = {"idx": C_IDX, "qmap": C_QMAP, "cursor": C_CURSOR,
              "bsearch": C_BSEARCH, "scan": C_SCAN}
    markers = {"idx": "o", "qmap": "s", "cursor": "^",
               "bsearch": "D", "scan": "v"}
    styles = {"idx": "-", "qmap": "-", "cursor": "--",
              "bsearch": "-.", "scan": ":"}

    table: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for meth in methods:
        rr = [r for r in rows if r["method"] == meth]
        rr.sort(key=lambda r: int(r["E"]))
        table[meth] = (np.array([int(r["E"]) for r in rr]),
                       np.array([float(r["ns_median"]) for r in rr]))

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12.8, 5.3), dpi=160,
                                 gridspec_kw={"width_ratios": [1.35, 1.0]})

    # ---- left: scaling curves ----
    for meth in methods:
        e, ns = table[meth]
        lw = 2.8 if meth == "qmap" else 1.9
        ax.plot(e, ns, styles[meth], color=colors[meth], lw=lw,
                marker=markers[meth], ms=7, markeredgecolor=SURFACE,
                markeredgewidth=1.3, label=labels[meth], zorder=4)
    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    es = table["idx"][0]
    ax.set_xticks(es)
    ax.set_xticklabels([str(int(v)) for v in es])
    ax.axvline(8, color=MUTED, ls=":", lw=1.5, zorder=1)
    for t in ax.get_xticklabels():
        if t.get_text() == "8":
            t.set_fontweight("bold")
            t.set_color(C_QMAP)
    ax.set_xlabel("bound event count   E      (deployment count E = 8 marked)")
    ax.set_ylabel("median ns / query  (log scale)")
    ax.set_title("Query-cost scaling: direct maps stay flat",
                 fontweight="bold", loc="left")
    ax.legend(loc="upper left", fontsize=8.8, framealpha=0.94,
              edgecolor="#c3c2b7")

    # ---- right: normalized overhead at E=8 ----
    e8 = {m: float(np.interp(8, table[m][0], table[m][1])) for m in methods}
    base = e8["idx"]
    ratios = [e8[m] / base for m in methods]
    xp = np.arange(len(methods))
    bars = bx.bar(xp, ratios, 0.62,
                  color=[colors[m] for m in methods],
                  edgecolor=SURFACE, linewidth=1.3, zorder=3)
    bx.axhline(1.0, color="#c3c2b7", lw=1.4, zorder=2)
    bx.axhline(1.10, color=BAD, ls="--", lw=1.8, zorder=2)
    bx.text(0.995, 1.115, "+10% budget",
            transform=bx.get_yaxis_transform(), ha="right", va="bottom",
            color=BAD, fontsize=9.5, fontweight="bold", zorder=6,
            bbox=dict(boxstyle="round,pad=0.18", fc=SURFACE, ec="none",
                      alpha=0.85))
    bx.set_yscale("log")
    bx.set_ylim(0.7, max(ratios) * 1.7)
    for rect, m, r in zip(bars, methods, ratios):
        ovh = (r - 1) * 100
        txt = "0% (base)" if m == "idx" else f"{ovh:+.0f}%"
        col = GOOD_TXT if m == "qmap" else INK
        weight = "bold" if m == "qmap" else "normal"
        bx.text(rect.get_x() + rect.get_width() / 2, r * 1.17,
                f"{r:.2f}×\n{txt}", ha="center", va="bottom",
                fontsize=8.8, color=col, fontweight=weight)
    bx.set_xticks(xp)
    bx.set_xticklabels(["idx\nordinal", "qmap\nposition", "cursor\nsweep",
                        "bsearch", "scan"], fontsize=8.8)
    bx.set_ylabel("cost ratio vs idx  (log)")
    bx.set_title("Per-query overhead at E = 8",
                 fontweight="bold", loc="left")

    fig.tight_layout()
    out = ART / "query_overhead_curves.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)
    print("E8 ratios:", {m: round(e8[m] / base, 3) for m in methods})


if __name__ == "__main__":
    loss_figure()
    overhead_figure()
