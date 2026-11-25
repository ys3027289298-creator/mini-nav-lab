"""Render the trade-off figure from results/curve.json.

Left panel: undo success rate vs number of fields a fixed-set snapshot stores.
Right panel: snapshot volume vs the same x, with the 30% budget line.

The three data tiers are computed independently; if they agree the line is
drawn once and the agreement is annotated, because three identical lines on
top of each other would be unreadable.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Reference palette, light mode.  Validated with validate_palette.js:
# CVD worst adjacent dE 9.2 (deutan), normal-vision 27.6, all checks pass.
# Aqua is below 3:1 contrast, so every series carries a direct label and the
# numbers also appear in docs/REVERSIBILITY.md as a table.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
INK_3 = "#8a8984"
GRID = "#e5e4df"
S_EMPIRICAL = "#2a78d6"   # slot 1, blue
S_PREDICTED = "#eb6834"   # slot 2, orange
S_CHOSEN = "#1baf7a"      # slot 3, aqua
BUDGET = "#e34948"        # red, used only for the limit rule

TIERS = ("small", "medium", "large")
TOLERANCE = 0.005  # 0.5 percentage points; curves must agree at least this well


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _agree(data: dict, curve: str) -> float:
    """Max spread of a y-value across tiers at each shared x."""
    xs = [p["fields_stored"] for p in data["large"]["curves"][curve]]
    worst = 0.0
    for x in xs:
        values = []
        for tier in TIERS:
            for p in data[tier]["curves"][curve]:
                if p["fields_stored"] == x:
                    values.append(p["undo_success_rate"])
        if values:
            worst = max(worst, max(values) - min(values))
    return worst


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parent.parent)
    args = parser.parse_args(argv)
    data = _load(args.root / "results" / "curve.json")
    out_dir = args.root / "results" / "fig"
    out_dir.mkdir(parents=True, exist_ok=True)

    chosen = dict(data["large"]["delta_per_record"])
    full = data["large"]["full"]
    # Mean stored fields for the per-record scheme comes from the acceptance
    # run (evaluate.py), which is the only place it is counted.
    metrics = _load(args.root / "results" / "metrics.json")
    chosen["mean_stored_fields"] = \
        metrics["tiers"]["large"]["chosen"]["mean_stored_fields"]
    spread = _agree(data, "empirical")

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.edgecolor": GRID,
        "axes.linewidth": 0.8,
        "text.color": INK,
        "axes.labelcolor": INK_2,
        "xtick.color": INK_2,
        "ytick.color": INK_2,
    })

    fig, (ax_r, ax_s) = plt.subplots(
        1, 2, figsize=(11.2, 4.5), facecolor=SURFACE, dpi=200)
    for ax in (ax_r, ax_s):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.7, zorder=0)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    # ---------------------------------------------------------- reliability
    emp = data["large"]["curves"]["empirical"]
    pred = data["large"]["curves"]["predicted"]
    xe = [p["fields_stored"] for p in emp]
    ye = [p["undo_success_rate"] * 100 for p in emp]
    xp = [p["fields_stored"] for p in pred]
    yp = [p["undo_success_rate"] * 100 for p in pred]

    ax_r.plot(xe, ye, color=S_EMPIRICAL, linewidth=2, zorder=3,
              marker="o", markersize=4.5, markerfacecolor=S_EMPIRICAL,
              markeredgecolor=SURFACE, markeredgewidth=1.2)
    ax_r.plot(xp, yp, color=S_PREDICTED, linewidth=2, zorder=3,
              marker="s", markersize=4.5, markerfacecolor=S_PREDICTED,
              markeredgecolor=SURFACE, markeredgewidth=1.2)
    ax_r.axhline(100, color=INK_3, linewidth=0.9, linestyle=(0, (4, 3)), zorder=1)

    # Chosen per-record scheme: exact at a fraction of the fields.
    ax_r.scatter([chosen["mean_stored_fields"]], [100], s=80, zorder=6,
                 color=S_CHOSEN, edgecolor=SURFACE, linewidth=1.4)
    ax_r.annotate(
        f"per-record delta: 100%\nat {chosen['mean_stored_fields']:.1f} fields avg",
        xy=(chosen["mean_stored_fields"], 100), xytext=(13, 83),
        fontsize=8.5, color=INK, va="top",
        arrowprops=dict(arrowstyle="-", color=S_CHOSEN, linewidth=1.0))

    ax_r.annotate("layout order:\nchecksum comes last,\nso 0% until field 56",
                  xy=(30, 0.4), xytext=(8, 40), fontsize=8.5, color=S_PREDICTED,
                  va="top",
                  arrowprops=dict(arrowstyle="-", color=S_PREDICTED,
                                  linewidth=0.9))
    ax_r.annotate("empirical order:\ncheap padding slots first,\nexact only near 51-53",
                  xy=(xe[9], ye[9]), xytext=(27, 72), fontsize=8.5,
                  color=S_EMPIRICAL, va="top",
                  arrowprops=dict(arrowstyle="-", color=S_EMPIRICAL,
                                  linewidth=0.9))

    ax_r.set_xlabel("fields a fixed-set snapshot stores")
    ax_r.set_ylabel("undo success rate  (%)")
    ax_r.set_title("Reliability: fixed sets are exact near 56;\nthe delta is exact at ~2",
                   loc="left", fontsize=10.5, color=INK, pad=10)
    ax_r.set_xlim(0, 59)
    ax_r.set_ylim(-4, 112)
    ax_r.set_yticks([0, 25, 50, 75, 100])

    # ---------------------------------------------------------- volume
    sizes = [p["fields_stored"] for p in emp]
    vol = [p["snapshot_over_old"] * 100 for p in emp]
    ax_s.plot(sizes, vol, color=S_EMPIRICAL, linewidth=2, zorder=3,
              marker="o", markersize=4.5, markerfacecolor=S_EMPIRICAL,
              markeredgecolor=SURFACE, markeredgewidth=1.2)
    ax_s.axhline(30, color=BUDGET, linewidth=1.4, zorder=2)
    ax_s.text(38, 32, "budget 30% = 68.4 B", color=BUDGET, fontsize=8.5)

    ax_s.scatter([chosen["mean_stored_fields"]],
                 [chosen["snapshot_over_old"] * 100], s=80, zorder=6,
                 color=S_CHOSEN, edgecolor=SURFACE, linewidth=1.4)
    ax_s.annotate(
        f"per-record delta\n{chosen['snapshot_over_old'] * 100:.1f}% "
        f"({chosen['mean_snapshot_bytes']:.1f} B)",
        xy=(chosen["mean_stored_fields"], chosen["snapshot_over_old"] * 100),
        xytext=(13, 78), fontsize=8.5, color=INK, va="top",
        arrowprops=dict(arrowstyle="-", color=S_CHOSEN, linewidth=1.0))

    ax_s.scatter([57], [full["snapshot_over_old"] * 100], s=60, zorder=6,
                 color="#8a8984", edgecolor=SURFACE, linewidth=1.2)
    ax_s.annotate(f"full copy {full['snapshot_over_old'] * 100:.0f}%",
                  xy=(57, full["snapshot_over_old"] * 100), xytext=(30, 122),
                  fontsize=8.5, color=INK_2, va="top",
                  arrowprops=dict(arrowstyle="-", color=INK_3, linewidth=0.9))

    ax_s.set_xlabel("fields a fixed-set snapshot stores")
    ax_s.set_ylabel("snapshot volume  (% of 228 B record)")
    ax_s.set_title("Cost: a full copy is larger\nthan the record it protects",
                   loc="left", fontsize=10.5, color=INK, pad=10)
    ax_s.set_xlim(0, 59)
    ax_s.set_ylim(0, 132)
    ax_s.set_yticks([0, 30, 60, 90, 120])

    # ---------------------------------------------------------- legend
    from matplotlib.lines import Line2D
    handles = [
        Line2D([], [], color=S_EMPIRICAL, linewidth=2, marker="o",
               markersize=4.5, label="empirical field order"),
        Line2D([], [], color=S_PREDICTED, linewidth=2, marker="s",
               markersize=4.5, label="layout (struct) field order"),
        Line2D([], [], color=S_CHOSEN, linewidth=0, marker="o", markersize=7,
               label="chosen: per-record delta"),
        Line2D([], [], color=BUDGET, linewidth=1.4, label="30% size budget"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False,
               bbox_to_anchor=(0.5, -0.02), fontsize=8.8, labelcolor=INK_2)

    fig.suptitle(
        "Undo reliability vs snapshot size  ·  NAV0 -> NAV1, 228-byte path records",
        fontsize=12, color=INK, x=0.012, ha="left", y=0.985)
    fig.text(0.012, 0.925,
             f"small / medium / large tiers agree within {spread * 100:.2f} pp; "
             "only `large` is drawn",
             fontsize=8.5, color=INK_3, ha="left")

    fig.tight_layout(rect=(0, 0.06, 1, 0.90))
    out = out_dir / "tradeoff.png"
    fig.savefig(out, facecolor=SURFACE, bbox_inches="tight")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
