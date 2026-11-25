#!/usr/bin/env python3
"""Run the tube-boundary sweep and emit curves + acceptance report.

The simulation itself lives in tube/src/tube.c so `make test` and this report
share one implementation.  This script rebuilds the CSV, selects the smallest
radius that clears all three density tiers, and renders the two trade-off
curves: boundary violations and detour overhead.
"""
from __future__ import annotations

import csv
import json
import subprocess
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

ROOT = Path(__file__).resolve().parent.parent
TUBE_DIR = ROOT / "tube"
ART = ROOT / "artifacts"
CSV_PATH = ART / "tube_curves.csv"

DENSITY_NAME = {0: "low", 1: "medium", 2: "high"}
COLORS = {0: "#2a78d6", 1: "#eb6834", 2: "#1baf7a"}
MARKERS = {0: "o", 1: "s", 2: "^"}
SURFACE = "#fcfcfb"
GRID = "#e1e0d9"
INK = "#0b0b0b"
MUTED = "#52514e"
CRITICAL = "#d03b3b"


def run_c_sweep() -> list[dict[str, float | int | str]]:
    subprocess.run(["make", "csv"], cwd=TUBE_DIR, check=True,
                   stdout=subprocess.DEVNULL)
    with CSV_PATH.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    typed = []
    for row in rows:
        out: dict[str, float | int | str] = {}
        for key, value in row.items():
            if key in {"density", "n_clusters", "n_peds",
                       "demand_violations", "outside_samples",
                       "wall_samples"}:
                out[key] = int(value)
            else:
                out[key] = float(value)
        typed.append(out)
    return typed


def select_radius(rows) -> tuple[float, list[dict]]:
    by_r = defaultdict(list)
    for row in rows:
        by_r[row["R"]].append(row)

    feasible = []
    for R in sorted(by_r):
        group = by_r[R]
        if len(group) != 3:
            continue
        if all(r["demand_violations"] == 0
               and r["outside_samples"] == 0
               and r["detour_pct"] <= 15.0
               for r in group):
            feasible.append((R, group))
    if not feasible:
        raise RuntimeError("no tube radius satisfies the acceptance bounds")
    return feasible[0]


def plot(rows, r_star: float, path: Path) -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans, sans-serif",
        "axes.edgecolor": "#c3c2b7",
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "axes.titlecolor": INK,
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
    })
    fig, (ax_v, ax_d) = plt.subplots(1, 2, figsize=(12.2, 4.8))

    for d in (0, 1, 2):
        xs = [r["R"] for r in rows if r["density"] == d]
        violations = [r["demand_violations"] for r in rows
                      if r["density"] == d]
        detour = [r["detour_pct"] for r in rows if r["density"] == d]
        label = f"{DENSITY_NAME[d]} density"

        ax_v.plot(xs, violations, color=COLORS[d], marker=MARKERS[d],
                  linewidth=2.1, markersize=5.5, label=label)
        ax_d.plot(xs, detour, color=COLORS[d], marker=MARKERS[d],
                  linewidth=2.1, markersize=5.5, label=label)
        ax_d.annotate(f"{detour[-1]:.1f}%", xy=(xs[-1], detour[-1]),
                      xytext=(5, 0), textcoords="offset points",
                      va="center", fontsize=9, color=MUTED)

    for ax, title, ylabel in (
        (ax_v, "Boundary exceedance", "exceedance episodes per run"),
        (ax_d, "Detour overhead", "extra path length (%)"),
    ):
        ax.axvline(r_star, color=MUTED, linestyle="--", linewidth=1.4)
        ax.set_title(title, fontsize=13, fontweight="bold", pad=10)
        ax.set_xlabel("tube radius R (m)")
        ax.set_ylabel(ylabel)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        ax.xaxis.set_major_locator(MultipleLocator(0.2))
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)

    ax_v.set_ylim(bottom=-0.25, top=8.6)
    ax_v.yaxis.set_major_locator(MultipleLocator(1))
    ax_v.text(r_star + 0.02, 8.25, f"selected R={r_star:.2f} m",
              color=MUTED, fontsize=9, va="top")
    ax_v.legend(frameon=False, fontsize=9, loc="upper right")

    ax_d.axhline(15.0, color=CRITICAL, linestyle=":", linewidth=1.8)
    ax_d.text(1.61, 15.18, "15% cap", color=CRITICAL, fontsize=10,
              fontweight="bold", ha="right", va="bottom")
    ax_d.set_ylim(0, 16.6)
    ax_d.text(r_star + 0.02, 16.15, f"selected R={r_star:.2f} m",
              color=MUTED, fontsize=9, va="top")

    fig.suptitle("Tube radius calibration: zero exceedance first, then shortest detour",
                 fontsize=14, fontweight="bold", x=0.055, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=170, bbox_inches="tight")
    plt.close(fig)


def write_report(rows, r_star: float, selected: list[dict], path: Path) -> dict:
    by_d = {r["density"]: r for r in selected}
    high = by_d[2]
    r_lb = max(r["raw_demand"] for r in rows if r["density"] == 2)

    # Curve excerpts around the hard transition and selected radius.
    excerpt_R = {0.60, 0.80, 0.90, 0.95, 1.10, 1.40, 1.60}
    excerpt = [r for r in rows if r["R"] in excerpt_R]

    summary = {
        "selected_R_m": r_star,
        "high_density_geometric_lower_bound_m": r_lb,
        "agent_radius_m": 0.30,
        "clearance_m": 0.18,
        "acceptance": {
            DENSITY_NAME[d]: {
                "boundary_violations": by_d[d]["demand_violations"],
                "outside_samples": by_d[d]["outside_samples"],
                "wall_samples": by_d[d]["wall_samples"],
                "detour_pct": round(by_d[d]["detour_pct"], 3),
                "detour_m": round(by_d[d]["length"] - by_d[d]["base_length"], 3),
                "length_m": round(by_d[d]["length"], 3),
                "global_length_m": round(by_d[d]["base_length"], 3),
            }
            for d in (0, 1, 2)
        },
    }
    (ART / "tube_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")

    lines: list[str] = []
    lines += [
        "# Tube boundary calibration report",
        "",
        "## Responsibility boundary",
        "",
        "Let \\(\\Gamma=\\bigcup_i[p_i,p_{i+1}]\\) be the global walkable polyline.",
        "The local layer may choose \\(q(t)\\) only in",
        "",
        "$$T_R(\\Gamma)=\\{x:\\operatorname{dist}(x,\\Gamma)\\le R\\}, "
        "\\qquad R=r+\\varepsilon,$$",
        "",
        "where \\(r=0.30\\,m\\) is the character radius and "
        "\\(\\varepsilon\\) is the radius-external offset budget.  Equivalently, "
        "the swept disk \\(D(q(t),r)\\) stays in the outer tube of radius "
        "\\(R+r\\).",
        "",
        "For a crowd cross-section with raw occupied width \\(B\\), its "
        "inflated one-sided bypass demand is",
        "",
        "$$b=\\frac B2+r+c,$$",
        "",
        "with personal-space clearance \\(c=0.18\\,m\\).  The local offset "
        "exceeds the tube exactly when",
        "",
        "$$b>\\frac{W}{2}=R",
        "\\iff \\frac B2+r+c>R.$$",
        "",
        "Therefore the geometric lower bound is",
        "",
        "$$\\boxed{R_{\\rm lb}=\\frac{B_{\\max}}2+r+c}.$$",
        "",
        "The high-density fixture has effective \\(B_{\\max}=0.88\\,m\\), giving",
        f"\\(R_{{\\rm lb}}={r_lb:.2f}\\,m\\).  The sweep grid is 0.05 m, so the "
        f"committed value is \\(R^*={r_star:.2f}\\,m\\).",
        "",
        "## Selection rule",
        "",
        "$$R^*=\\min\\left\\{R:\\ "
        "\\max_d V_d(R)=0,\\ "
        "\\max_d \\Delta L_d(R)/L_{\\Gamma,d}\\le0.15\\right\\}.$$",
        "",
        "This takes the smallest tube that is actually feasible: a narrow tube "
        "creates exceedance/projection and wall rubbing, while extra width does "
        "not improve the constrained optimum and only gives a wander-prone "
        "local layer more room to return into explored space.",
        "",
        "## Actual three-density run at the selected radius",
        "",
        "| density | clusters | pedestrians | violations | outside samples | wall samples | global L (m) | local L (m) | detour (m) | detour increase |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for d in (0, 1, 2):
        r = by_d[d]
        lines.append(
            f"| {DENSITY_NAME[d]} | {r['n_clusters']} | {r['n_peds']} | "
            f"**{r['demand_violations']}** | {r['outside_samples']} | "
            f"{r['wall_samples']} | {r['base_length']:.3f} | "
            f"{r['length']:.3f} | {r['length']-r['base_length']:.3f} | "
            f"**{r['detour_pct']:.2f}%** |")
    lines += [
        "",
        "Required acceptance:",
        "",
        f"- violations: low={by_d[0]['demand_violations']}, "
        f"medium={by_d[1]['demand_violations']}, "
        f"high={by_d[2]['demand_violations']} — all zero;",
        f"- detour increases: low={by_d[0]['detour_pct']:.3f}%, "
        f"medium={by_d[1]['detour_pct']:.3f}%, "
        f"high={by_d[2]['detour_pct']:.3f}% — all ≤ 15%.",
        "",
        "## Curve excerpts",
        "",
        "| R (m) | density | violations | wall samples | detour % |",
        "|---:|---|---:|---:|---:|",
    ]
    for r in excerpt:
        lines.append(
            f"| {r['R']:.2f} | {DENSITY_NAME[r['density']]} | "
            f"{r['demand_violations']} | {r['wall_samples']} | "
            f"{r['detour_pct']:.3f} |")
    lines += [
        "",
        "Machine-readable full sweep: `artifacts/tube_curves.csv`; "
        "chart: `artifacts/tube_curves.png`.",
        "",
        "The fixture is deterministic because the repository contains no "
        "recorded crowd snapshot.  Replacing the crowd list with real "
        "pedestrian cross-sections leaves the boundary expression and "
        "selection rule unchanged.",
        "",
        "Reproduce with:",
        "",
        "```sh",
        "make -C tube test",
        "python3 analysis/tube_experiment.py",
        "```",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> None:
    ART.mkdir(exist_ok=True)
    rows = run_c_sweep()
    r_star, selected = select_radius(rows)
    plot(rows, r_star, ART / "tube_curves.png")
    summary = write_report(rows, r_star, selected, ART / "tube_report.md")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"report={ART / 'tube_report.md'}")
    print(f"chart={ART / 'tube_curves.png'}")


if __name__ == "__main__":
    main()
