#!/usr/bin/env python3
"""Run deterministic random failure-path campaigns for grid A*.

Outputs probabilities, inverse round counts, gcov branch coverage curves,
timing curves, figures, and a machine-readable result bundle under
artifacts/random_failure_testing/.
"""
from __future__ import annotations

import csv
import json
import math
import subprocess
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[2]
GRID_DIR = ROOT / "grid"
FUZZ_C = GRID_DIR / "fuzz" / "grid_random_fuzz.c"
GRID_C = GRID_DIR / "src" / "grid.c"
ART = ROOT / "artifacts" / "random_failure_testing"
DATA = ART / "data"
COV = ART / "coverage"
BUILD = ART / "build"
FIG = ART / "figures"
for d in (DATA, COV, BUILD, FIG):
    d.mkdir(parents=True, exist_ok=True)

CAP = 16
EXPANSION_LIMIT = 96
PILOT_ROUNDS = 100_000
CURVE_BUDGETS = [0, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192]
FINAL_ROUNDS = 4096
CURVE_SEED = 20_261_126
PILOT_SEED = 99_173
TARGET_COVERAGE = 0.90

DENSITIES = [
    ("low", 0.10, "低密度 $d=0.10$", "#2a78d6"),
    ("medium", 0.25, "中密度 $d=0.25$", "#eb6834"),
    ("high", 0.40, "高密度 $d=0.40$", "#1baf7a"),
]
FAILURE_KEYS = [
    ("timeout", "T", "超时：$X>E_*$", "#2a78d6"),
    ("unreachable", "U", "不可达：$C(s)\\ne C(g)$", "#eb6834"),
    ("long", "L", "路径过长：$D(s,g)+1>K_*$", "#1baf7a"),
]

# Use the installed CJK face when present; otherwise matplotlib falls back.
for candidate in ("Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"):
    if any(candidate in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = candidate
        break
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams.update({
    "figure.facecolor": "#fcfcfb",
    "axes.facecolor": "#fcfcfb",
    "axes.edgecolor": "#c3c2b7",
    "axes.labelcolor": "#0b0b0b",
    "text.color": "#0b0b0b",
    "xtick.color": "#52514e",
    "ytick.color": "#52514e",
    "grid.color": "#e1e0d9",
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.titleweight": "semibold",
    "legend.frameon": False,
})


def run(cmd: list[str], **kw: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, check=True, text=True, capture_output=True, **kw)


def compile_binaries() -> tuple[Path, Path]:
    fast = BUILD / "fuzz_fast"
    cov = BUILD / "fuzz_cov"
    common = [
        "gcc", "-std=c11", "-D_POSIX_C_SOURCE=200809L",
        "-Wall", "-Wextra", str(FUZZ_C), "-lm",
    ]
    run(common + ["-O2", "-o", str(fast)])
    run(common + [
        "-O0", "-g", "--coverage",
        "-fprofile-abs-path", "-o", str(cov),
    ])
    return fast, cov


def last_json(stdout: str) -> dict[str, Any]:
    return json.loads(stdout.strip().splitlines()[-1])


def run_campaign(
    binary: Path,
    label: str,
    density: float,
    seed: int,
    rounds: int,
    *,
    coverage: bool = False,
    stop_after_all: bool = False,
) -> tuple[dict[str, Any], Path]:
    csv_path = DATA / f"{label}_d{density:.2f}_n{rounds}"
    if stop_after_all:
        csv_path = Path(str(csv_path) + "_adaptive.csv")
    else:
        csv_path = Path(str(csv_path) + ".csv")
    proc = run([
        str(binary), f"{density:.2f}", str(seed), str(rounds),
        str(CAP), str(EXPANSION_LIMIT), str(csv_path),
        "1" if stop_after_all else "0",
    ])
    summary = last_json(proc.stdout)
    summary["csv_path"] = str(csv_path.relative_to(ROOT))
    if coverage:
        summary["selftest_stdout_tail"] = proc.stdout.splitlines()[-22:-1]
    return summary, csv_path


def wilson_interval(x: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return 0.0, 0.0
    p = x / n
    z2 = z * z
    denom = 1 + z2 / n
    center = (p + z2 / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / denom
    return max(0.0, center - half), min(1.0, center + half)


def n_for_confidence(p: float, target: float = TARGET_COVERAGE) -> int | None:
    if p <= 0.0:
        return None
    if p >= 1.0:
        return 1
    return math.ceil(math.log(1.0 - target) / math.log1p(-p))


def gcov_grid(cov_binary: Path) -> dict[str, Any]:
    gcda = next(BUILD.glob("fuzz_cov*.gcda"))
    proc = run(
        ["gcov", "-b", "--json-format", "--stdout", gcda.name],
        cwd=BUILD,
    )
    data = json.loads(proc.stdout)
    for f in data["files"]:
        if f["file"].endswith("grid/src/grid.c"):
            return f
    raise RuntimeError("grid.c missing from gcov JSON")


def parse_branches(gcov_file: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line in gcov_file.get("lines", []):
        for arc, b in enumerate(line.get("branches", [])):
            rows.append({
                "line": line["line_number"],
                "arc": arc,
                "count": int(b["count"]),
                "fallthrough": bool(b.get("fallthrough", False)),
                "throw": bool(b.get("throw", False)),
            })
    for i, row in enumerate(rows, start=1):
        row["branch_id"] = f"G-B{i:03d}"
    return rows


def semantic_arcs(source_lines: list[str]) -> dict[str, tuple[int, int]]:
    """Locate the true/exit arcs without hard-coding shifted line numbers."""
    timeout_line = next(
        i + 1 for i, s in enumerate(source_lines)
        if "if (expanded > expansion_limit)" in s
    )
    long_line = next(
        i + 1 for i, s in enumerate(source_lines)
        if "if (n > cap) return -2;" in s
    )
    while_line = next(
        i + 1 for i, s in enumerate(source_lines)
        if "while (nopen > 0)" in s
    )
    # gcov orders these arcs as the taken/true return first for the two
    # one-line ifs; for while, the fall-through arc is the exhaustion exit.
    return {
        "timeout": (timeout_line, 0),
        "long": (long_line, 0),
        "unreachable": (while_line, 1),
    }


def reset_gcda() -> None:
    for p in BUILD.glob("fuzz_cov*.gcda"):
        p.unlink()


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def make_probability_chart(prob_rows: list[dict[str, Any]]) -> None:
    fig, (axp, axn) = plt.subplots(1, 2, figsize=(11.2, 4.2))
    x = range(len(DENSITIES))
    width = 0.24
    key_to_color = {k: c for k, _, _, c in FAILURE_KEYS}

    for j, (key, _code, label, color) in enumerate(FAILURE_KEYS):
        ps = [next(r for r in prob_rows if r["density_label"] == lab and r["key"] == key)["p"]
              for lab, _, _, _ in DENSITIES]
        ns = [next(r for r in prob_rows if r["density_label"] == lab and r["key"] == key)["n90"]
              for lab, _, _, _ in DENSITIES]
        offsets = [i + (j - 1) * width for i in x]
        axp.bar(offsets, [max(p, 2e-5) for p in ps], width=width,
                color=color, label=label.split("：")[0])
        for xi, p in zip(offsets, ps):
            axp.annotate(f"{p:.2%}", (xi, max(p, 2e-5)),
                         xytext=(0, 4), textcoords="offset points",
                         ha="center", va="bottom", fontsize=8,
                         color="#52514e", rotation=90)
        axn.bar(offsets, ns, width=width, color=color)
        for xi, n in zip(offsets, ns):
            axn.annotate(f"{n:,}", (xi, n), xytext=(0, 3),
                         textcoords="offset points", ha="center",
                         va="bottom", fontsize=8, color="#52514e")

    for ax in (axp, axn):
        ax.set_xticks(list(x), [lab for _, _, lab, _ in DENSITIES])
        ax.grid(axis="y", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
    axp.set_yscale("log")
    axp.set_ylim(2e-5, 1.2)
    axp.set_ylabel("单轮触发概率 $p_k(d)$（对数轴）")
    axp.set_title("失败条件落入比例")
    axn.set_ylabel("$N_{90}=\\lceil\\ln 0.1/\\ln(1-p)\\rceil$")
    axn.set_ylim(0, max(r["n90"] for r in prob_rows) * 1.22)
    axn.set_title("九成单分支命中概率所需轮数")
    axp.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(FIG / f"probability_and_rounds.{ext}", dpi=180)
    plt.close(fig)


def make_tradeoff_chart(curve_rows: list[dict[str, Any]]) -> None:
    fig, (axc, axt) = plt.subplots(
        2, 1, figsize=(8.6, 6.4), sharex=True, gridspec_kw={"hspace": 0.08}
    )
    for label, d, series_label, color in DENSITIES:
        rows = sorted(
            (r for r in curve_rows if r["density_label"] == label),
            key=lambda r: r["rounds"],
        )
        xs = [r["rounds"] for r in rows]
        cov = [100 * r["branch_coverage"] for r in rows]
        ms = [r["elapsed_ms"] for r in rows]
        axc.plot(xs, cov, color=color, linewidth=2, marker="o", markersize=5,
                 markerfacecolor=color, markeredgecolor="#fcfcfb",
                 markeredgewidth=1.5, label=series_label)
        axt.plot(xs, ms, color=color, linewidth=2, marker="o", markersize=5,
                 markerfacecolor=color, markeredgecolor="#fcfcfb",
                 markeredgewidth=1.5, label=series_label)
        end = rows[-1]
        axc.annotate(f"{end['branch_coverage']:.1%}",
                     (end["rounds"], 100 * end["branch_coverage"]),
                     xytext=(6, 0), textcoords="offset points",
                     va="center", fontsize=8, color="#52514e")

    axc.axhline(90, color="#d03b3b", linewidth=1.5, linestyle="-", alpha=0.75)
    axc.text(0.02, 90.8, "90% 目标", color="#a52a2a", fontsize=9,
             transform=axc.get_yaxis_transform())
    for ax in (axc, axt):
        ax.axvline(FINAL_ROUNDS, color="#898781", linewidth=1.2, alpha=0.8)
        ax.grid(axis="y", linewidth=0.8)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_xscale("symlog", linthresh=16, linscale=0.5)
    axt.set_xlim(0, 9000)
    axc.set_ylabel("gcov 分支覆盖率")
    axc.set_ylim(55, 102)
    axc.legend(loc="lower right", ncol=3)
    axc.set_title("随机测试轮数的覆盖率—时长取舍")
    axt.set_ylabel("内部计时（毫秒）")
    axt.set_xlabel("每档地图的随机测试轮数（对称对数轴）")
    final_ms = [r["elapsed_ms"] for r in curve_rows if r["rounds"] == FINAL_ROUNDS]
    timing_note = (f"{min(final_ms):.1f}–{max(final_ms):.1f} ms / 档"
                   if final_ms else "耗时见 CSV")
    axt.text(0.56, 0.78,
             f"选定 $N_*={FINAL_ROUNDS:,}$\n{timing_note}",
             transform=axt.transAxes, fontsize=9, color="#52514e",
             ha="left", va="top")
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(FIG / f"coverage_time_tradeoff.{ext}", dpi=180)
    plt.close(fig)


def main() -> None:
    fast_bin, cov_bin = compile_binaries()
    source_lines = GRID_C.read_text(encoding="utf-8").splitlines()
    semantic = semantic_arcs(source_lines)

    # 1) Large fixed pilots estimate p_k(d), the failure-condition mass.
    prob_rows: list[dict[str, Any]] = []
    pilot_summaries: dict[str, dict[str, Any]] = {}
    for label, density, _, _ in DENSITIES:
        summary, _ = run_campaign(
            fast_bin, "pilot", density, PILOT_SEED, PILOT_ROUNDS
        )
        pilot_summaries[label] = summary
        n = summary["rounds_attempted"]
        hit_fields = {
            "timeout": "hits_timeout",
            "unreachable": "hits_unreachable",
            "long": "hits_long",
        }
        first_fields = {
            "timeout": "first_timeout",
            "unreachable": "first_unreachable",
            "long": "first_long",
        }
        for key, code, _zh, _color in FAILURE_KEYS:
            hits = int(summary[hit_fields[key]])
            lo, hi = wilson_interval(hits, n)
            p = hits / n
            prob_rows.append({
                "density_label": label,
                "density": density,
                "key": key,
                "code": code,
                "rounds": n,
                "hits": hits,
                "p": p,
                "wilson95_low": lo,
                "wilson95_high": hi,
                "n90": n_for_confidence(p),
                "n90_conservative": n_for_confidence(lo),
                "observed_first_hit": summary[first_fields[key]],
            })
    write_csv(DATA / "probabilities.csv", prob_rows, list(prob_rows[0].keys()))
    (DATA / "pilot_summaries.json").write_text(
        json.dumps(pilot_summaries, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    planning_n = max(r["n90_conservative"] or 0 for r in prob_rows)

    # 2) Coverage/time curve from gcov runs at prefix budgets.
    curve_rows: list[dict[str, Any]] = []
    final_branches_by_density: dict[str, list[dict[str, Any]]] = {}
    branch_catalog: list[dict[str, Any]] | None = None

    for label, density, _, _ in DENSITIES:
        for budget in CURVE_BUDGETS:
            reset_gcda()
            summary, csv_path = run_campaign(
                cov_bin, "curve", density, CURVE_SEED, budget, coverage=True
            )
            gcov = gcov_grid(cov_bin)
            branches = parse_branches(gcov)
            if budget == FINAL_ROUNDS:
                final_branches_by_density[label] = branches
                (COV / f"final_{label}_gcov.json").write_text(
                    json.dumps(gcov, ensure_ascii=False, indent=2), encoding="utf-8"
                )
            if branch_catalog is None:
                branch_catalog = []
                source_by_line = {
                    i + 1: line.strip() for i, line in enumerate(source_lines)
                }
                for b in branches:
                    sem = next((k for k, loc in semantic.items()
                                if loc == (b["line"], b["arc"])), "")
                    branch_catalog.append({
                        **{k: b[k] for k in ("branch_id", "line", "arc", "fallthrough")},
                        "semantic": sem,
                        "source": source_by_line.get(b["line"], ""),
                    })

            total = len(branches)
            covered = sum(b["count"] > 0 for b in branches)
            target_status = {}
            for key, (line, arc) in semantic.items():
                b = next(x for x in branches if x["line"] == line and x["arc"] == arc)
                bid = next(x["branch_id"] for x in branches
                           if x["line"] == line and x["arc"] == arc)
                target_status[f"{key}_branch_id"] = bid
                target_status[f"{key}_count"] = b["count"]
                target_status[f"{key}_covered"] = b["count"] > 0

            curve_rows.append({
                "density_label": label,
                "density": density,
                "rounds": summary["rounds_attempted"],
                "branch_total": total,
                "branch_covered": covered,
                "branch_coverage": covered / total,
                "line_coverage": (
                    sum(l["count"] > 0 for l in gcov["lines"]) /
                    max(1, len(gcov["lines"]))
                ),
                "elapsed_ns": summary["elapsed_ns"],
                "elapsed_ms": summary["elapsed_ns"] / 1e6,
                **target_status,
            })

    assert branch_catalog is not None
    write_csv(COV / "branch_catalog.csv", branch_catalog,
              ["branch_id", "line", "arc", "fallthrough", "semantic", "source"])
    write_csv(DATA / "coverage_time_curve.csv", curve_rows,
              list(curve_rows[0].keys()))

    # 3) Final N*=4096 acceptance table. The 4096 curve point is the one
    # acceptance run per density; gcov JSON and CSV are archived above.
    acceptance_rows: list[dict[str, Any]] = []
    uncovered_rows: list[dict[str, Any]] = []
    for label, density, _, _ in DENSITIES:
        crow = next(r for r in curve_rows
                    if r["density_label"] == label and r["rounds"] == FINAL_ROUNDS)
        branches = final_branches_by_density[label]
        bid_by_loc = {(b["line"], b["arc"]): b["branch_id"] for b in branches}
        count_by_loc = {(b["line"], b["arc"]): b["count"] for b in branches}
        first_hit = {
            "timeout": None,
            "unreachable": None,
            "long": None,
        }
        random_hits = {
            "timeout": 0,
            "unreachable": 0,
            "long": 0,
        }
        # First hits for this seed are available by scanning its archived CSV.
        final_csv = DATA / f"curve_d{density:.2f}_n{FINAL_ROUNDS}.csv"
        with final_csv.open(encoding="utf-8") as f:
            for row in csv.DictReader(f):
                rnd = int(row["round"])
                for key, col in (("timeout", "timeout_branch"),
                                 ("unreachable", "unreachable_branch"),
                                 ("long", "long_branch")):
                    if row[col] == "1":
                        random_hits[key] += 1
                        if first_hit[key] is None:
                            first_hit[key] = rnd

        for key, _code, _zh, _color in FAILURE_KEYS:
            loc = semantic[key]
            bid = bid_by_loc[loc]
            count = count_by_loc[loc]
            triggered = count > 0 and first_hit[key] is not None
            acceptance_rows.append({
                "density_label": label,
                "density": density,
                "failure": key,
                "branch_id": bid,
                "location": f"grid.c:{loc[0]} arc#{loc[1]}",
                "triggered": int(triggered),
                "first_hit_round": first_hit[key] if triggered else "",
                "random_hit_count": random_hits[key],
                "attempted_rounds": FINAL_ROUNDS,
                "gcov_arc_count_including_selftest": count,
                "grid_branch_coverage": f"{crow['branch_coverage']:.4f}",
                "elapsed_ms": f"{crow['elapsed_ms']:.3f}",
            })

        for b in branches:
            if b["count"] == 0:
                uncovered_rows.append({
                    "density_label": label,
                    "density": density,
                    "branch_id": b["branch_id"],
                    "location": f"grid.c:{b['line']} arc#{b['arc']}",
                    "attempted_rounds": FINAL_ROUNDS,
                })

    write_csv(DATA / "acceptance.csv", acceptance_rows,
              list(acceptance_rows[0].keys()))
    write_csv(DATA / "uncovered_branches.csv", uncovered_rows,
              list(uncovered_rows[0].keys()) if uncovered_rows else
              ["density_label", "density", "branch_id", "location", "attempted_rounds"])

    make_probability_chart(prob_rows)
    make_tradeoff_chart(curve_rows)

    result = {
        "settings": {
            "width": 16,
            "height": 12,
            "cells": 192,
            "path_cap_vertices": CAP,
            "expansion_limit": EXPANSION_LIMIT,
            "pilot_rounds": PILOT_ROUNDS,
            "final_rounds": FINAL_ROUNDS,
            "curve_budgets": CURVE_BUDGETS,
            "curve_seed": CURVE_SEED,
            "pilot_seed": PILOT_SEED,
            "target_coverage": TARGET_COVERAGE,
        },
        "planning_n90_conservative_max": planning_n,
        "probabilities": prob_rows,
        "curve": curve_rows,
        "acceptance": acceptance_rows,
        "uncovered": uncovered_rows,
        "artifacts": {
            "probability_chart_png": str((FIG / "probability_and_rounds.png").relative_to(ROOT)),
            "tradeoff_chart_png": str((FIG / "coverage_time_tradeoff.png").relative_to(ROOT)),
        },
    }
    (ART / "results.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps({
        "ok": True,
        "results": str((ART / "results.json").relative_to(ROOT)),
        "planning_n90_conservative_max": planning_n,
        "final_rounds": FINAL_ROUNDS,
        "all_targets_triggered": all(r["triggered"] for r in acceptance_rows),
        "uncovered_branch_rows": len(uncovered_rows),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
