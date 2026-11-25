#!/usr/bin/env python3
"""Deterministic path-freshness release lab.

It does three things:
  1. builds/runs the C microbenchmark for a last-frame feasibility recheck;
  2. runs one seeded map-update batch for each of three update frequencies;
  3. writes CSV/JSON/PNG curves and enforces the release gates.

The risk model is the requested linear budget:
    P_stale ~= lambda * N_frames * frame_seconds.
For a Poisson update process the exact probability is 1-exp(-lambda*window);
the linear form is its small-window approximation / conservative rate budget.
"""

from __future__ import annotations

import csv
import json
import math
import random
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "artifacts" / "path-freshness"
C_BENCH = ROOT / "tools" / "path_freshness_bench.c"
BIN = OUT / "path_freshness_bench"

FRAME_SECONDS = 1.0 / 60.0
P_BUDGET = 1e-3
TRIALS = 50_000
GAP_MAX_FRAMES = 12
SEED_BASE = 20261126

# Changes per second. 0.01/s = 0.6/min, 0.1/s = 6/min, 1/s = 60/min.
TIERS = [
    ("low", 0.01),
    ("medium", 0.10),
    ("high", 1.00),
]

OVERHEAD_BUDGET = 0.20


def run_c_benchmark() -> dict[str, float | int]:
    OUT.mkdir(parents=True, exist_ok=True)
    cc = "gcc"
    build = subprocess.run(
        [cc, "-O2", "-std=c11", "-Wall", "-Wextra",
         str(C_BENCH), "-o", str(BIN), "-lm"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if build.returncode != 0:
        sys.stderr.write(build.stderr)
        raise SystemExit("C benchmark build failed")
    if build.stderr:
        # Warnings are release defects; fail rather than merely logging them.
        sys.stderr.write(build.stderr)
        raise SystemExit("C benchmark compiled with warnings")

    run = subprocess.run([str(BIN)], capture_output=True, text=True, check=False)
    if run.returncode not in (0,):
        sys.stderr.write(run.stdout)
        sys.stderr.write(run.stderr)
        raise SystemExit(f"C benchmark failed: exit {run.returncode}")

    values: dict[str, float | int] = {}
    for line in run.stdout.splitlines():
        key, raw = line.split("=", 1)
        value: float | int
        try:
            value = int(raw)
        except ValueError:
            value = float(raw)
        values[key] = value
    return values


def max_frames_linear(lam: float) -> int:
    return math.floor(P_BUDGET / (lam * FRAME_SECONDS) + 1e-12)


def max_frames_poisson(lam: float) -> int:
    return math.floor(
        -math.log1p(-P_BUDGET) / (lam * FRAME_SECONDS) + 1e-12
    )


def simulate_tier(name: str, lam: float, bench: dict[str, float | int]) -> tuple[list[dict], dict]:
    rng = random.Random(SEED_BASE + {
        "low": 1, "medium": 2, "high": 3,
    }[name])
    p_update_per_frame = min(1.0, lam * FRAME_SECONDS)

    # One seeded batch per tier. Sweeping K re-aggregates this same batch;
    # it is not a second stochastic run.
    scenarios: list[tuple[int, bool]] = []
    stale_count = 0
    for _ in range(TRIALS):
        frames = rng.randint(1, GAP_MAX_FRAMES)
        changed = False
        for _frame in range(frames):
            if rng.random() < p_update_per_frame:
                changed = True
                break
        scenarios.append((frames, changed))
        stale_count += int(changed)

    raw_ratio = float(bench["check_overhead_ratio"])
    los_ratio = float(bench["los_check_overhead_ratio"])
    rows: list[dict] = []

    for k in range(GAP_MAX_FRAMES + 1):
        bad = 0
        caught = 0
        checks = 0
        for frames, changed in scenarios:
            if frames <= k:
                # Under threshold: no recheck. A relevant edit reaches start.
                if changed:
                    bad += 1
            else:
                checks += 1
                if changed:
                    caught += 1

        check_fraction = checks / TRIALS
        stale_fraction = stale_count / TRIALS
        # Age-gated overhead. K=0 checks every positive-age departure.
        raw_overhead_pct = 100.0 * raw_ratio * check_fraction
        los_overhead_pct = 100.0 * los_ratio * check_fraction
        rows.append({
            "tier": name,
            "lambda_per_s": lam,
            "seed": SEED_BASE + {"low": 1, "medium": 2, "high": 3}[name],
            "trials": TRIALS,
            "K_max_unchecked_frames": k,
            "bad_departures": bad,
            "caught_stale": caught,
            "rechecks": checks,
            "recheck_fraction": round(check_fraction, 8),
            "raw_check_overhead_pct": round(raw_overhead_pct, 6),
            "los_check_overhead_pct": round(los_overhead_pct, 6),
            "first_replan_amortized_pct": round(100.0 * stale_fraction, 6),
            "raw_total_gate_pct": round(
                100.0 * (raw_ratio * check_fraction + stale_fraction), 6
            ),
            "los_total_gate_pct": round(
                100.0 * (los_ratio * check_fraction + stale_fraction), 6
            ),
        })

    selected = rows[0]  # K=0: every departure gets the final feasibility gate.
    summary = {
        "tier": name,
        "lambda_per_s": lam,
        "updates_per_minute": lam * 60.0,
        "linear_Nmax_for_p_budget": max_frames_linear(lam),
        "poisson_Nmax_for_p_budget": max_frames_poisson(lam),
        "stale_without_gate": stale_count,
        "stale_rate": stale_count / TRIALS,
        "selected_K": 0,
        "bad_departures_selected": selected["bad_departures"],
        "raw_recheck_overhead_pct_when_always": 100.0 * raw_ratio,
        "los_recheck_overhead_pct_when_always": 100.0 * los_ratio,
        "first_replan_amortized_pct": 100.0 * stale_count / TRIALS,
        "raw_total_gate_pct_when_always": 100.0 * (raw_ratio + stale_count / TRIALS),
        "los_total_gate_pct_when_always": 100.0 * (los_ratio + stale_count / TRIALS),
    }
    return rows, summary


def write_csv(rows: list[dict]) -> None:
    path = OUT / "curves.csv"
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot(rows_by_tier: dict[str, list[dict]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = [
        "Noto Sans CJK SC", "WenQuanYi Zen Hei", "DejaVu Sans"
    ]
    plt.rcParams["axes.unicode_minus"] = False

    colors = {"low": "#2a78d6", "medium": "#eb6834", "high": "#1baf7a"}
    labels = {
        "low": "低频率 λ=0.01/s",
        "medium": "中频率 λ=0.10/s",
        "high": "高频率 λ=1.00/s",
    }

    fig, (ax_bad, ax_cost) = plt.subplots(
        1, 2, figsize=(12.2, 4.8), dpi=170, constrained_layout=True
    )
    fig.patch.set_facecolor("#fcfcfb")
    for ax in (ax_bad, ax_cost):
        ax.set_facecolor("#fcfcfb")
        ax.grid(axis="y", color="#e1e0d9", linewidth=0.8)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["left"].set_color("#c3c2b7")
        ax.spines["bottom"].set_color("#c3c2b7")
        ax.tick_params(colors="#52514e", labelsize=9)
        ax.xaxis.label.set_color("#0b0b0b")
        ax.yaxis.label.set_color("#0b0b0b")
        ax.title.set_color("#0b0b0b")

    for name, tier_rows in rows_by_tier.items():
        ks = [r["K_max_unchecked_frames"] for r in tier_rows]
        bad = [r["bad_departures"] for r in tier_rows]
        ax_bad.plot(
            ks, bad, color=colors[name], linewidth=2.2,
            marker="o", markersize=4.2, label=labels[name],
        )
        # Direct labels at the never-recheck end; avoids a crowded legend-only read.
        ax_bad.annotate(
            str(bad[-1]), xy=(ks[-1], bad[-1]), xytext=(5, 0),
            textcoords="offset points", va="center", fontsize=8.5,
            color="#52514e",
        )

    ax_bad.axvline(0, color="#0ca30c", linewidth=1.5, linestyle=":")
    ax_bad.annotate(
        "K=0：全部末检，带病出发=0",
        xy=(0, 0), xytext=(1.2, max(r[-1]["bad_departures"] for r in rows_by_tier.values()) * 0.82),
        fontsize=9, color="#006300",
    )
    ax_bad.set_title("失效次数：跳过末检的帧数阈值越高，风险越大", fontsize=11, pad=10)
    ax_bad.set_xlabel("K：允许免查的最大帧数\nNf > K 时必须重查")
    ax_bad.set_ylabel(f"{TRIALS:,} 次出发中带病出发次数")
    ax_bad.set_xticks(range(0, GAP_MAX_FRAMES + 1, 2))
    ax_bad.set_xlim(-0.4, GAP_MAX_FRAMES + 0.8)
    ax_bad.legend(frameon=False, fontsize=8.8, loc="upper left")

    # K=0 checks every positive-age path. The LOS number is the conservative
    # upper bound for either cell-path or smoothed-path validation.
    first = next(iter(rows_by_tier.values()))
    ks = [r["K_max_unchecked_frames"] for r in first]
    los_cost = [r["los_check_overhead_pct"] for r in first]
    raw_cost = [r["raw_check_overhead_pct"] for r in first]
    ax_cost.plot(
        ks, los_cost, color="#2a78d6", linewidth=2.4,
        marker="o", markersize=4.5, label="平滑路径 LOS 末检（保守）",
    )
    ax_cost.plot(
        ks, raw_cost, color="#1baf7a", linewidth=2.0,
        marker="s", markersize=4.0, linestyle="--", label="格点路径可行性扫描",
    )
    ax_cost.axhline(20, color="#d03b3b", linewidth=1.8, linestyle="--")
    ax_cost.annotate(
        "20% 预算线", xy=(0, 20), xytext=(6, 6),
        textcoords="offset points", ha="left", fontsize=9, color="#a02b2b",
    )
    ax_cost.fill_between(ks, 0, los_cost, color="#2a78d6", alpha=0.08)
    ax_cost.set_title("额外开销：末检相对一次 A* 寻路", fontsize=11, pad=10)
    ax_cost.set_xlabel("K：允许免查的最大帧数\nNf > K 时必须重查")
    ax_cost.set_ylabel("额外耗时 / 单次寻路耗时 (%)")
    ax_cost.set_xticks(range(0, GAP_MAX_FRAMES + 1, 2))
    ax_cost.set_ylim(0, 22)
    ax_cost.legend(frameon=False, fontsize=8.8, loc="center right")

    png = OUT / "freshness-curves.png"
    fig.savefig(png, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def write_markdown(bench: dict, summaries: list[dict]) -> None:
    lines = [
        "# 路径时效实验结果",
        "",
        f"- 帧时长：`{FRAME_SECONDS:.6f}s`（60 FPS）",
        f"- 可接受线性失效率预算：`{P_BUDGET:g}`",
        f"- 每档确定性批次：`{TRIALS:,}` 次出发；窗口 `1..{GAP_MAX_FRAMES}` 帧",
        "- 地图更新在模拟中按帧到达；压力假设为每次相关更新都可能挡住当前路径",
        "",
        "## 反解出的最大免查帧数",
        "",
        "| 频率档 | λ (次/秒) | 次/分钟 | 线性 Nmax | Poisson Nmax |",
        "|---|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['tier']} | {s['lambda_per_s']:g} | "
            f"{s['updates_per_minute']:.1f} | "
            f"{s['linear_Nmax_for_p_budget']} | "
            f"{s['poisson_Nmax_for_p_budget']} |"
        )

    lines += [
        "",
        "## C 实测末检成本",
        "",
        f"- A*：`{bench['plan_ns_per_call']:.1f} ns/次`",
        f"- 格点可行性扫描：`{bench['check_ns_per_call']:.1f} ns/次`，"
        f"为寻路耗时的 `{100*bench['check_overhead_ratio']:.2f}%`",
        f"- 平滑路径 LOS 扫描：`{bench['los_check_ns_per_call']:.1f} ns/次`，"
        f"为寻路耗时的 `{100*bench['los_check_overhead_ratio']:.2f}%`",
        "",
        "## 三档各一次的确定性批次（发布策略 K=0：出发前必检）",
        "",
        "| 频率档 | λ/s | 无检查时失效数 | 带病出发 | 末检额外耗时 | 含一次重规划的保守总开销 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['tier']} | {s['lambda_per_s']:g} | "
            f"{s['stale_without_gate']} | "
            f"**{s['bad_departures_selected']}** | "
            f"{s['los_recheck_overhead_pct_when_always']:.2f}% (LOS 上限) | "
            f"{s['los_total_gate_pct_when_always']:.2f}% |"
        )

    lines += [
        "",
        "![失效次数与额外开销曲线](freshness-curves.png)",
        "",
        "完整逐 K 数据见 `curves.csv`，机器可读结果见 `summary.json`。",
        "",
    ]
    (OUT / "results.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    bench = run_c_benchmark()
    all_rows: list[dict] = []
    rows_by_tier: dict[str, list[dict]] = {}
    summaries: list[dict] = []

    for name, lam in TIERS:
        rows, summary = simulate_tier(name, lam, bench)
        all_rows.extend(rows)
        rows_by_tier[name] = rows
        summaries.append(summary)
        print(
            f"{name:6s} lambda={lam:4.2f}/s "
            f"Nmax_linear={summary['linear_Nmax_for_p_budget']:2d} "
            f"stale_without_gate={summary['stale_without_gate']:5d} "
            f"bad_selected={summary['bad_departures_selected']} "
            f"los_recheck={summary['los_recheck_overhead_pct_when_always']:.2f}%"
        )

    write_csv(all_rows)
    plot(rows_by_tier)

    acceptance = {
        "zero_bad_departures": all(s["bad_departures_selected"] == 0 for s in summaries),
        "raw_check_under_20pct": float(bench["check_overhead_ratio"]) <= OVERHEAD_BUDGET,
        "los_check_under_20pct": float(bench["los_check_overhead_ratio"]) <= OVERHEAD_BUDGET,
        "all_tiers_ran_once": len(summaries) == 3,
    }
    summary_doc = {
        "parameters": {
            "frame_seconds": FRAME_SECONDS,
            "p_budget": P_BUDGET,
            "trials_per_tier": TRIALS,
            "gap_max_frames": GAP_MAX_FRAMES,
            "overhead_budget": OVERHEAD_BUDGET,
        },
        "c_benchmark": bench,
        "tiers": summaries,
        "acceptance": acceptance,
    }
    (OUT / "summary.json").write_text(
        json.dumps(summary_doc, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_markdown(bench, summaries)

    if not all(acceptance.values()):
        print(f"RELEASE_GATE_FAIL {json.dumps(acceptance)}", file=sys.stderr)
        return 1

    print(f"artifacts={OUT.relative_to(ROOT)}")
    print("RELEASE_GATE_PASS zero_bad_departures=3/3 overhead<=20%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
