#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运行全部实验并把结果写到 hotupdate/results/。"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim import (  # noqa: E402
    SWITCH_T, T0, T_LAG, T_ROLLBACK, SIM_START, SIM_END,
    N_CLIENTS, CHEAT_RATE, simulate, summarize, window,
)

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.normpath(os.path.join(HERE, "..", "results"))

TIERS = [15, 60, 180]          # 三档客户端最大同步延迟（秒）
POLICIES = ["buggy", "fixed", "latebump"]
K_GRID = [0, 1, 2, 3, 4, 5]    # 严格程度：允许的不可走点数


def clock(t: int) -> str:
    """仿真秒 -> 事故时钟字符串（300s = 10:05:00）。"""
    base_min = 10 * 60 + 5      # SWITCH_T=300 -> 10:05
    total_min = base_min + (t - SWITCH_T) // 60
    sec = (t - SWITCH_T) % 60
    return f"{total_min // 60:02d}:{total_min % 60:02d}:{sec:02d}"


def run_tier(d_max):
    out = {"d_max": d_max, "policies": {}}
    for policy in POLICIES:
        rows, server, clients, maps = simulate(d_max, policy, k=0)
        s = summarize(rows, server)
        # 切换致因误判诊断（题目要求：被拒路径版本号 + 当前地图版本）
        diag = [
            dict(t=r.t, clock=clock(r.t), client=r.client,
                 gen_version=r.gen_version, label_version=r.label_version,
                 server_map_version=r.eval_map_version,
                 bad_on_eval_map=r.bad_eval, bad_on_own_map=r.bad_own,
                 reason=r.reason)
            for r in server.rejects
            if (not r.is_cheat) and r.switch_induced
            and SWITCH_T <= r.t <= T_ROLLBACK
        ]
        # 反解：用切换致因拒绝的衰减包络（回滚窗口内 + 全窗口两组）
        inferred_dmax = s["inferred_d_max_observed_s"]
        inferred_bump = SWITCH_T - inferred_dmax
        inferred_full = s["inferred_d_max_full_s"]
        out["policies"][policy] = {
            "bump_t": server.bump_t,
            "bump_clock": clock(server.bump_t) if server.bump_t is not None else None,
            "lead_time_s": (SWITCH_T - server.bump_t)
            if server.bump_t is not None else None,
            "summary": s,
            "spike_clock": clock(s["spike_t"]) if s["spike_t"] else None,
            "inferred_d_max_s": inferred_dmax,
            "inferred_required_bump_t": inferred_bump,
            "inferred_bump_clock": clock(inferred_bump),
            "inferred_d_max_full_s": inferred_full,
            "inferred_full_bump_clock": clock(SWITCH_T - inferred_full),
            "censored_by_rollback": s["inference_censored_by_rollback"],
            "n_switch_false_rejects": s["switch_false_rejects"],
            "diagnostics_total": len(diag),
            "diagnostics_sample": diag[:6],
            "series": [
                dict(t=r["t"], reject_rate=r["reject_rate"], fpr=r["fpr"],
                     cir=r["cir"], switch_induced=r["switch_induced"],
                     edge_noise=r["edge_noise"], skew=r["skew"])
                for r in rows
            ],
        }
    return out


def run_strictness(d_max=60):
    """平时（cutover 前）误判率 FPR vs 作弊拦截率 CIR 两条曲线。"""
    curve = []
    for k in K_GRID:
        rows, *_ = simulate(d_max, "fixed", k=k)
        pre = window(rows, SIM_START, SWITCH_T - 1)
        fpr = sum(r["rej_honest"] for r in pre) / max(1, sum(r["honest"] for r in pre))
        cir = sum(r["rej_cheat"] for r in pre) / max(1, sum(r["cheat"] for r in pre))
        # 版本化 + 足提前量下，任何严格程度切换致因误伤都必须为 0
        sw = window(rows, SWITCH_T, T_ROLLBACK)
        sw_false = sum(r["switch_induced"] for r in sw)
        curve.append(dict(k=k, fpr=fpr, cir=cir, switch_false=sw_false))
    eligible = [c for c in curve if c["fpr"] <= 0.01]
    chosen = max(eligible, key=lambda c: c["cir"]) if eligible else curve[0]
    return curve, chosen


def main():
    os.makedirs(RESULTS, exist_ok=True)
    tiers = []
    for d in TIERS:
        print(f"== tier D_max={d}s ==")
        res = run_tier(d)
        for p, pr in res["policies"].items():
            sm = pr["summary"]
            print(f"  {p:9s} bump={pr['bump_clock']} lead={pr['lead_time_s']}s "
                  f"spike={pr['spike_clock']} peak={sm['peak_rate']:.3f} "
                  f"switch_false={pr['n_switch_false_rejects']} "
                  f"infer_obs={pr['inferred_d_max_s']}s "
                  f"infer_full={pr['inferred_d_max_full_s']}s "
                  f"censored={pr['censored_by_rollback']}")
        tiers.append(res)

    print("== strictness sweep (D_max=60, fixed, pre-cutover window) ==")
    curve, chosen = run_strictness(60)
    for c in curve:
        print(f"  k={c['k']} FPR={c['fpr']:.4f} CIR={c['cir']:.4f} "
              f"switch_false={c['switch_false']}")
    print(f"  chosen operating point: k={chosen['k']} "
          f"(FPR={chosen['fpr']:.4f}, CIR={chosen['cir']:.4f})")

    payload = {
        "meta": dict(
            tick_seconds=1, switch_t=SWITCH_T, t0_report=T0, t_lag=T_LAG,
            t_rollback=T_ROLLBACK, sim_start=SIM_START, sim_end=SIM_END,
            n_clients=N_CLIENTS, cheat_rate=CHEAT_RATE, tiers_s=TIERS,
            k_grid=K_GRID,
            clock_mapping="300s=10:05:00, each tick +1s",
        ),
        "tiers": tiers,
        "strictness": {"curve": curve, "chosen": chosen},
    }
    out = os.path.join(RESULTS, "sim_results.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)
    print("wrote", out)


if __name__ == "__main__":
    main()
