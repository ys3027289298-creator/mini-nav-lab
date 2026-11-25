#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
selfcheck：对仿真结果做不变量断言，`make test` 运行。
覆盖：判据表达式、根因、反解精度、零误伤约束、严格程度单调性。
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sim import (  # noqa: E402
    build_maps, astar, supercover_line, count_blocked, check_path,
    idx, simulate, summarize, SWITCH_T, T_ROLLBACK,
)

failures = 0


def check(name, ok, detail=""):
    print(f"{name}={'PASS' if ok else 'FAIL'} {detail}")
    if not ok:
        global failures
        failures += 1


def main():
    v1, v2 = build_maps()

    # --- 判据 C-1/C-2：逐点可走性，任一点不可走即拒绝（k=0） ---
    p = astar(v1, idx(1, 1), idx(14, 10))
    check("criterion_honest_path_on_v1", check_path(v1, p, 0))
    # 构造一条经过 v2 增量墙单元 (7,4) 的路径：该点 v1 可走、v2 不可走
    delta_cell = idx(7, 4)
    check("delta_cell_walkable_in_v1", delta_cell not in v1.blocked)
    check("delta_cell_blocked_in_v2", delta_cell in v2.blocked)
    skew_path = p + [delta_cell]
    check("criterion_rejects_skew_on_v2", not check_path(v2, skew_path, 0))
    check("criterion_skew_fine_on_v1", check_path(v1, skew_path, 0))
    # k 放宽到坏点数时恰好通过
    b = count_blocked(v2, skew_path)
    check("criterion_k_threshold",
          (not check_path(v2, skew_path, b - 1)) and check_path(v2, skew_path, b))

    # 作弊直线在自己的地图上必然穿墙
    line = supercover_line(idx(1, 1), idx(14, 10))
    check("cheat_line_pierces_wall", count_blocked(v1, line) > 0,
          f"bad={count_blocked(v1, line)}")

    # --- 三档延迟 × 三策略的事故不变量 ---
    with open(os.path.join(HERE, "..", "results", "sim_results.json"),
              encoding="utf-8") as f:
        data = json.load(f)

    for tier in data["tiers"]:
        d = tier["d_max"]
        buggy = tier["policies"]["buggy"]
        fixed = tier["policies"]["fixed"]
        late = tier["policies"]["latebump"]

        # 1) 修复策略：因地图切换导致的误判必须为零
        check(f"d{d}_fixed_zero_switch_misjudge",
              fixed["n_switch_false_rejects"] == 0,
              f"got {fixed['n_switch_false_rejects']}")
        # 2) 修复策略 bump 提前量恰为 D_max
        check(f"d{d}_fixed_lead_equals_dmax",
              fixed["lead_time_s"] == d)
        # 3) 事故与 bump 不足策略必然产生切换致因误判
        check(f"d{d}_buggy_has_misjudge",
              buggy["n_switch_false_rejects"] > 0)
        check(f"d{d}_latebump_has_misjudge",
              late["n_switch_false_rejects"] > 0)
        # 4) 诊断记录含路径版本号与服务端地图版本，且二者不同
        for pol_name, pr in (("buggy", buggy), ("latebump", late)):
            sample = pr["diagnostics_sample"]
            ok = bool(sample) and all(
                r["label_version"] != r["server_map_version"]
                and r["bad_on_own_map"] == 0
                and r["bad_on_eval_map"] > 0
                for r in sample)
            check(f"d{d}_{pol_name}_diagnostic_versions", ok)
        # 5) 反解：未截断时误差不超过 2 个采样 tick
        if not buggy["censored_by_rollback"]:
            err = abs(buggy["inferred_d_max_s"] - d)
            check(f"d{d}_inference_accuracy", err <= 2,
                  f"true={d} inferred={buggy['inferred_d_max_s']}")
        else:
            # 截断档：观测值被回滚窗口限制，全窗口还原值误差 <=2
            err_full = abs(buggy["inferred_d_max_full_s"] - d)
            check(f"d{d}_censored_full_recovery", err_full <= 2,
                  f"true={d} full={buggy['inferred_d_max_full_s']} "
                  f"observed={buggy['inferred_d_max_s']}")
            check(f"d{d}_censored_flag", buggy["censored_by_rollback"])

    # --- 严格程度曲线：FPR 单调不增、CIR 单调不增，且切换误伤恒为零 ---
    curve = data["strictness"]["curve"]
    fpr = [c["fpr"] for c in curve]
    cir = [c["cir"] for c in curve]
    check("strictness_fpr_monotone",
          all(fpr[i] >= fpr[i + 1] for i in range(len(fpr) - 1)))
    check("strictness_cir_monotone",
          all(cir[i] >= cir[i + 1] for i in range(len(cir) - 1)))
    check("strictness_zero_switch_misjudge",
          all(c["switch_false"] == 0 for c in curve))
    chosen = data["strictness"]["chosen"]
    check("chosen_point_fpr_budget", chosen["fpr"] <= 0.01)

    # --- 直接重跑一次 D=60 fixed，确认运行期零切换误伤（不依赖 JSON） ---
    rows, server, _, _ = simulate(60, "fixed", k=0)
    s = summarize(rows, server)
    check("rerun_fixed_zero_misjudge", s["switch_false_rejects"] == 0)
    check("rerun_no_spike", s["spike_t"] is None)

    print(f"\nall_ok={'1' if failures == 0 else '0'}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
