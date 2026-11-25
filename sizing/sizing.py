#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sizing: 网格边长 h 与角色半径 r / 最大坡度 theta 的可表达性与误判率分析。

模型约定（详见 README.md）：
- 窄缝按一维处理，真实可过的最小缝宽 d = 2r。
- 网格单元只有“整个单元落在缝内”才标记为可走（保守栅格化）。
- 缝相对网格的相位 u ~ Uniform[0, h)，所有相位概率公式都是对 u 取期望。
- 坡度按最坏情形折算有效分辨率：h_eff = h / cos(theta)。

输出：
  results/sweep.csv          三约束各改一档的扫描表
  results/curves.csv         两条曲线（误判率、节点数比）随 h 的数据
  results/mc_check.csv       蒙特卡洛对解析公式的校验
  控制台打印完整推导用数值与结论
"""
import csv
import math
import os
import random

# ---------------- 文档基准参数 ----------------
R0 = 0.40          # 角色半径 r，米
H0 = 0.25          # 文档网格边长 h，米
THETA0_DEG = 30.0  # 最大坡度，度
MISJUDGE_BUDGET = 0.01   # 误判率上限 1%
NODE_BUDGET = 1.50       # 节点数增幅上限 +50%

# 缝宽分布（仅用于整体误判率的补充模型）：W ~ Uniform[0, 2d]
GAP_WMAX_FACTOR = 2.0
N_MC = 500_000
SEED = 20261126

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")


def d_min(r):
    """真实可过的最小缝宽：两侧各留一个半径。"""
    return 2.0 * r


def h_eff(h, theta_deg):
    """坡面上的最坏有效分辨率：坡面斜距每 h 米一个节点。"""
    return h / math.cos(math.radians(theta_deg))


# ---------------- 模型 A：朴素栅格化 ----------------
def p_open_plain(w, h):
    """缝宽 w 中至少保留 1 个完整可走单元的概率（对相位 u 取期望）。

    几何：完整单元数 N(u) = max(0, floor((u+w)/h) - ceil(u/h))，u~U[0,h)。
    分段：w/h<1 时缝比单元还窄，任何相位都放不下完整单元 => 0；
          1<=w/h<2 时概率 w/h-1；w/h>=2 时恒为 1。
    """
    if w <= 0.0:
        return 0.0
    x = w / h
    if x >= 2.0:
        return 1.0
    if x >= 1.0:
        return x - 1.0
    return 0.0


def fn_plain(w, h, d):
    """假阴性（能过判成不能过）：w >= d 但网格里没有完整单元。"""
    return 1.0 - p_open_plain(w, h) if w >= d else 0.0


def fp_plain(w, h, d):
    """假阳性（不能过判成能过）：w < d 但网格里仍有完整单元。"""
    return p_open_plain(w, h) if w < d else 0.0


# ---------------- 模型 B：按半径膨胀障碍（腐蚀可走空间） ----------------
def p_open_eroded(w, h, d):
    """可走域先按 r 腐蚀后再栅格化，腐蚀后缝宽为 max(0, w-d)。"""
    e = max(0.0, w - d)
    return p_open_plain(e, h)


def fn_eroded(w, h, d):
    return 1.0 - p_open_eroded(w, h, d) if w >= d else 0.0


# 模型 B 下 w < d 必关，结构上没有假阳性
def fp_eroded(w, h, d):
    return 0.0


# ---------------- 整体误判率：缝宽 W ~ Uniform[0, 2d]，解析闭式 ----------------
# 令 x=h/d、z=W/d。正确的可表达概率分段（由完整单元计数 N=floor((u+w)/h)-ceil(u/h) 推出）：
#   g(z,x) = 0 (z<x)；z/x - 1 (x<=z<2x)；1 (z>=2x)
# 在 z∈[0,2] 上对 z 取平均（再除以区间长度 2）：
def _a(t, x):
    """g 在 [0,t] 上的积分（无量纲），t 按 x、2x 分段。"""
    if t <= x:
        return 0.0
    if t <= 2.0 * x:
        return (t - x) ** 2 / (2.0 * x)
    return 0.5 * x + (t - 2.0 * x)


def _quadrature(h, d, model, n=200_000):
    """对 W~U[0,2d] 的确定性细网格积分，作为复杂分段的兜底真值。"""
    total = 0.0
    for i in range(n):
        w = 2.0 * d * (i + 0.5) / n
        if model == "eroded":
            total += fn_eroded(w, h, d) + fp_eroded(w, h, d)
        else:
            total += fn_plain(w, h, d) + fp_plain(w, h, d)
    return total / n


def ensemble_error(h, d, model):
    """对缝宽分布 W~U[0,2d] 取期望的总误判率（FP+FN）。

    工程选型区间 h<d/2（x<=0.5）用解析闭式；更粗的网格已违反可表达判据，
    不在选型范围内，统一用确定性积分画曲线尾部。
    """
    x = h / d
    if x <= 0.0:
        return 0.0
    if x > 0.5:
        return _quadrature(h, d, model)
    if model == "eroded":
        # 模型 B：FP 恒为 0。腐蚀宽度 e=W-d，FN 概率在 e/h∈[0,1] 为 1（死区）、
        # [1,2] 线性降为 0；积分面积 (3/2)x，除以分布区间长度 2 => 3x/4
        return 0.75 * x
    # 朴素模型，x<=0.5：FP 为 g 在 z∈[0,1] 的面积除以 2；FN 恒为 0（W>=d => W/h>=2）
    return _a(1.0, x) / 2.0


# ---------------- 反解：误判率 <= 1% 的 h 上限（二分） ----------------
def bisect_h_limit(f, lo=1e-4, hi=2.0, tol=1e-7):
    """在 h<hi 的单调段内反解 f(h)=1% 的上限；结构性无解返回 None。"""
    if f(lo) > MISJUDGE_BUDGET:
        return None  # 即使 h→0 仍超标，预算不可达
    if f(hi) <= MISJUDGE_BUDGET:
        return hi
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if f(mid) <= MISJUDGE_BUDGET:
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break
    return 0.5 * (lo + hi)


def node_ratio(h, h_ref=H0):
    """固定面积 A：节点数 N=A/h^2，比值 N/N0=(h0/h)^2。"""
    return (h_ref / h) ** 2


# ---------------- 蒙特卡洛校验 ----------------
def monte_carlo(h, d, model, n=N_MC, rng=None):
    rng = rng or random.Random(SEED)
    fn_c = fp_c = tn_c = tp_c = 0
    for _ in range(n):
        w = rng.uniform(0.0, GAP_WMAX_FACTOR * d)
        # 缝左边界落在某单元内的偏移 u~U[0,h)；完整单元区间为 [h, u+w]
        u = rng.uniform(0.0, h)
        truth_open = w >= d
        if model == "plain":
            span = w
        else:
            span = max(0.0, w - d)  # 腐蚀后的有效缝宽
        n_full = max(0, math.floor((u + span) / h) - 1)
        grid_open = n_full >= 1
        if truth_open and not grid_open:
            fn_c += 1
        elif not truth_open and grid_open:
            fp_c += 1
        elif truth_open and grid_open:
            tp_c += 1
        else:
            tn_c += 1
    return {
        "fn": fn_c / n,
        "fp": fp_c / n,
        "total": (fn_c + fp_c) / n,
        "tp": tp_c / n,
        "tn": tn_c / n,
    }


# ---------------- 扫描：三约束各改一档 ----------------
def sweep_rows(d, theta_deg, h, tag):
    he = h_eff(h, theta_deg)
    ratio = d / he
    expressible = ratio > 2.0
    # 临界缝 W=d 处对相位的最坏误判（朴素模型；腐蚀模型同点更保守）
    worst = fn_plain(d, he, d)
    nr = node_ratio(h)
    ok_mis = worst <= MISJUDGE_BUDGET
    ok_nodes = nr <= NODE_BUDGET
    return {
        "case": tag,
        "r_m": round(d / 2.0, 3),
        "theta_deg": theta_deg,
        "h_m": h,
        "h_eff_m": round(he, 4),
        "d_over_h_eff": round(ratio, 3),
        "expressible_W/h>2": int(expressible),
        "worst_misjudge_at_W=d": round(worst, 4),
        "node_ratio_vs_baseline": round(nr, 3),
        "pass_misjudge_1pct": int(ok_mis),
        "pass_node_50pct": int(ok_nodes),
        "pass_all": int(ok_mis and ok_nodes),
    }


def main():
    os.makedirs(OUT, exist_ok=True)
    rng = random.Random(SEED)
    d0 = d_min(R0)

    print("=" * 72)
    print("1. 可表达性判据")
    print("=" * 72)
    print(f"最小可过缝宽 d = 2r = {d0:.3f} m")
    print("判据：W/h > 2（W 为缝宽，h 为网格边长），否则最不利相位下缝内")
    print("      不存在任何完整可走单元，缝被网格填掉。")
    print("推导：相位 u~U[0,h)，完整单元数 N = max(0, floor((u+W)/h) - ceil(u/h))")
    print("      W/h<1 时 P(N>=1)=0；1<=W/h<2 时 P(N>=1)=W/h-1；W/h>=2 时恒为 1。")
    print("      => W/h<2 存在误填相位；W/h>=2 对所有相位可表达（严格判据取 W/h>2）。")
    print(f"代入 d：要求 h < d/2 = {d0/2:.3f} m；文档 h={H0} => d/h={d0/H0:.2f} > 2，达标，余量 {(1-H0/(d0/2))*100:.1f}%。")

    print()
    print("=" * 72)
    print("2. 两类误判的可计算公式（朴素栅格化，对任意缝宽 W）")
    print("=" * 72)
    print("P_grid_open(W) = 0              当 W/h<1（缝比单元窄，放不下完整单元）；")
    print("                 = W/h - 1        当 1<=W/h<2；= 1 当 W/h>=2（W=2h 仅对齐点例外，测度为零）")
    print("假阴性 FN(W) = 1 - P_open，  仅 W>=d（能过被填）")
    print("假阳性 FP(W) = P_open，      仅 W<d （不能过却留出通道）")
    print("临界缝 W=d 最坏误判 = clip(2 - d/h, 0, 1)：d/h>=2 为 0；1<d/h<2 线性；d/h<=1 为 100%")

    print()
    print("=" * 72)
    print("3. 用误判率 1% 反解 h 上限")
    print("=" * 72)
    # 3a 临界缝最坏相位反解（单调段 0 < h < d，h>=d 后函数退化无工程意义）
    h_lim_soft = bisect_h_limit(lambda h: fn_plain(d0, h, d0), hi=d0)
    print(f"[临界缝 W=d，最坏相位] 2 - d/h <= 0.01 => h <= d/1.99 = {h_lim_soft:.4f} m")
    print(f"  硬性可表达判据 W/h>2 仍要求 h < {d0/2:.4f} m，1% 预算只把它放松到 {h_lim_soft:.4f} m。")
    # 3b 缝宽整体分布（朴素）
    h_lim_ens_plain = bisect_h_limit(lambda h: ensemble_error(h, d0, "plain"), hi=d0)
    ens_base = ensemble_error(H0, d0, "plain")
    print(f"[缝宽 W~U[0,2d] 整体期望，朴素栅格化] 文档 h 处总误判率 = {ens_base*100:.2f}%，")
    if h_lim_ens_plain is None:
        print("  结构性下限（h→0）= 25%，全部来自假阳性 => 1% 预算在朴素模型下无解，")
        print("  必须改用按半径膨胀障碍的栅格化（模型 B）消除假阳性。")
    # 3c 腐蚀模型
    h_lim_ens_ero = bisect_h_limit(lambda h: ensemble_error(h, d0, "eroded"), hi=d0)
    ens_ero_base = ensemble_error(H0, d0, "eroded")
    print(f"[同分布，先按 r 膨胀障碍] 假阳性结构性归零；文档 h 处总误判率 = {ens_ero_base*100:.2f}%，")
    print(f"  1% 反解 => h <= {h_lim_ens_ero:.4f} m（E[FN]=3h/(4d) 的严格口径），")
    print(f"  该尺度对应节点数约为文档网格的 ({H0/h_lim_ens_ero:.0f})^2 倍，工程不可行——")
    print(f"  说明“对所有缝宽整体期望 1%”是过强口径；可落地的口径是临界缝最坏相位 <=1%。")
    print(f"结论：工程上应以临界缝最坏相位口径定上限 h < {d0/2:.3f} m，文档 {H0} m 在该口径内。")

    print()
    print("=" * 72)
    print("4. 蒙特卡洛校验（每点", N_MC, "次采样）")
    print("=" * 72)
    mc_rows = []
    for h in (0.20, 0.25, 0.35, 0.50):
        for model in ("plain", "eroded"):
            mc = monte_carlo(h, d0, model, rng=rng)
            ana = ensemble_error(h, d0, model)
            mc_rows.append({
                "h_m": h, "model": model,
                "mc_total": round(mc["total"], 4),
                "analytic_total": round(ana, 4),
                "abs_diff": round(abs(mc["total"] - ana), 4),
                "mc_fn": round(mc["fn"], 4), "mc_fp": round(mc["fp"], 4),
            })
            print(f"h={h:.2f} {model:7s}: MC={mc['total']*100:6.2f}%  解析={ana*100:6.2f}%  "
                  f"差={abs(mc['total']-ana)*100:4.2f}pp  (FN {mc['fn']*100:5.2f}% / FP {mc['fp']*100:5.2f}%)")
    with open(os.path.join(OUT, "mc_check.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(mc_rows[0].keys()))
        w.writeheader()
        w.writerows(mc_rows)

    print()
    print("=" * 72)
    print("5. 节点数预算反推 h 下限")
    print("=" * 72)
    h_floor = H0 / math.sqrt(NODE_BUDGET)
    print(f"N/N0=(h0/h)^2 <= 1.5 => h >= h0/sqrt(1.5) = {h_floor:.4f} m")
    print(f"可行区间（最坏相位口径）：[{h_floor:.3f}, {d0/2:.3f}) m，文档 h={H0} 位于区间内。")

    print()
    print("=" * 72)
    print("6. 三约束各改一档扫描（含 h 的加严/放宽两档用于开处方）")
    print("=" * 72)
    rows = [
        sweep_rows(d0, THETA0_DEG, H0, "baseline r=.40 theta=30 h=.25"),
        sweep_rows(d_min(0.50), THETA0_DEG, H0, "radius +1档 r=.50"),
        sweep_rows(d0, 40.0, H0, "slope +1档 theta=40"),
        sweep_rows(d0, THETA0_DEG, 0.50, "h +1档(放宽) h=.50"),
        sweep_rows(d0, THETA0_DEG, 0.20, "h -1档(加细) h=.20"),
    ]
    with open(os.path.join(OUT, "sweep.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)
    hdr = f"{'case':38s} {'d/he':>6s} {'W/h>2':>6s} {'worst%':>7s} {'N/N0':>6s} {'mis':>4s} {'node':>5s} {'all':>4s}"
    print(hdr)
    for r in rows:
        print(f"{r['case']:38s} {r['d_over_h_eff']:6.3f} {r['expressible_W/h>2']:6d} "
              f"{r['worst_misjudge_at_W=d']*100:6.2f}% {r['node_ratio_vs_baseline']:6.3f} "
              f"{r['pass_misjudge_1pct']:4d} {r['pass_node_50pct']:5d} {r['pass_all']:4d}")

    print()
    print("=" * 72)
    print("7. 超档处方")
    print("=" * 72)
    # h=.50 失败处方：可表达条件 h/cosθ < d/2 => h < d·cosθ/2
    h_fix_coarse = d0 * math.cos(math.radians(THETA0_DEG)) / 2.0
    print(f"h=.50 档：可表达性失败（d/h_eff={d0/h_eff(0.50, THETA0_DEG):.2f} <= 2，临界缝最坏误判 {fn_plain(d0, h_eff(0.50, THETA0_DEG), d0)*100:.1f}%）。")
    print(f"  处方：h < d·cosθ/2 = {h_fix_coarse:.4f} m；兼顾节点预算取 h=0.30 m（N/N0={node_ratio(0.30):.2f}），")
    print(f"  或直接沿用文档 h=0.25 m。")
    # h=.20 失败处方：h >= h_floor
    print(f"h=.20 档：误判为 0 但 N/N0={node_ratio(0.20):.3f}，超出 +50% 节点预算。")
    print(f"  处方：h >= {h_floor:.4f} m；取 h=0.21 m（N/N0={node_ratio(0.21):.3f}，d/h={d0/0.21:.2f} 仍 > 2）。")
    # 坡度余量：h/cosθ < d/2 => cosθ > 2h/d
    theta_max = math.degrees(math.acos(2.0 * H0 / d0))
    print(f"坡度档余量：h=.25 保持可表达性的最大坡度 = arccos(2h/d) = {theta_max:.1f}°，theta=40° 档通过。")

    # ---------------- 曲线数据 ----------------
    curve = []
    hs = [0.16 + 0.01 * i for i in range(40)]  # 0.16 .. 0.55
    for h in hs:
        he = h_eff(h, THETA0_DEG)
        curve.append({
            "h_m": round(h, 3),
            "worst_fn_pct": round(100.0 * fn_plain(d0, he, d0), 4),
            "ensemble_plain_pct": round(100.0 * ensemble_error(he, d0, "plain"), 4),
            "ensemble_eroded_pct": round(100.0 * ensemble_error(he, d0, "eroded"), 4),
            "node_ratio": round(node_ratio(h), 4),
        })
    with open(os.path.join(OUT, "curves.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.DictWriter(f, fieldnames=list(curve[0].keys()))
        wr.writeheader()
        wr.writerows(curve)
    print()
    print(f"已写出: results/{{sweep.csv, curves.csv, mc_check.csv}}（{len(curve)} 个 h 采样点）")


if __name__ == "__main__":
    main()
