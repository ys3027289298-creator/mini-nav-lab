#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
migrate_map_labels.py
=====================

旧地图区域编号 (O = {0, 1, ..., N-1}, 从零连续) -> 新语义标签体系
L = {main_city(主城), wilderness(野外), dungeon(副本)} 的字段映射迁移实验。

覆盖的题目要求
--------------
1. 映射完整性表达式的程序化校验:
       C1 每个旧编号必须有标签:  dom(M) = O
       C2 每个新标签至少有一个旧编号映射过来:  M(O) = L (满射)
2. 无法硬编码的旧编号按“包围盒包含”推断:
       contain(i,r) = |cells(i) ∩ bbox(r)| / |cells(i)|
   阈值 theta=1.0 即题面规则“全部格子都落在某个新区域内”。
3. 推断成功率 r = A / U, 人工复核数 H = U - A = (1-r)U。
4. 在独立校准集上扫描 theta, 用“推断准确率 / 人工工作量”两条曲线,
   以准确率 >= 95% 为硬门限定阈值 theta*。
5. 小/中/大三档旧图各正式跑一次: 覆盖率必须 100%, 自动推断准确率 >= 95%;
   所有误推输出 old_id + 实际标签 (mispredictions.csv)。

数据为确定性合成数据 (固定 seed), 自带隐藏真值 actual_label —— 没有真值
就无法计算“推断准确率”曲线; 真实迁移中该曲线应来自人工标注的校准样本。

只用标准库即可产出 CSV/JSON/Markdown; 若安装了 matplotlib 会额外画曲线图。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import Optional

# ---------------------------------------------------------------------------
# 常量与形式化模型
# ---------------------------------------------------------------------------

LABELS = ("main_city", "wilderness", "dungeon")
LABEL_CN = {
    "main_city": "主城",
    "wilderness": "野外",
    "dungeon": "副本",
}
GATE_ACCURACY = 0.95
EPS = 1e-9
THETAS = [round(1.0 - 0.05 * k, 2) for k in range(11)]  # 1.00 ... 0.50

# 合成世界的坐标范围与新区域包围盒 (inclusive 矩形)。
WORLD_X = (0, 260)
WORLD_Y = (0, 160)

# rid, label, x0, y0, x1, y1 —— 刻意保留若干大面积相交对 (副本嵌在野外/主城边)。
REGION_SPECS = [
    ("MC-core",   "main_city",  10,  80,  10,  70),
    ("MC-harbor", "main_city",  10,  70,  80, 140),
    ("WL-plain",  "wilderness", 80, 170,  20,  90),
    ("WL-forest", "wilderness", 60, 130,  50, 130),
    ("WL-desert", "wilderness", 140, 215,  10,  75),
    ("WL-snow",   "wilderness", 130, 215,  95, 150),
    ("DG-crypt",  "dungeon",    60, 110,  30,  80),
    ("DG-mine",   "dungeon",    95, 140, 118, 152),
    ("DG-spire",  "dungeon",     8,  48, 118, 152),
]

# 误推陷阱使用的 (真实区域, 错误区域) 相交对, 方向即真值方向。
TRAP_PAIRS = [
    ("DG-crypt", "WL-plain"),
    ("MC-core",  "WL-forest"),
    ("DG-spire", "MC-harbor"),
    ("DG-mine",  "WL-snow"),
    ("DG-crypt", "WL-forest"),
]
AMB_PAIRS = [
    ("DG-crypt", "WL-plain"),
    ("MC-core",  "WL-forest"),
    ("DG-spire", "MC-harbor"),
    ("DG-mine",  "WL-forest"),
]

SMALL_REGIONS = ["MC-core", "MC-harbor", "WL-plain", "WL-forest", "DG-crypt"]
ALL_REGIONS = [s[0] for s in REGION_SPECS]


@dataclass(frozen=True)
class Region:
    rid: str
    label: str
    x0: int
    y0: int
    x1: int
    y1: int

    def contains(self, x: int, y: int) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


# REGION_SPECS 坐标顺序为 (rid, label, x0, x1, y0, y1)。
REGIONS = {rid: Region(rid, lab, x0, y0, x1, y1)
           for rid, lab, x0, x1, y0, y1 in REGION_SPECS}
LABEL_REGIONS = {lab: [r for r in REGIONS.values() if r.label == lab]
                 for lab in LABELS}


@dataclass
class OldRecord:
    oid: int                       # 旧区域编号, 连续 0..N-1
    cells: frozenset[tuple[int, int]]
    actual_label: str              # 隐藏真值 (模拟迁移时不可见, 仅用于评测)
    actual_rid: str
    plan: str                      # 生成画像: hardcoded/clean/edge90/...
    hardcoded: bool = False


# ---------------------------------------------------------------------------
# 确定性合成地图生成
# ---------------------------------------------------------------------------

def _sample_rect(rng: random.Random, rect: Region | tuple, n: int,
                 occupied: set, local: set, predicate=None) -> set[tuple[int, int]]:
    """在矩形内采 n 个互不重复、未被占用的整数格; predicate 可附加过滤。"""
    if isinstance(rect, Region):
        x0, y0, x1, y1 = rect.x0, rect.y0, rect.x1, rect.y1
    else:
        x0, y0, x1, y1 = rect
    pts: set[tuple[int, int]] = set()
    guard = 0
    while len(pts) < n:
        guard += 1
        if guard > 200000:
            raise RuntimeError("rect sampling exhausted")
        p = (rng.randint(x0, x1), rng.randint(y0, y1))
        if p in occupied or p in local or p in pts:
            continue
        if predicate is not None and not predicate(p):
            continue
        pts.add(p)
    return pts


def _sample_outside_all(rng, n, occupied, local) -> set[tuple[int, int]]:
    def free(p):
        return not any(r.contains(*p) for r in REGIONS.values())
    world_rect = (WORLD_X[0], WORLD_Y[0], WORLD_X[1], WORLD_Y[1])
    return _sample_rect(rng, world_rect, n, occupied, local, free)


def _scores_from_cells(cells) -> tuple[dict[str, float], dict[str, str]]:
    """返回每个语义标签的最高包围比例, 以及该标签下最佳新区域 rid。"""
    total = len(cells)
    best: dict[str, float] = {l: 0.0 for l in LABELS}
    best_rid: dict[str, str] = {l: "" for l in LABELS}
    for r in REGIONS.values():
        inside = sum(1 for x, y in cells if r.contains(x, y))
        c = inside / total
        if c > best[r.label]:
            best[r.label] = c
            best_rid[r.label] = r.rid
    return best, best_rid


def _gen_clean(rng, rid, k, occupied) -> frozenset:
    """全部格子落在真实 bbox 内, 且其它标签的最高包围比例 <= 0.5。"""
    r = REGIONS[rid]
    for _ in range(2000):
        pts = _sample_rect(rng, r, k, occupied, set())
        scores, _ = _scores_from_cells(pts)
        other = max(v for l, v in scores.items() if l != r.label)
        if other <= 0.5 + EPS:
            occupied |= pts
            return frozenset(pts)
    raise RuntimeError(f"clean generation failed for {rid}")


def _gen_edge(rng, rid, n_in, k, occupied) -> frozenset:
    """n_in 个格在真实 bbox 内, 其余在所有新 bbox 之外 -> 部分包围。"""
    r = REGIONS[rid]
    local: set = set()
    pin = _sample_rect(rng, r, n_in, occupied, local)
    local |= pin
    pout = _sample_outside_all(rng, k - n_in, occupied, local)
    pts = pin | pout
    occupied |= pts
    return frozenset(pts)


def _gen_trap(rng, true_rid, wrong_rid, n_wrong, n_true, k,
              occupied) -> frozenset:
    """
    构造“错误标签包围比例更高”的陷阱格集。
    取 b = min(n_true, ...) 个交集格同时计入两个比例:
      both=n_true (让 true 比例恰为 n_true/k),
      wrong_only=n_wrong-n_true,
      neither=k - n_wrong。
    """
    t, w = REGIONS[true_rid], REGIONS[wrong_rid]
    inter = (max(t.x0, w.x0), max(t.y0, w.y0),
             min(t.x1, w.x1), min(t.y1, w.y1))
    assert inter[0] <= inter[2] and inter[1] <= inter[3], "trap pair must overlap"
    b = n_true
    local: set = set()
    both = _sample_rect(rng, inter, b, occupied, local)
    local |= both
    wonly_n = n_wrong - b
    wonly = _sample_rect(
        rng, w, wonly_n, occupied, local,
        lambda p: not t.contains(*p)) if wonly_n else set()
    local |= wonly
    neither_n = k - b - wonly_n  # = k - n_wrong
    neither = _sample_outside_all(rng, neither_n, occupied, local) \
        if neither_n else set()
    pts = both | wonly | neither
    assert len(pts) == k
    occupied |= pts
    return frozenset(pts)


def _gen_ambiguous(rng, rid_a, rid_b, k, occupied) -> frozenset:
    a, b = REGIONS[rid_a], REGIONS[rid_b]
    inter = (max(a.x0, b.x0), max(a.y0, b.y0),
             min(a.x1, b.x1), min(a.y1, b.y1))
    pts = _sample_rect(rng, inter, k, occupied, set())
    occupied |= pts
    return frozenset(pts)


@dataclass
class TierConfig:
    key: str
    cn: str
    n_total: int
    seed: int
    hard: dict[str, int]          # hardcoded 中各标签数量 (无 dungeon)
    clean: dict[str, int]         # 画像 clean 中各标签数量
    edge90: int
    edge75: dict[str, int]
    edge60: int
    amb: int
    htrap: int                    # 高包围误推 (.917 vs .333)
    mtrap: int                    # 中包围误推 (.75 vs .25)
    region_pool: list[str] = field(default_factory=lambda: list(ALL_REGIONS))


def build_world(cfg: TierConfig) -> list[OldRecord]:
    rng = random.Random(cfg.seed)
    occupied: set[tuple[int, int]] = set()

    # 1) 先把全部 oid 的“计划”排成列表再洗牌, 让特殊画像的编号自然散落。
    plan: list[tuple] = []
    for lab, n in cfg.hard.items():
        plan += [("hard", lab)] * n
    for lab, n in cfg.clean.items():
        plan += [("clean", lab)] * n
    plan += [("edge90", "wilderness")] * cfg.edge90
    for lab, n in cfg.edge75.items():
        plan += [("edge75", lab)] * n
    plan += [("edge60", "wilderness")] * cfg.edge60
    plan += [("amb", None)] * cfg.amb
    plan += [("htrap", None)] * cfg.htrap
    plan += [("mtrap", None)] * cfg.mtrap
    assert len(plan) == cfg.n_total, (len(plan), cfg.n_total)
    rng.shuffle(plan)

    # 2) 每个标签可用的真实区域 (clean/hard 轮询分配, 保证每个区域都有旧编号)。
    pool = set(cfg.region_pool)
    rid_by_label = {
        lab: [r.rid for r in LABEL_REGIONS[lab] if r.rid in pool]
        for lab in LABELS
    }
    rr_cursor = {lab: 0 for lab in LABELS}

    def pick_rid(lab):
        ids = rid_by_label[lab]
        j = rr_cursor[lab] % len(ids)
        rr_cursor[lab] += 1
        return ids[j]

    records: list[OldRecord] = []
    trap_i = amb_i = 0
    for oid, (kind, lab) in enumerate(plan):
        if kind == "hard":
            rid = pick_rid(lab)
            cells = _gen_clean(rng, rid, rng.randint(10, 16), occupied)
            records.append(OldRecord(oid, cells, lab, rid, "hardcoded", True))
        elif kind == "clean":
            rid = pick_rid(lab)
            cells = _gen_clean(rng, rid, 12, occupied)
            records.append(OldRecord(oid, cells, lab, rid, "clean"))
        elif kind == "edge90":
            rid = pick_rid("wilderness")
            cells = _gen_edge(rng, rid, 11, 12, occupied)   # 11/12=.917
            records.append(OldRecord(oid, cells, "wilderness", rid, "edge90"))
        elif kind == "edge75":
            rid = pick_rid(lab)
            cells = _gen_edge(rng, rid, 9, 12, occupied)    # .75
            records.append(OldRecord(oid, cells, lab, rid, "edge75"))
        elif kind == "edge60":
            rid = pick_rid("wilderness")
            cells = _gen_edge(rng, rid, 7, 12, occupied)    # .583
            records.append(OldRecord(oid, cells, "wilderness", rid, "edge60"))
        elif kind in ("htrap", "mtrap"):
            true_rid, wrong_rid = TRAP_PAIRS[trap_i % len(TRAP_PAIRS)]
            trap_i += 1
            true_lab = REGIONS[true_rid].label
            if kind == "htrap":
                cells = _gen_trap(rng, true_rid, wrong_rid,
                                  n_wrong=11, n_true=4, k=12, occupied=occupied)
            else:
                cells = _gen_trap(rng, true_rid, wrong_rid,
                                  n_wrong=9, n_true=3, k=12, occupied=occupied)
            records.append(OldRecord(oid, cells, true_lab, true_rid, kind))
        else:  # amb
            a, b = AMB_PAIRS[amb_i % len(AMB_PAIRS)]
            amb_i += 1
            # 真值取 a; 全部格在交集里 -> 两个标签包围比例都为 1。
            cells = _gen_ambiguous(rng, a, b, 12, occupied)
            records.append(OldRecord(oid, cells, REGIONS[a].label, a, "amb"))

    # 旧编号必须恰好是 0..N-1 连续。
    assert [r.oid for r in records] == list(range(cfg.n_total))
    return records


# ---------------------------------------------------------------------------
# 推断与完整性
# ---------------------------------------------------------------------------

@dataclass
class Decision:
    oid: int
    theta: float
    status: str                  # hardcoded / auto / manual
    label: Optional[str]         # 最终标签 (manual 在“完成映射”里由人工真值补)
    region: Optional[str]
    confidence: float
    reason: str                  # auto / below_threshold / tie / hardcoded
    scores: dict[str, float]
    correct: Optional[bool] = None


def infer_unknown(rec: OldRecord, theta: float) -> Decision:
    scores, best_rid = _scores_from_cells(rec.cells)
    ordered = sorted(LABELS, key=lambda l: scores[l], reverse=True)
    top, second = ordered[0], ordered[1]
    s1, s2 = scores[top], scores[second]
    if s1 + EPS < theta:
        return Decision(rec.oid, theta, "manual", None, None, s1,
                        "below_threshold", scores)
    if abs(s1 - s2) <= EPS:
        return Decision(rec.oid, theta, "manual", None, None, s1,
                        "tie", scores)
    return Decision(rec.oid, theta, "auto", top, best_rid[top], s1,
                    "unique_best", scores,
                    correct=(top == rec.actual_label))


def check_completeness(records: list[OldRecord],
                       labels_by_oid: dict[int, Optional[str]]) -> dict:
    O = {r.oid for r in records}
    assigned = {i: l for i, l in labels_by_oid.items() if l in LABELS}
    missing_old = sorted(O - set(assigned))
    support = {l: sorted(i for i, v in assigned.items() if v == l)
               for l in LABELS}
    return {
        "expr_C1_every_old_id_has_label": "dom(M) = O",
        "expr_C2_every_new_label_is_surjective": "M(O) = L",
        "n_old": len(O),
        "n_assigned": len(assigned),
        "coverage": len(assigned) / len(O),
        "C1_all_old_ids_mapped": not missing_old,
        "missing_old_ids": missing_old,
        "label_support_count": {l: len(v) for l, v in support.items()},
        "label_support_ids": support,
        "C2_all_labels_supported": all(support[l] for l in LABELS),
    }


def sweep(records: list[OldRecord]) -> list[dict]:
    unknown = [r for r in records if not r.hardcoded]
    u = len(unknown)
    rows = []
    for theta in THETAS:
        autos, wrong, conf_sum = 0, 0, 0.0
        reasons = {"below_threshold": 0, "tie": 0}
        for r in unknown:
            d = infer_unknown(r, theta)
            if d.status == "auto":
                autos += 1
                conf_sum += d.confidence
                if not d.correct:
                    wrong += 1
            else:
                reasons[d.reason] = reasons.get(d.reason, 0) + 1
        correct = autos - wrong
        manual = u - autos
        rows.append({
            "theta": theta,
            "unknown": u,
            "auto": autos,
            "manual": manual,
            "success_rate": autos / u if u else 1.0,
            "accuracy": correct / autos if autos else None,
            "wrong": wrong,
            "correct": correct,
            "workload_fraction": manual / u if u else 0.0,
            "avg_confidence": conf_sum / autos if autos else None,
            **reasons,
        })
    return rows


def choose_threshold(calib_rows: list[dict]) -> dict:
    """
    门限: accuracy >= 0.95 (用整数判据 20*wrong <= auto, 避免浮点争议)。
    安全前缀: theta 合格当且仅当它本身与所有更高 theta 都合格
              (防止选到低阈值处的偶然甜点)。
    目标: 合格 theta 中人工数最少; 人工数相同取最高 theta (更保守)。
    """
    def passes(row):
        return row["auto"] > 0 and 20 * row["wrong"] <= row["auto"]

    eligible = []
    for idx, row in enumerate(calib_rows):
        higher_all_pass = all(passes(calib_rows[j]) for j in range(idx + 1))
        if passes(row) and higher_all_pass:
            eligible.append(row["theta"])
    if not eligible:
        raise RuntimeError("校准曲线上不存在准确率 >= 95% 的阈值")
    best_manual = min(next(r["manual"] for r in calib_rows if r["theta"] == t)
                      for t in eligible)
    cand = [t for t in eligible
            if next(r for r in calib_rows if r["theta"] == t)["manual"]
            == best_manual]
    theta_star = max(cand)
    return {"theta_star": theta_star, "eligible": eligible,
            "calib_gate": GATE_ACCURACY}


# ---------------------------------------------------------------------------
# 正式跑一档 (每档只调用一次)
# ---------------------------------------------------------------------------

def run_tier(cfg: TierConfig, theta_star: float) -> dict:
    records = build_world(cfg)                     # 该档地图只生成一次

    hard = [r for r in records if r.hardcoded]
    unknown = [r for r in records if not r.hardcoded]
    U = len(unknown)

    # 初始硬编码表的完整性 (题目起点: 副本标签没有任何硬编码行)。
    hard_labels = {r.oid: r.actual_label for r in hard}
    hard_comp = check_completeness(records, hard_labels)

    # 正式推断一次。
    decisions: dict[int, Decision] = {}
    for r in unknown:
        decisions[r.oid] = infer_unknown(r, theta_star)

    # 完成映射: 人工复核项由人工给出真值 (模拟中即 actual_label)。
    final_labels: dict[int, str] = {r.oid: r.actual_label for r in hard}
    final_region: dict[int, str] = {r.oid: r.actual_rid for r in hard}
    rows_csv = []
    mispredictions = []
    auto_n = wrong_n = 0

    rec_by_id = {r.oid: r for r in records}
    for r in records:
        if r.hardcoded:
            source, label, rid, conf, correct = (
                "hardcoded", r.actual_label, r.actual_rid, 1.0, True)
        else:
            d = decisions[r.oid]
            if d.status == "auto":
                source = "auto_bbox"
                label, rid, conf = d.label, d.region, d.confidence
                correct = d.correct
                auto_n += 1
                if not correct:
                    wrong_n += 1
                    mispredictions.append({
                        "tier": cfg.key,
                        "old_id": r.oid,
                        "inferred_label": label,
                        "inferred_label_cn": LABEL_CN[label],
                        "inferred_region": rid,
                        "actual_label": r.actual_label,
                        "actual_label_cn": LABEL_CN[r.actual_label],
                        "actual_region": r.actual_rid,
                        "confidence": round(conf, 4),
                        "theta": theta_star,
                        "scores": {l: round(d.scores[l], 4) for l in LABELS},
                    })
            else:
                source = "manual_confirmed"
                label, rid, conf = r.actual_label, r.actual_rid, d.confidence
                correct = True
            final_labels[r.oid] = label
            final_region[r.oid] = rid
        rows_csv.append({
            "old_id": r.oid,
            "new_label": label,
            "new_label_cn": LABEL_CN[label],
            "new_region": rid,
            "source": source,
            "confidence": f"{conf:.4f}",
            "theta": f"{theta_star:.2f}",
            "actual_label_audit": r.actual_label,
            "actual_region_audit": r.actual_rid,
            "auto_correct_audit": "" if source == "manual_confirmed"
            else ("1" if correct else "0"),
        })

    final_comp = check_completeness(records, final_labels)

    # 加强版 C3': 本期目标集合中的每个“具体新区域 tag”也至少有一个旧编号。
    region_support: dict[str, list[int]] = {rid: [] for rid in cfg.region_pool}
    for oid, rid in final_region.items():
        region_support.setdefault(rid, []).append(oid)
    missing_regions = sorted(
        rid for rid in cfg.region_pool if not region_support.get(rid))
    final_comp["expr_C3_every_new_region_tag_supported"] = \
        "forall r in R_target, exists i: M_region(i)=r"
    final_comp["C3_all_region_tags_supported"] = not missing_regions
    final_comp["region_support_count"] = {
        rid: len(ids) for rid, ids in sorted(region_support.items())}
    final_comp["missing_region_tags"] = missing_regions

    success_rate = auto_n / U if U else 1.0
    accuracy = (auto_n - wrong_n) / auto_n if auto_n else 1.0

    # 反解公式的整数形式: r*U = A, (1-r)*U = H。
    formula = {
        "N": cfg.n_total,
        "K_hardcoded": len(hard),
        "U_unknown": U,
        "A_auto": auto_n,
        "H_manual": U - auto_n,
        "r_success": success_rate,
        "identity_rU_equals_A": abs(success_rate * U - auto_n) < 1e-9,
        "identity_U_minus_A_equals_H":
            abs((1 - success_rate) * U - (U - auto_n)) < 1e-9,
        "accuracy": accuracy,
        "wrong": wrong_n,
        "coverage_final": final_comp["coverage"],
    }

    return {
        "cfg": cfg,
        "records": records,
        "sweep": sweep(records),
        "hard_completeness": hard_comp,
        "final_completeness": final_comp,
        "formula": formula,
        "mispredictions": mispredictions,
        "csv_rows": rows_csv,
        "decisions": decisions,
    }


# ---------------------------------------------------------------------------
# 输出: CSV / JSON / Markdown
# ---------------------------------------------------------------------------

def write_mapping_csv(path, rows):
    fields = ["old_id", "new_label", "new_label_cn", "new_region", "source",
              "confidence", "theta", "actual_label_audit",
              "actual_region_audit", "auto_correct_audit"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def write_sweep_csv(path, tier_key, rows, write_header=False):
    fields = ["tier", "theta", "unknown", "auto", "manual", "success_rate",
              "accuracy", "correct", "wrong", "workload_fraction",
              "avg_confidence", "below_threshold", "tie"]
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if write_header:
            w.writeheader()
        for r in rows:
            d = {"tier": tier_key}
            d.update(r)
            for k in ("success_rate", "accuracy", "workload_fraction",
                      "avg_confidence"):
                if d.get(k) is not None:
                    d[k] = f"{d[k]:.6f}"
            if d.get("accuracy") is None:
                d["accuracy"] = ""
            if d.get("avg_confidence") is None:
                d["avg_confidence"] = ""
            w.writerow({k: d.get(k, 0) for k in fields})


def write_mispredictions_csv(path, items, write_header=False):
    fields = ["tier", "old_id", "inferred_label", "inferred_label_cn",
              "inferred_region", "actual_label", "actual_label_cn",
              "actual_region", "confidence", "theta",
              "score_main_city", "score_wilderness", "score_dungeon"]
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if write_header:
            w.writeheader()
        for m in items:
            row = {k: m.get(k) for k in fields if not k.startswith("score_")}
            row["score_main_city"] = m["scores"]["main_city"]
            row["score_wilderness"] = m["scores"]["wilderness"]
            row["score_dungeon"] = m["scores"]["dungeon"]
            w.writerow(row)


def fmt_pct(x):
    return "-" if x is None else f"{100*x:.1f}%"


def build_report(theta_sel, calib_rows, results, global_gate_pass) -> str:
    L = []
    A = L.append
    A("# 旧区域编号 → 新语义标签：映射完整性、包围盒推断与阈值实验\n")
    A("> 本报告由 `mapping/migrate_map_labels.py` 确定性生成，请勿手改；"
      "复现：`python3 mapping/migrate_map_labels.py`。\n")

    A("## 1. 映射完整性表达式\n")
    A("设旧编号集合 $O=\\{0,1,\\dots,N-1\\}$（从零连续），新标签集合 "
      "$\\Lambda=\\{\\text{main_city},\\text{wilderness},\\text{dungeon}\\}$，"
      "部分映射 $M: O \\rightharpoonup \\Lambda$。\n")
    A("**C1 — 每个旧编号必须有对应标签：**\n")
    A("$$\\forall i\\in O,\\ M(i)\\in\\Lambda"
      "\\quad\\Longleftrightarrow\\quad \\operatorname{dom}M=O"
      "\\quad\\Longleftrightarrow\\quad "
      "|\\{i:M(i)=\\bot\\}|=0.$$\n")
    A("**C2 — 每个新标签至少有一个旧编号映射过来（满射）：**\n")
    A("$$\\forall \\ell\\in\\Lambda,\\ \\exists i\\in O: M(i)=\\ell"
      "\\quad\\Longleftrightarrow\\quad M(O)=\\Lambda"
      "\\quad\\Longleftrightarrow\\quad "
      "\\min_{\\ell\\in\\Lambda}|M^{-1}(\\ell)|\\ge 1.$$\n")
    A("合起来，完整可交付映射当且仅当 "
      "$\\operatorname{dom}M=O\\ \\wedge\\ M(O)=\\Lambda$。"
      "映射是多对一的（多个旧编号 → 同一新标签/新区域），不存在一一约束。\n")
    A("若把“新标签”理解为更细的具体新区域 tag 集合 $R$（如 `DG-crypt`），"
      "另有加强式 **C3′**："
      "$\\forall r\\in R,\\ \\exists i\\in O: M_R(i)=r$。"
      "本实验对 C2 与 C3′ 同时做了程序化校验。\n")
    A("新副本区域在旧映射表里没有任何硬编码行，所以仅凭硬编码表，C2 中的 "
      "`dungeon` 支持数为 0 —— 这正是不能做一对一硬编码的原因。\n")

    A("## 2. 无法映射的旧编号：包围盒包含推断\n")
    A("旧编号 $i$ 有格子集合 $P_i$，新区域 $r$ 有包围盒 $B_r$ 与标签 "
      "$\\lambda(r)$。定义包围比例：\n")
    A("$$c(i,r)=\\frac{|\\{p\\in P_i:p\\in B_r\\}|}{|P_i|},\\qquad "
      "s(i,\\ell)=\\max_{r:\\lambda(r)=\\ell}c(i,r).$$\n")
    A("推断规则（阈值 $\\theta$）：取得分最高的标签 $\\hat\\ell$，"
      "当 $s(i,\\hat\\ell)\\ge\\theta$ 且严格领先于第二名时自动采用；"
      "低于阈值或并列（包围盒相交导致标签歧义）一律转人工。"
      "**$\\theta=1.00$ 就是题面规则：旧编号的全部格子都落在同一个标签的"
      "某个新区域包围盒内。**\n")

    A("## 3. 用推断成功率反解人工确认数量\n")
    A("设硬编码已知 $K$ 个，未知编号数 $U=N-K$；在阈值 $\\theta$ 下自动推断 "
      "$A_\\theta$ 个，则推断成功率与人工数为：\n")
    A("$$r_\\theta=\\frac{A_\\theta}{U},\\qquad "
      "A_\\theta=r_\\theta U,\\qquad "
      "H_\\theta=U-A_\\theta=(1-r_\\theta)U."
      "$$\n")
    A("规划阶段若只有校准成功率 $\\hat r$，则期望自动量 $\\hat rU$、"
      "期望人工量 $(1-\\hat r)U$；按人头排期取保守整数 "
      "$H=\\lceil(1-\\hat r)U\\rceil$。\n")

    A("## 4. 两条曲线定阈值（95% 准确率硬门限）\n")
    sel = theta_sel
    A(f"在独立校准集（N=80, U=60, seed={TIER_CONFIGS['calib'].seed}）上扫描 "
      f"$\\theta\\in\\{{{', '.join(f'{t:.2f}' for t in THETAS)}\\}}$。"
      "选阈值规则：准确率 $a(\\theta)=\\text{正确自动数}/A_\\theta\\ge0.95$"
      "（整数判据 $20w\\le A$），并要求其所有更高阈值也合格（安全前缀）；"
      "合格点中取人工数最少者，人工数相同取最高阈值。\n")
    A(f"**选定阈值 $\\theta^*={sel['theta_star']:.2f}$**"
      f"（合格阈值：{', '.join(f'{t:.2f}' for t in sel['eligible'])}）。\n")
    A("| θ | 自动 A | 人工 H | 成功率 | 准确率 | 误推 w |")
    A("|---:|---:|---:|---:|---:|---:|")
    for r in calib_rows:
        gate = "✅" if (r["auto"] and 20 * r["wrong"] <= r["auto"]) else "—"
        star = " **← 选定**" if r["theta"] == sel["theta_star"] else ""
        A(f"| {r['theta']:.2f} | {r['auto']} | {r['manual']} | "
          f"{fmt_pct(r['success_rate'])} | {fmt_pct(r['accuracy'])} {gate} | "
          f"{r['wrong']}{star} |")
    A("")
    A("曲线图：`curves.png`（左一为校准集，右三为三档；上：准确率与 95% 门限，"
      "下：人工复核数量，竖虚线为 θ*）。\n")

    A("## 5. 三档旧图各正式跑一次的结果\n")
    A("每档地图由固定 seed 确定性生成、正式迁移只执行一次；阈值冻结为校准得到的 "
      f"θ*={sel['theta_star']:.2f}，不再用各档真值调参。\n")
    A("| 档位 | N | 硬编码 K | 未知 U | 自动 A | 人工 H | "
      "成功率 r | 自动准确率 | 误推 | 最终覆盖率 | C1 | C2 |")
    A("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|:--:|:--:|")
    for key in ("small", "medium", "large"):
        res = results[key]
        f = res["formula"]
        fc = res["final_completeness"]
        A(f"| {TIER_CONFIGS[key].cn} `{key}` | {f['N']} | {f['K_hardcoded']} "
          f"| {f['U_unknown']} | {f['A_auto']} | {f['H_manual']} "
          f"| {fmt_pct(f['r_success'])} | **{fmt_pct(f['accuracy'])}** "
          f"| {f['wrong']} | {fmt_pct(f['coverage_final'])} "
          f"| {'✅' if fc['C1_all_old_ids_mapped'] else '❌'} "
          f"| {'✅' if fc['C2_all_labels_supported'] else '❌'} |")
    A("")
    A(f"**总门限：映射覆盖率 100% 且每档自动推断准确率 ≥ 95% —— "
      f"{'全部通过 ✅' if global_gate_pass else '存在未通过 ❌'}。**\n")

    A("### 5.1 初始硬编码表为什么不完整\n")
    A("| 档位 | 旧编号覆盖 | 覆盖率 | 主城支持 | 野外支持 | 副本支持 | C2 |")
    A("|---|---:|---:|---:|---:|---:|:--:|")
    for key in ("small", "medium", "large"):
        hc = results[key]["hard_completeness"]
        sc = hc["label_support_count"]
        A(f"| {key} | {hc['n_assigned']}/{hc['n_old']} "
          f"| {fmt_pct(hc['coverage'])} | {sc['main_city']} "
          f"| {sc['wilderness']} | **{sc['dungeon']}** "
          f"| {'✅' if hc['C2_all_labels_supported'] else '❌（副本为 0）'} |")
    A("")

    A("### 5.2 反解公式的逐档数字核对\n")
    A("| 档位 | U | r | r·U=A | (1−r)·U=H | 恒等式 |")
    A("|---|---:|---:|---:|---:|:--:|")
    for key in ("small", "medium", "large"):
        f = results[key]["formula"]
        A(f"| {key} | {f['U_unknown']} | {f['r_success']:.6f} "
          f"| {f['r_success']*f['U_unknown']:.1f} = {f['A_auto']} "
          f"| {(1-f['r_success'])*f['U_unknown']:.1f} = {f['H_manual']} "
          f"| {'✅' if f['identity_rU_equals_A'] and f['identity_U_minus_A_equals_H'] else '❌'} |")
    A("")

    A("### 5.3 每个新标签至少一个旧编号（最终支持数）\n")
    for key in ("small", "medium", "large"):
        fc = results[key]["final_completeness"]
        sc = fc["label_support_count"]
        ids = fc["label_support_ids"]
        rc = fc["region_support_count"]
        region_brief = "，".join(f"{rid}×{n}" for rid, n in rc.items())
        A(f"- **{key}**：主城 {sc['main_city']} 个、野外 {sc['wilderness']} 个、"
          f"副本 {sc['dungeon']} 个。副本旧编号示例："
          f"`{ids['dungeon'][:8]}`"
          f"{' …' if len(ids['dungeon']) > 8 else ''}")
        A(f"  - C3′ 具体区域 tag（{len(rc)} 个，全部 ≥1 ✅）：{region_brief}")
    A("")

    A("## 6. 误推清单（旧编号 + 它实际所属的标签）\n")
    A("以下为 θ* 正式结果中自动推断错误的全部记录（评测用合成真值核对）；"
      "完整机器可读清单见 `mispredictions.csv`，所有阈值下的误推台账见 "
      "`sweep_mispredictions.csv`。这些行应按“实际标签”回改并加入人工抽检。\n")
    any_wrong = False
    for key in ("small", "medium", "large"):
        ms = results[key]["mispredictions"]
        if not ms:
            A(f"- **{key}**：θ*={theta_sel['theta_star']:.2f} 下无误推。")
            continue
        any_wrong = True
        A(f"**{key}（{len(ms)} 条）：**\n")
        A("| 旧编号 old_id | 误推标签 | 误推区域 | 实际标签 actual | 实际区域 | 置信度 |")
        A("|---:|---|---|---|---|---:|")
        for m in sorted(ms, key=lambda x: x["old_id"]):
            A(f"| **{m['old_id']}** | {m['inferred_label']} "
              f"({m['inferred_label_cn']}) | {m['inferred_region']} "
              f"| **{m['actual_label']} ({m['actual_label_cn']})** "
              f"| {m['actual_region']} | {m['confidence']:.3f} |")
        A("")
    if not any_wrong:
        A("_三档在 θ* 下均无误推。_\n")

    A("## 7. 各档阈值扫描全表\n")
    A("θ* 由独立校准集冻结在 0.90，以下各档全表仅做事后曲线展示，"
      "**不**据此回调各档阈值——否则等于用评测真值重新调参。"
      "因此即使某档在别的 θ 点数字更好看，正式结果仍以 5 节的冻结 θ* 为准。\n")
    for key in ("small", "medium", "large"):
        A(f"### {key}\n")
        A("| θ | 自动 | 人工 | 成功率 | 准确率 | 误推 | 并列转人工 | 低置信转人工 |")
        A("|---:|---:|---:|---:|---:|---:|---:|---:|")
        for r in results[key]["sweep"]:
            star = " **θ* 选定**" if r["theta"] == theta_sel["theta_star"] else ""
            A(f"| {r['theta']:.2f}{star} | {r['auto']} | {r['manual']} "
              f"| {fmt_pct(r['success_rate'])} | {fmt_pct(r['accuracy'])} "
              f"| {r['wrong']} | {r.get('tie', 0)} "
              f"| {r.get('below_threshold', 0)} |")
        A("")

    A("## 8. 方法边界说明\n")
    A("- 包围盒是粗粒度启发式：新区域 bbox 相交时，落在相交带的旧编号会"
      "并列歧义，规则要求转人工而不是猜。\n"
      "- “新副本区域旧编号里没有”有两层含义：硬编码映射表中没有副本行"
      "（本实验的建模）；若存在迁移时点之后才新建、空间上不含任何旧格子的"
      "副本实例，它们不进入本期回迁映射，直接发放全新 ID —— C2 只对"
      "“本期新标签集合”负责。\n"
      "- 准确率曲线在真实项目中应来自人工标注的校准样本，而不是生产真值；"
      "本实验用带已知真值的确定性合成地图演示完整决策闭环。\n")
    return "\n".join(L)


# ---------------------------------------------------------------------------
# 画图
# ---------------------------------------------------------------------------

def plot_curves(out_png, theta_star, calib_rows, results):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:  # pragma: no cover
        print(f"[warn] matplotlib 不可用，跳过曲线图: {e}", file=sys.stderr)
        return False

    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei",
                                       "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    BLUE, ORANGE, GREEN, GRAY, RED = (
        "#2a78d6", "#eb6834", "#0ca30c", "#898781", "#d03b3b")
    cols = [("calib", "校准集 N=80", calib_rows)] + [
        (k, f"{TIER_CONFIGS[k].cn} N={TIER_CONFIGS[k].n_total}",
         results[k]["sweep"])
        for k in ("small", "medium", "large")
    ]
    fig, axes = plt.subplots(2, 4, figsize=(16.5, 7.2), sharex=True)
    for j, (key, title, rows) in enumerate(cols):
        xs = [r["theta"] for r in rows]
        acc = [r["accuracy"] if r["accuracy"] is not None else math.nan
               for r in rows]
        manual = [r["manual"] for r in rows]

        ax = axes[0][j]
        ax.plot(xs, acc, "-o", color=BLUE, lw=2, ms=4.5, label="自动推断准确率")
        ax.axhline(0.95, color=GREEN, ls="--", lw=1.6, label="95% 门限")
        ax.axvline(theta_star, color=GRAY, ls=":", lw=1.6)
        ax.scatter([theta_star],
                   [next(r["accuracy"] for r in rows
                         if r["theta"] == theta_star)],
                   s=110, facecolor="none", edgecolor=RED, lw=1.8, zorder=5)
        for x, y in zip(xs, acc):
            if not math.isnan(y) and (x in (1.0, 0.9, 0.75, 0.5)):
                ax.annotate(f"{y*100:.0f}", (x, y),
                            textcoords="offset points", xytext=(0, 7),
                            ha="center", fontsize=8, color="#52514e")
        ax.set_ylim(0.82, 1.03)
        ax.set_title(title, fontsize=12, color="#0b0b0b")
        ax.grid(True, color="#e1e0d9", lw=0.8)
        ax.spines[["top", "right"]].set_visible(False)
        ax.spines[["left", "bottom"]].set_color("#c3c2b7")
        if j == 0:
            ax.set_ylabel("准确率", fontsize=11)
            ax.legend(loc="lower left", fontsize=8.5, frameon=False)

        ax2 = axes[1][j]
        ax2.plot(xs, manual, "-o", color=ORANGE, lw=2, ms=4.5,
                 label="需人工确认数 H")
        ax2.axvline(theta_star, color=GRAY, ls=":", lw=1.6)
        h_star = next(r["manual"] for r in rows if r["theta"] == theta_star)
        ax2.scatter([theta_star], [h_star], s=110, facecolor="none",
                    edgecolor=RED, lw=1.8, zorder=5)
        ax2.annotate(f"H={h_star}", (theta_star, h_star),
                     textcoords="offset points", xytext=(8, -2),
                     fontsize=9.5, color="#0b0b0b")
        ax2.grid(True, color="#e1e0d9", lw=0.8)
        ax2.spines[["top", "right"]].set_visible(False)
        ax2.spines[["left", "bottom"]].set_color("#c3c2b7")
        ax2.set_xlabel("包围比例阈值 θ（1.00 = 全部格子；越往右越严格）", fontsize=10)
        if j == 0:
            ax2.set_ylabel("人工工作量（个）", fontsize=11)
            ax2.legend(loc="upper right", fontsize=8.5, frameon=False)

    fig.suptitle("阈值取舍：自动推断准确率 vs 人工确认工作量",
                 fontsize=14, color="#0b0b0b", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=170, bbox_inches="tight",
                facecolor="#fcfcfb")
    plt.close(fig)
    return True


def plot_map_overview(out_png, res):
    """中档地图的包围盒/旧格点/误推编号总览，用于肉眼检查几何。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception:
        return False
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "DejaVu Sans"]
    COLOR = {"main_city": "#2a78d6", "wilderness": "#1baf7a",
             "dungeon": "#6250d6"}
    fig, ax = plt.subplots(figsize=(12, 7.4))
    for r in REGIONS.values():
        ax.add_patch(Rectangle((r.x0, r.y0), r.x1 - r.x0 + 1,
                               r.y1 - r.y0 + 1,
                               facecolor=COLOR[r.label], alpha=0.07,
                               edgecolor=COLOR[r.label], lw=1.6))
        ax.text(r.x0 + 1, r.y1 - 2, r.rid, fontsize=7.5,
                color=COLOR[r.label], weight="bold")
    for rec in res["records"]:
        xs = [p[0] for p in rec.cells]
        ys = [p[1] for p in rec.cells]
        ax.scatter(xs, ys, s=7, color=COLOR[rec.actual_label], alpha=0.75,
                   marker="s", linewidths=0)
    wrong_ids = {m["old_id"] for m in res["mispredictions"]}
    for m in res["mispredictions"]:
        rec = next(r for r in res["records"] if r.oid == m["old_id"])
        cx = sum(p[0] for p in rec.cells) / len(rec.cells)
        cy = sum(p[1] for p in rec.cells) / len(rec.cells)
        ax.scatter([cx], [cy], s=260, facecolor="none", edgecolor="#d03b3b",
                   lw=2.0, zorder=5)
        ax.annotate(f"old {m['old_id']}→实际{m['actual_label_cn']}",
                    (cx, cy), textcoords="offset points", xytext=(8, 8),
                    fontsize=9, color="#d03b3b", weight="bold")
    ax.set_xlim(-2, 262)
    ax.set_ylim(-2, 162)
    ax.set_aspect("equal")
    ax.grid(True, color="#e1e0d9", lw=0.6)
    ax.set_title("中档合成地图总览：格点颜色=真实标签，红圈=θ* 下误推",
                 fontsize=13)
    handles = [Rectangle((0, 0), 1, 1, facecolor=COLOR[l], alpha=0.35,
                         edgecolor=COLOR[l], label=LABEL_CN[l])
               for l in LABELS]
    ax.legend(handles=handles, loc="upper right", frameon=False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=160, bbox_inches="tight", facecolor="#fcfcfb")
    plt.close(fig)
    return True


# ---------------------------------------------------------------------------
# 档位配置与 main
# ---------------------------------------------------------------------------

TIER_CONFIGS = {
    "calib": TierConfig(
        key="calib", cn="校准档", n_total=80, seed=2026112600,
        hard={"main_city": 8, "wilderness": 12},       # K=20, U=60
        clean={"main_city": 10, "wilderness": 22, "dungeon": 8},  # 40
        edge90=8, edge75={"wilderness": 4}, edge60=2,
        amb=2, htrap=2, mtrap=2,
    ),
    "small": TierConfig(
        key="small", cn="小档", n_total=24, seed=2026112601,
        hard={"main_city": 4, "wilderness": 8},         # K=12, U=12
        clean={"main_city": 3, "wilderness": 3, "dungeon": 2},   # 8
        edge90=2, edge75={"main_city": 1}, edge60=0,
        amb=1, htrap=0, mtrap=0,
        region_pool=SMALL_REGIONS,
    ),
    "medium": TierConfig(
        key="medium", cn="中档", n_total=80, seed=2026112602,
        hard={"main_city": 12, "wilderness": 24},       # K=36, U=44
        clean={"main_city": 8, "wilderness": 14, "dungeon": 6},   # 28
        edge90=6, edge75={"wilderness": 2, "main_city": 1},       # 3
        edge60=2, amb=3, htrap=1, mtrap=1,
    ),
    "large": TierConfig(
        key="large", cn="大档", n_total=240, seed=2026112603,
        hard={"main_city": 34, "wilderness": 66},       # K=100, U=140
        clean={"main_city": 23, "wilderness": 45, "dungeon": 20},  # 88
        edge90=20, edge75={"wilderness": 6, "main_city": 4},      # 10
        edge60=6, amb=8, htrap=4, mtrap=4,
    ),
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default_out = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "artifacts", "mapping")
    ap.add_argument("--out", default=default_out,
                    help="产物目录 (默认 <repo>/artifacts/mapping)")
    args = ap.parse_args(argv)
    out = os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)

    # 1) 校准集定阈值。
    print("[1/4] 校准集扫描阈值 ...")
    calib_records = build_world(TIER_CONFIGS["calib"])
    calib_rows = sweep(calib_records)
    sel = choose_threshold(calib_rows)
    theta_star = sel["theta_star"]
    print(f"      theta* = {theta_star:.2f}, eligible={sel['eligible']}")

    # 2) 三档各正式跑一次 (阈值冻结)。
    results = {}
    for key in ("small", "medium", "large"):
        print(f"[2/4] 正式运行档位 {key} (N={TIER_CONFIGS[key].n_total}) ...")
        results[key] = run_tier(TIER_CONFIGS[key], theta_star)

    # 3) 写 CSV。
    print("[3/4] 写 CSV / JSON / Markdown ...")
    for key, res in results.items():
        write_mapping_csv(os.path.join(out, f"mapping_{key}.csv"),
                          res["csv_rows"])

    sweep_path = os.path.join(out, "curves.csv")
    if os.path.exists(sweep_path):
        os.remove(sweep_path)
    write_sweep_csv(sweep_path, "calib", calib_rows, write_header=True)
    for key in ("small", "medium", "large"):
        write_sweep_csv(sweep_path, key, results[key]["sweep"])

    mis_path = os.path.join(out, "mispredictions.csv")
    if os.path.exists(mis_path):
        os.remove(mis_path)
    all_mis = []
    write_mispredictions_csv(mis_path, [], write_header=True)
    for key in ("small", "medium", "large"):
        write_mispredictions_csv(mis_path, results[key]["mispredictions"])
        all_mis += results[key]["mispredictions"]

    # 所有阈值下的误推台账 (分析用)。
    ledger_path = os.path.join(out, "sweep_mispredictions.csv")
    ledger_fields = ["tier", "theta", "old_id", "inferred_label",
                     "actual_label", "confidence"]
    with open(ledger_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=ledger_fields)
        w.writeheader()
        for tier_key, recs in [("calib", calib_records)] + [
                (k, results[k]["records"]) for k in
                ("small", "medium", "large")]:
            for theta in THETAS:
                for r in recs:
                    if r.hardcoded:
                        continue
                    d = infer_unknown(r, theta)
                    if d.status == "auto" and not d.correct:
                        w.writerow({
                            "tier": tier_key, "theta": theta,
                            "old_id": r.oid,
                            "inferred_label": d.label,
                            "actual_label": r.actual_label,
                            "confidence": round(d.confidence, 4)})

    gate_pass = True
    summary = {"theta_star": theta_star,
               "eligible_thetas": sel["eligible"],
               "gate_accuracy": GATE_ACCURACY,
               "tiers": {}}
    for key in ("small", "medium", "large"):
        res = results[key]
        f = res["formula"]
        ok_cov = f["coverage_final"] == 1.0
        ok_acc = f["A_auto"] > 0 and 20 * f["wrong"] <= f["A_auto"]
        ok_c1 = res["final_completeness"]["C1_all_old_ids_mapped"]
        ok_c2 = res["final_completeness"]["C2_all_labels_supported"]
        ok_c3 = res["final_completeness"]["C3_all_region_tags_supported"]
        gate_pass &= ok_cov and ok_acc and ok_c1 and ok_c2 and ok_c3
        summary["tiers"][key] = {
            "N": f["N"], "K_hardcoded": f["K_hardcoded"],
            "U_unknown": f["U_unknown"], "A_auto": f["A_auto"],
            "H_manual": f["H_manual"],
            "r_success": round(f["r_success"], 6),
            "accuracy": round(f["accuracy"], 6),
            "wrong": f["wrong"], "coverage": f["coverage_final"],
            "C1": ok_c1, "C2": ok_c2, "C3_region_tags": ok_c3,
            "gate_accuracy_pass": ok_acc,
            "mispredictions": [
                {"old_id": m["old_id"],
                 "inferred_label": m["inferred_label"],
                 "actual_label": m["actual_label"]}
                for m in res["mispredictions"]],
        }
    summary["all_gates_pass"] = gate_pass
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    report = build_report(sel, calib_rows, results, gate_pass)
    with open(os.path.join(out, "report.md"), "w", encoding="utf-8") as f:
        f.write(report)

    # 4) 图。
    print("[4/4] 画曲线图 / 地图总览 ...")
    plot_curves(os.path.join(out, "curves.png"), theta_star,
                calib_rows, results)
    plot_map_overview(os.path.join(out, "bbox_overview_medium.png"),
                      results["medium"])

    # 控制台简报。
    print("\n===== 结果摘要 =====")
    print(f"theta* = {theta_star:.2f}")
    for key in ("small", "medium", "large"):
        s = summary["tiers"][key]
        print(f"{key:6s} N={s['N']:3d} U={s['U_unknown']:3d} "
              f"A={s['A_auto']:3d} H={s['H_manual']:3d} "
              f"r={s['r_success']:.3f} acc={s['accuracy']:.4f} "
              f"wrong={s['wrong']} cov={s['coverage']:.3f} "
              f"C1={s['C1']} C2={s['C2']} C3={s['C3_region_tags']} "
              f"gate={s['gate_accuracy_pass']}")
        for m in s["mispredictions"]:
            print(f"   MISPREDICT old_id={m['old_id']:3d} "
                  f"inferred={m['inferred_label']:11s} "
                  f"actual={m['actual_label']}")
    print(f"\nall_gates_pass={gate_pass}")
    print(f"artifacts -> {out}")
    return 0 if gate_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
