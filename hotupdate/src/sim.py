#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hotupdate: 地图热更新 + 服务端路径校验事故仿真。

模型与 grid/src/grid.c 对齐：16x12 网格、四邻域、blocked 不可走、A* 寻路。
只依赖标准库，可用 `python3 -I sim.py` 运行。

时间单位 tick（1 tick = 1 秒）。事故时钟映射：
  10:02 首个穿墙报告  -> T0        = 270s
  10:05 服务端换图    -> SWITCH_T  = 300s（cutover：v1 地图退役，只认 v2）
  10:09 全服卡顿      -> T_LAG     = 324s
  10:15 回滚          -> T_ROLLBACK= 360s

关键建模约定
------------
1. 校验判据（README 公式 C-1/C-2）：逐点检查可走性，
   valid(k) = ( #{p_i : p_i in blocked(M)} <= k )，k=0 即原始严格判据。
2. 每个包带两个版本：
   - gen_version   路径实际生成所用地图（客户端真实内容）；
   - label_version 包上填写的版本号字段。
   fixed/latebump 两者恒等；buggy 事故中内容换了但版本号字段不更新，
   label_version 恒为 1。
3. 服务端 cutover：t >= SWITCH_T 后 v1 地图退役，label_version=1 的在途
   包只能拿到 v2 地图校验 —— 版本号提前 bump 的意义就是保证 cutover 时刻
   没有 label=1 的包。
4. 误判因果分类（诚实包被拒时）：
   - switch-induced：在自己生成地图上能过（bad_own <= k），在服务端
     实际校验地图上被拒（bad_eval > k）—— 纯粹由地图/版本错位造成；
   - edge-noise：在自己地图上本来就过不了（贴边量化噪声），与切换无关。
   题目要求为零的是 switch-induced 这一类。

三种策略
--------
  buggy    从不 bump 版本号；服务端 SWITCH_T 直接 cutover 到 v2。
  fixed    SWITCH_T - D_max 时刻 bump，最慢客户端恰在 cutover 前激活。
  latebump 只提前 D_max/2 bump，cutover 时仍有 v1 在途包，用于展示误判诊断。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

W, H = 16, 12
N_CELLS = W * H

SWITCH_T = 300
T0 = 270
T_LAG = 324
T_ROLLBACK = 360
SIM_START, SIM_END = 120, 660
N_CLIENTS = 4000
CHEAT_RATE = 0.05
SAMPLE_PER_TICK = 0.08
RNG_SEED = 20261126


# --------------------------------------------------------------------------
# 确定性随机数（xorshift64*）
# --------------------------------------------------------------------------
class Rng:
    def __init__(self, seed: int):
        self.s = seed & ((1 << 64) - 1) or 0x9E3779B97F4A7C15

    def next_u64(self) -> int:
        x = self.s
        x ^= (x << 13) & ((1 << 64) - 1)
        x ^= x >> 7
        x ^= (x << 17) & ((1 << 64) - 1)
        self.s = x & ((1 << 64) - 1)
        return (self.s * 0x2545F4914F6CDD1D) & ((1 << 64) - 1)

    def uniform(self) -> float:
        return self.next_u64() / float(1 << 64)

    def randint(self, a: int, b: int) -> int:
        return a + self.next_u64() % (b - a + 1)

    def choice(self, seq):
        return seq[self.next_u64() % len(seq)]


def idx(x: int, y: int) -> int:
    return y * W + x


def xy(i: int):
    return i % W, i // W


# --------------------------------------------------------------------------
# 地图（与 grid.c 的 Map/blocked/walkable 同构）
# --------------------------------------------------------------------------
class Map:
    def __init__(self, version: int, blocked: frozenset[int] | None = None, name: str = ""):
        self.version = version
        self.name = name
        self.blocked = frozenset(blocked or ())

    def walkable(self, x: int, y: int) -> bool:
        if x < 0 or y < 0 or x >= W or y >= H:
            return False
        return idx(x, y) not in self.blocked


def build_maps() -> tuple[Map, Map]:
    """v1 带基线墙体；v2 = v1 + 热更新增量（堵门 + 竖墙 + 新柱子）。"""
    base: set[int] = set()
    # 横墙 y=4, x=1..13，留一个门洞 (7,4)
    for x in range(1, 14):
        if x != 7:
            base.add(idx(x, 4))
    # 基线柱子
    base.add(idx(12, 9))
    base.add(idx(3, 10))
    v1 = Map(1, frozenset(base), "v1-old")

    delta: set[int] = set(base)
    # 热更新：堵死门洞并向下延伸竖墙 x=7, y=4..9
    for y in range(4, 10):
        delta.add(idx(7, y))
    # 两根新柱子
    delta.add(idx(11, 6))
    delta.add(idx(4, 8))
    v2 = Map(2, frozenset(delta), "v2-new")

    # 保证出生/目标点在两张图上都可走
    safe = [idx(1, 1), idx(14, 1), idx(2, 2), idx(13, 2), idx(1, 11),
            idx(14, 10), idx(2, 10), idx(10, 10), idx(13, 11), idx(5, 1)]
    for p in safe:
        assert p not in v1.blocked and p not in v2.blocked, f"unsafe cell {xy(p)}"
    return v1, v2


# --------------------------------------------------------------------------
# 四邻域 A*，与 grid.c 语义一致；不可达返回 None
# --------------------------------------------------------------------------
def astar(m: Map, s: int, g: int) -> list[int] | None:
    sx, sy = xy(s)
    gx, gy = xy(g)
    if not m.walkable(sx, sy) or not m.walkable(gx, gy):
        return None
    INF = 1 << 29
    gscore = [INF] * N_CELLS
    fscore = [INF] * N_CELLS
    closed = [False] * N_CELLS
    parent = [-1] * N_CELLS
    gscore[s] = 0
    fscore[s] = abs(gx - sx) + abs(gy - sy)
    open_set = {s}
    nei = ((1, 0), (-1, 0), (0, 1), (0, -1))
    while open_set:
        cur = min(open_set, key=lambda c: fscore[c])
        open_set.discard(cur)
        if cur == g:
            rev, c = [], cur
            while c != -1:
                rev.append(c)
                c = parent[c]
            return rev[::-1]
        closed[cur] = True
        cx, cy = xy(cur)
        for dx, dy in nei:
            nx, ny = cx + dx, cy + dy
            if not m.walkable(nx, ny):
                continue
            ni = idx(nx, ny)
            if closed[ni]:
                continue
            ng = gscore[cur] + 1
            if ng < gscore[ni]:
                gscore[ni] = ng
                fscore[ni] = ng + abs(gx - nx) + abs(gy - ny)
                parent[ni] = cur
                open_set.add(ni)
    return None


def supercover_line(s: int, g: int) -> list[int]:
    """Bresenham supercover，与 smooth.c 的 LOS 取点方式一致。"""
    x0, y0 = xy(s)
    x1, y1 = xy(g)
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    out = []
    while True:
        if 0 <= x < W and 0 <= y < H:
            out.append(idx(x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return out


# --------------------------------------------------------------------------
# 校验判据 C-1/C-2
# --------------------------------------------------------------------------
def count_blocked(m: Map, path: list[int]) -> int:
    return sum(1 for p in path if p in m.blocked)


def check_path(m: Map, path: list[int], k: int) -> bool:
    return count_blocked(m, path) <= k


@dataclass
class Packet:
    t: int
    client: int
    gen_version: int          # 实际生成地图
    label_version: int        # 包上填写的版本号
    path: list[int]
    is_cheat: bool
    bad_honest: int           # 诚实包自带的边缘噪声坏点数


@dataclass
class RejectRecord:
    t: int
    client: int
    gen_version: int
    label_version: int
    eval_map_version: int     # 服务端实际拿来校验的地图版本
    bad_eval: int
    bad_own: int
    is_cheat: bool
    switch_induced: bool
    reason: str


@dataclass
class Client:
    cid: int
    is_cheat: bool
    delay: int
    active_version: int = 1


# 固定的出生/目标点（两张图上均可走）
SAFE_POINTS = [idx(1, 1), idx(14, 1), idx(2, 2), idx(13, 2), idx(1, 11),
               idx(14, 10), idx(2, 10), idx(10, 10), idx(13, 11), idx(5, 1)]
# 作弊端点：刻意让直线穿过基线横墙/竖墙，坏点数覆盖 1..5 的梯度
CHEAT_PAIRS = [
    (idx(1, 1), idx(14, 10)),    # 长距，穿横墙+竖墙
    (idx(2, 2), idx(13, 11)),
    (idx(14, 1), idx(1, 11)),
    (idx(5, 1), idx(10, 10)),
    (idx(13, 2), idx(2, 10)),
    (idx(1, 1), idx(12, 9)),     # 中距
    (idx(14, 1), idx(3, 10)),
    (idx(2, 2), idx(14, 10)),
]

# 路径缓存：同一 (地图版本, 起, 终) 的 A*/直线只算一次，
# 仿真只从固定端点集合采样，缓存命中后速度提升两个数量级。
_PATH_CACHE: dict[tuple, list[int]] = {}
_LINE_CACHE: dict[tuple, list[int]] = {}


def honest_path(m: Map, s: int, g: int) -> list[int]:
    key = (m.version, s, g)
    p = _PATH_CACHE.get(key)
    if p is None:
        p = astar(m, s, g) or [s, g]
        _PATH_CACHE[key] = p
    return list(p)


def cheat_line(m: Map, s: int, g: int) -> list[int]:
    key = (m.version, s, g)
    p = _LINE_CACHE.get(key)
    if p is None:
        p = supercover_line(s, g)
        if not any(c in m.blocked for c in p) and m.blocked:
            mid = len(p) // 2
            p = p[:mid] + [next(iter(m.blocked))] + p[mid:]
        _LINE_CACHE[key] = p
    return list(p)


def make_honest_packet(rng: Rng, maps: dict[int, Map], c: Client, t: int) -> Packet:
    m = maps[c.active_version]
    s = rng.choice(SAFE_POINTS)
    g = rng.choice([p for p in SAFE_POINTS if p != s])
    path = honest_path(m, s, g)
    # 边缘噪声：3.0% 诚实包含 1 个坏点，0.4% 含 2 个（贴边移动量化误差）
    r = rng.uniform()
    bad_honest = 2 if r < 0.004 else (1 if r < 0.034 else 0)
    if bad_honest and m.blocked:
        cells = list(m.blocked)
        path = path + [rng.choice(cells) for _ in range(bad_honest)]
    return Packet(t, c.cid, c.active_version, c.active_version, path, False, bad_honest)


def make_cheat_packet(rng: Rng, maps: dict[int, Map], c: Client, t: int) -> Packet:
    m = maps[c.active_version]
    s, g = rng.choice(CHEAT_PAIRS)
    line = cheat_line(m, s, g)
    return Packet(t, c.cid, c.active_version, c.active_version, line, True, 0)


# --------------------------------------------------------------------------
# 服务端
# --------------------------------------------------------------------------
@dataclass
class Server:
    policy: str
    maps: dict[int, Map]
    d_max: int
    k: int = 0
    bump_t: int | None = None
    switch_t: int = SWITCH_T
    rejects: list[RejectRecord] = field(default_factory=list)

    def __post_init__(self):
        if self.policy == "buggy":
            self.bump_t = None
        elif self.policy == "fixed":
            self.bump_t = self.switch_t - self.d_max
        else:  # latebump：提前量不足
            self.bump_t = self.switch_t - max(1, self.d_max // 2)

    def current_version(self, t: int) -> int:
        return 2 if t >= self.switch_t else 1

    def eval_map_version(self, pkt: Packet, t: int) -> tuple[int, str]:
        """服务端依据包版本号选地图；cutover 后 v1 退役。"""
        if t < self.switch_t:
            return pkt.label_version, "overlap"
        # cutover 之后只接受 v2
        if pkt.label_version == 2:
            return 2, "current"
        return 2, "cutover-skew"

    def validate(self, pkt: Packet, t: int) -> tuple[bool, RejectRecord | None]:
        eval_ver, reason = self.eval_map_version(pkt, t)
        m_eval = self.maps[eval_ver]
        m_own = self.maps[pkt.gen_version]
        bad_eval = count_blocked(m_eval, pkt.path)
        bad_own = count_blocked(m_own, pkt.path)
        ok = bad_eval <= self.k
        rec = None
        if not ok:
            if pkt.is_cheat:
                switch_induced = False
                tag = "cheat"
            else:
                switch_induced = bad_own <= self.k
                tag = "switch-induced" if switch_induced else "edge-noise"
                if reason == "cutover-skew":
                    tag = "switch-induced" if switch_induced else "edge-noise"
            rec = RejectRecord(t, pkt.client, pkt.gen_version, pkt.label_version,
                               eval_ver, bad_eval, bad_own, pkt.is_cheat,
                               switch_induced, tag)
            self.rejects.append(rec)
        return ok, rec


# --------------------------------------------------------------------------
# 单次仿真
# --------------------------------------------------------------------------
def simulate(d_max: int, policy: str, k: int = 0, seed: int = RNG_SEED):
    # 注意：不能用 hash(policy)，str 哈希受 PYTHONHASHSEED 随机化影响
    policy_seed = {"buggy": 11, "fixed": 22, "latebump": 33}[policy]
    rng = Rng(seed ^ (d_max * 7919) ^ policy_seed)
    v1, v2 = build_maps()
    maps = {1: v1, 2: v2}

    d_min = max(1, math.ceil(0.2 * d_max))
    clients: list[Client] = []
    for cid in range(N_CLIENTS):
        is_cheat = rng.uniform() < CHEAT_RATE
        delay = d_min + rng.randint(0, d_max - d_min)
        clients.append(Client(cid, is_cheat, delay))
    server = Server(policy, maps, d_max, k=k)
    bump = server.bump_t

    rows = []
    for t in range(SIM_START, SIM_END + 1):
        # 1) 客户端地图内容传播
        for c in clients:
            if c.active_version != 1:
                continue
            if policy == "buggy":
                # 事故里内容仍按延迟到达，但没有版本号 bump 配合
                if t >= SWITCH_T + c.delay:
                    c.active_version = 2
            elif t >= bump + c.delay:
                c.active_version = 2

        sample_n = max(1, int(round(N_CLIENTS * SAMPLE_PER_TICK)))
        total = honest_n = cheat_n = 0
        rej_honest = rej_cheat = 0
        switch_induced_n = edge_noise_n = skew_n = 0
        for _ in range(sample_n):
            c = clients[rng.randint(0, N_CLIENTS - 1)]
            pkt = (make_cheat_packet(rng, maps, c, t) if c.is_cheat
                   else make_honest_packet(rng, maps, c, t))
            if policy == "buggy":
                pkt.label_version = 1      # 版本号字段从未更新
            total += 1
            if pkt.is_cheat:
                cheat_n += 1
            else:
                honest_n += 1
            ok, rec = server.validate(pkt, t)
            if not ok:
                if pkt.is_cheat:
                    rej_cheat += 1
                else:
                    rej_honest += 1
                    if rec.switch_induced:
                        switch_induced_n += 1
                    else:
                        edge_noise_n += 1
                if rec.reason == "cutover-skew":
                    skew_n += 1
        rows.append(dict(
            t=t, total=total, honest=honest_n, cheat=cheat_n,
            rej_honest=rej_honest, rej_cheat=rej_cheat,
            switch_induced=switch_induced_n, edge_noise=edge_noise_n,
            skew=skew_n,
            reject_rate=(rej_honest + rej_cheat) / total,
            fpr=rej_honest / honest_n if honest_n else 0.0,
            cir=rej_cheat / cheat_n if cheat_n else 0.0,
        ))
    return rows, server, clients, maps


def window(rows, lo, hi):
    return [r for r in rows if lo <= r["t"] <= hi]


def summarize(rows, server):
    pre = window(rows, SIM_START, SWITCH_T - 1)
    sw = window(rows, SWITCH_T, T_ROLLBACK)
    base_fpr = sum(r["rej_honest"] for r in pre) / max(1, sum(r["honest"] for r in pre))
    base_cir = sum(r["rej_cheat"] for r in pre) / max(1, sum(r["cheat"] for r in pre))
    base_rr = (sum(r["rej_honest"] + r["rej_cheat"] for r in pre)
               / max(1, sum(r["total"] for r in pre)))
    # 拒绝率突增：cutover 后连续 2 个 tick 超过基线 5 个百分点（抗采样抖动）
    post = [r for r in rows if r["t"] >= SWITCH_T]
    spike_t = None
    for a, b in zip(post, post[1:]):
        if (a["reject_rate"] >= base_rr + 0.05
                and b["reject_rate"] >= base_rr + 0.05):
            spike_t = a["t"]
            break
    # 切换致因拒绝包络：观测窗口（到回滚）内最后一个此类拒绝
    si_obs = [r for r in sw if r["switch_induced"] > 0]
    last_obs = max((r["t"] for r in si_obs), default=None)
    # 全窗口包络（若不回滚、仿真继续）：用于还原未截断的真实 D_max
    si_full = [r for r in post if r["switch_induced"] > 0]
    last_full = max((r["t"] for r in si_full), default=None)
    peak = max(sw, key=lambda r: r["reject_rate"]) if sw else None
    inferred_obs = (last_obs - SWITCH_T) if last_obs is not None else 0
    inferred_full = (last_full - SWITCH_T) if last_full is not None else 0
    censored = last_full is not None and last_obs is not None and last_full > last_obs
    return dict(
        base_fpr=base_fpr, base_cir=base_cir, base_rr=base_rr,
        spike_t=spike_t,
        peak_t=peak["t"] if peak else None,
        peak_rate=peak["reject_rate"] if peak else 0.0,
        last_switch_induced_t_observed=last_obs,
        last_switch_induced_t_full=last_full,
        inferred_d_max_observed_s=inferred_obs,
        inferred_d_max_full_s=inferred_full,
        inference_censored_by_rollback=censored,
        switch_false_rejects=sum(r["switch_induced"] for r in sw),
        edge_noise_rejects=sum(r["edge_noise"] for r in sw),
        skew_packets=sum(r["skew"] for r in sw),
    )
