# hotupdate

地图热更新 × 服务端路径校验事故的建模与仿真。模型与 [`../grid`](../grid) 同构：
16×12 网格、四邻域、`blocked` 不可走、A* 寻路；本目录为 Python 确定性仿真。

- `make sim`：跑三档延迟 × 三策略 + 严格程度扫描，结果写 `results/sim_results.json`
- `make plots`：在 `figures/` 出 4 张图，并写 `results/misjudgement_diagnostics.csv`
- `make test`：27 项不变量断言（判据、根因、反解精度、零误伤、曲线单调性）

## 事故时间线与时钟映射

| 线上事件 | 时刻 | 仿真秒 |
|---|---|---|
| 玩家报告穿墙 | 10:02 | 270 |
| 服务端校验开始拒绝路径（换图/cutover） | 10:05 | 300 = `SWITCH_T` |
| 全服卡顿 | 10:09 | 324 |
| 回滚 | 10:15 | 360 |

仿真 4000 个客户端，每 tick 抽 8% 上报路径，作弊客户端占 5%，确定性随机种子
（xorshift64*，`RNG_SEED=20261126`），结果可复现。

## 1. 校验判据表达式

设路径为点序列 `P = (p_1, …, p_n)`，服务端当前校验地图为 `M`，其阻挡集为
`blocked(M)`，可走性谓词 `walkable(M, p) = (p ∉ blocked(M))`。

**逐点检查可走性，任一点不可走即拒绝**（原始严格判据，k=0）：

```
C-1  valid(P, M) = ∧_{i=1..n} walkable(M, p_i)
                 = ( ∀i: p_i ∉ blocked(M) )
      reject(P)  = ¬valid = ∨_{i=1..n} ( p_i ∈ blocked(M) )
```

引入严格程度参数 `k`（允许的不可走点个数），第 4 节的两条曲线都基于它：

```
C-2  valid(P, M; k) = ( #{ i : p_i ∈ blocked(M) } ≤ k )
```

代码对应 `src/sim.py` 的 `count_blocked()` / `check_path()`。

## 2. 根因：版本号与地图内容错位

热更新时**客户端地图内容换了，版本号字段没换**；服务端却在 10:05 直接把校验
地图切到新图并退役旧图。于是在旧图上合法的路径被新图判据逐条否决：

```
旧路径 P 生成于 M_v1：valid(P, M_v1; 0) = true
服务端用 M_v2 校验： valid(P, M_v2; 0) = false   ← 误判（穿墙举报的镜像问题）
```

仿真用两个版本字段精确刻画该错位：

- `gen_version`：路径实际生成所用地图（客户端真实内容）；
- `label_version`：包上填写的版本号。事故策略（`buggy`）里内容已切到 v2 但
  `label_version` 恒为 1；服务端 cutover 后只认 v2，`label=1` 的在途包全部
  拿到 v2 地图校验。

**误判的因果分类**（诚实包被拒时）：

- `switch-induced`：在自己生成的地图上能过（`bad_own ≤ k`），在服务端实际
  校验地图上被拒（`bad_eval > k`）——纯由地图/版本错位造成，题目要求为零的
  就是这一类；
- `edge-noise`：在自己地图上本来就过不了（贴边量化噪声），与切换无关。

## 3. 用拒绝率突增反解版本号该在什么时候更新

### 3.1 时序不等式

设客户端最大同步延迟为 `D_max`。cutover 发生在 `T_switch = 300`。要保证 cutover
瞬间没有任何 `label=1` 的在途包，版本号 bump 时刻 `T_bump` 必须满足最慢的客户端
也已完成激活：

```
T_bump + D_max ≤ T_switch
⇒ T_bump ≤ T_switch − D_max          （必须早于地图切换完成，提前量 = D_max）
```

### 3.2 从拒绝率反解 D_max

事故里没有 bump，cutover 后切换致因拒绝随客户端陆续完成同步而**单调衰减到零**，
其包络宽度就是同步延迟的上界。记最后一个 switch-induced 拒绝出现的时刻为
`T_last`：

```
D_max(估) = T_last − T_switch
T_bump(反解) = T_switch − D_max(估)
```

拒绝率突增的检测用抗噪判据：cutover 后**连续 2 个 tick** 拒绝率超过基线
5 个百分点（避免单 tick 采样抖动误判）。

### 3.3 回滚截断

观测窗口在 10:15（`T=360`）随回滚结束。当真实 `D_max` 超过窗口宽度
（60s）时，`T_last` 被截断，观测值只能给出下界；仿真同时计算"不回滚、继续
观测"的全窗口包络来还原真实值，并置 `censored_by_rollback=true`。

## 4. 严格程度的取舍：两条曲线定 k

校验严则作弊少但误伤多，松则反之。对 `k = 0..5` 在 cutover 前的稳态窗口统计：

- **诚实路径误判率 FPR(k)** = 被拒诚实包 / 全部诚实包（越低越好）；
- **作弊拦截率 CIR(k)** = 被拒作弊包 / 全部作弊包（越高越好）。

| k | FPR | CIR | 切换致因误伤 |
|---|---|---|---|
| 0 | 3.49% | 100.0% | 0 |
| **1（选定）** | **0.44%** | **67.4%** | **0** |
| 2 | 0.00% | 19.3% | 0 |
| 3 | 0.00% | 1.7% | 0 |
| 4 | 0.00% | 0.0% | 0 |
| 5 | 0.00% | 0.0% | 0 |

工作点取 **FPR 预算 1% 约束下 CIR 最大者 → k=1**。注意严格程度解决的是**平时
贴边噪声**的误伤；地图切换致因的误伤由第 2、3 节的版本化机制保证为零，与 k
无关（表中最后一列在所有 k 下恒为 0）。

## 5. 三档同步延迟实验结果

三种策略：`buggy`（从不 bump，复现事故）、`fixed`（提前 `D_max` bump +
双地图版本化校验）、`latebump`（只提前 `D_max/2`，制造可诊断的误判）。

| D_max | 策略 | bump 时刻 | 拒绝率尖峰 | 峰值拒绝率 | 切换致因误伤 | 反解 D_max（观测/全窗口） |
|---|---|---|---|---|---|---|
| 15s | buggy | 从不 | 10:05:00 | 0.391 | 849 | 14 / 14 |
| 15s | **fixed** | 10:04:45 | 无 | 0.138 | **0** | — |
| 15s | latebump | 10:04:53 | 10:05:00 | 0.253 | 231 | 7 / 7 |
| 60s | buggy | 从不 | 10:05:00 | 0.419 | 3366 | 58 / 58 |
| 60s | **fixed** | 10:04:00 | 无 | 0.119 | **0** | — |
| 60s | latebump | 10:04:30 | 10:05:00 | 0.278 | 864 | 29 / 29 |
| 180s | buggy | 从不 | 10:05:00 | 0.441 | 5558 | 60（截断）/ **179** |
| 180s | **fixed** | 10:02:00 | 无 | 0.119 | **0** | — |
| 180s | latebump | 10:03:30 | 10:05:00 | 0.325 | 2455 | 60（截断）/ 88 |

结论：

1. **`fixed` 三档的切换致因误伤全部为 0**，且没有拒绝率尖峰；D=180s 时 bump
   时刻恰好是 10:02，与首个玩家报告同时刻——提前量必须按最坏情况（最大延迟）
   而非平均延迟规划。
2. 反解在未截断时误差 ≤ 2 个采样 tick（15→14、60→58）；D=180 被回滚截断，
   全窗口还原 179 ≈ 180。
3. `latebump` 的反解值（7/29/88）对应其**不足的提前量**而非真实 D_max，
   说明该反解量的是"cutover 时仍在途的旧版本窗口"，这正是 bump 提前量要
   填平的量。

### 出现误判时的诊断（题目要求字段）

`results/misjudgement_diagnostics.csv` 对每条切换致因误判给出被拒路径的版本号
与当前地图版本。典型记录（D=60，buggy，10:05:00）：

```
path_label_version=1  path_gen_version=1  server_map_version=2
bad_on_eval_map=5     bad_on_own_map=0    reason=switch-induced
```

即：路径在自己的 v1 地图上零坏点，服务端却用 v2 地图判出 5 个不可走点。
`fixed` 三档在 CSV 中显式记为 `ZERO switch-induced misjudgement`。

## 6. 图表

| 文件 | 内容 |
|---|---|
| `figures/fig1_reject_timelines.png` | 三档延迟 × 三策略拒绝率时间线（尖峰宽度随 D_max 增大） |
| `figures/fig2_spike_decomposition.png` | D=60 事故尖峰的因果分解：尖峰全部来自版本错位 |
| `figures/fig3_strictness_curves.png` | FPR / CIR 两条严格程度曲线与选定工作点 k=1 |
| `figures/fig4_bump_timing.png` | bump 时机时间条、拒绝率反解点与回滚截断标记 |

## 文件说明

```
src/sim.py             地图/A*/判据 C-1,C-2/客户端同步/三种服务端策略
src/run_experiments.py 三档×三策略实验 + 严格程度扫描 + 反解，写 results/
src/make_plots.py      读 results 出图与诊断 CSV
src/selfcheck.py       27 项不变量断言
results/sim_results.json             全部逐 tick 序列与汇总
results/misjudgement_diagnostics.csv 误判路径版本号诊断
```
