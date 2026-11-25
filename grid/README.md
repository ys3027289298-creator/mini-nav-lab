# grid

`make test` 跑自检。

## `find_path` 边界契约

```c
FindResult find_path(const Map* m, int sx, int sy, int gx, int gy,
                     int* out, int cap);
```

`cap` 是 `out` 可容纳的 **格子 ID 数**；`FindResult.length` /
`required_length` 也按顶点数计，边数为 `length - 1`。负的 `status`
表示错误且不承诺写入路径；非负 `status` 是位掩码，`out[0..length)`
一定是合法、相邻、可走的前缀。

| `status` | 含义 | `out` 内容 |
|---|---|---|
| `FIND_OK = 0` | 找到完整路径且容量足够 | 完整路径，末点为目标 |
| `FIND_NEAREST` | 目标在另一连通分量，返回路径距离反解出的最近可达点 | 到降级点的完整路径 |
| `FIND_TRUNCATED` | 完整结果超过 `cap` | 从起点开始的 `cap` 个格子前缀 |
| `FIND_MISDIRECTED` | 降级点位于“起点—目标”方向的后方 | 仍写入，调用方可决定是否拒绝 |
| `FIND_FAR` | 反向路径步数超过策略预算 `r*=18` | 仍写入，严格策略可将其视为失败 |
| `FIND_E_ARG = -1` | `m/out == NULL` 或 `cap <= 0` | 不写入 |
| `FIND_E_START_BLOCKED = -2` | 起点越界或在障碍中 | 不写入 |
| `FIND_E_GOAL_BLOCKED = -3` | 目标越界或在障碍中 | 不写入 |

`FIND_NEAREST` 可以与 `FIND_TRUNCATED`、`FIND_MISDIRECTED`、
`FIND_FAR` 按位或。因此调用方不能只判断 `status != 0` 后把前缀误当成
完整路径。

## 判据表达式

令 \(B(x,y)\) 为阻挡值，\(I(x,y)=[0\le x<W\land 0\le y<H]\)。

可走性：

\[
Q(p)=I(p)\land \neg B(p)
\]

起点合法性：

\[
S_{\rm ok}=Q(s)
\]

连通分量 \(C(p)\) 由四邻域 flood-fill 得到（与 `region/` 同一规则）。
在起终点都可走时，可达性为：

\[
R(s,g)=[C(s)=C(g)]
\]

设 \(V(P)\) 为路径顶点数。输出缓存容量为 `cap`，模块内部搜索/缓存容量为
\(K=WH=192\)，则完整路径可放入的判据是：

\[
V(P^\*)\le \min(\text{cap},K)
\]

等价地，按边数 \(L=V-1\)：

\[
L(P^\*)\le \min(\text{cap},K)-1
\]

若另有一个容量为 \(C_{\rm cache}\) 的路径缓存，只需把上式的 `cap`
替换为 \(\min(\text{cap},C_{\rm cache})\)。超过上限时本实现 **显式截断**：
写入 `cap` 个起点前缀，置 `FIND_TRUNCATED`，并在 `required_length`
给出完整顶点数；不做沉默截断。

## 目标不可达时的“最近”度量

当 \(Q(s)\land Q(g)\) 成立但 \(C(s)\ne C(g)\) 时，普通合法路径距离

\[
d_{\rm walk}(v,g)=\infty,\qquad \forall v\in C(s)
\]

对所有起点分量中的候选点都相同。因此，若把“路径距离”严格定义为
“不穿过障碍的合法路径长度”，跨连通分量的最近点实际上 **未定义**；
直线距离可以选点，但它会把墙厚和绕行成本忽略掉。

为使“用路径距离反解”可执行，这里采用显式声明的扩展距离
\(D^\dagger(v,g)\)：在反向 Dijkstra 中允许进入障碍格，但每进入一个
障碍格收 `BLOCK_CROSS_COST=1024`，普通一步代价为 1。由于本地图最长
简单路径小于 1024，最小化 \(D^\dagger\) 等价于按

\[
(\text{障碍穿越次数},\ \text{普通步数})
\]

做字典序最小化。实现从目标反向计算距离场，再在 \(C(s)\) 中选择

\[
v^\*=\arg\min_{v\in C(s)}D^\dagger(v,g)
\]

并沿 BFS parent 反解出起点到 \(v^\*\) 的真实合法路径。这个度量考虑了
墙厚和绕行；它不是欧氏距离，也不是跨分量的合法路径，而是带明示越障
代价的路径距离投影。

## 用可达率/误导率决定降级策略

在固定 `16×12` 地图上生成随机障碍和墙段，只保留起终点可走但不连通的
请求；一次标定得到 `N=1175` 个不可达请求。令 \(r_i\) 为第 \(i\) 个请求
所选投影点的反向普通步数，\(I_i(r)=[r_i\le r]\)。

可达率：

\[
A(r)=\frac{1}{N}\sum_i I_i(r)
\]

误导率定义为降级终点落到“过起点、垂直于目标方向”的后方半平面：

\[
M(r)=
\frac{\sum_i I_i(r)\,
[(v_i^\*-s_i)\cdot(g_i-s_i)<0]}
{\sum_i I_i(r)}
\]

本次标定的采样曲线：

| \(r\) | 可达率 \(A(r)\) | 误导率 \(M(r)\) |
|---:|---:|---:|
| 10 | 81.6% | 0.5% |
| 12 | 86.5% | 0.9% |
| 14 | 90.5% | 1.0% |
| 16 | 94.2% | 1.4% |
| **18** | **96.5%** | **1.7%** |
| 20 | 98.0% | 1.8% |
| 24 | 99.4% | 2.0% |

取策略门槛 \(A(r)\ge95\%\) 且 \(M(r)\le5\%\)，最小采样预算为
\(r^\*=18\)。曲线和生成脚本在：

- `../artifacts/fallback_curves.png`
- `../artifacts/fallback_curves.csv`
- `../artifacts/fallback_policy.py`

取舍落地为两层行为：

1. **默认体验策略**：返回 \(v^\*\) 和真实可走前缀，同时给出明确状态，
   避免玩家点击墙后时角色毫无反应。
2. **严格/自动导航策略**：调用方把含 `FIND_MISDIRECTED` 或
   `FIND_FAR` 的结果当作硬失败。这样“直接失败”仍是可选策略，但错误
   不再沉默，也不会把误导性终点伪装成成功到达目标。

这些曲线是该固定地图分布上的标定结果，不是所有障碍分布下的通用保证；
更换地图尺寸、障碍密度或玩家方向误差阈值后应重新生成。

## 四个判据叶子用例

自检中的四个主用例分别对应决策树的四个叶子：

1. `case_reachable_fit`：开放地图 `(0,0) -> (2,2)`。
   `status=FIND_OK`，顶点数 `5`。
2. `case_unreachable_path_distance_nearest`：`x=8` 全高墙，并在
   `(9,4..6)` 加厚。起点 `(0,5)`，目标 `(10,5)`。
   欧氏最近是 `(7,5)`（cell `87`），路径距离投影选择薄墙侧
   `(7,3)`（cell `55`）；`status=FIND_NEAREST`，路径含 `10` 个顶点，
   反向距离为 1 次穿越、4 个普通步。
3. `case_blocked_start`：以墙格 `(8,0)` 为起点。
   返回 `FIND_E_START_BLOCKED=-2`，不写 `out`。
4. `case_over_limit_explicitly_truncated`：开放地图
   `(0,0) -> (3,3)` 的完整路径含 `7` 个顶点；`cap=3` 时返回
   `FIND_TRUNCATED`，写入前 3 个顶点，`required_length=7`，且前缀与
   完整路径前 3 项一致。`bonus_fallback_cap_combination` 额外验证
   `FIND_NEAREST | FIND_TRUNCATED` 组合状态。

### 四情形未覆盖的分支

上面的四叶表默认“目标合法”。**起点合法但目标在障碍里或越界**是未被
这四类覆盖的第五种输入；它不能被误归类为“目标不可达后返回最近点”，
因为请求目标本身不是一个合法格子。实现显式返回
`FIND_E_GOAL_BLOCKED=-3`，自检项
`uncovered_goal_blocked_or_out_of_bounds` 同时覆盖阻挡目标和越界目标。

另一个需要写明的度量空白是：若不定义 \(D^\dagger\) 之类的跨分量扩展
距离，“用合法路径距离选跨分量最近点”在数学上无定义；不能沉默地回退
成欧氏距离。
