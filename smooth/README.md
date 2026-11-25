# smooth

视线检测（LOS）与路径平滑（string-pull）。`make test` 跑自检。

## 视线判据

格是单位方块 `[x,x+1]×[y,y+1]`，节点取格中心。对障碍格集合 B、弦段
p0→p1、角色半径 r：

```
LOS(p0,p1,r)  ⇔  min_{b∈B} dist(segment(p0,p1), square(b)) > r
```

即半径 r 的扫掠管不进入任何障碍格；**接触即阻挡，所以是严格不等式**。
`dist` 用解析法精确计算（分段二次距离函数的候选参数：端点、与方块
四个面的交点、四个角点的投影），自检里另有 2048 点密采样估计量交叉
验证，两者误差为 0（要求 ≤ 半格）。

旧实现的 Bresenham supercover 沿格心行走，会漏掉被线段实际穿过的格：
本用例的 (0,0)→(12,7) 中心线穿过 (4,3)、(9,6)，Bresenham 都没走到，
于是墙不可见、每条平滑弦都被放行——15 节点的阶梯路径永远不会收缩。
自检保留了该反例（`bresenham_misses_pillars`）。

## 平滑

- `smooth_greedy`：贪心拉线，半径感知 LOS；
- `smooth_min_points`：O(n²) DP，求满足 LOS 的最少保留子序列（本用例
  贪心结果与 DP 最优一致）。

平滑强度 = 半径 r。r 越小越贴近直线但擦障碍角；r 越大点数越多、间隙
越大。同一张图的扫描结果见 `tools/sweep.html` / `tools/sweep.csv`：
r=0 时 3 点（最差间隙 0.077 格），r=0.35 时 **4 点**（原 15 节点的
0.267 < 1/3），三条平滑弦全部通过 LOS，最差间隙 **0.4856 格 > r**，
14 条原始段中 11 条被弦吸收（保留段 0、10、13）。

## 文件

- `src/smooth.c` — 实现 + 自检（`make test`）
- `tools/proto.py` — Python 数值原型，`python3 -I proto.py`
- `tools/sweep.html` — 两条取舍曲线（点数/长度/间隙 vs r）与地图
- `tools/sweep.csv` — 扫描数据
- `tools/ANALYSIS.md` — 判据推导与标定过程
