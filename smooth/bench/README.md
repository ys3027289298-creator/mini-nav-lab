# smooth/bench — A/B 贪心平滑对照实验

对比两种只在停止条件上不同的贪心平滑：

- **A（正向）**：`for (j=i+1; j<=n-1; j++) if (!los(i,j)) break;` —— 第一个 LOS 失败即停，保留 j−1。
- **B（反向）**：`for (j=n-1; j>i+1; j--) if (los(i,j)) break;` —— 从尾回扫，第一个可见即停，保留 j（即 `../src/smooth.c` 的现实现）。

## 构建与运行

```sh
make
./bench <输出目录> <地图模式> <LOS 语义> [档位起 [档位止]]
# 例：./bench ../../artifacts random40 strict
make run       # 跑全部头条场景
make report    # 由 artifacts 下的 CSV 生成 smooth_report.html
```

档位：0=32×24，1=128×96，2=512×384。

## LOS 两种语义

- `plain`：与 `../src/smooth.c` 完全一致的 Bresenham 超覆盖，对角步不检查侧邻格。
- `strict`：对角步额外要求两个正交邻格都可走（禁止切角）。这是触发 A 版退化的几何前提——贴墙 90° 弯在 `los(i, i+2)` 即失败。

## 地图模式

| 模式 | 几何 | 用途 |
|---|---|---|
| `random` | 25% 随机障碍 | 仓库原 LOS 下的对照，A/B 都不退化 |
| `random40` | 40% 随机障碍 + strict LOS | 选型主场景，小图 A==B，大图分道 |
| `staircase` | 实心墙中的阶梯廊道 + 周期开阔带 | A 退化极限：r_A 随规模跌到 0.17 |
| `stair2` | 高开阔带楼梯 | 净距权衡曲线（全量 B 贴墙，窗口 B 保净距） |
| `pillars`/`dense`/`forest`/`walls`/`pinball`/`snake`/`zigzag`/`lanes`/`corridor`/`comb`/`rooms` | 探索性生成器 | 复现/排查几何敏感性 |

## 输出文件（写入 artifacts/）

- `smooth_bench_<模式>_<LOS>.csv`：每档每版的输出点数、减少比例、LOS 试探次数、最小净距、平均跨越长度。
- `smooth_sweep_<模式>_<LOS>.csv`：窗口化 B 在不同试探起点 S 下的同一组指标，用于定 S 的两条曲线。

## 核心恒等式（实测三档全部精确成立）

A 版试探区间互不重叠，故 `P_A = n + m − 3`；由减少比例 `r=(n−m)/n` 反解：
`P_A = n(2−r) − 3`。当近场视线处处失败，平均跨越 reach→1、r→0、P_A→2n，A 退化为原路径。
