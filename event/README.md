# event

动画事件绑定 × 路径平滑的根因复现与修复自检。

```sh
make test                 # 构建并跑全部 key=0/1 自检
python3 run_experiment.py # 额外：5 次基准聚合并在 ../artifacts 出 CSV/JSON/图
python3 make_charts.py    # 只根据 ../artifacts/*.csv 重画两条曲线
```

程序在同一条 43 点绕路上跑三档贪心 LOS 平滑（`cap=2 / 4 / 不限`）：

- 旧绑定 `事件 -> 原始路径点序号 a_e`：平滑删点后序号重排，`a_e` 被删即丢事件。
  逐档打印 `MISS level=… event=… anchor=… x=… y=… cell=…`。
- 修复绑定 `事件 -> 归一化弧长位置区间 [u_e-w, u_e+w]`，两种查询结构都验证：
  有序区间 + 单调游标（摊还 O(1)），以及预光栅化的弧长量子直接表 `qmap`
  （每次查询与序号直查同为一次寻址，满足查询开销增幅 ≤10% 的硬门）。
- `GATE` 行给 11 批 ×1024 万次查询的中位/最佳比值，`query_overhead_le_10pct=1`
  才通过；`CSV_LOSS / CSV_LAW / CSV_COST` 是两条决策曲线的原始数据。

根因、公式推导、取舍与三档结果表见 `../artifacts/animation-event-root-cause.md`。
