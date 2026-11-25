# multigoal

多目标点顺序复现与穷举/贪心对比。仅依赖 Python 3 标准库。

```sh
python3 multigoal/repro_multigoal.py
# 可选：--budget-ms 16 --tiers 3,6,9 --curve-max 10
```

脚本会：

1. 复现循环内 `return`/`break` 导致只到第一个点的问题；
2. 用四邻域 A* 计算点间代价，并穷举 `n!` 种访问顺序；
3. 对比最近邻贪心，输出三档点数的代价、顺序和偏差；
4. 写 `artifacts/multigoal/curve.csv` 与可悬停查看数值的 `curves.html`。

默认交互耗时上界为 `100 ms`。上界和机器速度不同时，用脚本实测系数反解：

\[
n_{\max}=\max\{n:a\,n\cdot n!+\tau n(n+1)/2\le B\}.
\]

本次参考运行（100 ms 上界）中，成对缓存代价后可穷举到 `n=9`；`n=10`
约 0.8 s，因此在 `n=10` 切换到最近邻贪心，当次贪心偏差为 3.28%。
若采用单帧 16 ms 上界，则切换点为 `n=9`。
