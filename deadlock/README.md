# deadlock

多角色共同走向目标时的确定性卡死矩阵实验。只依赖 Python 3 标准库。

## 运行

```sh
make test
# 或仓库根目录：
python3 deadlock/src/deadlock_lab.py run --out artifacts/deadlock-lab
```

单个四维单元：

```sh
python3 deadlock/src/deadlock_lab.py one \
  --n 2 --distribution opposed --goal same --width-d 1
```

## 方法

卡死谓词在连续 30 帧内检查一对尚未到达目标的角色：

```text
d_ij < r_i + r_j + ε_d
|v_i - v_j| ≤ ε_v
```

四维取值为：

- `n = 1,2,3,4,6`
- 起始分布：`column | row | arc | opposed`
- 目标：`same | distinct`
- 通道宽度：`w/D = 1,1.25,1.5,2,2.5,3`，其中 `D=2r`

全笛卡尔积是 `5×4×2×6=240`；卡死谓词要求至少有一对，因此 `n=1` 的 48 个负对照不进入可判定矩阵，剩余 `4×4×2×6=192` 例。

工具会运行全量预言机、200 次重复抽样曲线、25% 四维分层样本、漏掉组合审计，并把所有已复现卡死单元闭合进最终回归矩阵。生成结果在 `../../artifacts/deadlock-lab/report.md`。

这是一个故意保留局部贪心、无全局通行权协商的参考控制器，用来确定哪些维度组合能复现卡死；它不是现有 `grid/` A* 的动态避障实现。
