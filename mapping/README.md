# mapping

旧地图区域编号（`0..N-1` 连续）→ 新语义标签体系
（`main_city` 主城 / `wilderness` 野外 / `dungeon` 副本）的字段映射迁移实验。

```sh
make test        # 等同 python3 migrate_map_labels.py --out ../artifacts/mapping
```

纯 Python 标准库可跑（CSV/JSON/Markdown）；装了 matplotlib 会额外生成曲线图。
固定 seed，结果确定性；门限不过时进程退出码为 `2`。

## 映射完整性表达式

设旧编号集合 \(O=\{0,\dots,N-1\}\)，标签集合
\(\Lambda=\{\text{main_city},\text{wilderness},\text{dungeon}\}\)，
部分映射 \(M:O\rightharpoonup\Lambda\)：

- **C1 每个旧编号必须有标签**：
  \(\forall i\in O,\ M(i)\in\Lambda\)，即 \(\operatorname{dom}M=O\)。
- **C2 每个新标签至少有一个旧编号映射过来**：
  \(\forall\ell\in\Lambda,\exists i:M(i)=\ell\)，即 \(M(O)=\Lambda\)（满射）。

合起来：`dom(M) = O ∧ M(O) = Λ`。映射天然多对一，不能一对一硬编码——
硬编码表中没有任何 `dungeon` 行（新副本语义在旧表中不存在），C2 初始必失败。

## 无法硬编码的编号：包围盒包含推断

旧编号 \(i\) 的格子集合 \(P_i\) 对新区域 \(r\) 的包围比例：

\[
c(i,r)=\frac{|\{p\in P_i:p\in B_r\}|}{|P_i|},\qquad
s(i,\ell)=\max_{r:\lambda(r)=\ell}c(i,r).
\]

取得分最高的标签，当 \(s_1\ge\theta\) 且严格领先第二名时自动采用；
低于阈值（`below_threshold`）或并列（bbox 相交导致 `tie`）转人工。
**\(\theta=1.00\) 即“全部格子都落在某个新区域内”的原始规则。**

## 成功率反解人工数

未知编号 \(U=N-K\)，自动 \(A\) 个，则推断成功率与人工确认数：

\[
r=\frac{A}{U},\qquad A=rU,\qquad H=U-A=(1-r)U.
\]

## 阈值与三档结果（seed 固定，可复现）

阈值在独立校准集（N=80/U=60）的准确率–人工量曲线上按
**准确率 ≥ 95%（整数判据 `20w ≤ A`）+ 安全前缀 + 人工最少** 冻结为
**θ\* = 0.90**，三档正式运行不再用各档真值调参：

| 档位 | N | 硬编码 K | 未知 U | 自动 A | 人工 H | 成功率 | 自动准确率 | 最终覆盖率 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| small  | 24  | 12  | 12  | 10  | 2  | 83.3% | 100.0% | 100% |
| medium | 80  | 36  | 44  | 35  | 9  | 79.5% | 97.1%  | 100% |
| large  | 240 | 100 | 140 | 112 | 28 | 80.0% | 96.4%  | 100% |

误推记录（旧编号 + 实际标签，完整字段见
`../artifacts/mapping/mispredictions.csv`）：

| 档位 | old_id | 误推标签 | 实际标签 |
|---|---:|---|---|
| medium | **79** | wilderness | **main_city** |
| large | **6** | wilderness | **dungeon** |
| large | **138** | wilderness | **dungeon** |
| large | **150** | wilderness | **dungeon** |
| large | **156** | wilderness | **main_city** |

小档在 θ\* 下无误推。所有阈值点上的误推台账见
`sweep_mispredictions.csv`。

## 产物（`../artifacts/mapping/`）

- `report.md` — 完整中文报告（表达式、公式、曲线表、误推清单）
- `curves.png` — 校准集 + 三档的准确率 / 人工工作量曲线与 θ\*
- `bbox_overview_medium.png` — 中档包围盒、格点真值与误推编号总览
- `mapping_<tier>.csv` — 每档完整映射表（含来源与审计真值列）
- `mispredictions.csv` — θ\* 正式误推清单（old_id ↔ actual_label）
- `sweep_mispredictions.csv` — 所有 θ 扫描点的误推台账
- `curves.csv` — 曲线原始数据；`summary.json` — 门限机读结果

## 数据说明

仓库没有提供真实旧地图导出，因此这里使用**带已知真值的确定性合成地图**
演示完整决策闭环；没有真值就无法计算“推断准确率”曲线。真实迁移中，
该曲线应由人工标注的校准样本给出，θ\* 冻结后再跑全量。
