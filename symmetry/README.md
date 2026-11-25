# symmetry

镜像区域的寻路复用验证：对称性判据、复用比例反解、三档区域一致性核对，
以及“复用比例 vs 误判风险”容差曲线。C / Python 双实现互证，仅用标准库。

```sh
python3 symmetry/repro_symmetry.py     # 主验证：三档 + 反例 + 曲线 + result.json
make -C symmetry test                  # C 版同一套判据/三档/诊断自测
```

Python 脚本可用 `python3 -I` 隔离运行（此时跳过可选的 matplotlib PNG，
HTML/CSV/JSON 照常生成）。产物写入 `artifacts/symmetry/`。

## 资源清单

- `resource_manifest.json` —— “资源清单”。地图 `four-wing-mirror`，
  世界 32×24，四个 16×12 区域 `NW/NE/SW/SE`；`NW↔NE`、`SW↔SE`
  分别为东西镜像对，附 12 条查询。
- `quadrants.navmap` —— 同一地图的极简文本版，供 C 自测读取。
- `--write-files` 可由脚本中的确定性蛇形布局重新生成这两个文件。

所有区域都是单缺口隔墙的树状走廊，任意两点间最短路唯一，因此
“镜像变换后的路径”和“独立 A* 路径”可以做**逐坐标**的严格一致比较，
不会被 A* 平局规则影响。

## 判据表达式

对两个 W×H 区域 A、B，东西镜像变换为 `ρ(x,y)=(W−1−x,y)`，则

```
D(A,B) = Σ_y Σ_x | walk_A[x,y] − walk_B[ρ(x,y)] |
Sym(A,B) ⇔ D(A,B) = 0
```

清单实测 `D(NW,NE)=D(SW,SE)=0`，跨对 `D(NW,SW)=8`。

## 复用比例：由镜像对数量反解

R 个区域、P 个互不重叠的确认镜像对：

```
N_run = R − P          # 基线 R 次 A*，每对 2→1
η = P / R              # 算力节省比例
P_min(R) = ⌈0.30 R⌉    # 三成门槛所需的最少镜像对
```

R=4、P=2 时 `η=2/4=50%`，只需算两遍。三档结果（一次运行的确定性计数）：

| R | P | η | P_min | A* 调用 | 扩展节点节省 | 全口径操作节省¹ | 路径一致率 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 | 2 | 50% | 2 | 48→24 | 50.00% | 37.51% | 48/48 = 100% |
| 8 | 4 | 50% | 3 | 96→48 | 50.00% | 37.59% | 96/96 = 100% |
| 16 | 8 | 50% | 5 | 192→96 | 50.00% | 37.71% | 192/192 = 100% |

¹ 含每对一次的 192 格判据扫描与 O(path) 镜像变换；C 与 Python 在 R=4
上得到完全相同的 8708→5442 ops。best-of-30 墙钟节省约 46–47%（参考）。

## 第一处差异坐标 + 所在区域

比较器从第 0 格起逐坐标比对两条路径，不一致时输出：区域 id、查询序号、
路径下标、local/world 两种坐标。验收三档均为 `first_diff=null`。

反例诊断（脚本与 C 均复现同一结果）：在 NE 隔墙 `(2,2)`（world `(18,2)`）
开一个洞，判据立即变为 `D=1` 并拒绝复用；强制复用 NW 镜像路径时，
第 1 条查询在 path index 13 首次分叉：

```
region=NE index=13
reused   local=(1,1) world=(17,1)
baseline local=(2,2) world=(18,2)
```

## 取舍与决策

容差 τ 从 0 放宽时，压力总体（220 个候选镜像对）上的复用比例上升，
但只要放进 **1 个**差异格，经验路径误判风险就达到 14.37%。曲线：

- `artifacts/symmetry/reuse_risk_curves.csv`
- `artifacts/symmetry/reuse_risk_curves.html`（悬停提示 + 数据表）
- `artifacts/symmetry/reuse_risk_curves.png`

**启用，但只启用严格档 `D=0`。** 真实舰队严格成对：一致率 100%、
节省约 50%（全口径 ≥37.5%），同时风险为 0；任何 τ>0 都会引入路径不一致，
过不了 100% 验收线。完整推导、二项错误接受概率公式与当次曲线数值见
`artifacts/symmetry/symmetry_report.md`。
