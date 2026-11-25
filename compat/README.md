# 地图文件兼容层 —— 交接备注

写给三个月后的接手者。只讲一件事：**地图文件没有版本号，改了格式为什么不能只靠全量重导，兼容层该保留几代**。代码在本目录，跑 `make check` 可复现下面所有数字。

先给结论，再给依据。

---

## 0. 结论

1. **加版本号，从这一版起。** 这是本目录做的唯一一件不可逆的事：gen 2 有 magic + version 字段，以后再改格式，读入时一眼就能分辨，不会再走到「猜结构」这一步。
2. **保留 1 代旧 reader（gen 0），保留到所有旧文件重导完成，然后删掉。** 不是 2 代，也不是 0 代。理由见 §3。
3. **「旧图读入成功率 100%」和「只保留一代」并不冲突**，因为今天盘上只有 gen 0 一种旧格式。这条要求真正的代价是：**只要还有一张没重导的旧图，就不能删 gen 0 reader**。删除的触发条件是文件计数归零，不是时间。
4. **升级唯一会丢精度的地方是 gen 1 的亚格 query 坐标**（gen 0 没有 query，无损）。丢的精度是小数点后的部分，量化规则必须由 reader 和重导工具**共用同一个函数**，否则同一个旧图走两条路会得到两条不同的路径 —— 本目录的 bench 专门盯着这一点。

---

## 1. 兼容判据（读入时看什么）

用户的原话是「先看文件头是否能按当前结构解析，能就直读，不能就按旧结构解析再升级」。落到代码里是 `map_detect_version()`（`src/format.c`），**全模块只在这里做一次判断**，reader 和工具都不各自猜：

```
detect(b, n) = 2                   如果 v2_header_ok(b, n)
             = 1                   如果 n == 4 + w*h + 16, w,h = le16(b)
             = 0                   如果 n == 192 且每个字节 ∈ {0,1}
             = MAP_DET_FUTURE      如果 b[0..3] == "MNL\0" 且 version != 2
             = MAP_DET_UNKNOWN     其他

read(b, n, q) = parse_v2(b, n)                 detect == 2
              = upgrade_1(parse_1(b, n))       detect == 1，且本构建保留 gen 1
              = upgrade_0(parse_0(b, n), q)    detect == 0，且本构建保留 gen 0
              = MAP_ERR_VERSION                detect 命中已删除的代数
              = MAP_ERR_FORMAT                 其他
```

三条关键性质，都有测试覆盖：

- **magic 优先，且不可降级。** 只要头 4 字节是 `MNL\0`，就是 gen 2 文件；payload 坏了就报 gen 2 的错误，**绝不会**被当成旧格式再试一遍。否则一个损坏的新文件会被误读成一张看似合法的旧地图，这比读失败严重得多（`test_rejections` / `gen2_short_payload`）。
- **删掉 reader 的代数仍能被识别。** `detect` 不看本构建是否保留该代；`read` 才返回 `MAP_ERR_VERSION`。这样「我们不读这种格式了」和「这不是地图」是两种不同的答复，迁移统计能分清（`make test-drop-v1`）。
- **旧格式靠长度等式区分。** gen 1 是 `4+w*h+16`，gen 0 固定 192 字节。两者不可能同时成立：gen 0 前四字节是 cell 值，w/h 只可能是 0/1/256/257，代入 `4+w*h+16 == 192` 得 `w*h == 172`，无解。

### 各代格式（都在 `include/mapfile.h` 顶部）

| 代 | 有 magic | 有维度 | query | 长度 | 升级损失 |
|---|---|---|---|---|---|
| gen 0 | 否 | 否（固定 16×12） | 无 | 192 | **无** |
| gen 1 | 否 | 是 | f32 亚格坐标 | `4+wh+16` | **亚格精度 → 整格** |
| gen 2 | 是 | 是 | u16 整格 | 变长（可选 RLE） | 当前格式 |

gen 1 在本仓库**没有任何文件**，写它的原因见 §3。

---

## 2. 保留几代：从数量反解

「每代旧格式需要一段解析代码，保留代数由还有多少旧地图没重导决定」。代码在 `src/retention.c`，两条规则给出两个答案，都实现了，因为它们在低文件量时会分叉：

**规则 A（本项目的硬要求：读入成功率 100%，不允许重导兜底）**

```
kept_mask = OR over g of (files[g] > 0) << g
```

即：某一代只要还有 1 个文件没重导，就必须留着它的 reader。这是 `map_min_kept_mask()`，一行逻辑，因为要求是绝对的。没有取舍余地可言 —— 除非放弃 100% 读入。

**规则 B（允许一次性重导兜底，按成本选）**

对每一代 g：

```
保留 g 的 reader  ⟺  files[g] > LOC(g)
```

其中 `LOC(g)` 是从源码实测的 reader 行数（`make loc`），一次重导按「1 行 reader」计价。`map_plan_retention()` 把 2^2 种组合全枚举、打分 `lines(mask) + 1.0 × reexport_files(mask)` 并按成本排序，所以能看到整条曲线而不只是最优点。

实测行数（`make loc`，去空行去纯注释）：

```
src/gen0.c       38     ← gen 0 reader
src/gen1.c       45     ← gen 1 reader
src/gen2.c      149     ← 当前格式，非兼容代码
src/format.c    114     ← 分发表，任何构建都付
```

于是规则 B 的两个阈值是 **38 个文件**（gen 0）和 **45 个文件**（gen 1）。

> 注意：`1 行 = 1 次重导` 是模型里唯一的人为假设。它不是测出来的，是「一次人工/脚本重导大约值多少行常驻代码」的判断。想改就改 `bench.c` 里的 `unit`，阈值会跟着变。

---

## 3. 用两条曲线定代数：三档结果

三档是**旧文件数量**，不是地图尺寸 —— gen 0 按定义固定 16×12，尺寸无法变化。所以这里唯一能变的量就是「还有多少张没重导」，正好是决策的自变量。

`make bench` 的输出（节选）：

```
RETENTION  existing old files readable vs reader lines
  generations kept       small  mid    large   lines reads
  0 (re-export all)        0.0%   0.0%   0.0%    114 no old file; all re-exported
  1 (gen 0)              100.0% 100.0% 100.0%    152 every old file that exists today
  2 (gen 0 + gen 1)      100.0% 100.0% 100.0%    197 today's files, plus gen 1 if it ever appears

RETENTION DECISION  old-file count -> readers kept
  RULE A: 100% read, no re-export allowed
    gen 0 reader = 38 lines (measured), gen 1 = 45 lines
  RULE B: re-export allowed, 1 unit per file
    keep gen0 reader while un-re-exported files > 38
    keep gen1 reader while un-re-exported files > 45
  tier=small  files=8     A: keep mask 0x1 (gen0)   B: keep mask 0x0, 114 lines, 8 re-exports
  tier=mid    files=80    A: keep mask 0x1 (gen0)   B: keep mask 0x1, 152 lines, 0 re-exports
  tier=large  files=800   A: keep mask 0x1 (gen0)   B: keep mask 0x1, 152 lines, 0 re-exports
```

读法：

- **规则 A 下，三档完全相同：保留 1 代。** 这是有用的发现 —— 100% 读入要求让「数量」几乎不起作用，只要有文件就得留。真正起作用的是「有没有文件」，不是「有多少」。
- **规则 B 下，只有小档翻盘。** 8 个文件时，一次性重导 8 张的成本不高于养 38 行 reader（相同时按「少留 reader」破平），所以规则 B 选 0 代；80 和 800 档都选 1 代。
- **交接时该选哪个：A。** 因为用户明确要求「旧图读入成功率必须是百分之百」，规则 B 的小档方案会让那 8 张图在重导完成前读不进来，直接违反要求。**这就是取舍的落点：A 比 B 多花 38 行常驻代码，换来的是旧图永不掉队。**
- **gen 1 之所以没进保留清单**，不是因为它便宜，而是因为**盘上 0 个文件**。它的 reader 已经写好、已测过（见下），阈值 45 也算了。哪天出现第一张 gen 1 文件，答案自动变成 mask 0x3，不需要重新设计。

### 关于 gen 1 这个「虚构」的一代

必须写清楚，免得三个月后有人以为漏了什么：

- 本仓库 `grid/`、`smooth/`、`region/` 三个模块的 `Map` 结构是 `unsigned char blocked[16*12]`，**只在内存里，从不落盘**。所以严格说，今天**一张旧地图文件都不存在**，「旧图数量」在真实世界里是 0。
- 因此 bench 里所有旧图都是 `tests/fixtures.c` 合成的，形状按「这三个模块如果当年写盘会长什么样」构造。报告第一行就写着这一点。
- **gen 1 是故意留的第二个代**：只有一代（gen 0）时，「保留代数由文件数量反解」这套机制根本没被验证过 —— 一个布尔值也能凑出答案。gen 1 让 N=2 的情况真的跑起来：分叉、曲线、删 reader 后的行为、长度等式区分，全部有覆盖。代价是 45 行，且不读任何真实文件。
- 如果你接手时认为「永不存在的格式不值得留代码」，把 `MAP_KEEP_V1` 置 0 即可（`include/mapfile.h` 顶部），`make test-drop-v1` 会验证删掉后仍能识别其形状并返回 `MAP_ERR_VERSION` 而不是 `MAP_ERR_FORMAT`。但**那时你必须先有 gen 1 的替代验证**，否则等于删掉了这套机制唯一的双代回归。---

## 4. 验收：100% 读入 + 逐点相同 + ≥99%

`make bench` 的 ACCEPTANCE 段，按**代 × 档**分开报，因为各代丢精度的方式不同，混在一起会掩盖问题：

```
ACCEPTANCE  compat-read path vs re-export-then-read path
tier   gen    files   read    read%   shared points   shared% snapped(same path)
small  gen0       8      8  100.00%      133/133      100.00% (none)
small  gen1       8      8  100.00%      133/133      100.00% sx=8 sy=8 gx=8 gy=8
mid    gen0      80     80  100.00%     1318/1318     100.00% (none)
mid    gen1      80     80  100.00%     1318/1318     100.00% sx=79 sy=80 gx=80 gy=80
large  gen0     800    800  100.00%    13346/13346    100.00% (none)
large  gen1     800    800  100.00%    13346/13346    100.00% sx=799 sy=799 gx=799 gy=799
```

每一项怎么算的：

- **`read`**：把每个旧文件交给 `map_read()`，要求 `MAP_OK`。任何不是 100% 都让 `bench` 退出码非零（`READ_FLOOR_PCT = 100.0`），不是打印个百分比就算完。
- **两路对照**：同一个旧文件跑两遍 ——
  - **路 A（兼容读）**：`旧字节 → map_read() 升级 → 算路径`
  - **路 B（重导）**：`旧字节 → reexport_to_gen2() 写出 gen 2 → 读回 → 算路径`
  两条路径用**同一份 A* 代码**（`src/nav.c`，四邻域、f 最小、平局取先入、曼哈顿启发）在**各自的 cells 和 query** 上计算。算法细节刻意和 `grid/src/grid.c` 对齐，否则「路径不同」可能只是 tie-break 不同，信号会被淹没。
- **`shared`**：逐点比较，相同点数 / max(nA, nB)。低于 99% 直接失败（`SHARED_FLOOR_PCT = 99.0`）。实测三档六个格子全部 **100%（完全相同）**，远高于 99% 下限。
- **`snapped(same path)`**：**snap 丢了精度、但路径没变**的文件数，按字段记。这是关键区分：
  - `gen0 (none)` —— gen 0 没有 query，本来就没东西可丢，不是「碰巧没丢」。
  - `gen1 sx=799 ...` —— gen 1 每个文件的 query 都带亚格小数（fixture 故意生成的），整格量化必然丢掉小数部分。但 A* 的起点落在**同一个格子**，所以路径逐点相同。
  - **这不是 bug，而是代价被量化了**：丢的是格内位置，没丢格号。任何依赖格内精确位置的逻辑（比如把 query 当连续坐标算朝向）会受影响，依赖格号寻路的不会。
- **额外的硬检查**：两路读出的 **cells 必须相同**（否则路径比较没意义），**query 必须相同**（query 是 gen 1 升级唯一可能丢的东西，不能靠「路径碰巧一样」去推断）。两处任一不同都会让 bench 失败。这两项在 ACCEPTANCE 里全绿 —— 说明两路升级结果在 query 层面也一致，不只是路径层面。

### 不同时怎么报「哪个字段丢了精度」

`q_lossy` 是**逐字段的位掩码**（bit 0=sx, 1=sy, 2=gx, 3=gy），不是「丢了 / 没丢」一个布尔值。触发路径差异时，报告直接点名：

```
PATH DIFF gen=1 first_diff=0 shared=0/14 snap_fields=sx,sy,gx,gy
  at 0: compat=(4,10) reexport=(5,11)
```

协议是这样定的：**grid 的实现里，除 query 量化外没有任何一步允许丢精度** —— cells 是 0/1 直拷，RLE 无损且只在更短时才用，`map_write` 是逐字节精确的。所以只要路径不同，责任**必然**出在 `map_q_quantise()`，`snap_fields` 就是完整的病因，不用再查别处。

### 分叉探针（证明这套验收不是空转）

正常配置下三档全部 100% 相同。为证明「相同」是测出来的而不是永远成立，bench 末尾故意让 reader 和工具用**不同的量化规则**（reader 向下取整、工具四舍五入）：

```
DIVERGENCE PROBE  reader floors, offline re-export rounds.
  small  gen1 files=8    read=8    path_diffs=6    shared=0/101 (0.00%) moved_by_field: sx=6 sy=6 gx=6 gy=6
  mid    gen1 files=80   read=80   path_diffs=59   shared=246/1061 (23.19%) moved_by_field: sx=58 sy=59 gx=59 gy=59
  large  gen1 files=800  read=800  path_diffs=614  shared=2576/11050 (23.31%) moved_by_field: sx=613 sy=613 gx=613 gy=613
```

结论：**量化规则一旦分叉，超过一半的 gen 1 图路径会变，共享点掉到 0–23%，远破 99% 线。** 这就是为什么 `map_q_quantise()` 必须是 reader 和工具共用的**唯一**量化入口（`src/format.c`），谁都不许自己写一份。

---

## 5. 三个月后怎么接

```sh
cd compat
make check        # 单元测试 + bench + 三种删 reader 的构建验证
make loc          # 重新量 reader 行数（改了代码就重跑，曲线会跟着动）
make bench        # 只看验收数字
```

删 gen 0 reader 的正确时机与步骤：

1. 统计还没重导的 gen 0 文件数（本仓库目前 = 0，因为从来没有文件）。
2. 若为 0：把 `MAP_KEEP_V0` 置 0，跑 `make check`。`test-drop-v0` 会编译出删除 gen 0 reader 的构建，并确认 gen 0 形状仍被识别、`read` 返回 `MAP_ERR_VERSION`、工具也拒绝转换。
3. 若不为 0：**不要删**。先跑 `build/reexport` 把剩下的转完。工具用法：
   ```sh
   build/reexport --query sx,sy,gx,gy old.map new.map   # gen 0 需要外部给 query
   build/reexport old1.map new1.map                     # gen 1 自带 query
   ```
   输出会打印 `lossy_fields=`，非空即表示该文件升级时丢了亚格精度。

改格式的流程（这次做对了，以后照做）：

1. 加一代 gen 3，**不要**在原布局上改。`map_detect_version()` 加一个分支。
2. 旧的 gen 2 降级为「又一代 legacy」，填进 retention 表。
3. 跑 `make bench`，看新曲线，按 §2 反解保留代数。
4. 改完先跑 `make check`，确认 ACCEPTANCE 里每一行 `read%` 仍是 100、`shared%` 仍 ≥99。

---

## 6. 已知边界（别当成 bug）

- **fixtures 是合成的**，不是真实关卡数据。所有百分比描述的是 reader 和升级逻辑，不是你的地图库。
- **gen 1 盘上无文件**，是回归用的第二维；`MAP_KEEP_V1=0` 可删，但见 §3 的警告。
- **规则 B 的 `1 行 = 1 次重导`** 是假设值，不是测量值。
- **量化默认 `MAP_Q_FLOOR`（向下取整）**，因为它复现旧运行时的 `(int)coord`。如果你希望旧图升级后更贴近原坐标，`MAP_Q_ROUND` 就在那里，但**必须同时**改 reader 和工具的默认值，否则又会掉进分叉探针那个坑。
- **ACCEPTANCE 的 100% 与 99% 是两条独立的线。** 100% 是读入成功；99% 是路径相同点占比。前者不达标直接失败，后者同理。两者不可互相补偿。

---

## 附：文件清单

| 路径 | 内容 |
|---|---|
| `include/mapfile.h` | 三代格式定义、`MapFile`、detect 判据、量化开关 |
| `include/nav.h` | 两路对照用的 A* |
| `include/reexport.h` | 一次性重导接口 |
| `include/retention.h` | 保留代数反解与成本枚举 |
| `src/format.c` | 状态码、量化、**detect 分发** |
| `src/gen0.c` | gen 0 reader（38 行） |
| `src/gen1.c` | gen 1 reader（45 行） |
| `src/gen2.c` | 当前格式读写 + RLE |
| `src/reexport.c` | 旧 → gen 2 |
| `src/retention.c` | 计数 → 保留代数 |
| `src/nav.c` | A* + 逐点比较 |
| `tools/bench.c` | 验收 / 曲线 / 分叉探针 |
| `tools/reexport.c` | 命令行重导工具 |
| `tools/loc.awk` | reader 行数测量 |
| `tests/fixtures.c` | 合成旧图 |
| `tests/test_mapfile.c` | 单元与删除后行为测试 |
