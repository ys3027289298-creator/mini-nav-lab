# mini-nav-lab

导航与寻路基础模块，C 实现，每目录独立构建可自测。

| 目录 | 内容 |
|---|---|
| `grid/` | 可走性地图、四邻域 A*、阻挡与不可达 |
| `smooth/` | 视线检测、贪心路径平滑 |
| `region/` | 连通分量、同区域查询 |

上线前使用 [`RELEASE_CHECKLIST.md`](RELEASE_CHECKLIST.md)，并运行：

```bash
python3 tools/path_freshness_lab.py
```

该实验验证“寻路完成 → 角色出发”时间窗口内的路径失效风险、出发前末检成本，以及三档地图更新频率下的零带病出发门禁。
