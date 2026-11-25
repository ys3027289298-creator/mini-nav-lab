#!/usr/bin/env python3
"""Generate a self-contained HTML report comparing smoothing A vs B.

Reads the bench/sweep CSVs written by bench.c and emits
artifacts/smooth_report.html with inline SVG charts (no external deps),
a hover tooltip, light/dark themes, and table views for every chart.
"""
import csv
import html
import math
import os

ART = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "artifacts")
ART = os.path.normpath(ART)

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK_MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"


def read_csv(name):
    with open(os.path.join(ART, name), newline="") as f:
        return list(csv.DictReader(f))


def bench_rows(tag):
    return read_csv(f"smooth_bench_{tag}.csv")


def sweep_rows(tag):
    return read_csv(f"smooth_sweep_{tag}.csv")


def group(rows, key):
    out = {}
    for r in rows:
        out.setdefault(r[key], []).append(r)
    return out


# ---------------- tiny SVG line chart ----------------
class LineChart:
    def __init__(self, title, x_label, y_label, width=520, height=300,
                 y_log=False, y_max=None, y_min=0.0):
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        self.w, self.h = width, height
        self.ml, self.mr, self.mt, self.mb = 56, 16, 40, 46
        self.series = []
        self.refs = []
        self.y_log = y_log
        self.y_max = y_max
        self.y_min = y_min
        self.x_ticks = None

    def add_series(self, name, pts, color):
        self.series.append((name, pts, color))

    def add_refline(self, y, label, color=INK_MUTED):
        self.refs.append((y, label, color))

    def _yscale(self, y):
        top, bot = self.mt, self.h - self.mb
        if self.y_log:
            lo, hi = math.log10(self._lo), math.log10(self._hi)
            return bot - (math.log10(max(y, self._lo)) - lo) / (hi - lo) * (bot - top)
        hi = self.y_max if self.y_max is not None else self._hi
        return bot - (y - self.y_min) / (hi - self.y_min) * (bot - top)

    def _xscale(self, x, xmin, xmax, logx):
        l, r = self.ml, self.w - self.mr
        if logx:
            lo, hi = math.log10(xmin), math.log10(xmax)
            return l + (math.log10(x) - lo) / (hi - lo) * (r - l)
        return l + (x - xmin) / (xmax - xmin) * (r - l)

    def render(self):
        all_x = [p[0] for _, pts, _ in self.series for p in pts]
        all_y = [p[1] for _, pts, _ in self.series for p in pts]
        logx = any(s and s[0][0] >= 2 for s in [self.series[0][1]]) if self.series else False
        logx = all(abs(p[0]) >= 1 for _, pts, _ in self.series for p in pts) and \
            max(all_x) / max(min(all_x), 1) > 8
        xmin, xmax = min(all_x), max(all_x)
        if logx:
            xmin = max(xmin, 1)
        self._lo = max(min(all_y + [r[0] for r in self.refs]), 1e-9)
        self._hi = max(all_y + [r[0] for r in self.refs])
        if not self.y_log and self.y_max is None:
            self._hi *= 1.08
        if self.y_log:
            self._lo = min(self._lo, 0.8)
        parts = [f'<svg viewBox="0 0 {self.w} {self.h}" role="img" aria-label="{html.escape(self.title)}">']
        top, bot, l, r = self.mt, self.h - self.mb, self.ml, self.w - self.mr
        # y grid + ticks
        if self.y_log:
            ticks = [10 ** e for e in range(int(math.log10(self._lo)), int(math.log10(self._hi)) + 1)]
        else:
            hi = self.y_max if self.y_max is not None else self._hi
            ticks = [hi * i / 4 for i in range(5)]
        for t in ticks:
            yy = self._yscale(t)
            parts.append(f'<line x1="{l}" y1="{yy:.1f}" x2="{r}" y2="{yy:.1f}" stroke="var(--grid)" stroke-width="1"/>')
            label = f"{t:.0f}" if t >= 100 else (f"{t:.2f}".rstrip("0").rstrip("."))
            parts.append(f'<text x="{l-6}" y="{yy+3.5:.1f}" text-anchor="end" class="tick">{label}</text>')
        # x ticks
        if logx:
            xt = [10 ** e for e in range(int(math.log10(xmin)), int(math.log10(xmax)) + 1)]
        else:
            xt = sorted(set(all_x))
        for t in xt:
            xx = self._xscale(t, xmin, xmax, logx)
            parts.append(f'<line x1="{xx:.1f}" y1="{bot}" x2="{xx:.1f}" y2="{bot+4}" stroke="var(--baseline)"/>')
            parts.append(f'<text x="{xx:.1f}" y="{bot+18}" text-anchor="middle" class="tick">{t:g}</text>')
        # axes
        parts.append(f'<line x1="{l}" y1="{top}" x2="{l}" y2="{bot}" stroke="var(--baseline)"/>')
        parts.append(f'<line x1="{l}" y1="{bot}" x2="{r}" y2="{bot}" stroke="var(--baseline)"/>')
        # reference lines
        for y, lab, col in self.refs:
            yy = self._yscale(y)
            parts.append(f'<line x1="{l}" y1="{yy:.1f}" x2="{r}" y2="{yy:.1f}" '
                         f'stroke="{col}" stroke-width="1.2" stroke-dasharray="5 4"/>')
            parts.append(f'<text x="{r}" y="{yy-4:.1f}" text-anchor="end" class="reflabel">{html.escape(lab)}</text>')
        # series
        for name, pts, color in self.series:
            d = " ".join(
                ("M" if i == 0 else "L") +
                f"{self._xscale(x, xmin, xmax, logx):.1f},{self._yscale(y):.1f}"
                for i, (x, y) in enumerate(pts))
            parts.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="2"/>')
            for x, y in pts:
                xx = self._xscale(x, xmin, xmax, logx)
                yy = self._yscale(y)
                tip = f"{name}: {self.x_label}={x:g}, {self.y_label}={y:g}"
                parts.append(f'<circle cx="{xx:.1f}" cy="{yy:.1f}" r="4.5" fill="{color}" '
                             f'stroke="var(--surface)" stroke-width="2" class="pt" data-tip="{html.escape(tip)}"/>')
        parts.append(f'<text x="{self.w/2}" y="22" text-anchor="middle" class="title">{html.escape(self.title)}</text>')
        parts.append(f'<text x="{self.w/2}" y="{self.h-6}" text-anchor="middle" class="axislabel">{html.escape(self.x_label)}</text>')
        parts.append(f'<text transform="translate(14,{self.h/2}) rotate(-90)" text-anchor="middle" class="axislabel">{html.escape(self.y_label)}</text>')
        parts.append("</svg>")
        return "".join(parts)


def legend(items):
    return ('<div class="legend">' + "".join(
        f'<span class="lg"><span class="sw" style="background:{c}"></span>{html.escape(n)}</span>'
        for n, c in items) + "</div>")


def table(headers, rows):
    th = "".join(f"<th>{html.escape(h)}</th>" for h in headers)
    trs = []
    for r in rows:
        trs.append("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in r) + "</tr>")
    return f'<table><thead><tr>{th}</tr></thead><tbody>{"".join(trs)}</tbody></table>'


def main():
    r_plain = group(bench_rows("random_plain"), "size")
    r_r40 = group(bench_rows("random40_strict"), "size")
    r_stair = group(bench_rows("staircase_strict"), "size")
    sw_r40 = group(sweep_rows("random40_strict"), "size")
    sw_st2 = group(sweep_rows("stair2_strict"), "size")
    sizes = ["32x24", "128x96", "512x384"]
    x_by_size = {s: i + 1 for i, s in enumerate(sizes)}

    # ---- C1: reduction across sizes, two scenarios ----
    def reduction_panel(title, data):
        ch = LineChart(title, "地图档位（小→大）", "点数减少比例 r", y_max=1.0)
        for ver, col in [("A", BLUE), ("B", ORANGE)]:
            pts = [(x_by_size[s], float([d for d in data[s] if d["version"] == ver][0]["reduction"]))
                   for s in sizes if s in data]
            ch.add_series(f"{ver} 版", pts, col)
        ch.add_refline(0.5, "r = 0.5 取舍线")
        return ch.render() + legend([("A 版（正向试探）", BLUE), ("B 版（反向试探）", ORANGE)])

    c1a = reduction_panel("随机障碍 40% + 禁止切角 LOS", r_r40)
    c1b = reduction_panel("阶梯廊道（近场视线被墙封死）", r_stair)

    # ---- C2: probe counts, staircase, log scale ----
    ch2 = LineChart("试探次数 P 随路径点数 n 增长（阶梯廊道）", "原始路径点数 n", "LOS 试探次数 P（对数）",
                    width=620, y_log=True)
    for ver, col in [("A", BLUE), ("B", ORANGE)]:
        pts = [(int(r_stair[s][0]["raw_n"]),
                int([d for d in r_stair[s] if d["version"] == ver][0]["los_calls"]))
               for s in sizes]
        ch2.add_series(f"{ver} 版", pts, col)
    # identity line P_A = n + m_A - 3 (exact; ~ 2n when A degenerates)
    ch2.add_series("恒等式 P_A = n+m−3",
                   [(int(r_stair[s][0]["raw_n"]),
                     int(r_stair[s][0]["raw_n"]) +
                     int([d for d in r_stair[s] if d["version"] == "A"][0]["out_m"]) - 3)
                    for s in sizes], AQUA)
    c2 = ch2.render() + legend([("A 版", BLUE), ("B 版", ORANGE), ("P_A 恒等线", AQUA)])

    # ---- C3: horizon sweep random40 512: r(S) and probes(S) ----
    big = sw_r40["512x384"]
    ch3a = LineChart("压缩率 vs 试探起点 S（随机40%，512×384）", "试探起点 S（格，对数）", "点数减少比例 r", y_max=0.75)
    ch3a.add_series("窗口化 B", [(int(d["horizon_S"]), float(d["reduction"])) for d in big], ORANGE)
    ch3a.add_refline(0.7018, "全量 B 饱和值 0.702")
    c3a = ch3a.render()
    ch3b = LineChart("试探成本 vs 试探起点 S（同图）", "试探起点 S（格，对数）", "LOS 试探次数（对数）", y_log=True)
    ch3b.add_series("窗口化 B", [(int(d["horizon_S"]), int(d["los_calls"])) for d in big], BLUE)
    c3b = ch3b.render()

    # ---- C4: stair2 32: r(S) + clearance(S) ----
    st2 = sw_st2["32x24"]
    ch4a = LineChart("压缩率 vs S（开阔带楼梯，32×24）", "试探起点 S（格，对数）", "点数减少比例 r", y_max=1.0)
    ch4a.add_series("窗口化 B", [(int(d["horizon_S"]), float(d["reduction"])) for d in st2], ORANGE)
    c4a = ch4a.render()
    ch4b = LineChart("离障碍最小距离 vs S（同图）", "试探起点 S（格，对数）", "路径最小净距（格）", y_max=2.0)
    ch4b.add_series("最小净距", [(int(d["horizon_S"]), float(d["min_clearance"])) for d in st2], AQUA)
    c4b = ch4b.render()

    # ---- tables ----
    def bench_table(data, label):
        rows = []
        for s in sizes:
            if s not in data:
                continue
            n = data[s][0]["raw_n"]
            a = [d for d in data[s] if d["version"] == "A"][0]
            b = [d for d in data[s] if d["version"] == "B"][0]
            rows.append([s, n, a["out_m"], f'{float(a["reduction"]):.3f}', a["los_calls"],
                         b["out_m"], f'{float(b["reduction"]):.3f}', b["los_calls"]])
        return f"<h3>{html.escape(label)}</h3>" + table(
            ["地图", "原始 n", "A 输出 m", "A 减少比例", "A 试探 P", "B 输出 m", "B 减少比例", "B 试探 P"], rows)

    t_plain = bench_table(r_plain, "对照场景：25% 随机障碍，仓库原 LOS（两版都不退化）")
    t_r40 = bench_table(r_r40, "分化场景：40% 随机障碍，禁止切角 LOS")
    t_stair = bench_table(r_stair, "退化场景：阶梯廊道，禁止切角 LOS")

    sw_table_rows = []
    for d in big:
        sw_table_rows.append([d["horizon_S"], d["out_m"], f'{float(d["reduction"]):.4f}',
                              d["los_calls"], d["min_clearance"]])
    t_sweep = "<h3>试探起点扫描明细（随机40%，512×384）</h3>" + table(
        ["S（格）", "输出 m", "减少比例 r", "试探次数", "最小净距"], sw_table_rows)
    st2_rows = [[d["horizon_S"], d["out_m"], f'{float(d["reduction"]):.4f}',
                 d["los_calls"], d["min_clearance"]] for d in st2]
    t_st2 = "<h3>净距权衡扫描明细（开阔带楼梯，32×24）</h3>" + table(
        ["S（格）", "输出 m", "减少比例 r", "试探次数", "最小净距"], st2_rows)

    verdict_rows = []
    for s in sizes:
        if s not in r_r40:
            continue
        a = [d for d in r_r40[s] if d["version"] == "A"][0]
        b = [d for d in r_r40[s] if d["version"] == "B"][0]
        verdict_rows.append([s, f'{float(b["reduction"]):.3f}',
                             "达标" if float(b["reduction"]) > 0.5 else "不达标",
                             f'{float(a["reduction"]):.3f}（对照）',
                             a["los_calls"], b["los_calls"]])
    verdict_table = table(
        ["地图", "B 减少比例（选用）", "是否超过 0.5", "A 减少比例（对照）", "A 试探 P", "B 试探 P"],
        verdict_rows)

    css = """
.viz-root { color-scheme: light; --surface:#fcfcfb; --page:#f9f9f7; --ink:#0b0b0b;
  --ink2:#52514e; --muted:#898781; --grid:#e1e0d9; --baseline:#c3c2b7; }
@media (prefers-color-scheme: dark) {
 .viz-root { color-scheme: dark; --surface:#1a1a19; --page:#0d0d0d; --ink:#f0efec;
  --ink2:#c3c2b7; --muted:#898781; --grid:#2c2c2a; --baseline:#383835; } }
body { background:var(--page); color:var(--ink); font-family:system-ui,-apple-system,"Segoe UI",sans-serif;
  margin:0; padding:32px; line-height:1.6; }
.wrap { max-width:1080px; margin:0 auto; }
h1 { font-size:26px; margin:0 0 6px; } h2 { font-size:20px; margin:34px 0 10px; border-bottom:1px solid var(--grid); padding-bottom:6px; }
h3 { font-size:15px; margin:18px 0 6px; color:var(--ink2); }
p, li { font-size:14.5px; } code { background:var(--grid); padding:1px 5px; border-radius:4px; font-size:13px; }
.grid2 { display:grid; grid-template-columns:1fr 1fr; gap:18px; }
.card { background:var(--surface); border:1px solid var(--grid); border-radius:10px; padding:14px; }
svg { width:100%; height:auto; display:block; }
.tick { font-size:11px; fill:var(--muted); }
.axislabel { font-size:12px; fill:var(--ink2); }
.title { font-size:13.5px; font-weight:600; fill:var(--ink); }
.reflabel { font-size:10.5px; fill:var(--muted); }
.legend { display:flex; gap:16px; justify-content:center; margin-top:6px; font-size:12.5px; color:var(--ink2); }
.lg { display:flex; align-items:center; gap:6px; } .sw { width:12px; height:12px; border-radius:3px; display:inline-block; }
.pt { cursor:pointer; }
table { border-collapse:collapse; width:100%; font-size:12.5px; margin:8px 0 18px; }
th, td { border:1px solid var(--grid); padding:5px 8px; text-align:right; font-variant-numeric:tabular-nums; }
th { color:var(--ink2); background:var(--page); } td:first-child, th:first-child { text-align:left; }
details { margin:10px 0; } summary { cursor:pointer; font-size:13.5px; color:var(--ink2); }
.formula { background:var(--surface); border:1px solid var(--grid); border-radius:8px; padding:10px 14px; font-size:14px; overflow-x:auto; }
#tip { position:fixed; pointer-events:none; background:var(--ink); color:var(--surface); padding:5px 9px;
  border-radius:6px; font-size:12px; display:none; z-index:9; }
.verdict { border-left:4px solid #eb6834; padding:8px 14px; background:var(--surface); border-radius:0 8px 8px 0; }
@media (max-width:860px){ .grid2{grid-template-columns:1fr;} }
"""
    js = """
const tip = document.getElementById('tip');
document.querySelectorAll('.pt').forEach(el => {
  el.addEventListener('mouseenter', e => { tip.textContent = el.dataset.tip; tip.style.display='block'; });
  el.addEventListener('mousemove', e => { tip.style.left=(e.clientX+12)+'px'; tip.style.top=(e.clientY+14)+'px'; });
  el.addEventListener('mouseleave', () => tip.style.display='none');
});
"""
    html_doc = f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>路径平滑 A/B 两版对照实验</title><style>{css}</style></head>
<body><div class="viz-root wrap">
<h1>贪心路径平滑：正向试探（A）与反向试探（B）对照</h1>
<p style="color:var(--ink2)">数据由 <code>smooth/bench/bench.c</code> 在三档地图（32×24 / 128×96 / 512×384）实测生成，LOS 调用有计数，净距由多源 Dijkstra 距离场计算。</p>

<h2>1. 结论：B 版正确，仓库现有实现就是 B</h2>
<div class="verdict">
<p><b>B 版（从尾往回找、找到即停）是正确的贪心平滑。</b>它在每个锚点取<em>最远的可见后继</em>，输出段数达到该贪心策略的最小上界；A 版在第一个 LOS 失败处即停，取的是<em>最近的失败前继</em>，只保证不犯错、不保证压缩。</p>
<p>小地图上两版结果相同，是因为路径短、锚点的可见范围在小图里常常一直延伸到终点，两种停止方式落到同一个后继上。地图变大后可见性不再嵌套（近场被墙角挡住、远点反而可贯穿），两版就此分道扬镳。</p>
</div>

<h3>选用裁决（三档实测，阈值 r &gt; 0.5）</h3>
<p>正式选用场景取 40% 随机障碍 + 禁止切角 LOS（贴近真实导航网格的贴墙几何）。B 版三档减少比例全部超过一半；A 版实际比例作为对照一并列出：</p>
{verdict_table}
<p>在该场景的 32×24 小图上两版完全相同（r 都为 0.698），与「小地图结果一样」的观察吻合；分化从 128×96 开始，512×384 上 B 比 A 多压缩 5.4 个百分点，而 A 的试探成本仅为 B 的 0.6%（2024 对 321592 次 LOS）。阶梯廊道场景进一步展示 A 的退化极限：r<sub>A</sub> 随图增大跌到 0.166（&lt; 0.5，故该场景下 A 不合格），用于解释机理而非作为选型依据。</p>

<h2>2. 时间复杂度表达式</h2>
<p>设原始路径 n 个点，平滑输出 m 个点，单次 LOS 代价 O(L)（L 为线段覆盖格数）。</p>
<div class="formula">
A 版：每个锚点从 i+1 顺序探到第一个失败点，失败即退。所有锚点的试探区间互不重叠且覆盖索引 1..n−1，<br>
&nbsp;&nbsp;LOS 调用恒等式 &nbsp;<b>P<sub>A</sub> = n + m − 3</b>（实测三档全部精确成立，见下表对照），<br>
&nbsp;&nbsp;最坏 m→n 时 <b>T<sub>A</sub>(n) = O(n·L) = O(n²)</b>，但常数项小：每次失败只付 1 次 LOS，总计 ≈ 2n 次。<br><br>
B 版：每个锚点从 n−1 回扫到第一个可见点，找到即停。单个锚点最坏扫 O(n) 个候选，共 m−1 个锚点，<br>
&nbsp;&nbsp;<b>T<sub>B</sub>(n) = O(m·n·L) = O(n²)</b>，最坏 P<sub>B</sub> 达 n(n−1)/2 量级；常数项远大于 A——大图实测 32 万次对 2 千次。<br><br>
两版最坏都是平方级，<b>差别在常数项与停止位置的语义</b>：A 为「廉价但保守」，B 为「昂贵但激进」。
</div>

<h2>3. 用减少比例反解试探次数：A 为何在大图上趋近于零压缩</h2>
<p>点数减少比例定义为 <code>r = (n − m)/n</code>，反解输出点数 <code>m = n(1 − r)</code>，代入 A 的恒等式：</p>
<div class="formula">
P<sub>A</sub> = n + m − 3 = n + n(1−r) − 3 = <b>n(2 − r) − 3</b>，即 r = 2 − (P<sub>A</sub>+3)/n。<br>
定义锚点平均跨越长度 reach = (n−1)/(m−1)，则 r = 1 − 1/reach（渐近）。<br>
当近场视线处处失败时 reach → 1，于是 <b>r → 0、m → n、P<sub>A</sub> → 2n</b>：A 退化成原路径，
却仍付出近 2n 次 LOS——平滑名存实亡。
</div>
<p>阶梯廊道实测（禁止切角 LOS，<code>los(i,i+2)</code> 失败率 0.27→0.74→0.83）：</p>
<ul>
<li>32×24：r<sub>A</sub> = 0.588，reach = 2.50；</li>
<li>128×96：r<sub>A</sub> = <b>0.237</b>，reach = 1.31；</li>
<li>512×384：r<sub>A</sub> = <b>0.166</b>，reach = 1.20，P<sub>A</sub> = 1631 ≈ 2n−3（恒等式精确成立）。</li>
</ul>
<p>作为对照，25% 随机障碍 + 仓库原 LOS 的场景里近场失败率低、reach 稳定在 6.3，r<sub>A</sub> 三档始终 ≥ 0.84——A 的退化是<b>几何条件（贴墙弯 + 禁止切角）触发的，不是规模本身</b>；规模只是让触发后的 r 被稀释到趋近于零。</p>

<h2>4. 三档地图实测：压缩率与试探成本</h2>
<div class="grid2"><div class="card">{c1a}</div><div class="card">{c1b}</div></div>
<div class="card" style="margin-top:18px">{c2}</div>

<h2>5. 试探起点的取舍：两条曲线定 S</h2>
<p>B 版从尾找最激进，长跳线段可能贴障碍太近。给反向探测加窗口上限 S（从 <code>min(i+S, n−1)</code> 开始回扫），用两条曲线选 S：</p>
<div class="grid2"><div class="card">{c3a}</div><div class="card">{c3b}</div></div>
<p>随机40% 大图上 r 在 <b>S≈12 即饱和（0.702）</b>，此后 S 再大压缩率零增益、试探次数却线性涨到 32 万；肘部 S=12 只花 4161 次（全量 B 的 1.3%）。</p>
<div class="grid2"><div class="card">{c4a}</div><div class="card">{c4b}</div></div>
<p>开阔带楼梯小图上两曲线交叉给出更明确的选择：全量 B（S=50）虽然 r 最高（0.961），净距却掉到 <b>1.000（贴墙）</b>；取 <b>S=32</b> 时 r=0.941（仅损失 2 个百分点）而净距保持 <b>1.414</b>。更关键的是 S 从 32 增到 48 时输出点数完全不变（r 同为 0.941），净距却从 1.414 跌到 1.000——<em>同样的压缩率，更大的 S 只带来更贴墙的路径</em>。这就是「用点数减少比例与离障碍最小距离两条曲线定试探起点」的实操：在 r 曲线进入平台、且净距曲线尚未跌破安全阈值的区间取最小 S。</p>

<h2>6. 数据表（图表的表格视图）</h2>
{t_plain}
{t_r40}
{t_stair}
<details><summary>展开试探起点扫描明细（随机40% 大图）</summary>{t_sweep}</details>
<details><summary>展开净距权衡扫描明细（开阔带楼梯小图）</summary>{t_st2}</details>

<p style="color:var(--muted);font-size:12.5px;margin-top:28px">
复现：<code>make -C smooth/bench && smooth/bench/bench artifacts &lt;mode&gt; &lt;plain|strict&gt;</code>；
报告：<code>python3 smooth/bench/report.py</code>。模式含 random / random40 / staircase / stair2 等。</p>
</div><div id="tip"></div><script>{js}</script></body></html>"""

    out = os.path.join(ART, "smooth_report.html")
    with open(out, "w", encoding="utf-8") as f:
        f.write(html_doc)
    print("wrote", out)


if __name__ == "__main__":
    main()
