#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 results/curves.csv 与 sweep.csv，生成自包含交互图 chart.html。"""
import csv
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")

with open(os.path.join(RES, "curves.csv"), encoding="utf-8") as f:
    curves = list(csv.DictReader(f))
with open(os.path.join(RES, "sweep.csv"), encoding="utf-8") as f:
    sweep = list(csv.DictReader(f))

data_js = json.dumps({
    "curves": [{k: float(v) for k, v in row.items()} for row in curves],
    "sweep": sweep,
}, ensure_ascii=False)

html = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>网格边长选型：误判率与节点数曲线</title>
<style>
  .viz-root {
    color-scheme: light;
    --surface-1: #fcfcfb;
    --page: #f9f9f7;
    --text-primary: #0b0b0b;
    --text-secondary: #52514e;
    --muted: #898781;
    --gridline: #e1e0d9;
    --baseline: #c3c2b7;
    --border: rgba(11,11,11,0.10);
    --series-1: #2a78d6;   /* 临界缝最坏误判 */
    --series-2: #eb6834;   /* 整体误判-朴素 */
    --series-3: #1baf7a;   /* 整体误判-腐蚀 */
    --node: #6250d6;       /* 节点数比 */
    --limit: #d03b3b;
    --band: rgba(42,120,214,0.08);
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) .viz-root {
      color-scheme: dark;
      --surface-1: #1a1a19;
      --page: #0d0d0d;
      --text-primary: #f0efec;
      --text-secondary: #c3c2b7;
      --muted: #898781;
      --gridline: #2c2c2a;
      --baseline: #383835;
      --border: rgba(255,255,255,0.10);
      --series-1: #3987e5;
      --series-2: #d95926;
      --series-3: #199e70;
      --node: #9085e9;
      --limit: #e66767;
      --band: rgba(57,135,229,0.14);
    }
  }
  :root[data-theme="dark"] .viz-root {
    color-scheme: dark;
    --surface-1: #1a1a19; --page: #0d0d0d;
    --text-primary: #f0efec; --text-secondary: #c3c2b7; --muted: #898781;
    --gridline: #2c2c2a; --baseline: #383835; --border: rgba(255,255,255,0.10);
    --series-1: #3987e5; --series-2: #d95926; --series-3: #199e70;
    --node: #9085e9; --limit: #e66767; --band: rgba(57,135,229,0.14);
  }
  body.viz-root {
    margin: 0; background: var(--page); color: var(--text-primary);
    font: 14px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif;
    padding: 28px 20px 48px;
  }
  .wrap { max-width: 1080px; margin: 0 auto; }
  h1 { font-size: 20px; margin: 0 0 4px; }
  .sub { color: var(--text-secondary); margin: 0 0 20px; font-size: 13px; }
  .theme-btn {
    float: right; border: 1px solid var(--border); background: var(--surface-1);
    color: var(--text-secondary); border-radius: 6px; padding: 4px 10px;
    font: inherit; font-size: 12px; cursor: pointer;
  }
  .panel {
    background: var(--surface-1); border: 1px solid var(--border);
    border-radius: 10px; padding: 16px 12px 8px; margin-bottom: 18px;
  }
  .panel h2 { font-size: 14px; margin: 0 4px 6px; color: var(--text-primary); }
  .panel .note { margin: 0 4px 8px; font-size: 12px; color: var(--muted); }
  svg { width: 100%; height: auto; display: block; }
  .axis { stroke: var(--baseline); stroke-width: 1; }
  .grid { stroke: var(--gridline); stroke-width: 1; }
  .tick { fill: var(--muted); font-size: 11px; }
  .axis-title { fill: var(--text-secondary); font-size: 12px; }
  .series-label { font-size: 12px; font-weight: 600; }
  .limit-line { stroke: var(--limit); stroke-width: 1.5; stroke-dasharray: 6 4; }
  .limit-label { fill: var(--limit); font-size: 11px; font-weight: 600; }
  .ref-line { stroke: var(--baseline); stroke-width: 1.2; stroke-dasharray: 3 3; }
  .crosshair { stroke: var(--muted); stroke-width: 1; stroke-dasharray: 2 3; opacity: 0; }
  .dot { opacity: 0; }
  #tip {
    position: fixed; pointer-events: none; opacity: 0; z-index: 10;
    background: var(--surface-1); border: 1px solid var(--border);
    border-radius: 8px; padding: 8px 10px; font-size: 12px;
    box-shadow: 0 4px 16px rgba(0,0,0,0.18); min-width: 150px;
  }
  #tip .t-title { font-weight: 700; margin-bottom: 4px; }
  #tip .t-row { display: flex; justify-content: space-between; gap: 12px; }
  #tip .sw { display: inline-block; width: 9px; height: 9px; border-radius: 2px; margin-right: 6px; }
  table { width: 100%; border-collapse: collapse; font-size: 12px; margin-top: 6px; }
  th, td { border-bottom: 1px solid var(--gridline); padding: 6px 8px; text-align: right;
           font-variant-numeric: tabular-nums; }
  th:first-child, td:first-child { text-align: left; }
  th { color: var(--text-secondary); font-weight: 600; }
  .fail { color: var(--limit); font-weight: 700; }
  .ok { color: var(--series-3); font-weight: 700; }
  caption { text-align: left; color: var(--muted); font-size: 12px; margin-bottom: 6px; }
</style>
</head>
<body class="viz-root">
<div class="wrap">
  <button class="theme-btn" id="themeBtn" type="button">切换深色</button>
  <h1>寻路网格边长 h 选型：误判率与节点数两条曲线</h1>
  <p class="sub">基准 r=0.40 m（d=2r=0.80 m）、θ=30°、文档 h=0.25 m；横轴统一为网格边长 h（米）。
     阴影为可行区间 [0.204, 0.400) m。数据见 <code>results/curves.csv</code> / <code>sweep.csv</code>。</p>

  <div class="panel">
    <h2>① 误判率曲线（百分比）</h2>
    <p class="note">蓝：临界缝 W=d 最坏相位假阴性（工程定标口径）；橙：朴素栅格化对缝宽 W~U[0,2d] 的整体误判；
       绿：按半径膨胀障碍后的整体误判；红虚线为 1% 预算。</p>
    <svg id="chart1" viewBox="0 0 980 360" role="img" aria-label="误判率随网格边长变化的三条曲线"></svg>
  </div>

  <div class="panel">
    <h2>② 节点数曲线 N/N₀ = (h₀/h)²（相对文档网格）</h2>
    <p class="note">紫：固定面积下节点数相对文档网格的倍数；红虚线为 +50% 节点预算（1.5）。</p>
    <svg id="chart2" viewBox="0 0 980 300" role="img" aria-label="节点数比随网格边长变化的曲线"></svg>
  </div>

  <div class="panel">
    <h2>③ 三约束各改一档扫描表</h2>
    <table id="sweepTable">
      <caption>worst% = 临界缝 W=d 的最坏相位误判；N/N₀ = 节点数相对基准；两条硬门槛：worst% ≤ 1% 且 N/N₀ ≤ 1.5。</caption>
      <thead><tr>
        <th>档位</th><th>h_eff (m)</th><th>d/h_eff</th><th>W/h&gt;2</th>
        <th>worst%</th><th>N/N₀</th><th>误判门槛</th><th>节点门槛</th><th>总评</th>
      </tr></thead>
      <tbody></tbody>
    </table>
  </div>
</div>
<div id="tip"></div>

<script>
const DATA = __DATA__;
const M = {top: 18, right: 130, bottom: 46, left: 60};
const XMIN = 0.16, XMAX = 0.55;
const css = name => getComputedStyle(document.body).getPropertyValue(name).trim();

function scales(svg) {
  const W = +svg.viewBox.baseVal.width, H = +svg.viewBox.baseVal.height;
  const iw = W - M.left - M.right, ih = H - M.top - M.bottom;
  const x = v => M.left + (v - XMIN) / (XMAX - XMIN) * iw;
  return {W, H, iw, ih, x};
}
function path(points, x, y, key) {
  return points.map((p, i) => (i ? 'L' : 'M') + x(p.h_m).toFixed(2) + ',' + y(p[key]).toFixed(2)).join(' ');
}
function drawAxes(svg, x, y, ymax, ytitle, xtitle) {
  const {H, ih} = scales(svg);
  let s = '';
  for (let gx = 0.20; gx <= 0.55 + 1e-9; gx += 0.05) {
    s += `<line class="grid" x1="${x(gx)}" y1="${M.top}" x2="${x(gx)}" y2="${M.top+ih}"/>`;
    s += `<text class="tick" x="${x(gx)}" y="${H-22}" text-anchor="middle">${gx.toFixed(2)}</text>`;
  }
  const ticks = ymax <= 1.6 ? [0, 0.5, 1.0, 1.5] :
                ymax <= 5 ? [0,1,2,3,4,5] : [0,20,40,60,80];
  for (const t of ticks) {
    const yy = y(t);
    s += `<line class="grid" x1="${M.left}" y1="${yy}" x2="${M.left+scales(svg).iw}" y2="${yy}"/>`;
    s += `<text class="tick" x="${M.left-8}" y="${yy+4}" text-anchor="end">${t}</text>`;
  }
  s += `<line class="axis" x1="${M.left}" y1="${M.top+ih}" x2="${M.left+scales(svg).iw}" y2="${M.top+ih}"/>`;
  s += `<line class="axis" x1="${M.left}" y1="${M.top}" x2="${M.left}" y2="${M.top+ih}"/>`;
  s += `<text class="axis-title" x="${M.left+scales(svg).iw/2}" y="${H-4}" text-anchor="middle">${xtitle}</text>`;
  s += `<text class="axis-title" transform="translate(16,${M.top+ih/2}) rotate(-90)" text-anchor="middle">${ytitle}</text>`;
  return s;
}
function feasibleBand(svg, x, y) {
  const x1 = x(0.2041), x2 = x(0.4), {top} = M, bot = M.top + scales(svg).ih;
  return `<rect x="${x1}" y="${top}" width="${x2-x1}" height="${bot-top}" fill="var(--band)"/>`;
}
function label(svg, x, y, key, text, color, lastPoint) {
  return `<text class="series-label" x="${x(XMAX)+8}" y="${y(lastPoint[key])+4}" fill="${color}">${text}</text>`;
}

function buildChart1() {
  const svg = document.getElementById('chart1');
  const {x} = scales(svg);
  const ymax = 80;
  const y = v => M.top + scales(svg).ih * (1 - v / ymax);
  const c1 = css('--series-1'), c2 = css('--series-2'), c3 = css('--series-3');
  const pts = DATA.curves, last = pts[pts.length-1];
  let s = feasibleBand(svg, x, y);
  s += drawAxes(svg, x, y, ymax, '误判率 (%)', '网格边长 h (m)');
  // 1% budget line
  s += `<line class="limit-line" x1="${M.left}" y1="${y(1)}" x2="${M.left+scales(svg).iw}" y2="${y(1)}"/>`;
  s += `<text class="limit-label" x="${M.left+6}" y="${y(1)-5}">误判预算 1%</text>`;
  // reference h=0.25
  s += `<line class="ref-line" x1="${x(0.25)}" y1="${M.top}" x2="${x(0.25)}" y2="${M.top+scales(svg).ih}"/>`;
  s += `<text class="tick" x="${x(0.25)}" y="${M.top+12}" text-anchor="middle">文档 h=0.25</text>`;
  s += `<polyline fill="none" stroke="${c2}" stroke-width="2" points="${pts.map(p=>x(p.h_m)+','+y(p.ensemble_plain_pct)).join(' ')}"/>`;
  s += `<polyline fill="none" stroke="${c3}" stroke-width="2" points="${pts.map(p=>x(p.h_m)+','+y(p.ensemble_eroded_pct)).join(' ')}"/>`;
  s += `<polyline fill="none" stroke="${c1}" stroke-width="2" points="${pts.map(p=>x(p.h_m)+','+y(p.worst_fn_pct)).join(' ')}"/>`;
  s += label(svg, x, y, 'worst_fn_pct', '临界缝最坏FN', c1, last);
  s += label(svg, x, y, 'ensemble_plain_pct', '整体误判·朴素', c2, last);
  s += label(svg, x, y, 'ensemble_eroded_pct', '整体误判·腐蚀', c3, last);
  s += hoverLayer(svg, x, y, [
    {key:'worst_fn_pct', name:'临界缝最坏FN', color:c1, fmt:v=>v.toFixed(2)+'%'},
    {key:'ensemble_plain_pct', name:'整体误判·朴素', color:c2, fmt:v=>v.toFixed(2)+'%'},
    {key:'ensemble_eroded_pct', name:'整体误判·腐蚀', color:c3, fmt:v=>v.toFixed(2)+'%'},
  ]);
  svg.innerHTML = s;
}

function buildChart2() {
  const svg = document.getElementById('chart2');
  const {x} = scales(svg);
  const ymax = 1.6;
  const y = v => M.top + scales(svg).ih * (1 - v / ymax);
  const cn = css('--node');
  const pts = DATA.curves, last = pts[pts.length-1];
  let s = feasibleBand(svg, x, y);
  s += drawAxes(svg, x, y, ymax, '节点数比 N/N₀', '网格边长 h (m)');
  s += `<line class="limit-line" x1="${M.left}" y1="${y(1.5)}" x2="${M.left+scales(svg).iw}" y2="${y(1.5)}"/>`;
  s += `<text class="limit-label" x="${M.left+6}" y="${y(1.5)-5}">节点预算 1.5（+50%）</text>`;
  s += `<line class="ref-line" x1="${x(0.25)}" y1="${M.top}" x2="${x(0.25)}" y2="${M.top+scales(svg).ih}"/>`;
  s += `<text class="tick" x="${x(0.25)+4}" y="${M.top+scales(svg).ih-6}" text-anchor="start">文档 h=0.25</text>`;
  s += `<polyline fill="none" stroke="${cn}" stroke-width="2" points="${pts.map(p=>x(p.h_m)+','+y(p.node_ratio)).join(' ')}"/>`;
  s += label(svg, x, y, 'node_ratio', '节点数比 N/N₀', cn, last);
  s += hoverLayer(svg, x, y, [
    {key:'node_ratio', name:'节点数比 N/N₀', color:cn, fmt:v=>v.toFixed(3)},
  ]);
  svg.innerHTML = s;
}

function hoverLayer(svg, x, y, series) {
  const {iw, ih} = scales(svg);
  let s = `<line class="crosshair" id="${svg.id}-ch" x1="0" y1="${M.top}" x2="0" y2="${M.top+ih}"/>`;
  for (const ser of series)
    s += `<circle class="dot" id="${svg.id}-dot-${ser.key}" r="4.5" fill="${ser.color}" stroke="var(--surface-1)" stroke-width="2"/>`;
  s += `<rect id="${svg.id}-hit" x="${M.left}" y="${M.top}" width="${iw}" height="${ih}" fill="transparent"/>`;
  setTimeout(() => attachHover(svg, x, y, series), 0);
  return s;
}
function attachHover(svg, x, y, series) {
  const hit = document.getElementById(svg.id + '-hit');
  const ch = document.getElementById(svg.id + '-ch');
  const tip = document.getElementById('tip');
  hit.addEventListener('mousemove', ev => {
    const rect = svg.getBoundingClientRect();
    const sx = (ev.clientX - rect.left) / rect.width * svg.viewBox.baseVal.width;
    const h = XMIN + (sx - M.left) / scales(svg).iw * (XMAX - XMIN);
    if (h < XMIN || h > XMAX) return;
    const p = DATA.curves.reduce((a, b) => Math.abs(b.h_m - h) < Math.abs(a.h_m - h) ? b : a);
    ch.setAttribute('x1', x(p.h_m)); ch.setAttribute('x2', x(p.h_m)); ch.style.opacity = 1;
    let html = `<div class="t-title">h = ${p.h_m.toFixed(2)} m</div>`;
    for (const ser of series) {
      const dot = document.getElementById(svg.id + '-dot-' + ser.key);
      dot.setAttribute('cx', x(p.h_m)); dot.setAttribute('cy', y(p[ser.key])); dot.style.opacity = 1;
      html += `<div class="t-row"><span><span class="sw" style="background:${ser.color}"></span>${ser.name}</span><b>${ser.fmt(p[ser.key])}</b></div>`;
    }
    tip.innerHTML = html;
    tip.style.opacity = 1;
    tip.style.left = Math.min(ev.clientX + 14, window.innerWidth - 190) + 'px';
    tip.style.top = (ev.clientY + 14) + 'px';
  });
  hit.addEventListener('mouseleave', () => {
    ch.style.opacity = 0; tip.style.opacity = 0;
    series.forEach(ser => document.getElementById(svg.id + '-dot-' + ser.key).style.opacity = 0);
  });
}

function buildTable() {
  const tb = document.querySelector('#sweepTable tbody');
  tb.innerHTML = DATA.sweep.map(r => {
    const cell = (ok) => `<td class="${ok ? 'ok' : 'fail'}">${ok ? '通过' : '不通过'}</td>`;
    return `<tr>
      <td>${r.case}</td><td>${r.h_eff_m}</td><td>${r.d_over_h_eff}</td>
      <td>${r['expressible_W/h>2'] === '1' ? '是' : '<span class="fail">否</span>'}</td>
      <td>${(r['worst_misjudge_at_W=d']*100).toFixed(2)}%</td>
      <td>${r.node_ratio_vs_baseline}</td>
      ${cell(+r.pass_misjudge_1pct)}${cell(+r.pass_node_50pct)}
      <td class="${+r.pass_all ? 'ok' : 'fail'}">${+r.pass_all ? '达标' : '超档'}</td></tr>`;
  }).join('');
}

document.getElementById('themeBtn').addEventListener('click', () => {
  const root = document.documentElement;
  const dark = root.dataset.theme !== 'dark';
  root.dataset.theme = dark ? 'dark' : 'light';
  document.getElementById('themeBtn').textContent = dark ? '切换浅色' : '切换深色';
  buildChart1(); buildChart2();
});
if (new URLSearchParams(location.search).get('theme') === 'dark') {
  document.documentElement.dataset.theme = 'dark';
  document.getElementById('themeBtn').textContent = '切换浅色';
}
buildChart1(); buildChart2(); buildTable();
</script>
</body>
</html>
"""

html = html.replace("__DATA__", data_js)
out = os.path.join(HERE, "chart.html")
with open(out, "w", encoding="utf-8") as f:
    f.write(html)
print(f"wrote {out} ({len(html)} bytes)")
