#!/usr/bin/env python3
"""Build a dependency-free interactive dashboard from clearance_report.json."""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ART = ROOT / "artifacts"
report_blob = (ART / "clearance_report.json").read_text(encoding="utf-8")
payload = json.loads(report_blob)

HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>留边距离权衡 · mini-nav-lab</title>
<style>
  :root {
    color-scheme: light;
    --surface: #fcfcfb;
    --page: #f9f9f7;
    --text: #0b0b0b;
    --secondary: #52514e;
    --muted: #898781;
    --grid: #e1e0d9;
    --axis: #c3c2b7;
    --border: rgba(11,11,11,.10);
    --blue: #2a78d6;
    --orange: #eb6834;
    --aqua: #1baf7a;
    --good: #0ca30c;
    --critical: #d03b3b;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      color-scheme: dark;
      --surface: #1a1a19;
      --page: #0d0d0d;
      --text: #f0efec;
      --secondary: #c3c2b7;
      --muted: #898781;
      --grid: #2c2c2a;
      --axis: #383835;
      --border: rgba(255,255,255,.10);
      --blue: #3987e5;
      --orange: #d95926;
      --aqua: #199e70;
      --good: #0ca30c;
      --critical: #e66767;
    }
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 32px 20px 56px; background: var(--page); color: var(--text);
    font: 15px/1.55 system-ui, -apple-system, "Segoe UI", "Noto Sans CJK SC", sans-serif;
  }
  main { max-width: 980px; margin: 0 auto; }
  h1 { font-size: 26px; line-height: 1.25; margin: 0 0 6px; }
  h2 { font-size: 18px; margin: 30px 0 10px; }
  .sub { color: var(--secondary); margin: 0 0 22px; }
  .card {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 14px; padding: 18px; margin: 14px 0;
  }
  .legend { display: flex; flex-wrap: wrap; gap: 16px; margin: 4px 0 12px; color: var(--secondary); font-size: 14px; }
  .legend span { display: inline-flex; align-items: center; gap: 7px; }
  .key { width: 18px; height: 3px; border-radius: 2px; display: inline-block; }
  .key.easy { background: var(--blue); }
  .key.medium { background: var(--orange); }
  .key.hard { background: var(--aqua); }
  .chart-wrap { position: relative; margin: 8px 0 18px; }
  svg { width: 100%; height: auto; display: block; }
  .grid-line { stroke: var(--grid); stroke-width: 1; shape-rendering: crispEdges; }
  .axis-line { stroke: var(--axis); stroke-width: 1; shape-rendering: crispEdges; }
  text.tick, .axis-label { fill: var(--muted); font-size: 12px; }
  .panel-title { fill: var(--text); font-size: 14px; font-weight: 650; }
  .series-line { fill: none; stroke-width: 2; stroke-linecap: round; stroke-linejoin: round; }
  .series-line.easy { stroke: var(--blue); }
  .series-line.medium { stroke: var(--orange); }
  .series-line.hard { stroke: var(--aqua); }
  .hover-dot { stroke: var(--surface); stroke-width: 2; r: 5; }
  .hover-dot.easy { fill: var(--blue); }
  .hover-dot.medium { fill: var(--orange); }
  .hover-dot.hard { fill: var(--aqua); }
  .hover-rule { stroke: var(--text); stroke-opacity: .35; stroke-width: 1; }
  .selected-rule { stroke: var(--text); stroke-width: 1.5; stroke-dasharray: 6 5; stroke-opacity: .72; }
  .overlay { fill: transparent; cursor: crosshair; }
  .tooltip {
    position: absolute; pointer-events: none; z-index: 2; min-width: 178px;
    background: var(--surface); border: 1px solid var(--border); border-radius: 10px;
    padding: 9px 11px; box-shadow: 0 8px 24px rgba(0,0,0,.14);
    font-size: 13px; display: none;
  }
  .tooltip .tt-title { font-weight: 650; margin-bottom: 5px; }
  .tt-row { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
  .tt-name { display: inline-flex; align-items: center; gap: 6px; color: var(--secondary); }
  .dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
  .dot.easy { background: var(--blue); } .dot.medium { background: var(--orange); } .dot.hard { background: var(--aqua); }
  table { border-collapse: collapse; width: 100%; font-size: 14px; }
  th, td { padding: 9px 10px; border-bottom: 1px solid var(--grid); text-align: right; }
  th:first-child, td:first-child { text-align: left; }
  th { color: var(--secondary); font-weight: 650; }
  td { font-variant-numeric: tabular-nums; }
  .pass { color: var(--good); font-weight: 700; }
  .fail { color: var(--critical); font-weight: 700; }
  .note { color: var(--secondary); font-size: 14px; }
  code { background: color-mix(in srgb, var(--muted) 16%, transparent); padding: 1px 5px; border-radius: 5px; }
  details { margin-top: 14px; }
  summary { cursor: pointer; color: var(--secondary); }
  .table-scroll { max-height: 420px; overflow: auto; border: 1px solid var(--border); border-radius: 10px; margin-top: 10px; }
</style>
</head>
<body>
<main>
  <h1>留边距离 b：卡墙、绕远与视线遮挡的交集</h1>
  <p class="sub">网格步长 0.10 m；前视距离 2.70 m；接触阈值 0.30 m。绿色横带为目标区间，蓝色竖带为三档共同可行的离散参数区间 <strong>0.40–0.50 m</strong>，黑色虚线为确认运行取值 <strong>0.45 m</strong>。鼠标/指针移动到图上查看数值。</p>

  <section class="card">
    <div class="legend">
      <span><i class="key easy"></i>一档 · 单墙转角</span>
      <span><i class="key medium"></i>二档 · 错位门缝</span>
      <span><i class="key hard"></i>三档 · 交错双墙</span>
    </div>
    <div id="charts"></div>
  </section>

  <section class="card">
    <h2 style="margin-top:0">b = 0.45 m 的三档确认结果</h2>
    <div style="overflow-x:auto"><table id="final-table"></table></div>
    <p id="boundary" class="note"></p>
  </section>

  <section class="card">
    <h2 style="margin-top:0">判据</h2>
    <p class="note">目标区间：最小净距 <code>[0.35, 0.90] m</code>；绕远比 <code>[1.00, 1.35]</code>；遮挡段占比 <code>[0, 0.20]</code>；卡墙簇数 <code>0</code>。完整 5 cm 扫描数据也已写入 <code>artifacts/clearance_sweep.csv</code>。</p>
    <details>
      <summary>展开完整扫描表（可访问数据表）</summary>
      <div class="table-scroll"><table id="sweep-table"></table></div>
    </details>
  </section>
</main>
<script>
const DATA = __DATA__;
const REPORT = __REPORT__;
const SCENARIOS = [
  {key:'easy', name:'一档 单墙转角'},
  {key:'medium', name:'二档 错位门缝'},
  {key:'hard', name:'三档 交错双墙'}
];
const PANELS = [
  {key:'wall_events', label:'卡墙次数 Nw（次）', min:0, max:4.25, ticks:[0,1,2,3,4], fmt:v=>String(Math.round(v))},
  {key:'detour_ratio', label:'绕远比 ρ = L / D', min:1.0, max:1.35, ticks:[1,1.05,1.10,1.15,1.20,1.25,1.30,1.35], fmt:v=>v.toFixed(3), target:REPORT.constants.detour_ratio_target},
  {key:'occluded_fraction', label:'遮挡段占比 η', min:0, max:0.50, ticks:[0,.1,.2,.3,.4,.5], fmt:v=>(100*v).toFixed(1)+'%', target:REPORT.constants.occluded_fraction_target}
];
const W=900,H=280,M={top:24,right:24,bottom:42,left:58};
const PW=W-M.left-M.right, PH=H-M.top-M.bottom;
const XMIN=0,XMAX=.8;
const x = b => M.left + (b-XMIN)/(XMAX-XMIN)*PW;
const y = (v,p) => M.top + PH - (v-p.min)/(p.max-p.min)*PH;
const rowAt = (key,b) => DATA.find(r => r.scenario===key && Math.abs(r.margin-b)<1e-9);
const margins = [...new Set(DATA.map(r=>r.margin))].sort((a,b)=>a-b);

function svgEl(tag, attrs={}) {
  const n=document.createElementNS('http://www.w3.org/2000/svg',tag);
  for (const [k,v] of Object.entries(attrs)) n.setAttribute(k,v);
  return n;
}

function makePanel(panel, index) {
  const wrap=document.createElement('div'); wrap.className='chart-wrap';
  const svg=svgEl('svg',{viewBox:`0 0 ${W} ${H}`,role:'img','aria-label':panel.label+' 随留边距离变化'});
  const tooltip=document.createElement('div'); tooltip.className='tooltip';
  wrap.append(svg,tooltip);

  // Target band and common feasible interval.
  if (panel.target) {
    const [lo,hi]=panel.target;
    svg.append(svgEl('rect',{x:M.left,y:y(hi,panel),width:PW,height:y(lo,panel)-y(hi,panel),fill:'var(--good)','fill-opacity':'.10'}));
    svg.append(svgEl('line',{x1:M.left,x2:M.left+PW,y1:y(hi,panel),y2:y(hi,panel),stroke:'var(--muted)','stroke-width':1,'stroke-dasharray':'2 3'}));
  }
  const fr=REPORT.common_feasible_range_grid;
  if (fr) svg.append(svgEl('rect',{x:x(fr[0]),y:M.top,width:x(fr[1])-x(fr[0]),height:PH,fill:'var(--blue)','fill-opacity':'.08'}));

  // Grid, ticks, labels.
  for (const tv of panel.ticks) {
    const yy=y(tv,panel);
    svg.append(svgEl('line',{class:'grid-line',x1:M.left,x2:M.left+PW,y1:yy,y2:yy}));
    const t=svgEl('text',{x:M.left-9,y:yy+4,'text-anchor':'end'}); t.textContent=panel.fmt(tv); t.setAttribute('class','tick');
    svg.append(t);
  }
  for (let b=0;b<=.8001;b+=.1) {
    const xx=x(b);
    svg.append(svgEl('line',{class:'grid-line',x1:xx,x2:xx,y1:M.top,y2:M.top+PH}));
    const t=svgEl('text',{x:xx,y:M.top+PH+20,'text-anchor':'middle'}); t.textContent=b.toFixed(1); t.setAttribute('class','tick'); svg.append(t);
  }
  svg.append(svgEl('line',{class:'axis-line',x1:M.left,x2:M.left+PW,y1:M.top+PH,y2:M.top+PH}));
  svg.append(svgEl('line',{class:'axis-line',x1:M.left,x2:M.left,y1:M.top,y2:M.top+PH}));
  const title=svgEl('text',{class:'panel-title',x:M.left,y:16}); title.textContent=panel.label; svg.append(title);
  if (index===PANELS.length-1) {
    const xl=svgEl('text',{class:'axis-label',x:M.left+PW/2,y:H-4,'text-anchor':'middle'}); xl.textContent='留边距离 b (m)'; svg.append(xl);
  }

  // Series lines.
  for (const sc of SCENARIOS) {
    const pts=DATA.filter(r=>r.scenario===sc.key && r.reachable && r[panel.key] !== undefined && r[panel.key] !== null)
      .sort((a,b)=>a.margin-b.margin)
      .map(r=>[x(r.margin), y(r[panel.key],panel)]);
    const d=pts.map((p,i)=>(i?'L':'M')+p[0].toFixed(1)+' '+p[1].toFixed(1)).join(' ');
    svg.append(svgEl('path',{class:`series-line ${sc.key}`,d}));
  }
  const sx=x(REPORT.constants.selected_margin_m);
  svg.append(svgEl('line',{class:'selected-rule',x1:sx,x2:sx,y1:M.top,y2:M.top+PH}));

  const hoverRule=svgEl('line',{class:'hover-rule',y1:M.top,y2:M.top+PH,visibility:'hidden'});
  svg.append(hoverRule);
  const dots={};
  for (const sc of SCENARIOS) {
    dots[sc.key]=svgEl('circle',{class:`hover-dot ${sc.key}`,visibility:'hidden'});
    svg.append(dots[sc.key]);
  }
  const overlay=svgEl('rect',{class:'overlay',x:M.left,y:M.top,width:PW,height:PH});
  svg.append(overlay);

  overlay.addEventListener('pointermove', ev => {
    const rect=svg.getBoundingClientRect();
    const mx=(ev.clientX-rect.left)/rect.width*W;
    let b=Math.round(((mx-M.left)/PW*(XMAX-XMIN)+XMIN)/.05)*.05;
    b=Math.max(0,Math.min(.8,b));
    const xx=x(b);
    hoverRule.setAttribute('x1',xx); hoverRule.setAttribute('x2',xx); hoverRule.setAttribute('visibility','visible');
    let html=`<div class="tt-title">b = ${b.toFixed(2)} m</div>`;
    for (const sc of SCENARIOS) {
      const r=rowAt(sc.key,b);
      const dot=dots[sc.key];
      if (r && r.reachable && r[panel.key] !== undefined) {
        const yy=y(r[panel.key],panel);
        dot.setAttribute('cx',xx); dot.setAttribute('cy',yy); dot.setAttribute('visibility','visible');
        const val=panel.key==='wall_events' ? `${r[panel.key]} 次` : panel.fmt(r[panel.key]);
        html += `<div class="tt-row"><span class="tt-name"><i class="dot ${sc.key}"></i>${sc.name}</span><strong>${val}</strong></div>`;
      } else {
        dot.setAttribute('visibility','hidden');
        html += `<div class="tt-row"><span class="tt-name"><i class="dot ${sc.key}"></i>${sc.name}</span><strong>不可达</strong></div>`;
      }
    }
    tooltip.innerHTML=html;
    tooltip.style.display='block';
    const wrapRect=wrap.getBoundingClientRect();
    let left=(ev.clientX-wrapRect.left)+14;
    if (left+200>wrapRect.width) left=(ev.clientX-wrapRect.left)-190;
    tooltip.style.left=left+'px';
    tooltip.style.top=((ev.clientY-wrapRect.top)+12)+'px';
  });
  overlay.addEventListener('pointerleave', () => {
    tooltip.style.display='none'; hoverRule.setAttribute('visibility','hidden');
    Object.values(dots).forEach(d=>d.setAttribute('visibility','hidden'));
  });
  return wrap;
}

const charts=document.getElementById('charts');
PANELS.forEach((p,i)=>charts.append(makePanel(p,i)));

function table(elId, columns, rows) {
  const t=document.getElementById(elId);
  t.innerHTML='<thead><tr>'+columns.map(c=>`<th>${c.head}</th>`).join('')+'</tr></thead><tbody>'+
    rows.map(r=>'<tr>'+columns.map(c=>`<td>${c.get(r)}</td>`).join('')+'</tr>').join('')+'</tbody>';
}
table('final-table', [
  {head:'场景', get:r=>r.name},
  {head:'d_min (m)', get:r=>r.min_clearance_m.toFixed(3)},
  {head:'ρ', get:r=>r.detour_ratio.toFixed(3)},
  {head:'η', get:r=>(100*r.occluded_fraction).toFixed(1)+'%'},
  {head:'N_w', get:r=>r.wall_events},
  {head:'结论', get:r=>r.all_pass?'<span class="pass">✓ 全部达标</span>':'<span class="fail">✗ 有越界</span>'}
], REPORT.final_runs);

const bc=REPORT.boundary_checks.map(x=>`b=${x.margin.toFixed(2)} m，${x.name}：`+x.violations.map(v=>v.direction).join('；')).join('<br>');
document.getElementById('boundary').innerHTML='边界反例：<br>'+bc;

const sweepRows=DATA.filter(r=>r.reachable);
table('sweep-table', [
  {head:'b', get:r=>r.margin.toFixed(2)},
  {head:'场景', get:r=>r.name || r.scenario},
  {head:'d_min', get:r=>r.min_clearance_m.toFixed(3)},
  {head:'ρ', get:r=>r.detour_ratio.toFixed(3)},
  {head:'η', get:r=>(100*r.occluded_fraction).toFixed(1)+'%'},
  {head:'N_w', get:r=>r.wall_events}
], sweepRows);
</script>
</body>
</html>
"""

out = ART / "clearance_dashboard.html"
out.write_text(HTML.replace("__DATA__", json.dumps(payload["sweep"], ensure_ascii=False))
                   .replace("__REPORT__", json.dumps(payload["report"], ensure_ascii=False)),
               encoding="utf-8")
print(out)
