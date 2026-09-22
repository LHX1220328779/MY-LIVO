/* LIVO Studio v2 front-end (vanilla JS, no build step). Talks to tools/studio/server.py. */
"use strict";

const S = {
  page: "home", state: null, system: null, vehicles: [], stages: {}, runs: [], jobs: [], dataset: null,
  cal: null, versionDetail: null, runDetail: null, live: null, job: null, jobLog: [], jobLogNext: 0,
  fs: null, drawer: null, fsTarget: null, busy: false, calibForm: { stage: "ground" }, timers: {},
};
const $ = (sel, root = document) => root.querySelector(sel);
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const PAGES = { home: "工作台", data: "数据源", calib: "标定", map: "建图", results: "结果", system: "系统" };
const PHASES = [["starting", "启动"], ["playing", "回放"], ["draining", "收尾"], ["finalizing", "优化导出"], ["exported", "已导出"], ["evaluating", "评估"], ["done", "完成"]];

// ------------------------------------------------------------------ plumbing
async function api(path, method = "GET", body) {
  const r = await fetch(path, { method, headers: body ? { "Content-Type": "application/json" } : {}, body: body ? JSON.stringify(body) : undefined });
  let doc = null;
  try { doc = await r.json(); } catch (e) { doc = { error: `HTTP ${r.status}` }; }
  if (!r.ok) throw new Error(doc.error || `HTTP ${r.status}`);
  return doc;
}
function toast(msg, kind = "") {
  const el = document.createElement("div");
  el.className = `toast ${kind}`; el.textContent = msg;
  $("#toasts").appendChild(el);
  setTimeout(() => el.remove(), kind === "bad" ? 7000 : 3500);
}
async function act(fn, okMsg) {
  if (S.busy) return;
  S.busy = true;
  try { const r = await fn(); if (okMsg) toast(okMsg, "ok"); return r; }
  catch (e) { toast(e.message || String(e), "bad"); }
  finally { S.busy = false; }
}
const fmtT = (iso) => iso ? String(iso).replace("T", " ").slice(0, 19) : "—";
const fmtDur = (s) => s == null ? "—" : s >= 3600 ? `${(s / 3600).toFixed(1)} h` : s >= 60 ? `${Math.floor(s / 60)} 分 ${Math.round(s % 60)} 秒` : `${Number(s).toFixed(1)} 秒`;
const fmtGB = (b) => b == null ? "—" : `${(b / 1e9).toFixed(2)} GB`;
const num = (v, d = 3) => (v == null || v === "" || Number.isNaN(Number(v))) ? "—" : Number(v).toFixed(d);
const cm = (m) => m == null ? "—" : `${(Number(m) * 100).toFixed(1)} cm`;
const fm = (v) => v == null ? "—" : Math.abs(Number(v)) < 1 ? `${(Number(v) * 100).toFixed(1)} cm` : `${Number(v).toFixed(2)} m`;
const f1 = (v) => v == null ? "—" : Number(v).toFixed(1);
const f2 = (v) => v == null ? "—" : Number(v).toFixed(2);
const pct = (v) => v == null ? "—" : `${Number(v).toFixed(1)}%`;
const sdeg = (v) => v == null ? "—" : `${Number(v) > 0 ? "+" : ""}${Number(v).toFixed(2)}°`;
const secondsBetween = (a, b) => (a && b) ? Math.max(0, (new Date(b) - new Date(a)) / 1000) : null;
const baseName = (p) => String(p || "").replace(/\/+$/, "").split("/").pop();
function debounce(fn, ms) { let t; return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); }; }
const saveState = debounce((patch) => api("/api/state", "POST", patch).then((s) => { S.state = s; }).catch((e) => toast(e.message, "bad")), 400);

// ------------------------------------------------------------------ data loading
async function loadOverview() {
  const o = await api("/api/overview");
  Object.assign(S, { state: o.state, system: o.system, vehicles: o.vehicles, stages: o.stages, runs: o.runs, jobs: o.jobs, dataset: o.dataset });
  const active = o.system.active_job;
  if (active && (!S.job || S.job.id !== active.id)) { S.job = active; S.jobLog = []; S.jobLogNext = 0; }
  if (!active && S.job && S.job.status === "running") { S.job = null; }
  if (active) S.job = { ...S.job, ...active };
}
async function loadCatalogue(refresh = false) { S.cal = await api(`/api/calibrations${refresh ? "?refresh=1" : ""}`); S.stages = S.cal.stages; }
async function loadRuns() { S.runs = await api("/api/runs?limit=150"); }
async function loadJobs() { S.jobs = await api("/api/jobs?limit=80"); }
async function pollJob() {
  if (!S.job) return;
  const doc = await api(`/api/jobs/${S.job.id}?offset=${S.jobLogNext}`);
  const log = doc.log; delete doc.log;
  if (log.offset !== S.jobLogNext) { S.jobLog = []; }
  S.jobLog.push(...log.lines); S.jobLogNext = log.next;
  if (S.jobLog.length > 4000) S.jobLog.splice(0, S.jobLog.length - 4000);
  const finished = S.job.status === "running" && doc.status !== "running";
  S.job = doc;
  if (finished) {
    toast(doc.status === "done" ? `✓ ${doc.title} 完成` : `${doc.title}：${doc.error || doc.status}`, doc.status === "done" ? "ok" : "bad");
    await Promise.all([loadOverview(), loadCatalogue(true), loadRuns()]);
    render();
  }
}
function currentRound() {
  if (S.job && S.job.status === "running") {
    if (S.job.kind === "mapping") return S.job.meta.round;
    if (S.job.kind === "pipeline") return S.job.results.round || (((S.job.steps[S.job.current] || {}).title || "").match(/calib_[A-Za-z0-9_]+/) || [])[0] || null;
  }
  return S.runDetail?.round || null;
}
async function pollLive() {
  const round = currentRound();
  if (!round) { S.live = null; return; }
  S.live = await api(`/api/runs/${round}/live`);
}
function schedule() {
  clearInterval(S.timers.main);
  S.timers.main = setInterval(async () => {
    try {
      const running = S.job && S.job.status === "running";
      if (running) { await pollJob(); await pollLive(); patchLive(); }
      else if (S.tick++ % 3 === 0) { await loadOverview(); if (S.page === "results" && S.runDetail) await pollLive(); patchLive(); }
    } catch (e) { /* transient */ }
  }, 2000);
  S.tick = 0;
}
function patchLive() {
  // cheap in-place refresh of live widgets without re-rendering forms
  const pill = $("#jobpill"); if (pill) { pill.outerHTML = jobPill(); }
  const mon = $("#live-monitor"); if (mon) mon.outerHTML = liveMonitor();
  const jl = $("#job-log"); if (jl) { jl.innerHTML = jobLogHtml(); jl.scrollTop = jl.scrollHeight; }
  const js = $("#job-steps"); if (js) js.innerHTML = jobStepsHtml();
  const foot = $("#sidebar-foot"); if (foot) foot.innerHTML = footHtml();
}

// ------------------------------------------------------------------ shared components
function vehicle() { const v = S.state?.vehicle || {}; return S.vehicles.find((x) => x.mine === v.mine && x.truck === v.truck) || S.vehicles[0] || null; }
function jobPill() {
  const j = S.job;
  if (j && j.status === "running") {
    const step = j.steps[j.current] || {};
    return `<button class="pill run" id="jobpill" data-act="drawer-job"><span class="dot run"></span>${esc(j.title)}${j.steps.length > 1 ? ` · ${j.current + 1}/${j.steps.length} ${esc(step.title)}` : ""}</button>`;
  }
  return `<button class="pill" id="jobpill" data-act="drawer-job"><span class="dot ${S.system?.mapper_alive ? "warn" : ""}"></span>${S.system?.mapper_alive ? "检测到外部建图进程" : "空闲"}</button>`;
}
function footHtml() {
  const s = S.system; if (!s) return "";
  return `<div>${esc(s.branch)} · ${esc((s.git || "").split(" ")[0])}</div><div>ROS ${s.ros?.ok ? "<span class=ok>就绪</span>" : "<span class=bad>缺失</span>"} · 磁盘 ${s.disk_free_gb} GB 可用</div><div>Studio v${esc(s.version)}</div>`;
}
function datasetBadges(d) {
  if (!d) return `<span class="badge bad">未选择</span>`;
  if (d.error) return `<span class="badge bad">不可用</span>`;
  return `${d.lio_ready ? '<span class="badge ok">LIO 就绪</span>' : '<span class="badge bad">缺 LiDAR/IMU</span>'} ${d.livo_ready ? '<span class="badge ok">LIVO 就绪</span>' : '<span class="badge warn">无相机</span>'} ${d.has_ins ? '<span class="badge accent">INS/RTK</span>' : '<span class="badge warn">无 INS</span>'}`;
}
function datasetSummary(d, compact = false) {
  if (!d) return `<div class="empty">尚未选择数据源</div>`;
  if (d.error) return `<div class="callout bad">${esc(d.path)}<br>${esc(d.error)}</div>`;
  const stats = `<div class="stats">
    <div class="stat"><div class="k">时长</div><div class="v">${fmtDur(d.duration_s)}</div></div>
    <div class="stat"><div class="k">分段</div><div class="v">${d.segment_count}<small>${d.kind === "raw_split" ? "原始分段" : "单 bag"}</small></div></div>
    <div class="stat"><div class="k">大小</div><div class="v">${d.size_gb}<small>GB</small></div></div>
    <div class="stat"><div class="k">LiDAR</div><div class="v">${(d.lidar_topics || []).length}<small>路</small></div></div>
    <div class="stat"><div class="k">相机</div><div class="v">${d.has_camera ? "有" : "无"}</div></div>
    <div class="stat"><div class="k">采集时间</div><div class="v" style="font-size:13px">${esc(d.start_iso)}</div></div></div>`;
  if (compact) return stats;
  const gaps = (d.coverage_gaps || []).length ? `<div class="callout warn" style="margin-top:10px">分段之间存在 ${d.coverage_gaps.length} 处 >0.5 s 的间隙：${d.coverage_gaps.map((g) => `${esc(g.after)} → ${esc(g.before)} (${g.gap_s}s)`).join("；")}</div>` : "";
  const segs = `<table style="margin-top:12px"><thead><tr><th>分段</th><th class=num>时长</th><th class=num>消息</th><th>话题</th></tr></thead><tbody>${(d.segments || []).map((s) => `<tr><td class="mono small">${esc(s.name)}</td><td class=num>${fmtDur(s.duration_s)}</td><td class=num>${s.messages}</td><td class="small muted">${Object.keys(s.topics).map((t) => t.replace(/^\//, "")).join(", ")}</td></tr>`).join("")}</tbody></table>`;
  const topics = `<details style="margin-top:10px"><summary>话题清单（${Object.keys(d.topics || {}).length}）</summary><table><tbody>${Object.entries(d.topics || {}).map(([n, t]) => `<tr><td class="mono small">${esc(n)}</td><td class="small muted">${esc(t.type)}</td><td class="num small">${t.messages}</td></tr>`).join("")}</tbody></table></details>`;
  return stats + gaps + segs + topics;
}
function checksHtml(v) {
  if (!v) return `<div class="empty">没有车辆</div>`;
  return `<div class="list checks">${v.checks.map((c) => `<div class="item"><div><span class="dot ${c.ok ? "ok" : c.optional ? "" : "warn"}"></span> &nbsp;${esc(c.label)}${c.value != null ? ` <span class="mono small muted">${c.value} s</span>` : ""}</div><div class="small ${c.ok ? "ok" : "muted"}">${c.ok ? "已完成" : c.optional ? "可选" : "待完成"}</div></div>`).join("")}</div>`;
}
function stepperHtml(v) {
  const done = new Set(v?.stages_done || []);
  const hasRun = (mode) => S.runs.some((r) => r.mode === mode && r.status === "done");
  const items = [
    ["import", "导入出厂标定", done.has("import")], ["ground", "静止地面精修", done.has("ground") || done.has("precheck")],
    ["lio", "LIO 建图", hasRun("lio") || done.has("driven")], ["driven", "行驶段精修", done.has("driven")],
    ["camtime", "相机时间偏移", !!(v?.checks || []).find((c) => c.key === "camtime")?.ok],
    ["livo", "LIVO 建图", hasRun("livo") || done.has("camlidar")], ["camlidar", "相机外参复核", done.has("camlidar"), true],
    ["active", "激活生产版本", !!v?.active],
  ];
  const nextKey = v?.next?.stage === "import" ? "import" : v?.next?.pipeline === "lidar" ? "ground" : v?.next?.pipeline === "camera" ? "camtime" : null;
  return `<div class="stepper">${items.map(([k, l, ok, opt], i) => `<div class="step ${ok ? "done" : ""} ${k === nextKey ? "next" : ""} ${opt ? "optional" : ""}"><div class="n">${ok ? "✓" : i + 1}</div><div class="l">${l}</div><div class="d">${opt ? "可选 · 人工确认" : ""}</div></div>`).join("")}</div>`;
}
function stageLabel(stage) { return (S.stages[stage] || {}).label || stage; }
function chart(series, key, color = "#0071e3", ymaxHint) {
  const pts = (series || []).filter((p) => p[key] != null && Number.isFinite(p[key]));
  if (pts.length < 2) return `<div class="empty small">暂无曲线数据</div>`;
  const W = 640, H = 150, L = 42, R = 8, T = 8, B = 20;
  const t0 = pts[0].t, t1 = pts[pts.length - 1].t || t0 + 1;
  const ymax = Math.max(ymaxHint || 0, ...pts.map((p) => p[key])) * 1.1 || 1;
  const x = (t) => L + ((t - t0) / Math.max(1e-6, t1 - t0)) * (W - L - R);
  const y = (v) => T + (1 - v / ymax) * (H - T - B);
  const d = pts.map((p, i) => `${i ? "L" : "M"}${x(p.t).toFixed(1)},${y(p[key]).toFixed(1)}`).join(" ");
  const ticks = [0, 0.25, 0.5, 0.75, 1].map((f) => `<line x1="${L}" x2="${W - R}" y1="${y(ymax * f)}" y2="${y(ymax * f)}"/><text x="2" y="${y(ymax * f) + 3}">${(ymax * f).toFixed(ymax < 1 ? 2 : 1)}</text>`).join("");
  return `<svg class="chart" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none"><g class="grid">${ticks}</g><path d="${d}" fill="none" stroke="${color}" stroke-width="1.6"/><text x="${L}" y="${H - 4}">${(t1 - t0).toFixed(0)} s</text></svg>`;
}

// ------------------------------------------------------------------ live monitor
function liveMonitor() {
  const j = S.job && S.job.status === "running" ? S.job : null;
  const live = S.live;
  const rec = live?.record;
  const round = currentRound();
  if (!j && !rec) return `<div id="live-monitor"></div>`;
  const phase = rec?.phase || (j ? "starting" : "");
  const player = live?.player || {};
  const pct = phase === "done" ? 100 : player.percent != null ? Math.min(99, player.percent) : 0;
  const latest = live?.latest || {};
  const phaseIdx = PHASES.findIndex((p) => p[0] === phase);
  const stepInfo = j && j.steps.length > 1 ? `<div class="small muted">流水线步骤 ${j.current + 1}/${j.steps.length}：${esc((j.steps[j.current] || {}).title)}</div>` : "";
  const stopBtns = j ? `<div class="row"><button class="btn" data-act="stop" data-mode="finish" ${j.stop_mode ? "disabled" : ""} title="停止回放，让算法完成优化、导出与评估">提前结束并保存</button><button class="btn danger" data-act="stop" data-mode="abort">强制终止</button></div>` : "";
  return `<div id="live-monitor" class="card" style="border:1px solid var(--accent-soft)">
    <div class="card-h"><h2><span class="dot ${j ? "run" : rec?.status === "done" ? "ok" : rec?.status === "failed" ? "bad" : ""}"></span>${esc(round || j?.title || "")} ${rec?.mode || j?.meta?.mode ? `<span class="badge">${esc((rec?.mode || j?.meta?.mode || "").toUpperCase())}</span>` : ""}</h2>${stopBtns}</div>
    ${stepInfo}
    <div class="phases" style="margin:8px 0 10px">${PHASES.map(([k, l], i) => `<span class="phase ${k === phase ? "on" : i < phaseIdx ? "done" : ""}">${l}</span>`).join("")}${rec?.status === "failed" ? `<span class="phase" style="background:var(--bad-soft);color:var(--bad)">失败</span>` : ""}</div>
    <div class="progress"><div style="width:${pct}%"></div></div>
    <div class="row small muted" style="margin-top:6px;justify-content:space-between"><span>${player.t != null ? `回放 ${player.t.toFixed(1)} / ${player.total.toFixed(1)} s · ${player.msgs} 条 · 延迟 ${player.lag_ms} ms` : !round && j ? "当前步骤不是建图轮次（见任务日志）" : phase === "starting" ? "正在启动建图节点…" : ""}</span><span>${rec?.start ? `开始 ${fmtT(rec.start)}` : ""}</span></div>
    ${rec?.error ? `<div class="callout bad" style="margin-top:8px">${esc(rec.error)}</div>` : ""}
    <div class="grid g4" style="margin-top:14px">
      <div class="stat"><div class="k">关键帧</div><div class="v">${latest.keyframes ?? "—"}</div></div>
      <div class="stat"><div class="k">前端 RTK 残差</div><div class="v">${cm(latest.frontend_rtk_residual_m)}</div></div>
      <div class="stat"><div class="k">后端 RTK 残差</div><div class="v">${cm(latest.rtk_residual_m)}</div></div>
      <div class="stat"><div class="k">车速</div><div class="v">${num(latest.frontend_speed_mps, 1)}<small>m/s</small></div></div>
    </div>
    <div class="grid g2" style="margin-top:12px">
      <div><h4>前端 RTK 残差 (m)</h4>${chart(live?.diagnostics, "frontend_rtk_residual_m", "#0071e3", 0.1)}</div>
      <div><h4>在线轨迹 vs INS 差 (m)</h4>${chart(live?.online_series, "latest", "#5e5ce6", 0.1)}</div>
    </div>
    ${live?.launch_events?.length ? `<details style="margin-top:10px"><summary>算法事件（${live.launch_events.length}）</summary><div class="events">${live.launch_events.slice(-40).map((e) => `<div class="${/WARN/.test(e) ? "warn" : /ERROR|FATAL|died/.test(e) ? "bad" : ""}">${esc(e)}</div>`).join("")}</div></details>` : ""}
  </div>`;
}

// ------------------------------------------------------------------ pages
function render() {
  document.querySelectorAll("#nav a").forEach((a) => a.classList.toggle("active", a.dataset.page === S.page));
  $("#crumbs").innerHTML = `LIVO Studio <span>/ ${PAGES[S.page]}</span>`;
  const v = vehicle(); const d = S.dataset;
  $("#context").innerHTML = `<span class="ctx">车辆 <b>${esc(v ? `${v.mine}/${v.truck}` : "—")}</b></span><span class="ctx">数据源 <b title="${esc(d?.path || "")}">${esc(d ? d.name : "—")}</b></span><span class="ctx">标定 <b>${esc(v?.active ? v.active.split("/").pop() : "无")}</b></span>`;
  $("#jobpill").outerHTML = jobPill();
  $("#sidebar-foot").innerHTML = footHtml();
  const page = { home: pageHome, data: pageData, calib: pageCalib, map: pageMap, results: pageResults, system: pageSystem }[S.page];
  $("#page").innerHTML = page();
  renderDrawer();
}

function pageHome() {
  const v = vehicle(); const d = S.dataset; const running = S.job && S.job.status === "running";
  const m = S.state.mapping;
  const canLIVO = d && d.livo_ready, canLIO = d && d.lio_ready;
  const mode = m.mode === "livo" && !canLIVO ? "lio" : m.mode;
  const nextBtn = !v ? `<button class="btn primary" data-act="go" data-page="calib">导入出厂标定</button>`
    : v.next.pipeline ? `<button class="btn primary" data-act="pipeline" data-kind="${v.next.pipeline}" ${running ? "disabled" : ""}>${v.next.pipeline === "lidar" ? "运行 LiDAR 标定流水线" : "运行相机标定流水线"}</button>`
    : v.next.stage === "import" ? `<button class="btn primary" data-act="go" data-page="calib">导入出厂标定</button>` : `<span class="badge ok">标定完整</span>`;
  return `
  <div class="hero" style="margin-bottom:18px"><div><h1>工作台</h1><div class="muted">标准流程：① 选择原始录制 → ② 确认车辆标定 → ③ 一键建图，结果自动评估并归档到 results/&lt;轮次&gt;/。原始 ROS 2 mcap 直接使用，不需要转换。</div></div></div>
  ${liveMonitor()}
  <div class="grid g3" style="margin-top:${running || S.live ? 16 : 0}px">
    <div class="card"><div class="card-h"><h2><span class="stepnum ${canLIO ? "done" : ""}">1</span>数据源</h2><button class="btn sm" data-act="go" data-page="data">更换</button></div>
      <div class="t" style="font-weight:600">${esc(d?.name || "未选择")}</div><div class="small muted ellipsis" title="${esc(d?.path || "")}">${esc(d?.path || "")}</div>
      <div class="row" style="margin:8px 0 12px">${datasetBadges(d)}</div>${datasetSummary(d, true)}</div>
    <div class="card"><div class="card-h"><h2><span class="stepnum ${v?.ready_level === "livo" ? "done" : v?.active ? "warn" : ""}">2</span>车辆与标定</h2><button class="btn sm" data-act="go" data-page="calib">管理</button></div>
      <div class="row" style="margin-bottom:8px"><select data-act="set-vehicle" style="max-width:220px">${S.vehicles.map((x) => `<option value="${x.id}" ${v && x.id === v.id ? "selected" : ""}>${esc(x.mine)} / ${esc(x.truck)}</option>`).join("")}${S.vehicles.length ? "" : "<option>（无车辆）</option>"}</select>
        ${v?.active ? `<span class="badge dark" title="${esc(v.active)}">${esc(v.active.split("/").pop())}</span>` : '<span class="badge bad">无激活版本</span>'}</div>
      ${checksHtml(v)}<div class="row" style="margin-top:12px;justify-content:space-between"><span class="small muted">${esc(v?.next?.reason || "")}</span>${nextBtn}</div></div>
    <div class="card"><div class="card-h"><h2><span class="stepnum">3</span>一键建图</h2><button class="btn sm" data-act="go" data-page="map">高级设置</button></div>
      <div class="seg big" style="margin-bottom:12px"><button class="${mode === "lio" ? "active" : ""}" data-act="mode" data-mode="lio" ${canLIO ? "" : "disabled"}>LIO</button><button class="${mode === "livo" ? "active" : ""}" data-act="mode" data-mode="livo" ${canLIVO ? "" : "disabled"}>LIVO（含相机）</button></div>
      <div class="stack">
        ${toggle("use_rtk", "RTK / INS 融合", m.use_rtk)}${toggle("save_map", "保存地图 (PCD)", m.save_map)}${toggle("rviz", "打开 RViz", m.rviz)}${toggle("evaluate", "结束后自动精度评估", m.evaluate)}
      </div>
      <div class="small muted" style="margin:10px 0 12px">标定：${esc(v?.active || "未激活")}${m.calibration_id && m.calibration_id !== v?.active ? ` → 指定 ${esc(m.calibration_id)}` : ""}</div>
      <button class="btn primary big" style="width:100%" data-act="start" ${running || !canLIO || !v?.active ? "disabled" : ""}>${running ? "建图进行中…" : `开始 ${mode.toUpperCase()} 建图`}</button>
      ${!v?.active ? '<div class="small bad" style="margin-top:6px">请先在“标定”页导入并激活一个标定版本</div>' : ""}
    </div>
  </div>
  <div class="grid g2" style="margin-top:16px">
    <div class="card"><div class="card-h"><h2>最近结果</h2><a data-act="go" data-page="results">全部</a></div>${runsTable(S.runs.slice(0, 6), true)}</div>
    <div class="card"><div class="card-h"><h2>最近任务</h2><a data-act="go" data-page="system">历史</a></div>${jobsTable(S.jobs.slice(0, 6))}</div>
  </div>`;
}
function toggle(key, label, on, section = "mapping") {
  return `<label class="check"><input type="checkbox" data-act="toggle" data-section="${section}" data-key="${key}" ${on ? "checked" : ""}><span class="switch"></span><span>${label}</span></label>`;
}
function metricCells(r) {
  const h = r.metrics || {}; const n = h.native, o = h.optimized;
  const evalBtn = (label) => `<button class="btn sm" data-act="evaluate" data-round="${esc(r.round)}" ${r.status === "running" ? "disabled" : ""}>${label}</button>`;
  if (!r.evaluated) return { native: '<span class="muted">未评估</span>', opt: evalBtn("评估"), map: "—", pole: "—" };
  if (!o && !n) return { native: '<span class="badge warn" title="评估输出不完整（summary 解析失败），请重新评估">评估异常</span>', opt: evalBtn("重新评估"), map: "—", pole: "—" };
  const native = n ? `<span title="全程后轴 3D 误差 中位 · 终点（无对齐）">${fm(n.median_m)} · 终点 ${fm(n.final_m)}</span>${n.final_xy_m != null ? `<div class="small muted">终点 XY ${fm(n.final_xy_m)} · Z ${fm(n.final_z_m)}${n.yaw_drift_end_deg != null ? ` · 偏航 ${sdeg(n.yaw_drift_end_deg)}` : ""}${n.path_length_ratio != null ? ` · <span title="原生里程 / RTK 融合里程：偏离 1 即前端存在尺度误差" class="${Math.abs(n.path_length_ratio - 1) > 0.005 ? "warn" : ""}">尺度 ${(n.path_length_ratio * 100).toFixed(2)}%</span>` : ""}</div>` : ""}${h.startup?.native ? `<div class="small muted">起步 30 m 中位 ${cm(h.startup.native.median_m)} · 最大 ${cm(h.startup.native.max_m)}</div>` : ""}` : "—";
  const opt = o ? `<span title="RTK/INS 融合后 后轴 3D 误差 中位 / P95 / 最大">${cm(o.median_m)} / ${cm(o.p95_m)} / ${cm(o.max_m)}</span> <span class="badge ${o.budget === "PASS" ? "ok" : "warn"}">${esc(o.budget || "—")}</span>${o.route_le10cm_pct != null ? `<div class="small muted">路线 ≤10 cm ${pct(o.route_le10cm_pct)} · >20 cm ${pct(o.route_gt20cm_pct)}</div>` : ""}` : "—";
  const g = h.ground_5_20;
  const map = g ? `<span title="5–20 m 地面同点平面 RMS 厚度 原生 → 优化后（中位）">${f1(g.median_before_mm)} → <b>${f1(g.median_after_mm)}</b> mm</span><div class="small muted">带内 ${pct(g.within_bound_pct)} · ${g.cells} 格</div>` : "—";
  const pp = h.pole_pair_separation_cm;
  const pole = pp ? `<span title="灯柱轴线 XY 多期分离 中位 / P95 / 最大">${f1(pp.median)} / ${f1(pp.p95)} / ${f1(pp.max)} cm</span><div class="small muted">${h.pole_chunks} 期</div>` : "—";
  return { native, opt, map, pole };
}
function runsTable(runs, compact = false) {
  if (!runs.length) return `<div class="empty">还没有建图结果</div>`;
  const badge = (r) => r.status === "running" ? '<span class="badge accent"><span class="dot run"></span>运行中</span>' : r.status === "done" ? '<span class="badge ok">完成</span>' : r.status === "failed" ? '<span class="badge bad">失败</span>' : r.status === "lost" ? '<span class="badge warn">中断</span>' : `<span class="badge">${esc(r.status)}</span>`;
  const head = compact
    ? "<th>原生轨迹 vs RTK</th><th>RTK 优化后</th>"
    : `<th title="前端里程计（RViz 红线）后轴 3D 误差，全程、无对齐，随里程累积">原生轨迹 vs RTK（全程）</th><th title="RTK/INS 融合结果（蓝线）后轴 3D 误差">RTK 优化后（中位 / P95 / 最大）</th><th title="5–20 m 地面同点平面厚度，原生 → 优化后">地面厚度 5–20 m</th><th title="同一灯柱在不同地图块中的轴线 XY 分离">灯柱多期一致性</th>`;
  return `<table><thead><tr><th>轮次</th><th>模式</th><th>状态</th>${compact ? "" : "<th>数据源</th><th>标定</th>"}<th>开始</th>${head}</tr></thead><tbody>
  ${runs.map((r) => { const c = metricCells(r); return `<tr class="click ${S.runDetail?.round === r.round ? "sel" : ""}" data-act="open-run" data-round="${esc(r.round)}"><td class="mono">${esc(r.round)}</td><td><span class="badge">${esc((r.mode || "").toUpperCase())}</span></td><td>${badge(r)}</td>${compact ? "" : `<td class="small muted ellipsis" style="max-width:180px" title="${esc(r.dataset)}">${esc(baseName(r.dataset))}</td><td class="small muted">${esc(r.calibration ? baseName(r.calibration.replace(/\/calibration\.json$/, "")) : "yaml")}</td>`}<td class="small nowrap">${fmtT(r.start)}</td><td class="small">${c.native}</td><td class="small">${c.opt}</td>${compact ? "" : `<td class="small">${c.map}</td><td class="small">${c.pole}</td>`}</tr>`; }).join("")}</tbody></table>`;
}
function jobsTable(jobs) {
  if (!jobs.length) return `<div class="empty">暂无任务</div>`;
  const badge = (j) => ({ running: '<span class="badge accent"><span class="dot run"></span>运行中</span>', done: '<span class="badge ok">完成</span>', failed: '<span class="badge bad">失败</span>', stopped: '<span class="badge warn">已终止</span>', lost: '<span class="badge warn">中断</span>' }[j.status] || esc(j.status));
  return `<table><thead><tr><th>任务</th><th>状态</th><th>开始</th><th class=num>耗时</th></tr></thead><tbody>${jobs.map((j) => `<tr class="click" data-act="open-job" data-id="${j.id}"><td>${esc(j.title)}${j.steps.length > 1 ? ` <span class="small muted">${j.steps.filter((s) => s.status === "done").length}/${j.steps.length} 步</span>` : ""}${j.error ? `<div class="small bad ellipsis" style="max-width:420px">${esc(j.error)}</div>` : ""}</td><td>${badge(j)}</td><td class="small nowrap">${fmtT(j.started)}</td><td class="num small">${fmtDur(secondsBetween(j.started, j.finished || (j.status === "running" ? new Date().toISOString() : null)))}</td></tr>`).join("")}</tbody></table>`;
}

function pageData() {
  const d = S.dataset; const recent = S.state.data.recent || [];
  return `<h1 style="margin-bottom:6px">数据源</h1><div class="muted" style="margin-bottom:18px">直接选择车端原始录制目录（包含 Perception_* / Camera_* 分段的 ros2bag 根目录）或任意单个 bag 目录。Studio 只读取 metadata，不做任何转换；回放时按时间戳在线合并全部分段。</div>
  <div class="grid" style="grid-template-columns: 380px 1fr">
    <div class="stack">
      <div class="card"><div class="card-h"><h2>最近使用</h2></div><div class="list">${recent.map((p) => `<div class="item click ${d && d.path === p ? "sel" : ""}" data-act="select-dataset" data-path="${esc(p)}"><div><div class="t">${esc(baseName(p))}</div><div class="s ellipsis" style="max-width:260px" title="${esc(p)}">${esc(p)}</div></div><div class="row">${d && d.path === p ? '<span class="badge accent">当前</span>' : ""}<button class="btn sm ghost" data-act="forget-dataset" data-path="${esc(p)}" title="从列表移除">✕</button></div></div>`).join("") || '<div class="empty">无</div>'}</div></div>
      <div class="card"><div class="card-h"><h2>浏览目录</h2></div>
        <div class="inline-path"><input type="text" id="fs-path" value="${esc(S.fs?.path || "/home/project")}" placeholder="/home/project/data/..."><button class="btn" data-act="fs-go">打开</button></div>
        <div class="fsb" style="margin-top:8px">${fsHtml()}</div>
        <div class="small muted" style="margin-top:6px">带 <span class="badge ok">录制</span> / <span class="badge">bag</span> 标记的目录可直接选用。</div></div>
    </div>
    <div class="card"><div class="card-h"><h2>${esc(d?.name || "未选择")}</h2><div class="row">${datasetBadges(d)}${d && !d.error ? `<button class="btn sm" data-act="refresh-dataset">重新读取</button>` : ""}</div></div>
      <div class="small muted mono" style="margin-bottom:10px">${esc(d?.path || "")}</div>${datasetSummary(d)}</div>
  </div>`;
}
function fsHtml() {
  const f = S.fs; if (!f) return `<div class="e dim">输入路径后点击“打开”</div>`;
  const up = `<div class="e" data-act="fs-open" data-path="${esc(f.parent)}"><span>↰</span><span class="n muted">上一级</span></div>`;
  const here = f.kind ? `<div class="e" data-act="select-dataset" data-path="${esc(f.path)}"><span>✓</span><span class="n accent">选用当前目录 ${esc(baseName(f.path))}</span><span class="badge ${f.kind === "recording" ? "ok" : ""}">${f.kind === "recording" ? "录制" : "bag"}</span></div>` : "";
  return up + here + f.entries.filter((e) => e.dir).map((e) => `<div class="e" data-act="${e.kind ? "select-dataset" : "fs-open"}" data-path="${esc(e.path)}"><span>${e.kind ? "▣" : "▸"}</span><span class="n">${esc(e.name)}</span>${e.kind ? `<span class="badge ${e.kind === "recording" ? "ok" : ""}">${e.kind === "recording" ? "录制" : "bag"}</span>` : ""}</div>`).join("");
}

function pageCalib() {
  const v = vehicle(); const cal = S.cal || { versions: [], active: {}, sources: [] };
  const versions = cal.versions.filter((x) => v && x.mine === v.mine && x.truck === v.truck).sort((a, b) => (a.version > b.version ? -1 : 1));
  const d = S.dataset; const running = S.job && S.job.status === "running";
  const c = S.state.calibration; const f = S.calibForm;
  const stageOpts = Object.entries(S.stages).sort((a, b) => a[1].order - b[1].order).filter(([k]) => k !== "import");
  const info = S.stages[f.stage] || {};
  const needs = info.needs || [];
  const modeForRun = info.mapping;
  const runOpts = S.runs.filter((r) => r.fusion_dir && r.status === "done" && (!modeForRun || modeForRun === "any" || r.mode === modeForRun));
  const parents = versions.map((x) => `<option value="${esc(x.id)}" ${(f.parent || v?.active) === x.id ? "selected" : ""}>${esc(x.version)} · ${esc(x.stage_label)}${x.active ? " ★" : ""}</option>`).join("");
  return `<h1 style="margin-bottom:6px">标定</h1><div class="muted" style="margin-bottom:18px">每辆车的标定按 <code>config/carlibration/&lt;矿山&gt;/&lt;车辆&gt;/vNNN_&lt;阶段&gt;/</code> 版本化保存；每个阶段都以父版本为输入生成新版本，绝不覆盖。SLAM 只读取“激活”的版本。</div>
  <div class="card"><div class="card-h"><h2>车辆 &nbsp;<select data-act="set-vehicle">${S.vehicles.map((x) => `<option value="${x.id}" ${v && x.id === v.id ? "selected" : ""}>${esc(x.mine)} / ${esc(x.truck)}</option>`).join("")}</select></h2>
    <div class="row">${v?.active ? `<span class="badge dark">激活：${esc(v.active.split("/").pop())}</span>` : '<span class="badge bad">无激活版本</span>'}<button class="btn sm" data-act="toggle-import">${S.showImport ? "收起" : "新建车辆 / 导入出厂标定"}</button></div></div>
    ${S.showImport || !S.vehicles.length ? importForm(c) : ""}
    ${stepperHtml(v)}
    <div class="grid g2" style="margin-top:8px">
      <div class="stat" style="padding:14px"><div class="row" style="justify-content:space-between"><div><b>LiDAR 标定流水线</b><div class="small muted">静止地面精修 → LIO 建图（保存地图）→ 行驶段 6-DoF 精修 → 校验${c.auto_activate ? " → 激活" : ""}</div><div class="small muted">约 5 分钟（含一次完整回放）· 需要 LiDAR/IMU 数据源</div></div><button class="btn primary" data-act="pipeline" data-kind="lidar" ${running || !v?.active || !d?.lio_ready ? "disabled" : ""}>运行</button></div></div>
      <div class="stat" style="padding:14px"><div class="row" style="justify-content:space-between"><div><b>相机标定流水线</b><div class="small muted">相机时间偏移 → LIVO 建图 A → 相机-LiDAR 外参复核（叠加图）→ LIVO 建图 B（复核版本）→ 闭环判定 A/B（原生里程尺度 / 后轴误差 / 偏航）</div><div class="small muted">约 12 分钟（两次完整回放）· 需要含相机的数据源 · 复核版本仅在闭环通过后才建议激活</div></div><button class="btn primary" data-act="pipeline" data-kind="camera" ${running || !v?.active || !d?.livo_ready ? "disabled" : ""}>运行</button></div></div>
    </div>
    <div class="row" style="margin-top:10px;justify-content:space-between"><div class="small muted">父版本：<b>${esc(v?.active || "—")}</b> · 数据源：<b>${esc(d?.name || "未选择")}</b> ${datasetBadges(d)}</div><label class="check small">${`<input type="checkbox" data-act="toggle" data-section="calibration" data-key="auto_activate" ${c.auto_activate ? "checked" : ""}><span class="switch"></span><span>流水线完成后自动激活（相机外参复核除外）</span>`}</label></div>
  </div>
  <div class="grid" style="grid-template-columns: 1fr 380px; margin-top:16px">
    <div class="card"><div class="card-h"><h2>版本（${versions.length}）</h2><span class="small muted">点击查看详情 · ★ 为激活版本</span></div>
      ${versions.length ? `<table><thead><tr><th>版本</th><th>阶段</th><th>父版本</th><th>创建</th><th>说明</th></tr></thead><tbody>${versions.map((x) => `<tr class="click ${S.versionDetail?.id === x.id ? "sel" : ""}" data-act="open-version" data-id="${esc(x.id)}"><td class="mono nowrap">${x.active ? "★ " : ""}${esc(x.version)}</td><td><span class="badge ${x.advisory ? "warn" : ""}">${esc(x.stage_label)}</span>${x.closed_loop ? `<div class="small"><span class="badge ${x.closed_loop === "pass" ? "ok" : "bad"}" title="LIVO 闭环验证（原生轨迹 A/B）">闭环 ${x.closed_loop === "pass" ? "通过" : "未通过"}</span></div>` : x.advisory ? '<div class="small muted">仅供参考 · 未闭环验证</div>' : ""}</td><td class="mono small muted">${esc(x.parent ? x.parent.split("/").pop() : "—")}${x.parent_missing ? ' <span class="bad">(已删)</span>' : ""}</td><td class="small nowrap">${fmtT(x.created)}</td><td class="small muted" style="max-width:360px">${esc(x.note || "")}</td></tr>`).join("")}</tbody></table>` : '<div class="empty">该车辆还没有标定版本，请先导入出厂标定</div>'}</div>
    <div class="card"><div class="card-h"><h2>单步执行（高级）</h2></div>
      <div class="stack">
        <div class="field"><label>阶段</label><select data-act="calib-stage">${stageOpts.map(([k, s]) => `<option value="${k}" ${f.stage === k ? "selected" : ""}>${s.order}. ${esc(s.label)}</option>`).join("")}</select>
          <div class="small muted">输入：${esc(info.input || "")}<br>输出：${esc(info.output || "")}${info.advisory ? '<br><span class="warn">结果仅供参考，需人工确认叠加图后再激活</span>' : ""}</div></div>
        <div class="field"><label>父版本</label><select data-act="calib-field" data-key="parent">${parents || "<option>（无）</option>"}</select></div>
        ${needs.includes("dataset") ? `<div class="field"><label>数据源${needs.includes("camera") ? "（需含相机）" : ""}</label><input type="text" value="${esc(d?.path || "")}" disabled></div>` : ""}
        ${needs.includes("run") ? `<div class="field"><label>建图结果（${modeForRun === "livo" ? "LIVO" : "LIO"} 轮次，需保存了地图）</label><select data-act="calib-field" data-key="run">${runOpts.map((r) => `<option value="${esc(r.fusion_dir)}" ${f.run === r.fusion_dir ? "selected" : ""}>${esc(r.round)} · ${fmtT(r.start)}</option>`).join("") || "<option value=''>（没有可用轮次）</option>"}</select></div>` : ""}
        ${needs.includes("run_dir") ? `<div class="field"><label>建图轮次（含 Log/extrinsic_precheck.json）</label><select data-act="calib-field" data-key="run_dir">${S.runs.filter((r) => r.precheck).map((r) => `<option value="${esc(r.path)}" ${f.run_dir === r.path ? "selected" : ""}>${esc(r.round)}</option>`).join("") || "<option value=''>（没有自检记录）</option>"}</select></div>` : ""}
        ${f.stage === "manual" ? `<div class="form"><div class="field"><label>相机时间偏移 (s)</label><input type="number" step="0.001" data-act="calib-field" data-key="camera_offset" value="${esc(f.camera_offset ?? "")}"></div><div class="field"><label>LiDAR 时间偏移 (s)</label><input type="number" step="0.001" data-act="calib-field" data-key="lidar_offset" value="${esc(f.lidar_offset ?? "")}"></div><div class="field wide"><label>相机旋转修正 rx ry rz (deg, 相机系)</label><input type="text" data-act="calib-field" data-key="camera_rot_delta" value="${esc(f.camera_rot_delta ?? "")}" placeholder="0 0 0"></div></div>` : ""}
        ${f.stage === "camlidar" ? `<div class="field"><label>参与对齐的图像帧数</label><input type="number" data-act="calib-field" data-key="frames" value="${esc(f.frames ?? 12)}"></div>` : ""}
        <div class="field"><label>说明</label><input type="text" data-act="calib-field" data-key="note" value="${esc(f.note ?? "")}" placeholder="可选备注，写入版本记录"></div>
        <label class="check"><input type="checkbox" data-act="calib-field" data-key="activate" ${f.activate ? "checked" : ""}><span class="switch"></span><span>完成后激活</span></label>
        <button class="btn primary" data-act="run-stage" ${running || !versions.length ? "disabled" : ""}>执行 ${esc(info.label || "")}</button>
      </div></div>
  </div>`;
}
function importForm(c) {
  const sources = (S.cal?.sources || []);
  return `<div class="callout" style="margin-bottom:14px"><b>导入出厂标定</b>：把厂家/感知标定 JSON 规范化为 v001_import（矩阵正交化、相机内参、默认时间偏移）。新矿山/新车直接填写名称即可创建。
    <div class="form" style="margin-top:10px">
      <div class="field"><label>矿山</label><input type="text" id="imp-mine" value="${esc(S.state.vehicle.mine)}"></div>
      <div class="field"><label>车辆编号</label><input type="text" id="imp-truck" value="${esc(S.state.vehicle.truck)}"></div>
      <div class="field wide"><label>出厂标定 JSON</label><div class="inline-path"><input type="text" id="imp-source" list="imp-sources" value="${esc(c.source_file || "")}"><datalist id="imp-sources">${sources.map((s) => `<option value="${esc(s)}">`).join("")}</datalist></div></div>
      <div class="field"><label>LiDAR 时间偏移 (s)</label><input type="number" step="0.001" id="imp-lidar" value="${esc(c.lidar_offset ?? -0.125)}"></div>
      <div class="field"><label>相机时间偏移 (s，未知填 0)</label><input type="number" step="0.001" id="imp-camera" value="${esc(c.camera_offset ?? 0)}"></div>
      <div class="field"><label>说明</label><input type="text" id="imp-note" placeholder="例如：2026-05 出厂标定"></div>
      <div class="field" style="justify-content:flex-end"><button class="btn primary" data-act="import">导入并激活</button></div>
    </div></div>`;
}

function pageMap() {
  const v = vehicle(); const d = S.dataset; const m = S.state.mapping; const running = S.job && S.job.status === "running";
  const versions = (S.cal?.versions || []).filter((x) => v && x.mine === v.mine && x.truck === v.truck);
  const canLIVO = d && d.livo_ready, canLIO = d && d.lio_ready;
  const mode = m.mode === "livo" && !canLIVO ? "lio" : m.mode;
  return `<h1 style="margin-bottom:6px">建图</h1><div class="muted" style="margin-bottom:18px">一次建图 = 启动 fastlivo_mapping → 按时间戳实时回放原始录制 → 收尾优化导出轨迹/地图 → 自动精度评估。所有输出在 <code>results/&lt;轮次&gt;/</code>（round.json 记录完整参数与阶段）。</div>
  ${liveMonitor()}
  <div class="card" style="margin-top:16px"><div class="card-h"><h2>参数</h2><div class="row"><span class="small muted">数据源</span><b>${esc(d?.name || "未选择")}</b>${datasetBadges(d)}<button class="btn sm" data-act="go" data-page="data">更换</button></div></div>
    <div class="form">
      <div class="field"><label>模式</label><div class="seg"><button class="${mode === "lio" ? "active" : ""}" data-act="mode" data-mode="lio" ${canLIO ? "" : "disabled"}>LIO</button><button class="${mode === "livo" ? "active" : ""}" data-act="mode" data-mode="livo" ${canLIVO ? "" : "disabled"}>LIVO</button></div></div>
      <div class="field"><label>标定版本</label><select data-act="field" data-section="mapping" data-key="calibration_id"><option value="" ${!m.calibration_id ? "selected" : ""}>激活版本（${esc(v?.active ? v.active.split("/").pop() : "无")}）</option>${versions.map((x) => `<option value="${esc(x.id)}" ${m.calibration_id === x.id ? "selected" : ""}>${esc(x.version)} · ${esc(x.stage_label)}</option>`).join("")}</select></div>
      <div class="field"><label>轮次前缀</label><input type="text" data-act="field" data-section="mapping" data-key="round_prefix" value="${esc(m.round_prefix)}"></div>
      <div class="field"><label>回放倍速</label><input type="number" step="0.1" min="0.1" max="4" data-act="field" data-section="mapping" data-key="rate" value="${esc(m.rate)}"></div>
      <div class="field"><label>起始偏移 (s)</label><input type="number" step="1" min="0" data-act="field" data-section="mapping" data-key="start_offset" value="${esc(m.start_offset ?? 0)}"></div>
      <div class="field"><label>回放时长 (s，空=全部)</label><input type="number" step="1" min="0" data-act="field" data-section="mapping" data-key="duration" value="${esc(m.duration ?? "")}"></div>
      <div class="field"><label>相机帧抽稀 (frame_divider)</label><input type="number" step="1" min="1" max="10" data-act="field" data-section="mapping" data-key="frame_divider" value="${esc(m.frame_divider ?? 1)}"></div>
      <div class="field wide"><label>额外参数覆盖（空格分隔 key=value，例如 vio.img_point_cov=100 time_offset.img_time_offset=-0.115）</label><input type="text" class="mono" data-act="field" data-section="mapping" data-key="extra_sets" value="${esc(m.extra_sets || "")}"></div>
    </div>
    <div class="row" style="margin-top:14px;gap:22px">${toggle("use_rtk", "RTK / INS 融合", m.use_rtk)}${toggle("save_map", "保存地图 (PCD/扫描存档)", m.save_map)}${toggle("rviz", "RViz", m.rviz)}${toggle("precheck", "起步静止外参自检", m.precheck)}${toggle("evaluate", "自动评估", m.evaluate)}</div>
    <div class="sep"></div>
    <div class="row" style="justify-content:space-between"><div class="small muted">启动前会校验数据源话题与标定版本；进行中可“提前结束并保存”（停止回放，保留优化/导出/评估）。</div><button class="btn primary big" data-act="start" ${running || !canLIO || !v?.active ? "disabled" : ""}>${running ? "建图进行中…" : `开始 ${mode.toUpperCase()} 建图`}</button></div>
  </div>`;
}

function pageResults() {
  const ev = S.runs.filter((r) => r.evaluated && r.metrics?.optimized);
  const legend = `<div class="callout" style="margin-bottom:14px"><b>怎么读这张表</b> · <b>原生轨迹</b>＝前端里程计（LIO / LIVO，RViz 红线）与 RTK 后轴位置之差，起点对齐后不做任何轨迹对齐，误差随里程累积，用来比较前端与标定的好坏；<b>RTK 优化后</b>＝融合后输出（蓝线），是交付地图的精度；<b>地面厚度</b>与<b>灯柱一致性</b>反映多期地图的几何一致性。点击行查看完整指标（姿态漂移、分时段、起步段、后端统计）；未评估的轮次可直接点「评估」。</div>`;
  return `<h1 style="margin-bottom:6px">结果</h1><div class="muted" style="margin-bottom:12px">每个轮次目录包含 run.log / round.json / launch.log / playback.log / Log/fusion/&lt;stamp&gt;/（轨迹、地图、诊断）与 evaluation/（trajectory.log、summary.txt、metrics.json）。</div>${legend}
  <div class="card"><div class="card-h"><h2>建图轮次（${S.runs.length}，已评估 ${ev.length}）</h2><button class="btn sm" data-act="reload-runs">刷新</button></div>${runsTable(S.runs)}</div>`;
}

function pageSystem() {
  const s = S.system || {}; const ros = s.ros || {};
  return `<h1 style="margin-bottom:18px">系统</h1>
  <div class="grid g2">
    <div class="card"><div class="card-h"><h2>运行环境</h2><button class="btn sm" data-act="reload-system">刷新</button></div>
      <div class="kv">
        <div class="k">Studio</div><div>v${esc(s.version)} · PID ${s.pid} · ${fmtT(s.time)}</div>
        <div class="k">ROS 2</div><div>${ros.ok ? `<span class="ok">就绪</span> ${esc(ros.ros_distro)} · ${esc(ros.rmw)} · ${esc(ros.transports || "")}` : `<span class="bad">缺失</span> ${esc(ros.error || "")}`}</div>
        <div class="k">overlay</div><div class="mono small">${esc(ros.overlay_setup)} ${ros.overlay_exists ? "" : '<span class="bad">不存在</span>'}</div>
        <div class="k">仓库</div><div class="mono small">${esc(s.repo)} · ${esc(s.branch)} · ${esc(s.git)}</div>
        <div class="k">建图程序</div><div>${s.binary_present ? `编译时间 ${fmtT(s.binary_mtime)}` : '<span class="bad">未编译</span>'}</div>
        <div class="k">CPU / 负载</div><div>${s.cpus} 核 · ${(s.load || []).map((x) => x.toFixed(2)).join(" / ")}</div>
        <div class="k">GPU</div><div>${esc(s.gpu || "—")}</div>
        <div class="k">磁盘</div><div>${s.disk_free_gb} / ${s.disk_total_gb} GB 可用</div>
        <div class="k">进程</div><div>建图 ${s.mapper_alive ? '<span class="warn">运行中</span>' : "无"} · 回放 ${s.player_alive ? '<span class="warn">运行中</span>' : "无"}</div>
        <div class="k">任务记录</div><div class="mono small">${esc(s.jobs_dir)}</div>
      </div>
      <div class="sep"></div>
      <div class="row"><button class="btn danger" data-act="kill">清理残留 ROS 进程</button><span class="small muted">仅在建图卡死、无法通过“强制终止”结束时使用。</span></div></div>
    <div class="card"><div class="card-h"><h2>约定</h2></div>
      <div class="kv small">
        <div class="k">数据源</div><div>原始 ros2bag 目录，Perception_*（LiDAR/IMU/INS）+ Camera_*（相机）分段按时间戳在线合并回放。</div>
        <div class="k">标定版本</div><div>config/carlibration/&lt;矿山&gt;/&lt;车辆&gt;/vNNN_&lt;阶段&gt;/calibration.json；registry.json 记录激活版本；SLAM 只读取激活文件。</div>
        <div class="k">阶段后缀</div><div>${Object.entries(S.stages).sort((a, b) => a[1].order - b[1].order).map(([k, v]) => `<code>${k}</code> ${esc(v.label)}`).join(" · ")}</div>
        <div class="k">建图输出</div><div>results/&lt;轮次&gt;/：round.json（参数与阶段）、config.yaml（生效配置）、launch.log、playback.log、Log/fusion/*/、evaluation/。</div>
        <div class="k">停止语义</div><div>“提前结束并保存”= 中断回放，算法继续收尾优化、导出与评估；“强制终止”= 杀掉全部进程，结果不完整。</div>
      </div></div>
  </div>
  <div class="card" style="margin-top:16px"><div class="card-h"><h2>任务历史</h2><button class="btn sm" data-act="reload-jobs">刷新</button></div>${jobsTable(S.jobs)}</div>`;
}

// ------------------------------------------------------------------ drawers
function renderDrawer() {
  const el = $("#drawer");
  if (!S.drawer) { el.hidden = true; el.innerHTML = ""; return; }
  el.hidden = false;
  el.innerHTML = { job: drawerJob, version: drawerVersion, run: drawerRun }[S.drawer]();
  const log = $("#job-log"); if (log) log.scrollTop = log.scrollHeight;
}
function jobStepsHtml() {
  const j = S.job; if (!j) return "";
  const ic = { done: "✓", running: "●", failed: "✕", stopped: "■", skipped: "–", pending: "○", lost: "?" };
  return j.steps.map((s, i) => `<div class="item"><div><span class="${s.status === "done" ? "ok" : s.status === "failed" ? "bad" : s.status === "running" ? "accent" : "muted"}">${ic[s.status] || "○"}</span> &nbsp;${i + 1}. ${esc(s.title)}</div><div class="small muted">${s.status === "running" ? "进行中" : s.finished ? fmtDur(secondsBetween(s.started, s.finished)) : ""}</div></div>`).join("");
}
function jobLogHtml() {
  return S.jobLog.slice(-1500).map((l) => `<div class="${/^=== /.test(l) ? "hl" : /error|Error|FATAL|failed|Traceback|退出码/.test(l) ? "err" : /^\{"id"|completed|PASS/.test(l) ? "ok" : ""}">${esc(l)}</div>`).join("");
}
function drawerJob() {
  const j = S.job;
  if (!j) return `<div class="drawer-h"><h2>任务</h2><button class="btn sm" data-act="close">关闭</button></div><div class="empty">当前没有选中的任务</div>`;
  const res = j.results || {};
  const resHtml = Object.keys(res).length ? `<div class="kv small" style="margin:10px 0">${Object.entries(res).map(([k, v]) => `<div class="k">${esc(k)}</div><div class="mono">${esc(v)}</div></div>`).join("")}</div>` : "";
  return `<div class="drawer-h"><div><h2>${esc(j.title)}</h2><div class="small muted">${fmtT(j.started)} · ${j.status === "running" ? "运行中" : esc(j.status)} ${j.error ? `· <span class="bad">${esc(j.error)}</span>` : ""}</div></div>
    <div class="row">${j.status === "running" ? `<button class="btn" data-act="stop" data-mode="finish">提前结束并保存</button><button class="btn danger" data-act="stop" data-mode="abort">强制终止</button>` : ""}<button class="btn sm" data-act="close">关闭</button></div></div>
    ${j.steps.length > 1 ? `<div class="list" id="job-steps">${jobStepsHtml()}</div>` : ""}${resHtml}
    ${res.result_id ? `<div class="row" style="margin-bottom:10px"><button class="btn" data-act="open-version" data-id="${esc(res.result_id)}">查看版本 ${esc(res.result_id.split("/").pop())}</button>${res.camlidar_id ? `<button class="btn" data-act="open-version" data-id="${esc(res.camlidar_id)}">查看相机复核 ${esc(res.camlidar_id.split("/").pop())}</button>` : ""}</div>` : ""}
    ${j.meta?.round || res.round ? `<div class="row" style="margin-bottom:10px"><button class="btn" data-act="open-run" data-round="${esc(res.round || j.meta.round)}">查看轮次 ${esc(res.round || j.meta.round)}</button></div>` : ""}
    <div class="log" id="job-log">${jobLogHtml()}</div>`;
}
function drawerVersion() {
  const d = S.versionDetail; if (!d) return `<div class="empty">加载中…</div>`;
  const m = d.meta || {}; const sum = m.summary || {}; const ch = m.changes_vs_parent || {}; const val = d.validation || {};
  const lidars = (sum.lidars || []).map((l) => `<tr><td>${esc(l.lidar)}</td><td class="mono small">${l.t.map((x) => x.toFixed(3)).join(", ")}</td><td class="mono small">${l.rpy_deg.map((x) => x.toFixed(2)).join(", ")}</td><td class="mono small">${ch.lidars?.[l.lidar] ? `Δt ${ch.lidars[l.lidar].dt_cm.join("/")} cm · Δrpy ${ch.lidars[l.lidar].drpy_deg.join("/")}°` : '<span class="faint">无变化</span>'}</td></tr>`).join("");
  const cam = sum.camera_to_rear ? `<tr><td>camera→rear</td><td class="mono small">${sum.camera_to_rear.t.map((x) => x.toFixed(3)).join(", ")}</td><td class="mono small">${sum.camera_to_rear.rpy_deg.map((x) => x.toFixed(2)).join(", ")}</td><td class="mono small">${ch.camera_to_rear ? `Δt ${ch.camera_to_rear.dt_cm.join("/")} cm · Δrpy ${ch.camera_to_rear.drpy_deg.join("/")}°` : '<span class="faint">无变化</span>'}</td></tr>` : "";
  const offs = sum.time_offsets || {};
  const offHtml = Object.entries(offs).filter(([k, v]) => typeof v === "number").map(([k, v]) => `<span class="badge ${ch.time_offsets?.[k] ? "accent" : ""}">${esc(k)} ${v} s${ch.time_offsets?.[k] ? ` (原 ${ch.time_offsets[k].from})` : ""}</span>`).join(" ");
  const metrics = m.metrics && Object.keys(m.metrics).length ? `<details><summary>阶段指标</summary><pre class="small">${esc(JSON.stringify(m.metrics, null, 1))}</pre></details>` : "";
  const cl = d.closed_loop;
  const closedLoop = cl ? `<div class="callout ${cl.verdict === "pass" ? "ok" : "bad"}" style="margin:10px 0"><b>LIVO 闭环验证${cl.verdict === "pass" ? "通过" : "未通过"}</b>（原生轨迹 A/B：${esc(cl.baseline?.round || "")} → ${esc(cl.candidate?.round || "")}）<div class="small" style="margin-top:4px">原生里程尺度 ${cl.baseline?.scale != null ? (cl.baseline.scale * 100).toFixed(2) + "%" : "—"} → ${cl.candidate?.scale != null ? (cl.candidate.scale * 100).toFixed(2) + "%" : "—"}${cl.common_window ? ` · 后轴误差@${cl.common_window.t0}–${cl.common_window.t1} s ${f2(cl.common_window.baseline_rear_m)} → ${f2(cl.common_window.candidate_rear_m)} m` : ""} · 偏航 P95 ${f2(cl.baseline?.yaw_p95_deg)} → ${f2(cl.candidate?.yaw_p95_deg)}°${cl.reasons?.length ? `<br>原因：${esc(cl.reasons.join("；"))}` : ""}${cl.verdict !== "pass" ? "<br><b>请勿激活该版本</b>；继续使用其父版本（相机时间偏移版本）。" : ""}</div></div>` : (stage.advisory ? `<div class="callout warn" style="margin:10px 0">该版本仅供参考：相机-LiDAR 边缘对齐分数对外参误差不敏感，激活前必须做 LIVO 闭环验证（用该版本跑一轮 LIVO，比较「原生里程尺度」与「原生后轴误差」是否优于父版本）。相机流水线现已自动完成此验证。</div>` : "");
  const files = Object.entries(d.files || {}).map(([n, t]) => `<details><summary>${esc(n)}</summary><pre class="small" style="max-height:300px;overflow:auto">${esc(t)}</pre></details>`).join("");
  const overlays = d.overlays.length ? `<div class="sep"></div><h3>相机投影叠加图（${d.overlays.length}）</h3><div class="small muted" style="margin:4px 0 8px">“相机-LiDAR 外参复核”阶段把 LiDAR 深度边缘投影到图像上生成这些图（before/after）。该阶段结果仅供参考：请确认 after 图中边缘更贴合后再激活。</div><div class="overlays">${d.overlays.map((n) => `<a href="/api/calibrations/file?id=${encodeURIComponent(d.id)}&name=overlays/${encodeURIComponent(n)}" target="_blank"><img loading="lazy" src="/api/calibrations/file?id=${encodeURIComponent(d.id)}&name=overlays/${encodeURIComponent(n)}" title="${esc(n)}"><div class="small muted ellipsis">${esc(n)}</div></a>`).join("")}</div>` : "";
  const stage = d.stage_info || {};
  return `<div class="drawer-h"><div><h2>${esc(d.id)} ${d.active ? '<span class="badge dark">激活</span>' : ""}</h2><div class="small muted">${esc(m.stage_label || stage.label || m.stage)}${stage.advisory ? ' · <span class="warn">仅供参考</span>' : ""} · ${fmtT(m.created)} · 父版本 ${esc(m.parent || "无")}</div></div>
    <div class="row"><button class="btn primary" data-act="activate" data-id="${esc(d.id)}" ${d.active ? "disabled" : ""}>激活</button><button class="btn" data-act="validate" data-id="${esc(d.id)}">校验</button><button class="btn" data-act="use-parent" data-id="${esc(d.id)}">作为父版本</button><button class="btn danger sm" data-act="delete-version" data-id="${esc(d.id)}" ${d.active || d.children.length ? "disabled" : ""} title="${d.children.length ? "存在子版本" : d.active ? "激活中" : ""}">删除</button><button class="btn sm" data-act="close">关闭</button></div></div>
    <div class="callout ${val.ok ? (val.warnings?.length ? "warn" : "ok") : "bad"}">${val.ok ? "校验通过" : "校验失败"}${(val.errors || []).map((e) => `<div>✕ ${esc(e)}</div>`).join("")}${(val.warnings || []).map((e) => `<div>△ ${esc(e)}</div>`).join("")}</div>
    <div class="kv small" style="margin:12px 0"><div class="k">说明</div><div>${esc(m.note || "")}</div><div class="k">阶段</div><div>${esc(stage.input || "")} → ${esc(stage.output || "")}</div><div class="k">谱系</div><div class="mono">${(d.lineage || []).map((x) => x.split("/").pop()).join(" → ")}</div><div class="k">输入</div><div class="mono">${esc(JSON.stringify(m.inputs || {}))}</div><div class="k">文件</div><div class="mono">${esc(m.calibration_file || d.dir)}</div></div>
    <h3>外参摘要（后轴系，t 单位 m，rpy 单位 °）</h3>
    <table style="margin:6px 0 10px"><thead><tr><th>传感器</th><th>t (x,y,z)</th><th>rpy</th><th>相对父版本</th></tr></thead><tbody>${lidars}${cam}</tbody></table>
    <div class="row">${offHtml}${sum.camera?.intrinsics ? `<span class="badge">fx/fy/cx/cy ${sum.camera.intrinsics.map((x) => x.toFixed(1)).join(" / ")}</span>` : ""}</div>
    ${closedLoop}<div class="sep"></div>${metrics}${files}${overlays}`;
}
function metricsPanel(l) {
  const h = l.metrics || {}; const d = l.metrics_full;
  if (!l.evaluated) return '<div class="callout warn">该轮次尚未评估。点击「运行评估」生成轨迹精度、姿态漂移、地图厚度与灯柱一致性指标（约 1 分钟，可在建图结束后随时运行）。</div>';
  if (!d) return h.optimized ? "" : '<div class="callout bad">评估输出不完整（summary.txt 解析失败），请点击「重新评估」。</div>';
  const s = d.summary && !d.summary.error ? d.summary : null; const n = h.native || {}, o = h.optimized || {};
  const st = h.startup || d.startup; const mp = d.map || {}; const bk = d.backend || {}; const run = d.run || {}; const dg = d.diagnostics || {}; const pb = d.playback || {}; const prm = d.parameters || {};
  const stat = (k, v, sub) => `<div class="stat"><div class="k">${k}</div><div class="v" style="font-size:15px">${v}${sub ? `<div class="small muted" style="font-weight:400">${sub}</div>` : ""}</div></div>`;
  const stats = `<div class="stats" style="margin:10px 0">
    ${stat("原生 全程中位", fm(n.median_m), `终点 ${fm(n.final_m)}`)}${stat("原生 终点 XY / Z", `${fm(n.final_xy_m)} / ${fm(n.final_z_m)}`, n.yaw_drift_end_deg != null ? `末段偏航 ${sdeg(n.yaw_drift_end_deg)} · 俯仰 ${sdeg(n.pitch_drift_end_deg)}` : "")}
    ${n.path_length_ratio != null ? stat("原生里程尺度", `<span class="${Math.abs(n.path_length_ratio - 1) > 0.005 ? "warn" : "ok"}">${(n.path_length_ratio * 100).toFixed(2)}%</span>`, `原生 ${run.native_path_m} m / 融合 ${run.optimized_path_m} m`) : ""}
    ${st?.native ? stat(`原生 起步 ${Math.round(st.length_m || 30)} m`, `${cm(st.native.median_m)} / ${cm(st.native.max_m)}`, "中位 / 最大") : ""}
    ${stat("优化后 中位 / P95 / 最大", `${cm(o.median_m)} / ${cm(o.p95_m)} / ${cm(o.max_m)}`, o.budget ? `预算 <b class="${o.budget === "PASS" ? "ok" : "warn"}">${esc(o.budget)}</b> · ≤10 cm ${pct(o.route_le10cm_pct)}` : "")}
    ${h.ground_5_20 ? stat("地面厚度 5–20 m", `${f1(h.ground_5_20.median_before_mm)} → ${f1(h.ground_5_20.median_after_mm)} mm`, `带内 ${pct(h.ground_5_20.within_bound_pct)}`) : ""}
    ${h.pole_pair_separation_cm ? stat("灯柱多期分离", `${f1(h.pole_pair_separation_cm.median)} / ${f1(h.pole_pair_separation_cm.p95)} cm`, `中位 / P95 · ${h.pole_chunks} 期`) : ""}
  </div>`;
  const trow = (label, e) => e ? `<tr><td>${label}</td><td class=num>${fm(e.median_m)}</td><td class=num>${fm(e.p95_m)}</td><td class=num>${fm(e.max_m)}</td><td class=num>${fm(e.final_m)}</td><td class=num>${fm(e.final_xy_m)}</td><td class=num>${fm(e.final_z_m)}</td><td class=num>${pct(e.route_le10cm_pct)}</td><td class=num>${pct(e.route_gt20cm_pct)}</td><td>${e.budget ? `<span class="badge ${e.budget === "PASS" ? "ok" : "warn"}">${esc(e.budget)}</span>` : "—"}</td></tr>` : "";
  const traj = `<details open><summary>轨迹精度 · 后轴 vs RTK（全程，起点对齐后不做任何轨迹对齐）</summary>
    <table class="small"><thead><tr><th></th><th class=num>中位</th><th class=num>P95</th><th class=num>最大</th><th class=num>终点</th><th class=num>终点 XY</th><th class=num>终点 Z</th><th class=num>路线 ≤10 cm</th><th class=num>>20 cm</th><th>预算</th></tr></thead><tbody>${trow("原生（前端里程计，红线）", n)}${trow("RTK 优化后（蓝线）", o)}</tbody></table>
    <div class="small muted" style="margin-top:6px">原生误差随里程累积，反映前端（LIO / LIVO）自身的漂移；里程尺度（原生路径长 / 融合路径长）偏离 100% 说明前端存在尺度误差——LIVO 模式下这是相机外参（俯仰）误差最灵敏的指标（约 11%/°）；优化后误差反映 RTK/INS 融合输出的质量。路线百分比按行驶距离加权。${s?.native?.imu ? ` IMU 轨迹 vs INS 推算：原生 ${fm(s.native.imu.median_m)} / 优化后 ${fm(s.optimized?.imu?.median_m)}（中位）。` : ""}</div></details>`;
  const sides = (st?.sides || []).map((x) => `${x.length_m} m：对侧 ${x.opposite}/${x.samples}`).join(" · ");
  const startup = st?.native ? `<details><summary>起步段 · 前 ${Math.round(st.length_m || 30)} m（原生 ${cm(st.native.median_m)} / ${cm(st.native.p95_m)} / ${cm(st.native.max_m)}；优化后 ${st.fused ? `${cm(st.fused.median_m)} / ${cm(st.fused.p95_m)} / ${cm(st.fused.max_m)}` : "—"}）</summary>
    <div class="kv small"><div class="k">中位 / P95 / 最大</div><div>原生 ${cm(st.native.median_m)} / ${cm(st.native.p95_m)} / ${cm(st.native.max_m)} · 优化后 ${st.fused ? `${cm(st.fused.median_m)} / ${cm(st.fused.p95_m)} / ${cm(st.fused.max_m)}` : "—"}</div>
    ${sides ? `<div class="k">路线对侧样本</div><div>${sides}</div>` : ""}${st.segment_distance_m ? `<div class="k">RTK–原生 XY 线段距离</div><div>P50 ${cm(st.segment_distance_m.p50)} · P95 ${cm(st.segment_distance_m.p95)} · 最大 ${cm(st.segment_distance_m.max)}</div>` : ""}${st.correction_tilt_deg ? `<div class="k">修正倾角</div><div>${f2(st.correction_tilt_deg.min)} / ${f2(st.correction_tilt_deg.median)} / ${f2(st.correction_tilt_deg.max)}°</div>` : ""}</div></details>` : "";
  const arow = (label, a) => a ? `<tr><td>${label}</td>${["pitch", "roll", "yaw"].map((k) => `<td class=num>${f2(a[k].median)}</td><td class=num>${f2(a[k].p95)}</td><td class=num>${f2(a[k].max)}</td>`).join("")}</tr>` : "";
  const att = s?.native?.attitude_deg ? `<details open><summary>姿态 vs INS（°）· 偏航 P95：原生 ${f2(s.native.attitude_deg.yaw.p95)} / 优化后 ${f2(s.optimized?.attitude_deg?.yaw?.p95)}</summary>
    <table class="small"><thead><tr><th></th><th class=num>俯仰 中位</th><th class=num>P95</th><th class=num>最大</th><th class=num>横滚 中位</th><th class=num>P95</th><th class=num>最大</th><th class=num>偏航 中位</th><th class=num>P95</th><th class=num>最大</th></tr></thead><tbody>${arow("原生", s.native.attitude_deg)}${arow("优化后", s.optimized?.attitude_deg)}</tbody></table>
    <div class="small muted" style="margin-top:6px">${s.installation_misalignment_deg ? `后端估计的 INS 安装失准角（body x,y,z）：${s.installation_misalignment_deg.map((v) => sdeg(v)).join(" / ")}（同一车辆应在各轮次间稳定，随前端模式变化即说明前端姿态存在系统偏差）。` : ""}${s.attitude_factor_residuals_deg ? ` 姿态因子残差 P95：x ${f2(s.attitude_factor_residuals_deg.x.p95)} · y ${f2(s.attitude_factor_residuals_deg.y.p95)} · z ${f2(s.attitude_factor_residuals_deg.z.p95)}°（n=${s.attitude_factor_residuals_deg.n}）。` : ""}</div></details>` : "";
  const win = s?.windows?.length ? `<details><summary>分时段漂移（10 s 窗口）· 原生末段偏航 ${sdeg(s.windows[s.windows.length - 1].native.dyaw_deg)}，后轴误差 ${fm(s.windows[s.windows.length - 1].native.rear_m)}</summary>
    <table class="small"><thead><tr><th>窗口 (s)</th><th class=num>原生 Δ俯仰</th><th class=num>Δ横滚</th><th class=num>Δ偏航</th><th class=num>后轴误差</th><th class=num>优化后 Δ俯仰</th><th class=num>Δ横滚</th><th class=num>Δ偏航</th><th class=num>后轴</th><th class=num>IMU</th></tr></thead><tbody>
    ${s.windows.map((w) => `<tr><td class=mono>${w.t0}–${w.t1}</td><td class=num>${sdeg(w.native.dpitch_deg)}</td><td class=num>${sdeg(w.native.droll_deg)}</td><td class=num>${sdeg(w.native.dyaw_deg)}</td><td class=num>${fm(w.native.rear_m)}</td><td class=num>${sdeg(w.opt.dpitch_deg)}</td><td class=num>${sdeg(w.opt.droll_deg)}</td><td class=num>${sdeg(w.opt.dyaw_deg)}</td><td class=num>${fm(w.opt.rear_m)}</td><td class=num>${fm(w.opt.imu_m)}</td></tr>`).join("")}</tbody></table>
    ${s.speed_by_10s ? `<div class="small muted" style="margin-top:6px">原生速度 / INS 速度（m/s，每 10 s 中位）：${s.speed_by_10s.map((x) => `${f1(x[0])}/${f1(x[1])}`).join(" · ")}</div>` : ""}</details>` : "";
  const cls = (c) => ({ "all": "全部", "ground-like": "地面", "wall-like": "墙面" }[c] || c);
  const map = (mp.thickness || mp.plane_offset) ? `<details><summary>地图质量 · 同点平面厚度与跨扫描平面偏移${h.ground_5_20 ? `（5–20 m 地面 ${f1(h.ground_5_20.median_before_mm)} → ${f1(h.ground_5_20.median_after_mm)} mm，带内 ${pct(h.ground_5_20.within_bound_pct)}）` : ""}</summary>
    ${mp.thickness ? `<table class="small"><thead><tr><th>距离带</th><th>类别</th><th class=num>格数</th><th class=num>厚度 原生 → 优化后 (mm)</th><th class=num>增厚 P95 (mm)</th><th class=num>带内比例</th></tr></thead><tbody>${mp.thickness.map((t) => `<tr><td>${t.band_m} m</td><td>${cls(t.class)}</td><td class=num>${t.cells}</td><td class=num>${f1(t.median_before_mm)} → ${f1(t.median_after_mm)}</td><td class=num>${f1(t.increase_p95_mm)}</td><td class=num>${pct(t.within_bound_pct)}</td></tr>`).join("")}</tbody></table>` : ""}
    ${mp.plane_offset ? `<table class="small" style="margin-top:8px"><thead><tr><th>距离带</th><th>类别</th><th class=num>n</th><th class=num>|平面偏移| 中位 原生 → 优化后 (mm)</th><th class=num>P95 (mm)</th><th class=num>优化后 >5 cm</th></tr></thead><tbody>${mp.plane_offset.map((t) => `<tr><td>${t.band_m} m</td><td>${cls(t.class)}</td><td class=num>${t.n}</td><td class=num>${f1(t.median_raw_mm)} → ${f1(t.median_opt_mm)}</td><td class=num>${f1(t.p95_raw_mm)} → ${f1(t.p95_opt_mm)}</td><td class=num>${pct(t.opt_gt5cm_pct)}</td></tr>`).join("")}</tbody></table>` : ""}
    <div class="small muted" style="margin-top:6px">厚度：同一批点按原生 / 优化后位姿投影后的平面 RMS；跨扫描平面偏移：间隔 ≥5 s 的两次观测对同一平面的偏移（≥20 m 基线）。均为回归诊断，不是绝对精度证书。${mp.jaccard ? ` 占据 Jaccard（${mp.jaccard.map((j) => `${j.cell_cm} cm：${pct(j.raw_pct)} → ${pct(j.opt_pct)}`).join("，")}）。` : ""}</div></details>` : "";
  const pole = (s?.pole || d.pole_pairs) ? `<details><summary>灯柱多期一致性${h.pole_pair_separation_cm ? ` · 轴线 XY 分离 中位 ${f1(h.pole_pair_separation_cm.median)} · P95 ${f1(h.pole_pair_separation_cm.p95)} · 最大 ${f1(h.pole_pair_separation_cm.max)} cm（${h.pole_chunks} 期）` : ""}</summary>
    ${s?.pole?.chunks ? `<table class="small"><thead><tr><th>地图块</th><th class=num>杆身点数</th><th class=num>轴线 X</th><th class=num>轴线 Y</th><th class=num>顶部 Z (P99)</th><th class=num>倾角</th></tr></thead><tbody>${s.pole.chunks.map((c) => `<tr><td class=mono>map_${c.chunk}</td><td class=num>${c.shaft_points}</td><td class=num>${Number(c.axis_x).toFixed(3)}</td><td class=num>${Number(c.axis_y).toFixed(3)}</td><td class=num>${f2(c.top_z_p99)}</td><td class=num>${f1(c.tilt_deg)}°</td></tr>`).join("")}</tbody></table>` : ""}
    ${d.pole_pairs ? `<div class="small" style="margin-top:8px">${Object.entries(d.pole_pairs).map(([k, v]) => `<div>map_${k.replace("_", " vs map_")}：等高 XY 表面分离 ${v.equal_height_xy_m ? `${cm(v.equal_height_xy_m.median)} / ${cm(v.equal_height_xy_m.p95)} / ${cm(v.equal_height_xy_m.max)}` : "—"}${v.nn_ab_m ? ` · 最近邻中位 ${cm(v.nn_ab_m.median)}` : ""}</div>`).join("")}</div>` : ""}
    <div class="small muted" style="margin-top:6px">同一物理灯柱在不同时间段地图块中的位置差，反映多期地图的一致性（含采样与遮挡影响，非绝对精度）。</div></details>` : "";
  const kv = (pairs) => `<div class="kv small">${pairs.filter((x) => x[1] != null && x[1] !== "").map(([k, v]) => `<div class="k">${k}</div><div>${v}</div>`).join("")}</div>`;
  const backend = `<details><summary>RTK/INS 后端与运行统计</summary>${kv([
    ["帧数 / 时长 / 里程", `${run.frames ?? "—"} 帧 · ${run.duration_s ?? "—"} s · ${run.distance_m ?? "—"} m`],
    ["帧间隔 / 每帧点数", run.frame_dt_median_s != null ? `${(run.frame_dt_median_s * 1000).toFixed(1)} ms · ${run.points_per_frame_median} 点（LIVO 模式下 LiDAR 帧按图像时刻切分）` : null],
    ["关键帧 / 位置因子 / 姿态因子", `${bk.keys ?? dg.keyframes ?? "—"} / ${bk.position_factors ?? dg.position_factors ?? "—"} / ${bk.attitude_factors ?? "—"}`],
    ["修正节点 / 单步旋转", s?.corrections ? `${s.corrections.knots} · 中位 ${f2(s.corrections.rotation_step_deg.median)}° · P95 ${f2(s.corrections.rotation_step_deg.p95)}° · 最大 ${f2(s.corrections.rotation_step_deg.max)}°` : null],
    ["末关键帧最大 RTK 残差", bk.final_keyframe_max_rtk_residual_m != null ? cm(bk.final_keyframe_max_rtk_residual_m) : null],
    ["后端处理耗时", dg.processing_ms ? `中位 ${dg.processing_ms.median} ms · P95 ${dg.processing_ms.p95} ms · 最大 ${dg.processing_ms.max} ms` : null],
    ["发布滞后 / 回放滞后", `${dg.publish_lag_max_s != null ? `${(dg.publish_lag_max_s * 1000).toFixed(0)} ms` : "—"} / ${pb.max_lag_ms != null ? `${pb.max_lag_ms} ms` : "—"}`],
    ["回放", pb.messages != null ? `${pb.messages} 条消息 · ${pb.elapsed_s} s · ${esc(pb.status)}` : null],
    ["VIO 无点告警", bk.vio_no_point_msgs != null ? `${bk.vio_no_point_msgs} 次` : null],
    ["WARN / ERROR", `${bk.warnings ?? 0} / ${bk.errors ?? 0}`],
    ["外参自检", d.precheck ? `${esc(d.precheck.status)}（参考 ${esc(d.precheck.reference_topic || "")}）` : null],
    ["评估时间偏移", prm.time_offset_sec != null ? `位置 ${prm.time_offset_sec} s · 姿态 ${prm.attitude_time_offset_sec ?? 0} s` : null],
    ["前端参数", `img_time_offset ${prm["time_offset.img_time_offset"] ?? "—"} · lidar_time_offset ${prm["time_offset.lidar_time_offset"] ?? "—"} · frame_divider ${prm["camera.frame_divider"] ?? "—"} · img_point_cov ${prm["vio.img_point_cov"] ?? "—"}`],
    ["指标文件", `<a class="badge" href="/api/runs/${encodeURIComponent(d.round)}/file?name=evaluation/metrics.json" target="_blank">evaluation/metrics.json</a> 生成于 ${fmtT(d.generated)}`],
  ])}</details>`;
  return stats + traj + att + win + startup + map + pole + backend;
}
function drawerRun() {
  const l = S.live; const r = l?.record;
  if (!l) return `<div class="empty">加载中…</div>`;
  if (!l.exists || !r) return `<div class="drawer-h"><h2>${esc(l.round || "")}</h2><button class="btn sm" data-act="close">关闭</button></div><div class="empty">轮次目录不存在</div>`;
  const pre = l.precheck;
  const preHtml = pre ? `<details><summary>起步静止外参自检 ${pre.status ? `· ${esc(pre.status)}` : ""}</summary><pre class="small" style="max-height:260px;overflow:auto">${esc(JSON.stringify(pre, null, 1))}</pre><button class="btn sm" data-act="from-precheck" data-round="${esc(r.round)}">采纳自检修正为新版本</button></details>` : "";
  const files = ["run.log", "round.json", "launch.log", "playback.log", "evaluation/summary.txt", "evaluation/trajectory.log", "evaluation/metrics.json", "config.yaml"].map((n) => `<a class="badge" href="/api/runs/${encodeURIComponent(r.round)}/file?name=${encodeURIComponent(n)}" target="_blank">${n}</a>`).join(" ");
  return `<div class="drawer-h"><div><h2>${esc(r.round)} <span class="badge">${esc((r.mode || "").toUpperCase())}</span></h2><div class="small muted">${fmtT(r.start)} → ${fmtT(r.end)} · ${esc(r.status)}${r.phase ? ` / ${esc(r.phase)}` : ""} · 提交 ${esc(r.commit || "")}</div></div>
    <div class="row"><button class="btn" data-act="evaluate" data-round="${esc(r.round)}" ${r.status === "running" ? "disabled" : ""}>${l.evaluated ? "重新评估" : "运行评估"}</button><button class="btn sm" data-act="close">关闭</button></div></div>
    ${r.error ? `<div class="callout bad">${esc(r.error)}</div>` : ""}
    <div class="kv small" style="margin:10px 0"><div class="k">数据源</div><div class="mono">${esc(r.dataset)}</div><div class="k">标定</div><div class="mono">${esc(r.calibration || "config yaml 内置")}</div><div class="k">参数</div><div class="mono">${esc(JSON.stringify(r.options || {}))} ${esc(Array.isArray(r.overrides) ? r.overrides.join(" ") : r.overrides || "")}</div><div class="k">fusion</div><div class="mono">${esc(l.fusion_dir || "")}</div><div class="k">地图</div><div>${l.map_chunks ?? 0} 块 · ${l.map_size_mb ?? 0} MB${l.rgb_map_chunks ? ` · RGB ${l.rgb_map_chunks} 块` : ""}</div></div>
    ${metricsPanel(l)}
    <div class="grid g2" style="margin:12px 0"><div><h4>前端 RTK 残差 (m)</h4>${chart(l.diagnostics, "frontend_rtk_residual_m", "#0071e3", 0.1)}</div><div><h4>在线轨迹 vs INS (m)</h4>${chart(l.online_series, "latest", "#5e5ce6", 0.1)}</div></div>
    ${l.summary ? `<details><summary>评估摘要原文（summary.txt）</summary><pre class="small" style="max-height:320px;overflow:auto">${esc(l.summary.join("\n"))}</pre></details>` : ""}
    ${preHtml}
    <details><summary>算法事件（${(l.launch_events || []).length}）</summary><div class="events">${(l.launch_events || []).map((e) => `<div class="${/WARN/.test(e) ? "warn" : /ERROR|FATAL|died/.test(e) ? "bad" : ""}">${esc(e)}</div>`).join("")}</div></details>
    <div class="sep"></div><div class="row">${files}</div>`;
}

// ------------------------------------------------------------------ actions
const A = {
  go(el) {
    const page = el.dataset.page;
    if (!PAGES[page]) return;
    S.page = page; S.drawer = null;
    if (location.hash !== `#${page}`) history.replaceState(null, "", `#${page}`);
    saveState({ ui: { page } });
    render();
    if (page === "data" && !S.fs) A["fs-open"]({ dataset: { path: S.dataset?.path ? S.dataset.path.replace(/\/[^/]+$/, "") : "/home/project" } });
  },
  close() { S.drawer = null; renderDrawer(); },
  async "drawer-job"() { S.drawer = "job"; if (!S.job && S.jobs.length) { await A["open-job"]({ dataset: { id: S.jobs[0].id } }); return; } renderDrawer(); if (S.job) { await pollJob(); renderDrawer(); } },
  async "open-job"(el) { const id = el.dataset.id; S.job = { id, status: "loading", steps: [], meta: {}, results: {} }; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; renderDrawer(); await pollJob(); renderDrawer(); },
  mode(el) { S.state.mapping.mode = el.dataset.mode; saveState({ mapping: { mode: el.dataset.mode } }); render(); },
  toggle(el) { const sec = el.dataset.section, key = el.dataset.key; S.state[sec][key] = el.checked; saveState({ [sec]: { [key]: el.checked } }); if (sec === "calibration") render(); },
  field(el) { const sec = el.dataset.section, key = el.dataset.key; const v = el.type === "number" ? (el.value === "" ? "" : Number(el.value)) : el.value; S.state[sec][key] = v; saveState({ [sec]: { [key]: v } }); },
  "set-vehicle"(el) { const [mine, truck] = el.value.split("/"); S.state.vehicle = { mine, truck }; S.calibForm.parent = null; saveState({ vehicle: { mine, truck } }); render(); },
  async start() {
    const v = vehicle(); const m = S.state.mapping; const d = S.dataset;
    const mode = m.mode === "livo" && !d?.livo_ready ? "lio" : m.mode;
    await act(async () => { const job = await api("/api/mapping/start", "POST", { ...m, mode, dataset: d.path, mine: v.mine, truck: v.truck }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.live = null; await loadOverview(); render(); }, "建图已启动");
  },
  async stop(el) {
    const mode = el.dataset.mode;
    if (mode === "abort" && !confirm("强制终止会杀掉建图与回放进程，本轮结果不完整。确定？")) return;
    await act(async () => { if (S.job?.id) await api(`/api/jobs/${S.job.id}/stop`, "POST", { mode }); else await api("/api/mapping/stop", "POST", { mode }); await pollJob(); patchLive(); }, mode === "finish" ? "已停止回放，正在收尾导出" : "已发送终止信号");
  },
  async kill() { if (!confirm("清理所有残留的 fastlivo_mapping / 回放 / launch 进程？")) return; await act(() => api("/api/mapping/kill", "POST", {}), "已发送清理信号"); await loadOverview(); render(); },
  async pipeline(el) {
    const v = vehicle(); const d = S.dataset; const kind = el.dataset.kind; const c = S.state.calibration;
    if (!v?.active) return toast("请先导入并激活一个标定版本", "bad");
    const label = kind === "lidar" ? "LiDAR 标定流水线（地面精修 → LIO 建图 → 行驶段精修）" : "相机标定流水线（时间偏移 → LIVO 建图 A → 相机外参复核 → LIVO 建图 B → 闭环判定）";
    if (!confirm(`运行 ${label}\n父版本：${v.active}\n数据源：${d?.path}\n流水线包含一次完整回放建图，请勿同时启动其他建图。`)) return;
    await act(async () => { const job = await api("/api/calibrations/pipeline", "POST", { kind, id: v.active, dataset: d.path, activate: c.auto_activate !== false, use_rtk: true, rate: 1.0, note: "" }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; await loadOverview(); render(); }, "流水线已启动");
  },
  "toggle-import"() { S.showImport = !S.showImport; render(); },
  async import() {
    const body = { mine: $("#imp-mine").value.trim(), truck: $("#imp-truck").value.trim(), source: $("#imp-source").value.trim(), lidar_offset: $("#imp-lidar").value, camera_offset: $("#imp-camera").value, note: $("#imp-note").value.trim(), activate: true };
    await act(async () => { const job = await api("/api/calibrations/import", "POST", body); S.state.vehicle = { mine: body.mine, truck: body.truck }; saveState({ vehicle: S.state.vehicle, calibration: { source_file: body.source, lidar_offset: Number(body.lidar_offset), camera_offset: Number(body.camera_offset) } }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; S.showImport = false; render(); }, "导入已开始");
  },
  "calib-stage"(el) { S.calibForm.stage = el.value; render(); },
  "calib-field"(el) { S.calibForm[el.dataset.key] = el.type === "checkbox" ? el.checked : el.value; },
  async "run-stage"() {
    const v = vehicle(); const f = S.calibForm; const d = S.dataset;
    const body = { stage: f.stage, id: f.parent || v?.active, dataset: d?.path, run: f.run || ($("select[data-key=run]") || {}).value, run_dir: f.run_dir || ($("select[data-key=run_dir]") || {}).value, note: f.note, activate: !!f.activate, frames: f.frames };
    if (f.stage === "manual") { if (f.camera_offset !== "" && f.camera_offset != null) body.camera_offset = Number(f.camera_offset); if (f.lidar_offset !== "" && f.lidar_offset != null) body.lidar_offset = Number(f.lidar_offset); if (f.camera_rot_delta && f.camera_rot_delta.trim()) body.camera_rot_delta = f.camera_rot_delta.trim().split(/[\s,]+/).map(Number); }
    await act(async () => { const job = await api("/api/calibrations/stage", "POST", body); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "阶段已启动");
  },
  async "open-version"(el) { const id = el.dataset.id; S.drawer = "version"; S.versionDetail = null; renderDrawer(); await act(async () => { S.versionDetail = await api(`/api/calibrations/version?id=${encodeURIComponent(id)}`); render(); }); },
  async activate(el) {
    const v = S.versionDetail; const adv = v && v.id === el.dataset.id && (v.stage?.advisory || v.meta?.advisory);
    const cl = v && v.id === el.dataset.id ? v.closed_loop : null;
    if (cl && cl.verdict === "fail" && !confirm(`该版本未通过 LIVO 闭环验证：原生轨迹终点误差 ${cl.baseline?.final_m ?? "—"} m → ${cl.candidate?.final_m ?? "—"} m。激活后 LIVO 前端漂移会变大，确定仍要激活？`)) return;
    if (!cl && adv && !confirm("该版本为「相机-LiDAR 外参精修」结果，仅供参考：尚未通过 LIVO 闭环验证（原生轨迹漂移可能显著变大）。\n建议先用该版本运行一轮 LIVO 建图并与父版本比较原生轨迹终点误差。\n确定激活？")) return;
    await act(async () => { await api("/api/calibrations/activate", "POST", { id: el.dataset.id }); await Promise.all([loadOverview(), loadCatalogue(true)]); S.versionDetail = await api(`/api/calibrations/version?id=${encodeURIComponent(el.dataset.id)}`); render(); }, "已激活"); },
  async validate(el) { await act(async () => { const r = await api("/api/calibrations/validate", "POST", { id: el.dataset.id, strict: true }); toast(r.ok ? `校验通过${r.warnings.length ? `（${r.warnings.length} 条提示）` : ""}` : `校验失败：${r.errors.join("；")}`, r.ok ? "ok" : "bad"); }); },
  "use-parent"(el) { S.calibForm.parent = el.dataset.id; S.drawer = null; S.page = "calib"; render(); toast(`单步执行的父版本已设为 ${el.dataset.id.split("/").pop()}`); },
  async "delete-version"(el) { if (!confirm(`删除版本 ${el.dataset.id}？（目录将被移除，不可恢复）`)) return; await act(async () => { await api("/api/calibrations/delete", "POST", { id: el.dataset.id }); S.drawer = null; S.versionDetail = null; await Promise.all([loadOverview(), loadCatalogue(true)]); render(); }, "已删除"); },
  async "open-run"(el) { const round = el.dataset.round; S.runDetail = { round }; S.drawer = "run"; S.live = null; renderDrawer(); await act(async () => { S.live = await api(`/api/runs/${round}/live`); render(); }); },
  async evaluate(el) { await act(async () => { const job = await api("/api/evaluate", "POST", { round: el.dataset.round }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "评估已启动"); },
  async "from-precheck"(el) { const v = vehicle(); await act(async () => { const job = await api("/api/calibrations/stage", "POST", { stage: "precheck", id: v.active, run_dir: `${S.system.results}/${el.dataset.round}`, note: `adopted precheck of ${el.dataset.round}` }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "已启动"); },
  async "reload-runs"() { await loadRuns(); render(); },
  async "reload-jobs"() { await loadJobs(); render(); },
  async "reload-system"() { await loadOverview(); render(); },
  async "fs-go"() { await A["fs-open"]({ dataset: { path: $("#fs-path").value.trim() } }); },
  async "fs-open"(el) { await act(async () => { S.fs = await api(`/api/fs?path=${encodeURIComponent(el.dataset.path)}`); const box = $(".fsb"); if (box) box.innerHTML = fsHtml(); const inp = $("#fs-path"); if (inp) inp.value = S.fs.path; }); },
  async "select-dataset"(el) { await act(async () => { const r = await api("/api/datasets/select", "POST", { path: el.dataset.path }); S.state = r.state; S.dataset = r.dataset; await loadOverview(); render(); }, "已选择数据源"); },
  async "forget-dataset"(el) { await act(async () => { S.state = await api("/api/datasets/forget", "POST", { path: el.dataset.path }); render(); }); },
  async "refresh-dataset"() { await act(async () => { S.dataset = await api(`/api/datasets?path=${encodeURIComponent(S.dataset.path)}&refresh=1`); render(); }, "已重新读取"); },
};
document.addEventListener("click", (ev) => {
  const nav = ev.target.closest("#nav a[data-page]");
  if (nav) { ev.preventDefault(); A.go(nav); return; }
  const el = ev.target.closest("[data-act]");
  if (!el || el.tagName === "SELECT" || el.tagName === "INPUT") return;
  if (el.tagName === "LABEL") return;
  const fn = A[el.dataset.act]; if (fn) { ev.preventDefault(); fn(el); }
});
window.addEventListener("hashchange", () => {
  const page = location.hash.replace("#", "");
  if (PAGES[page] && page !== S.page) A.go({ dataset: { page } });
});
document.addEventListener("change", (ev) => {
  const el = ev.target.closest("[data-act]");
  if (!el || (el.tagName !== "SELECT" && el.tagName !== "INPUT")) return;
  const fn = A[el.dataset.act]; if (fn) fn(el);
});
document.addEventListener("keydown", (ev) => { if (ev.key === "Escape" && S.drawer) { S.drawer = null; renderDrawer(); } });

// ------------------------------------------------------------------ boot
(async function boot() {
  try {
    await loadOverview(); await loadCatalogue(); await loadRuns();
    const hashPage = location.hash.replace("#", "");
    S.page = PAGES[hashPage] ? hashPage : (S.state.ui?.page && PAGES[S.state.ui.page] ? S.state.ui.page : "home");
    if (S.job) { await pollJob(); await pollLive(); }
    render();
    if (S.page === "data") A["fs-open"]({ dataset: { path: S.dataset?.path ? S.dataset.path.replace(/\/[^/]+$/, "") : "/home/project" } });
    schedule();
  } catch (e) {
    $("#page").innerHTML = `<div class="callout bad">无法连接 Studio 后端：${esc(e.message)}</div>`;
  }
})();
