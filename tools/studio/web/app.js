/* LIVO Studio v2 front-end (vanilla JS, no build step). Talks to tools/studio/server.py. */
"use strict";

const S = {
  page: "home", state: null, system: null, vehicles: [], stages: {}, runs: [], jobs: [], dataset: null,
  cal: null, versionDetail: null, runDetail: null, live: null, job: null, jobLog: [], jobLogNext: 0,
  fs: null, drawer: null, fsTarget: null, busy: false, calibForm: { stage: "ground" }, timers: {},
  // data-assessment and calibration time-window selection (seconds from dataset start). mode =
  //   "auto"  : use the motion-profile suggestion (skip a standstill head, keep 8 s rest for IMU init)
  //   "head"  : use the first `head_dur` seconds (typical short static+drive window)
  //   "custom": explicit [start, end) in seconds
  winAssess: { mode: "auto", head_dur: 60, start: 0, end: "" },
  winCalib:  { mode: "auto", head_dur: 60, start: 0, end: "" },
  overlayForm: { times: "20,60,100", delta: "0 0 0", run: "" }, overlay: null, overlayBusy: false,
  assess: null, assessSummary: null, assessLoading: false,
  body: null, bodyBusy: false, bodyForm: { t: 0, max_points: 40000, view: "top", viewer: null },
};
const $ = (sel, root = document) => root.querySelector(sel);
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const PAGES = { home: "工作台", data: "数据源", calib: "标定", body: "车身过滤", map: "建图", results: "结果", system: "系统" };
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
function captureForms() {
  // keep uncontrolled form fields across re-renders (poller re-renders while the user types)
  S.drafts = S.drafts || {};
  document.querySelectorAll("#page input[id], #page select[id], #page textarea[id]").forEach((el) => {
    S.drafts[el.id] = el.type === "checkbox" ? el.checked : el.value;
  });
}
const saveState = (patch) => api("/api/state", "POST", patch).catch((e) => toast(e.message, "bad"));

// ------------------------------------------------------------------ data loading
async function loadOverview() {
  const o = await api("/api/overview");
  Object.assign(S, { state: o.state, system: o.system, vehicles: o.vehicles, stages: o.stages, runs: o.runs, jobs: o.jobs, dataset: o.dataset, assessSummary: o.assessment || null });
  const pid = S.state?.calibration?.parent_id;
  if (pid && !S.calibForm.parent && pid.startsWith(`${S.state.vehicle.mine}/${S.state.vehicle.truck}/`)) S.calibForm.parent = pid;
  const active = o.system.active_job;
  if (active && (!S.job || S.job.id !== active.id)) { S.job = active; S.jobLog = []; S.jobLogNext = 0; }
  if (active) S.job = { ...S.job, ...active };
}
async function loadCatalogue(refresh = false) { S.cal = await api(`/api/calibrations${refresh ? "?refresh=1" : ""}`); S.stages = S.cal.stages; }
async function loadRuns() { S.runs = await api("/api/runs?limit=150"); }
async function loadJobs() { S.jobs = await api("/api/jobs?limit=80"); }
async function pollJob() {
  if (!S.job) return;
  const doc = await api(`/api/jobs/${S.job.id}?offset=${S.jobLogNext}`);
  const log = doc.log || { lines: [], next: S.jobLogNext, offset: S.jobLogNext };
  delete doc.log;
  if (log.offset !== S.jobLogNext) { S.jobLog = []; }
  S.jobLog.push(...(log.lines || [])); S.jobLogNext = log.next ?? S.jobLogNext;
  if (S.jobLog.length > 4000) S.jobLog.splice(0, S.jobLog.length - 4000);
  const finished = S.job.status === "running" && doc.status !== "running";
  const stepChanged = (S.job.current ?? -1) !== (doc.current ?? -1);
  S.job = doc;
  if (finished) {
    toast(doc.status === "done" ? `✓ ${doc.title} 完成` : `${doc.title}：${doc.error || doc.status}`, doc.status === "done" ? "ok" : "bad");
    await Promise.all([loadOverview(), loadCatalogue(true), loadRuns()]);
    if (doc.kind === "calibration" && doc.meta?.source && doc.status === "done" &&
        doc.meta.mine === S.state.vehicle.mine && doc.meta.truck === S.state.vehicle.truck) {
      S.calibForm.parent = doc.results.result_id;
      S.state.calibration.parent_id = S.calibForm.parent;
      saveState({ calibration: { parent_id: S.calibForm.parent } });
    }
    if (doc.kind === "assess" || doc.kind === "pipeline") await loadAssessment(true);
    render();
    return;
  }
  if (stepChanged) {
    // pipeline steps land new calibration versions on disk: refresh so the stepper/checklist turn green live
    await loadCatalogue(true);
    await loadOverview();
    render();
  }
}
function currentRound() {
  const selectedDataset = S.dataset?.path || S.state?.data?.dataset || "";
  // Keep the just-finished mapping round pinned; otherwise a page reload loses the
  // live/result panel even though round.json and evaluation metrics are present.
  if (S.job?.kind === "mapping" && S.job.meta?.round &&
      (!selectedDataset || !S.job.meta.dataset || S.job.meta.dataset === selectedDataset) &&
      !(S.job.status === "done" && S.job.meta.options?.save_map === false)) return S.job.meta.round;
  if (S.job?.status === "running" && S.job.kind === "pipeline")
    return S.job.results.round || (((S.job.steps[S.job.current] || {}).title || "").match(/calib_[A-Za-z0-9_]+/) || [])[0] || null;
  if (S.runDetail?.round) return S.runDetail.round;
  if (S.page === "mapping" && Array.isArray(S.runs)) {
    const candidates = S.runs.filter((r) => r.mode && (!selectedDataset || r.dataset === selectedDataset));
    candidates.sort((a, b) => String(b.start || "").localeCompare(String(a.start || "")));
    const mapped = candidates.find((r) => r.status === "done" && r.options?.save_map !== false && r.map_chunks > 0);
    const newestFailure = candidates.find((r) => ["failed", "lost", "incomplete"].includes(r.status));
    if (newestFailure && (!mapped || String(newestFailure.start || "") >= String(mapped.start || ""))) return newestFailure.round;
    return mapped?.round || candidates[0]?.round || null;
  }
  return null;
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
      if (running) {
        try { await pollJob(); S.pollFails = 0; }
        catch (e) {
          S.pollFails = (S.pollFails || 0) + 1;  // never leave the console button-locked on a dead poll
          if (S.pollFails >= 15) { S.job = null; toast("任务状态轮询失败，已解除运行锁定（可在“系统”页查看历史）", "bad"); render(); }
        }
        await pollLive(); patchLive();
      }
      else if (S.tick++ % 3 === 0) { await loadOverview(); if ((S.page === "results" && S.runDetail) || (S.page === "mapping" && currentRound())) await pollLive(); patchLive(); }
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
function calibFlags() { return (S.state && S.state.calibration) || {}; }
function vehicle() { const v = S.state?.vehicle || {}; return S.vehicles.find((x) => x.mine === v.mine && x.truck === v.truck) || null; }
function cameraLabel(name) { return name === "fisheye" ? "鱼眼相机" : "中距相机"; }
function hasCamera(dataset, name) {
  const topic = name === "fisheye" ? "fish_camera/ffmpeg" : "midrange_camera/ffmpeg";
  return !!dataset?.camera_topics?.some((t) => t.replace(/^\//, "") === topic);
}
function cameraSelector(section, name) {
  name = name || "midrange";
  return `<div class="field"><label>${section === "calibration" ? "标定相机" : "LIVO 相机"}</label><select data-act="field" data-section="${section}" data-key="camera"><option value="midrange" ${name === "midrange" ? "selected" : ""}>中距相机</option><option value="fisheye" ${name === "fisheye" ? "selected" : ""} ${section === "mapping" && S.state.mapping.mode === "gaussian" ? "disabled" : ""}>鱼眼相机（LIVO）</option></select><div class="small ${hasCamera(S.dataset, name) ? "muted" : "warn"}">${hasCamera(S.dataset, name) ? "数据源已包含该相机" : "当前数据源未包含该相机话题"} · 初始参数读取同一出厂标定文件${section === "mapping" && S.state.mapping.mode === "gaussian" ? " · 高斯建图目前仅支持中距" : ""}</div></div>`;
}
function cameraVehicle(v, camera) {
  if (v?.camera_statuses?.[camera || "midrange"]) return v.camera_statuses[camera || "midrange"];
  if (!v || (v.camera || "midrange") === (camera || "midrange")) return v;
  return { ...v, ready_level: v.active ? "lio" : v.ready_level,
    checks: v.checks.map((c) => c.key.startsWith("cam") ? { ...c, ok: false, value: null } : c),
    stages_done: v.stages_done.filter((stage) => !stage.startsWith("cam")),
    next: { stage: "camtime", pipeline: "camera", reason: `${cameraLabel(camera)}需要独立标定与闭环验收` } };
}
function selectedParent() {
  const v = vehicle();
  const versions = (S.cal?.versions || []).filter((x) => x.mine === v?.mine && x.truck === v?.truck);
  return versions.some((x) => x.id === S.calibForm.parent) ? S.calibForm.parent : v?.active || "";
}
// Factory import at the root of a version's lineage: where the LiDAR pipeline starts by default.
function factoryOf(id) { const by = new Map((S.cal?.versions || []).map((x) => [x.id, x])); let cur = by.get(id); const seen = new Set(); while (cur && cur.parent && by.has(cur.parent) && !seen.has(cur.id)) { seen.add(cur.id); cur = by.get(cur.parent); } return cur && cur.stage === "import" ? cur.id : null; }

function bfParentVersions() {
  const v = vehicle(); if (!v) return [];
  return (S.cal?.versions || []).filter((x) => x.mine === v.mine && x.truck === v.truck)
    .sort((a, b) => (a.version > b.version ? -1 : 1));
}
function bfCurrentParentId() {
  const v = vehicle(); const list = bfParentVersions();
  const statePid = S.state?.calibration?.parent_id;
  if (statePid && list.some((x) => x.id === statePid)) return statePid;
  const active = list.find((x) => x.id === v?.active);
  if (active?.parent && list.some((x) => x.id === active.parent)) return active.parent;
  return v?.active || "";
}
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
  const segs = `<table><thead><tr><th>分段</th><th class=num>时长</th><th class=num>消息</th><th>话题</th></tr></thead><tbody>${(d.segments || []).map((s) => `<tr><td class="mono small">${esc(s.name)}</td><td class=num>${fmtDur(s.duration_s)}</td><td class=num>${s.messages}</td><td class="small muted">${Object.keys(s.topics).map((t) => t.replace(/^\//, "")).join(", ")}</td></tr>`).join("")}</tbody></table>`;
  const topics = `<details style="margin-top:10px"><summary>话题清单（${Object.keys(d.topics || {}).length}）</summary><table><tbody>${Object.entries(d.topics || {}).map(([n, t]) => `<tr><td class="mono small">${esc(n)}</td><td class="small muted">${esc(t.type)}</td><td class="num small">${t.messages}</td></tr>`).join("")}</tbody></table></details>`;
  const segCount = (d.segments || []).length;
  return stats + gaps + `<details style="margin-top:10px" ${segCount <= 10 ? "open" : ""}><summary>分段明细（${segCount}）</summary>${segs}</details>` + topics;
}
function checksHtml(v) {
  if (!v) return `<div class="empty">没有车辆</div>`;
  return `<div class="list checks">${v.checks.map((c) => `<div class="item"><div><span class="dot ${c.ok ? "ok" : c.optional ? "" : "warn"}"></span> &nbsp;${esc(c.label)}${c.value != null ? ` <span class="mono small muted">${c.value} s</span>` : ""}</div><div class="small ${c.ok ? "ok" : "muted"}">${c.ok ? "已完成" : c.optional ? "可选" : "待完成"}</div></div>`).join("")}</div>`;
}
function stepperHtml(v) {
  const done = new Set(v?.stages_done || []);
  const hasRun = (mode) => S.runs.some((r) => r.mode === mode && r.status === "done" && (mode !== "livo" || (r.camera || "midrange") === (S.state.calibration.camera || "midrange")));
  const items = [
    ["import", "导入出厂标定", done.has("import")], ["axle", "后轴对齐", !!(v?.checks || []).find((c) => c.key === "axle")?.ok],
    ["ground", "静止地面精修", done.has("ground") || done.has("precheck")],
    ["lio", "LIO 建图", hasRun("lio") || done.has("driven")], ["driven", "行驶段精修", done.has("driven")],
    ["rtktime", "RTK 关联时间", done.has("rtktime")],
    ["camtime", "相机时间偏移", !!(v?.checks || []).find((c) => c.key === "camtime")?.ok],
    ["livo", "LIVO 建图", hasRun("livo") || done.has("camlidar")], ["camlidar", "相机外参复核", done.has("camlidar"), true],
    ["active", "激活生产版本", !!v?.active && (v.active_version?.camera || "midrange") === (v.camera || "midrange")],
  ];
  if (v?.camera === "fisheye") { items.splice(items.length - 1, 0, ["camfocus", "多帧内参诊断", done.has("camfocus")], ["camphoto", "联合与独立验证", !!(v.checks || []).find((c) => c.key === "fish_validation")?.ok]); }
  const nextKey = v?.next?.stage === "import" ? "import" : v?.next?.pipeline === "lidar" ? (v.next.stage || "ground") : v?.next?.pipeline === "camera" ? "camtime" : v?.next?.pipeline === "camloop" ? "camloop" : null;
  return `<div class="stepper">${items.map(([k, l, ok, opt], i) => `<div class="step ${ok ? "done" : ""} ${k === nextKey ? "next" : ""} ${opt ? "optional" : ""}"><div class="n">${ok ? "✓" : i + 1}</div><div class="l">${l}</div><div class="d">${opt ? "可选 · 人工确认" : ""}</div></div>`).join("")}</div>`;
}
function stageLabel(stage) { return (S.stages[stage] || {}).label || stage; }
function chart(series, key, color = "#0071e3", ymaxHint) {
  return multiChart([{ series, key, color, label: "" }], ymaxHint);
}
// lines: [{series, key, color, label, dash}] drawn on a shared time axis (t of the first line defines 0)
function multiChart(lines, ymaxHint) {
  const L0 = lines.map((l) => ({ ...l, pts: (l.series || []).filter((p) => p && p[l.key] != null && Number.isFinite(p[l.key])) })).filter((l) => l.pts.length >= 2);
  if (!L0.length) return `<div class="empty small">暂无曲线数据</div>`;
  const W = 640, H = 150, L = 42, R = 8, T = 8, B = 20;
  const t0 = Math.min(...L0.map((l) => l.pts[0].t)), t1 = Math.max(...L0.map((l) => l.pts[l.pts.length - 1].t), t0 + 1);
  const ymax = Math.max(ymaxHint || 0, ...L0.flatMap((l) => l.pts.map((p) => p[l.key]))) * 1.1 || 1;
  const x = (t) => L + ((t - t0) / Math.max(1e-6, t1 - t0)) * (W - L - R);
  const y = (v) => T + (1 - Math.min(v, ymax) / ymax) * (H - T - B);
  const paths = L0.map((l) => `<path d="${l.pts.map((p, i) => `${i ? "L" : "M"}${x(p.t).toFixed(1)},${y(p[l.key]).toFixed(1)}`).join(" ")}" fill="none" stroke="${l.color}" stroke-width="${l.dash ? 1.2 : 1.8}" ${l.dash ? `stroke-dasharray="${l.dash}"` : ""}/>`).join("");
  const ticks = [0, 0.25, 0.5, 0.75, 1].map((f) => `<line x1="${L}" x2="${W - R}" y1="${y(ymax * f)}" y2="${y(ymax * f)}"/><text x="2" y="${y(ymax * f) + 3}">${(ymax * f).toFixed(ymax < 1 ? 2 : 1)}</text>`).join("");
  const labelled = L0.filter((l) => l.label);
  const legend = labelled.length ? `<g class="legend">${labelled.map((l, i) => `<line x1="${L + 6}" x2="${L + 26}" y1="${T + 8 + i * 13}" y2="${T + 8 + i * 13}" stroke="${l.color}" stroke-width="2" ${l.dash ? `stroke-dasharray="${l.dash}"` : ""}/><text x="${L + 30}" y="${T + 11 + i * 13}">${esc(l.label)}</text>`).join("")}</g>` : "";
  return `<svg class="chart" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none"><g class="grid">${ticks}</g>${paths}${legend}<text x="${L}" y="${H - 4}">${(t1 - t0).toFixed(0)} s</text></svg>`;
}
// Trajectory-vs-RTK panel: the optimized (delivered map) trajectory is the primary solid line; the online
// published pose is the dashed secondary. Before evaluation the backend keyframe residual stands in.
function trajectoryChart(live, noRtk) {
  const es = live?.error_series;
  if (es && es.length >= 2) {
    const lines = noRtk ? [] : [{ series: es, key: "opt_rear", color: "#0071e3", label: "优化后轨迹 vs RTK（后轴，评估）" }];
    lines.push({ series: live.online_series, key: "latest", color: "#5e5ce6", dash: "5 4", label: "在线轨迹 vs INS（实时）" });
    return multiChart(lines, 0.1);
  }
  return multiChart([
    ...(noRtk ? [] : [{ series: live?.diagnostics, key: "rtk_residual_m", color: "#0071e3", label: "在线发布位姿 vs RTK（后轴，发布时刻定值）" }]),
    { series: live?.online_series, key: "latest", color: "#5e5ce6", dash: "5 4", label: "在线轨迹 vs INS（实时）" },
  ], 0.1);
}
// Throughput / resource panel (results/<round>/resources.csv written by tools/livo/resmon.py)
function resourceStats(live) {
  const r = live?.resources; if (!r) return "";
  const n = (k) => { const v = parseFloat(r[k]); return Number.isFinite(v) ? v : null; };
  const gb = (mb) => (mb == null ? "—" : (mb / 1024).toFixed(mb >= 10240 ? 0 : 1));
  const rt = n("rt_factor"), tp = live?.throughput;
  const skip = live?.player?.skip_windows ? `<div class="small muted" style="margin-top:4px">长时间静止跳过：${live.player.skip_windows} 段，共 ${live.player.skip_s.toFixed(0)} s（快进期间 IMU/INS 全量、激光按间隔抽稀）</div>` : "";
  return `<div class="grid g4" style="margin-top:10px">
      <div class="stat"><div class="k">平均帧率</div><div class="v">${tp ? tp.avg_fps.toFixed(1) : "—"}<small>Hz</small></div><div class="small muted">${tp ? `处理 ${tp.frames} 帧 · 活跃 ${tp.window_s.toFixed(0)} s` : "暂无处理数据"}</div></div>
      <div class="stat"><div class="k">实时倍率</div><div class="v ${rt != null && rt < 0.95 ? "warn" : ""}">${rt == null ? "—" : rt.toFixed(2)}<small>×</small></div></div>
      <div class="stat"><div class="k">后端耗时（均值）</div><div class="v">${n("backend_ms") == null ? "—" : n("backend_ms").toFixed(0)}<small>ms/帧</small></div></div>
      <div class="stat"><div class="k">建图进程内存 / CPU</div><div class="v">${gb(n("mapper_rss_mb"))}<small>GB · ${n("mapper_cpu_pct") == null ? "—" : n("mapper_cpu_pct").toFixed(0)}%</small></div></div>
    </div>
    <div class="small muted" style="margin-top:4px">系统内存 ${gb(n("sys_used_mb"))} GB · 交换 ${gb(n("swap_used_mb"))} GB${n("decoder_rss_mb") ? ` · 相机解码 ${gb(n("decoder_rss_mb"))} GB / ${n("decoder_cpu_pct")?.toFixed(0)}%` : ""} · 回放 CPU ${n("player_cpu_pct") == null ? "—" : n("player_cpu_pct").toFixed(0)}%　（实时倍率 &lt; 1 表示建图慢于录制，回放会自动等待）</div>${skip}`;
}
function resourceCharts(live) {
  const s = live?.resource_series; if (!s || s.length < 2) return "";
  const gbS = s.map((p) => ({ t: p.t, mapper: p.mapper_rss_mb != null ? p.mapper_rss_mb / 1024 : null, sys: p.sys_used_mb != null ? p.sys_used_mb / 1024 : null, swap: p.swap_used_mb != null ? p.swap_used_mb / 1024 : null }));
  return `<div class="grid g2" style="margin-top:12px">
      <div><h4>处理帧率 (Hz)　<span class="small muted">墙钟时间轴</span></h4>${multiChart([{ series: s, key: "fps", color: "#34c759", label: "已处理帧/秒" }], 12)}</div>
      <div><h4>内存 (GB)</h4>${multiChart([{ series: gbS, key: "mapper", color: "#ff9500", label: "建图进程 RSS" }, { series: gbS, key: "sys", color: "#8e8e93", dash: "4 3", label: "系统已用" }, { series: gbS, key: "swap", color: "#ff3b30", dash: "2 3", label: "交换" }], 1)}</div>
    </div>`;
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
  const phases = rec?.mode === "gaussian" ? [...PHASES.slice(0, -1), ["gaussian", "高斯优化"], PHASES.at(-1)] : PHASES;
  const phaseIdx = phases.findIndex((p) => p[0] === phase);
  const stepInfo = j && j.steps.length > 1 ? `<div class="small muted">流水线步骤 ${j.current + 1}/${j.steps.length}：${esc((j.steps[j.current] || {}).title)}</div>` : "";
  const stopBtns = j ? `<div class="row"><button class="btn" data-act="stop" data-mode="finish" ${j.stop_mode ? "disabled" : ""} title="停止回放，让算法完成优化、导出与评估">提前结束并保存</button><button class="btn danger" data-act="stop" data-mode="abort">强制终止</button></div>` : "";
  return `<div id="live-monitor" class="card" style="border:1px solid var(--accent-soft)">
    <div class="card-h"><h2><span class="dot ${j ? "run" : rec?.status === "done" ? "ok" : rec?.status === "failed" ? "bad" : ""}"></span>${esc(round || j?.title || "")} ${rec?.mode || j?.meta?.mode ? `<span class="badge">${esc((rec?.mode || j?.meta?.mode || "").toUpperCase())}</span>` : ""}</h2>${stopBtns}</div>
    ${stepInfo}
    <div class="phases" style="margin:8px 0 10px">${phases.map(([k, l], i) => `<span class="phase ${k === phase ? "on" : i < phaseIdx ? "done" : ""}">${l}</span>`).join("")}${rec?.status === "failed" ? `<span class="phase" style="background:var(--bad-soft);color:var(--bad)">失败</span>` : rec?.status === "incomplete" ? `<span class="phase" style="background:var(--warn-soft, #fff4e5);color:var(--warn)" title="${esc(rec.error || "")}">不完整</span>` : ""}</div>
    <div class="progress"><div style="width:${pct}%"></div></div>
    <div class="row small muted" style="margin-top:6px;justify-content:space-between"><span>${player.t != null ? `回放 ${player.t.toFixed(1)} / ${player.total.toFixed(1)} s · ${player.msgs} 条 · 延迟 ${player.lag_ms} ms` : !round && j ? "当前步骤不是建图轮次（见任务日志）" : phase === "starting" ? "正在启动建图节点…" : ""}</span><span>${rec?.start ? `开始 ${fmtT(rec.start)}` : ""}</span></div>
    ${rec ? `<div class="callout ${rec.error ? "bad" : rec.status === "done" ? "ok" : "warn"}" style="margin-top:8px"><b>状态：</b>${esc(rec.status || phase || "未知")} · <b>错误：</b>${rec.error ? esc(rec.error) : rec.status === "done" ? "无" : "暂未报告"}${rec.map_chunks != null ? ` · 地图 ${rec.map_chunks} 块` : ""}${rec.metrics_complete != null ? ` · 评估 ${rec.metrics_complete ? "完整" : "不完整"}` : ""}</div>` : ""}
    <div class="grid g4" style="margin-top:14px">
      <div class="stat"><div class="k">关键帧</div><div class="v">${latest.keyframes ?? "—"}</div></div>
      <div class="stat"><div class="k">前端 RTK 残差</div><div class="v">${cm(latest.frontend_rtk_residual_m)}</div></div>
      <div class="stat"><div class="k">后端 RTK 残差</div><div class="v">${cm(latest.rtk_residual_m)}</div></div>
      <div class="stat"><div class="k">车速</div><div class="v">${num(latest.frontend_speed_mps, 1)}<small>m/s</small></div></div>
    </div>
    ${resourceStats(live)}
    <div class="grid g2" style="margin-top:12px">
      <div><h4>前端 RTK 残差 (m)</h4>${chart(live?.diagnostics, "frontend_rtk_residual_m", "#0071e3", 0.1)}</div>
      <div><h4>${live?.record?.options?.use_rtk === false ? `在线轨迹 vs INS (m)　<span class="small muted">无 RTK/INS 融合</span>` : live?.error_series?.length ? `优化后轨迹 vs RTK (m)　<span class="small muted">实线：交付轨迹（评估）；虚线：在线轨迹 vs INS</span>` : `在线位姿 vs RTK (m)　<span class="small muted">发布时刻的因果误差，写入后不再改变；交付轨迹误差在导出评估后显示</span>`}</h4>${trajectoryChart(live, live?.record?.options?.use_rtk === false)}</div>
    </div>
    <div class="small muted" style="margin-top:4px">曲线按固定时间分箱（每箱取最大值）绘制，已完成的时间段不会随刷新改变。</div>
    ${resourceCharts(live)}
    ${live?.launch_events?.length ? `<details style="margin-top:10px"><summary>算法事件（${live.launch_events.length}）</summary><div class="events">${live.launch_events.slice(-40).map((e) => `<div class="${/WARN/.test(e) ? "warn" : /ERROR|FATAL|died/.test(e) ? "bad" : ""}">${esc(e)}</div>`).join("")}</div></details>` : ""}
  </div>`;
}

function modeLabel(r) { const m = (r?.mode || "").toUpperCase() + (["livo", "gaussian"].includes(r?.mode) ? ` · ${cameraLabel(r.camera)}` : ""); return r?.options?.use_rtk === false ? m : (m ? m + "+RTK" : m); }

// ------------------------------------------------------------------ pages
function render() {
  captureForms();
  document.querySelectorAll("#nav a").forEach((a) => a.classList.toggle("active", a.dataset.page === S.page));
  $("#crumbs").innerHTML = `LIVO Studio <span>/ ${PAGES[S.page]}</span>`;
  const v = vehicle(); const d = S.dataset;
  $("#context").innerHTML = `<span class="ctx">车辆 <b>${esc(v ? `${v.mine}/${v.truck}` : "—")}</b></span><span class="ctx">数据源 <b title="${esc(d?.path || "")}">${esc(d ? ((d.path || "").split("/").filter(Boolean).slice(-2).join("/") || d.name) : "—")}</b></span><span class="ctx">标定 <b title="${S.state?.mapping?.calibration_id ? "本轮指定版本" : "按当前录制自动匹配"}">${esc(S.state?.mapping?.calibration_id ? S.state.mapping.calibration_id.split("/").pop() : "自动匹配数据")}</b></span>`;
  $("#jobpill").outerHTML = jobPill();
  $("#sidebar-foot").innerHTML = footHtml();
  const page = { home: pageHome, data: pageData, calib: pageCalib, body: pageBody, map: pageMap, results: pageResults, system: pageSystem }[S.page];
  $("#page").innerHTML = page();
  if (S.page === "body") initBodyCanvases();
  renderDrawer();
  if (S.page === "data" && S.dataset && !S.dataset.error && (!S.assess || S.assess.path !== S.dataset.path) && !S.assessLoading) {
    S.assessLoading = true;
    loadAssessment(true).then(() => { S.assessLoading = false; render(); }).catch(() => { S.assessLoading = false; });
  }
}

function pageHome() {
  const v = vehicle(); const d = S.dataset; const running = S.job && S.job.status === "running";
  const m = S.state.mapping;
  const canLIVO = d && d.livo_ready, canLIO = d && d.lio_ready;
  const mode = ["livo", "gaussian"].includes(m.mode) && !canLIVO ? "lio" : m.mode;
  const cv = cameraVehicle(v, S.state.calibration.camera);
  const nextBtn = !cv ? `<button class="btn primary" data-act="go" data-page="calib">导入出厂标定</button>`
    : cv.next.pipeline ? `<button class="btn primary" data-act="pipeline" data-kind="${cv.next.pipeline}" ${running ? "disabled" : ""}>${{ lidar: "运行 LiDAR 标定流水线", camera: "运行相机标定流水线", camloop: "运行相机外参闭环微调" }[cv.next.pipeline]}</button>`
    : cv.next.stage === "import" ? `<button class="btn primary" data-act="go" data-page="calib">导入出厂标定</button>` : `<span class="badge ok">标定完整</span>`;
  return `
  <div class="hero" style="margin-bottom:18px"><div><h1>工作台</h1><div class="muted">标准流程：① 选择原始录制 → ② 确认车辆标定 → ③ 一键建图，结果自动评估并归档到 results/&lt;轮次&gt;/。原始 ROS 2 mcap 直接使用，不需要转换。</div></div></div>
  ${liveMonitor()}
  <div class="grid g3" style="margin-top:${running || S.live ? 16 : 0}px">
    <div class="card"><div class="card-h"><h2><span class="stepnum ${canLIO ? "done" : ""}">1</span>数据源</h2><button class="btn sm" data-act="go" data-page="data">更换</button></div>
      <div class="t" style="font-weight:600">${esc(d?.name || "未选择")}</div><div class="small muted ellipsis" title="${esc(d?.path || "")}">${esc(d?.path || "")}</div>
      <div class="row" style="margin:8px 0 12px">${datasetBadges(d)}${assessBadgeHome()}</div>${datasetSummary(d, true)}</div>
    <div class="card"><div class="card-h"><h2><span class="stepnum ${v?.ready_level === "livo" ? "done" : v?.active ? "warn" : ""}">2</span>车辆与标定</h2><button class="btn sm" data-act="go" data-page="calib">管理</button></div>
      <div class="row" style="margin-bottom:8px"><select data-act="set-vehicle" style="max-width:220px">${S.vehicles.map((x) => `<option value="${x.id}" ${v && x.id === v.id ? "selected" : ""}>${esc(x.mine)} / ${esc(x.truck)}</option>`).join("")}${S.vehicles.length ? "" : "<option>（无车辆）</option>"}</select>
        ${v?.active ? `<span class="badge dark" title="${esc(v.active)}">${esc(v.active.split("/").pop())}</span>` : '<span class="badge bad">无激活版本</span>'}</div>
      ${checksHtml(cameraVehicle(v, S.state.calibration.camera))}<div class="row" style="margin-top:12px;justify-content:space-between"><span class="small muted">${esc(cv?.next?.reason || "")}</span>${nextBtn}</div></div>
    <div class="card"><div class="card-h"><h2><span class="stepnum">3</span>一键建图</h2><button class="btn sm" data-act="go" data-page="map">高级设置</button></div>
      <div class="seg big" style="margin-bottom:12px"><button class="${mode === "lio" ? "active" : ""}" data-act="mode" data-mode="lio" ${canLIO ? "" : "disabled"}>LIO</button><button class="${mode === "livo" ? "active" : ""}" data-act="mode" data-mode="livo" ${canLIVO ? "" : "disabled"}>LIVO（含相机）</button><button class="${mode === "gaussian" ? "active" : ""}" data-act="mode" data-mode="gaussian" ${canLIVO && m.camera !== "fisheye" ? "" : "disabled"} title="高斯建图目前仅支持中距相机">高斯建图</button></div>
      ${mode !== "lio" ? cameraSelector("mapping", m.camera) : ""}
      <div class="stack">
        ${toggle("use_rtk", "RTK / INS 融合", m.use_rtk)}${toggle("save_map", "保存地图 (PCD)", m.save_map)}${toggle("rviz", "打开 RViz", m.rviz)}${toggle("evaluate", "结束后自动精度评估", m.evaluate)}
      </div>
      <div class="small muted" style="margin:10px 0 12px">标定：${esc(v?.active || "未激活")}${m.calibration_id && m.calibration_id !== v?.active ? ` → 指定 ${esc(m.calibration_id)}` : ""}</div>
      <button class="btn primary big" style="width:100%" data-act="start" ${running || !canLIO || !v?.active || (mode !== "lio" && !hasCamera(d, m.camera || "midrange")) || (mode === "gaussian" && m.camera === "fisheye") ? "disabled" : ""}>${running ? "建图进行中…" : `开始 ${mode.toUpperCase()} 建图`}</button>
      ${!v?.active ? '<div class="small bad" style="margin-top:6px">请先在“标定”页导入并激活一个标定版本</div>' : ""}
    </div>
  </div>
  <div class="grid g2" style="margin-top:16px">
    <div class="card"><div class="card-h"><h2>最近结果</h2><a data-act="go" data-page="results">全部</a></div>${runsTable(S.runs.slice(0, 6), true)}</div>
    <div class="card"><div class="card-h"><h2>最近任务</h2><a data-act="go" data-page="system">历史</a></div>${jobsTable(S.jobs.slice(0, 6))}</div>
  </div>`;
}
function loopsKv(l) {
  const x = l.loops;
  const off = [].concat(l.record?.overrides || []).join(" ").includes("rtk.loop_closure_enabled=false");
  if (!x) return off ? `<div class="k">回环</div><div class="muted">已关闭</div>` : "";
  const rej = Object.entries(x.rejected || {}).map(([k, n]) => `${esc(k)} ${n}`).join("，");
  return `<div class="k">回环</div><div>${x.pairs} 对回访候选 · 配准接受 ${x.accepted}${x.rtk_dropped ? ` · RTK 约束剔除 ${x.rtk_dropped}` : ""} · <b>最终保留 ${x.kept}</b>${x.correction_median_m != null ? ` · 修正中位 ${(x.correction_median_m * 100).toFixed(1)} cm` : ""}${rej ? `<div class="small muted">未接受：${rej}</div>` : ""}</div>`;
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
  const noRtk = r.options && r.options.use_rtk === false;
  const opt = noRtk ? `<span class="badge" title="本轮未启用 RTK/INS 融合：交付轨迹即原生轨迹，无优化指标">无融合</span>` : o ? `<span title="RTK/INS 融合后 后轴 3D 误差 中位 / P95 / 最大">${cm(o.median_m)} / ${cm(o.p95_m)} / ${cm(o.max_m)}</span> <span class="badge ${o.budget === "PASS" ? "ok" : "warn"}">${esc(o.budget || "—")}</span>${o.route_le10cm_pct != null ? `<div class="small muted">路线 ≤10 cm ${pct(o.route_le10cm_pct)} · >20 cm ${pct(o.route_gt20cm_pct)}</div>` : ""}` : "—";
  const g = h.ground_5_20;
  const map = g ? `<span title="5–20 m 地面同点平面 RMS 厚度 原生 → 优化后（中位）">${f1(g.median_before_mm)} → <b>${f1(g.median_after_mm)}</b> mm</span><div class="small muted">带内 ${pct(g.within_bound_pct)} · ${g.cells} 格</div>` : "—";
  const pp = h.pole_pair_separation_cm;
  const pole = pp ? `<span title="灯柱环拟合中心 多期分离 中位 / P95 / 最大（视角无关）">${f1(pp.median)} / ${f1(pp.p95)} / ${f1(pp.max)} cm</span><div class="small muted">${h.pole_chunks} 期</div>` : "—";
  return { native, opt, map, pole };
}
function runsTable(runs, compact = false) {
  if (!runs.length) return `<div class="empty">还没有建图结果</div>`;
  const badge = (r) => r.status === "running" ? '<span class="badge accent"><span class="dot run"></span>运行中</span>' : r.status === "done" ? '<span class="badge ok">完成</span>' : r.status === "failed" ? '<span class="badge bad">失败</span>' : r.status === "lost" ? '<span class="badge warn">中断</span>' : r.status === "incomplete" ? `<span class="badge warn" title="${esc(r.error || "")}">不完整</span>` : `<span class="badge">${esc(r.status)}</span>`;
  const head = compact
    ? "<th>原生轨迹 vs RTK</th><th>RTK 优化后</th>"
    : `<th title="前端里程计（RViz 红线）后轴 3D 误差，全程、无对齐，随里程累积。融合轮的前端启用 INS 姿态辅助（仅横滚/俯仰弱约束，σ0.3°，不碰位置/速度/偏航），无融合轮为纯 LIO——两种模式的原生漂移不可直接对比">原生轨迹 vs RTK（全程）</th><th title="RTK/INS 融合结果（蓝线）后轴 3D 误差">RTK 优化后（中位 / P95 / 最大）</th><th title="5–20 m 地面同点平面厚度，原生 → 优化后">地面厚度 5–20 m</th><th title="同一灯柱在不同地图块中的轴线 XY 分离">灯柱多期一致性</th>`;
  return `<table><thead><tr><th>轮次</th><th>模式</th><th>状态</th>${compact ? "" : "<th>数据源</th><th>标定</th>"}<th>开始</th>${head}</tr></thead><tbody>
  ${runs.map((r) => { const c = metricCells(r); return `<tr class="click ${S.runDetail?.round === r.round ? "sel" : ""}" data-act="open-run" data-round="${esc(r.round)}"><td class="mono">${esc(r.round)}</td><td><span class="badge">${esc(modeLabel(r))}</span></td><td>${badge(r)}</td>${compact ? "" : `<td class="small muted ellipsis" style="max-width:180px" title="${esc(r.dataset)}">${esc(baseName(r.dataset))}</td><td class="small muted">${esc(r.calibration ? baseName(r.calibration.replace(/\/calibration\.json$/, "")) : "yaml")}</td>`}<td class="small nowrap">${fmtT(r.start)}</td><td class="small">${c.native}</td><td class="small">${c.opt}</td>${compact ? "" : `<td class="small">${c.map}</td><td class="small">${c.pole}</td>`}</tr>`; }).join("")}</tbody></table>`;
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
  </div>${assessmentCard()}`;
}
// ------------------------------------------------------------------ time-window selector
function winEffective(w, durS) {
  // Returns {start_s, duration_s} or null for "all". durS may be null/0 -> unknown; validation deferred to backend.
  if (!w || w.mode === "auto") return null;
  if (w.mode === "head") {
    const d = Number(w.head_dur);
    if (!(d > 0)) return null;
    return { start_s: 0, duration_s: d };
  }
  const s = Number(w.start) || 0;
  const e = w.end === "" || w.end === null ? NaN : Number(w.end);
  if (isFinite(s) && isFinite(e) && e > s) return { start_s: s, duration_s: e - s };
  if (isFinite(s) && !isFinite(e)) return { start_s: s, duration_s: 0 }; // 0 = "until end"
  return null;
}
function winLabel(w, durS) {
  const eff = winEffective(w, durS);
  if (!eff) return "自动（按运动窗口，跳过静止起步段，保留 8 s 静止初始化 IMU）";
  if (w.mode === "head") return `仅开头 ${eff.duration_s} s`;
  if (!(eff.duration_s > 0)) return `从 ${eff.start_s} s 起（到结束）`;
  return `${eff.start_s} – ${(eff.start_s + eff.duration_s).toFixed(0)} s（${eff.duration_s} s）`;
}
function winProblem(w, durS) {
  // A typed-but-unusable window must not silently fall back to "auto".
  if (!w || w.mode === "auto") return "";
  if (w.mode === "head") return Number(w.head_dur) > 0 ? (durS && Number(w.head_dur) > durS ? "超过数据总长，将用到结尾" : "") : "请填写开头时长";
  const s = Number(w.start) || 0;
  const e = w.end === "" || w.end == null ? NaN : Number(w.end);
  if (durS && s >= durS) return "起始超过数据总长";
  if (isFinite(e) && e <= s) return "结束需大于起始，当前按自动处理";
  return "";
}
function windowWidget(stateKey, title, note = "") {
  const w = S[stateKey];
  const dur = S.dataset?.duration_s || 0;
  const mode = w.mode || "auto";
  const problem = winProblem(w, dur);
  const hint = `<div class="small muted" style="grid-column:1/-1">当前：<b>${esc(winLabel(w, dur))}</b>${dur ? ` · 数据总长 ${Math.round(dur)} s` : ""}${problem ? ` · <b class="warn">${esc(problem)}</b>` : ""}${mode !== "auto" && note ? `<div>${esc(note)}</div>` : ""}</div>`;
  const setMode = (m) => `data-act="win-mode" data-key="${stateKey}" data-mode="${m}"`;
  const fi = (key, val, step, ph) => `<input type="number" step="${step}" min="0" data-act="win-field" data-key="${stateKey}" data-field="${key}" value="${val == null ? "" : esc(val)}" placeholder="${ph || ""}">`;
  return `<section class="ax-win"><header><b>${esc(title)}</b><span class="small muted">评估/标定取哪一段时间</span></header>
    <div class="ax-win-seg">
      <button class="${mode === "auto" ? "active" : ""}" ${setMode("auto")}>自动（推荐）</button>
      <button class="${mode === "head" ? "active" : ""}" ${setMode("head")}>开头 N 秒</button>
      <button class="${mode === "custom" ? "active" : ""}" ${setMode("custom")}>自定义区间</button>
    </div>
    <div class="ax-win-grid">
      ${mode === "head" ? `<div class="field"><label>开头使用时长 (s)</label>${fi("head_dur", w.head_dur, "1", "60")}</div>` : ""}
      ${mode === "custom" ? `<div class="field"><label>起始 (s)</label>${fi("start", w.start, "0.1", "0")}</div>
        <div class="field"><label>结束 (s，空=到结尾)</label>${fi("end", w.end, "0.1", "留空")}</div>` : ""}
      ${hint}
    </div></section>`;
}
// ------------------------------------------------------------------ data assessment (数据评估)
const AX_STATUS = { ok: "正常", warn: "关注", bad: "异常", info: "参考", na: "未检" };
async function loadAssessment(force = false) {
  const d = S.dataset;
  if (!d || d.error) { S.assess = null; return; }
  if (!force && S.assess && S.assess.path === d.path) return;
  const w = winEffective(S.winAssess, d.duration_s);
  const qs = new URLSearchParams({ path: d.path });
  if (w) { qs.set("start", String(w.start_s)); qs.set("duration", w.duration_s > 0 ? String(w.duration_s) : "0"); }
  try { const res = await api(`/api/datasets/assessment?${qs.toString()}`); S.assess = { ...res, path: d.path, _w: w }; }
  catch (e) { S.assess = { path: d.path, result: null, error: e.message || String(e) }; }
}
function verdictBadge(v, prefix = "评估") {
  const m = { ok: ["ok", "正常"], warn: ["warn", "关注"], bad: ["bad", "异常"] }[v] || ["", "未评估"];
  return `<span class="badge ${m[0]}">${prefix ? prefix + " · " : ""}${m[1]}</span>`;
}
function assessBadgeHome() {
  const s = S.assessSummary;
  if (!s || !s.verdict) return `<span class="badge" title="在“数据源”页运行数据评估，或由标定流水线自动完成">未评估</span>`;
  const n = s.alerts || 0;
  return `<span class="badge ${s.verdict}" title="${esc(s.title || "")}">评估 · ${AX_STATUS[s.verdict]}${n ? ` · ${n} 项不可标定补救` : ""}</span>`;
}
function axNum(x, unit) {
  if (x == null || Number.isNaN(x)) return "—";
  const a = Math.abs(x);
  const s = (a >= 100 ? x.toFixed(0) : a >= 10 ? x.toFixed(1) : x.toFixed(2)).replace(/\.0+$/, "").replace(/(\.\d*[1-9])0+$/, "$1");
  return unit ? `${s} ${unit}` : s;
}
function axThresholds(m) {
  const t = m.thresholds;
  if (!t) return m.limits ? `<span class="ax-lim" title="${esc(m.limits)}">${esc(m.limits)}</span>` : `<span class="ax-lim muted">—</span>`;
  const c = t.higher ? "≥" : "≤";
  const same = t.warn === t.ok;
  return `<span class="ax-tk ok" title="正常">${c} ${axNum(t.ok, m.unit)}</span>${same ? "" : `<span class="ax-tk warn" title="关注">${c} ${axNum(t.warn, m.unit)}</span>`}`;
}
function axBar(m) {
  const t = m.thresholds;
  if (!t || typeof m.value !== "number" || !Number.isFinite(m.value)) return "";
  const lo = Math.min(t.ok, t.warn), hi = Math.max(t.ok, t.warn);
  if (hi <= 0) return "";
  const span = Math.max(hi * 1.6, m.value * 1.15, 1e-9);
  const pct = (x) => Math.max(0, Math.min(100, (100 * x) / span));
  const segs = t.higher ? [["bad", 0, pct(lo)], ["warn", pct(lo), pct(hi)], ["ok", pct(hi), 100]] : [["ok", 0, pct(lo)], ["warn", pct(lo), pct(hi)], ["bad", pct(hi), 100]];
  return `<div class="ax-bar" aria-hidden="true">${segs.map(([k, a, b]) => `<i class="${k}" style="left:${a}%;width:${Math.max(0, b - a)}%"></i>`).join("")}<b style="left:${pct(m.value)}%"></b></div>`;
}
function axGauge(counts, verdict, total) {
  const parts = [["ok", counts.ok || 0], ["warn", counts.warn || 0], ["bad", counts.bad || 0], ["info", (counts.info || 0) + (counts.na || 0)]];
  const n = Math.max(1, parts.reduce((a, [, v]) => a + v, 0));
  const r = 40, C = 2 * Math.PI * r;
  let off = 0;
  const arcs = parts.filter(([, v]) => v > 0).map(([k, v]) => {
    const len = (C * v) / n;
    const s = `<circle class="${k}" r="${r}" cx="50" cy="50" stroke-dasharray="${len - 2} ${C - len + 2}" stroke-dashoffset="${-off}"/>`;
    off += len; return s;
  }).join("");
  return `<svg class="ax-gauge ${verdict || ""}" viewBox="0 0 100 100"><circle class="track" r="${r}" cx="50" cy="50"/>${arcs}<text x="50" y="47" class="v">${AX_STATUS[verdict] || "—"}</text><text x="50" y="63" class="n">${total} 项</text></svg>`;
}
function axValue(display) {
  const d = String(display ?? "");
  const i = d.indexOf("（");
  if (d.length > 26 && i > 0 && d.endsWith("）")) return `<span class="mono">${esc(d.slice(0, i))}</span><div class="ax-vd">${esc(d.slice(i + 1, -1))}</div>`;
  if (d.length > 40 && d.includes(" · ")) { const parts = d.split(" · "); return `<span class="mono">${esc(parts[0])}</span><div class="ax-vd">${esc(parts.slice(1).join(" · "))}</div>`; }
  return `<span class="mono">${esc(d)}</span>`;
}
const AX_FIX_SHORT = { "标定/建图自动补偿": "自动补偿", "需运行离线标定阶段": "离线标定阶段", "需调整采集或回放流程": "采集/回放流程", "标定无法补救": "标定无法补救", "调整回放设置（分段 / 倍速）": "回放设置" };
function axMetricRow(m) {
  const tip = [m.note, m.basis ? `依据：${m.basis}` : ""].filter(Boolean).join("\n");
  const fix = m.fix_label ? `<div class="ax-fix ${m.alert ? "alert" : ""}" title="${esc(m.fix_label)}">${m.alert ? "⚠ " : ""}${esc(AX_FIX_SHORT[m.fix_label] || m.fix_label)}</div>` : "";
  const plain = m.plain ? `<div class="ax-plain">${esc(m.plain)}</div>` : "";
  return `<tr class="ax-r ${m.status}" title="${esc(tip)}"><td class="ax-l"><span class="dot ${m.status}"></span><div class="ax-lb">${esc(m.label)}${plain}</div></td><td class="ax-v">${axValue(m.display)}${axBar(m)}</td><td class="ax-th">${axThresholds(m)}</td><td class="ax-s"><span class="badge sm ${m.status === "info" || m.status === "na" ? "" : m.status}">${AX_STATUS[m.status] || m.status}</span>${fix}</td></tr>`;
}
function axGroup(g) {
  const c = { ok: 0, warn: 0, bad: 0, other: 0 };
  g.metrics.forEach((m) => { if (m.status in c) c[m.status] += 1; else c.other += 1; });
  const n = g.metrics.length, w = c.warn + c.bad;
  const sub = w ? `${n} 项 · ${w} 项需关注` : `${n} 项`;
  return `<section class="ax-group" id="ag-${g.id}"><header class="ax-gh"><span class="dot ${g.status}"></span><h3>${esc(g.label)}</h3><span class="ax-gc">${sub}</span><span class="badge sm ${g.status === "ok" ? "ok" : g.status}">${AX_STATUS[g.status] || "—"}</span></header><p class="ax-blurb">${esc(g.blurb || "")}</p>
    <table class="ax-t"><thead><tr><th>指标</th><th>数值</th><th title="正常 / 关注 的判定边界；其余为异常">阈值</th><th>判定</th></tr></thead><tbody>${g.metrics.map(axMetricRow).join("")}</tbody></table>
    <details class="ax-more"><summary>说明与依据</summary><dl>${g.metrics.map((m) => `<dt>${esc(m.label)}</dt><dd>${esc(m.note || "")}${m.basis ? `<span class="muted"> 依据：${esc(m.basis)}</span>` : ""}</dd>`).join("")}</dl></details></section>`;
}
function assessmentCard() {
  const d = S.dataset;
  if (!d || d.error) return "";
  const a = S.assess && S.assess.path === d.path ? S.assess : null;
  const r = a && a.result;
  const running = S.job && S.job.status === "running";
  const mine = running && (S.job.kind === "assess" || (S.job.kind === "pipeline" && /数据评估/.test(S.job.steps?.[S.job.current]?.title || "")));
  const btn = mine ? `<button class="btn sm" data-act="drawer-job"><span class="dot run"></span>&nbsp;评估进行中 · 查看进度</button>`
    : `<button class="btn sm ${r ? "" : "primary"}" data-act="assess" ${running ? "disabled" : ""}>${r ? "重新评估" : "开始评估"}</button>`;
  const head = `<div class="card-h"><h2>数据评估 ${r ? verdictBadge(r.verdict, "") : ""}</h2><div class="row">${btn}</div></div>`;
  if (!r) {
    return `<div class="card ax" style="margin-top:16px">${head}
      ${windowWidget("winAssess", "评估时间窗口")}
      <div class="ax-hero ax-idle">${axGauge({}, null, 42)}<div class="ax-sum"><div class="ax-title">一次并行流式读取，给出这份录制能达到的精度上限</div>
        <div class="ax-text">按本项目在 truck29 / truck05 上遇到的全部数据问题逐项检查：完整性、时间同步、RTK/INS 参考自洽性、雷达几何与静止段外参、相机时间戳与图像内容、行驶工况与资源。每项都有阈值，并标明是标定/建图流程能自动补偿，还是<b>标定无法补救</b>。</div>
        <div class="ax-legend"><span><i class="dot ok"></i>7 个检查组 · 42 项指标</span><span><i class="dot"></i>truck29（120 s）≈ 2 s · truck05（720 s，25 GB）≈ 11 s</span><span><i class="dot accent"></i>标定流水线起步时自动执行，已有报告则跳过</span></div></div></div>
      ${a && a.error ? `<div class="callout bad">${esc(a.error)}</div>` : ""}</div>`;
  }
  const c = r.counts || {};
  const o = r.outlook || {};
  const total = (r.groups || []).reduce((n, g) => n + g.metrics.length, 0);
  const ref = r.reference || {};
  const calName = r.calibration ? r.calibration.split("/").slice(-2, -1)[0] : "";
  const facts = [["时长", fmtDur(r.dataset?.duration_s || 0)], ["行驶", `${Math.round(ref.path_m || 0)} m`], ["消息", (r.messages || 0).toLocaleString()], ["数据量", `${((r.bytes || 0) / 1e9).toFixed(1)} GB`],
    ["用时", `${r.elapsed_s} s${r.workers > 1 ? ` · ${r.workers} 进程` : ""}`], ["外参检查", calName || "未提供标定"], ["生成", fmtT(r.generated)]];
  const winCard = windowWidget("winAssess", "评估时间窗口");
  const hero = `<div class="ax-hero ${r.verdict}">${axGauge(c, r.verdict, total)}
    <div class="ax-sum"><div class="ax-title">${esc(o.title || "")}</div><div class="ax-text">${esc(o.text || "")}</div>
      <div class="ax-legend"><span><i class="dot ok"></i>正常 <b>${c.ok || 0}</b></span><span><i class="dot warn"></i>关注 <b>${c.warn || 0}</b></span><span><i class="dot bad"></i>异常 <b>${c.bad || 0}</b></span><span><i class="dot"></i>参考 <b>${(c.info || 0) + (c.na || 0)}</b></span><span class="ax-alertn ${(r.alerts || []).length ? "on" : ""}">⚠ 不可标定补救 <b>${(r.alerts || []).length}</b></span></div></div>
    <dl class="ax-facts">${facts.map(([k, v]) => `<dt>${k}</dt><dd>${esc(String(v))}</dd>`).join("")}</dl></div>`;
  const strip = `<nav class="ax-strip">${(r.groups || []).map((g) => { const w = g.metrics.filter((m) => m.status === "warn" || m.status === "bad").length; return `<button class="ax-chip ${g.status}" data-act="ax-jump" data-target="ag-${g.id}"><span class="dot ${g.status}"></span>${esc(g.label)}<em>${w ? `${w} 项` : "✓"}</em></button>`; }).join("")}</nav>`;
  const alerts = (r.alerts || []).length ? `<div class="callout ${r.alerts.some((x) => x.status === "bad") ? "bad" : "warn"} ax-alerts"><div class="ax-ah"><b>⚠ 标定无法补救 · ${r.alerts.length} 项</b><span>来自参考系统、传感器同步、录制流程或场景本身；外参 / 时间偏移微调不改变这些数值，它们直接决定精度下限</span></div>
    ${r.alerts.map((x, i) => `<div class="ax-ai"><span class="ax-no">${String(i + 1).padStart(2, "0")}</span><div class="ax-ab"><div class="ax-at"><b>${esc(x.label)}</b><span class="ax-ag">${esc(x.group)}</span><span class="mono ax-av">${esc(x.display)}</span></div><div class="ax-ar">${esc(x.note || "")}</div></div><span class="badge sm ${x.status === "bad" ? "bad" : "warn"}">${esc(x.fix_label)}</span></div>`).join("")}</div>` : "";
  const fixable = (r.fixable || []).length ? `<div class="callout ax-fixable"><div class="ax-ah"><b>可由标定流程 / 建图自检补偿 · ${r.fixable.length} 项</b><span>运行 LiDAR 标定流水线（后轴对齐 → 地面精修 → LIO → 行驶段精修）或依赖建图起步自检 / 合帧容差即可，不需要重新采集</span></div>
    ${r.fixable.map((x) => `<div class="ax-fi"><span class="dot ${x.status}"></span><span><b>${esc(x.label)}</b><span class="ax-ag">${esc(x.group)}</span></span><span class="mono ax-av">${esc(x.display)}</span><span class="badge sm accent">${esc(x.fix_label)}</span></div>`).join("")}</div>` : "";
  const groups = `<div class="ax-grid">${(r.groups || []).map(axGroup).join("")}</div>`;
  const cams = (r.camera_samples || []).filter((s) => s.decoded && s.thumb);
  const thumbs = cams.length ? `<section class="ax-cams"><header class="ax-gh"><span class="dot ok"></span><h3>相机样例</h3><span class="ax-gc">均匀抽取 ${cams.length} 个时刻 · 时刻 / 亮度均值 / 纹理（过曝 >0.5 % 时标注）</span></header><div class="ax-thumbs">${cams.map((s) => `<figure class="ax-thumb"><img src="${s.thumb}" alt=""><figcaption class="mono" title="时刻 ${s.t_s} s · 亮度均值 ${Math.round(s.mean)} · 过曝 ${s.clipped_pct.toFixed(1)} % · 纹理 ${Math.round(s.texture)}">${Math.round(s.t_s)} s · ${Math.round(s.mean)}${s.clipped_pct > 0.5 ? ` · ${s.clipped_pct.toFixed(1)} %` : ""} · ${Math.round(s.texture)}</figcaption></figure>`).join("")}</div></section>` : "";
  const foot = `<div class="ax-foot small muted">评估范围 ${r.window ? `${r.window[0]}–${r.window[0] + r.window[1]} s` : "全程"} · 静止起步 ${ref.static_start_s != null ? ref.static_start_s.toFixed(1) : "—"} s · 行驶 ${Math.round(ref.moving_s || 0)} s · 报告 <span class="mono">${esc(a.file || "")}</span></div>`;
  return `<div class="card ax" style="margin-top:16px">${head}${winCard}${hero}${strip}${alerts}${fixable}${groups}${thumbs}${foot}</div>`;
}
function fsHtml() {
  const f = S.fs; if (!f) return `<div class="e dim">输入路径后点击“打开”</div>`;
  const up = `<div class="e" data-act="fs-open" data-path="${esc(f.parent)}"><span>↰</span><span class="n muted">上一级</span></div>`;
  const here = f.kind ? `<div class="e" data-act="select-dataset" data-path="${esc(f.path)}"><span>✓</span><span class="n accent">选用当前目录 ${esc(baseName(f.path))}</span><span class="badge ${f.kind === "recording" ? "ok" : ""}">${f.kind === "recording" ? "录制" : "bag"}</span></div>` : "";
  return up + here + f.entries.filter((e) => e.dir).map((e) => `<div class="e" data-act="${e.kind ? "select-dataset" : "fs-open"}" data-path="${esc(e.path)}"><span>${e.kind ? "▣" : "▸"}</span><span class="n">${esc(e.name)}</span>${e.kind ? `<span class="badge ${e.kind === "recording" ? "ok" : ""}">${e.kind === "recording" ? "录制" : "bag"}</span>` : ""}</div>`).join("");
}

function currentAxleLimits(vehicleId = vehicle()?.id) {
  const saved = S.state?.calibration?.axle_limits_by_vehicle?.[vehicleId] || {};
  return Object.fromEntries(Object.entries(S.cal?.axle_limits || {}).map(([key, spec]) => [key, saved[key] ?? spec.default]));
}
function axleLimitsHtml(vehicleId = vehicle()?.id) {
  const specs = S.cal?.axle_limits || {}, values = currentAxleLimits(vehicleId);
  return `<details open style="margin-top:12px"><summary>后轴标定质量门限 · ${esc(vehicleId || "当前车辆")}</summary>
    <div class="form" style="margin-top:10px">${Object.entries(specs).map(([key, spec]) => `<div class="field"><label>${esc(spec.label)}</label><input type="number" min="0" step="${spec.step}" data-act="axle-limit" data-vehicle="${esc(vehicleId)}" data-key="${key}" value="${esc(values[key])}" title="${key}；默认 ${spec.default}"></div>`).join("")}</div>
    <div class="small muted">上限用于决定是否拒绝结果；警告阈值用于标记不一致。修改只影响之后启动或续跑的后轴阶段，已完成阶段保持原结果。门限按车辆保存并写入每次标定记录。</div>
    <button class="btn sm" data-act="reset-axle-limits" data-vehicle="${esc(vehicleId)}">恢复默认门限</button></details>`;
}
function pageCalib() {
  const v = vehicle(); const cal = S.cal || { versions: [], active: {}, sources: [] };
  const versions = cal.versions.filter((x) => v && x.mine === v.mine && x.truck === v.truck).sort((a, b) => (a.version > b.version ? -1 : 1));
  const d = S.dataset; const running = S.job && S.job.status === "running";
  const c = S.state.calibration; const f = S.calibForm;
  const stageOpts = Object.entries(S.stages).sort((a, b) => a[1].order - b[1].order).filter(([k, st]) => k !== "import" && !st.internal);
  const info = S.stages[f.stage] || {};
  const needs = info.needs || [];
  const modeForRun = info.mapping;
  const runOpts = S.runs.filter((r) => r.fusion_dir && r.status === "done" && (!modeForRun || modeForRun === "any" || r.mode === modeForRun) && (modeForRun !== "livo" || (r.camera || "midrange") === (c.camera || "midrange")));
  const validationSelected = Array.isArray(c.validation_run) ? c.validation_run : c.validation_run ? [c.validation_run] : [];
  const validationOpts = S.runs.filter((r) => r.fusion_dir && r.has_archive && r.status === "done" && r.mode === "livo" && (r.camera || "midrange") === (c.camera || "midrange") && r.vehicle === v?.id);
  const pipelineParent = selectedParent();
  const cameraReady = hasCamera(d, c.camera || "midrange");
  const parents = versions.map((x) => `<option value="${esc(x.id)}" ${pipelineParent === x.id ? "selected" : ""}>${esc(x.version)} · ${esc(x.stage_label)} · ${cameraLabel(x.camera)}${x.active ? " ★" : ""}</option>`).join("");
  return `<h1 style="margin-bottom:6px">标定</h1><div class="muted" style="margin-bottom:18px">每辆车的标定按 <code>config/carlibration/&lt;矿山&gt;/&lt;车辆&gt;/vNNN_&lt;阶段&gt;/</code> 版本化保存；每个阶段都以父版本为输入生成新版本，绝不覆盖。建图按当前录制与相机匹配版本，也可显式指定。</div>
  <div class="card"><div class="card-h"><h2>车辆 &nbsp;<select data-act="set-vehicle">${S.vehicles.map((x) => `<option value="${x.id}" ${v && x.id === v.id ? "selected" : ""}>${esc(x.mine)} / ${esc(x.truck)}</option>`).join("")}</select></h2>
    <div class="row">${v?.active ? `<span class="badge dark">激活：${esc(v.active.split("/").pop())}</span>` : '<span class="badge bad">无激活版本</span>'}<button class="btn sm" data-act="toggle-import">${S.showImport ? "收起" : "新建车辆 / 导入出厂标定"}</button></div></div>
    ${S.showImport || !S.vehicles.length ? importForm(c) : ""}
    ${stepperHtml(cameraVehicle(v, c.camera))}
    ${windowWidget("winCalib", "标定时间窗口", "标定只使用窗口内的数据；窗口从行驶中开始时，建图轮次自动从窗口前最近的静止处起播（IMU 需静止初始化），窗口内没有静止段时地面静止帧取窗口前的静止段。")}
    <div class="form" style="margin-top:12px">${cameraSelector("calibration", c.camera)}<div class="small muted">中距与鱼眼分别保存时间、内外参和验证状态。切换相机后先导入该相机参数；相机标定、投影复核和验证建图使用同一个选择。</div></div>
    <div class="grid g2" style="margin-top:8px">
      <div class="stat" style="padding:14px"><div class="row" style="justify-content:space-between"><div><b>LiDAR 标定流水线</b><div class="small muted">后轴对齐（RTK/INS 位姿，参考雷达→后轴，整机修正）→ 静止地面精修 → LIO 建图（保存地图）→ 行驶段 6-DoF 精修 → 校验${c.auto_activate ? " → 激活" : ""}</div><div class="small muted">180 s 窗口实测 7–10 分钟（2–3 轮 LIO 回放，2.5 倍速；RTK 关联时间已自洽时省掉探测轮）· 需要 LiDAR/IMU/INS 数据源 · 父版本可在下方选择（任意版本）</div></div><button class="btn primary" data-act="pipeline" data-kind="lidar" ${running || !v?.active || !d?.lio_ready ? "disabled" : ""}>运行</button></div></div>
      <div class="stat" style="padding:14px"><div class="row" style="justify-content:space-between"><div><b>相机标定流水线</b><div class="small muted">${c.camera === "fisheye" ? "鱼眼时间偏移 → LIVO 基线 → 灯柱偏航复核 → 多帧内参诊断 → 新基线建图 → 旋转/时间/内参/残余畸变联合标定 → 独立多包路段验证 → LIVO 验证建图 → 闭环判定" : "相机时间偏移 → LIVO 建图 A（基线）→ 偏航复核（俯仰不动）→ 俯仰闭环（标准 ±0.05° 扫描；快速沿用同相机实测斜率）→ 验证建图 → 闭环判定 → 校验"}${c.auto_activate ? " → 通过则激活" : ""}</div><div class="small muted">${c.camera === "fisheye" ? "鱼眼完整流程使用下方所选独立验证轮次；不选仍可诊断和试验建图，但不能采纳激活。各阶段及独立验证均使用多数据包、多帧证据" : "默认每轮 120 s（受所选窗口限制），按运动量选窗；俯仰不可观测时保留原激活版本；未通过闭环不会激活"}</div></div><button class="btn primary" data-act="pipeline" data-kind="camera" ${running || !v?.active || !cameraReady ? "disabled" : ""}>运行</button></div></div>
      <div class="stat" style="padding:14px"><div class="row" style="justify-content:space-between"><div><b>相机外参闭环微调（日常，最快）</b><div class="small muted">以当前激活版本为起点：<b>复用最近一次同版本 LIVO 轮次</b>作为零俯仰点（无需重跑）→ 用已知斜率算出过零点 → 闭环版本 → 一轮 LIVO 验证 → 闭环判定${c.auto_activate ? " → 通过则激活" : ""}</div><div class="small muted">快速 ≈4–5 min（1 次回放）· 标准 ≈15 min（4 次回放）· 复用要求同窗口、同版本且已评估，时长至少覆盖本次窗口的 90%</div></div><button class="btn primary" data-act="pipeline" data-kind="camloop" ${running || !v?.active || !cameraReady ? "disabled" : ""}>运行</button></div></div>
      <div class="stat" style="padding:14px"><div class="row" style="justify-content:space-between"><div><b>相机光度标定（内参复核 + 外参/时间精修 + 暗角）</b><div class="small muted">以所选父版本为起点：LIVO 基线建图 → 多视角光度一致性联合复核旋转、时间、内参与残余畸变 → 留出数据及所选独立路段验证 → LIVO 验证建图 → 原生轨迹闭环判定。中距和鱼眼均可微调；只有通过验证的候选参数才写入新版本${c.auto_activate ? " → 通过则激活" : ""}</div><div class="small muted">默认 2 次 120 s 回放 + 光度计算；可复用下方另一段已有 LIVO 扫描存档检验固定候选。鱼眼自动激活还要求多阶段及独立多包验证全部通过</div></div><button class="btn primary" data-act="pipeline" data-kind="camphoto" ${running || !v?.active || !cameraReady ? "disabled" : ""}>运行</button></div></div>
    </div>
    <div class="field" style="margin-top:12px"><label>光度标定的独立路段验证（可多选，流水线和单步共用）</label><select multiple size="4" data-act="photo-validation"><option value="" ${!validationSelected.length ? "selected" : ""}>不指定：仅生成诊断候选，鱼眼不可激活</option>${validationSelected.filter((value) => !validationOpts.some((r) => r.fusion_dir === value)).map((value) => `<option value="${esc(value)}" selected>所选轮次已不可用，请取消并重新选择</option>`).join("")}${validationOpts.map((r) => `<option value="${esc(r.fusion_dir)}" ${validationSelected.includes(r.fusion_dir) ? "selected" : ""}>${esc(r.round)} · ${esc(baseName(r.dataset))} · ${cameraLabel(r.camera)}</option>`).join("")}</select><div class="small muted">可选择同车、同相机的其他路段或录制，按 Ctrl / Command 多选。流水线会核对已有轮次的相机模型，参数改变时按原录制窗口重建验证基线。验证数据不参与拟合；鱼眼拟合及每个验证轮次均需 ≥2 个相机数据包、≥50 帧、≥3 个视角，另需时间、偏航、内参诊断、光度联合标定及当前版本的 LIVO 闭环通过。单帧或缺独立验证只能试验建图，不能激活。</div></div>
    <div class="grid g2" style="margin-top:10px;align-items:end"><div class="field"><label>流水线 / 单步 / 车身过滤拼接的父版本（可选任意版本）</label><select data-act="calib-field" data-key="parent">${parents || "<option>（无）</option>"}</select><div class="small muted">当前选择：<b>${esc(pipelineParent || "—")}</b>；激活版本：${esc(v?.active || "—")}。自动化流水线会先把当前车辆的 body_filter 写入该父版本，再开始各标定阶段。</div></div><div class="small muted">数据源：<b>${esc(d?.name || "未选择")}</b> ${datasetBadges(d)}</div></div>
    <div class="row" style="gap:18px;flex-wrap:wrap;margin-top:4px"><label class="check small">${`<input type="checkbox" data-act="toggle" data-section="calibration" data-key="auto_activate" ${c.auto_activate ? "checked" : ""}><span class="switch"></span><span>流水线完成后自动激活（相机需闭环通过，鱼眼还需多阶段及独立多包验证）</span>`}</label><label class="check small">${`<input type="checkbox" data-act="toggle" data-section="calibration" data-key="fast" ${c.fast !== false ? "checked" : ""}><span class="switch"></span><span>快速模式（沿用本相机实测俯仰斜率只测零点；本相机无斜率或单点拟合被拒时自动转标准扫描；取消勾选＝标准扫描）</span>`}</label><label class="check small">${`<input type="checkbox" data-act="toggle" data-section="calibration" data-key="keep_maps" ${c.keep_maps === true ? "checked" : ""}><span class="switch"></span><span>保留标定轮次的地图与扫描存档（每轮 1–4 GB；默认关闭＝流水线成功后删除 PCD/scans.bin，轨迹与评估保留）</span>`}</label></div>
  </div>
  <div class="card" style="margin-top:16px">${axleLimitsHtml()}</div>
  <div class="grid" style="grid-template-columns: 1fr 380px; margin-top:16px">
    <div class="card"><div class="card-h"><h2>版本（${versions.length}）</h2><span class="small muted">点击查看详情 · ★ 为激活版本</span></div>
      ${versions.length ? `<table><thead><tr><th>版本</th><th>阶段</th><th>父版本</th><th>创建</th><th>说明</th></tr></thead><tbody>${versions.map((x) => `<tr class="click ${S.versionDetail?.id === x.id ? "sel" : ""}" data-act="open-version" data-id="${esc(x.id)}"><td class="mono nowrap">${x.active ? "★ " : ""}${esc(x.version)}<div class="small muted">${cameraLabel(x.camera)}</div></td><td><span class="badge ${x.advisory ? "warn" : ""}">${esc(x.stage_label)}</span>${x.closed_loop ? `<div class="small"><span class="badge ${x.closed_loop === "pass" ? "ok" : "bad"}" title="LIVO 闭环验证（原生轨迹 A/B）">闭环 ${x.closed_loop === "pass" ? "通过" : "未通过"}</span></div>` : x.advisory || x.stage?.startsWith("cam") ? '<div class="small muted">未闭环验证</div>' : ""}</td><td class="mono small muted">${esc(x.parent ? x.parent.split("/").pop() : "—")}${x.parent_missing ? ' <span class="bad">(已删)</span>' : ""}</td><td class="small nowrap">${fmtT(x.created)}</td><td class="small muted" style="max-width:360px">${esc(x.note || "")}</td></tr>`).join("")}</tbody></table>` : '<div class="empty">该车辆还没有标定版本，请先导入出厂标定</div>'}</div>
    <div class="card"><div class="card-h"><h2>单步执行（高级）</h2></div>
      <div class="stack">
        <div class="field"><label>阶段</label><select data-act="calib-stage">${stageOpts.map(([k, s]) => `<option value="${k}" ${f.stage === k ? "selected" : ""}>${s.order}. ${esc(s.label)}</option>`).join("")}</select>
          <div class="small muted">输入：${esc(info.input || "")}<br>输出：${esc(info.output || "")}${info.advisory ? '<br><span class="warn">结果仅供参考，需人工确认叠加图后再激活</span>' : ""}</div></div>
        <div class="field"><label>父版本</label><select data-act="calib-field" data-key="parent">${parents || "<option>（无）</option>"}</select></div>
        ${needs.includes("dataset") ? `<div class="field"><label>数据源${needs.includes("camera") ? "（需含相机）" : ""}</label><input type="text" value="${esc(d?.path || "")}" disabled></div>` : ""}
        ${needs.includes("run") ? `<div class="field"><label>建图结果（${modeForRun === "livo" ? "LIVO" : "LIO"} 轮次，需保存了地图）</label><select data-act="calib-field" data-key="run">${runOpts.map((r) => `<option value="${esc(r.fusion_dir)}" ${f.run === r.fusion_dir ? "selected" : ""}>${esc(r.round)} · ${fmtT(r.start)}</option>`).join("") || "<option value=''>（没有可用轮次）</option>"}</select></div>` : ""}
        ${needs.includes("run_dir") ? `<div class="field"><label>建图轮次（含 Log/extrinsic_precheck.json）</label><select data-act="calib-field" data-key="run_dir">${S.runs.filter((r) => r.precheck).map((r) => `<option value="${esc(r.path)}" ${f.run_dir === r.path ? "selected" : ""}>${esc(r.round)}</option>`).join("") || "<option value=''>（没有自检记录）</option>"}</select></div>` : ""}
        ${f.stage === "manual" ? `<div class="form"><div class="field"><label>相机时间偏移 (s)</label><input type="number" step="0.001" data-act="calib-field" data-key="camera_offset" value="${esc(f.camera_offset ?? "")}"></div><div class="field"><label>LiDAR 时间偏移 (s)</label><input type="number" step="0.001" data-act="calib-field" data-key="lidar_offset" value="${esc(f.lidar_offset ?? "")}"></div><div class="field wide"><label>相机旋转修正 rx ry rz (deg, 相机系)</label><input type="text" data-act="calib-field" data-key="camera_rot_delta" value="${esc(f.camera_rot_delta ?? "")}" placeholder="0 0 0"></div></div>` : ""}
        ${f.stage === "camlidar" ? `<div class="field"><label>检查的图像帧数（其中含灯柱的帧参与偏航估计）</label><input type="number" data-act="calib-field" data-key="frames" value="${esc(f.frames ?? 30)}"></div>` : ""}
        <div class="field"><label>说明</label><input type="text" data-act="calib-field" data-key="note" value="${esc(f.note ?? "")}" placeholder="可选备注，写入版本记录"></div>
        <label class="check"><input type="checkbox" data-act="calib-field" data-key="activate" ${c.camera === "fisheye" ? "disabled" : f.activate ? "checked" : ""}><span class="switch"></span><span>${c.camera === "fisheye" ? "鱼眼单步仅生成候选，完成独立验收后再激活" : "完成后激活"}</span></label>
        <button class="btn primary" data-act="run-stage" ${running || !versions.length ? "disabled" : ""}>执行 ${esc(info.label || "")}</button>
      </div></div>
  </div>`;
}
function importForm(c) {
  const sources = (S.cal?.sources || []);
  return `<div class="callout" style="margin-bottom:14px"><b>导入出厂标定</b>：把厂家/感知标定 JSON（厂家原始导出 <span class="mono">lidar[] + tf_lidar_to_camera</span> 或 schema-5 文件）规范化为 v001_import（矩阵正交化、相机→后轴外参、相机内参、默认时间偏移）。原始出厂外参导入后先运行 LiDAR 标定流水线（第一步“后轴对齐”自动修正参考雷达→后轴）。新矿山/新车直接填写名称即可创建。
    <div class="form" style="margin-top:10px">
      <div class="field"><label>矿山</label><input type="text" id="imp-mine" value="${esc(S.state.vehicle.mine)}"></div>
      <div class="field"><label>车辆编号</label><input type="text" id="imp-truck" value="${esc(S.state.vehicle.truck)}"></div>
      <div class="field wide"><label>出厂标定 JSON</label><div class="inline-path"><input type="text" id="imp-source" list="imp-sources" value="${esc(c.source_file || "")}"><datalist id="imp-sources">${sources.map((s) => `<option value="${esc(s)}">`).join("")}</datalist></div></div>
      <div class="field"><label>LiDAR 时间偏移 (s)</label><input type="number" step="0.001" id="imp-lidar" value="${esc(S.drafts?.["imp-lidar"] ?? "")}" placeholder="留空 = 不改写，由 axle 阶段自动实测"></div>
      <div class="field"><label>相机时间偏移 (s)</label><input type="number" step="0.001" id="imp-camera" value="${esc(S.drafts?.["imp-camera"] ?? "")}" placeholder="留空 = 由 camtime 自动实测"></div>
      <div class="field"><label>说明</label><input type="text" id="imp-note" placeholder="例如：2026-05 出厂标定"></div>
      <div class="field" style="justify-content:flex-end"><button class="btn primary" data-act="import">导入并激活</button></div>
    </div></div>`;
}

function pageMap() {
  const v = vehicle(); const d = S.dataset; const m = S.state.mapping; const running = S.job && S.job.status === "running";
  const versions = (S.cal?.versions || []).filter((x) => v && x.mine === v.mine && x.truck === v.truck);
  // A stale calibration_id from another vehicle must never survive: the <select> would silently
  // display the first option (active version) while submitting the stale id. Reset it.
  if (m.calibration_id && S.cal && !versions.some((x) => x.id === m.calibration_id)) { m.calibration_id = ""; S.state.mapping.calibration_id = ""; saveState({ mapping: { calibration_id: "" } }); }
  const canLIVO = d && d.livo_ready, canLIO = d && d.lio_ready;
  const mode = ["livo", "gaussian"].includes(m.mode) && !canLIVO ? "lio" : m.mode;
  return `<h1 style="margin-bottom:6px">建图</h1><div class="muted" style="margin-bottom:18px">一次建图 = 启动 fastlivo_mapping → 按时间戳实时回放原始录制 → 收尾优化导出轨迹/地图 → 自动精度评估。所有输出在 <code>results/&lt;轮次&gt;/</code>（round.json 记录完整参数与阶段）。</div>
  ${liveMonitor()}
  <div class="card" style="margin-top:16px"><div class="card-h"><h2>参数</h2><div class="row"><span class="small muted">数据源</span><b>${esc(d?.name || "未选择")}</b>${datasetBadges(d)}${assessBadgeHome()}<button class="btn sm" data-act="go" data-page="data">更换</button></div></div>
    <div class="form">
      <div class="field"><label>模式</label><div class="seg"><button class="${mode === "lio" ? "active" : ""}" data-act="mode" data-mode="lio" ${canLIO ? "" : "disabled"}>LIO</button><button class="${mode === "livo" ? "active" : ""}" data-act="mode" data-mode="livo" ${canLIVO ? "" : "disabled"}>LIVO</button><button class="${mode === "gaussian" ? "active" : ""}" data-act="mode" data-mode="gaussian" ${canLIVO && m.camera !== "fisheye" ? "" : "disabled"} title="高斯建图目前仅支持中距相机">高斯建图</button></div></div>
      ${mode !== "lio" ? cameraSelector("mapping", m.camera) : ""}
      <div class="field"><label>标定版本</label><select data-act="field" data-section="mapping" data-key="calibration_id"><option value="" ${!m.calibration_id ? "selected" : ""}>自动按当前录制匹配（无匹配时使用激活版本）</option>${versions.map((x) => `<option value="${esc(x.id)}" ${m.calibration_id === x.id ? "selected" : ""} title="标定数据：${esc(x.source_dataset || "未登记")}">${esc(x.version)} · ${esc(x.stage_label)} · ${cameraLabel(x.camera)}${x.source_dataset && d?.path && x.source_dataset !== d.path ? " · 其他录制" : ""}</option>`).join("")}</select></div>

      <div class="field"><label>回放倍速</label><input type="number" step="0.1" min="0.1" max="4" data-act="field" data-section="mapping" data-key="rate" value="${esc(m.rate)}"></div>
      <div class="field"><label>起始偏移 (s)</label><input type="number" step="0.001" min="0" data-act="field" data-section="mapping" data-key="start_offset" value="${esc(m.start_offset ?? 0)}"><div class="small muted">数据评估后自动填入全程最早可用静止段的起点，可手动修改。${S.assessSummary?.startup?.stationary ? `检测静止段：${num(S.assessSummary.startup.stationary.start_s, 3)}–${num(S.assessSummary.startup.stationary.end_s, 3)} s。` : ""}</div></div>
      <div class="field"><label>回放时长 (s，空=全部)</label><input type="number" step="1" min="0" data-act="field" data-section="mapping" data-key="duration" value="${esc(m.duration ?? "")}"></div>
      <div class="field"><label>几何点云截取距离 (m)</label><input type="number" step="1" min="0" data-act="field" data-section="mapping" data-key="map_range" value="${esc(m.map_range ?? 45)}" ${m.save_map ? "" : "disabled"}></div>
      <div class="field"><label>强度点云过滤值 (m，默认 0.1)</label><input type="number" step="0.005" min="0.005" max="0.1" data-act="field" data-section="mapping" data-key="map_filter" value="${esc(m.map_filter ?? 0.1)}" ${m.save_map ? "" : "disabled"} title="map_N.pcd：每个分块里每个该尺寸的小格保留离雷达最近的一个原始回波。所有扫描均参与，包括停车帧，避免覆盖判定与实际写出不一致造成空洞；值越大重复点越少、文件越小。距离优先、看穿和地面细节补点保持启用。"></div>
      <div class="field"><label>颜色来源截取距离 (m，LIVO 生效)</label><input type="number" step="1" min="0" data-act="field" data-section="mapping" data-key="rgb_range" value="${esc(m.rgb_range ?? 60)}" ${m.save_map ? "" : "disabled"} title="彩色图先使用强度图的 D 回波掩码，再筛选同帧有效颜色；此距离只进一步裁剪，不能扩大原生保留范围。LIO 模式不生成彩色点云"></div>
      <div class="field"><label>彩色点云体素 (m，0 = 每行保留，LIVO 生效)</label><input type="number" step="0.01" min="0" max="0.5" data-act="field" data-section="mapping" data-key="rgb_voxel" value="${esc(m.rgb_voxel ?? 0.03)}" ${m.save_map ? "" : "disabled"} title="RGB 先使用与强度图相同的 D 掩码（含当前分块过滤），再按全图体素选择近距离、低转速的同帧着色原始观测；默认 3 cm，0 保留全部已通过 D 掩码的着色行"></div>
      <div class="field"><label>相机帧抽稀 (frame_divider)</label><input type="number" step="1" min="1" max="10" data-act="field" data-section="mapping" data-key="frame_divider" value="${esc(m.frame_divider ?? 1)}"></div>
      <div class="field"><label>静止跳过：最短静止时长 (s)</label><input type="number" step="5" min="10" data-act="field" data-section="mapping" data-key="stationary_min" value="${esc(m.stationary_min ?? 30)}" ${m.skip_stationary ? "" : "disabled"} title="起步后、持续时间不短于该值的静止段被快进；起步前的静止段始终正常回放（IMU 初始化、外参自检）"></div>
      <div class="field"><label>静止跳过：快进期激光间隔 (s)</label><input type="number" step="0.5" min="0.2" max="5" data-act="field" data-section="mapping" data-key="stationary_lidar_interval" value="${esc(m.stationary_lidar_interval ?? 1)}" ${m.skip_stationary ? "" : "disabled"} title="快进期间每隔多少秒保留一帧激光（IMU/INS 全量回放）"></div>
      <div class="field"><label>强度地图导出：体素 (m，默认 0.1)</label><input type="number" step="0.01" min="0.01" max="2" data-act="field" data-section="mapping" data-key="map_voxel" value="${esc(m.map_voxel ?? 0.1)}" ${m.save_map && m.map_export !== false ? "" : "disabled"} title="global_map.pcd 的体素边长；每个体素输出质心与平均强度"></div>
      <div class="field"><label>地图导出：点云来源</label><select data-act="field" data-section="mapping" data-key="map_source" ${m.save_map && m.map_export !== false ? "" : "disabled"}><option value="all" ${(m.map_source || "all") === "all" ? "selected" : ""}>全部扫描（最密）</option><option value="keyframes" ${m.map_source === "keyframes" ? "selected" : ""}>仅后端关键帧（约 1/5 数据量）</option></select></div>
      <div class="field"><label>地图导出：分块 (m，0 = 仅整图)</label><input type="number" step="10" min="0" data-act="field" data-section="mapping" data-key="map_tile" value="${esc(m.map_tile ?? 0)}" ${m.save_map && m.map_export !== false ? "" : "disabled"}></div>
      <div class="field"><label>地图导出：每体素最少点数 (1 = 全保留)</label><input type="number" step="1" min="1" max="1000" data-act="field" data-section="mapping" data-key="map_min_points" value="${esc(m.map_min_points ?? 1)}" ${m.save_map && m.map_export !== false ? "" : "disabled"} title="少于该点数的体素视为噪点/短暂出现的移动物体而剔除（如 3）；静止结构在 5 cm 体素内通常累积数十至数百个回波"></div>
      <div class="field wide"><label>额外参数覆盖（空格分隔 key=value，例如 vio.img_point_cov=100 time_offset.img_time_offset=-0.115）</label><input type="text" class="mono" data-act="field" data-section="mapping" data-key="extra_sets" value="${esc(m.extra_sets || "")}"></div>
    </div>
    <div class="small muted" style="margin-top:10px">截取距离仅控制导出的 PCD 范围，0 表示不限距离。彩色点云共用强度图的 D 回波掩码，再保留同帧能着色的点并按彩色体素归约；颜色距离只进一步裁剪；彩色点云只在 LIVO 模式生成，在 LIO 模式下也可以预先设置。</div>
    <div class="row" style="margin-top:14px;gap:22px">${toggle("use_rtk", "RTK / INS 融合", m.use_rtk)}${toggle("save_map", "保存地图 (PCD/扫描存档)", m.save_map)}${toggle("rviz", "RViz", m.rviz)}${toggle("precheck", "起步静止外参自检", m.precheck)}${toggle("evaluate", "自动评估", m.evaluate)}${toggle("skip_stationary", "长时间静止跳过（仅 LIO）", m.skip_stationary)}${toggle("loop_closure", "回环检测（收尾，需保存地图）", m.loop_closure !== false)}<label><input type="checkbox" checked disabled>距离优先与看穿检查</label>${toggle("map_export", "导出标准地图", m.map_export !== false)}${toggle("map_keyframes", "导出关键帧点云", m.map_keyframes)}${toggle("auto_cleanup", "磁盘不足时自动精简旧轮次", m.auto_cleanup !== false)}</div>
    <div class="row" style="margin-top:8px;gap:10px"><label class="small">自动精简阈值：可用空间低于 <input type="number" min="15" step="5" style="width:70px" data-act="field" data-section="mapping" data-key="auto_cleanup_free_gb" value="${esc(m.auto_cleanup_free_gb ?? 50)}" ${m.auto_cleanup !== false ? "" : "disabled"}> GB 时，开跑前按时间从旧到新精简未保护的历史轮次（见 系统 › 磁盘与历史数据）</label></div>
    <div class="small muted" style="margin-top:6px">地图产品（保存地图时生成，<code>Log/fusion/&lt;运行&gt;/map/</code>）：<b>global_map.pcd</b> 体素滤波整图（全部扫描或仅关键帧）、LIVO 另有 <b>global_map_rgb.pcd</b>；可选分块 tiles/ 与关键帧点云 keyframes/（机体系点云 + poses.txt）；<b>map_info.json</b> 记录坐标系、体素、范围、点数与输入。原始时间序分块 map_N.pcd（评估用）与扫描存档始终保留，可在结果页按其他体素/来源重新导出而无需重新建图。</div>
    <div class="small muted" style="margin-top:6px">长时间静止跳过（仅 LIO）：起步后的长静止段（停车等待）只快进中段——前 3 s 与重新起步前 5 s 照常回放，快进期间 IMU/INS 全量连续、激光按间隔抽稀；状态估计与 RTK/INS 关联保持连续，地图不重复累积静止帧。LIVO 轮次忽略此选项、照常回放（相机硬解码输出有延迟，快进后视觉更新会中断）。</div>
    <div class="sep"></div>
    <div class="row" style="justify-content:space-between"><div class="small muted">启动前会校验数据源话题与标定版本；进行中可“提前结束并保存”（停止回放，保留优化/导出/评估）。</div><button class="btn primary big" data-act="start" ${running || !canLIO || !v?.active || (mode !== "lio" && !hasCamera(d, m.camera || "midrange")) || (mode === "gaussian" && m.camera === "fisheye") ? "disabled" : ""}>${running ? "建图进行中…" : `开始 ${mode.toUpperCase()} 建图`}</button></div>
  </div>`;
}

function pageResults() {
  const ev = S.runs.filter((r) => r.evaluated && r.metrics?.optimized);
  const legend = `<div class="callout" style="margin-bottom:14px"><b>怎么读这张表</b> · <b>原生轨迹</b>＝前端里程计（LIO / LIVO，RViz 红线）与 RTK 后轴位置之差，起点对齐后不做任何轨迹对齐，误差随里程累积，用来比较前端与标定的好坏（注意：融合轮的前端含 INS 姿态辅助——仅横滚/俯仰弱约束，不注入 RTK 位置——所以其原生漂移通常小于无融合轮的纯 LIO，两种模式之间不可直接对比）；<b>RTK 优化后</b>＝融合后输出（蓝线），是交付地图的精度；<b>地面厚度</b>与<b>灯柱一致性</b>反映多期地图的几何一致性。点击行查看完整指标（姿态漂移、分时段、起步段、后端统计）；未评估的轮次可直接点「评估」。</div>`;
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
  ${resultsUsageCard()}
  <div class="card" style="margin-top:16px"><div class="card-h"><h2>任务历史</h2><button class="btn sm" data-act="reload-jobs">刷新</button></div>${jobsTable(S.jobs)}</div>`;
}
function resultsUsageCard() {
  const u = S.usage; const f = S.cleanupForm || (S.cleanupForm = { target: 100, allowDelete: false });
  const gb = (b) => (b / 1e9).toFixed(1);
  const badge = (r) => [r.running ? '<span class="badge accent">运行中</span>' : "", r.pinned ? '<span class="badge ok">保留</span>' : "", r.recent && !r.running ? '<span class="badge">最近</span>' : "", r.slim ? '<span class="badge">已精简</span>' : ""].join(" ");
  const rows = (u?.rounds || []).slice().reverse().map((r) => `<tr><td class="small nowrap">${esc(r.date.replace("T", " "))}</td><td class="mono small">${esc(r.round)}</td><td class="small">${esc(r.status || "—")}${r.kind === "calibration" ? ' <span class="muted">标定</span>' : ""}</td><td class="num">${gb(r.bytes)}</td><td class="num">${gb(r.heavy_bytes)}</td><td>${badge(r)}</td>
      <td class="nowrap"><button class="btn sm" data-act="results-pin" data-round="${esc(r.round)}" data-pinned="${r.pinned ? 0 : 1}" ${r.running ? "disabled" : ""}>${r.pinned ? "取消保留" : "保留"}</button>
      <button class="btn sm" data-act="results-cleanup" data-action="slim" data-round="${esc(r.round)}" ${r.protected || r.slim ? "disabled" : ""}>精简</button>
      <button class="btn sm danger" data-act="results-cleanup" data-action="delete" data-round="${esc(r.round)}" ${r.protected ? "disabled" : ""}>删除</button></td></tr>`).join("");
  return `<div class="card" style="margin-top:16px"><div class="card-h"><h2>磁盘与历史数据</h2><button class="btn sm" data-act="results-usage">刷新</button></div>
    ${u ? `<div class="small" style="margin-bottom:8px">磁盘可用 <b>${u.disk.free_gb} GB</b> / ${u.disk.total_gb} GB（已用 ${u.disk.used_pct}%）· results 共 ${gb(u.total_bytes)} GB · 可精简释放 ${gb(u.reclaimable_bytes)} GB</div>
    <div class="row" style="gap:10px;flex-wrap:wrap;margin-bottom:8px"><label class="small">释放到可用 <input type="number" min="10" step="10" style="width:80px" data-act="cleanup-field" data-key="target" value="${esc(f.target)}"> GB</label>
      <label class="small"><input type="checkbox" data-act="cleanup-field" data-key="allowDelete" ${f.allowDelete ? "checked" : ""}> 精简不够时删除整轮</label>
      <button class="btn sm" data-act="results-auto">一键清理（从最旧开始）</button></div>
    <div class="small muted" style="margin-bottom:8px">精简 = 删除扫描存档（scans/colors/sources.bin）、全分辨率分块 map_N/rgb_map_N.pcd、分块/关键帧点云与 roslog，保留轨迹、评估指标、标准整图 global_map.pcd 与 map_info.json（结果页数字不变，可随时查看整图）；删除 = 移除整个轮次目录。运行中、最近 3 轮与“保留”的轮次受保护。建图页可开启“磁盘不足时自动精简”。</div>
    <div style="max-height:360px;overflow:auto"><table class="tbl small"><thead><tr><th>时间</th><th>轮次</th><th>状态</th><th class=num>总计 GB</th><th class=num>可精简 GB</th><th></th><th></th></tr></thead><tbody>${rows}</tbody></table></div>` : `<div class="small muted">加载中…</div>`}</div>`;
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
  return `<div class="drawer-h"><div><h2>${esc(j.title)}</h2><div class="small muted">${fmtT(j.started)} · ${j.status === "running" ? "运行中" : esc(j.status)} · 错误：${j.error ? `<span class="bad">${esc(j.error)}</span>` : j.status === "done" ? "无" : "暂未报告"}</div></div>
    <div class="row">${j.status === "running" ? `<button class="btn" data-act="stop" data-mode="finish">提前结束并保存</button><button class="btn danger" data-act="stop" data-mode="abort">强制终止</button>` : ""}${j.kind === "pipeline" && ["failed", "stopped", "lost"].includes(j.status) ? `<button class="btn" data-act="resumeJob" title="从第一个未完成的步骤继续，已完成的阶段与轮次不再重跑">从失败步骤续跑</button>` : ""}<button class="btn sm" data-act="close">关闭</button></div></div>
    ${j.kind === "pipeline" && ["failed", "stopped", "lost"].includes(j.status) && j.meta?.kind === "lidar" ? axleLimitsHtml(`${j.meta.mine}/${j.meta.truck}`) : ""}
    ${j.steps.length > 1 ? `<div class="list" id="job-steps">${jobStepsHtml()}</div>` : ""}${resHtml}
    ${res.result_id ? `<div class="row" style="margin-bottom:10px"><button class="btn" data-act="open-version" data-id="${esc(res.result_id)}">查看版本 ${esc(res.result_id.split("/").pop())}</button>${res.camlidar_id ? `<button class="btn" data-act="open-version" data-id="${esc(res.camlidar_id)}">查看相机复核 ${esc(res.camlidar_id.split("/").pop())}</button>` : ""}</div>` : ""}
    ${j.meta?.round || res.round ? `<div class="row" style="margin-bottom:10px"><button class="btn" data-act="open-run" data-round="${esc(res.round || j.meta.round)}">查看轮次 ${esc(res.round || j.meta.round)}</button></div>` : ""}
    <div class="log" id="job-log">${jobLogHtml()}</div>`;
}
function drawerVersion() {
  const d = S.versionDetail; if (!d) return `<div class="empty">加载中…</div>`;
  const m = d.meta || {}; const sum = m.summary || {}; const ch = m.changes_vs_parent || {}; const val = d.validation || {};
  const lidars = (sum.lidars || []).map((l) => `<tr><td>${esc(l.lidar)}</td><td class="mono small">${l.t.map((x) => x.toFixed(3)).join(", ")}</td><td class="mono small">${l.rpy_deg.map((x) => x.toFixed(2)).join(", ")}</td><td class="mono small">${ch.lidars?.[l.lidar] ? `Δt ${ch.lidars[l.lidar].dt_cm.join("/")} cm · Δrpy ${ch.lidars[l.lidar].drpy_deg.join("/")}°` : '<span class="faint">无变化</span>'}</td></tr>`).join("");
  const cam = sum.camera_to_rear ? `<tr><td>${cameraLabel(d.camera)}→rear</td><td class="mono small">${sum.camera_to_rear.t.map((x) => x.toFixed(3)).join(", ")}</td><td class="mono small">${sum.camera_to_rear.rpy_deg.map((x) => x.toFixed(2)).join(", ")}</td><td class="mono small">${ch.camera_to_rear ? `Δt ${ch.camera_to_rear.dt_cm.join("/")} cm · Δrpy ${ch.camera_to_rear.drpy_deg.join("/")}°` : '<span class="faint">无变化</span>'}</td></tr>` : "";
  const offs = sum.time_offsets || {};
  const tsrc = (d.calibration && d.calibration.time_offsets) || {};
  const offHtml = Object.entries(offs).filter(([k, v]) => typeof v === "number").map(([k, v]) => {
    const src = tsrc[`${k}_source`] || "";
    return `<span class="badge ${ch.time_offsets?.[k] ? "accent" : ""}" title="${esc(src)}">${esc(k)} ${v} s${ch.time_offsets?.[k] ? ` (原 ${ch.time_offsets[k].from})` : (src ? ` · ${esc(src)}` : "")}</span>`;
  }).join(" ");
  const metrics = m.metrics && Object.keys(m.metrics).length ? `<details><summary>阶段指标</summary><pre class="small">${esc(JSON.stringify(m.metrics, null, 1))}</pre></details>` : "";
  const stage = d.stage_info || {};
  const cl = d.closed_loop;
  const photo = d.photo_validation || {}, coverage = photo.coverage || {}, independent = photo.validation || {};
  const photoEvidence = d.camera === "fisheye" ? `<div class="callout ${d.activation?.allowed ? "ok" : "warn"}" style="margin:10px 0"><b>鱼眼激活验收：${d.activation?.allowed ? "通过" : "尚未通过"}</b>${d.activation?.reason ? `<div class="small">${esc(d.activation.reason)}</div>` : ""}<div class="small">联合模型：${esc(photo.model || "未完成")} · 拟合覆盖 ${coverage.camera_segment_count ?? 0} 个相机包 / ${coverage.frames ?? 0} 帧 / ${coverage.views ?? 0} 视角</div><div class="small">独立固定候选验证：${independent.passed === true && independent.refitted === false ? "通过" : "未完成或未通过"} · ${(independent.runs || []).length} 轮</div>${(independent.runs || []).map((r) => `<div class="small">${esc(baseName(r.run))} · ${r.coverage?.camera_segment_count ?? 0} 个包 / ${r.coverage?.frames ?? 0} 帧 / ${r.coverage?.views ?? 0} 视角 · ${r.passed ? "通过" : "未通过"}${r.gains ? ` · 两半光度增益 ${r.gains.map((g) => `${(100 * g).toFixed(2)}%`).join(" / ")}` : ""}${r.reason ? ` · ${esc(r.reason)}` : ""}</div>`).join("")}</div>` : "";
  const closedLoop = cl ? `<div class="callout ${cl.verdict === "pass" ? "ok" : "bad"}" style="margin:10px 0"><b>${cameraLabel(d.camera)} LIVO 闭环验证${cl.verdict === "pass" ? "通过" : "未通过"}</b>（原生轨迹 A/B：${esc(cl.baseline?.round || "")} → ${esc(cl.candidate?.round || "")}）<div class="small" style="margin-top:4px">原生里程尺度 ${cl.baseline?.scale != null ? (cl.baseline.scale * 100).toFixed(2) + "%" : "—"} → ${cl.candidate?.scale != null ? (cl.candidate.scale * 100).toFixed(2) + "%" : "—"}${cl.common_window ? ` · 后轴误差@${cl.common_window.t0}–${cl.common_window.t1} s ${f2(cl.common_window.baseline_rear_m)} → ${f2(cl.common_window.candidate_rear_m)} m` : ""} · 偏航 P95 ${f2(cl.baseline?.yaw_p95_deg)} → ${f2(cl.candidate?.yaw_p95_deg)}°${cl.reasons?.length ? `<br>原因：${esc(cl.reasons.join("；"))}` : ""}${cl.verdict !== "pass" ? "<br><b>请勿激活该版本</b>；继续使用基线版本。" : ""}</div></div>` : (stage.advisory || m.stage?.startsWith("cam") ? `<div class="callout warn" style="margin:10px 0">该版本仅供参考，尚未做 LIVO 闭环验证：投影对齐只用于偏航/横滚复核；俯仰需有充分运动的闭环或多视角光度标定确认。请通过相机闭环或光度流水线验证后再激活。</div>` : "");
  const files = Object.entries(d.files || {}).map(([n, t]) => `<details><summary>${esc(n)}</summary><pre class="small" style="max-height:300px;overflow:auto">${esc(t)}</pre></details>`).join("");
  const overlays = d.overlays.length ? `<div class="sep"></div><h3>阶段叠加图（${d.overlays.length}）</h3><div class="small muted" style="margin:4px 0 8px">偏航复核阶段的灯柱裁剪图：红＝按父版本外参投影的 LiDAR 灯柱轴线，绿＝图像中量得的灯柱中心列；两者的列偏差（像素）÷ 焦距即偏航修正量（0.1° ≈ 3 px）。</div><div class="overlays">${d.overlays.map((n) => `<a href="/api/calibrations/file?id=${encodeURIComponent(d.id)}&name=overlays/${encodeURIComponent(n)}" target="_blank"><img loading="lazy" src="/api/calibrations/file?id=${encodeURIComponent(d.id)}&name=overlays/${encodeURIComponent(n)}" title="${esc(n)}"><div class="small muted ellipsis">${esc(n)}</div></a>`).join("")}</div>` : "";
  return `<div class="drawer-h"><div><h2>${esc(d.id)} ${d.active ? '<span class="badge dark">激活</span>' : ""}</h2><div class="small muted">${esc(m.stage_label || stage.label || m.stage)}${stage.advisory && cl?.verdict !== "pass" ? ' · <span class="warn">仅供参考</span>' : ""} · ${fmtT(m.created)} · 父版本 ${esc(m.parent || "无")}</div></div>
    <div class="row"><button class="btn primary" data-act="activate" data-id="${esc(d.id)}" ${d.active || m.stage === "scan" || d.activation?.allowed === false ? "disabled" : ""} title="${esc(d.activation?.reason || (m.stage === "scan" ? "临时扫描点不能激活" : ""))}">激活</button><button class="btn" data-act="validate" data-id="${esc(d.id)}">校验</button><button class="btn" data-act="use-parent" data-id="${esc(d.id)}">作为父版本</button><button class="btn danger sm" data-act="delete-version" data-id="${esc(d.id)}" ${d.active || d.children.length ? "disabled" : ""} title="${d.children.length ? "存在子版本" : d.active ? "激活中" : ""}">删除</button><button class="btn sm" data-act="close">关闭</button></div></div>
    <div class="callout ${val.ok ? (val.warnings?.length ? "warn" : "ok") : "bad"}">${val.ok ? "校验通过" : "校验失败"}${(val.errors || []).map((e) => `<div>✕ ${esc(e)}</div>`).join("")}${(val.warnings || []).map((e) => `<div>△ ${esc(e)}</div>`).join("")}</div>
    <div class="kv small" style="margin:12px 0"><div class="k">相机</div><div>${cameraLabel(d.camera)} · ${esc(d.calibration?.camera?.topic || "")}<br>同文件可用：${(d.available_cameras || []).map(cameraLabel).join("、")}</div><div class="k">说明</div><div>${esc(m.note || "")}</div><div class="k">阶段</div><div>${esc(stage.input || "")} → ${esc(stage.output || "")}</div><div class="k">谱系</div><div class="mono">${(d.lineage || []).map((x) => x.split("/").pop()).join(" → ")}</div><div class="k">输入</div><div class="mono">${esc(JSON.stringify(m.inputs || {}))}</div><div class="k">文件</div><div class="mono">${esc(m.calibration_file || d.dir)}</div></div>
    <h3>外参摘要（后轴系，t 单位 m，rpy 单位 °）</h3>
    <table style="margin:6px 0 10px"><thead><tr><th>传感器</th><th>t (x,y,z)</th><th>rpy</th><th>相对父版本</th></tr></thead><tbody>${lidars}${cam}</tbody></table>
    <div class="row">${offHtml}${sum.camera?.intrinsics ? `<span class="badge">fx/fy/cx/cy ${sum.camera.intrinsics.map((x) => x.toFixed(1)).join(" / ")}</span>` : ""}</div>
    ${photoEvidence}${closedLoop}<div class="sep"></div>${overlayPanel(d)}${metrics}${files}${overlays}`;
}
function overlayPanel(d) {
  const f = S.overlayForm; const ds = S.dataset; const o = S.overlay && S.overlay.id === d.id ? S.overlay : null;
  const runs = S.runs.filter((r) => r.fusion_dir && r.status === "done" && r.mode === "livo" && (r.camera || "midrange") === (d.camera || "midrange"));
  const run = (runs.some((r) => r.fusion_dir === f.run) ? f.run : "") || (runs.find((r) => r.calibration_id === d.id) || runs[0] || {}).fusion_dir || "";
  const deltaVals = (f.delta || "").trim().split(/[\s,]+/).map(Number);
  const hasDelta = deltaVals.length === 3 && deltaVals.every(Number.isFinite) && deltaVals.some((v) => v !== 0);
  return `<details class="overlay-tool" ${o ? "open" : ""}><summary><b>目视复核：LiDAR 点云投影到相机图像</b> <span class="small muted">（用一次 LIVO 建图的扫描存档，约 10 s；用于确认外参没有粗大错误，不作为标定手段——肉眼分辨约 2–3 px ≈ 0.1°，而尺度漂移要求 0.02°）</span></summary>
    <div class="form" style="margin-top:8px">
      <div class="field wide"><label>LIVO 建图轮次（需保存了地图/扫描存档）</label><select data-act="ov-field" data-key="run">${runs.map((r) => `<option value="${esc(r.fusion_dir)}" ${run === r.fusion_dir ? "selected" : ""}>${esc(r.round)} · ${esc((r.calibration_id || "").split("/").pop())} · ${fmtT(r.start)}</option>`).join("") || "<option value=''>（没有可用的 LIVO 轮次）</option>"}</select></div>
      <div class="field"><label>时刻（秒，逗号分隔）</label><input type="text" data-act="ov-field" data-key="times" value="${esc(f.times)}"></div>
      <div class="field"><label>试探旋转 rx ry rz（°，相机系；仅预览）</label><input type="text" class="mono" data-act="ov-field" data-key="delta" value="${esc(f.delta)}" placeholder="0 0 0"></div>
      <div class="field" style="justify-content:flex-end;gap:8px"><button class="btn primary" data-act="ov-render" data-id="${esc(d.id)}" ${S.overlayBusy || !run || !ds?.livo_ready ? "disabled" : ""}>${S.overlayBusy ? "生成中…" : "生成叠加图"}</button>${hasDelta ? `<button class="btn" data-act="ov-save" data-id="${esc(d.id)}" title="以该版本为父版本，按上面的旋转修正生成 manual 版本（激活前请做闭环验证）">保存为手动版本</button>` : ""}</div>
    </div>
    <div class="small muted" style="margin:6px 0">rx 绕相机 x（右）轴＝俯仰：正值使投影点上移；ry 绕 y（下）轴＝偏航：正值使投影点左移；rz＝横滚。颜色：红近 → 蓝远（对数深度，60 m 以上为深蓝）。</div>
    ${o ? `<div class="overlays big">${(o.images || []).map((im) => `<a href="${im.url}" target="_blank"><img loading="lazy" src="${im.url}" title="t=+${im.t}s"><div class="small muted">t=+${im.t} s · ${im.speed_mps} m/s · ${im.points} 点${o.camera_rot_delta_deg_cam_xyz ? ` · Δ ${o.camera_rot_delta_deg_cam_xyz.join("/")}°` : ""}</div></a>`).join("")}</div>${(o.frames || []).filter((x) => x.error).map((x) => `<div class="small bad">t=+${x.t}s：${esc(x.error)}</div>`).join("")}` : ""}
  </details>`;
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
    ${h.pole_pair_separation_cm ? stat("灯柱多期分离", `${f1(h.pole_pair_separation_cm.median)} / ${f1(h.pole_pair_separation_cm.p95)} cm`, `诊断 · 中位 / P95 · ${h.pole_chunks} 期（验收看壳层 P90）`) : ""}
  </div>`;
  const trow = (label, e) => e ? `<tr><td>${label}</td><td class=num>${fm(e.median_m)}</td><td class=num>${fm(e.p95_m)}</td><td class=num>${fm(e.max_m)}</td><td class=num>${fm(e.final_m)}</td><td class=num>${fm(e.final_xy_m)}</td><td class=num>${fm(e.final_z_m)}</td><td class=num>${pct(e.route_le10cm_pct)}</td><td class=num>${pct(e.route_gt20cm_pct)}</td><td>${e.budget ? `<span class="badge ${e.budget === "PASS" ? "ok" : "warn"}">${esc(e.budget)}</span>` : "—"}</td></tr>` : "";
  const traj = `<details open><summary>轨迹精度 · 后轴 vs RTK（全程，起点对齐后不做任何轨迹对齐）</summary>
    <table class="small"><thead><tr><th></th><th class=num>中位</th><th class=num>P95</th><th class=num>最大</th><th class=num>终点</th><th class=num>终点 XY</th><th class=num>终点 Z</th><th class=num>路线 ≤10 cm</th><th class=num>>20 cm</th><th>预算</th></tr></thead><tbody>${trow("原生（前端里程计，红线）", n)}${trow("RTK 优化后（蓝线）", o)}</tbody></table>
    <div class="small muted" style="margin-top:6px">原生误差随里程累积，反映本轮实际运行的前端（LIO / LIVO）自身漂移——融合模式下前端含 INS 姿态辅助（仅横滚/俯仰），无 RTK 模式为纯 LIO；里程尺度（原生路径长 / 融合路径长）偏离 100% 说明前端存在尺度误差——LIVO 模式下这是相机外参（俯仰）误差最灵敏的指标（约 11%/°）；优化后误差反映 RTK/INS 融合输出的质量。路线百分比按行驶距离加权。${s?.native?.imu ? ` IMU 轨迹 vs INS 推算：原生 ${fm(s.native.imu.median_m)} / 优化后 ${fm(s.optimized?.imu?.median_m)}（中位）。` : ""}</div></details>`;
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
  const pole = (s?.pole || d.pole_pairs) ? `<details><summary>灯柱多期一致性${h.pole_pair_separation_cm ? ` · 环心跨期 中位 ${f1(h.pole_pair_separation_cm.median)} · P95 ${f1(h.pole_pair_separation_cm.p95)} · 最大 ${f1(h.pole_pair_separation_cm.max)} cm（${h.pole_chunks} 期 · 诊断，非验收指标）` : ""}</summary>
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
  return `<div class="drawer-h"><div><h2>${esc(r.round)} <span class="badge">${esc(modeLabel(r))}</span></h2><div class="small muted">${fmtT(r.start)} → ${fmtT(r.end)} · ${esc(r.status)}${r.phase ? ` / ${esc(r.phase)}` : ""} · 提交 ${esc(r.commit || "")}</div></div>
    <div class="row"><button class="btn" data-act="evaluate" data-round="${esc(r.round)}" ${r.status === "running" ? "disabled" : ""}>${l.evaluated ? "重新评估" : "运行评估"}</button><button class="btn sm" data-act="close">关闭</button></div></div>
    ${r.error ? `<div class="callout bad">${esc(r.error)}</div>` : ""}
    <div class="kv small" style="margin:10px 0"><div class="k">数据源</div><div class="mono">${esc(r.dataset)}</div><div class="k">标定</div><div class="mono">${esc(r.calibration || "config yaml 内置")}</div><div class="k">参数</div><div class="mono">${esc(JSON.stringify(r.options || {}))} ${esc(Array.isArray(r.overrides) ? r.overrides.join(" ") : r.overrides || "")}</div><div class="k">fusion</div><div class="mono">${esc(l.fusion_dir || "")}</div><div class="k">地图</div><div>${l.map_chunks ?? 0} 块 · ${l.map_size_mb ?? 0} MB${l.rgb_map_chunks ? ` · RGB ${l.rgb_map_chunks} 块` : ""}</div>${loopsKv(l)}</div>
    ${metricsPanel(l)}
    <div class="grid g2" style="margin:12px 0"><div><h4>前端 RTK 残差 (m)</h4>${chart(l.diagnostics, "frontend_rtk_residual_m", "#0071e3", 0.1)}</div><div><h4>${l.record?.options?.use_rtk === false ? `在线轨迹 vs INS (m)　<span class="small muted">无 RTK/INS 融合</span>` : `优化后轨迹 vs RTK (m)　<span class="small muted">虚线：在线轨迹 vs INS</span>`}</h4>${trajectoryChart(l, l.record?.options?.use_rtk === false)}</div></div>
    ${mapProductsPanel(r, l)}
    ${l.summary ? `<details><summary>评估摘要原文（summary.txt）</summary><pre class="small" style="max-height:320px;overflow:auto">${esc(l.summary.join("\n"))}</pre></details>` : ""}
    ${preHtml}
    <details><summary>算法事件（${(l.launch_events || []).length}）</summary><div class="events">${(l.launch_events || []).map((e) => `<div class="${/WARN/.test(e) ? "warn" : /ERROR|FATAL|died/.test(e) ? "bad" : ""}">${esc(e)}</div>`).join("")}</div></details>
    <div class="sep"></div><div class="row">${files}</div>`;
}

function mapProductsPanel(r, l) {
  if (!l.fusion_dir) return "";
  const prods = l.map_products || [];
  const f = S.exportForm || (S.exportForm = { voxel: 0.1, source: "all", tile: 0, keyframes: false, min_points: 1 });
  const rows = prods.map((p) => `<tr><td class="mono">${esc(p.dir)}</td><td>${p.voxel_m === 0 ? "不滤波" : `${(p.voxel_m * 100).toFixed(0)} cm`}</td><td>${p.source === "keyframes" ? "关键帧" : "全部扫描"}${p.min_points > 1 ? ` · ≥${p.min_points} 点` : ""}</td><td>${p.tile_m ? `${p.tile_m} m` : "—"}</td><td>${p.files.map((x) => `<a href="/api/runs/${encodeURIComponent(r.round)}/file?name=${encodeURIComponent(`Log/fusion/${l.fusion_dir.split("/").pop()}/${x.name}`)}" target="_blank">${esc(x.name.split("/").pop())}</a> ${x.points != null ? `${(x.points / 1e6).toFixed(1)} M 点` : ""} · ${x.mb} MB`).join("<br>")}</td><td class="small muted">${esc(fmtT(p.created))}${p.elapsed_s != null ? ` · ${p.elapsed_s} s` : ""}</td></tr>`).join("");
  const gs = l.gaussian;
  const gsUrl = (name) => `/api/runs/${encodeURIComponent(r.round)}/file?name=${encodeURIComponent(`Log/fusion/${l.fusion_dir.split("/").pop()}/${name}`)}`;
  const gaussianPanel = gs ? `<div class="card"><b>高斯地图</b> · ${gs.gaussians.toLocaleString()} 个高斯 · ${gs.frames} 帧 · ${gs.submaps} 子图 · ${num(gs.elapsed_sec, 1)} s<div class="row">${gs.files.map((name) => `<a href="${gsUrl(name)}" target="_blank">${esc(name.split("/").pop())}</a>`).join(" · ")}</div><div class="overlays">${gs.renders.map((name) => `<a href="${gsUrl(name)}" target="_blank"><img loading="lazy" src="${gsUrl(name)}" title="留出视角渲染"></a>`).join("")}</div></div>` : "";
  return `${gaussianPanel}<details ${prods.length ? "" : "open"}><summary>地图产品（${prods.length}）</summary>
    ${prods.length ? `<table class="tbl small"><thead><tr><th>目录</th><th>体素</th><th>来源</th><th>分块</th><th>文件</th><th>生成</th></tr></thead><tbody>${rows}</tbody></table>` : `<div class="small muted">尚未导出标准地图产品。</div>`}
    ${l.has_archive ? `<div class="row" style="margin-top:8px;gap:10px;flex-wrap:wrap"><label class="small">体素 (m) <input type="number" step="0.01" min="0.01" max="2" style="width:80px" data-act="export-field" data-key="voxel" value="${esc(f.voxel)}"></label><label class="small">来源 <select data-act="export-field" data-key="source"><option value="all" ${f.source === "all" ? "selected" : ""}>全部扫描</option><option value="keyframes" ${f.source === "keyframes" ? "selected" : ""}>仅关键帧</option></select></label><label class="small">分块 (m) <input type="number" step="10" min="0" style="width:80px" data-act="export-field" data-key="tile" value="${esc(f.tile)}"></label><label class="small">最少点数/体素 <input type="number" step="1" min="1" style="width:60px" data-act="export-field" data-key="min_points" value="${esc(f.min_points ?? 1)}"></label><label class="small"><input type="checkbox" data-act="export-field" data-key="keyframes" ${f.keyframes ? "checked" : ""}> 关键帧点云</label><button class="btn sm" data-act="export-map" data-round="${esc(r.round)}">按此设置重新导出</button></div><div class="small muted">从扫描存档按最终优化位姿重投影（与建图输出一致），不需要重新建图；默认设置写入 map/，其他设置写入独立目录。</div>` : `<div class="small muted">本轮没有扫描存档（scans.bin），无法重新导出。</div>`}
  </details>`;
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
    if (page === "mapping" && currentRound()) pollLive().then(() => { if (S.page === "mapping") render(); }).catch(() => {});
    if (page === "system") A["results-usage"]();
    if (page === "body") A["bf-load"]();
  },
  close() { S.drawer = null; renderDrawer(); },
  async "drawer-job"() { S.drawer = "job"; if (!S.job && S.jobs.length) { await A["open-job"]({ dataset: { id: S.jobs[0].id } }); return; } renderDrawer(); if (S.job) { await pollJob(); renderDrawer(); } },
  async "open-job"(el) { const id = el.dataset.id; S.job = { id, status: "loading", steps: [], meta: {}, results: {} }; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; renderDrawer(); await pollJob(); renderDrawer(); },
  mode(el) { S.state.mapping.mode = el.dataset.mode; saveState({ mapping: { mode: el.dataset.mode } }); render(); },
  toggle(el) { const sec = el.dataset.section, key = el.dataset.key; S.state[sec][key] = el.checked; saveState({ [sec]: { [key]: el.checked } }); if (sec === "calibration" || (sec === "mapping" && key === "save_map")) render(); },
  field(el) { const sec = el.dataset.section, key = el.dataset.key; const v = el.type === "number" ? (el.value === "" ? "" : Number(el.value)) : el.value; S.state[sec][key] = v; const patch = { [key]: v }; if (sec === "calibration" && key === "camera") { S.state.calibration.validation_run = ""; patch.validation_run = ""; } saveState({ [sec]: patch }); if (el.tagName === "SELECT") render(); },
  async "set-vehicle"(el) { const [mine, truck] = el.value.split("/"); S.state.vehicle = { mine, truck }; S.calibForm.parent = null; S.state.calibration.parent_id = ""; S.state.mapping.calibration_id = ""; await saveState({ vehicle: { mine, truck }, calibration: { parent_id: "" }, mapping: { calibration_id: "" } }); render(); if (S.page === "body") A["bf-load"](); },
  async start() {
    const v = vehicle(); const m = S.state.mapping; const d = S.dataset;
    const mode = ["livo", "gaussian"].includes(m.mode) && !d?.livo_ready ? "lio" : m.mode;
    await act(async () => { const job = await api("/api/mapping/start", "POST", { ...m, mode, dataset: d.path, mine: v.mine, truck: v.truck }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.live = null; await loadOverview(); render(); }, "建图已启动");
  },
  async stop(el) {
    const mode = el.dataset.mode;
    if (mode === "abort" && !confirm("强制终止会杀掉建图与回放进程，本轮结果不完整。确定？")) return;
    await act(async () => { if (S.job?.id) await api(`/api/jobs/${S.job.id}/stop`, "POST", { mode }); else await api("/api/mapping/stop", "POST", { mode }); await pollJob(); patchLive(); }, mode === "finish" ? "已停止回放，正在收尾导出" : "已发送终止信号");
  },
  async resumeJob() {
    const j = S.job; if (!j) return;
    await act(async () => { const job = await api("/api/calibrations/pipeline", "POST", { ...(j.meta?.options || {}), axle_limits: currentAxleLimits(`${j.meta.mine}/${j.meta.truck}`), kind: j.meta?.kind, resume: j.id }); S.job = job; S.jobLog = []; S.jobLogNext = 0; await loadOverview(); render(); }, "已从失败步骤续跑");
  },
  "axle-limit"(el) {
    const value = el.value === "" ? S.cal.axle_limits[el.dataset.key].default : Number(el.value);
    if (!Number.isFinite(value) || value < 0) { el.value = currentAxleLimits(el.dataset.vehicle)[el.dataset.key]; return toast("门限必须是有限非负数", "bad"); }
    const all = { ...(S.state.calibration.axle_limits_by_vehicle || {}) };
    all[el.dataset.vehicle] = { ...(all[el.dataset.vehicle] || {}), [el.dataset.key]: value };
    S.state.calibration.axle_limits_by_vehicle = all;
    saveState({ calibration: { axle_limits_by_vehicle: all } });
  },
  "reset-axle-limits"(el) {
    const all = { ...(S.state.calibration.axle_limits_by_vehicle || {}) };
    delete all[el.dataset.vehicle]; S.state.calibration.axle_limits_by_vehicle = all;
    saveState({ calibration: { axle_limits_by_vehicle: all } }); render();
  },
  async kill() { if (!confirm("清理所有残留的 fastlivo_mapping / 回放 / launch 进程？")) return; await act(() => api("/api/mapping/kill", "POST", {}), "已发送清理信号"); await loadOverview(); render(); },
  async pipeline(el) {
    const v = vehicle(); const d = S.dataset; const kind = el.dataset.kind; const c = S.state.calibration;
    if (!v?.active) return toast("请先导入并激活一个标定版本", "bad");
    const std = c.fast === false;
    const label = { lidar: "LiDAR 标定流水线（后轴对齐 → 地面精修 → LIO 建图 → 行驶段精修 → 校验）", camera: c.camera === "fisheye" ? "鱼眼完整标定（时间 → 基线 → 偏航 → 内参诊断 → 新基线 → 联合光度标定 → 独立多包验证 → 建图闭环）" : `相机标定流水线（时间偏移 → LIVO 建图 A → 灯柱偏航复核 → 俯仰闭环${std ? "扫描" : "（快速）"} → 验证建图 → 闭环判定）`, camloop: `相机外参闭环微调（${std ? "标准" : "快速"}：${std ? "两轮扫描" : "复用已有轮次 + 一轮验证"} → 闭环判定）`, camphoto: "相机光度标定（基线建图 → 旋转/时间/内参与畸变联合复核 → 独立验证 → 验证建图 → 判定）" }[kind];
    const w = winEffective(S.winCalib, d?.duration_s);
    const winInfo = w ? `\n时间窗口：${winLabel(S.winCalib, d?.duration_s)}` : "\n时间窗口：自动（按运动窗口）";
    const parent = selectedParent();
    if (!confirm(`运行 ${label}${kind !== "lidar" ? `（${cameraLabel(c.camera)}）` : ""}\n父版本：${parent}${parent !== v.active ? `（当前激活 ${v.active}）` : ""}\n数据源：${d?.path}${winInfo}\n流水线包含一次完整回放建图，请勿同时启动其他建图。`)) return;
    await act(async () => { const job = await api("/api/calibrations/pipeline", "POST", { kind, camera: c.camera || "midrange", mine: v.mine, truck: v.truck, id: parent, parent_id: parent, from_factory: false, body_filter_source_id: v.active, dataset: d.path, activate: c.auto_activate !== false, use_rtk: true, rate: 1.0, note: "", mode: c.fast === false ? "standard" : "fast", window_start: w?.start_s ?? null, window_duration: w?.duration_s ?? null, keep_calib_maps: c.keep_maps === true, validation_run: (kind === "camphoto" || (kind === "camera" && c.camera === "fisheye")) ? c.validation_run || [] : [], axle_limits: currentAxleLimits() }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; await loadOverview(); render(); }, "流水线已启动");
  },
  "toggle-import"() { S.showImport = !S.showImport; render(); },
  async import() {
    const body = { camera: S.state.calibration.camera || "midrange", mine: $("#imp-mine").value.trim(), truck: $("#imp-truck").value.trim(), source: $("#imp-source").value.trim(), note: $("#imp-note").value.trim(), activate: true };
    const lidar = ($("#imp-lidar").value || "").trim();
    const camera = ($("#imp-camera").value || "").trim();
    if (lidar !== "") body.lidar_offset = lidar;  // 留空 = 不传，程序自动检测时间偏移
    if (camera !== "") body.camera_offset = camera;
    await act(async () => { const job = await api("/api/calibrations/import", "POST", body); S.state.vehicle = { mine: body.mine, truck: body.truck }; S.calibForm.parent = null; S.state.calibration.parent_id = ""; S.state.mapping.calibration_id = ""; await saveState({ vehicle: S.state.vehicle, mapping: { calibration_id: "" }, calibration: { parent_id: "", source_file: body.source, lidar_offset: lidar === "" ? null : Number(lidar), camera_offset: camera === "" ? null : Number(camera) } }); S.drafts = {}; S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; S.showImport = false; render(); }, "导入已开始");
  },
  "photo-validation"(el) { const values = Array.from(el.selectedOptions, (o) => o.value).filter(Boolean); S.state.calibration.validation_run = values; saveState({ calibration: { validation_run: values } }); },
  "calib-stage"(el) { S.calibForm.stage = el.value; render(); },
  "calib-field"(el) {
    const k = el.dataset.key;
    S.calibForm[k] = el.type === "checkbox" ? el.checked : el.value;
    if (k === "parent") {
      S.state.calibration.parent_id = S.calibForm.parent || "";
      saveState({ calibration: { parent_id: S.state.calibration.parent_id } });
    }
    if (k === "parent") render();
  },
  async "run-stage"() {
    const v = vehicle(); const f = S.calibForm; const d = S.dataset;
    const w = winEffective(S.winCalib, d?.duration_s);
    const body = { stage: f.stage, camera: S.state.calibration.camera || "midrange", mine: v?.mine, truck: v?.truck, id: selectedParent(), dataset: d?.path, run: f.run || ($("select[data-key=run]") || {}).value, run_dir: f.run_dir || ($("select[data-key=run_dir]") || {}).value, note: f.note, activate: S.state.calibration.camera === "fisheye" ? false : !!f.activate, frames: f.frames, window_start: w?.start_s ?? null, window_duration: w?.duration_s ?? null };
    if (f.stage === "axle") body.axle_limits = currentAxleLimits();
    if (f.stage === "camphoto") body.validation_run = S.state.calibration.validation_run || [];
    if (f.stage === "manual") { if (f.camera_offset !== "" && f.camera_offset != null) body.camera_offset = Number(f.camera_offset); if (f.lidar_offset !== "" && f.lidar_offset != null) body.lidar_offset = Number(f.lidar_offset); if (f.camera_rot_delta && f.camera_rot_delta.trim()) body.camera_rot_delta = f.camera_rot_delta.trim().split(/[\s,]+/).map(Number); }
    await act(async () => { const job = await api("/api/calibrations/stage", "POST", body); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "阶段已启动");
  },
  "ov-field"(el) { S.overlayForm[el.dataset.key] = el.value; if (el.dataset.key === "run" || el.dataset.key === "delta") renderDrawer(); },
  async "ov-render"(el) {
    const f = S.overlayForm; const d = S.dataset; const sel = $("select[data-key=run]");
    const run = f.run || (sel ? sel.value : "");
    const delta = (f.delta || "").trim() ? f.delta.trim().split(/[\s,]+/).map(Number) : null;
    S.overlayBusy = true; renderDrawer();
    try {
      S.overlay = await api("/api/calibrations/overlay", "POST", { id: el.dataset.id, camera: S.versionDetail?.camera || "midrange", run, dataset: d?.path, times: f.times, camera_rot_delta: delta });
      S.overlay.id = el.dataset.id;
    } catch (e) { toast(e.message || String(e), "bad"); }
    S.overlayBusy = false; renderDrawer();
  },
  async "ov-save"(el) {
    const f = S.overlayForm; const delta = f.delta.trim().split(/[\s,]+/).map(Number);
    if (delta.length !== 3 || !delta.every(Number.isFinite)) return toast("旋转修正需要 3 个数", "bad");
    if (!confirm(`以 ${el.dataset.id.split("/").pop()} 为父版本生成手动版本：相机旋转修正 ${delta.join(" / ")}°？\n该版本不会自动激活，请先用“相机外参闭环微调”或一轮 LIVO 建图验证。`)) return;
    await act(async () => { const job = await api("/api/calibrations/stage", "POST", { stage: "manual", id: el.dataset.id, camera: S.versionDetail?.camera || "midrange", camera_rot_delta: delta, note: `目视复核后的相机旋转修正 ${delta.join("/")}°（未闭环验证）` }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "已生成手动版本任务");
  },
  async "open-version"(el) { const id = el.dataset.id; S.drawer = "version"; S.versionDetail = null; renderDrawer(); await act(async () => { S.versionDetail = await api(`/api/calibrations/version?id=${encodeURIComponent(id)}`); render(); }); },
  async activate(el) {
    const v = S.versionDetail; const adv = v && v.id === el.dataset.id && (v.stage?.advisory || v.meta?.advisory);
    const cl = v && v.id === el.dataset.id ? v.closed_loop : null;
    if (cl && cl.verdict === "fail" && !confirm(`该版本未通过 LIVO 闭环验证：原生轨迹终点误差 ${cl.baseline?.final_m ?? "—"} m → ${cl.candidate?.final_m ?? "—"} m。激活后 LIVO 前端漂移会变大，确定仍要激活？`)) return;
    if (!cl && adv && !confirm("该版本为「相机-LiDAR 外参精修」结果，仅供参考：尚未通过 LIVO 闭环验证（原生轨迹漂移可能显著变大）。\n建议先用该版本运行一轮 LIVO 建图并与父版本比较原生轨迹终点误差。\n确定激活？")) return;
    await act(async () => { await api("/api/calibrations/activate", "POST", { id: el.dataset.id }); await Promise.all([loadOverview(), loadCatalogue(true)]); S.versionDetail = await api(`/api/calibrations/version?id=${encodeURIComponent(el.dataset.id)}`); render(); }, "已激活"); },
  async validate(el) { await act(async () => { const r = await api("/api/calibrations/validate", "POST", { id: el.dataset.id, strict: true }); toast(r.ok ? `校验通过${r.warnings.length ? `（${r.warnings.length} 条提示）` : ""}` : `校验失败：${r.errors.join("；")}`, r.ok ? "ok" : "bad"); }); },
  "use-parent"(el) { S.calibForm.parent = el.dataset.id; S.state.calibration.parent_id = el.dataset.id; saveState({ calibration: { parent_id: el.dataset.id } }); S.drawer = null; S.page = "calib"; render(); toast(`单步执行、自动化流水线与车身过滤拼接的父版本已设为 ${el.dataset.id.split("/").pop()}`); },
  async "delete-version"(el) { if (!confirm(`删除版本 ${el.dataset.id}？（目录将被移除，不可恢复）`)) return; await act(async () => { await api("/api/calibrations/delete", "POST", { id: el.dataset.id }); S.drawer = null; S.versionDetail = null; await Promise.all([loadOverview(), loadCatalogue(true)]); render(); }, "已删除"); },
  async "open-run"(el) { const round = el.dataset.round; S.runDetail = { round }; S.drawer = "run"; S.live = null; renderDrawer(); await act(async () => { S.live = await api(`/api/runs/${round}/live`); render(); }); },
  async evaluate(el) { await act(async () => { const job = await api("/api/evaluate", "POST", { round: el.dataset.round }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "评估已启动"); },
  async "from-precheck"(el) { const v = vehicle(); await act(async () => { const job = await api("/api/calibrations/stage", "POST", { stage: "precheck", id: v.active, run_dir: `${S.system.results}/${el.dataset.round}`, note: `adopted precheck of ${el.dataset.round}` }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "已启动"); },
  async "reload-runs"() { await loadRuns(); render(); },
  "export-field"(el) { const f = S.exportForm || (S.exportForm = {}); const k = el.dataset.key; f[k] = el.type === "checkbox" ? el.checked : el.tagName === "SELECT" ? el.value : parseFloat(el.value); },
  async "export-map"(el) { const round = el.dataset.round; await act(async () => { const job = await api(`/api/runs/${encodeURIComponent(round)}/export-map`, "POST", S.exportForm || {}); S.job = job; S.jobLog = []; S.jobLogNext = 0; }, "地图导出已启动（完成后刷新本页查看产品）"); },
  async "reload-jobs"() { await loadJobs(); render(); },
  async "reload-system"() { await loadOverview(); await A["results-usage"](); },
  async "results-usage"() { try { S.usage = await api("/api/results/usage"); } catch (e) { toast(String(e.message || e), "bad"); } render(); },
  "cleanup-field"(el) { const f = S.cleanupForm || (S.cleanupForm = {}); f[el.dataset.key] = el.type === "checkbox" ? el.checked : parseFloat(el.value); },
  async "results-pin"(el) { await act(() => api("/api/results/pin", "POST", { round: el.dataset.round, pinned: el.dataset.pinned === "1" }), el.dataset.pinned === "1" ? "已设为保留" : "已取消保留"); await A["results-usage"](); },
  async "results-cleanup"(el) {
    const action = el.dataset.action, round = el.dataset.round;
    if (!confirm(action === "delete" ? `删除整个轮次 ${round}？此操作不可恢复。` : `精简轮次 ${round}？将删除扫描存档与全分辨率分块（保留轨迹、评估与标准整图）。`)) return;
    await act(() => api("/api/results/cleanup", "POST", { action, rounds: [round] }), action === "delete" ? "已删除" : "已精简"); await A["results-usage"]();
  },
  async "results-auto"() {
    const f = S.cleanupForm || {};
    const plan = await api("/api/results/cleanup", "POST", { action: "auto", apply: false, target_free_gb: f.target || 100, allow_delete: !!f.allowDelete });
    if (!plan.plan.length) { toast("无需清理或没有可清理的轮次（受保护的轮次不会被动）", "ok"); return; }
    if (!confirm(`将按时间从旧到新处理 ${plan.plan.length} 个轮次，释放约 ${(plan.freed_bytes / 1e9).toFixed(1)} GB：\n` + plan.plan.map((p) => `${p.action === "delete" ? "删除" : "精简"} ${p.round}（${(p.bytes / 1e9).toFixed(1)} GB）`).join("\n"))) return;
    await act(() => api("/api/results/cleanup", "POST", { action: "auto", target_free_gb: f.target || 100, allow_delete: !!f.allowDelete }), "清理完成"); await A["results-usage"]();
  },
  async "fs-go"() { await A["fs-open"]({ dataset: { path: $("#fs-path").value.trim() } }); },
  async "fs-open"(el) { await act(async () => { S.fs = await api(`/api/fs?path=${encodeURIComponent(el.dataset.path)}`); const box = $(".fsb"); if (box) box.innerHTML = fsHtml(); const inp = $("#fs-path"); if (inp) inp.value = S.fs.path; }); },
  async "select-dataset"(el) { await act(async () => { const r = await api("/api/datasets/select", "POST", { path: el.dataset.path }); S.state = r.state; S.dataset = r.dataset; await loadOverview(); render(); }, "已选择数据源"); },
  async "forget-dataset"(el) { await act(async () => { S.state = await api("/api/datasets/forget", "POST", { path: el.dataset.path }); render(); }); },
  "ax-jump"(el) { const t = document.getElementById(el.dataset.target); if (t) t.scrollIntoView({ behavior: "smooth", block: "start" }); },
  "win-mode"(el) { const k = el.dataset.key; S[k].mode = el.dataset.mode; render(); },
  "win-field"(el) { const k = el.dataset.key, f = el.dataset.field; const v = el.value === "" ? "" : Number(el.value); S[k][f] = v; render(); },
  async assess() {
    const d = S.dataset; const v = vehicle();
    if (!d || d.error) return toast("请先选择可用的数据源", "bad");
    const w = winEffective(S.winAssess, d?.duration_s);
    await act(async () => { const job = await api("/api/datasets/assess", "POST", { path: d.path, mine: v?.mine, truck: v?.truck, window_start: w?.start_s ?? null, window_duration: w?.duration_s ?? null }); S.job = job; S.jobLog = []; S.jobLogNext = 0; S.drawer = "job"; render(); }, "数据评估已启动");
  },
  async "refresh-dataset"() { await act(async () => { S.dataset = await api(`/api/datasets?path=${encodeURIComponent(S.dataset.path)}&refresh=1`); render(); }, "已重新读取"); },
};

// ------------------------------------------------------------------ body-point visual filter
function bfKey(t) { return String(t || "").replace(/^\/+/, "").replace(/\//g, "_"); }
function bfNum(v) { return (v == null || v === "") ? "" : String(Math.round(Number(v) * 1000) / 1000); }
function bfNormalizeRect(r) {
  if (!r) return null;
  const x1 = Number(r.x_min), x2 = Number(r.x_max), y1 = Number(r.y_min), y2 = Number(r.y_max);
  if (![x1, x2, y1, y2].every(Number.isFinite)) return null;
  return { x_min: Math.min(x1, x2), x_max: Math.max(x1, x2), y_min: Math.min(y1, y2), y_max: Math.max(y1, y2) };
}
function bfUpdateFusedRectInputs() {
  const r = S.body && S.body.body_filter && S.body.body_filter.fused_rectangle;
  document.querySelectorAll('[data-act="bf-fused-rect"]').forEach((input) => {
    input.value = r ? bfNum(r[input.dataset.k]) : "";
  });
}
function bfBodyModeNote(f) {
  const rect = f && f.fused_rectangle;
  const legacyCount = Object.values((f && f.lidar_rectangles) || {}).reduce((n, rs) => n + (rs || []).length, 0);
  if (rect) return `<div class="callout ok">当前使用统一后轴坐标系矩形；保存后会自动移除旧版逐雷达矩形，并保留原文件备份。</div>`;
  if (legacyCount) return `<div class="bf-legacy-note small">当前版本仍使用旧版逐雷达多框（${legacyCount} 个）；红点是旧规则的预览。请在俯视图拖出一个统一矩形，红点将立即切换为新框的过滤预览；保存后自动替换旧多框。</div>`;
  return `<div class="callout warn">尚未设置统一矩形。切到俯视图，在点云上拖动一次即可绘制；红色点会即时显示将被过滤的点。</div>`;
}
function bfUpdateBodyModeNote() {
  const f = S.body && S.body.body_filter, note = document.getElementById("bf-mode-note");
  if (note) note.innerHTML = bfBodyModeNote(f);
  const clear = document.querySelector('[data-act="bf-clear-rect"]');
  if (clear) clear.disabled = !(f && f.fused_rectangle);
}
function bfDefaultRect() { return { x_min: -1.5, y_min: -1.5, x_max: 1.5, y_max: 1.5 }; }
function bfYamlOverrides(f) {
  const bf = { enabled: !!f.enabled, minimum_z: Number(f.minimum_z) };
  if (f.fused_rectangle) {
    const r = f.fused_rectangle;
    bf.fused_rectangle = [Number(r.x_min), Number(r.y_min), Number(r.x_max), Number(r.y_max)];
  } else {
    const std = ["front_right_lidar", "back_lidar", "front_left_lidar", "front_lidar"];
    const keys = Array.from(new Set([...std, ...Object.keys(f.lidar_rectangles || {})]));
    keys.forEach((k) => {
      const flat = [];
      (f.lidar_rectangles[k] || []).forEach((r) => flat.push(Number(r.x_min), Number(r.y_min), Number(r.x_max), Number(r.y_max)));
      bf[k] = flat;
    });
  }
  const b = f.rear_axle_box || { x_min: -2.8, x_max: 2.8, y_min: -4.5, y_max: 9.5 };
  return { body_filter: bf, precheck: { body_box: [Number(b.x_min), Number(b.x_max), Number(b.y_min), Number(b.y_max)] } };
}
function bfPointInPoly(x, y, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / ((yj - yi) || 1e-9) + xi)) inside = !inside;
  }
  return inside;
}
function bfRecomputeMulti() {
  const b = S.body, f = b && b.body_filter, m = b && b.multi_scan;
  if (!f || !m || !m.series) return;
  const minz = Number(f.minimum_z), enabled = !!f.enabled;
  m.series.forEach((s) => {
    const points = s.points || [];
    if (f.fused_rectangle) {
      const r = f.fused_rectangle;
      s.filtered = points.map((p) => enabled && p[0] > r.x_min && p[0] < r.x_max &&
        p[1] > r.y_min && p[1] < r.y_max && p[2] > minz);
    } else {
      const local = s.points_local || [];
      if (local.length !== points.length) return;
      const rects = (f.lidar_rectangles || {})[s.topic_key] || [];
      s.filtered = local.map((p) => {
        const inside = rects.some((r) => p[0] > r.x_min && p[0] < r.x_max && p[1] > r.y_min && p[1] < r.y_max);
        return enabled && inside && p[2] > minz;
      });
    }
    s.filtered_count = s.filtered.filter(Boolean).length;
  });
  m.filtered_count = m.series.reduce((n, s) => n + (s.filtered_count || 0), 0);
  b.yaml_overrides = bfYamlOverrides(f);
  const c = document.getElementById("bf-all-count");
  if (c) c.innerHTML = `${f.fused_rectangle ? "统一矩形" : "旧版多框"}: <b>${m.filtered_count}</b> / ${m.sampled_points || 0} 点（原始 ${m.raw_points || 0} 点）`;
  const pre = document.getElementById("bf-yaml");
  if (pre) pre.textContent = JSON.stringify(b.yaml_overrides, null, 2);
}
function bfRecompute() {
  const b = S.body, f = b && b.body_filter, s = b && b.scan;
  if (!f || !s) return;
  const rects = (f.lidar_rectangles || {})[b.topic_key] || [];
  const minz = Number(f.minimum_z), enabled = !!f.enabled;
  let count = 0;
  s.filtered = (s.points || []).map((p) => {
    const inside = rects.some((r) => p[0] > r.x_min && p[0] < r.x_max && p[1] > r.y_min && p[1] < r.y_max);
    const hit = enabled && inside && p[2] > minz;
    if (hit) count++;
    return hit;
  });
  s.filtered_count = count;
  b.yaml_overrides = bfYamlOverrides(f);
  const c = document.getElementById("bf-filtered-count");
  if (c) c.innerHTML = `当前样本过滤 <b>${count}</b> / ${(s.points || []).length} 点（原始 ${s.raw_points || 0} 点）`;
  const pre = document.getElementById("bf-yaml");
  if (pre) pre.textContent = JSON.stringify(b.yaml_overrides, null, 2);
  bfRecomputeMulti();
}
function bfX2p(x, b, w) { return (x - b.x_min) / (b.x_max - b.x_min) * w; }
function bfY2p(y, b, h) { return h - (y - b.y_min) / (b.y_max - b.y_min) * h; }
function bfP2x(px, b, w) { return b.x_min + px / w * (b.x_max - b.x_min); }
function bfP2y(py, b, h) { return b.y_min + (h - py) / h * (b.y_max - b.y_min); }
function bfCanvasSize(c) { return { w: Math.max(320, c.clientWidth || 600), h: Math.max(220, c.clientHeight || 320) }; }
function bfSetupCanvas(c) {
  const { w, h } = bfCanvasSize(c), dpr = window.devicePixelRatio || 1;
  const W = Math.round(w * dpr), H = Math.round(h * dpr);
  if (c.width !== W || c.height !== H) { c.width = W; c.height = H; }
  const ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h };
}
function bfCanvasPoint(ev, c) { const r = c.getBoundingClientRect(); return { x: ev.clientX - r.left, y: ev.clientY - r.top }; }
function bfGrid(ctx, w, h, b, label) {
  ctx.fillStyle = "#fbfbfd"; ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#ececf0"; ctx.lineWidth = 1;
  for (let i = 1; i < 10; i++) {
    const x = w * i / 10, y = h * i / 10;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  if (b.x_min < 0 && b.x_max > 0) { const x = bfX2p(0, b, w); ctx.strokeStyle = "#d2d2d7"; ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); }
  if (b.y_min < 0 && b.y_max > 0) { const y = bfY2p(0, b, h); ctx.strokeStyle = "#d2d2d7"; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
  ctx.fillStyle = "#8e8e93"; ctx.font = "11px sans-serif";
  ctx.fillText(label, 8, 15);
  ctx.fillText(`x ${b.x_min.toFixed(1)}..${b.x_max.toFixed(1)} m`, 8, h - 8);
  ctx.textAlign = "right"; ctx.fillText(`y ${b.y_min.toFixed(1)}..${b.y_max.toFixed(1)} m`, w - 8, h - 8); ctx.textAlign = "left";
}
function bfDrawRect(ctx, r, b, w, h, selected, draft) {
  const x1 = bfX2p(r.x_min, b, w), x2 = bfX2p(r.x_max, b, w);
  const y1 = bfY2p(r.y_max, b, h), y2 = bfY2p(r.y_min, b, h);
  ctx.fillStyle = draft ? "rgba(255,159,10,.18)" : selected ? "rgba(0,113,227,.12)" : "rgba(10,132,255,.07)";
  ctx.fillRect(x1, y1, x2 - x1, y2 - y1);
  ctx.strokeStyle = draft ? "#ff9f0a" : selected ? "#0071e3" : "#0a84ff";
  ctx.lineWidth = selected || draft ? 2 : 1.2;
  ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
  if (selected && !draft) {
    ctx.fillStyle = "#0071e3";
    [[x1, y1], [x2, y1], [x1, y2], [x2, y2], [(x1 + x2) / 2, y1], [(x1 + x2) / 2, y2], [x1, (y1 + y2) / 2], [x2, (y1 + y2) / 2]].forEach(([x, y]) => ctx.fillRect(x - 3, y - 3, 6, 6));
  }
}
function bfDrawPoints(ctx, s, b, w, h, axis) {
  const pts = s.points || [], fil = s.filtered || [];
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i], x = p[0], y = axis === "xy" ? p[1] : p[2];
    if (x < b.x_min || x > b.x_max || y < b.y_min || y > b.y_max) continue;
    ctx.fillStyle = fil[i] ? "rgba(255,59,48,.75)" : "rgba(0,113,227,.30)";
    const px = bfX2p(x, b, w), py = bfY2p(y, b, h);
    ctx.fillRect(px - 1, py - 1, 2, 2);
  }
}
function bfHitRect(rects, pt, b, w, h) {
  const tol = 8;
  for (let i = rects.length - 1; i >= 0; i--) {
    const r = rects[i], x1 = bfX2p(r.x_min, b, w), x2 = bfX2p(r.x_max, b, w);
    const y1 = bfY2p(r.y_max, b, h), y2 = bfY2p(r.y_min, b, h), xm = (x1 + x2) / 2, ym = (y1 + y2) / 2;
    const handles = [
      ["x_min_y_max", x1, y1], ["x_max_y_max", x2, y1], ["x_min_y_min", x1, y2], ["x_max_y_min", x2, y2],
      ["y_max", xm, y1], ["y_min", xm, y2], ["x_min", x1, ym], ["x_max", x2, ym],
    ];
    for (const [k, x, y] of handles) if (Math.abs(pt.x - x) <= tol && Math.abs(pt.y - y) <= tol) return { i, handle: k };
  }
  for (let i = rects.length - 1; i >= 0; i--) {
    const r = rects[i], x1 = bfX2p(r.x_min, b, w), x2 = bfX2p(r.x_max, b, w);
    const y1 = bfY2p(r.y_max, b, h), y2 = bfY2p(r.y_min, b, h);
    if (pt.x >= x1 && pt.x <= x2 && pt.y >= y1 && pt.y <= y2) return { i, handle: null };
  }
  return { i: null, handle: null };
}
function bfResizeRect(orig, handle, wx, wy) {
  const r = { ...orig };
  if (handle.includes("x_min")) r.x_min = wx;
  if (handle.includes("x_max")) r.x_max = wx;
  if (handle.includes("y_min")) r.y_min = wy;
  if (handle.includes("y_max")) r.y_max = wy;
  return bfNormalizeRect(r) || r;
}
function bfRearBounds(f) {
  const b = (f && f.rear_axle_box) || { x_min: -2.8, x_max: 2.8, y_min: -4.5, y_max: 9.5 };
  return { x_min: Math.min(-8, b.x_min - 2), x_max: Math.max(8, b.x_max + 2), y_min: Math.min(-10, b.y_min - 2), y_max: Math.max(12, b.y_max + 2) };
}
function bfDrawXY() {
  const c = document.getElementById("bf-xy"), b = S.body;
  if (!c || !b || !b.scan || !b.body_filter) return;
  const { ctx, w, h } = bfSetupCanvas(c), vb = b.scan.xy;
  bfGrid(ctx, w, h, vb, `XY 俯视 · ${b.topic}`);
  bfDrawPoints(ctx, b.scan, vb, w, h, "xy");
  const rects = b.body_filter.lidar_rectangles[b.topic_key] || [], drag = S.bodyForm.drag;
  rects.forEach((r, i) => {
    if (drag && drag.i === i && drag.mode !== "draw") return;
    bfDrawRect(ctx, r, vb, w, h, S.bodyForm.sel === i, false);
  });
  if (drag && drag.rect) bfDrawRect(ctx, drag.rect, vb, w, h, true, true);
}
function bfDrawXZ() {
  const c = document.getElementById("bf-xz"), b = S.body;
  if (!c || !b || !b.scan || !b.body_filter) return;
  const { ctx, w, h } = bfSetupCanvas(c), vb = b.scan.xz;
  bfGrid(ctx, w, h, vb, "XZ 侧视 · 横轴 X / 纵轴 Z");
  bfDrawPoints(ctx, b.scan, vb, w, h, "xz");
  const z = Number(b.body_filter.minimum_z), py = bfY2p(z, vb, h);
  ctx.strokeStyle = "#ff3b30"; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]);
  ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = "#ff3b30"; ctx.font = "11px sans-serif"; ctx.fillText(`minimum_z = ${z.toFixed(2)} m`, 8, Math.max(14, py - 5));
}
function bfDrawRear() {
  const c = document.getElementById("bf-rear"), b = S.body;
  if (!c || !b || !b.body_filter) return;
  const { ctx, w, h } = bfSetupCanvas(c), vb = bfRearBounds(b.body_filter);
  bfGrid(ctx, w, h, vb, "后轴 XY 框（网格示意）");
  const drag = S.bodyForm.rearDrag;
  if (!drag || drag.mode === "draw") bfDrawRect(ctx, b.body_filter.rear_axle_box, vb, w, h, false, false);
  if (drag && drag.rect) bfDrawRect(ctx, drag.rect, vb, w, h, true, true);
  ctx.fillStyle = "#8e8e93"; ctx.font = "11px sans-serif"; ctx.fillText("+Y 车头方向", 8, 30);
}
function bfDrawAllLidars() {
  const c = document.getElementById("bf-all"), b = S.body, m = b && b.multi_scan;
  if (!c || !m) return;
  const { ctx, w, h } = bfSetupCanvas(c), vb = m.xy || { x_min: -12, x_max: 12, y_min: -12, y_max: 12 };
  bfGrid(ctx, w, h, vb, "多雷达拼接俯视 · 后轴坐标系");
  (m.series || []).forEach((s) => {
    const pts = s.points || [], filt = s.filtered || [];
    ctx.save();
    ctx.globalAlpha = 0.78; ctx.fillStyle = s.color || "#0071e3";
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i], x = bfX2p(p[0], vb, w), y = bfY2p(p[1], vb, h);
      ctx.fillRect(x - 1, y - 1, 2, 2);
    }
    ctx.restore();
    ctx.save();
    ctx.fillStyle = "rgba(255,59,48,.92)";
    for (let i = 0; i < pts.length; i++) {
      if (!filt[i]) continue;
      const p = pts[i], x = bfX2p(p[0], vb, w), y = bfY2p(p[1], vb, h);
      ctx.fillRect(x - 1.6, y - 1.6, 3.2, 3.2);
    }
    ctx.restore();
    (s.rectangles || []).forEach((r) => {
      const poly = r.polygon || []; if (poly.length < 3) return;
      ctx.save();
      ctx.beginPath();
      poly.forEach(([x, y], i) => {
        const px = bfX2p(x, vb, w), py = bfY2p(y, vb, h);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.closePath();
      ctx.strokeStyle = s.color || "#0a84ff"; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.globalAlpha = 0.10; ctx.fillStyle = s.color || "#0a84ff"; ctx.fill();
      ctx.restore();
    });
  });
  if (m.rear_axle_box) {
    const r = m.rear_axle_box;
    ctx.save();
    ctx.strokeStyle = "#8e8e93"; ctx.lineWidth = 1.2; ctx.setLineDash([5, 4]);
    ctx.strokeRect(bfX2p(r.x_min, vb, w), bfY2p(r.y_max, vb, h),
                   bfX2p(r.x_max, vb, w) - bfX2p(r.x_min, vb, w),
                   bfY2p(r.y_min, vb, h) - bfY2p(r.y_max, vb, h));
    ctx.restore();
  }
  ctx.fillStyle = "#8e8e93"; ctx.font = "11px sans-serif";
  ctx.fillText("+Y 车头方向 · 虚线＝后轴车身框", 8, 30);
}
function bfDrawAll() { bfDrawXY(); bfDrawXZ(); bfDrawRear(); bfDrawAllLidars(); if (S.bodyForm.viewer) S.bodyForm.viewer.draw(); }
function bfAttachXY(c) {
  c.addEventListener("pointerdown", (ev) => {
    const b = S.body; if (!b || !b.scan || !b.body_filter) return;
    const { w, h } = bfCanvasSize(c), pt = bfCanvasPoint(ev, c), vb = b.scan.xy;
    const wx = bfP2x(pt.x, vb, w), wy = bfP2y(pt.y, vb, h);
    const rects = b.body_filter.lidar_rectangles[b.topic_key] || (b.body_filter.lidar_rectangles[b.topic_key] = []);
    const hit = bfHitRect(rects, pt, vb, w, h);
    let drag;
    if (hit.handle) drag = { mode: "resize", i: hit.i, handle: hit.handle, orig: { ...rects[hit.i] }, rect: { ...rects[hit.i] }, start: { wx, wy } };
    else if (hit.i != null) drag = { mode: "move", i: hit.i, orig: { ...rects[hit.i] }, rect: { ...rects[hit.i] }, start: { wx, wy } };
    else drag = { mode: "draw", rect: { x_min: wx, y_min: wy, x_max: wx, y_max: wy }, start: { wx, wy } };
    S.bodyForm.drag = drag; S.bodyForm.sel = hit.i;
    c.setPointerCapture(ev.pointerId); bfDrawXY();
  });
  c.addEventListener("pointermove", (ev) => {
    const drag = S.bodyForm.drag, b = S.body; if (!drag || !b) return;
    const { w, h } = bfCanvasSize(c), pt = bfCanvasPoint(ev, c), vb = b.scan.xy;
    const wx = bfP2x(pt.x, vb, w), wy = bfP2y(pt.y, vb, h);
    if (drag.mode === "draw") drag.rect = bfNormalizeRect({ x_min: drag.start.wx, y_min: drag.start.wy, x_max: wx, y_max: wy });
    else if (drag.mode === "move") {
      const dx = wx - drag.start.wx, dy = wy - drag.start.wy;
      drag.rect = { x_min: drag.orig.x_min + dx, x_max: drag.orig.x_max + dx, y_min: drag.orig.y_min + dy, y_max: drag.orig.y_max + dy };
    } else drag.rect = bfResizeRect(drag.orig, drag.handle, wx, wy);
    bfDrawXY();
  });
  const finish = () => {
    const drag = S.bodyForm.drag, b = S.body; if (!drag || !b) return;
    S.bodyForm.drag = null;
    const rects = b.body_filter.lidar_rectangles[b.topic_key] || (b.body_filter.lidar_rectangles[b.topic_key] = []);
    const r = bfNormalizeRect(drag.rect);
    if (r && r.x_max - r.x_min > 0.05 && r.y_max - r.y_min > 0.05) {
      if (drag.mode === "draw") { rects.push(r); S.bodyForm.sel = rects.length - 1; }
      else { rects[drag.i] = r; S.bodyForm.sel = drag.i; }
    }
    bfRecompute(); render();
  };
  c.addEventListener("pointerup", finish);
  c.addEventListener("pointercancel", () => { S.bodyForm.drag = null; bfDrawXY(); });
}
function bfAttachXZ(c) {
  const setZ = (ev) => {
    const b = S.body; if (!b || !b.body_filter || !b.scan) return;
    const { h } = bfCanvasSize(c), pt = bfCanvasPoint(ev, c);
    b.body_filter.minimum_z = Math.round(bfP2y(pt.y, b.scan.xz, h) * 100) / 100;
    bfRecompute(); bfDrawAll();
    const inp = document.querySelector('[data-act="bf-global"][data-k="minimum_z"]'); if (inp) inp.value = b.body_filter.minimum_z;
  };
  c.addEventListener("pointerdown", (ev) => { S.bodyForm.zDrag = true; c.setPointerCapture(ev.pointerId); setZ(ev); });
  c.addEventListener("pointermove", (ev) => { if (S.bodyForm.zDrag) setZ(ev); });
  c.addEventListener("pointerup", () => { S.bodyForm.zDrag = false; });
  c.addEventListener("pointercancel", () => { S.bodyForm.zDrag = false; });
}
function bfAttachRear(c) {
  c.addEventListener("pointerdown", (ev) => {
    const b = S.body; if (!b || !b.body_filter) return;
    const { w, h } = bfCanvasSize(c), pt = bfCanvasPoint(ev, c), vb = bfRearBounds(b.body_filter);
    const wx = bfP2x(pt.x, vb, w), wy = bfP2y(pt.y, vb, h), orig = b.body_filter.rear_axle_box;
    const hit = bfHitRect([orig], pt, vb, w, h);
    let drag;
    if (hit.handle) drag = { mode: "resize", handle: hit.handle, orig: { ...orig }, rect: { ...orig }, start: { wx, wy } };
    else if (hit.i != null) drag = { mode: "move", orig: { ...orig }, rect: { ...orig }, start: { wx, wy } };
    else drag = { mode: "draw", rect: { x_min: wx, y_min: wy, x_max: wx, y_max: wy }, start: { wx, wy } };
    S.bodyForm.rearDrag = drag; c.setPointerCapture(ev.pointerId); bfDrawRear();
  });
  c.addEventListener("pointermove", (ev) => {
    const drag = S.bodyForm.rearDrag, b = S.body; if (!drag || !b) return;
    const { w, h } = bfCanvasSize(c), pt = bfCanvasPoint(ev, c), vb = bfRearBounds(b.body_filter);
    const wx = bfP2x(pt.x, vb, w), wy = bfP2y(pt.y, vb, h);
    if (drag.mode === "draw") drag.rect = bfNormalizeRect({ x_min: drag.start.wx, y_min: drag.start.wy, x_max: wx, y_max: wy });
    else if (drag.mode === "move") {
      const dx = wx - drag.start.wx, dy = wy - drag.start.wy;
      drag.rect = { x_min: drag.orig.x_min + dx, x_max: drag.orig.x_max + dx, y_min: drag.orig.y_min + dy, y_max: drag.orig.y_max + dy };
    } else drag.rect = bfResizeRect(drag.orig, drag.handle, wx, wy);
    bfDrawRear();
  });
  const finish = () => {
    const drag = S.bodyForm.rearDrag, b = S.body; if (!drag || !b) return;
    S.bodyForm.rearDrag = null;
    const r = bfNormalizeRect(drag.rect);
    if (r && r.x_max - r.x_min > 0.1 && r.y_max - r.y_min > 0.1) b.body_filter.rear_axle_box = r;
    bfRecompute(); render();
  };
  c.addEventListener("pointerup", finish);
  c.addEventListener("pointercancel", () => { S.bodyForm.rearDrag = null; bfDrawRear(); });
}
function initBodyCanvases() {
  const canvas = document.getElementById("bf-scene");
  if (!canvas || !window.LivoBodyViewer) { S.bodyForm.viewer = null; return; }
  S.bodyForm.viewer = window.LivoBodyViewer.attach(canvas, () => {
    const body = S.body, filter = body && body.body_filter, scan = body && body.multi_scan;
    return { series: (scan && scan.series) || [], bounds: scan && scan.xy,
      rectangle: filter && filter.fused_rectangle, minimum_z: filter && filter.minimum_z,
      enabled: !(filter && filter.enabled === false) };
  }, (rectangle) => {
    const filter = S.body && S.body.body_filter; if (!filter) return;
    filter.fused_rectangle = rectangle;
    bfUpdateFusedRectInputs();
    bfRecomputeMulti();
    bfUpdateBodyModeNote();
  }, S.bodyForm.view || "top");
  document.querySelectorAll(".bf-view-btn").forEach((button) => {
    button.classList.toggle("primary", button.dataset.view === (S.bodyForm.view || "top"));
  });
  bfDrawAll();
}
window.addEventListener("resize", () => { if (S.page === "body") bfDrawAll(); });

function pageBody() {
  const b = S.body, f = b && b.body_filter, v = vehicle(), d = S.dataset;
  const m = (b && b.multi_scan) || {}, series = m.series || [], rect = f && f.fused_rectangle;
  const parentVersions = bfParentVersions(), parentId = (b && b.parent_id) || bfCurrentParentId();
  const parentOptions = parentVersions.map((x) => `<option value="${esc(x.id)}" ${parentId === x.id ? "selected" : ""}>${esc(x.version)} · ${esc(x.stage_label)} · ${cameraLabel(x.camera)}${x.active ? " ★" : ""}</option>`).join("");
  const legacyCount = Object.values((f && f.lidar_rectangles) || {}).reduce((n, rs) => n + (rs || []).length, 0);
  const err = (b && b.error) || (m && m.error), dur = (b && b.duration_s) || 0;
  const modeNote = bfBodyModeNote(f);
  return `
  <div class="card">
    <div class="card-h"><h2>车身点云可视化过滤</h2><div class="row">
      <button class="btn sm" data-act="bf-reload" ${S.bodyBusy ? "disabled" : ""}>重新加载</button>
      <button class="btn sm primary" data-act="bf-save" ${S.bodyBusy || !f ? "disabled" : ""}>保存唯一矩形到标定文件</button>
    </div></div>
    <div class="small muted">所有 LiDAR 先按所选父版本外参拼接到后轴坐标系，再使用一个 XY 矩形和 minimum_z 过滤；红色点是将被删除的点。保存写入该车激活版本的 <code>calibration.json:body_filter</code>，流水线会把它同步到所选父版本。</div>
    <div class="form" style="margin-top:12px">
      <div class="field"><label>车辆</label><select data-act="set-vehicle">${S.vehicles.map((x) => `<option value="${esc(x.id)}" ${v && x.id === v.id ? "selected" : ""}>${esc(x.mine)} / ${esc(x.truck)}</option>`).join("")}</select></div>
      <div class="field"><label>标定版本</label><input type="text" value="${esc((b && b.id) || (v && v.active) || "—")}" readonly></div>
      <div class="field wide"><label>标定文件</label><input type="text" class="mono small" value="${esc((b && b.calibration_file) || "")}" readonly></div>
      <div class="field"><label>父版本（拼接外参 / 自动化标定）</label><select data-act="bf-parent">${parentOptions || "<option>（无）</option>"}</select></div>
      <div class="field wide"><label>父版本标定文件</label><input type="text" class="mono small" value="${esc((b && b.parent_calibration_file) || "")}" readonly></div>
      <div class="field wide"><label>数据源</label><input type="text" class="mono small" value="${esc((b && b.dataset) || (d && d.path) || "")}" readonly></div>
      <div class="field"><label>整帧采样时刻（s，0–${bfNum(dur)}）</label><input type="number" min="0" max="${bfNum(dur)}" step="0.1" data-act="bf-param" data-key="t" value="${bfNum(m.t_s || 0)}"></div>
    </div>
    ${err ? `<div class="callout bad" style="margin-top:10px">${esc(err)}</div>` : ""}
  </div>

  <div class="card">
    <div class="card-h"><h2>全部 LiDAR 拼接整帧 · 后轴坐标系</h2><span class="small muted" id="bf-all-count">${rect ? "统一矩形" : "旧版多框"}: <b>${m.filtered_count || 0}</b> / ${m.sampled_points || 0} 点（原始 ${m.raw_points || 0} 点）</span></div>
    <div class="bf-scene-toolbar">
      <button class="btn sm bf-view-btn ${S.bodyForm.view !== "orbit" ? "primary" : ""}" data-act="bf-view" data-view="top">俯视 · XY</button>
      <button class="btn sm bf-view-btn ${S.bodyForm.view === "orbit" ? "primary" : ""}" data-act="bf-view" data-view="orbit">平视 / 360°环视 · Z-up</button>
      <button class="btn sm" data-act="bf-view-reset">复位视角</button>
      <button class="btn sm danger" data-act="bf-clear-rect" ${rect ? "" : "disabled"}>清除统一矩形</button>
      <span class="small muted">${m.sampled_points || 0} 点 · ${series.length} 路 LiDAR</span>
    </div>
    <canvas id="bf-scene" class="bf-scene"></canvas>
    <div class="bf-legend" id="bf-all-legend">
      ${series.length ? series.map((x) => `<span><i style="background:${esc(x.color)}"></i>${esc(x.topic)} (${x.sampled_points || 0})</span>`).join("") : '<span class="muted">暂无多雷达采样</span>'}
      <span><i class="drop"></i>将过滤点</span><span><i class="rect"></i>唯一过滤框</span>
    </div>
    <div class="small muted">请求时刻 ${bfNum(m.requested_t_s)} s · 实际整帧 ${bfNum(m.t_s)} s · header 时间跨度 ${bfNum(m.frame_time_span_ms)} ms（在线同步阈值 20 ms）</div>
    ${m.alignment_warning && m.frame_time_span_ms != null ? `<div class="callout warn small" style="margin-top:8px">${esc(m.alignment_warning)}</div>` : ""}
    <div class="bf-view-help small muted">俯视：滚轮围绕鼠标位置缩放；右键 / 中键拖动或 Shift+左键拖动平移；左键空白拖动新建框，框内拖动平移，四角或边线拖动缩放。平视/环视：左键水平拖动可绕世界 Z 轴连续 360°旋转，垂直拖动改变仰角且保持 Z-up；滚轮缩放，Shift+拖动平移。</div>
    <div id="bf-mode-note" style="margin-top:12px">${modeNote}</div>
    ${m.error ? `<div class="callout bad" style="margin-top:8px">${esc(m.error)}</div>` : ""}
  </div>

  <div class="card">
    <div class="bf-scene-fields">
      <label class="check"><input type="checkbox" data-act="bf-global" data-k="enabled" ${f && f.enabled ? "checked" : ""}><span class="switch"></span><span>启用车身过滤</span></label>
      ${["x_min", "x_max", "y_min", "y_max"].map((k) => `<div class="field"><label>统一框 ${k}（m）</label><input type="number" step="0.05" data-act="bf-fused-rect" data-k="${k}" value="${rect ? bfNum(rect[k]) : ""}" placeholder="俯视图拖动绘制"></div>`).join("")}
      <div class="field"><label>minimum_z（m）</label><input type="number" step="0.05" data-act="bf-global" data-k="minimum_z" value="${bfNum(f && f.minimum_z)}"></div>
    </div>
    <div class="small muted" style="margin-top:8px">坐标：后轴原点，X 横向、Y 朝车头、Z 向上。矩形按 fused 点的 X/Y 过滤，且仅删除 <code>z &gt; minimum_z</code> 的点；平视图显示该高度阈值和过滤棱柱。</div>
    <details style="margin-top:10px"><summary>生效参数与标定元数据</summary>
      <div class="kv small" style="margin:10px 0"><div class="k">来源</div><div>${esc((f && f.source) || "")}</div><div class="k">更新时间</div><div>${esc((f && f.updated) || "")}</div></div>
      <pre id="bf-yaml" class="small">${esc(JSON.stringify((b && b.yaml_overrides) || {}, null, 2))}</pre>
    </details>
  </div>`;
}

Object.assign(A, {
  async "bf-load"() {
    if (S.bodyBusy) return;
    const st = S.state || {}, v = st.vehicle || {};
    if (S.body && (S.body.mine !== v.mine || S.body.truck !== v.truck)) S.bodyForm.t = 0;
    S.bodyBusy = true; render();
    try {
      const q = new URLSearchParams({
        mine: v.mine || "", truck: v.truck || "", dataset: (st.data && st.data.dataset) || "",
        t: String(S.bodyForm.t || 0), max_points: String(S.bodyForm.max_points || 40000),
        parent_id: bfCurrentParentId(),
      });
      S.body = await api("/api/body-filter?" + q.toString());
      S.bodyForm.t = (S.body.multi_scan && S.body.multi_scan.t_s) || 0;
      if (S.body && S.body.parent_id) S.state.calibration.parent_id = S.body.parent_id;
    } catch (e) {
      S.body = { error: e.message || String(e) };
      toast(S.body.error, "bad");
    }
    S.bodyBusy = false; render();
  },
  "bf-param"(el) {
    const k = el.dataset.key;
    S.bodyForm[k] = k === "t" ? Number(el.value || 0) : el.value;
    A["bf-load"]();
  },
  "bf-reload"() { A["bf-load"](); },
  async "bf-save"() {
    const b = S.body; if (!b || !b.body_filter) return toast("尚未加载车身过滤参数", "bad");
    await act(async () => {
      const r = await api("/api/body-filter", "POST", {
        mine: b.mine, truck: b.truck, id: b.id, source: "Studio 融合整帧单矩形车身过滤",
        body_filter: b.body_filter,
      });
      toast(`已保存到 ${r.calibration_file}`, "ok");
      await A["bf-load"]();
    });
  },
  "bf-addrect"() {
    const b = S.body, f = b && b.body_filter; if (!f) return;
    const key = b.topic_key, rects = f.lidar_rectangles[key] || (f.lidar_rectangles[key] = []);
    rects.push(bfDefaultRect()); S.bodyForm.sel = rects.length - 1; bfRecompute(); render();
  },
  "bf-delrect"(el) {
    const b = S.body, f = b && b.body_filter; if (!f) return;
    const rects = f.lidar_rectangles[b.topic_key] || [];
    rects.splice(Number(el.dataset.i), 1); S.bodyForm.sel = null; bfRecompute(); render();
  },
  "bf-rect"(el) {
    const b = S.body, f = b && b.body_filter; if (!f) return;
    const rects = f.lidar_rectangles[b.topic_key] || [], r = rects[Number(el.dataset.i)];
    if (!r) return;
    r[el.dataset.k] = Number(el.value); const n = bfNormalizeRect(r);
    if (n) rects[Number(el.dataset.i)] = n;
    S.bodyForm.sel = Number(el.dataset.i); bfRecompute(); bfDrawAll();
  },
  "bf-global"(el) {
    const f = S.body && S.body.body_filter; if (!f) return;
    if (el.dataset.k === "enabled") f.enabled = el.checked;
    else if (el.dataset.k === "minimum_z") f.minimum_z = Number(el.value);
    bfRecomputeMulti(); bfDrawAll();
  },
  "bf-fused-rect"() {
    const f = S.body && S.body.body_filter; if (!f) return;
    const values = {};
    for (const key of ["x_min", "x_max", "y_min", "y_max"]) {
      const input = document.querySelector(`[data-act="bf-fused-rect"][data-k="${key}"]`);
      if (!input || input.value === "") return;
      values[key] = Number(input.value);
    }
    const rect = bfNormalizeRect(values);
    if (!rect || rect.x_max - rect.x_min < 0.05 || rect.y_max - rect.y_min < 0.05) return toast("统一矩形宽和长都必须大于 0.05 m", "bad");
    f.fused_rectangle = rect;
    bfUpdateFusedRectInputs(); bfRecomputeMulti(); bfDrawAll();
    bfUpdateBodyModeNote();
  },
  "bf-clear-rect"() {
    const f = S.body && S.body.body_filter; if (!f) return;
    f.fused_rectangle = null;
    f.lidar_rectangles = Object.fromEntries(["front_right_lidar", "back_lidar", "front_left_lidar", "front_lidar"].map((k) => [k, []]));
    bfRecomputeMulti(); render();
  },
  "bf-view"(el) {
    S.bodyForm.view = el.dataset.view === "orbit" ? "orbit" : "top";
    if (S.bodyForm.viewer) S.bodyForm.viewer.setView(S.bodyForm.view);
    document.querySelectorAll(".bf-view-btn").forEach((button) => button.classList.toggle("primary", button.dataset.view === S.bodyForm.view));
  },
  "bf-view-reset"() {
    if (S.bodyForm.viewer) S.bodyForm.viewer.reset();
  },
  "bf-rear"(el) {
    const f = S.body && S.body.body_filter; if (!f) return;
    f.rear_axle_box[el.dataset.k] = Number(el.value);
    const n = bfNormalizeRect(f.rear_axle_box); if (n) f.rear_axle_box = n;
    bfRecompute(); bfDrawAll();
  },
  "bf-parent"(el) {
    S.state.calibration.parent_id = el.value || "";
    saveState({ calibration: { parent_id: S.state.calibration.parent_id } });
    A["bf-load"]();
  },
  "bf-focus-topic"(el) {
    S.bodyForm.topic = el.dataset.topic || "";
    A["bf-load"]();
  },
});

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
    else if (S.page === "mapping" && currentRound()) await pollLive();
    render();
    if (S.page === "data") A["fs-open"]({ dataset: { path: S.dataset?.path ? S.dataset.path.replace(/\/[^/]+$/, "") : "/home/project" } });
    if (S.page === "system") A["results-usage"]();
    if (S.page === "body") A["bf-load"]();
    schedule();
  } catch (e) {
    $("#page").innerHTML = `<div class="callout bad">无法连接 Studio 后端：${esc(e.message)}</div>`;
  }
})();
