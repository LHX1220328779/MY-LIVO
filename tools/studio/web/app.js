/* LIVO Studio front-end (no build step, no external dependencies). */
const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const fmt = (v, d = 3) => (v === null || v === undefined || v === "" || Number.isNaN(+v)) ? "–" : (+v).toFixed(d);
const cm = (m) => (m === null || m === undefined || Number.isNaN(+m)) ? "–" : (+m * 100).toFixed(1) + " cm";

async function api(path, opts = {}) {
  const r = await fetch(path, { headers: { "Content-Type": "application/json" }, ...opts,
    body: opts.body !== undefined ? JSON.stringify(opts.body) : undefined });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
function toast(msg, kind = "") {
  const el = document.createElement("div");
  el.className = "toast " + kind; el.textContent = msg;
  $("#toast-root").appendChild(el);
  setTimeout(() => el.remove(), 4200);
}

const S = { state: null, page: "overview", calib: null, bags: [], runs: [], jobs: [], sel: { calib: null, run: null, job: null },
  mappingJob: null, timers: {}, chart: {} };

// ----------------------------------------------------------------------------- state persistence
let saveTimer = null;
function persist(section, patch) {
  Object.assign(S.state[section], patch);
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => api("/api/state", { method: "POST", body: { [section]: S.state[section] } }).catch((e) => toast(e.message, "bad")), 400);
}
function bind(section, root) {
  $$("[data-k]", root).forEach((el) => {
    const k = el.dataset.k; const v = S.state[section][k];
    if (el.type === "checkbox") el.checked = !!v; else if (v !== undefined) el.value = v;
    el.addEventListener(el.tagName === "SELECT" || el.type === "checkbox" ? "change" : "input", () => {
      persist(section, { [k]: el.type === "checkbox" ? el.checked : (el.type === "number" ? +el.value : el.value) });
    });
  });
}

// ----------------------------------------------------------------------------- modal / file picker
function modal(title, bodyHtml, footHtml = "") {
  const root = $("#modal-root");
  root.innerHTML = `<div class="modal-bg"><div class="modal"><header><span>${esc(title)}</span><button class="btn sm ghost" id="m-close">关闭</button></header>
    <div class="body">${bodyHtml}</div><footer>${footHtml}</footer></div></div>`;
  $("#m-close").onclick = closeModal;
  $(".modal-bg").addEventListener("click", (e) => { if (e.target.classList.contains("modal-bg")) closeModal(); });
  return root;
}
function closeModal() { $("#modal-root").innerHTML = ""; }

function pickPath(start, { dirsOnly = false, onPick }) {
  let cur = start || "/home/project"; let selected = null;
  const render = async () => {
    let d;
    try { d = await api("/api/fs?path=" + encodeURIComponent(cur)); } catch (e) { toast(e.message, "bad"); return; }
    cur = d.path;
    const rows = d.entries.filter((e) => !dirsOnly || e.dir).map((e) =>
      `<div class="fs-entry ${selected === e.path ? "sel" : ""}" data-p="${esc(e.path)}" data-dir="${e.dir ? 1 : 0}">
        <span>${e.dir ? (e.bag ? "🎞️" : "📁") : "📄"}</span><span class="n">${esc(e.name)}${e.bag ? ' <span class="badge blue">bag</span>' : ""}</span>
        <span class="s">${e.dir ? "" : (e.size / 1e6).toFixed(1) + " MB"}</span></div>`).join("");
    modal("选择路径", `<div class="row" style="margin-bottom:8px"><button class="btn sm" id="fs-up">↑ 上级</button>
        <input type="text" id="fs-cur" value="${esc(cur)}" style="flex:1"></div><div id="fs-list">${rows || '<div class="small">空目录</div>'}</div>`,
      `<span class="small" style="flex:1;align-self:center">${dirsOnly ? "双击进入目录，单击选中目录" : "单击选中文件，双击进入目录"}</span>
       <button class="btn" id="fs-use-cur">使用当前目录</button><button class="btn primary" id="fs-ok" ${selected ? "" : "disabled"}>选择</button>`);
    $("#fs-up").onclick = () => { cur = d.parent; selected = null; render(); };
    $("#fs-cur").onkeydown = (e) => { if (e.key === "Enter") { cur = e.target.value; selected = null; render(); } };
    $("#fs-use-cur").onclick = () => { onPick(cur); closeModal(); };
    $("#fs-ok").onclick = () => { if (selected) { onPick(selected); closeModal(); } };
    $$("#fs-list .fs-entry").forEach((el) => {
      el.onclick = () => { if (dirsOnly || el.dataset.dir === "0") { selected = el.dataset.p; $$("#fs-list .fs-entry").forEach((x) => x.classList.toggle("sel", x === el)); $("#fs-ok").disabled = false; } };
      el.ondblclick = () => { if (el.dataset.dir === "1") { cur = el.dataset.p; selected = null; render(); } };
    });
  };
  render();
}
function pathField(id, section, key, { dirs = true, placeholder = "" } = {}) {
  return `<div class="path-field"><input type="text" id="${id}" data-k="${key}" placeholder="${esc(placeholder)}">
    <button class="btn" data-pick="${id}" data-dirs="${dirs ? 1 : 0}" data-sec="${section}" data-key="${key}">浏览</button></div>`;
}
function wirePickers(root) {
  $$("[data-pick]", root).forEach((b) => {
    b.onclick = () => {
      const input = $("#" + b.dataset.pick);
      pickPath(input.value || "/home/project", { dirsOnly: b.dataset.dirs === "1", onPick: (p) => { input.value = p; persist(b.dataset.sec, { [b.dataset.key]: p }); input.dispatchEvent(new Event("change")); } });
    };
  });
}

// ----------------------------------------------------------------------------- job log viewer
function logHtml(lines) {
  return lines.map((l) => {
    let cls = "";
    if (/error|fatal|failed|traceback/i.test(l)) cls = "err"; else if (/warn/i.test(l)) cls = "warn";
    else if (/PASS|applied|DONE|written|finished|listening/i.test(l)) cls = "ok"; else if (/Extrinsic precheck|\[optimized\]|\[native\]/.test(l)) cls = "info";
    return `<span class="${cls}">${esc(l)}</span>`;
  }).join("\n");
}
function watchJob(jobId, el, { onDone } = {}) {
  let offset = 0; el.innerHTML = ""; let done = false;
  const tick = async () => {
    try {
      const j = await api(`/api/jobs/${jobId}?offset=${offset}`);
      if (j.log.length) { el.insertAdjacentHTML("beforeend", (offset ? "\n" : "") + logHtml(j.log)); offset += j.log.length; el.scrollTop = el.scrollHeight; }
      if (!j.alive && j.finished) {
        if (!done) { done = true; el.insertAdjacentHTML("beforeend", `\n<span class="${j.returncode === 0 ? "ok" : "err"}">── 结束，退出码 ${j.returncode} ──</span>`); el.scrollTop = el.scrollHeight; onDone && onDone(j); }
        return;
      }
    } catch (e) { el.insertAdjacentHTML("beforeend", `\n<span class="err">${esc(e.message)}</span>`); return; }
    S.timers["job" + jobId] = setTimeout(tick, 1000);
  };
  clearTimeout(S.timers["job" + jobId]); tick();
}

// ----------------------------------------------------------------------------- charts
function drawChart(canvas, series, { color = "#0a84ff", color2 = "#ff9f0a", yLabel = "", threshold = null, unit = 1, key = "v", key2 = null } = {}) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 600, h = canvas.clientHeight || 180;
  canvas.width = w * dpr; canvas.height = h * dpr;
  const ctx = canvas.getContext("2d"); ctx.scale(dpr, dpr); ctx.clearRect(0, 0, w, h);
  if (!series || series.length < 2) { ctx.fillStyle = "#8e8e93"; ctx.font = "12px -apple-system, sans-serif"; ctx.fillText("等待数据…", 12, 20); return; }
  const pad = { l: 44, r: 10, t: 10, b: 20 };
  const xs = series.map((p) => p.t), t0 = xs[0], t1 = xs[xs.length - 1] || t0 + 1;
  const vals = series.flatMap((p) => [p[key] * unit, key2 ? p[key2] * unit : p[key] * unit]).filter((v) => Number.isFinite(v));
  let ymax = Math.max(...vals, threshold ? threshold * 1.2 : 0) * 1.1 || 1; const ymin = 0;
  const X = (t) => pad.l + (t - t0) / Math.max(t1 - t0, 1e-6) * (w - pad.l - pad.r);
  const Y = (v) => h - pad.b - (v - ymin) / (ymax - ymin) * (h - pad.t - pad.b);
  ctx.strokeStyle = "#e5e5ea"; ctx.lineWidth = 1; ctx.fillStyle = "#8e8e93"; ctx.font = "11px -apple-system, sans-serif";
  for (let i = 0; i <= 4; i++) { const v = ymin + (ymax - ymin) * i / 4; ctx.beginPath(); ctx.moveTo(pad.l, Y(v)); ctx.lineTo(w - pad.r, Y(v)); ctx.stroke(); ctx.fillText(v.toFixed(v < 10 ? 1 : 0), 4, Y(v) + 4); }
  ctx.fillText(yLabel, pad.l, 10);
  ctx.fillText(`${(t1 - t0).toFixed(0)} s`, w - pad.r - 30, h - 6);
  if (threshold !== null) { ctx.strokeStyle = "#ff3b30"; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(pad.l, Y(threshold)); ctx.lineTo(w - pad.r, Y(threshold)); ctx.stroke(); ctx.setLineDash([]); }
  const line = (k, c) => { ctx.strokeStyle = c; ctx.lineWidth = 1.6; ctx.beginPath(); let first = true;
    series.forEach((p) => { const v = p[k] * unit; if (!Number.isFinite(v)) return; const x = X(p.t), y = Y(Math.min(v, ymax)); if (first) { ctx.moveTo(x, y); first = false; } else ctx.lineTo(x, y); }); ctx.stroke(); };
  line(key, color); if (key2) line(key2, color2);
}

// ----------------------------------------------------------------------------- pages
function nav(page) {
  S.page = page;
  $$("#nav a").forEach((a) => a.classList.toggle("active", a.dataset.page === page));
  $$(".page").forEach((p) => p.classList.toggle("active", p.id === "page-" + page));
  ({ overview: renderOverview, data: renderData, calib: renderCalib, mapping: renderMapping, results: renderResults })[page]();
}

// ---- overview
async function renderOverview() {
  const el = $("#page-overview");
  const [sys, cal, runs, jobs] = await Promise.all([api("/api/system"), api("/api/calibrations"), api("/api/runs"), api("/api/jobs")]);
  S.calib = cal; S.runs = runs; S.jobs = jobs;
  const running = jobs.filter((j) => j.alive);
  const active = Object.entries(cal.active || {});
  const done = runs.filter((r) => r.evaluated).slice(0, 8);
  el.innerHTML = `<h1>总览</h1><p class="sub">${esc(sys.repo)} · ${esc(sys.branch)} @ ${esc(sys.git)}</p>
  <div class="grid c4">
    <div class="card stat"><span class="v">${sys.disk_free_gb} <span class="small">GB</span></span><span class="k">磁盘剩余 / ${sys.disk_total_gb} GB</span></div>
    <div class="card stat"><span class="v">${sys.load.map((x) => x.toFixed(1)).join(" · ")}</span><span class="k">负载 (1/5/15 min) · ${sys.cpus} 核</span></div>
    <div class="card stat"><span class="v small">${esc(sys.gpu || "无 GPU 信息")}</span><span class="k">GPU</span></div>
    <div class="card stat"><span class="v small">${sys.mapper_alive ? '<span class="pulse"></span>建图节点运行中' : "空闲"}</span><span class="k">建图程序 · 二进制 ${esc((sys.binary_mtime || "").replace("T", " "))}</span></div>
  </div>
  <div class="grid c2" style="margin-top:18px">
    <div class="card"><h2>流程</h2>
      <div class="step ${S.bags.length || runs.length ? "done" : ""}"><span class="n">1</span><div><b>数据准备</b><div class="small">把原始 mcap 目录整理为一个全局有序的运行包（LIO 或 LIO+相机）。</div></div></div>
      <div class="step ${active.length ? "done" : "todo"}"><span class="n">2</span><div><b>标定管理</b><div class="small">导入出厂外参 → 静态地面 roll/pitch/z → 行驶段 6-DoF → 相机时间偏移 → 相机-LiDAR 边缘精修；每一步生成新版本，按矿区/车号/版本归档。</div></div></div>
      <div class="step ${done.length ? "done" : "todo"}"><span class="n">3</span><div><b>一键建图</b><div class="small">选择 LIO/LIVO、RTK 融合、保存地图、RViz；算法静态初始化阶段自动核对外参（小量修正，超限只告警）。</div></div></div>
      <div class="step ${done.length ? "done" : "todo"}"><span class="n">4</span><div><b>精度评估</b><div class="small">运行中实时显示前端/优化轨迹与 RTK 的差值，结束后自动评估：路线 ≤10 cm 占比、最大值、地面点云厚度。</div></div></div>
    </div>
    <div class="card"><h2>当前标定</h2>
      ${active.length ? active.map(([k, v]) => `<div class="item" onclick="location.hash='#calib'"><div class="t">${esc(k)}</div><div class="d">${esc(v)}</div></div>`).join("") : '<div class="small">还没有导入任何标定，请到“标定管理”导入出厂外参文件。</div>'}
      <div class="hr"></div><h2>正在运行</h2>
      ${running.length ? running.map((j) => `<div class="item" onclick="location.hash='#${j.kind === "mapping" ? "mapping" : j.kind === "prepare" ? "data" : "calib"}'"><div class="t"><span class="pulse"></span>${esc(j.title)}</div><div class="d">${esc(j.started)}</div></div>`).join("") : '<div class="small">无</div>'}
    </div>
  </div>
  <div class="card" style="margin-top:18px"><h2>最近评估结果</h2>${runsTable(done)}</div>`;
  $$("[data-run]", el).forEach((tr) => tr.onclick = () => { S.sel.run = tr.dataset.run; location.hash = "#results"; });
}
function runsTable(runs) {
  if (!runs.length) return '<div class="small">暂无</div>';
  return `<table><thead><tr><th>轮次</th><th>模式</th><th>时间</th><th>前端 中位/最大</th><th>优化后 中位/P95/最大</th><th>路线≤10cm</th><th>预算</th></tr></thead><tbody>
  ${runs.map((r) => { const n = r.metrics.native || {}, o = r.metrics.optimized || {}; return `<tr data-run="${esc(r.round)}" style="cursor:pointer"><td class="mono">${esc(r.round)}</td><td><span class="badge blue">${esc(r.mode.toUpperCase())}</span></td>
    <td class="small">${esc((r.start || r.mtime).replace("T", " ").slice(0, 19))}</td><td>${n.median_m !== undefined ? fmt(n.median_m, 2) + " / " + fmt(n.max_m, 2) + " m" : "–"}</td>
    <td>${o.median_m !== undefined ? `${cm(o.median_m)} / ${cm(o.p95_m)} / ${cm(o.max_m)}` : "–"}</td><td>${o.route_le10cm_pct !== undefined ? o.route_le10cm_pct.toFixed(1) + "%" : "–"}</td>
    <td>${o.budget ? `<span class="badge ${o.budget === "PASS" ? "ok" : "bad"}">${o.budget}</span>` : '<span class="badge">未评估</span>'}</td></tr>`; }).join("")}</tbody></table>`;
}

// ---- data
async function renderData() {
  const el = $("#page-data");
  S.bags = await api("/api/bags");
  el.innerHTML = `<h1>数据准备</h1><p class="sub">把一次采集的 mcap 目录整理成一个全局有序、只含建图所需话题的运行包。已经准备好的包可以直接用于标定和建图。</p>
  <div class="grid side">
    <div class="card"><h2>新建运行包</h2>
      <div class="field"><label>原始 mcap 目录（bag root）</label>${pathField("d-root", "data", "bag_root")}</div>
      <div class="field"><label>输出目录</label>${pathField("d-out", "data", "prepared_output")}</div>
      <div class="switch"><div><div class="lbl">仅 LIO 话题</div><div class="hint">不含相机流，体积小；LIVO 建图需要含相机的包</div></div><label class="toggle"><input type="checkbox" data-k="lio_only"><span></span></label></div>
      <div class="row" style="margin-top:14px"><button class="btn primary" id="d-run">开始准备</button><span class="small">耗时约等于数据时长的 0.5–1 倍</span></div>
      <div class="log" id="d-log" style="margin-top:14px;height:260px"></div>
    </div>
    <div class="card"><h2>已准备的运行包</h2>
      <table><thead><tr><th>名称</th><th>时长</th><th>消息数</th><th>相机</th><th>大小</th><th>路径</th></tr></thead><tbody>
      ${S.bags.map((b) => `<tr><td><b>${esc(b.name)}</b></td><td>${b.duration_s ?? "–"} s</td><td>${b.messages ?? "–"}</td><td>${b.camera ? '<span class="badge ok">有</span>' : '<span class="badge">无</span>'}</td><td>${b.size_mb ?? "–"} MB</td><td class="mono small">${esc(b.path)}</td></tr>`).join("") || '<tr><td colspan="6" class="small">暂无</td></tr>'}
      </tbody></table>
    </div></div>`;
  bind("data", el); wirePickers(el);
  const live = S.jobs.find((j) => j.kind === "prepare" && j.alive);
  if (live) watchJob(live.id, $("#d-log"), { onDone: renderData });
  $("#d-run").onclick = async () => {
    try {
      const j = await api("/api/prepare", { method: "POST", body: { bag_root: S.state.data.bag_root, output: S.state.data.prepared_output, lio_only: S.state.data.lio_only } });
      toast("数据准备已启动"); watchJob(j.id, $("#d-log"), { onDone: (jj) => { toast(jj.returncode === 0 ? "运行包准备完成" : "数据准备失败", jj.returncode === 0 ? "ok" : "bad"); renderData(); } });
    } catch (e) { toast(e.message, "bad"); }
  };
}

// ---- calibration
const STAGES = [
  { id: "ground", name: "静态地面 roll/pitch/z", need: ["bag"], hint: "用起步静止段（默认 2–20 s）各 LiDAR 的地面点对齐到参考 LiDAR，解每个 LiDAR 的 roll/pitch/z。" },
  { id: "driven", name: "行驶段 6-DoF 精修", need: ["bag", "run"], hint: "需要一次用父版本跑过的 LIO 轮次（fusion 目录）；用行驶窗口累积点云做 6-DoF 配准（竖直结构观测 yaw/xy）。" },
  { id: "camera-time", name: "相机时间偏移", need: ["bag"], hint: "光流角速度与陀螺仪互相关，得到相机时间戳相对 IMU 的偏移（写入 time_offsets.camera）。需要含相机的包。" },
  { id: "camera-lidar", name: "相机-LiDAR 外参精修", need: ["bag", "run"], noActivate: true, hint: "需要一次 LIVO 轮次的 fusion 目录；把累积点云的深度边缘对齐到图像边缘，精修相机→后轴外参，并输出前后叠加图供人工确认。LIVO 原生漂移对相机外参 0.1° 级变化非常敏感：新版本必须先跑一轮 LIVO 对比原生 yaw/漂移后再设为当前版本。" },
  { id: "from-precheck", name: "采纳 SLAM 静态自检结果", need: ["run_dir"], hint: "把某一轮建图时算法静态初始化自检得到的 roll/pitch/z 修正固化为新版本（results/<轮次>）。" },
  { id: "set", name: "手工设定参数", need: ["values"], hint: "直接写入相机时间偏移 / LiDAR 时间偏移 / 内参 / 畸变，或按相机坐标系给相机外参一个小旋转（闭环试验），生成新版本。" },
];
async function renderCalib() {
  const el = $("#page-calib");
  S.calib = await api("/api/calibrations"); S.jobs = await api("/api/jobs");
  const groups = {};
  S.calib.versions.forEach((v) => { const k = `${v.mine}/${v.truck}`; (groups[k] = groups[k] || []).push(v); });
  const c = S.state.calibration;
  if (!S.sel.calib && S.calib.versions.length) S.sel.calib = S.calib.active[`${c.mine}/${c.truck}`] || S.calib.versions[S.calib.versions.length - 1].id;
  el.innerHTML = `<h1>标定管理</h1><p class="sub">目录：config/carlibration/&lt;矿区&gt;/&lt;车号&gt;/&lt;版本&gt;/calibration.json。每个阶段都从父版本派生新版本，输入文件永不改写；“当前版本”是建图默认使用的文件。</p>
  <div class="grid side">
    <div>
      <div class="card" style="margin-bottom:18px"><h2>版本</h2>
        ${Object.keys(groups).length ? Object.entries(groups).map(([k, vs]) => `<div class="tree-group"><h4>${esc(k)}</h4><div class="list">
          ${vs.map((v) => `<div class="item ${S.sel.calib === v.id ? "sel" : ""}" data-cid="${esc(v.id)}"><div class="row between"><span class="t">${esc(v.version)}</span>${S.calib.active[k] === v.id ? '<span class="badge ok">当前</span>' : ""}</div>
            <div class="d">${esc(v.stage)} · ${esc((v.created || "").slice(0, 16).replace("T", " "))}</div><div class="d">${esc(v.note || "")}</div></div>`).join("")}</div></div>`).join("") : '<div class="small">尚无版本</div>'}
      </div>
      <div class="card"><h2>导入出厂外参</h2>
        <div class="field"><label>外参 JSON（schema 5，含 lidar[] 与 transforms.midrange_camera_to_rear_axle）</label>${pathField("c-src", "calibration", "source_file", { dirs: false })}</div>
        <div class="grid c2"><div class="field"><label>矿区</label><input type="text" data-k="mine"></div><div class="field"><label>车号</label><input type="text" data-k="truck"></div></div>
        <div class="grid c2"><div class="field"><label>LiDAR 时间偏移 (s)</label><input type="number" step="0.001" data-k="lidar_offset"></div><div class="field"><label>相机时间偏移 (s)</label><input type="number" step="0.001" data-k="camera_offset"></div></div>
        <div class="field"><label>备注</label><input type="text" id="c-note-import" placeholder="例如：2026-05 出厂人工标定"></div>
        <button class="btn primary" id="c-import">导入为新版本</button>
      </div>
    </div>
    <div>
      <div class="card" id="c-detail" style="margin-bottom:18px"></div>
      <div class="card"><h2>执行标定阶段</h2>
        <div class="field"><label>父版本</label><select id="c-parent">${S.calib.versions.map((v) => `<option value="${esc(v.id)}" ${S.sel.calib === v.id ? "selected" : ""}>${esc(v.id)}</option>`).join("")}</select></div>
        <div class="field"><label>阶段</label><div class="seg" id="c-stages">${STAGES.map((s, i) => `<button data-s="${s.id}" class="${i === 0 ? "on" : ""}">${s.name}</button>`).join("")}</div></div>
        <div class="small" id="c-hint" style="margin-bottom:10px">${STAGES[0].hint}</div>
        <div id="c-inputs"></div>
        <div class="field"><label>备注</label><input type="text" id="c-note" placeholder="可选"></div>
        <div class="switch"><div><div class="lbl">完成后设为当前版本</div><div class="hint">建图默认使用当前版本</div></div><label class="toggle"><input type="checkbox" id="c-activate" checked><span></span></label></div>
        <div class="row" style="margin-top:12px"><button class="btn primary" id="c-run">运行阶段</button></div>
        <div class="log" id="c-log" style="margin-top:14px;height:280px"></div>
      </div>
    </div>
  </div>`;
  bind("calibration", el); wirePickers(el);
  $$("[data-cid]", el).forEach((it) => it.onclick = () => { S.sel.calib = it.dataset.cid; $("#c-parent").value = it.dataset.cid; $$("[data-cid]", el).forEach((x) => x.classList.toggle("sel", x === it)); renderCalibDetail(); });
  renderCalibDetail();
  let stage = STAGES[0];
  const renderInputs = () => {
    $("#c-hint").textContent = stage.hint;
    let h = "";
    if (stage.need.includes("bag")) h += `<div class="field"><label>运行包目录</label><div class="path-field"><select id="ci-bag">${S.bags.map((b) => `<option value="${esc(b.path)}" ${b.path === c.bag ? "selected" : ""}>${esc(b.name)} (${b.duration_s}s${b.camera ? ", 含相机" : ""})</option>`).join("")}</select></div></div>`;
    if (stage.need.includes("run")) h += `<div class="field"><label>fusion 目录（results/&lt;轮次&gt;/Log/fusion/&lt;stamp&gt;）</label><select id="ci-run">${S.runs.filter((r) => r.fusion_dir).map((r) => `<option value="${esc(r.fusion_dir)}" ${r.fusion_dir === c.run ? "selected" : ""}>${esc(r.round)} [${esc(r.mode)}]</option>`).join("")}</select></div>`;
    if (stage.need.includes("run_dir")) h += `<div class="field"><label>建图轮次（含 Log/extrinsic_precheck.json）</label><select id="ci-rundir">${S.runs.filter((r) => r.precheck).map((r) => `<option value="${esc(r.path)}">${esc(r.round)}</option>`).join("")}</select></div>`;
    if (stage.id === "camera-lidar") h += `<div class="field"><label>使用图像帧数</label><input type="number" id="ci-frames" value="12" min="4" max="40"></div>`;
    if (stage.need.includes("values")) h += `<div class="grid c2"><div class="field"><label>相机时间偏移 (s，可空)</label><input type="number" step="0.001" id="ci-cam"></div><div class="field"><label>LiDAR 时间偏移 (s，可空)</label><input type="number" step="0.001" id="ci-lid"></div></div>
      <div class="field"><label>内参 fx fy cx cy（可空）</label><input type="text" id="ci-intr" placeholder="1642.57 1855.53 971.01 534.26"></div><div class="field"><label>畸变 k1 k2 p1 p2 k3（可空）</label><input type="text" id="ci-dist" placeholder="0 0 0 0 0"></div>
      <div class="field"><label>相机外参微调 rx ry rz（°，相机坐标系，可空）</label><input type="text" id="ci-camrot" placeholder="0.07 0.04 0"></div>`;
    $("#c-inputs").innerHTML = h;
    $("#c-activate").checked = !stage.noActivate;
  };
  if (!S.bags.length) S.bags = await api("/api/bags");
  if (!S.runs.length) S.runs = await api("/api/runs");
  renderInputs();
  $$("#c-stages button").forEach((b) => b.onclick = () => { $$("#c-stages button").forEach((x) => x.classList.toggle("on", x === b)); stage = STAGES.find((s) => s.id === b.dataset.s); renderInputs(); });
  const live = S.jobs.find((j) => j.kind === "calibration" && j.alive);
  if (live) watchJob(live.id, $("#c-log"), { onDone: renderCalib });
  $("#c-import").onclick = async () => {
    try {
      const j = await api("/api/calibrations/stage", { method: "POST", body: { stage: "import", source: c.source_file, mine: c.mine, truck: c.truck, lidar_offset: c.lidar_offset, camera_offset: c.camera_offset, note: $("#c-note-import").value, activate: true } });
      watchJob(j.id, $("#c-log"), { onDone: (jj) => { toast(jj.returncode === 0 ? "导入完成" : "导入失败", jj.returncode === 0 ? "ok" : "bad"); renderCalib(); } });
    } catch (e) { toast(e.message, "bad"); }
  };
  $("#c-run").onclick = async () => {
    const body = { stage: stage.id, id: $("#c-parent").value, note: $("#c-note").value, activate: $("#c-activate").checked };
    if ($("#ci-bag")) { body.bag = $("#ci-bag").value; persist("calibration", { bag: body.bag }); }
    if ($("#ci-run")) { body.run = $("#ci-run").value; persist("calibration", { run: body.run }); }
    if ($("#ci-rundir")) body.run_dir = $("#ci-rundir").value;
    if ($("#ci-frames")) body.frames = +$("#ci-frames").value;
    if ($("#ci-cam") && $("#ci-cam").value !== "") body.camera_offset = +$("#ci-cam").value;
    if ($("#ci-lid") && $("#ci-lid").value !== "") body.lidar_offset = +$("#ci-lid").value;
    if ($("#ci-intr") && $("#ci-intr").value.trim()) body.intrinsics = $("#ci-intr").value.trim().split(/\s+/).map(Number);
    if ($("#ci-dist") && $("#ci-dist").value.trim()) body.distortion = $("#ci-dist").value.trim().split(/\s+/).map(Number);
    if ($("#ci-camrot") && $("#ci-camrot").value.trim()) body.camera_rot_delta = $("#ci-camrot").value.trim().split(/\s+/).map(Number);
    try {
      const j = await api("/api/calibrations/stage", { method: "POST", body });
      toast(`阶段 ${stage.name} 已启动`);
      watchJob(j.id, $("#c-log"), { onDone: (jj) => { toast(jj.returncode === 0 ? "阶段完成，已生成新版本" : "阶段失败，请查看日志", jj.returncode === 0 ? "ok" : "bad"); renderCalib(); } });
    } catch (e) { toast(e.message, "bad"); }
  };
}
async function renderCalibDetail() {
  const box = $("#c-detail"); if (!box) return;
  if (!S.sel.calib) { box.innerHTML = '<h2>版本详情</h2><div class="small">请选择左侧版本</div>'; return; }
  let r; try { r = await api("/api/calibrations/report?id=" + encodeURIComponent(S.sel.calib)); } catch (e) { box.innerHTML = `<div class="small">${esc(e.message)}</div>`; return; }
  const v = S.calib.versions.find((x) => x.id === S.sel.calib) || {}; const s = v.summary || {};
  const isActive = S.calib.active[`${v.mine}/${v.truck}`] === v.id;
  box.innerHTML = `<div class="row between"><h2 style="margin:0">${esc(v.id)}</h2><div class="row">${isActive ? '<span class="badge ok">当前版本</span>' : '<button class="btn sm primary" id="c-act">设为当前</button>'}</div></div>
  <div class="small" style="margin:4px 0 10px">${esc(v.stage)} · ${esc(v.created || "")} · 父版本 ${esc(v.parent || "无")}<br>${esc(v.note || "")}</div>
  <table><thead><tr><th>LiDAR</th><th>t (m)</th><th>rpy (°)</th></tr></thead><tbody>${(s.lidars || []).map((l) => `<tr><td>${esc(l.lidar)}</td><td class="mono">${l.t.map((x) => x.toFixed(3)).join(", ")}</td><td class="mono">${l.rpy_deg.map((x) => x.toFixed(3)).join(", ")}</td></tr>`).join("")}
  ${s.camera_to_rear ? `<tr><td>camera → rear</td><td class="mono">${s.camera_to_rear.t.map((x) => x.toFixed(3)).join(", ")}</td><td class="mono">${s.camera_to_rear.rpy_deg.map((x) => x.toFixed(3)).join(", ")}</td></tr>` : ""}</tbody></table>
  <div class="grid c2" style="margin-top:10px"><div class="kv"><span class="k">相机内参</span><span class="mono">${esc((s.camera?.intrinsics || []).map((x) => x.toFixed(2)).join(" "))}</span><span class="k">畸变</span><span class="mono">${esc((s.camera?.distortion || []).join(" "))}</span></div>
  <div class="kv"><span class="k">LiDAR 时间偏移</span><span class="mono">${esc(s.time_offsets?.lidar)} s</span><span class="k">相机时间偏移</span><span class="mono">${esc(s.time_offsets?.camera)} s</span><span class="k">RTK 位置延时</span><span class="mono">${esc(s.time_offsets?.rtk_position)} s</span></div></div>
  ${v.metrics && Object.keys(v.metrics).length ? `<details style="margin-top:10px"><summary class="small">阶段指标</summary><pre class="doc">${esc(JSON.stringify(v.metrics, null, 2))}</pre></details>` : ""}
  ${r.overlays.length ? `<h3 style="margin-top:12px">叠加图（红=精修前，绿=精修后，黄=图像边缘）</h3><div class="gallery">${r.overlays.map((n) => `<img src="/api/calibrations/file?id=${encodeURIComponent(v.id)}&name=overlays/${encodeURIComponent(n)}" alt="${esc(n)}">`).join("")}</div>` : ""}
  <details style="margin-top:10px"><summary class="small">report.md</summary><pre class="doc">${esc(r.files["report.md"] || "")}</pre></details>
  <div class="small mono" style="margin-top:8px">${esc(v.calibration_file || "")}</div>`;
  const act = $("#c-act"); if (act) act.onclick = async () => { try { await api("/api/calibrations/activate", { method: "POST", body: { id: v.id } }); toast("已设为当前版本", "ok"); renderCalib(); } catch (e) { toast(e.message, "bad"); } };
}

// ---- mapping
async function renderMapping() {
  const el = $("#page-mapping");
  [S.bags, S.calib, S.jobs] = await Promise.all([api("/api/bags"), api("/api/calibrations"), api("/api/jobs")]);
  const m = S.state.mapping; const c = S.state.calibration;
  const activeId = S.calib.active[`${c.mine}/${c.truck}`] || Object.values(S.calib.active)[0] || "";
  if (!m.calibration_id || !S.calib.versions.some((v) => v.id === m.calibration_id)) m.calibration_id = activeId;
  const live = S.jobs.find((j) => j.kind === "mapping" && j.alive);
  S.mappingJob = live || S.mappingJob;
  el.innerHTML = `<h1>一键建图</h1><p class="sub">启动完整流程：启动建图节点 → 有序 1× 回放 → 尾部排空 → 优化并导出轨迹与完整 PCD → 自动评估。所有产物写入 results/&lt;轮次&gt;/。</p>
  <div class="grid side">
    <div class="card">
      <h2>参数</h2>
      <div class="field"><label>轮次前缀</label><input type="text" data-k="round_prefix"></div>
      <div class="field"><label>模式</label><div class="seg" id="m-mode"><button data-v="lio" class="${m.mode === "lio" ? "on" : ""}">LIO（多 LiDAR）</button><button data-v="livo" class="${m.mode === "livo" ? "on" : ""}">LIVO（含相机）</button></div></div>
      <div class="field"><label>运行包</label><select data-k="bag" id="m-bag">${S.bags.map((b) => `<option value="${esc(b.path)}">${esc(b.name)} (${b.duration_s}s${b.camera ? ", 含相机" : ""})</option>`).join("")}</select></div>
      <div class="field"><label>标定版本</label><select data-k="calibration_id" id="m-cal">${S.calib.versions.map((v) => `<option value="${esc(v.id)}">${esc(v.id)}${activeId === v.id ? "（当前）" : ""}</option>`).join("")}<option value="">（使用 yaml 中的文件）</option></select></div>
      <div class="switch"><div><div class="lbl">RTK 融合</div><div class="hint">关闭则只输出原生前端轨迹</div></div><label class="toggle"><input type="checkbox" data-k="use_rtk"><span></span></label></div>
      <div class="switch"><div><div class="lbl">保存地图</div><div class="hint">结束时导出全部 map_*.pcd（LIVO 另含 rgb_map_*.pcd）</div></div><label class="toggle"><input type="checkbox" data-k="save_map"><span></span></label></div>
      <div class="switch"><div><div class="lbl">RViz</div><div class="hint">红=前端，蓝=优化后，绿=INS 参考</div></div><label class="toggle"><input type="checkbox" data-k="rviz"><span></span></label></div>
      <div class="switch"><div><div class="lbl">静态初始化外参自检</div><div class="hint">起步静止段自动核对各 LiDAR roll/pitch/z，小量修正，超限只告警</div></div><label class="toggle"><input type="checkbox" data-k="precheck"><span></span></label></div>
      <div class="switch"><div><div class="lbl">结束后自动评估</div><div class="hint">轨迹 vs RTK、地面厚度等指标</div></div><label class="toggle"><input type="checkbox" data-k="evaluate"><span></span></label></div>
      <div class="grid c2" style="margin-top:10px"><div class="field"><label>回放速率</label><input type="number" step="0.1" min="0.1" data-k="rate"></div>
        <div class="field" id="m-div-field"><label>相机帧分频（LIVO）</label><select data-k="frame_divider"><option value="1">1 · 全部图像 (30 Hz)</option><option value="2">2 · 每 2 帧 (15 Hz)</option><option value="3">3 · 每 3 帧 (10 Hz)</option></select></div></div>
      <div class="field"><label>额外参数覆盖（空格分隔 key=value；相机内参与时间偏移以标定文件为准）</label><input type="text" data-k="extra_sets" placeholder="例如 vio.img_point_cov=300"></div>
      <div style="margin-top:12px"><button class="btn primary big" id="m-start" style="width:100%" ${live ? "disabled" : ""}>▶ 开始建图</button></div>
      <div class="row" style="margin-top:8px">
        <button class="btn sm" id="m-finish" ${live ? "" : "disabled"}>提前结束并导出</button>
        <button class="btn sm danger" id="m-kill" ${live ? "" : "disabled"}>强制终止</button>
      </div>
    </div>
    <div id="m-status"></div>
  </div>`;
  bind("mapping", el);
  $$("#m-mode button").forEach((b) => b.onclick = () => { $$("#m-mode button").forEach((x) => x.classList.toggle("on", x === b)); persist("mapping", { mode: b.dataset.v }); $("#m-div-field").style.opacity = b.dataset.v === "livo" ? 1 : .4; });
  $("#m-div-field").style.opacity = m.mode === "livo" ? 1 : .4;
  if (m.bag) $("#m-bag").value = m.bag; else if (S.bags.length) persist("mapping", { bag: S.bags[S.bags.length - 1].path });
  $("#m-cal").value = m.calibration_id;
  $("#m-start").onclick = async () => {
    try {
      const j = await api("/api/mapping/start", { method: "POST", body: S.state.mapping });
      S.mappingJob = j; toast("建图已启动：" + j.meta.round, "ok"); renderMapping();
    } catch (e) { toast(e.message, "bad"); }
  };
  $("#m-finish").onclick = async () => { if (S.mappingJob) { await api(`/api/jobs/${S.mappingJob.id}/stop`, { method: "POST", body: { graceful: true } }); toast("已停止回放，正在优化并导出…"); } };
  $("#m-kill").onclick = async () => { if (confirm("强制终止将丢失本轮结果，确定？")) { await api("/api/mapping/kill", { method: "POST", body: {} }); if (S.mappingJob) await api(`/api/jobs/${S.mappingJob.id}/stop`, { method: "POST", body: { graceful: false } }); toast("已终止", "bad"); setTimeout(renderMapping, 1500); } };
  renderStatus($("#m-status"), S.mappingJob ? S.mappingJob.meta.round : null, S.mappingJob);
}

function renderStatus(box, round, job) {
  clearTimeout(S.timers.live);
  if (!round) { box.innerHTML = `<div class="card"><h2>状态</h2><div class="small">尚未启动。填写左侧参数后点击“开始建图”。</div></div>`; return; }
  box.innerHTML = `<div class="card" style="margin-bottom:18px"><div class="row between"><h2 style="margin:0" id="s-title">${esc(round)}</h2><span id="s-state" class="badge">…</span></div>
    <div class="metric-row" id="s-metrics" style="margin-top:12px"></div>
    <div class="grid c2" style="margin-top:12px"><div><h3>前端位姿 vs RTK（cm，虚线 10 cm）</h3><canvas class="chart" id="s-chart1"></canvas></div><div><h3>已发布优化轨迹 vs INS 参考（cm）</h3><canvas class="chart" id="s-chart2"></canvas></div></div>
    <h3 style="margin-top:12px">算法事件</h3><div class="events" id="s-events"></div>
    <div id="s-precheck" style="margin-top:12px"></div>
    <div id="s-summary" style="margin-top:12px"></div></div>
    <div class="card"><h2>运行日志</h2><div class="log" id="s-log"></div></div>`;
  if (job) watchJob(job.id, $("#s-log"), { onDone: () => { $$("#m-start,#m-finish,#m-kill").forEach((b) => { if (b) b.disabled = b.id !== "m-start"; }); } });
  else fetch(`/api/runs/${encodeURIComponent(round)}/file?name=run.log`).then((r) => r.ok ? r.text() : "").then((t) => { const el = $("#s-log"); if (el) { el.innerHTML = logHtml(t.split("\n").slice(-400)); el.scrollTop = el.scrollHeight; } });
  let warned = false;
  const tick = async () => {
    let d; try { d = await api(`/api/runs/${encodeURIComponent(round)}/live`); } catch (e) { if (!warned) { warned = true; toast("读取运行状态失败：" + e.message, "bad"); } S.timers.live = setTimeout(tick, 5000); return; }
    const st = $("#s-state"); if (!st) return;
    const jobAlive = job ? (S.jobs.find((j) => j.id === job.id) || {}).alive !== false : false;
    const phase = d.summary ? "已评估" : d.player_alive ? "回放中" : d.mapper_alive ? "优化/导出中" : (d.map_chunks ? "已导出" : "启动中");
    st.className = "badge " + (d.summary ? "ok" : (d.mapper_alive || d.player_alive ? "blue" : "warn")); st.innerHTML = (d.mapper_alive || d.player_alive ? '<span class="pulse"></span>' : "") + phase;
    const L = d.latest || {}, O = d.online || {};
    $("#s-metrics").innerHTML = [
      ["帧数", d.frames ?? "–"], ["关键帧", L.keyframes ?? "–"], ["前端 vs RTK", cm(L.frontend_rtk_residual_m)], ["优化 最新", cm(O.latest_imu_difference_m)], ["优化 最大", cm(O.max_imu_difference_m)],
      ["速度 / 延迟", `${fmt(L.frontend_speed_mps, 1)} m/s / ${fmt(L.publish_lag_sec, 1)} s`]].map(([k, v]) => `<div class="metric"><div class="v">${v}</div><div class="k">${k}</div></div>`).join("");
    if (d.diagnostics) drawChart($("#s-chart1"), d.diagnostics, { key: "frontend_rtk_residual_m", unit: 100, threshold: 10, yLabel: "cm" });
    if (d.diagnostics && d.diagnostics.some((p) => p.published_diff_m !== null)) drawChart($("#s-chart2"), d.diagnostics, { key: "published_diff_m", unit: 100, threshold: 10, yLabel: "cm" });
    else if (d.online_series) drawChart($("#s-chart2"), d.online_series, { key: "latest", key2: "max", unit: 100, threshold: 10, yLabel: "cm" });
    $("#s-events").innerHTML = (d.launch_events || []).slice().reverse().map((l) => `<div class="${/WARN|exceeds|not checked/.test(l) ? "warn" : ""}">${esc(l)}</div>`).join("") || '<div class="small">等待算法输出…</div>';
    if (d.precheck && d.precheck.sources) {
      const p = d.precheck;
      const st = { applied: ["ok", "已修正"], gated: ["bad", "超限，仅告警"], confirmed: ["ok", "已确认"], no_change: ["blue", "无修正"], skipped: ["", "未执行"] }[p.status] || ["", p.status];
      $("#s-precheck").innerHTML = `<h3>静态初始化外参自检 · <span class="badge ${st[0]}">${esc(st[1])}</span> <span class="small">参考 ${esc(p.reference_topic)} · 阈值 ±${p.config?.max_rotation_deg}° / ±${(p.config?.max_height_m * 100).toFixed(0)} cm</span></h3>
        <table><thead><tr><th>LiDAR</th><th>地面残差 前→后</th><th>rot_x</th><th>rot_y</th><th>dz</th><th>结论</th></tr></thead><tbody>${p.sources.map((s) => `<tr><td>${esc(s.topic)}</td><td>${s.solved ? `${fmt(s.median_before_cm, 2)} → ${fmt(s.median_after_cm, 2)} cm` : "–"}</td><td>${s.solved ? fmt(s.rot_x_deg, 3) + "°" : "–"}</td><td>${s.solved ? fmt(s.rot_y_deg, 3) + "°" : "–"}</td><td>${s.solved ? fmt(s.dz_cm, 2) + " cm" : "–"}</td><td>${s.applied ? '<span class="badge ok">已修正</span>' : `<span class="badge ${s.solved && !s.within_gates ? "bad" : ""}">${esc(s.note || (s.solved ? "确认无需修正" : "未检查"))}</span>`}</td></tr>`).join("")}</tbody></table>`;
    }
    if (d.summary) {
      const mm = d.metrics || {}; const o = mm.optimized || {}, n = mm.native || {};
      $("#s-summary").innerHTML = `<h3>评估结果</h3><div class="metric-row">
        <div class="metric"><div class="v">${o.budget ? `<span class="badge ${o.budget === "PASS" ? "ok" : "bad"}" style="font-size:14px">${o.budget}</span>` : "–"}</div><div class="k">路线预算（80% ≤10 cm，最大 20 cm）</div></div>
        <div class="metric"><div class="v">${o.route_le10cm_pct !== undefined ? o.route_le10cm_pct.toFixed(1) + "%" : "–"}</div><div class="k">优化轨迹 路线 ≤10 cm</div></div>
        <div class="metric"><div class="v">${cm(o.median_m)}</div><div class="k">优化轨迹 中位误差</div></div><div class="metric"><div class="v">${cm(o.max_m)}</div><div class="k">优化轨迹 最大误差</div></div>
        <div class="metric"><div class="v">${fmt(n.median_m, 2)} m</div><div class="k">原生前端 中位漂移</div></div><div class="metric"><div class="v">${fmt(n.max_m, 2)} m</div><div class="k">原生前端 最大漂移</div></div></div>
        ${(d.thickness || []).map((t) => `<div class="small mono" style="margin-top:6px">${esc(t)}</div>`).join("")}
        <details style="margin-top:8px"><summary class="small">summary.txt</summary><pre class="doc">${esc(d.summary.join("\n"))}</pre></details>
        <div class="small mono" style="margin-top:6px">地图分片 ${d.map_chunks} 个（${d.map_size_mb} MB）${d.rgb_map_chunks ? `，RGB 分片 ${d.rgb_map_chunks} 个` : ""} · ${esc(d.fusion_dir || "")}</div>`;
    }
    if (S.page === "mapping" || S.page === "results") S.timers.live = setTimeout(tick, d.summary && !(d.mapper_alive || d.player_alive) ? 15000 : 2000);
  };
  tick();
}

// ---- results
async function renderResults() {
  const el = $("#page-results");
  S.runs = await api("/api/runs");
  if (!S.sel.run && S.runs.length) S.sel.run = S.runs[0].round;
  el.innerHTML = `<h1>结果与精度</h1><p class="sub">每一轮建图的评估摘要；选择一轮查看实时/最终精度、静态自检结论与导出情况。</p>
  <div class="grid side"><div class="card" style="max-height:80vh;overflow:auto"><h2>轮次</h2><div class="list">${S.runs.map((r) => { const o = r.metrics.optimized || {}; return `<div class="item ${S.sel.run === r.round ? "sel" : ""}" data-r="${esc(r.round)}"><div class="row between"><span class="t mono">${esc(r.round)}</span>${o.budget ? `<span class="badge ${o.budget === "PASS" ? "ok" : "bad"}">${o.budget}</span>` : '<span class="badge">未评估</span>'}</div>
    <div class="d">${esc(r.mode.toUpperCase())} · ${esc((r.start || r.mtime).replace("T", " ").slice(0, 19))}${o.median_m !== undefined ? ` · 中位 ${cm(o.median_m)} 最大 ${cm(o.max_m)}` : ""}</div><div class="d">${esc(r.calibration !== "yaml" ? r.calibration.split("/").slice(-2, -1)[0] || r.calibration : "")}</div></div>`; }).join("") || '<div class="small">暂无</div>'}</div></div>
  <div id="r-status"></div></div>`;
  $$("[data-r]", el).forEach((it) => it.onclick = () => { S.sel.run = it.dataset.r; $$("[data-r]", el).forEach((x) => x.classList.toggle("sel", x === it)); renderStatus($("#r-status"), S.sel.run, null); });
  renderStatus($("#r-status"), S.sel.run, null);
}

// ----------------------------------------------------------------------------- boot
async function sysMini() {
  try { const s = await api("/api/system"); $("#sys-mini").innerHTML = `<div>${s.mapper_alive ? '<span class="pulse"></span>建图运行中' : "空闲"}</div><div>磁盘剩余 ${s.disk_free_gb} GB · 负载 ${s.load[0].toFixed(1)}</div><div>${esc(s.time.slice(11, 19))}</div>`; }
  catch (e) { $("#sys-mini").textContent = "服务不可达"; }
  setTimeout(sysMini, 10000);
}
(async () => {
  S.state = await api("/api/state");
  S.jobs = await api("/api/jobs");
  window.addEventListener("hashchange", () => nav((location.hash || "#overview").slice(1)));
  nav((location.hash || "#overview").slice(1));
  sysMini();
})();
