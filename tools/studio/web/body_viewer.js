/* Lightweight fused-cloud viewer for body filtering. No external 3D dependency. */
(() => {
  "use strict";

  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const finite = (x, fallback = 0) => Number.isFinite(Number(x)) ? Number(x) : fallback;

  function normalizedRect(r) {
    if (!r) return null;
    const x1 = finite(r.x_min, NaN), x2 = finite(r.x_max, NaN);
    const y1 = finite(r.y_min, NaN), y2 = finite(r.y_max, NaN);
    if (![x1, x2, y1, y2].every(Number.isFinite)) return null;
    const out = { x_min: Math.min(x1, x2), x_max: Math.max(x1, x2),
      y_min: Math.min(y1, y2), y_max: Math.max(y1, y2) };
    return out.x_max - out.x_min > 0.01 && out.y_max - out.y_min > 0.01 ? out : null;
  }

  function attach(canvas, modelProvider, onRectangleChange, initialView = "top") {
    let view = initialView === "orbit" ? "orbit" : "top";
    let zoom = 1, panX = 0, panY = 0;
    let yaw = 0, elevation = 18 * Math.PI / 180;
    let target = null, pointer = null;
    let cachedCanvasSize = null;

    function getModel() { return modelProvider() || {}; }
    function getSize() {
      const box = canvas.getBoundingClientRect();
      const w = Math.max(320, box.width || 640), h = Math.max(300, box.height || 560);
      const dpr = window.devicePixelRatio || 1;
      const W = Math.round(w * dpr), H = Math.round(h * dpr);
      if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
      const ctx = canvas.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      cachedCanvasSize = { w, h, dpr };
      return { ctx, w, h };
    }
    function getBounds(model) {
      const b = model.bounds || {};
      let x0 = finite(b.x_min, -12), x1 = finite(b.x_max, 12);
      let y0 = finite(b.y_min, -12), y1 = finite(b.y_max, 12);
      if (!(x1 > x0)) { x0 = -12; x1 = 12; }
      if (!(y1 > y0)) { y0 = -12; y1 = 12; }
      const r = normalizedRect(model.rectangle);
      if (r) { x0 = Math.min(x0, r.x_min); x1 = Math.max(x1, r.x_max); y0 = Math.min(y0, r.y_min); y1 = Math.max(y1, r.y_max); }
      let z0 = Infinity, z1 = -Infinity;
      (model.series || []).forEach((s) => (s.points || []).forEach((p) => {
        const z = Number(p[2]); if (Number.isFinite(z)) { z0 = Math.min(z0, z); z1 = Math.max(z1, z); }
      }));
      if (!Number.isFinite(z0) || !Number.isFinite(z1)) { z0 = -2; z1 = 3; }
      if (z1 - z0 < 0.5) { z0 -= 0.5; z1 += 0.5; }
      const dx = Math.max(1, x1 - x0) * 0.035, dy = Math.max(1, y1 - y0) * 0.035;
      return { x_min: x0 - dx, x_max: x1 + dx, y_min: y0 - dy, y_max: y1 + dy, z_min: z0, z_max: z1 };
    }
    function centerFor(bounds) {
      if (!target) target = { x: (bounds.x_min + bounds.x_max) / 2,
        y: (bounds.y_min + bounds.y_max) / 2, z: (bounds.z_min + bounds.z_max) / 2 };
      return target;
    }
    function topScale(bounds, w, h) {
      const spanX = Math.max(2, bounds.x_max - bounds.x_min);
      const spanY = Math.max(2, bounds.y_max - bounds.y_min);
      return Math.max(0.001, Math.min((w - 44) / spanX, (h - 54) / spanY) * zoom);
    }
    function topProject(x, y, bounds, w, h) {
      const c = centerFor(bounds), scale = topScale(bounds, w, h);
      return [w / 2 + (x - c.x) * scale + panX,
        h / 2 - (y - c.y) * scale + panY];
    }
    function topWorld(px, py, bounds, w, h) {
      const c = centerFor(bounds), scale = topScale(bounds, w, h);
      return { x: c.x + (px - w / 2 - panX) / scale,
        y: c.y - (py - h / 2 - panY) / scale };
    }
    function cameraBasis() {
      const ce = Math.cos(elevation), se = Math.sin(elevation);
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const forward = [-cy * ce, -sy * ce, -se];
      const right = [-sy, cy, 0];
      const up = [-cy * se, -sy * se, ce];
      return { forward, right, up };
    }
    function orbitProject(p, bounds, w, h) {
      const c = centerFor(bounds), basis = cameraBasis();
      const span = Math.max(bounds.x_max - bounds.x_min, bounds.y_max - bounds.y_min,
        bounds.z_max - bounds.z_min, 5);
      const distance = span * 2.15 / zoom;
      const focal = Math.min(w, h) * 0.90;
      const dx = p[0] - c.x, dy = p[1] - c.y, dz = p[2] - c.z;
      const rel = [dx, dy, dz];
      const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
      const depth = distance + dot(rel, basis.forward);
      if (depth <= span * 0.08) return null;
      return [w / 2 + dot(rel, basis.right) * focal / depth + panX,
        h / 2 - dot(rel, basis.up) * focal / depth + panY, depth];
    }
    function project(p, bounds, w, h) {
      if (view === "top") return topProject(p[0], p[1], bounds, w, h);
      return orbitProject(p, bounds, w, h);
    }
    function drawGrid(ctx, bounds, w, h) {
      ctx.fillStyle = "#f8fafc"; ctx.fillRect(0, 0, w, h);
      if (view === "top") {
        const scale = topScale(bounds, w, h);
        const desired = 74 / scale;
        const pow = Math.pow(10, Math.floor(Math.log10(Math.max(desired, 0.001))));
        const unit = desired / pow;
        const step = (unit < 1.5 ? 1 : unit < 3.5 ? 2 : unit < 7.5 ? 5 : 10) * pow;
        const startX = Math.floor(bounds.x_min / step) * step;
        const endX = Math.ceil(bounds.x_max / step) * step;
        const startY = Math.floor(bounds.y_min / step) * step;
        const endY = Math.ceil(bounds.y_max / step) * step;
        ctx.strokeStyle = "#e5eaf0"; ctx.lineWidth = 1;
        for (let x = startX; x <= endX; x += step) {
          const [px] = topProject(x, 0, bounds, w, h);
          ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
        }
        for (let y = startY; y <= endY; y += step) {
          const [, py] = topProject(0, y, bounds, w, h);
          ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
        }
        const origin = topProject(0, 0, bounds, w, h);
        ctx.lineWidth = 1.6;
        ctx.strokeStyle = "#d64045"; ctx.beginPath(); ctx.moveTo(0, origin[1]); ctx.lineTo(w, origin[1]); ctx.stroke();
        ctx.strokeStyle = "#238a5b"; ctx.beginPath(); ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], h); ctx.stroke();
        ctx.fillStyle = "#334155"; ctx.font = "12px system-ui, sans-serif";
        ctx.fillText("+X 横向", Math.max(8, w - 78), Math.max(18, origin[1] - 7));
        ctx.fillText("+Y 车头", Math.min(w - 60, origin[0] + 8), 18);
        return;
      }
      const step = Math.max(1, Math.pow(10, Math.floor(Math.log10(Math.max(
        bounds.x_max - bounds.x_min, bounds.y_max - bounds.y_min, 1) / 8))));
      const startX = Math.floor(bounds.x_min / step) * step, endX = Math.ceil(bounds.x_max / step) * step;
      const startY = Math.floor(bounds.y_min / step) * step, endY = Math.ceil(bounds.y_max / step) * step;
      ctx.strokeStyle = "#e3e8ef"; ctx.lineWidth = 1;
      for (let x = startX; x <= endX; x += step) {
        const a = orbitProject([x, startY, 0], bounds, w, h), b = orbitProject([x, endY, 0], bounds, w, h);
        if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
      }
      for (let y = startY; y <= endY; y += step) {
        const a = orbitProject([startX, y, 0], bounds, w, h), b = orbitProject([endX, y, 0], bounds, w, h);
        if (a && b) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
      }
      const axes = [
        { end: [Math.max(2, bounds.x_max - bounds.x_min) * 0.22, 0, 0], color: "#d64045", label: "X" },
        { end: [0, Math.max(2, bounds.y_max - bounds.y_min) * 0.22, 0], color: "#238a5b", label: "Y" },
        { end: [0, 0, Math.max(2, bounds.z_max - bounds.z_min) * 0.7], color: "#2878c8", label: "Z ↑" },
      ];
      axes.forEach((axis) => {
        const a = orbitProject([0, 0, 0], bounds, w, h), b = orbitProject(axis.end, bounds, w, h);
        if (!a || !b) return;
        ctx.strokeStyle = axis.color; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        ctx.fillStyle = axis.color; ctx.font = "bold 12px system-ui, sans-serif"; ctx.fillText(axis.label, b[0] + 4, b[1] - 4);
      });
      ctx.fillStyle = "#334155"; ctx.font = "12px system-ui, sans-serif";
      ctx.fillText("Z-up · 拖动水平环绕世界 Z 轴", 10, 19);
    }
    function drawPoints(ctx, model, bounds, w, h) {
      (model.series || []).forEach((s) => {
        const points = s.points || [], filtered = s.filtered || [];
        ctx.globalAlpha = 0.78; ctx.fillStyle = s.color || "#0071e3";
        for (let i = 0; i < points.length; i++) {
          if (filtered[i]) continue;
          const q = project(points[i], bounds, w, h); if (!q || q[0] < -4 || q[0] > w + 4 || q[1] < -4 || q[1] > h + 4) continue;
          ctx.fillRect(q[0] - 1, q[1] - 1, 2, 2);
        }
      });
      (model.series || []).forEach((s) => {
        const points = s.points || [], filtered = s.filtered || [];
        ctx.globalAlpha = 0.98; ctx.fillStyle = "#ff3b30";
        for (let i = 0; i < points.length; i++) {
          if (!filtered[i]) continue;
          const q = project(points[i], bounds, w, h); if (!q || q[0] < -5 || q[0] > w + 5 || q[1] < -5 || q[1] > h + 5) continue;
          ctx.fillRect(q[0] - 1.8, q[1] - 1.8, 3.6, 3.6);
        }
      });
      ctx.globalAlpha = 1;
    }
    function drawRect(ctx, rect, bounds, w, h, model) {
      const r = normalizedRect(rect); if (!r) return;
      const points = [[r.x_min, r.y_min], [r.x_max, r.y_min], [r.x_max, r.y_max], [r.x_min, r.y_max]];
      if (view === "top") {
        const poly = points.map(([x, y]) => topProject(x, y, bounds, w, h));
        ctx.beginPath(); poly.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.closePath();
        ctx.fillStyle = model.enabled ? "rgba(255,59,48,.10)" : "rgba(0,113,227,.08)"; ctx.fill();
        ctx.strokeStyle = model.enabled ? "#ff3b30" : "#0071e3"; ctx.lineWidth = 2; ctx.setLineDash(model.enabled ? [] : [7, 4]); ctx.stroke(); ctx.setLineDash([]);
        poly.forEach(([x, y]) => { ctx.fillStyle = "#fff"; ctx.strokeStyle = "#ff3b30"; ctx.lineWidth = 1.5; ctx.fillRect(x - 4, y - 4, 8, 8); ctx.strokeRect(x - 4, y - 4, 8, 8); });
        return;
      }
      const z0 = finite(model.minimum_z, -3), z1 = Math.max(bounds.z_max + 0.4, z0 + 0.2);
      const bottom = points.map(([x, y]) => [x, y, z0]);
      const top = points.map(([x, y]) => [x, y, z1]);
      const projectPoly = (arr) => arr.map((p) => orbitProject(p, bounds, w, h)).filter(Boolean);
      ctx.fillStyle = "rgba(255,59,48,.075)"; ctx.strokeStyle = "rgba(230,55,48,.86)"; ctx.lineWidth = 1.6;
      for (let i = 0; i < 4; i++) {
        const face = projectPoly([bottom[i], bottom[(i + 1) % 4], top[(i + 1) % 4], top[i]]);
        if (face.length < 3) continue;
        ctx.beginPath(); face.forEach(([x, y], j) => j ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.closePath(); ctx.fill(); ctx.stroke();
      }
      [bottom, top].forEach((ring) => {
        const poly = projectPoly([...ring, ring[0]]); if (poly.length < 3) return;
        ctx.beginPath(); poly.forEach(([x, y], i) => i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)); ctx.stroke();
      });
      ctx.fillStyle = "#d92d20"; ctx.font = "11px system-ui, sans-serif";
      const tag = orbitProject([r.x_max, r.y_max, z0], bounds, w, h);
      if (tag) ctx.fillText(`XY 框 · z > ${z0.toFixed(2)} m`, tag[0] + 5, tag[1] - 5);
    }
    function draw() {
      const { ctx, w, h } = getSize();
      const model = getModel(), bounds = getBounds(model);
      drawGrid(ctx, bounds, w, h);
      drawPoints(ctx, model, bounds, w, h);
      drawRect(ctx, model.rectangle, bounds, w, h, model);
      const count = (model.series || []).reduce((n, s) => n + (s.points || []).length, 0);
      ctx.fillStyle = "rgba(15,23,42,.78)"; ctx.fillRect(8, h - 29, 188, 21);
      ctx.fillStyle = "#fff"; ctx.font = "11px system-ui, sans-serif";
      ctx.fillText(`${view === "top" ? "俯视 XY" : "平视 / 3D 环视"} · ${count.toLocaleString()} 点`, 15, h - 14);
    }
    function hitRect(px, py, rect, bounds, w, h) {
      const r = normalizedRect(rect); if (!r) return null;
      const corners = [
        ["x_min_y_min", r.x_min, r.y_min], ["x_max_y_min", r.x_max, r.y_min],
        ["x_max_y_max", r.x_max, r.y_max], ["x_min_y_max", r.x_min, r.y_max],
      ];
      for (const [name, x, y] of corners) {
        const p = topProject(x, y, bounds, w, h);
        if (Math.hypot(px - p[0], py - p[1]) <= 9) return { handle: name };
      }
      const a = topProject(r.x_min, r.y_min, bounds, w, h), b = topProject(r.x_max, r.y_max, bounds, w, h);
      const left = Math.min(a[0], b[0]), right = Math.max(a[0], b[0]);
      const top = Math.min(a[1], b[1]), bottom = Math.max(a[1], b[1]);
      const tol = 7;
      if (px >= left - tol && px <= right + tol && py >= top - tol && py <= bottom + tol) {
        if (Math.abs(px - left) <= tol) return { handle: "x_min" };
        if (Math.abs(px - right) <= tol) return { handle: "x_max" };
        if (Math.abs(py - bottom) <= tol) return { handle: "y_min" };
        if (Math.abs(py - top) <= tol) return { handle: "y_max" };
        return { move: true };
      }
      return null;
    }
    function resizeRect(orig, handle, x, y) {
      const r = { ...orig };
      if (handle.includes("x_min")) r.x_min = x;
      if (handle.includes("x_max")) r.x_max = x;
      if (handle.includes("y_min")) r.y_min = y;
      if (handle.includes("y_max")) r.y_max = y;
      if (handle === "x_min") r.x_min = x;
      if (handle === "x_max") r.x_max = x;
      if (handle === "y_min") r.y_min = y;
      if (handle === "y_max") r.y_max = y;
      return { x_min: Math.min(r.x_min, r.x_max), x_max: Math.max(r.x_min, r.x_max),
        y_min: Math.min(r.y_min, r.y_max), y_max: Math.max(r.y_min, r.y_max) };
    }
    function setRect(rect, redraw = true) {
      onRectangleChange(rect ? normalizedRect(rect) : null);
      if (redraw) draw();
    }
    function pointerPosition(ev) {
      const r = canvas.getBoundingClientRect(); return { x: ev.clientX - r.left, y: ev.clientY - r.top };
    }

    canvas.addEventListener("pointerdown", (ev) => {
      const model = getModel(), bounds = getBounds(model), pos = pointerPosition(ev);
      if (view === "orbit") {
        pointer = { mode: (ev.shiftKey || ev.button !== 0) ? "orbit-pan" : "orbit",
          x: pos.x, y: pos.y, target: target ? { ...target } : centerFor(bounds) };
      } else if (ev.shiftKey || ev.button !== 0) {
        pointer = { mode: "top-pan", x: pos.x, y: pos.y, panX, panY };
      } else {
        const wpt = topWorld(pos.x, pos.y, bounds, cachedCanvasSize?.w || canvas.clientWidth,
          cachedCanvasSize?.h || canvas.clientHeight);
        const hit = hitRect(pos.x, pos.y, model.rectangle, bounds,
          cachedCanvasSize?.w || canvas.clientWidth, cachedCanvasSize?.h || canvas.clientHeight);
        const orig = normalizedRect(model.rectangle);
        if (hit?.handle && orig) pointer = { mode: "resize", handle: hit.handle, orig, start: wpt };
        else if (hit?.move && orig) pointer = { mode: "move", orig, start: wpt };
        else pointer = { mode: "draw", start: wpt, orig };
      }
      canvas.setPointerCapture(ev.pointerId);
      if (ev.cancelable) ev.preventDefault();
    });
    canvas.addEventListener("pointermove", (ev) => {
      if (!pointer) return;
      const pos = pointerPosition(ev), dx = pos.x - pointer.x, dy = pos.y - pointer.y;
      if (pointer.mode === "top-pan") { panX = pointer.panX + dx; panY = pointer.panY + dy; draw(); return; }
      if (pointer.mode === "orbit") {
        const { w, h } = getSize();
        yaw += dx * 2 * Math.PI / Math.max(w, 1);
        elevation = clamp(elevation - dy * Math.PI / Math.max(h, 1), -8 * Math.PI / 180, 84 * Math.PI / 180);
        pointer.x = pos.x; pointer.y = pos.y; draw(); return;
      }
      if (pointer.mode === "orbit-pan") {
        const model = getModel(), bounds = getBounds(model), { w, h } = getSize(), basis = cameraBasis();
        const span = Math.max(bounds.x_max - bounds.x_min, bounds.y_max - bounds.y_min, bounds.z_max - bounds.z_min, 5);
        const distance = span * 2.15 / zoom, focal = Math.min(w, h) * 0.90;
        const units = distance / focal;
        target.x += -dx * units * basis.right[0] + dy * units * basis.up[0];
        target.y += -dx * units * basis.right[1] + dy * units * basis.up[1];
        target.z += dy * units * basis.up[2];
        pointer.x = pos.x; pointer.y = pos.y; draw(); return;
      }
      const model = getModel(), bounds = getBounds(model), { w, h } = getSize();
      const wpt = topWorld(pos.x, pos.y, bounds, w, h);
      let rect;
      if (pointer.mode === "draw") rect = { x_min: pointer.start.x, y_min: pointer.start.y, x_max: wpt.x, y_max: wpt.y };
      else if (pointer.mode === "move") {
        const ddx = wpt.x - pointer.start.x, ddy = wpt.y - pointer.start.y;
        rect = { x_min: pointer.orig.x_min + ddx, x_max: pointer.orig.x_max + ddx,
          y_min: pointer.orig.y_min + ddy, y_max: pointer.orig.y_max + ddy };
      } else rect = resizeRect(pointer.orig, pointer.handle, wpt.x, wpt.y);
      setRect(rect);
    });
    const finishPointer = (ev) => {
      if (!pointer) return;
      if (pointer.mode === "draw") {
        const model = getModel(), r = normalizedRect(model.rectangle);
        if (!r || r.x_max - r.x_min < 0.05 || r.y_max - r.y_min < 0.05) setRect(pointer.orig || null);
      }
      pointer = null; draw();
      if (ev && canvas.hasPointerCapture(ev.pointerId)) canvas.releasePointerCapture(ev.pointerId);
    };
    canvas.addEventListener("pointerup", finishPointer);
    canvas.addEventListener("pointercancel", finishPointer);
    canvas.addEventListener("contextmenu", (ev) => ev.preventDefault());
    canvas.addEventListener("wheel", (ev) => {
      ev.preventDefault();
      const pos = pointerPosition(ev), factor = Math.exp(-ev.deltaY * 0.0012);
      if (view === "top") {
        const model = getModel(), bounds = getBounds(model), { w, h } = getSize();
        const before = topWorld(pos.x, pos.y, bounds, w, h), oldScale = topScale(bounds, w, h);
        zoom = clamp(zoom * factor, 0.08, 24);
        const newScale = topScale(bounds, w, h), after = topWorld(pos.x, pos.y, bounds, w, h);
        panX += (after.x - before.x) * newScale;
        panY -= (after.y - before.y) * newScale;
        if (!Number.isFinite(oldScale)) { panX = 0; panY = 0; }
      } else zoom = clamp(zoom * factor, 0.35, 12);
      draw();
    }, { passive: false });

    draw();
    return {
      draw,
      setView(next) { view = next === "orbit" ? "orbit" : "top"; pointer = null; draw(); },
      reset() { zoom = 1; panX = 0; panY = 0; target = null; yaw = 0; elevation = 18 * Math.PI / 180; draw(); },
      resize: draw,
    };
  }

  window.LivoBodyViewer = { attach };
})();
