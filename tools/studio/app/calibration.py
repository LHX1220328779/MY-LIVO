"""Calibration versions (config/carlibration/<mine>/<truck>/vNNN_<stage>) and the standard
calibration pipelines. Every mutation is a subprocess of tools/calib/livo_calib.py."""
import datetime as dt
import json
import pathlib
import re
import threading

from . import datasets
from .common import CALIB_TOOL, CAL_ROOT, PY, REPO, RESULTS, StudioError, read_json, run_tool, safe_path

_cache = {"sig": None, "doc": None}
_lock = threading.Lock()

# Fallback catalogue used until livo_calib.py reports its own (same keys as STAGES in the tool).
STAGE_FALLBACK = {
    "import": {"order": 1, "label": "导入出厂标定", "command": "import", "advisory": False, "mapping": None},
    "ground": {"order": 2, "label": "静止地面精修", "command": "ground", "advisory": False, "mapping": None},
    "driven": {"order": 3, "label": "行驶段 6-DoF 精修", "command": "driven", "advisory": False, "mapping": "lio"},
    "camtime": {"order": 4, "label": "相机时间偏移", "command": "camera-time", "advisory": False, "mapping": None},
    "camlidar": {"order": 5, "label": "相机-LiDAR 偏航/横滚复核", "command": "camera-lidar", "advisory": True, "mapping": "livo"},
    "camloop": {"order": 6, "label": "相机外参闭环标定", "command": "set", "advisory": False, "mapping": "livo", "internal": True},
    "precheck": {"order": 7, "label": "采纳建图自检修正", "command": "from-precheck", "advisory": False, "mapping": "any"},
    "manual": {"order": 8, "label": "手动设置", "command": "set", "advisory": False, "mapping": None},
    "scan": {"order": 9, "label": "闭环扫描点（临时）", "command": "set", "advisory": True, "mapping": None, "internal": True},
}
SCAN_STEP_DEG = 0.05  # pitch increment of the closed-loop scan rounds (0.55 % scale change, 20x the run-to-run noise)
DEFAULT_PITCH_SLOPE = 0.105  # path-length ratio change per degree of camera pitch (measured 0.104-0.106 on truck29)
OVERLAY_DIR = RESULTS / ".studio" / "overlays"


def _signature():
    sig = []
    reg = CAL_ROOT / "registry.json"
    if reg.exists():
        sig.append(("registry", reg.stat().st_mtime_ns))
    for meta in list(CAL_ROOT.glob("*/*/*/meta.json")) + list(CAL_ROOT.glob("*/*/*/closed_loop.json")):
        try:
            sig.append((str(meta), meta.stat().st_mtime_ns))
        except OSError:
            continue
    return tuple(sorted(sig))


def catalogue(refresh=False):
    """{"active": {...}, "versions": [...], "stages": {...}, "sources": [...]} (cached on file mtimes)."""
    sig = _signature()
    with _lock:
        if _cache["doc"] is not None and _cache["sig"] == sig and not refresh:
            return _cache["doc"]
    rc, text, payload = run_tool([PY, str(CALIB_TOOL), "list", "--json"], timeout=60)
    if rc != 0 or payload is None:
        # tool unavailable: fall back to reading meta.json directly so the UI still works
        reg = read_json(CAL_ROOT / "registry.json", {"active": {}})
        versions = []
        for meta in sorted(CAL_ROOT.glob("*/*/*/meta.json")):
            m = read_json(meta, {"id": str(meta.parent.relative_to(CAL_ROOT)), "error": "meta unreadable"})
            versions.append(m)
        payload = {"active": reg.get("active", {}), "versions": versions, "stages": STAGE_FALLBACK,
                   "tool_error": text.strip()[-400:]}
    payload.setdefault("stages", STAGE_FALLBACK)
    ids = {v.get("id") for v in payload["versions"]}
    for v in payload["versions"]:
        stage = v.get("stage")
        v.setdefault("stage_label", payload["stages"].get(stage, {}).get("label", stage))
        v.setdefault("advisory", bool(payload["stages"].get(stage, {}).get("advisory")))
        v.setdefault("children", sorted(c.get("id") for c in payload["versions"] if c.get("parent") == v.get("id")))
        v.setdefault("active", payload["active"].get(f"{v.get('mine')}/{v.get('truck')}") == v.get("id"))
        v["parent_missing"] = bool(v.get("parent")) and v.get("parent") not in ids
        d = CAL_ROOT.joinpath(*str(v.get("id", "")).strip("/").split("/"))
        v["dir"] = str(d)
        v["overlays"] = sorted(p.name for p in (d / "overlays").glob("*.*")) if (d / "overlays").is_dir() else []
        cl = read_json(d / "closed_loop.json", None) if (d / "closed_loop.json").is_file() else None
        v["closed_loop"] = cl.get("verdict") if isinstance(cl, dict) else None
    payload["sources"] = loose_sources()
    with _lock:
        _cache.update(sig=sig, doc=payload)
    return payload


def loose_sources():
    files = [p for p in sorted(CAL_ROOT.glob("*.json")) if p.name not in ("registry.json", "studio_state.json")]
    files += sorted((REPO / "config").glob("perception_*.json"))
    return [str(p) for p in files]


def lineage(versions_by_id, vid):
    chain, seen = [], set()
    while vid and vid in versions_by_id and vid not in seen:
        seen.add(vid)
        chain.insert(0, vid)
        vid = versions_by_id[vid].get("parent")
    return chain


def vehicle_status(cat, mine, truck):
    """Readiness of one truck: active version, which stages its lineage contains, recommended next step."""
    key = f"{mine}/{truck}"
    versions = [v for v in cat["versions"] if v.get("mine") == mine and v.get("truck") == truck]
    by_id = {v["id"]: v for v in versions}
    active_id = cat["active"].get(key)
    active = by_id.get(active_id)
    stages_done = []
    calib = {}
    if active:
        stages_done = [by_id[i].get("stage") for i in lineage(by_id, active_id)]
        calib = read_json(CAL_ROOT.joinpath(*active_id.split("/")) / "calibration.json", {}) or {}
    ctype = str(calib.get("calibration_type", ""))
    offsets = calib.get("time_offsets") or {}
    lidar_refined = any(s in ("ground", "driven", "precheck") for s in stages_done) or "refined" in ctype
    driven_done = "driven" in stages_done or "driven" in ctype or "stage2" in ctype
    camera_time_ok = isinstance(offsets.get("camera"), (int, float)) and (offsets.get("camera") != 0.0 or "camera_source" in offsets)
    camera_ext_refined = "camlidar" in stages_done or "camera_lidar" in ctype or "camera_pole_yaw" in ctype
    camera_loop_done = "camloop" in stages_done or "camera_closed_loop" in ctype
    checks = [
        {"key": "import", "label": "出厂标定已导入", "ok": bool(active), "stage": "import"},
        {"key": "lidar", "label": "LiDAR 外参已精修（地面/行驶段）", "ok": lidar_refined, "stage": "ground"},
        {"key": "driven", "label": "多 LiDAR 6-DoF 一致性已校正", "ok": driven_done, "stage": "driven"},
        {"key": "camtime", "label": "相机时间偏移已确认", "ok": camera_time_ok, "stage": "camtime",
         "value": offsets.get("camera")},
        {"key": "camlidar", "label": "相机偏航已复核（灯柱投影）", "ok": camera_ext_refined, "stage": "camlidar", "optional": True},
        {"key": "camloop", "label": "相机俯仰已闭环标定（原生里程尺度）", "ok": camera_loop_done, "stage": "camloop", "optional": True},
    ]
    if not active:
        next_step = {"stage": "import", "pipeline": None, "reason": "该车辆还没有标定版本，先导入出厂标定 JSON"}
        level = "none"
    elif not lidar_refined or not driven_done:
        next_step = {"stage": "ground", "pipeline": "lidar", "reason": "运行 LiDAR 标定流水线（地面精修 → LIO 建图 → 行驶段精修）"}
        level = "lio"
    elif not camera_time_ok:
        next_step = {"stage": "camtime", "pipeline": "camera", "reason": "运行相机标定流水线（时间偏移 → LIVO 建图 → 灯柱偏航复核 → 俯仰闭环扫描 → 验证）"}
        level = "lio"
    elif not camera_loop_done:
        next_step = {"stage": "camloop", "pipeline": "camloop",
                     "reason": "可建图；建议运行“相机外参闭环微调”（俯仰扫描 ±0.05° → 尺度过零点 → LIVO 验证），消除 LIVO 原生轨迹的尺度漂移"}
        level = "livo"
    else:
        next_step = {"stage": None, "pipeline": None, "reason": "标定完整，可直接建图"}
        level = "livo"
    return {"mine": mine, "truck": truck, "id": key, "active": active_id, "active_version": active,
            "versions": len(versions), "latest": max((v.get("created") or "" for v in versions), default=""),
            "stages_done": stages_done, "checks": checks, "next": next_step, "ready_level": level,
            "time_offsets": offsets, "calibration_file": str(CAL_ROOT.joinpath(*active_id.split("/")) / "calibration.json") if active_id else None}


def vehicles(cat=None):
    cat = cat or catalogue()
    pairs = sorted({(v.get("mine"), v.get("truck")) for v in cat["versions"] if v.get("mine") and v.get("truck")})
    for d in CAL_ROOT.glob("*/*/"):
        if d.is_dir() and d.parent.name not in ("",) and not d.name.startswith("."):
            pairs.append((d.parent.name, d.name))
    pairs = sorted(set(pairs))
    return [vehicle_status(cat, m, t) for m, t in pairs]


def version_detail(vid):
    d = CAL_ROOT.joinpath(*vid.strip("/").split("/"))
    if not (d / "calibration.json").exists():
        raise StudioError(f"标定版本不存在：{vid}")
    cat = catalogue()
    by_id = {v["id"]: v for v in cat["versions"]}
    meta = read_json(d / "meta.json", {}) or {}
    files = {}
    for name in ("report.md", "stage_report.txt", "camera_time.json", "stage.log"):
        if (d / name).exists():
            files[name] = (d / name).read_text(errors="replace")[-60_000:]
    rc, text, validation = run_tool([PY, str(CALIB_TOOL), "validate", "--id", vid], timeout=60)
    if validation is None:
        validation = {"ok": rc == 0, "errors": [text.strip()[-300:]] if rc != 0 else [], "warnings": []}
    return {"id": vid, "meta": meta, "files": files, "calibration": read_json(d / "calibration.json", {}),
            "overlays": sorted(p.name for p in (d / "overlays").glob("*.*")) if (d / "overlays").is_dir() else [],
            "closed_loop": read_json(d / "closed_loop.json", None) if (d / "closed_loop.json").is_file() else None,
            "children": by_id.get(vid, {}).get("children", []), "lineage": lineage(by_id, vid),
            "active": cat["active"].get(f"{meta.get('mine')}/{meta.get('truck')}") == vid,
            "validation": validation, "stage_info": cat["stages"].get(meta.get("stage"), {}), "dir": str(d)}


def version_file(vid, name):
    d = CAL_ROOT.joinpath(*vid.strip("/").split("/"))
    return safe_path(d / name)


def activate(vid):
    rc, text, payload = run_tool([PY, str(CALIB_TOOL), "activate", "--id", vid])
    if rc != 0:
        raise StudioError(text.strip()[-400:] or "激活失败")
    return payload or {"ok": True}


def delete(vid):
    rc, text, payload = run_tool([PY, str(CALIB_TOOL), "delete", "--id", vid])
    if rc != 0:
        raise StudioError(text.strip()[-400:] or "删除失败")
    return payload or {"ok": True}


def validate(vid, strict=False):
    cmd = [PY, str(CALIB_TOOL), "validate", "--id", vid] + (["--strict"] if strict else [])
    rc, text, payload = run_tool(cmd)
    return payload or {"ok": rc == 0, "errors": [text.strip()[-300:]], "warnings": []}


def calibration_file(vid):
    f = CAL_ROOT.joinpath(*vid.strip("/").split("/")) / "calibration.json"
    if not f.exists():
        raise StudioError(f"标定版本不存在：{vid}")
    return f


def active_id(mine, truck):
    return catalogue()["active"].get(f"{mine}/{truck}")


# ------------------------------------------------------------------ commands
def _note(body, default):
    return body.get("note") or default


def import_cmd(body):
    src = safe_path(body["source"])
    if not src.is_file():
        raise StudioError(f"出厂标定文件不存在：{src}")
    mine, truck = _ident(body)
    cmd = [PY, str(CALIB_TOOL), "import", "--source", str(src), "--mine", mine, "--truck", truck]
    for key, flag in (("lidar_offset", "--lidar-offset"), ("camera_offset", "--camera-offset")):
        if body.get(key) not in (None, ""):
            cmd += [flag, str(body[key])]
    cmd += ["--note", _note(body, f"imported from {src.name}")]
    if body.get("activate", True):
        cmd.append("--activate")
    return cmd


def _ident(body):
    mine = re.sub(r"[^A-Za-z0-9_\-]", "", str(body.get("mine") or ""))
    truck = re.sub(r"[^A-Za-z0-9_\-]", "", str(body.get("truck") or ""))
    if not mine or not truck:
        raise StudioError("矿山名和车辆编号只能包含字母、数字、下划线或短横线")
    return mine, truck


def _parent(body):
    vid = body.get("id") or body.get("parent_id")
    if not vid:
        raise StudioError("缺少父版本 id")
    calibration_file(vid)
    return vid


def _run_dir(raw):
    run = safe_path(raw)
    if (run / "Log" / "fusion").is_dir():  # a round directory was given: take its latest fusion run
        runs = sorted((run / "Log" / "fusion").glob("*/"))
        if not runs:
            raise StudioError(f"{run} 没有 fusion 输出")
        run = runs[-1]
    if not (run / "frames.csv").exists():
        raise StudioError(f"{run} 不是保存了地图帧的 fusion 结果目录（需要建图时开启“保存地图”）")
    return run


def stage_cmd(stage, body):
    """Command line for one calibration stage (also used by the pipelines through `build`)."""
    stages = catalogue()["stages"]
    command = stages.get(stage, {}).get("command", stage)
    if command == "import":
        return import_cmd(body)
    parent = _parent(body)
    cmd = [PY, str(CALIB_TOOL), command, "--id", parent]
    if command in ("ground", "driven", "camera-time", "camera-lidar"):
        dataset = str(safe_path(body.get("dataset") or ""))
        datasets.require(dataset, camera=command in ("camera-time", "camera-lidar"))
        cmd += ["--bag", dataset]
    if command in ("driven", "camera-lidar"):
        cmd += ["--run", str(_run_dir(body.get("run") or ""))]
    if command == "ground":
        if body.get("stationary"):
            cmd += ["--stationary", *[str(int(v)) for v in body["stationary"]]]
        if body.get("moving"):
            cmd += ["--moving", *[str(int(v)) for v in body["moving"]]]
    if command == "driven" and body.get("window"):
        cmd += ["--window", *[str(float(v)) for v in body["window"]]]
    if command == "camera-lidar":
        cmd += ["--frames", str(int(body.get("frames") or 12))]
        if body.get("solve_intrinsics"):
            cmd.append("--solve-intrinsics")
    if command == "from-precheck":
        cmd += ["--run-dir", str(safe_path(body.get("run_dir") or ""))]
    if command == "set":
        for k, flag in (("camera_offset", "--camera-offset"), ("lidar_offset", "--lidar-offset")):
            if body.get(k) not in (None, ""):
                cmd += [flag, str(body[k])]
        for k, flag in (("intrinsics", "--intrinsics"), ("distortion", "--distortion"),
                        ("camera_rot_delta", "--camera-rot-delta")):
            if body.get(k):
                cmd += [flag, *[str(v) for v in body[k]]]
        if len(cmd) == 5:
            raise StudioError("手动设置至少需要填写一个字段")
    if body.get("note"):
        cmd += ["--note", body["note"]]
    if body.get("activate"):
        cmd.append("--activate")
    return cmd


def parse_version_id(lines):
    for line in reversed(lines):
        line = line.strip()
        if line.startswith("{"):
            try:
                doc = json.loads(line)
            except json.JSONDecodeError:
                continue
            if doc.get("id"):
                return doc["id"]
    raise StudioError("标定阶段没有输出新版本 id")


def _verdict(lines):
    text = "\n".join(lines)
    i = text.find("{")
    if i < 0:
        return None
    try:
        return json.loads(text[i:]).get("verdict")
    except json.JSONDecodeError:
        return None


def latest_fusion_dir(round_name):
    runs = sorted((RESULTS / round_name / "Log" / "fusion").glob("*/"))
    if not runs:
        raise StudioError(f"建图轮次 {round_name} 没有 fusion 输出")
    return str(runs[-1]).rstrip("/")


def _json_tail(lines):
    for line in reversed(lines):
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    raise StudioError("步骤没有输出 JSON 结果")


def _camloop_steps(steps, body, mapping_cmd, *, truck, stamp, auto_activate, base_key, round0_key, baseline_key,
                   standard=True):
    """Closed-loop camera pitch appended to `steps`.

    ctx[base_key]     version whose camera extrinsic is tuned (pitch increment 0 == ctx[round0_key] round)
    ctx[baseline_key] round that is the A side of the final verdict

    standard: two temporary `scan` versions (pitch +-SCAN_STEP_DEG about the camera x axis) are mapped and the
              three native path-length ratios are fitted (slope + residual checked). ~11 min, 4 rounds.
    fast    : one measured point (the zero-pitch round) with the slope taken from the vehicle's last standard
              run (`pitch_slope`), i.e. only the intercept is measured. ~4 min, 2 rounds. Falls back to the
              standard fit when the vehicle has no measured slope yet or the scale is off by > 2 %.
    """
    reuse = None
    if not standard and body.get("reuse_round", True):
        reuse = reusable_round(body.get("_parent_id") or "", min_duration=float(body.get("min_reuse_s") or 110))
    if reuse:
        steps.append({"title": f"复用已有零俯仰轮次 {reuse['round']}（同版本，{reuse['duration_s']:.0f} s，尺度 {reuse.get('scale')}）",
                      "build": lambda ctx, r=reuse["round"]: ["/bin/echo", f"reuse round {r} as the zero-pitch point"],
                      "after": lambda ctx, lines, r=reuse["round"]: ctx.__setitem__(round0_key, r)})
    elif not standard:
        rn = f"calib_{truck}_livo_{stamp}_base"
        steps.append({"title": f"LIVO 零俯仰建图 {rn}（当前版本）", "retry": 1, "round": rn, "build": lambda ctx, rn=rn: mapping_cmd(
            rn, "livo", str(calibration_file(ctx[base_key])), body),
            "after": lambda ctx, lines, rn=rn: ctx.__setitem__(round0_key, rn)})
    if standard:
        for sign, tag in ((+1, "up"), (-1, "dn")):
            dp = sign * SCAN_STEP_DEG
            steps.append({"title": f"俯仰扫描点 Δpitch {dp:+.2f}°（临时版本，不导出地图）", "build": lambda ctx, dp=dp: [
                PY, str(CALIB_TOOL), "set", "--id", ctx[base_key], "--camera-rot-delta", f"{dp:.3f}", "0", "0",
                "--stage", "scan", "--note", f"闭环俯仰扫描点 Δpitch={dp:+.2f}°（流水线临时版本，勿激活）"],
                "after": lambda ctx, lines, tag=tag: ctx.__setitem__(f"scan_{tag}_id", parse_version_id(lines))})
            rn = f"calib_{truck}_livo_{stamp}_scan_{tag}"
            steps.append({"title": f"LIVO 扫描建图 {rn}（Δpitch {dp:+.2f}°，只评估轨迹）", "retry": 1, "round": rn,
                          "build": lambda ctx, rn=rn, tag=tag: mapping_cmd(
                              rn, "livo", str(calibration_file(ctx[f"scan_{tag}_id"])), body,
                              save_map=False, quick=True),
                          "after": lambda ctx, lines, rn=rn, tag=tag: ctx.__setitem__(f"scan_{tag}_round", rn)})
        steps.append({"title": "拟合原生里程尺度 vs 俯仰增量，求过零点", "build": lambda ctx: [
            PY, str(REPO / "scripts" / "analysis" / "camera_pitch_fit.py"),
            "--point", f"{ctx[round0_key]}=0", "--point", f"{ctx['scan_up_round']}={SCAN_STEP_DEG}",
            "--point", f"{ctx['scan_dn_round']}={-SCAN_STEP_DEG}"],
            "after": lambda ctx, lines: ctx.update(pitch_fit=_json_tail(lines),
                                                   pitch_delta_deg=_json_tail(lines).get("pitch_delta_deg"))})
    else:
        steps.append({"title": f"用已知俯仰斜率 {body.get('pitch_slope', DEFAULT_PITCH_SLOPE):.4f}/° 求过零点（只测零点）",
                      "build": lambda ctx: [
            PY, str(REPO / "scripts" / "analysis" / "camera_pitch_fit.py"), "--point", f"{ctx[round0_key]}=0",
            "--slope", f"{body.get('pitch_slope', DEFAULT_PITCH_SLOPE):.4f}", "--max-abs", "0.15"],
            "after": lambda ctx, lines: ctx.update(pitch_fit=_json_tail(lines),
                                                   pitch_delta_deg=_json_tail(lines).get("pitch_delta_deg"),
                                                   pitch_slope_source=body.get("pitch_slope_source"))})

    def set_cmd(ctx):
        fit = ctx["pitch_fit"]
        note = (f"相机俯仰闭环：Δpitch={fit['pitch_delta_deg']:+.3f}°（尺度斜率 {fit['slope_per_deg']:.3f}/°"
                f"{'，沿用历史值 ' + str(ctx.get('pitch_slope_source') or '模型') if fit.get('slope_source') else ''}"
                f"，{len(fit['points'])} 轮，预测原生尺度 {fit['predicted_ratio']:.4f}）")
        return [PY, str(CALIB_TOOL), "set", "--id", ctx[base_key], "--camera-rot-delta", f"{fit['pitch_delta_deg']:.4f}", "0", "0",
                "--stage", "camloop", "--metrics-json", json.dumps({"pitch_fit": fit}, ensure_ascii=False), "--note", note]
    steps.append({"title": "生成闭环标定版本 (camloop)", "build": set_cmd,
                  "after": lambda ctx, lines: ctx.update(camloop_id=parse_version_id(lines), result_id=parse_version_id(lines))})
    rv = f"calib_{truck}_livo_{stamp}_verify"
    steps.append({"title": f"LIVO 验证建图 {rv}（闭环版本）", "retry": 1, "round": rv, "build": lambda ctx: mapping_cmd(
        rv, "livo", str(calibration_file(ctx["camloop_id"])), body),
        "after": lambda ctx, lines: ctx.__setitem__("verify_round", rv)})
    margin = "0.002" if standard else "0.0005"
    steps.append({"title": "闭环判定：基线轮次 vs 闭环版本（原生里程尺度 / 后轴误差 / 偏航）", "build": lambda ctx: [
        PY, str(REPO / "scripts" / "analysis" / "closed_loop_verdict.py"), ctx[baseline_key], ctx["verify_round"],
        "--version-dir", str(CAL_ROOT.joinpath(*ctx["camloop_id"].split("/"))), "--scale-margin", margin, "--exit-zero"],
        "after": lambda ctx, lines: ctx.update(closed_loop=_verdict(lines))})
    steps.append({"title": "校验新版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["camloop_id"], "--strict"]})

    def activate_cmd(ctx):
        if ctx.get("closed_loop") == "pass" and auto_activate:
            return [PY, str(CALIB_TOOL), "activate", "--id", ctx["camloop_id"]]
        why = "闭环未通过" if ctx.get("closed_loop") != "pass" else "未勾选自动激活"
        return [PY, "-c", f"print('not activated: {why}; active version unchanged')"]
    steps.append({"title": "激活闭环版本（仅闭环通过且勾选自动激活时）", "build": activate_cmd,
                  "after": lambda ctx, lines: ctx.__setitem__("activated", ctx.get("closed_loop") == "pass" and auto_activate)})


def pipeline_steps(kind, body, mapping_cmd):
    """Standard pipelines. `mapping_cmd(round_name, mode, calibration_file, body)` builds the round command.

    lidar  : ground -> LIO round (save_map, evaluated) -> driven -> validate [-> activate]
    camera : camera-time -> LIVO round A -> camera-lidar (pole yaw, pitch untouched)
             -> pitch scan (2 temporary versions, 2 LIVO rounds) -> zero crossing -> camloop version
             -> LIVO verify round -> closed-loop verdict A vs verify -> validate [-> activate if pass]
    camloop: same pitch closed loop starting from the parent version (its own LIVO round is the 0 point)
    The native odometry scale reacts with ~11 %/deg of camera pitch error and is repeatable to ~0.03 %,
    so it is the acceptance test; edge/pole alignment alone cannot resolve pitch (36-line ring spacing).
    """
    parent = _parent(body)
    dataset = str(safe_path(body.get("dataset") or ""))
    mine, truck = parent.split("/")[:2]
    standard = (body.get("mode") or "standard") != "fast"
    slope_info = pitch_slope(mine, truck)
    body = {**body, "_parent_id": parent, "pitch_slope": slope_info["slope"], "pitch_slope_source": slope_info["source"]}
    stamp = dt.datetime.now().strftime("%m%d_%H%M%S")
    auto_activate = bool(body.get("activate", True))
    steps = []
    if kind == "lidar":
        datasets.require(dataset)
        steps.append({"title": "静止地面精修 (ground)", "build": lambda ctx: stage_cmd(
            "ground", {"id": parent, "dataset": dataset, "note": body.get("note") or "pipeline: ground"}),
            "after": lambda ctx, lines: ctx.__setitem__("ground_id", parse_version_id(lines))})
        round_name = f"calib_{truck}_lio_{stamp}"
        steps.append({"title": f"LIO 建图轮次 {round_name}（用于行驶段精修）", "retry": 1, "round": round_name, "build": lambda ctx: mapping_cmd(
            round_name, "lio", str(calibration_file(ctx["ground_id"])), body),
            "after": lambda ctx, lines: ctx.update(round=round_name, fusion_dir=latest_fusion_dir(round_name))})
        steps.append({"title": "行驶段 6-DoF 精修 (driven)", "build": lambda ctx: stage_cmd(
            "driven", {"id": ctx["ground_id"], "dataset": dataset, "run": ctx["fusion_dir"],
                       "note": body.get("note") or "pipeline: driven", "activate": auto_activate}),
            "after": lambda ctx, lines: ctx.__setitem__("driven_id", parse_version_id(lines))})
        steps.append({"title": "校验新版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["driven_id"]],
                      "after": lambda ctx, lines: ctx.__setitem__("result_id", ctx["driven_id"])})
    elif kind == "camera":
        datasets.require(dataset, camera=True)
        steps.append({"title": "相机时间偏移 (camera-time)", "build": lambda ctx: stage_cmd(
            "camera-time", {"id": parent, "dataset": dataset, "note": body.get("note") or "pipeline: camera-time"}),
            "after": lambda ctx, lines: ctx.__setitem__("camtime_id", parse_version_id(lines))})
        round_name = f"calib_{truck}_livo_{stamp}"
        steps.append({"title": f"LIVO 建图轮次 A {round_name}（基线，俯仰增量 0）", "retry": 1, "round": round_name, "build": lambda ctx: mapping_cmd(
            round_name, "livo", str(calibration_file(ctx["camtime_id"])), body),
            "after": lambda ctx, lines: ctx.update(round=round_name, fusion_dir=latest_fusion_dir(round_name))})
        steps.append({"title": "相机-LiDAR 偏航复核 (camera-lidar：灯柱投影列偏差，俯仰不动)", "build": lambda ctx: stage_cmd(
            "camera-lidar", {"id": ctx["camtime_id"], "dataset": dataset, "run": ctx["fusion_dir"],
                             "frames": body.get("frames") or 30, "note": body.get("note") or ""}),
            "after": lambda ctx, lines: ctx.update(camlidar_id=parse_version_id(lines), result_id=ctx["camtime_id"])})
        _camloop_steps(steps, body, mapping_cmd, truck=truck, stamp=stamp, auto_activate=auto_activate,
                       base_key="camlidar_id", round0_key="round", baseline_key="round", standard=standard)
    elif kind == "camloop":
        datasets.require(dataset, camera=True)
        steps.append({"title": "当前版本作为待微调版本", "build": lambda ctx: ["/bin/echo", "base version prepared"],
                      "after": lambda ctx, lines: ctx.__setitem__("base_id", parent)})
        _camloop_steps(steps, body, mapping_cmd, truck=truck, stamp=stamp, auto_activate=auto_activate,
                       base_key="base_id", round0_key="round", baseline_key="round", standard=standard)
    else:
        raise StudioError(f"未知流水线 {kind}")
    return steps, {"mine": mine, "truck": truck, "parent": parent, "dataset": dataset,
                   "mode": "standard" if standard else "fast",
                   "pitch_slope": None if standard else slope_info["slope"],
                   "pitch_slope_source": None if standard else slope_info["source"]}


def overlay(body):
    """Render LiDAR->camera projection overlays for a version (optionally with a trial rotation delta / time
    offset) using the scans of a LIVO round; ~10 s. Returns the image list served by /api/overlays/file."""
    vid = body.get("id") or ""
    calibration_file(vid)
    run = _run_dir(body.get("run") or "")
    dataset = str(safe_path(body.get("dataset") or ""))
    datasets.require(dataset, camera=True)
    times = [float(v) for v in str(body.get("times") or "20,60,100").replace(" ", "").split(",") if v]
    if not times or len(times) > 8:
        raise StudioError("请给出 1–8 个时刻（秒）")
    delta = body.get("camera_rot_delta")
    if isinstance(delta, str):
        delta = [float(v) for v in delta.replace(",", " ").split()] if delta.strip() else None
    if delta is not None and len(delta) != 3:
        raise StudioError("相机旋转修正需要 3 个角度 rx ry rz（度）")
    key = re.sub(r"[^A-Za-z0-9_.-]", "_", f"{vid}_{run.name}_{'_'.join(str(t) for t in times)}_"
                 f"{'_'.join(str(v) for v in (delta or [0, 0, 0]))}_{body.get('camera_offset') or 'v'}")[:150]
    out = OVERLAY_DIR / key
    if not (out / "index.json").exists():
        out.mkdir(parents=True, exist_ok=True)
        cmd = [PY, str(CALIB_TOOL), "overlay", "--id", vid, "--bag", dataset, "--run", str(run), "--out", str(out),
               "--times", ",".join(str(t) for t in times)]
        if delta is not None and any(abs(v) > 0 for v in delta):
            cmd += ["--camera-rot-delta", *[str(v) for v in delta]]
        if body.get("camera_offset") not in (None, ""):
            cmd += ["--camera-offset", str(float(body["camera_offset"]))]
        rc, text, _ = run_tool(cmd, timeout=300)
        if rc != 0:
            raise StudioError("叠加图生成失败：" + text.strip()[-400:])
    doc = read_json(out / "index.json", {}) or {}
    doc["key"] = key
    doc["images"] = [{"t": f.get("t"), "url": f"/api/overlays/file?key={key}&name={f['file']}", **f}
                     for f in doc.get("frames", []) if f.get("file")]
    return doc


def overlay_file(key, name):
    key = re.sub(r"[^A-Za-z0-9_.-]", "", key or "")
    return safe_path(OVERLAY_DIR / key / name)


def pitch_slope(mine, truck):
    """Slope of the image-scale-vs-camera-pitch relation from the newest version that measured it."""
    root = CAL_ROOT / mine / truck
    for d in sorted(root.glob("v*_*"), reverse=True):
        fit = ((read_json(d / "meta.json", {}) or {}).get("metrics") or {}).get("pitch_fit") or {}
        b = fit.get("slope_per_deg")
        if isinstance(b, (int, float)) and 0.03 <= b <= 0.4:
            return {"slope": float(b), "source": f"{mine}/{truck}/{d.name}"}
    return {"slope": DEFAULT_PITCH_SLOPE, "source": None}


def reusable_round(calibration_id, min_duration=110.0):
    """The newest finished LIVO round that already ran with this calibration: a free zero-pitch point for the
    closed loop (same version, full length), so the fast mode only has to map the verify round."""
    from . import mapping as mapping_module
    target = str(calibration_file(calibration_id))
    for r in mapping_module.list_runs(limit=40):
        if r.get("status") != "done" or r.get("mode") != "livo":
            continue
        if str(r.get("calibration") or "") != target:
            continue
        doc = read_json(pathlib.Path(r["path"]) / "evaluation" / "metrics.json", {}) or {}
        run = doc.get("run") or {}
        dur = float(run.get("duration_s") or 0)
        if run.get("frames") and dur >= min_duration:
            return {"round": r["round"], "duration_s": dur, "start": r.get("start"),
                    "scale": (doc.get("headline", {}).get("native") or {}).get("path_length_ratio")}
    return None


