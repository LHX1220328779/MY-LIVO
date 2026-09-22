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
    "camlidar": {"order": 5, "label": "相机-LiDAR 外参精修", "command": "camera-lidar", "advisory": True, "mapping": "livo"},
    "precheck": {"order": 6, "label": "采纳建图自检修正", "command": "from-precheck", "advisory": False, "mapping": "any"},
    "manual": {"order": 7, "label": "手动设置", "command": "set", "advisory": False, "mapping": None},
}


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
    camera_ext_refined = "camlidar" in stages_done or "camera_lidar" in ctype
    checks = [
        {"key": "import", "label": "出厂标定已导入", "ok": bool(active), "stage": "import"},
        {"key": "lidar", "label": "LiDAR 外参已精修（地面/行驶段）", "ok": lidar_refined, "stage": "ground"},
        {"key": "driven", "label": "多 LiDAR 6-DoF 一致性已校正", "ok": driven_done, "stage": "driven"},
        {"key": "camtime", "label": "相机时间偏移已确认", "ok": camera_time_ok, "stage": "camtime",
         "value": offsets.get("camera")},
        {"key": "camlidar", "label": "相机外参已复核（可选）", "ok": camera_ext_refined, "stage": "camlidar", "optional": True},
    ]
    if not active:
        next_step = {"stage": "import", "pipeline": None, "reason": "该车辆还没有标定版本，先导入出厂标定 JSON"}
        level = "none"
    elif not lidar_refined or not driven_done:
        next_step = {"stage": "ground", "pipeline": "lidar", "reason": "运行 LiDAR 标定流水线（地面精修 → LIO 建图 → 行驶段精修）"}
        level = "lio"
    elif not camera_time_ok:
        next_step = {"stage": "camtime", "pipeline": "camera", "reason": "运行相机标定流水线（时间偏移 → LIVO 建图 → 相机外参复核）"}
        level = "lio"
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


def pipeline_steps(kind, body, mapping_cmd):
    """Standard pipelines. `mapping_cmd(round_name, mode, calibration_file, body)` builds the round command.

    lidar : ground -> LIO round (save_map, evaluated) -> driven -> validate [-> activate]
    camera: camera-time -> LIVO round A (camtime, evaluated) -> camera-lidar (advisory, never auto-activated)
            -> LIVO round B (camlidar, evaluated) -> closed-loop verdict A vs B (closed_loop.json) -> validate
    The closed loop is the acceptance test for the camera extrinsic: the edge-alignment score alone is not
    sensitive enough, whereas the native odometry scale reacts with ~11 %/deg of camera pitch error.
    """
    parent = _parent(body)
    dataset = str(safe_path(body.get("dataset") or ""))
    mine, truck = parent.split("/")[:2]
    stamp = dt.datetime.now().strftime("%m%d_%H%M%S")
    auto_activate = bool(body.get("activate", True))
    steps = []
    if kind == "lidar":
        datasets.require(dataset)
        steps.append({"title": "静止地面精修 (ground)", "build": lambda ctx: stage_cmd(
            "ground", {"id": parent, "dataset": dataset, "note": body.get("note") or "pipeline: ground"}),
            "after": lambda ctx, lines: ctx.__setitem__("ground_id", parse_version_id(lines))})
        round_name = f"calib_{truck}_lio_{stamp}"
        steps.append({"title": f"LIO 建图轮次 {round_name}（用于行驶段精修）", "build": lambda ctx: mapping_cmd(
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
            "camera-time", {"id": parent, "dataset": dataset, "note": body.get("note") or "pipeline: camera-time",
                            "activate": auto_activate}),
            "after": lambda ctx, lines: ctx.__setitem__("camtime_id", parse_version_id(lines))})
        round_name = f"calib_{truck}_livo_{stamp}"
        steps.append({"title": f"LIVO 建图轮次 {round_name}（用于相机外参复核）", "build": lambda ctx: mapping_cmd(
            round_name, "livo", str(calibration_file(ctx["camtime_id"])), body),
            "after": lambda ctx, lines: ctx.update(round=round_name, fusion_dir=latest_fusion_dir(round_name))})
        steps.append({"title": "相机-LiDAR 外参复核 (camera-lidar，仅供人工确认)", "build": lambda ctx: stage_cmd(
            "camera-lidar", {"id": ctx["camtime_id"], "dataset": dataset, "run": ctx["fusion_dir"],
                             "frames": body.get("frames") or 12, "note": body.get("note") or "pipeline: camera-lidar"}),
            "after": lambda ctx, lines: ctx.update(camlidar_id=parse_version_id(lines), result_id=ctx["camtime_id"])})
        round_b = f"calib_{truck}_livo_{stamp}_camlidar"
        steps.append({"title": f"LIVO 闭环验证轮次 {round_b}（camera-lidar 版本）", "build": lambda ctx: mapping_cmd(
            round_b, "livo", str(calibration_file(ctx["camlidar_id"])), body),
            "after": lambda ctx, lines: ctx.update(round_b=round_b)})
        steps.append({"title": "闭环判定：原生轨迹 A(camtime) vs B(camlidar)", "build": lambda ctx: [
            PY, str(REPO / "scripts" / "analysis" / "closed_loop_verdict.py"), ctx["round"], ctx["round_b"],
            "--version-dir", str(CAL_ROOT.joinpath(*ctx["camlidar_id"].split("/"))), "--exit-zero"],
            "after": lambda ctx, lines: ctx.update(closed_loop=_verdict(lines))})
        steps.append({"title": "校验新版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["camtime_id"], "--strict"]})
    else:
        raise StudioError(f"未知流水线 {kind}")
    return steps, {"mine": mine, "truck": truck, "parent": parent, "dataset": dataset}
