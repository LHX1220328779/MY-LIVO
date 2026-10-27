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
    "axle": {"order": 2, "label": "后轴对齐（RTK/INS）", "command": "axle", "advisory": False, "mapping": None},
    "ground": {"order": 3, "label": "静止地面精修", "command": "ground", "advisory": False, "mapping": None},
    "driven": {"order": 4, "label": "行驶段 6-DoF 精修", "command": "driven", "advisory": False, "mapping": "lio"},
    "rtktime": {"order": 4.5, "label": "RTK 位置关联时间", "command": "rtk-time", "advisory": False, "mapping": "lio"},
    "gcons": {"order": 4.7, "label": "后向雷达地面一致性（tilt/z）", "command": "gcons", "advisory": False, "mapping": "lio"},
    "camtime": {"order": 5, "label": "相机时间偏移", "command": "camera-time", "advisory": False, "mapping": None},
    "camlidar": {"order": 6, "label": "相机-LiDAR 偏航/横滚复核", "command": "camera-lidar", "advisory": True, "mapping": "livo"},
    "camloop": {"order": 7, "label": "相机外参闭环标定", "command": "set", "advisory": False, "mapping": "livo", "internal": True},
    "camphoto": {"order": 7.5, "label": "相机光度标定（内参复核 + 外参/时间精修 + 暗角）", "command": "camera-photo", "advisory": False, "mapping": "livo"},
    "precheck": {"order": 8, "label": "采纳建图自检修正", "command": "from-precheck", "advisory": False, "mapping": "any"},
    "manual": {"order": 9, "label": "手动设置", "command": "set", "advisory": False, "mapping": None},
    "scan": {"order": 10, "label": "闭环扫描点（临时）", "command": "set", "advisory": True, "mapping": None, "internal": True},
}
# LIO rounds of the RTK time-association loop: long enough to contain start-up acceleration, turns and
# speed changes (a 48 s single pass leaves the offset unobservable: jackknife se 7 ms on truck28), capped by
# the recording; the window start is the pipeline's motion window like every other round.
RTK_ROUND_S = 240.0
SCAN_STEP_DEG = 0.05  # pitch increment of the closed-loop scan rounds (0.55 % scale change, 20x the run-to-run noise)
DEFAULT_PITCH_SLOPE = 0.105  # path-length ratio change per degree of camera pitch (measured 0.104-0.106 on truck29)

# Injected into the selected parent calibration.json before ANY automated calibration stage.
# The active vehicle calibration is the source of truth for body_filter; older/factory parents
# receive a copy (with a one-time backup) so every child version and mapping round inherits it.
BODY_FILTER_PREP_CODE = r"""
import json, pathlib, sys
sys.path.insert(0, sys.argv[3])
from livo.body_filter import normalize
parent = pathlib.Path(sys.argv[1])
source = pathlib.Path(sys.argv[2])
doc = json.loads(parent.read_text())
if source != parent and source.exists():
    src_doc = json.loads(source.read_text())
else:
    src_doc = doc
if "body_filter" in doc and str(doc["body_filter"].get("source", "")).startswith("Studio"):
    # A parent already tuned in the visual editor keeps its own vehicle-specific mask.
    bf = normalize(doc)
else:
    if "body_filter" not in src_doc and "body_filter" in doc:
        src_doc = doc
    bf = normalize(src_doc)
old = doc.get("body_filter")
if old != bf:
    bak = parent.with_name(parent.name + ".pre_body_filter.bak")
    if not bak.exists():
        bak.write_text(parent.read_text())
    doc["body_filter"] = bf
    parent.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
print(json.dumps({"body_filter_injected": old != bf, "parent": str(parent),
                  "source": str(source), "enabled": bf["enabled"],
                  "minimum_z": bf["minimum_z"]}, ensure_ascii=False))
"""

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
            m = read_json(meta, {"error": "meta unreadable"})
            m["id"] = str(meta.parent.relative_to(CAL_ROOT))  # directory is the identity (copied meta.json)
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


def factory_root(vid):
    """Factory import (stage import) at the root of the lineage of `vid`; None for versions without one."""
    by_id = {v["id"]: v for v in catalogue()["versions"]}
    chain = lineage(by_id, vid)
    return chain[0] if chain and by_id[chain[0]].get("stage") == "import" else None


def vehicle_status(cat, mine, truck):
    """Readiness of one truck: active version, which stages its lineage contains, recommended next step."""
    key = f"{mine}/{truck}"
    versions = [v for v in cat["versions"] if v.get("mine") == mine and v.get("truck") == truck]
    by_id = {v["id"]: v for v in versions}
    active_id = cat["active"].get(key)
    active = by_id.get(active_id)
    # Working chain = lineage of the newest leaf (the branch currently being built/tested), not of the
    # active version: a pipeline must light up its checklist the moment its stages land, before
    # activation. Temporary scan versions never pick the tip.
    leaves = [v for v in versions if not v.get("children") and v.get("stage") != "scan"]
    tip = max(leaves or versions, key=lambda v: (v.get("created") or "", v.get("version") or "")) if versions else None
    work = tip or active
    stages_done = []
    calib = {}
    root_ctype = ""
    if work:
        chain = lineage(by_id, work["id"])
        stages_done = [by_id[i].get("stage") for i in chain]
        calib = read_json(CAL_ROOT.joinpath(*work["id"].split("/")) / "calibration.json", {}) or {}
        if chain:
            root_ctype = str((read_json(CAL_ROOT.joinpath(*chain[0].split("/")) / "calibration.json", {}) or {})
                             .get("calibration_type", ""))
    ctype = str(calib.get("calibration_type", ""))
    offsets = calib.get("time_offsets") or {}
    # no import is aligned to the rear axle until the axle stage ran (hand-tuned exports included: truck29
    # v001_import was 0.2 deg off the INS)
    axle_done = "axle" in stages_done or "rear_axle_alignment" in ctype
    lidar_refined = any(s in ("ground", "driven", "precheck") for s in stages_done) or "refined" in ctype
    driven_done = "driven" in stages_done or "driven" in ctype or "stage2" in ctype
    rtk_time_done = "rtktime" in stages_done or "rtk-time" in str(offsets.get("rtk_position_source", ""))
    camera_time_ok = isinstance(offsets.get("camera"), (int, float)) and (offsets.get("camera") != 0.0 or "camera_source" in offsets)
    camera_ext_refined = "camlidar" in stages_done or "camera_lidar" in ctype or "camera_pole_yaw" in ctype
    camera_loop_failed = bool(work) and work.get("closed_loop") == "fail"
    camera_loop_done = ("camloop" in stages_done or "camera_closed_loop" in ctype) and not camera_loop_failed
    checks = [
        {"key": "import", "label": "出厂标定已导入", "ok": bool(active), "stage": "import"},
        {"key": "axle", "label": "参考雷达已对齐后轴（RTK/INS 位姿）", "ok": axle_done, "stage": "axle"},
        {"key": "lidar", "label": "LiDAR 外参已精修（地面/行驶段）", "ok": lidar_refined, "stage": "ground"},
        {"key": "driven", "label": "多 LiDAR 6-DoF 一致性已校正", "ok": driven_done, "stage": "driven"},
        {"key": "rtktime", "label": "RTK 位置关联时间已标定（地图自洽）", "ok": rtk_time_done, "stage": "rtktime", "optional": True},
        {"key": "camtime", "label": "相机时间偏移已确认", "ok": camera_time_ok, "stage": "camtime",
         "value": offsets.get("camera")},
        {"key": "camlidar", "label": "相机偏航已复核（灯柱投影）", "ok": camera_ext_refined, "stage": "camlidar", "optional": True},
        {"key": "camloop", "label": "相机俯仰已闭环标定（原生里程尺度）", "ok": camera_loop_done, "stage": "camloop", "optional": True},
    ]
    if not active:
        next_step = {"stage": "import", "pipeline": None, "reason": "该车辆还没有标定版本，先导入出厂标定 JSON"}
        level = "none"
    elif not axle_done or not lidar_refined or not driven_done:
        next_step = {"stage": "axle" if not axle_done else "ground", "pipeline": "lidar",
                     "reason": "运行 LiDAR 标定流水线（后轴对齐 → 地面精修 → LIO 建图 → 行驶段精修）"}
        level = "lio"
    elif not camera_time_ok:
        next_step = {"stage": "camtime", "pipeline": "camera", "reason": "运行相机标定流水线（时间偏移 → LIVO 建图 → 灯柱偏航复核 → 俯仰闭环扫描 → 验证）"}
        level = "lio"
    elif not camera_loop_done:
        if camera_loop_failed:
            next_step = {"stage": "camloop", "pipeline": "camera",
                         "reason": "上次俯仰闭环未通过（原生尺度/后轴误差劣化）；建议标准模式重跑相机标定流水线重新测定俯仰斜率"}
            level = "livo"
        else:
            next_step = {"stage": "camloop", "pipeline": "camloop",
                         "reason": "可建图；建议运行“相机外参闭环微调”（俯仰扫描 ±0.05° → 尺度过零点 → LIVO 验证），消除 LIVO 原生轨迹的尺度漂移"}
            level = "livo"
    else:
        next_step = {"stage": None, "pipeline": None, "reason": "标定完整，可直接建图"}
        level = "livo"
    return {"mine": mine, "truck": truck, "id": key, "active": active_id, "active_version": active,
            "versions": len(versions), "latest": max((v.get("created") or "" for v in versions), default=""),
            "stages_done": stages_done, "checks": checks, "next": next_step, "ready_level": level,
            "time_offsets": offsets,
            "frontier_tip": work["id"] if work else None,
            "frontier_stale": bool(tip and active_id and tip["id"] != active_id),
            "calibration_file": str(CAL_ROOT.joinpath(*active_id.split("/")) / "calibration.json") if active_id else None}


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
    if isinstance(payload, dict):
        payload.setdefault("ok", rc == 0)
        payload.setdefault("errors", [])
        payload.setdefault("warnings", [])
        return payload
    return {"ok": rc == 0, "errors": [text.strip()[-300:]], "warnings": []}


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
    vid = body.get("parent_id") or body.get("id")
    if not vid:
        raise StudioError("缺少父版本 id")
    calibration_file(vid)
    return vid


# Stages whose tool command reads a saved-map fusion run (--run) and the mapping mode that run must have.
RUN_STAGES = {"driven": "lio", "gcons": "lio", "rtk-time": "lio", "camera-lidar": "livo", "camera-focus": "livo",
              "camera-photo": "livo"}


def _run_dir(raw):
    if not str(raw or "").strip():
        raise StudioError("没有选择建图结果（run）")
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
    if command in ("axle", "ground", "driven", "gcons", "camera-time", "camera-lidar", "camera-focus", "camera-photo"):
        dataset = str(safe_path(body.get("dataset") or ""))
        datasets.require(dataset, camera=command in ("camera-time", "camera-lidar", "camera-focus", "camera-photo"))
        cmd += ["--bag", dataset]
    if command in RUN_STAGES:
        # Every run-based stage gets its --run here; an empty selection (the UI lists no finished round of the
        # right mode for this dataset) must fail with a clear message, not with the tool's argparse usage text.
        if not str(body.get("run") or "").strip():
            kind = "LIVO" if RUN_STAGES[command] == "livo" else "LIO"
            hint = ("；RTK 位置关联时间在 LiDAR 流水线中自动完成，单步执行时轮次必须是用该父版本建图的"
                    if command == "rtk-time" else "")
            raise StudioError(f"阶段 {stages.get(stage, {}).get('label', command)} 需要选择一轮已保存地图、"
                              f"状态为完成的 {kind} 建图结果（run）{hint}")
        cmd += ["--run", str(_run_dir(body["run"]))]
    uw = _user_window(body)
    if command == "ground":
        if body.get("stationary"):
            cmd += ["--stationary", *[str(int(v)) for v in body["stationary"]]]
        if body.get("moving"):
            cmd += ["--moving", *[str(int(v)) for v in body["moving"]]]
    if command == "axle":
        if body.get("window"):
            cmd += ["--window", *[str(float(v)) for v in body["window"]]]
        elif uw is not None:
            # refine_rear_axle_extrinsic.py --window is [start, END] seconds after the first reference scan (not a
            # duration): a window starting at 100 s with 120 s used to solve on 100..120 s only
            end = uw["start_s"] + uw["duration_s"] if uw["duration_s"] is not None else 1e9
            cmd += ["--window", f"{uw['start_s']:.3f}", f"{end:.3f}"]
        if body.get("dof") in ("rpy_xy", "rpy", "xy"):
            cmd += ["--dof", body["dof"]]
        if body.get("keep_time_offset"):
            cmd.append("--keep-time-offset")
    if command == "driven":
        if body.get("window"):
            cmd += ["--window", *[str(float(v)) for v in body["window"]]]
        elif uw is not None:
            # driven --window is seconds after the round start; the round itself begins at round_start, which
            # in manual mode equals uw["start_s"], so the driven window starts at 0 in round-relative time.
            # When the user asked for "to end", play a long window so all native poses in the run are used.
            if uw["duration_s"] is None:
                d_end = 3600.0
            else:
                d_end = max(8.0, float(uw["duration_s"]))
            cmd += ["--window", "0.0", f"{d_end:.3f}"]
    if command == "camera-time":
        if body.get("duration") not in (None, "", 0, "0"):
            cmd += ["--duration", str(float(body["duration"]))]
        elif uw is not None and uw["duration_s"] is not None:
            cmd += ["--duration", f"{min(60.0, uw['duration_s']):.3f}"]
        if body.get("start") not in (None, ""):
            cmd += ["--start", str(float(body["start"]))]
        elif uw is not None:
            cmd += ["--start", f"{uw['start_s']:.3f}"]
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


def _user_window(body):
    """Translate window_start / window_duration from the Studio form to a playback window.

    Returns (start_offset, forced_lio_duration, axle_window, ground_override) or None when the user picked
    "auto" (i.e. let _windows_step / suggest_windows decide). When a custom window is provided, no
    motion-profile step is emitted and every round/stage runs inside that window:
      * axle: --window [start, duration] (seconds relative to bag start)
      * ground: stationary frames taken from [start, start+4 s); moving frames from a straight segment inside
        the selected window (when detectable) or a 4 s stretch after start+10 s
      * driven: --window [0, duration - start_round_offset] (seconds relative to round start, i.e.
        the script's own convention)
      * rounds (LIO/LIVO playback): start_offset + forced duration
    """
    ws = body.get("window_start")
    wd = body.get("window_duration")
    if ws in (None, "", 0) and wd in (None, "", 0):
        return None
    try:
        s = float(ws) if ws not in (None, "") else 0.0
        d = float(wd) if wd not in (None, "") else 0.0
    except (TypeError, ValueError):
        return None
    if s < 0:
        s = 0.0
    return {"start_s": s, "duration_s": d if d > 0 else None}


def _round_start(ctx, body):
    """Playback offset of every calibration round: explicit window > explicit start_offset > motion-profile."""
    uw = _user_window(body)
    if uw is not None:
        return float((ctx.get("win") or {}).get("round_start_s", uw["start_s"]))
    if body.get("start_offset") not in (None, ""):
        return float(body["start_offset"]) or None
    return float((ctx.get("win") or {}).get("round_start_s") or 0.0) or None


def _round_duration(body, default):
    uw = _user_window(body)
    if uw is not None:
        # manual window: explicit duration always wins; duration=None means "play to end" (duration=0
        # is how the player/mapping pipeline interprets "use the whole recording from start_offset").
        if uw["duration_s"] is None:
            return None
        return float(uw["duration_s"])
    d = body.get("_round_duration")
    if d is not None:
        d = float(d)
        return d if d else None
    return default


def _driven_window(ctx, body):
    """driven --window (start, end) in seconds after the round's first frame for a manual segment."""
    uw = _user_window(body)
    lead = float((ctx.get("win") or {}).get("lead_in_s", 0.0))
    end = 3600.0 if uw["duration_s"] is None else lead + max(8.0, float(uw["duration_s"]))
    return [lead, end]


def _play_duration(ctx, body, default):
    """Round playback length: in manual mode lead-in + segment (None = to the end), else `default`."""
    uw = _user_window(body)
    if uw is None:
        return default
    if uw["duration_s"] is None:
        return None
    return float(uw["duration_s"]) + float((ctx.get("win") or {}).get("lead_in_s", 0.0))


def _cam_round_duration(ctx, body, default):
    """Length of a camera-loop LIVO round: the configured length (default 60 s), but never past the end of a
    manual data window (window + lead-in to the last standstill). Before, a 30 s window still mapped 60 s and a
    window with cam_round_duration=0 mapped the whole recording."""
    uw = _user_window(body)
    if uw is None or uw["duration_s"] is None:
        return default
    limit = float(uw["duration_s"]) + float((ctx.get("win") or {}).get("lead_in_s", 0.0))
    return limit if default is None else min(float(default), limit)


def _windows_step(steps, dataset, round_duration, body):
    uw = _user_window(body)
    if uw is not None:
        # Manual window: the motion analysis runs inside the chosen segment. Calibration data stay in the
        # segment; a round whose segment starts mid-drive begins at the last standstill before it (lead-in)
        # so the mapper initialises at rest (livo_calib manual_window).
        s, d = uw["start_s"], uw["duration_s"]
        args = ["--start", f"{s:.3f}"] + (["--duration", f"{d:.3f}"] if d else [])
        steps.append({"title": f"手动标定窗口：{s:.0f} s 起 · {f'{d:.0f} s' if d else '到结尾'}（窗口内选静止/直线帧，建图从窗口前最近的静止处起播）",
                      "build": lambda ctx: [PY, str(CALIB_TOOL), "windows", "--bag", dataset, *args],
                      "after": lambda ctx, lines: ctx.__setitem__("win", _json_tail(lines))})
        return
    steps.append({"title": "运动分析：选择标定窗口（跳过起步前静止段，保留 8 s 静止用于 IMU 初始化）",
                  "build": lambda ctx: [PY, str(CALIB_TOOL), "windows", "--bag", dataset,
                                        "--round-duration", f"{float(round_duration or 60):.0f}"],
                  "after": lambda ctx, lines: ctx.__setitem__("win", _json_tail(lines))})


def _camera_key(vid):
    c = read_json(calibration_file(vid), {}) or {}
    offs = c.get("time_offsets") or {}
    return json.dumps([(c.get("transforms") or {}).get("midrange_camera_to_rear_axle", {}).get("transform_matrix"),
                       (c.get("camera") or {}).get("intrinsics"), (c.get("camera") or {}).get("distortion"),
                       offs.get("camera"), offs.get("lidar"), c.get("lidar")], sort_keys=True)


def _round_budget(round_name):
    """Optimized-trajectory budget verdict (PASS/FAIL) of a finished round, None when not evaluated."""
    if not round_name:
        return None
    doc = read_json(RESULTS / round_name / "evaluation" / "metrics.json", {}) or {}
    return ((doc.get("headline") or {}).get("optimized") or {}).get("budget")


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
              run (`pitch_slope`), i.e. only the intercept is measured. ~4 min, 2 rounds. A vehicle without a
              measured slope runs the standard scan (pipeline_steps); when the one-point fit is rejected (native
              scale off by > 2 %: outside the linear range) the two scan rounds of the standard mode run
              automatically instead of failing the pipeline.
    Every round starts at _round_start (motion-profile window) so standstill heads are not calibrated on.
    """
    round_duration = body.get("_cam_round_duration")
    if round_duration is not None:
        round_duration = float(round_duration) or None
    has_zero_round = bool(body.get("_has_zero_round"))
    fit_script = str(REPO / "scripts" / "analysis" / "camera_pitch_fit.py")
    rn0 = f"calib_{truck}_livo_{stamp}_base"
    if has_zero_round:
        # camera pipeline: round A was mapped with the camera-time version; it is the zero point unless the
        # camera-lidar stage actually changed the camera (then the zero point is re-mapped with the new version)
        def zero_build(ctx, rn=rn0):
            if _camera_key(ctx[base_key]) == _camera_key(ctx["camtime_id"]):
                return ["/bin/echo", f"camera-lidar left the camera unchanged: round {ctx[round0_key]} is the zero-pitch point"]
            return mapping_cmd(rn, "livo", str(calibration_file(ctx[base_key])), body,
                               duration=_cam_round_duration(ctx, body, round_duration),
                               start_offset=_round_start(ctx, body))

        def zero_after(ctx, lines, rn=rn0):
            if not any("is the zero-pitch point" in line for line in lines):
                ctx[round0_key] = rn
        steps.append({"title": f"零俯仰点：camera-lidar 未改动相机时复用轮次 A，否则建图 {rn0}", "retry": 1, "round": rn0,
                      "build": zero_build, "after": zero_after})
    else:
        # camloop pipeline: its base IS the parent, so in fast mode a finished round of the parent that played the
        # same window is a free zero point; otherwise it is mapped
        def base_build(ctx, rn=rn0):
            if not standard and body.get("reuse_round", True):
                r = reusable_round(body.get("_parent_id") or "",
                                   min_duration=float(body.get("min_reuse_s") or (round_duration and round_duration * 0.9) or 55),
                                   start_offset=_round_start(ctx, body) or 0.0)
                if r:
                    ctx["_reused_round"] = r["round"]
                    return ["/bin/echo", f"reuse round {r['round']} ({r['duration_s']:.0f} s, scale {r.get('scale')}) as the zero-pitch point"]
            return mapping_cmd(rn, "livo", str(calibration_file(ctx[base_key])), body,
                               duration=_cam_round_duration(ctx, body, round_duration),
                               start_offset=_round_start(ctx, body))
        steps.append({"title": f"LIVO 零俯仰建图 {rn0}（当前版本{'；快速模式优先复用同窗口的已有轮次' if not standard else ''}）",
                      "retry": 1, "round": rn0, "build": base_build,
                      "after": lambda ctx, lines, rn=rn0: ctx.__setitem__(round0_key, ctx.pop("_reused_round", None) or rn)})
    if not standard:
        slope = float(body.get("pitch_slope", DEFAULT_PITCH_SLOPE))
        steps.append({"title": f"用已知俯仰斜率 {slope:.4f}/° 求过零点（只测零点；被拒绝时自动转标准扫描）",
                      "build": lambda ctx: [PY, fit_script, "--point", f"{ctx[round0_key]}=0", "--slope", f"{slope:.4f}",
                                            "--max-abs", "0.15", "--exit-zero"],
                      "after": lambda ctx, lines: ctx.update(pitch_fit=_json_tail(lines),
                                                             pitch_delta_deg=_json_tail(lines).get("pitch_delta_deg"),
                                                             pitch_slope_source=body.get("pitch_slope_source"))})

    def scan_needed(ctx):
        if "scan_needed" not in ctx:
            ctx["scan_needed"] = standard or not (ctx.get("pitch_fit") or {}).get("ok")
            if not standard and ctx["scan_needed"]:
                ctx["fast_fallback"] = (ctx.get("pitch_fit") or {}).get("reason")
        return ctx["scan_needed"]

    skip = "（快速拟合通过时跳过）" if not standard else ""
    for sign, tag in ((+1, "up"), (-1, "dn")):
        dp = sign * SCAN_STEP_DEG
        steps.append({"title": f"俯仰扫描点 Δpitch {dp:+.2f}°（临时版本，不导出地图）{skip}", "build": lambda ctx, dp=dp: [
            PY, str(CALIB_TOOL), "set", "--id", ctx[base_key], "--camera-rot-delta", f"{dp:.3f}", "0", "0",
            "--stage", "scan", "--note", f"闭环俯仰扫描点 Δpitch={dp:+.2f}°（流水线临时版本，勿激活）"]
            if scan_needed(ctx) else ["/bin/echo", "one-point fit accepted: scan point skipped"],
            "after": lambda ctx, lines, tag=tag: ctx.__setitem__(f"scan_{tag}_id", parse_version_id(lines)) if scan_needed(ctx) else None})
        rn = f"calib_{truck}_livo_{stamp}_scan_{tag}"
        steps.append({"title": f"LIVO 扫描建图 {rn}（Δpitch {dp:+.2f}°，只评估轨迹）{skip}", "retry": 1, "round": rn,
                      "build": lambda ctx, rn=rn, tag=tag: mapping_cmd(
                          rn, "livo", str(calibration_file(ctx[f"scan_{tag}_id"])), body,
                          duration=_cam_round_duration(ctx, body, round_duration), save_map=False, quick=True,
                          start_offset=_round_start(ctx, body))
                      if scan_needed(ctx) else ["/bin/echo", "one-point fit accepted: scan round skipped"],
                      "after": lambda ctx, lines, rn=rn, tag=tag: ctx.__setitem__(f"scan_{tag}_round", rn) if scan_needed(ctx) else None})
    steps.append({"title": f"拟合原生里程尺度 vs 俯仰增量，求过零点{skip}", "build": lambda ctx: [
        PY, fit_script, "--point", f"{ctx[round0_key]}=0", "--point", f"{ctx['scan_up_round']}={SCAN_STEP_DEG}",
        "--point", f"{ctx['scan_dn_round']}={-SCAN_STEP_DEG}", "--exit-zero"] if scan_needed(ctx)
        else ["/bin/echo", "one-point fit accepted: standard fit skipped"],
        "after": lambda ctx, lines: ctx.update(pitch_fit=_json_tail(lines), pitch_delta_deg=_json_tail(lines).get("pitch_delta_deg"),
                                               pitch_slope_source=None) if scan_needed(ctx) else None})

    def pitch_mode(ctx):
        """adopt: new pitch to verify | keep: evidence that the current pitch is best (drift minimum at 0) |
        kept: pitch unobservable / fit rejected -> the base version is the result, nothing to verify"""
        fit = ctx.get("pitch_fit") or {}
        if not fit.get("ok"):
            return "kept"
        return "keep" if fit.get("keep") else "adopt"

    def set_cmd(ctx):
        fit = ctx["pitch_fit"]
        mode = pitch_mode(ctx)
        if mode == "kept":
            return ["/bin/echo", f"pitch fit rejected ({fit.get('reason')}): camera pitch of {ctx[base_key]} kept, "
                                 "no camloop version"]
        if fit.get("method") == "native drift minimum":
            note = (f"相机俯仰闭环：原生尺度被 LiDAR 锁定（斜率 {fit['slope_per_deg']:+.4f}/°，俯仰不可经尺度观测），改用原生漂移"
                    f"最小点 {fit.get('drift_vertex_deg', 0):+.4f}°（rmse {fit.get('drift_min_m', 0):.3f} m，{len(fit['points'])} 轮）"
                    + ("，保持当前俯仰" if mode == "keep" else f"，Δpitch={fit['pitch_delta_deg']:+.4f}°"))
            return [PY, str(CALIB_TOOL), "set", "--id", ctx[base_key], "--camera-rot-delta", f"{fit['pitch_delta_deg']:.4f}",
                    "0", "0", "--stage", "camloop", "--metrics-json", json.dumps({"pitch_fit": fit}, ensure_ascii=False),
                    "--note", note]
        note = (f"相机俯仰闭环：Δpitch={fit['pitch_delta_deg']:+.3f}°（尺度斜率 {fit['slope_per_deg']:.3f}/°"
                f"{'，沿用历史值 ' + str(ctx.get('pitch_slope_source') or '模型') if fit.get('slope_source') else ''}"
                f"，{len(fit['points'])} 轮，预测原生尺度 {fit['predicted_ratio']:.4f}）")
        return [PY, str(CALIB_TOOL), "set", "--id", ctx[base_key], "--camera-rot-delta", f"{fit['pitch_delta_deg']:.4f}", "0", "0",
                "--stage", "camloop", "--metrics-json", json.dumps({"pitch_fit": fit}, ensure_ascii=False), "--note", note]
    def set_after(ctx, lines):
        vid = ctx[base_key] if pitch_mode(ctx) == "kept" else parse_version_id(lines)
        ctx.update(camloop_id=vid, result_id=vid)
    steps.append({"title": "生成闭环标定版本 (camloop；俯仰不可观测/拟合被拒绝时保持当前版本继续)", "build": set_cmd,
                  "after": set_after})
    rv = f"calib_{truck}_livo_{stamp}_verify"
    steps.append({"title": f"LIVO 验证建图 {rv}（闭环版本；俯仰保持不变时零俯仰轮次即验证，跳过）", "retry": 1, "round": rv,
                  "build": lambda ctx: mapping_cmd(rv, "livo", str(calibration_file(ctx["camloop_id"])), body,
                                                   duration=_cam_round_duration(ctx, body, round_duration),
                                                   start_offset=_round_start(ctx, body))
                  if pitch_mode(ctx) == "adopt" else
                  ["/bin/echo", f"camera pitch unchanged: round {ctx[round0_key]} already maps this extrinsic, verify skipped"],
                  "after": lambda ctx, lines: ctx.__setitem__("verify_round", rv if pitch_mode(ctx) == "adopt" else ctx[round0_key])})
    steps.append({"title": "闭环判定：基线轮次 vs 闭环版本（原生里程尺度 / 后轴误差 / 偏航）", "build": lambda ctx: [
        PY, str(REPO / "scripts" / "analysis" / "closed_loop_verdict.py"), ctx[baseline_key], ctx["verify_round"],
        "--version-dir", str(CAL_ROOT.joinpath(*ctx["camloop_id"].split("/"))),
        "--scale-margin", "0.002" if ctx.get("scan_needed", standard) else "0.0005", "--exit-zero"]
        if pitch_mode(ctx) == "adopt" else
        ["/bin/echo", {"keep": "native drift is minimal at the current pitch: closed loop pass (pitch kept)",
                       "kept": "camera pitch unobservable on this data: kept (base version is the result)"}[pitch_mode(ctx)]],
        "after": lambda ctx, lines: ctx.update(closed_loop=_verdict(lines) if pitch_mode(ctx) == "adopt"
                                               else ("pass" if pitch_mode(ctx) == "keep" else "kept"))})
    steps.append({"title": "校验新版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["camloop_id"], "--strict"]})

    def activate_ok(ctx):
        if not auto_activate:
            return False
        if ctx.get("closed_loop") == "pass":
            return True
        # pitch unobservable: the base version is still the best available one when its own round met the budget
        return ctx.get("closed_loop") == "kept" and _round_budget(ctx.get(round0_key)) == "PASS"

    def activate_cmd(ctx):
        if activate_ok(ctx):
            return [PY, str(CALIB_TOOL), "activate", "--id", ctx["camloop_id"]]
        why = ("未勾选自动激活" if not auto_activate else
               "俯仰不可观测且零俯仰轮次未达预算" if ctx.get("closed_loop") == "kept" else "闭环未通过")
        return [PY, "-c", f"print('not activated: {why}; active version unchanged')"]
    steps.append({"title": "激活闭环版本（闭环通过，或俯仰不可观测但零俯仰轮次达预算；需勾选自动激活）", "build": activate_cmd,
                  "after": lambda ctx, lines: ctx.__setitem__("activated", activate_ok(ctx))})


def _rtk_round_start(ctx, body, rtk_dur):
    """Start of the RTK time-association rounds. The offset is only observable through accelerations, so the
    round should be as long as allowed: among the motion-window candidates take the one with the most
    recording left (up to the round length), ties broken by the driving score. Explicit windows win."""
    if _user_window(body) is not None or body.get("start_offset") not in (None, ""):
        return _round_start(ctx, body)
    win = ctx.get("win") or {}
    total = float(win.get("duration_s") or 0.0)
    cands = [c for c in win.get("candidates") or [] if c.get("round_start_s") is not None]
    if not cands or total <= 0:
        return _round_start(ctx, body)
    cap = rtk_dur or total
    best = max(cands, key=lambda c: (round(min(cap, total - float(c["round_start_s"]))), float(c.get("score") or 0.0)))
    return float(best["round_start_s"]) or None


def _rtk_time_steps(steps, body, mapping_cmd, *, truck, stamp, auto_activate):
    """RTK position time association, fully automatic: LIO round with the driven version -> map-consistency
    estimate -> (only when observable and not already consistent) probe version + LIO round at the predicted
    optimum -> secant of both corrections -> rtktime version. Unobservable data keep the driven version."""
    rtk_dur = body.get("rtk_round_duration")
    rtk_dur = RTK_ROUND_S if rtk_dur in (None, "") else (float(rtk_dur) or None)
    r1, r2 = f"calib_{truck}_rtk1_{stamp}", f"calib_{truck}_rtk2_{stamp}"
    probing = lambda ctx: (ctx.get("rtk1") or {}).get("decision") == "probe"
    uw = _user_window(body)
    span = (f"手动窗口 {uw['duration_s']:.0f} s" if uw["duration_s"] else "手动窗口到结尾") if uw is not None else \
        f"{rtk_dur or 0:.0f} s，起点取剩余时长最长的运动窗口"
    steps.append({"title": f"LIO 建图轮次 {r1}（RTK 关联时间估计，{span}）", "retry": 1, "round": r1,
                  "build": lambda ctx: mapping_cmd(r1, "lio", str(calibration_file(ctx["driven_id"])), body,
                                                   duration=_play_duration(ctx, body, rtk_dur), start_offset=_rtk_round_start(ctx, body, rtk_dur), quick=True),
                  "after": lambda ctx, lines: ctx.update(rtk_round1=r1, rtk_fusion1=latest_fusion_dir(r1))})
    steps.append({"title": "RTK 关联时间：地图自洽估计（不可观测或已自洽则沿用行驶段版本）",
                  "build": lambda ctx: [PY, str(CALIB_TOOL), "rtk-time", "--id", ctx["driven_id"], "--run", ctx["rtk_fusion1"]],
                  "after": lambda ctx, lines: ctx.__setitem__("rtk1", _json_tail(lines))})
    steps.append({"title": "RTK 关联时间：探测版本（预测最优值，临时）",
                  "build": lambda ctx: [PY, str(CALIB_TOOL), "set", "--id", ctx["driven_id"], "--stage", "scan",
                                        "--rtk-position-offset", f"{ctx['rtk1']['probe_rtk_position_s']:.4f}",
                                        "--note", "rtk-time probe point (temporary)"]
                  if probing(ctx) else ["/bin/echo", f"no probe needed: {(ctx.get('rtk1') or {}).get('reason', '')}"],
                  "after": lambda ctx, lines: ctx.__setitem__("rtk_probe_id", parse_version_id(lines) if probing(ctx) else None)})
    steps.append({"title": f"LIO 建图轮次 {r2}（探测值，同一窗口）", "retry": 1, "round": r2,
                  "build": lambda ctx: mapping_cmd(r2, "lio", str(calibration_file(ctx["rtk_probe_id"])), body,
                                                   duration=_play_duration(ctx, body, rtk_dur), start_offset=_rtk_round_start(ctx, body, rtk_dur), quick=True)
                  if probing(ctx) else ["/bin/echo", "probe round skipped"],
                  "after": lambda ctx, lines: ctx.update(rtk_fusion2=latest_fusion_dir(r2) if probing(ctx) else None)})
    steps.append({"title": "RTK 关联时间：两轮割线定版（rtktime 版本）",
                  "build": lambda ctx: [PY, str(CALIB_TOOL), "rtk-time", "--id", ctx["driven_id"], "--run", ctx["rtk_fusion1"],
                                        "--run2", ctx["rtk_fusion2"]]
                  if probing(ctx) else ["/bin/echo", f"rtk_position kept: {(ctx.get('rtk1') or {}).get('reason', '')}"],
                  "after": lambda ctx, lines: ctx.__setitem__("result_id", parse_version_id(lines) if probing(ctx) else ctx["driven_id"])})
    steps.append({"title": "校验最终版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["result_id"]]})


def _camphoto_steps(steps, body, mapping_cmd, *, truck, stamp, auto_activate, parent):
    """Camera photometric calibration starting from `parent` (the active version), appended to `steps`:
    LIVO round with the parent (the data of the stage and the A side of the verdict) -> camera-photo (camera rotation
    + time offset from the multi-view photo-consistency of the round's returns; intrinsics checked, changed only on a
    significant held-out gain) -> LIVO verify round with the new version -> closed-loop verdict on the native front
    end (odometry scale, rear-axle error, yaw: the camera also drives the LIVO photometric update) -> validate
    [-> activate]. No significant / consistent correction: the parent is the result, verify and verdict skipped."""
    round_duration = body.get("_cam_round_duration")
    if round_duration is not None:
        round_duration = float(round_duration) or None
    rb, rv = f"calib_{truck}_livo_{stamp}_photo", f"calib_{truck}_livo_{stamp}_photo_verify"
    changed = lambda ctx: ctx.get("photo_id") not in (None, parent)  # noqa: E731
    steps.append({"title": f"LIVO 建图 {rb}（当前版本：光度标定的数据，也是判定的基线）", "retry": 1, "round": rb,
                  "build": lambda ctx: mapping_cmd(rb, "livo", str(calibration_file(parent)), body,
                                                   duration=_cam_round_duration(ctx, body, round_duration),
                                                   start_offset=_round_start(ctx, body)),
                  "after": lambda ctx, lines: ctx.update(photo_round=rb, photo_fusion=latest_fusion_dir(rb))})
    steps.append({"title": "相机光度标定 (camera-photo：多视角光度一致性 → 相机旋转 + 时间偏移、暗角平场，内参复核；无显著改善则沿用父版本)",
                  "build": lambda ctx: stage_cmd("camera-photo", {"id": parent, "dataset": body.get("dataset"),
                                                                    "run": ctx["photo_fusion"], "note": body.get("note") or ""}),
                  "after": lambda ctx, lines: ctx.__setitem__("photo_id", parse_version_id(lines))})
    steps.append({"title": f"LIVO 验证建图 {rv}（光度标定版本；沿用父版本时跳过）", "retry": 1, "round": rv,
                  "build": lambda ctx: mapping_cmd(rv, "livo", str(calibration_file(ctx["photo_id"])), body,
                                                   duration=_cam_round_duration(ctx, body, round_duration),
                                                   start_offset=_round_start(ctx, body))
                  if changed(ctx) else ["/bin/echo", "camera kept: verify round skipped"],
                  "after": lambda ctx, lines: ctx.__setitem__("verify_round", rv if changed(ctx) else None)})
    steps.append({"title": "判定：基线轮次 vs 光度标定版本（原生里程尺度 / 后轴误差 / 偏航不变差）",
                  "build": lambda ctx: [PY, str(REPO / "scripts" / "analysis" / "closed_loop_verdict.py"), ctx["photo_round"],
                                        ctx["verify_round"], "--version-dir", str(CAL_ROOT.joinpath(*ctx["photo_id"].split("/"))),
                                        "--scale-margin", "0.002", "--exit-zero"]
                  if changed(ctx) else ["/bin/echo", "camera kept: no verdict needed"],
                  "after": lambda ctx, lines: ctx.update(closed_loop=_verdict(lines) if changed(ctx) else "kept",
                                                         result_id=ctx["photo_id"] if changed(ctx) else parent)})
    steps.append({"title": "校验结果版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["result_id"], "--strict"]})

    def activate_cmd(ctx):
        if auto_activate and changed(ctx) and ctx.get("closed_loop") == "pass":
            return [PY, str(CALIB_TOOL), "activate", "--id", ctx["photo_id"]]
        why = ("未勾选自动激活" if not auto_activate else "相机沿用父版本" if not changed(ctx) else "判定未通过")
        return [PY, "-c", f"print('not activated: {why}; active version unchanged')"]
    steps.append({"title": "激活光度标定版本（判定通过且勾选自动激活）", "build": activate_cmd,
                  "after": lambda ctx, lines: ctx.__setitem__("activated", auto_activate and changed(ctx)
                                                              and ctx.get("closed_loop") == "pass")})


RESUME_PREFIX = "续跑："


def _resume_steps(body, mapping_cmd):
    """Continue a failed / stopped / lost pipeline from its first unfinished step.

    The step list is rebuilt from the original request with the same round-name stamp (current code, so a fix
    made after the failure is used), the finished steps are replaced by no-ops and the original job context
    (version ids, round directories, estimates) is restored first. Refused when the rebuilt step list no longer
    matches the original one (pipeline structure changed in between)."""
    import copy
    from . import jobs  # never hot-reloaded: holds the live and stored job records
    src = jobs.get(str(body["resume"]))
    if src is None:
        raise StudioError(f"找不到要续跑的任务 {body['resume']}")
    doc = src.to_json()
    if doc.get("kind") != "pipeline" or doc.get("status") not in ("failed", "stopped", "lost"):
        raise StudioError("只有失败、终止或中断的流水线任务可以续跑")
    meta = doc.get("meta") or {}
    opts = {k: v for k, v in (meta.get("options") or {}).items() if k != "resume"}
    # the data assessment step only exists when no cached report was present; it is done either way by now
    old = [st for st in doc.get("steps") or [] if not st["title"].startswith((RESUME_PREFIX, "数据评估"))]
    opts["skip_assess"] = True
    stamp = next((m.group(1) for st in old for m in [re.search(r"_(\d{4}_\d{6})", st["title"])] if m), None)
    if stamp is None:
        raise StudioError("原任务中找不到轮次时间戳，无法续跑")
    opts["_stamp"] = stamp
    steps, info = pipeline_steps(meta.get("kind") or opts.get("kind") or "lidar", opts, mapping_cmd)
    done = 0
    while done < len(old) and old[done]["status"] == "done":
        done += 1
    # only the finished prefix has to be the same pipeline (its results are reused); the remaining steps come
    # from the current code on purpose, so a fix made after the failure is picked up
    for a, b in zip(steps[:done], old[:done]):
        if a["title"] != b["title"]:
            raise StudioError(f"已完成的步骤与当前流水线不一致（原：{b['title']}；现：{a['title']}），请重新运行完整流水线")
    if done >= len(steps):
        raise StudioError("原任务的所有步骤都已完成")
    saved = copy.deepcopy(doc.get("results") or {})
    out = [{"title": f"{RESUME_PREFIX}恢复任务 {doc['id']} 的结果，从第 {done + 1} 步继续",
            "cmd": ["/bin/echo", f"resume {doc['id']} at step {done + 1}: {old[done]['title']}"],
            "after": lambda ctx, lines: ctx.update(copy.deepcopy(saved))}]
    out += [{"title": st["title"], "cmd": ["/bin/echo", f"done in job {doc['id']}"]} for st in steps[:done]]
    first = dict(steps[done])
    if first.get("round") and first.get("build"):
        # a half-written round directory of the failed attempt is kept for diagnosis, as the retry logic does
        def build(ctx, orig=first["build"], rn=first["round"]):
            d = RESULTS / rn
            if d.is_dir():
                k = 1
                while (RESULTS / f"{rn}.failed{k}").exists():
                    k += 1
                d.rename(RESULTS / f"{rn}.failed{k}")
            return orig(ctx)
        first["build"] = build
    out += [first] + steps[done + 1:]
    return out, {**info, "resumed_from": doc["id"], "resumed_at_step": done + 1}


_CLEANUP_CODE = """import pathlib, sys
root, pattern = pathlib.Path(sys.argv[1]), sys.argv[2]
total = count = 0
for run in sorted(root.glob(pattern)):
    for f in run.glob("Log/fusion/*/*"):
        if f.name in ("scans.bin", "sources.bin", "colors.bin") or (
                f.suffix == ".pcd" and f.name.startswith(("map_", "rgb_map_"))):
            total += f.stat().st_size; count += 1; f.unlink()
print("removed %d files, %.2f GB from %s (keep_calib_maps=false)" % (count, total / 1e9, pattern))
"""


def _cleanup_step(steps, body, truck, stamp):
    """Calibration rounds archive every scan (scans.bin/colors.bin/sources.bin) and export map_N/rgb_map_N.pcd
    because the later stages of the same pipeline read them (driven: frames.csv, rtk-time / camera-lidar:
    scans.bin). Once the pipeline succeeded they are dead weight (truck29: ~1.7 GB per LIO round, truck28 ~3.5 GB):
    removed unless keep_calib_maps. Trajectories, frames.csv, diagnostics and evaluation stay. A failed pipeline
    keeps everything (resume reuses the finished rounds)."""
    if body.get("keep_calib_maps"):
        return
    steps.append({"title": "清理标定轮次的地图 PCD 与扫描存档（默认不保留；轨迹/评估保留）",
                  "cmd": [PY, "-c", _CLEANUP_CODE, str(RESULTS), f"calib_{truck}_*_{stamp}*"]})


def pipeline_steps(kind, body, mapping_cmd):
    """Standard pipelines. `mapping_cmd(round_name, mode, calibration_file, body)` builds the round command.

    lidar  : axle (reference LiDAR -> rear axle from RTK/INS, rig correction; skip_axle=true skips it)
             -> ground -> LIO round (save_map, evaluated) -> driven -> validate -> RTK time association (LIO
             rounds) -> gcons (rear-facing LiDAR tilt/z vs the all-LiDAR ground; skip_gcons=true skips it)
             -> validate [-> activate]
    camera : camera-time -> LIVO round A -> camera-lidar (pole yaw, pitch untouched)
             -> pitch scan (2 temporary versions, 2 LIVO rounds) -> zero crossing -> camloop version
             -> LIVO verify round -> closed-loop verdict A vs verify -> validate [-> activate if pass]
    camloop: same pitch closed loop starting from the parent version (its own LIVO round is the 0 point)
    camphoto: LIVO round with the parent -> camera-photo (camera rotation + time offset from the multi-view
             photo-consistency, intrinsics check) -> LIVO verify round -> closed-loop verdict -> validate [-> activate]
    The native odometry scale reacts with ~11 %/deg of camera pitch error and is repeatable to ~0.03 %,
    so it is the acceptance test; edge/pole alignment alone cannot resolve pitch (36-line ring spacing).
    """
    if body.get("resume"):
        return _resume_steps(body, mapping_cmd)
    parent = requested_parent = _parent(body)
    # Any version of the current vehicle can be the parent of an automated pipeline.  The
    # legacy `from_factory` flag is intentionally ignored: the operator now chooses the parent
    # explicitly in Studio, including the factory import when a fresh start is desired.
    dataset = str(safe_path(body.get("dataset") or ""))
    mine, truck = parent.split("/")[:2]
    standard = (body.get("mode") or "standard") != "fast"
    slope_info = pitch_slope(mine, truck)
    mode_note = None
    if not standard and slope_info["source"] is None and kind in ("camera", "camloop"):
        # the slope depends on the camera geometry (mount pitch/height, focal length): another truck's value is no
        # prior for this one, so the first closed loop of a vehicle measures it with the standard scan
        standard, mode_note = True, "本车尚无实测俯仰斜率（其他车的斜率不适用），已自动改用标准扫描"
    cam_dur = body.get("cam_round_duration")
    cam_dur = 60.0 if cam_dur in (None, "") else (float(cam_dur) or None)  # default: 60 s rounds; 0 = full
    body = {**body, "_parent_id": parent, "pitch_slope": slope_info["slope"], "pitch_slope_source": slope_info["source"],
            "_cam_round_duration": cam_dur, "_has_zero_round": kind == "camera"}
    stamp = body.get("_stamp") or dt.datetime.now().strftime("%m%d_%H%M%S")  # _stamp: resume keeps round names
    auto_activate = bool(body.get("activate", True))
    steps = []
    source_id = body.get("body_filter_source_id") or active_id(mine, truck) or parent
    if not str(source_id).startswith(f"{mine}/{truck}/"):
        source_id = active_id(mine, truck) or parent
    try:
        source_file = str(calibration_file(source_id))
    except Exception:  # noqa: BLE001 - fall back to the selected parent's own body_filter
        source_id, source_file = parent, str(calibration_file(parent))
    body = {**body, "_body_filter_source_id": source_id}
    steps.append({
        "title": "前置：将当前车辆 body_filter 写入父版本标定文件",
        "cmd": [PY, "-c", BODY_FILTER_PREP_CODE, str(calibration_file(parent)), source_file,
                str(REPO / "tools")],
        "after": lambda ctx, lines: ctx.__setitem__("body_filter_source_id", source_id),
    })
    if body.get("dataset") and not body.get("skip_assess"):
        # data assessment first (cached per recording): warns about defects calibration cannot compensate and stops
        # the pipeline before any expensive stage when the recording is unusable
        try:
            base_file = str(calibration_file(parent))
        except Exception:  # noqa: BLE001 - parent without a file: assess without the extrinsic checks
            base_file = None
        step0 = datasets.assess_pipeline_step(
            dataset, base_file,
            window_start=body.get("window_start"), window_duration=body.get("window_duration"))
        if step0:
            steps.append(step0)
    if kind == "lidar":
        datasets.require(dataset)
        lio_duration = body.get("lidar_round_duration")
        lio_duration = 48.0 if lio_duration in (None, "") else float(lio_duration)
        lio_duration = _round_duration(body, lio_duration)
        _windows_step(steps, dataset, lio_duration or 60, body)
        if body.get("skip_axle"):
            steps.append({"title": "跳过后轴对齐（沿用父版本的参考雷达外参）", "build": lambda ctx: ["/bin/echo", "axle skipped"],
                          "after": lambda ctx, lines: ctx.__setitem__("axle_id", parent)})
        else:
            def _axle_body(ctx):
                return {"id": parent, "dataset": dataset, "note": body.get("note") or "pipeline: axle",
                        "window_start": body.get("window_start"), "window_duration": body.get("window_duration")}
            steps.append({"title": "后轴对齐 (axle：参考雷达→后轴，RTK/INS 位姿，整机刚体修正 + LiDAR 时间偏移)",
                          "build": lambda ctx: stage_cmd("axle", _axle_body(ctx)),
                          "after": lambda ctx, lines: ctx.__setitem__("axle_id", parse_version_id(lines))})
        def _ground_body(ctx):
            gb = {"id": ctx["axle_id"], "dataset": dataset, "note": body.get("note") or "pipeline: ground",
                  "stationary": ctx["win"]["ground_stationary_frames"],
                  "moving": ctx["win"]["ground_moving_frames"]}
            # In manual-window mode ground works entirely on frame indices derived from start; no --window needed.
            return gb
        steps.append({"title": "静止地面精修 (ground：静止帧取窗口前段，运动帧取窗口内直线段)",
                      "build": lambda ctx: stage_cmd("ground", _ground_body(ctx)),
            "after": lambda ctx, lines: ctx.__setitem__("ground_id", parse_version_id(lines))})
        round_name = f"calib_{truck}_lio_{stamp}"
        # The driven stage only reads native poses inside its window (default 8-40 s after the round start), so
        # the LIO round does not need the whole recording: cap playback at window end + drain margin. The round
        # starts ROUND_LEAD_S before the first drive (motion window), so 8-40 s is driving, not a standstill.
        # lidar_round_duration=0 restores the previous full-length behaviour.
        title_extra = (f"，{lio_duration:.0f} s（{'手动窗口' if _user_window(body) is not None else '起点按运动窗口'}）"
                       if lio_duration else "")
        steps.append({"title": f"LIO 建图轮次 {round_name}（用于行驶段精修{title_extra}）", "retry": 1, "round": round_name, "build": lambda ctx: mapping_cmd(
            round_name, "lio", str(calibration_file(ctx["ground_id"])), body, duration=_play_duration(ctx, body, lio_duration or None),
            start_offset=_round_start(ctx, body)),
            "after": lambda ctx, lines: ctx.update(round=round_name, fusion_dir=latest_fusion_dir(round_name))})
        def _driven_body(ctx):
            return {"id": ctx["ground_id"], "dataset": dataset, "run": ctx["fusion_dir"],
                    "note": body.get("note") or "pipeline: driven", "activate": False,
                    "window_start": body.get("window_start"), "window_duration": body.get("window_duration"),
                    **({"window": _driven_window(ctx, body)} if _user_window(body) is not None else {})}
        steps.append({"title": "行驶段 6-DoF 精修 (driven)", "build": lambda ctx: stage_cmd("driven", _driven_body(ctx)),
            "after": lambda ctx, lines: ctx.__setitem__("driven_id", parse_version_id(lines))})
        steps.append({"title": "校验新版本", "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["driven_id"]],
                      "after": lambda ctx, lines: ctx.__setitem__("result_id", ctx["driven_id"])})
        if not body.get("skip_rtk_time"):
            _rtk_time_steps(steps, body, mapping_cmd, truck=truck, stamp=stamp, auto_activate=auto_activate)
            if not body.get("skip_gcons"):
                # The RTK time-association round already maps the whole window with the driven extrinsics and
                # RTK-optimized poses: the rear-facing LiDAR is then tied to the ground seen by ALL LiDARs (the
                # driven stage could only use the front LiDAR's far, cross-time ground). No significant offset ->
                # the stage keeps the current version (it prints the parent id).
                steps.append({"title": "后向雷达地面一致性 (gcons：全部雷达共识地面 → 后向雷达 tilt/z，无显著偏差则沿用)",
                              "build": lambda ctx: [PY, str(CALIB_TOOL), "gcons", "--id", ctx["result_id"], "--bag", dataset,
                                                    "--run", ctx["rtk_fusion1"], "--note", body.get("note") or "pipeline: gcons"],
                              "after": lambda ctx, lines: ctx.__setitem__("result_id", parse_version_id(lines))})
                steps.append({"title": "校验最终版本（gcons 之后）",
                              "build": lambda ctx: [PY, str(CALIB_TOOL), "validate", "--id", ctx["result_id"]]})
        if auto_activate:
            # intermediate versions are never activated: a pipeline that stops half-way leaves the vehicle on its
            # previous calibration; only the final version, after validation, is promoted
            steps.append({"title": "激活最终版本（仅在全部步骤成功后）",
                          "build": lambda ctx: [PY, str(CALIB_TOOL), "activate", "--id", ctx["result_id"]],
                          "after": lambda ctx, lines: ctx.__setitem__("activated", ctx["result_id"])})
    elif kind == "camera":
        datasets.require(dataset, camera=True)
        _windows_step(steps, dataset, cam_dur or 60, body)
        def _camtime_body(ctx):
            b = {"id": parent, "dataset": dataset, "note": body.get("note") or "pipeline: camera-time"}
            if _user_window(body) is not None:
                # stage_cmd() will inject --start/--duration from the window fields
                b.update({"window_start": body.get("window_start"), "window_duration": body.get("window_duration")})
                if body.get("camtime_duration") not in (None, ""):
                    b["duration"] = float(body["camtime_duration"])
            else:
                b["duration"] = body.get("camtime_duration") if body.get("camtime_duration") not in (None, "") else 60
                b["start"] = ctx["win"].get("camera_time_start_s", 0.0)
            return b
        steps.append({"title": "相机时间偏移 (camera-time，行驶段 60 s)", "build": lambda ctx: stage_cmd("camera-time", _camtime_body(ctx)),
            "after": lambda ctx, lines: ctx.__setitem__("camtime_id", parse_version_id(lines))})
        round_name = f"calib_{truck}_livo_{stamp}"
        steps.append({"title": f"LIVO 建图轮次 A {round_name}（基线，俯仰增量 0）", "retry": 1, "round": round_name, "build": lambda ctx: mapping_cmd(
            round_name, "livo", str(calibration_file(ctx["camtime_id"])), body,
            duration=_cam_round_duration(ctx, body, cam_dur), start_offset=_round_start(ctx, body)),
            "after": lambda ctx, lines: ctx.update(round=round_name, fusion_dir=latest_fusion_dir(round_name))})
        steps.append({"title": "相机-LiDAR 偏航复核 (camera-lidar：灯柱投影列偏差，俯仰不动)", "build": lambda ctx: stage_cmd(
            "camera-lidar", {"id": ctx["camtime_id"], "dataset": dataset, "run": ctx["fusion_dir"],
                             "frames": body.get("frames") or 30, "note": body.get("note") or ""}),
            "after": lambda ctx, lines: ctx.update(camlidar_id=parse_version_id(lines), result_id=ctx["camtime_id"])})
        _camloop_steps(steps, body, mapping_cmd, truck=truck, stamp=stamp, auto_activate=auto_activate,
                       base_key="camlidar_id", round0_key="round", baseline_key="round", standard=standard)
    elif kind == "camphoto":
        datasets.require(dataset, camera=True)
        cam_dur_eff = _round_duration(body, cam_dur or 60)
        _windows_step(steps, dataset, cam_dur_eff or 60, body)
        _camphoto_steps(steps, body, mapping_cmd, truck=truck, stamp=stamp, auto_activate=auto_activate, parent=parent)
    elif kind == "camloop":
        datasets.require(dataset, camera=True)
        cam_dur_eff = _round_duration(body, cam_dur or 60)
        _windows_step(steps, dataset, cam_dur_eff or 60, body)
        steps.append({"title": "当前版本作为待微调版本", "build": lambda ctx: ["/bin/echo", "base version prepared"],
                      "after": lambda ctx, lines: ctx.__setitem__("base_id", parent)})
        _camloop_steps(steps, body, mapping_cmd, truck=truck, stamp=stamp, auto_activate=auto_activate,
                       base_key="base_id", round0_key="round", baseline_key="round", standard=standard)
    else:
        raise StudioError(f"未知流水线 {kind}")
    _cleanup_step(steps, body, truck, stamp)
    return steps, {"mine": mine, "truck": truck, "parent": parent, "requested_parent": requested_parent,
                   "body_filter_source_id": source_id, "dataset": dataset,
                   "mode": "standard" if standard else "fast", "mode_note": mode_note,
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
        if fit.get("slope_source") or len(fit.get("points") or []) < 3:
            continue  # a fast run records the slope it ASSUMED: only a standard scan measures one
        if isinstance(b, (int, float)) and 0.03 <= b <= 0.4:
            return {"slope": float(b), "source": f"{mine}/{truck}/{d.name}"}
    return {"slope": DEFAULT_PITCH_SLOPE, "source": None}


def reusable_round(calibration_id, min_duration=110.0, start_offset=None):
    """The newest finished LIVO round that already ran with this calibration: a free zero-pitch point for the
    closed loop (same version, full length), so the fast mode only has to map the verify round. With
    start_offset given, only a round that played the same window qualifies (native scale is window dependent)."""
    from . import mapping as mapping_module
    target = str(calibration_file(calibration_id))
    for r in mapping_module.list_runs(limit=40):
        if r.get("status") != "done" or r.get("mode") != "livo":
            continue
        if str(r.get("calibration") or "") != target:
            continue
        if start_offset is not None:
            man = read_json(pathlib.Path(r["path"]) / "round.json", {}) or {}
            if abs(float(man.get("start_offset") or 0.0) - float(start_offset)) > 0.5:
                continue
        doc = read_json(pathlib.Path(r["path"]) / "evaluation" / "metrics.json", {}) or {}
        run = doc.get("run") or {}
        dur = float(run.get("duration_s") or 0)
        if run.get("frames") and dur >= min_duration:
            return {"round": r["round"], "duration_s": dur, "start": r.get("start"),
                    "scale": (doc.get("headline", {}).get("native") or {}).get("path_length_ratio")}
    return None


