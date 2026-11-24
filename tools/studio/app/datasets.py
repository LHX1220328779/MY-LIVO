"""Recordings: inspection of raw ROS 2 recording roots / single bag folders via tools/livo/dataset.py
(no conversion, no prepared bag) plus a small file browser."""
import pathlib
import subprocess
import threading

from .common import (DATASET_TOOL, PY, REPO, STUDIO_DIR, TOOLS, StudioError, load_state,
                     now, read_json, safe_path, update_state)

ASSESS_TOOL = TOOLS / "livo" / "assess.py"
ASSESS_DIR = STUDIO_DIR / "assess"

_cache = {}
_cache_lock = threading.Lock()


def _signature(path):
    """Cheap change detector: metadata.yaml mtimes up to two levels down."""
    sig = []
    try:
        for cand in [path / "metadata.yaml", *path.glob("*/metadata.yaml"), *path.glob("*/*/metadata.yaml")]:
            try:
                sig.append((str(cand), cand.stat().st_mtime_ns))
            except OSError:
                continue
    except OSError:
        pass
    return tuple(sorted(sig))


def describe(raw_path, refresh=False, timeout=180):
    """Summary dict of a recording (kind, segments, topics, duration, lio_ready/livo_ready...)."""
    try:
        path = safe_path(raw_path)
    except PermissionError as exc:
        return {"path": str(raw_path), "error": str(exc)}
    if not path.is_dir():
        return {"path": str(path), "error": "目录不存在"}
    sig = _signature(path)
    if not sig:
        return {"path": str(path), "error": "该目录下没有 ROS 2 bag（找不到 metadata.yaml）"}
    with _cache_lock:
        hit = _cache.get(str(path))
        if hit and hit["sig"] == sig and not refresh:
            return hit["info"]
    try:
        proc = subprocess.run([PY, str(DATASET_TOOL), str(path), "--json"], cwd=REPO, capture_output=True,
                              text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"path": str(path), "error": "读取超时"}
    if proc.returncode != 0:
        return {"path": str(path), "error": (proc.stderr or proc.stdout).strip()[-600:] or "读取失败"}
    import json
    try:
        info = json.loads(proc.stdout)
    except json.JSONDecodeError:
        return {"path": str(path), "error": "dataset.py 输出无法解析"}
    info["name"] = path.name
    info["inspected"] = now()
    with _cache_lock:
        _cache[str(path)] = {"sig": sig, "info": info}
    return info


def require(raw_path, camera=False):
    """Describe a recording and raise a user-facing error when it cannot serve the requested mode."""
    info = describe(raw_path)
    if info.get("error"):
        raise StudioError(f"数据源不可用：{info['error']}")
    if not info.get("lio_ready"):
        raise StudioError("数据源缺少 LiDAR/IMU 话题，无法用于建图或标定")
    if camera:
        name = "midrange" if camera is True else camera
        if name not in ("midrange", "fisheye"):
            raise StudioError("相机必须选择中距或鱼眼")
        topic = "fish_camera/ffmpeg" if name == "fisheye" else "midrange_camera/ffmpeg"
        topics = {t.lstrip("/") for t in info.get("camera_topics", [])}
        if topic not in topics:
            label = "鱼眼" if name == "fisheye" else "中距"
            raise StudioError(f"数据源没有{label}相机话题（{topic}）；请选择包含该相机 Camera_* 分段的原始录制目录")
    return info


def browse(raw_path):
    p = safe_path(raw_path or "/home/project")
    if not p.is_dir():
        p = p.parent
    entries = []
    for child in sorted(p.iterdir(), key=lambda c: (not c.is_dir(), c.name.lower())):
        if child.name.startswith("."):
            continue
        try:
            is_dir = child.is_dir()
            kind = None
            if is_dir:
                if (child / "metadata.yaml").exists():
                    kind = "bag"
                else:
                    for sub in list(child.iterdir())[:80]:
                        if sub.is_dir() and (sub / "metadata.yaml").exists():
                            kind = "recording"
                            break
            entries.append({"name": child.name, "path": str(child), "dir": is_dir, "kind": kind,
                            "size": 0 if is_dir else child.stat().st_size})
        except OSError:
            continue
    here = None
    if (p / "metadata.yaml").exists():
        here = "bag"
    elif any(e["kind"] == "bag" for e in entries):
        here = "recording"
    return {"path": str(p), "parent": str(p.parent), "kind": here, "entries": entries[:500]}


def _win_key(window_start, window_duration):
    if window_start in (None, "", 0) and window_duration in (None, "", 0):
        return "full"
    try:
        s = float(window_start) if window_start not in (None, "") else 0.0
        d = float(window_duration) if window_duration not in (None, "") else 0.0
    except (TypeError, ValueError):
        return "full"
    if d <= 0 and s <= 0:
        return "full"
    return f"s{s:.2f}_d{d:.2f}" if d > 0 else f"s{s:.2f}_end"


def assess_key(info, window_start=None, window_duration=None):
    import hashlib
    wk = _win_key(window_start, window_duration)
    return hashlib.sha1(
        f"{info['path']}|{info.get('start_ns')}|{info.get('end_ns')}|{wk}".encode()
    ).hexdigest()[:16]


def assessment_file(raw_path, window_start=None, window_duration=None):
    """(report path, describe() info) of a recording; the key changes when the recording changes or the time
    window selection changes, so different windows are cached separately."""
    info = describe(raw_path)
    if info.get("error"):
        raise StudioError(f"数据源不可用：{info['error']}")
    return ASSESS_DIR / f"{assess_key(info, window_start, window_duration)}.json", info


def assessment(raw_path, window_start=None, window_duration=None):
    """Stored data-assessment report of a recording (result=None when it was never run)."""
    f, info = assessment_file(raw_path, window_start, window_duration)
    doc = read_json(f) if f.exists() else None
    return {"path": info["path"], "file": str(f), "result": doc}


def assessment_summary(raw_path):
    """Small verdict record for the overview / home page (uses the full-recording report)."""
    try:
        f, _ = assessment_file(raw_path)
    except StudioError:
        return None
    doc = read_json(f) if f.exists() else None
    if not doc:
        return None
    alerts = doc.get("alerts") or []
    return {"verdict": doc.get("verdict"), "generated": doc.get("generated"), "alerts": len(alerts),
            "bad_alerts": sum(1 for x in alerts if x.get("status") == "bad"), "counts": doc.get("counts"),
            "title": (doc.get("outlook") or {}).get("title"), "elapsed_s": doc.get("elapsed_s"),
            "startup": doc.get("startup")}


def _win_args(window_start, window_duration):
    args = []
    wk = _win_key(window_start, window_duration)
    if wk == "full":
        return args
    s = float(window_start) if window_start not in (None, "") else 0.0
    d = float(window_duration) if window_duration not in (None, "") else 0.0
    if d > 0:
        args += ["--window", f"{s:.3f}", f"{d:.3f}"]
    else:
        # assess.py --window expects (start, duration); pass start=0→1e9 as "start to end"
        args += ["--window", f"{s:.3f}", "1e9"]
    return args


def assess_cmd(raw_path, calibration_file=None, window_start=None, window_duration=None):
    """Command line of the assessment job + report path + describe() info."""
    f, info = assessment_file(raw_path, window_start, window_duration)
    ASSESS_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [PY, str(ASSESS_TOOL), info["path"], "--json", str(f)]
    cmd += _win_args(window_start, window_duration)
    if calibration_file and pathlib.Path(calibration_file).exists():
        cmd += ["--calibration", str(calibration_file)]
    return cmd, f, info


def assess_result_ctx(report):
    doc = read_json(report) or {}
    alerts = doc.get("alerts") or []
    stationary = (doc.get("startup") or {}).get("stationary")
    if stationary and load_state()["data"].get("dataset") == doc.get("dataset", {}).get("path"):
        update_state({"mapping": {"start_offset": stationary["start_s"]}})
    return {"verdict": doc.get("verdict", "?"), "alerts": len(alerts), "bad_alerts": sum(1 for x in alerts if x.get("status") == "bad"),
            "title": (doc.get("outlook") or {}).get("title", ""), "report": str(report), "elapsed_s": doc.get("elapsed_s"),
            "startup": doc.get("startup")}


def assess_pipeline_step(raw_path, calibration_file=None, force=False, window_start=None, window_duration=None):
    """Step 0 of a calibration pipeline: assess the recording unless a report for this exact recording+window exists.

    The report key is derived from path + first/last timestamp + window, so an unchanged recording/window is
    never read twice; a report made without a calibration file is redone once one is available (static extrinsic
    checks). Returns a job step spec or None (cache hit). Only *异常*-level uncorrectable alerts abort the pipeline;
    advisory warnings (including a short stationary startup prefix) are displayed but do not block it."""
    report, _info = assessment_file(raw_path, window_start, window_duration)
    doc = read_json(report) if report.exists() else None
    if doc and doc.get("schema") == 5 and not force and (doc.get("calibration") or not calibration_file):
        return None
    cmd, report, info = assess_cmd(raw_path, calibration_file, window_start=window_start, window_duration=window_duration)

    def after(ctx, lines, _r=report):
        res = assess_result_ctx(_r)
        ctx["assessment"] = res
        if res.get("bad_alerts"):
            raise StudioError(f"数据评估：{res['bad_alerts']} 项异常级问题无法通过标定补救（{res.get('title', '')}），已中止流水线；"
                              f"请在「数据源」页查看评估报告，更换或重新采集数据")

    return {"title": f"数据评估 · {info.get('name')}（并行流式读取；已有报告时自动跳过）", "cmd": cmd, "after": after}


def recording_kind(path):
    p = pathlib.Path(path)
    if (p / "metadata.yaml").exists():
        return "bag"
    if any((s / "metadata.yaml").exists() for s in p.glob("*")) or any((s / "metadata.yaml").exists() for s in p.glob("*/*")):
        return "recording"
    return None
