"""Recordings: inspection of raw ROS 2 recording roots / single bag folders via tools/livo/dataset.py
(no conversion, no prepared bag) plus a small file browser."""
import pathlib
import subprocess
import threading

from .common import DATASET_TOOL, PY, REPO, StudioError, now, safe_path

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
    if camera and not info.get("livo_ready"):
        raise StudioError("数据源没有相机话题（midrange_camera/ffmpeg）：LIVO / 相机标定需要包含 Camera_* 分段的原始录制目录")
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


def recording_kind(path):
    p = pathlib.Path(path)
    if (p / "metadata.yaml").exists():
        return "bag"
    if any((s / "metadata.yaml").exists() for s in p.glob("*")) or any((s / "metadata.yaml").exists() for s in p.glob("*/*")):
        return "recording"
    return None
