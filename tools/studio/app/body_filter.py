# Studio visual editor for vehicle-specific body-point filters.
# Reads one real point-cloud sample from a recording, serves it to the browser canvas,
# and persists the edited rectangles / minimum_z / rear-axle box in calibration.json.
from __future__ import annotations

import json
import math
import pathlib
import shutil

import numpy as np

from . import calibration
from .common import StudioError, load_state, read_json, safe_path
from livo import rosenv
from livo.body_filter import (DEFAULT_REAR_AXLE_BOX, LIDAR_TOPICS, lidar_filter_mask, normalize,
                              stamp, to_yaml_overrides)
from livo.dataset import Dataset

MAX_POINTS = 12000
SCAN_MESSAGES = 3
MAX_RANGE = 30.0


def _key(topic):
    return str(topic).strip("/").replace("/", "_")


def _finite(value, default=0.0):
    try:
        out = float(value)
        return out if math.isfinite(out) else float(default)
    except (TypeError, ValueError):
        return float(default)


def _defaults(mine, truck):
    vehicle = (load_state().get("vehicle") or {})
    return mine or vehicle.get("mine"), truck or vehicle.get("truck")


def _dataset_path(raw):
    if not raw:
        raw = (load_state().get("data") or {}).get("dataset")
    if not raw:
        raise StudioError("请先在「数据源」页选择录制目录")
    return str(safe_path(raw))


def _calibration_file(mine, truck, vid=None):
    mine, truck = _defaults(mine, truck)
    if not mine or not truck:
        raise StudioError("请先选择矿山 / 车辆")
    if vid:
        return calibration.calibration_file(vid), vid
    cat = calibration.catalogue()
    status = calibration.vehicle_status(cat, mine, truck)
    if status.get("calibration_file"):
        return pathlib.Path(status["calibration_file"]), status.get("active")
    tip = status.get("frontier_tip")
    if tip:
        return calibration.calibration_file(tip), tip
    raise StudioError(f"{mine}/{truck} 还没有可用的标定版本")


def _bounds(xs, ys, rectangles=(), extra_y=None, default_y=(-10.0, 10.0)):
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    xs = xs[np.isfinite(xs)]
    ys = ys[np.isfinite(ys)]
    if not len(xs) or not len(ys):
        return {"x_min": -10.0, "x_max": 10.0, "y_min": default_y[0], "y_max": default_y[1]}
    x_min = float(min(np.nanpercentile(xs, 1), -10.0))
    x_max = float(max(np.nanpercentile(xs, 99), 10.0))
    y_min = float(min(np.nanpercentile(ys, 1), default_y[0]))
    y_max = float(max(np.nanpercentile(ys, 99), default_y[1]))
    for r in rectangles or ():
        x_min = min(x_min, float(r["x_min"]))
        x_max = max(x_max, float(r["x_max"]))
        y_min = min(y_min, float(r["y_min"]))
        y_max = max(y_max, float(r["y_max"]))
    if extra_y is not None:
        y_min = min(y_min, float(extra_y))
        y_max = max(y_max, float(extra_y))
    dx = max(x_max - x_min, 1.0) * 0.05
    dy = max(y_max - y_min, 1.0) * 0.05
    return {"x_min": x_min - dx, "x_max": x_max + dx,
            "y_min": y_min - dy, "y_max": y_max + dy}


def _pick_topic(ds, wanted):
    topics = ds.lidar_topics
    if wanted:
        resolved = ds.resolve_topics([wanted])
        if not resolved:
            raise StudioError(f"数据集中没有 LiDAR 话题 {wanted}")
        return resolved[0]
    for t in topics:
        if _key(t) in LIDAR_TOPICS:
            return t
    if topics:
        return topics[0]
    raise StudioError("数据集中没有 LiDAR 点云话题")


def _sample_scan(ds, topic, filt, t_s, max_points):
    rosenv.ensure()
    from sensor_msgs_py import point_cloud2 as pc2

    topic = _pick_topic(ds, topic)
    t_s = max(0.0, min(_finite(t_s, 0.0), max(0.0, ds.duration_s - 0.1)))
    max_points = int(max(1000, min(_finite(max_points, MAX_POINTS), 30000)))
    start_ns = ds.start_ns + int(t_s * 1e9)
    end_ns = min(ds.end_ns, start_ns + int(2.0e9))
    clouds, stamps = [], []
    for _topic, msg, ts in ds.read_messages([topic], start_ns=start_ns, end_ns=end_ns):
        arr = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        P = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)
        P = P[(P * P).sum(1) < MAX_RANGE * MAX_RANGE]
        if len(P):
            clouds.append(P)
        stamps.append(round((ts - ds.start_ns) / 1e9, 3))
        if len(clouds) >= SCAN_MESSAGES:
            break
    if not clouds:
        return {"error": "该时间点附近没有读到点云；请调整采样时刻", "topic": topic,
                "topic_key": _key(topic), "t_s": t_s, "stamps": stamps, "points": [],
                "filtered": [], "filtered_count": 0, "raw_points": 0,
                "xy": {"x_min": -10, "x_max": 10, "y_min": -10, "y_max": 10},
                "xz": {"x_min": -10, "x_max": 10, "y_min": -4, "y_max": 4}}
    P = np.vstack(clouds)
    raw_count = int(len(P))
    if raw_count > max_points:
        rng = np.random.default_rng(20261026)
        P = P[rng.choice(raw_count, max_points, replace=False)]
    mask = lidar_filter_mask(P, filt, topic)
    rects = filt["lidar_rectangles"].get(_key(topic), [])
    xy = _bounds(P[:, 0], P[:, 1], rects)
    xz = _bounds(P[:, 0], P[:, 2], [], extra_y=filt["minimum_z"], default_y=(-4.0, 4.0))
    return {
        "topic": topic, "topic_key": _key(topic), "t_s": t_s, "stamps": stamps,
        "raw_points": raw_count, "points": P.round(3).tolist(),
        "filtered": mask.astype(bool).tolist(), "filtered_count": int(mask.sum()),
        "xy": xy, "xz": xz, "minimum_z": float(filt["minimum_z"]),
        "enabled": bool(filt["enabled"]),
    }


def get(mine=None, truck=None, vid=None, dataset=None, topic=None, t=None, max_points=MAX_POINTS):
    mine, truck = _defaults(mine, truck)
    path, version_id = _calibration_file(mine, truck, vid)
    calib = read_json(path, {}) or {}
    filt = normalize(calib)
    dataset_path = _dataset_path(dataset)
    try:
        ds = Dataset.open(dataset_path)
        topics = ds.lidar_topics
        duration_s = float(ds.duration_s)
        scan = _sample_scan(ds, topic, filt, t, max_points)
        topic = scan.get("topic") or _pick_topic(ds, topic)
    except Exception as exc:  # noqa: BLE001 - a bad dataset must not hide the saved parameters
        topics, duration_s = list(LIDAR_TOPICS), 0.0
        scan = {"error": str(exc), "topic": topic or LIDAR_TOPICS[0],
                "topic_key": _key(topic or LIDAR_TOPICS[0]), "t_s": _finite(t, 0.0),
                "stamps": [], "points": [], "filtered": [], "filtered_count": 0, "raw_points": 0,
                "xy": {"x_min": -10, "x_max": 10, "y_min": -10, "y_max": 10},
                "xz": {"x_min": -10, "x_max": 10, "y_min": -4, "y_max": 4}}
        topic = scan["topic"]
    return {
        "mine": mine, "truck": truck, "id": version_id, "calibration_file": str(path),
        "dataset": dataset_path, "duration_s": duration_s, "topics": topics,
        "topic": topic, "topic_key": scan.get("topic_key") or _key(topic),
        "body_filter": filt, "yaml_overrides": to_yaml_overrides(filt), "scan": scan,
    }


def save(body):
    if not isinstance(body, dict):
        raise StudioError("请求体必须是 JSON 对象")
    mine, truck = _defaults(body.get("mine"), body.get("truck"))
    path, version_id = _calibration_file(mine, truck, body.get("id"))
    calib = read_json(path, {})
    if not isinstance(calib, dict) or not calib:
        raise StudioError(f"标定文件不可读取：{path}")
    raw = body.get("body_filter")
    if not isinstance(raw, dict):
        raise StudioError("缺少 body_filter 参数")
    filt = stamp(normalize({"body_filter": raw}),
                 str(body.get("source") or "Studio 车身点云可视化过滤"))
    calib["body_filter"] = filt
    try:
        backup = path.with_name(path.name + ".body_filter.bak")
        shutil.copy2(path, backup)
    except OSError:
        pass
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(calib, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    tmp.replace(path)
    try:
        calibration.catalogue(refresh=True)
    except Exception:  # noqa: BLE001 - cache refresh is best effort
        pass
    return {"ok": True, "mine": mine, "truck": truck, "id": version_id,
            "calibration_file": str(path), "body_filter": filt,
            "yaml_overrides": to_yaml_overrides(filt)}
