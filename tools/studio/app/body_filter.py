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
from livo.body_filter import (DEFAULT_REAR_AXLE_BOX, LIDAR_TOPICS, lidar_filter_mask,
                              lidar_transform_matrix, normalize, stamp, to_yaml_overrides,
                              transform_points)
from livo.dataset import Dataset

MAX_POINTS = 12000
MULTI_COLORS = ("#0071e3", "#34c759", "#ff9f0a", "#af52de", "#ff2d55",
                "#5ac8fa", "#a2845e", "#30b0c7")
MULTI_MAX_POINTS = 12000
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
                "topic_key": _key(topic), "t_s": t_s, "stamps": stamps, "points": [], "points_full": [],
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
        "raw_points": raw_count, "points": P.round(3).tolist(), "points_full": P.tolist(),
        "filtered": mask.astype(bool).tolist(), "filtered_count": int(mask.sum()),
        "xy": xy, "xz": xz, "minimum_z": float(filt["minimum_z"]),
        "enabled": bool(filt["enabled"]),
    }


def _rect_polygons(rectangles, M):
    """Transform 2D LiDAR-frame rectangles into rear-axle-frame polygons."""
    out = []
    for r in rectangles or []:
        try:
            corners = np.array([
                [float(r["x_min"]), float(r["y_min"]), 0.0],
                [float(r["x_max"]), float(r["y_min"]), 0.0],
                [float(r["x_max"]), float(r["y_max"]), 0.0],
                [float(r["x_min"]), float(r["y_max"]), 0.0],
                [float(r["x_min"]), float(r["y_min"]), 0.0],
            ], dtype=float)
            Q = transform_points(corners, M)
            out.append({"rect": {k: float(r[k]) for k in ("x_min", "x_max", "y_min", "y_max")},
                        "polygon": Q[:, :2].round(3).tolist()})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def _multi_bounds(series, rear_box):
    xs, ys = [], []
    for item in series:
        for x, y, _z in item.get("points") or []:
            xs.append(float(x)); ys.append(float(y))
        for poly in item.get("rectangles") or []:
            for x, y in poly.get("polygon") or []:
                xs.append(float(x)); ys.append(float(y))
    for x in (rear_box["x_min"], rear_box["x_max"]):
        for y in (rear_box["y_min"], rear_box["y_max"]):
            xs.append(float(x)); ys.append(float(y))
    if not xs or not ys:
        return {"x_min": -12.0, "x_max": 12.0, "y_min": -12.0, "y_max": 12.0}
    xa, ya = np.asarray(xs, float), np.asarray(ys, float)
    xa, ya = xa[np.isfinite(xa)], ya[np.isfinite(ya)]
    if not len(xa) or not len(ya):
        return {"x_min": -12.0, "x_max": 12.0, "y_min": -12.0, "y_max": 12.0}
    x_min, x_max = float(np.nanpercentile(xa, 1)), float(np.nanpercentile(xa, 99))
    y_min, y_max = float(np.nanpercentile(ya, 1)), float(np.nanpercentile(ya, 99))
    x_min, x_max = min(x_min, float(xa.min())), max(x_max, float(xa.max()))
    y_min, y_max = min(y_min, float(ya.min())), max(y_max, float(ya.max()))
    dx = max(x_max - x_min, 2.0) * 0.06
    dy = max(y_max - y_min, 2.0) * 0.06
    return {"x_min": x_min - dx, "x_max": x_max + dx,
            "y_min": y_min - dy, "y_max": y_max + dy}


def _sample_multi_scan(ds, filt, t, max_points, parent_calib):
    """Sample every LiDAR topic and stitch the points into the rear-axle frame."""
    topics = list(ds.lidar_topics or [])
    if not topics:
        return {"error": "数据集中没有 LiDAR 点云话题", "series": [], "xy": _multi_bounds([], filt["rear_axle_box"]),
                "raw_points": 0, "sampled_points": 0, "filtered_count": 0, "topics": [],
                "colors": {}, "rear_axle_box": filt["rear_axle_box"]}
    budget = max(1000, min(MULTI_MAX_POINTS, _finite(max_points, MULTI_MAX_POINTS)))
    per_topic = max(1000, budget // len(topics))
    series, colors = [], {}
    for i, topic in enumerate(topics):
        key = _key(topic)
        color = MULTI_COLORS[i % len(MULTI_COLORS)]
        colors[key] = color
        M = lidar_transform_matrix(parent_calib, topic)
        try:
            one = _sample_scan(ds, topic, filt, t, per_topic)
            P = np.asarray(one.get("points_full") or one.get("points") or [], dtype=float).reshape(-1, 3)
            Q = transform_points(P, M)
            filtered = [bool(v) for v in (one.get("filtered") or [])]
            series.append({
                "topic": topic, "topic_key": key, "color": color,
                "points": Q.round(3).tolist(), "points_local": P.tolist(), "filtered": filtered,
                "raw_points": int(one.get("raw_points") or 0),
                "sampled_points": int(len(Q)),
                "filtered_count": int(sum(filtered)),
                "t_s": float(one.get("t_s") or 0.0),
                "stamps": one.get("stamps") or [],
                "rectangles": _rect_polygons(filt["lidar_rectangles"].get(key, []), M),
                "transform_identity": bool(np.allclose(M, np.eye(4))),
                "error": one.get("error"),
            })
        except Exception as exc:  # noqa: BLE001 - one bad topic must not hide the others
            series.append({
                "topic": topic, "topic_key": key, "color": color,
                "points": [], "points_local": [], "filtered": [], "raw_points": 0, "sampled_points": 0,
                "filtered_count": 0, "t_s": _finite(t, 0.0), "stamps": [],
                "rectangles": _rect_polygons(filt["lidar_rectangles"].get(key, []), M),
                "transform_identity": bool(np.allclose(M, np.eye(4))),
                "error": str(exc),
            })
    return {
        "frame": "vehicle_rear_axle", "topics": topics, "colors": colors,
        "series": series, "xy": _multi_bounds(series, filt["rear_axle_box"]),
        "rear_axle_box": filt["rear_axle_box"],
        "raw_points": int(sum(x["raw_points"] for x in series)),
        "sampled_points": int(sum(x["sampled_points"] for x in series)),
        "filtered_count": int(sum(x["filtered_count"] for x in series)),
        "t_s": float(series[0]["t_s"]) if series else _finite(t, 0.0),
    }


def get(mine=None, truck=None, vid=None, dataset=None, topic=None, t=None, max_points=MAX_POINTS,
        parent_id=None):
    mine, truck = _defaults(mine, truck)
    path, version_id = _calibration_file(mine, truck, vid)
    calib = read_json(path, {}) or {}
    filt = normalize(calib)
    parent_path, parent_version_id = path, version_id
    if parent_id:
        try:
            if not str(parent_id).startswith(f"{mine}/{truck}/"):
                raise StudioError("父版本不属于当前车辆")
            candidate = calibration.calibration_file(parent_id)
            parent_calib = read_json(candidate, {}) or {}
            if parent_calib:
                parent_path, parent_version_id = candidate, parent_id
            else:
                parent_calib = calib
        except Exception:  # noqa: BLE001 - bad parent id falls back to the active calibration
            parent_calib = calib
    else:
        parent_calib = calib
    dataset_path = _dataset_path(dataset)
    multi_scan = {"error": "数据集未打开", "series": [], "xy": _multi_bounds([], filt["rear_axle_box"]),
                  "raw_points": 0, "sampled_points": 0, "filtered_count": 0, "topics": [],
                  "colors": {}, "rear_axle_box": filt["rear_axle_box"]}
    try:
        ds = Dataset.open(dataset_path)
        topics = ds.lidar_topics
        duration_s = float(ds.duration_s)
        scan = _sample_scan(ds, topic, filt, t, max_points)
        topic = scan.get("topic") or _pick_topic(ds, topic)
        multi_scan = _sample_multi_scan(ds, filt, t, max_points, parent_calib)
    except Exception as exc:  # noqa: BLE001 - a bad dataset must not hide the saved parameters
        topics, duration_s = list(LIDAR_TOPICS), 0.0
        scan = {"error": str(exc), "topic": topic or LIDAR_TOPICS[0],
                "topic_key": _key(topic or LIDAR_TOPICS[0]), "t_s": _finite(t, 0.0),
                "stamps": [], "points": [], "filtered": [], "filtered_count": 0, "raw_points": 0,
                "xy": {"x_min": -10, "x_max": 10, "y_min": -10, "y_max": 10},
                "xz": {"x_min": -10, "x_max": 10, "y_min": -4, "y_max": 4}}
        topic = scan["topic"]
        multi_scan["error"] = str(exc)
    return {
        "mine": mine, "truck": truck, "id": version_id, "calibration_file": str(path),
        "parent_id": parent_version_id, "parent_calibration_file": str(parent_path),
        "dataset": dataset_path, "duration_s": duration_s, "topics": topics,
        "topic": topic, "topic_key": scan.get("topic_key") or _key(topic),
        "body_filter": filt, "yaml_overrides": to_yaml_overrides(filt), "scan": scan,
        "multi_scan": multi_scan,
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
