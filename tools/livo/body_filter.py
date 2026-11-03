"""Vehicle-specific body-point filter stored in calibration.json.

The current filter is one XY rectangle in the stitched rear-axle frame, with a
shared minimum-z cutoff. Older calibration files may still contain per-LiDAR
rectangles; those remain supported until a unified rectangle is saved.

Missing/invalid fields fall back to the historical truck29 values, so old
calibration files keep their previous behaviour.
"""
from __future__ import annotations

import datetime as dt
import json
import math
import pathlib
from typing import Any

import numpy as np

DEFAULT_MINIMUM_Z = -3.0
DEFAULT_REAR_AXLE_BOX = {"x_min": -2.8, "x_max": 2.8, "y_min": -4.5, "y_max": 9.5}
LIDAR_TOPICS = ("front_right_lidar", "back_lidar", "front_left_lidar", "front_lidar")
LIDAR_MATRIX_KEYS = {
    "front_right_lidar": "transform_matrix_lidar_1",
    "back_lidar": "transform_matrix_lidar_2",
    "front_left_lidar": "transform_matrix_lidar_3",
    "front_lidar": "transform_matrix_lidar_6",
}
DEFAULT_LIDAR_RECTANGLES = {
    "front_right_lidar": [
        {"x_min": -3.0, "y_min": -9.0, "x_max": 0.5, "y_max": 2.0},
        {"x_min": 0.0, "y_min": -1.0, "x_max": 1.0, "y_max": 1.0},
    ],
    "back_lidar": [
        {"x_min": -1.0, "y_min": -2.0, "x_max": 3.0, "y_max": 2.0},
    ],
    "front_left_lidar": [
        {"x_min": -3.0, "y_min": -3.0, "x_max": 1.0, "y_max": 1.0},
        {"x_min": -2.0, "y_min": 1.0, "x_max": 0.0, "y_max": 9.0},
    ],
    "front_lidar": [],
}


def _finite(value: Any, default: float) -> float:
    try:
        out = float(value)
        return out if math.isfinite(out) else float(default)
    except (TypeError, ValueError):
        return float(default)


def _rect(value: Any) -> dict | None:
    if isinstance(value, dict):
        x1 = _finite(value.get("x_min", value.get("x1")), np.nan)
        y1 = _finite(value.get("y_min", value.get("y1")), np.nan)
        x2 = _finite(value.get("x_max", value.get("x2")), np.nan)
        y2 = _finite(value.get("y_max", value.get("y2")), np.nan)
    elif isinstance(value, (list, tuple)) and len(value) >= 4:
        x1, y1, x2, y2 = (_finite(v, np.nan) for v in value[:4])
    else:
        return None
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
        return None
    x_min, x_max = sorted((x1, x2))
    y_min, y_max = sorted((y1, y2))
    if x_max <= x_min or y_max <= y_min:
        return None
    return {"x_min": x_min, "y_min": y_min, "x_max": x_max, "y_max": y_max}


def _topic_key(topic: str) -> str:
    return str(topic).strip("/").replace("/", "_")


def normalize(doc: dict | None) -> dict:
    raw = ((doc or {}).get("body_filter") or {}) if isinstance(doc, dict) else {}
    rear = _rect(raw.get("rear_axle_box") or DEFAULT_REAR_AXLE_BOX) or dict(DEFAULT_REAR_AXLE_BOX)
    fused = _rect(raw.get("fused_rectangle"))
    rects: dict[str, list[dict]] = {}
    raw_rects = raw.get("lidar_rectangles") or {}
    if fused:
        # A saved unified filter is authoritative. Do not carry a confusing set
        # of dormant per-sensor rectangles into the normalized calibration.
        rects = {topic: [] for topic in LIDAR_TOPICS}
    else:
        for topic in LIDAR_TOPICS:
            values = raw_rects.get(topic, raw_rects.get(_topic_key(topic), DEFAULT_LIDAR_RECTANGLES[topic]))
            rects[topic] = [r for r in (_rect(v) for v in values) if r]
        for topic, values in raw_rects.items():
            key = _topic_key(topic)
            if key not in rects:
                rects[key] = [r for r in (_rect(v) for v in values) if r]
    return {
        "enabled": bool(raw.get("enabled", True)),
        "minimum_z": _finite(raw.get("minimum_z"), DEFAULT_MINIMUM_Z),
        "rear_axle_box": rear,
        "fused_rectangle": fused,
        "lidar_rectangles": rects,
        "source": str(raw.get("source", "default")),
        "updated": str(raw.get("updated", "")),
    }


def lidar_transform_matrix(doc: dict | None, topic: str) -> np.ndarray:
    """4x4 lidar->rear-axle transform for `topic` from a calibration.json.

    The schema stores the matrices under lidar[0].transform_matrix_lidar_N; older
    files may expose a transforms.<topic>_to_rear_axle entry.  Missing entries
    fall back to identity so a new LiDAR topic still appears in the stitched view.
    """
    key = LIDAR_MATRIX_KEYS.get(_topic_key(topic))
    doc = doc or {}
    entries = doc.get("lidar") or []
    if isinstance(entries, dict):
        entries = [entries]
    for entry in entries:
        if isinstance(entry, dict) and key and key in entry:
            try:
                return np.asarray(entry[key], dtype=float).reshape(4, 4)
            except (TypeError, ValueError):
                break
    transforms = doc.get("transforms") or {}
    for name in (f"{_topic_key(topic)}_to_rear_axle", f"{topic}_to_rear_axle"):
        entry = transforms.get(name) if isinstance(transforms, dict) else None
        if isinstance(entry, dict) and "transform_matrix" in entry:
            try:
                return np.asarray(entry["transform_matrix"], dtype=float).reshape(4, 4)
            except (TypeError, ValueError):
                break
    return np.eye(4)


def transform_points(P: np.ndarray, M: np.ndarray) -> np.ndarray:
    """Apply a target-from-source 4x4 matrix to Nx3 points."""
    P = np.asarray(P, dtype=float)
    if P.size == 0:
        return np.empty((0, 3), dtype=float)
    return P @ np.asarray(M, dtype=float)[:3, :3].T + np.asarray(M, dtype=float)[:3, 3]


def from_calibration_file(path: str | pathlib.Path) -> dict:
    try:
        doc = json.loads(pathlib.Path(path).read_text())
    except (OSError, json.JSONDecodeError):
        doc = {}
    return normalize(doc)


def rear_axle_mask(Q: np.ndarray, filt: dict) -> np.ndarray:
    if not filt.get("enabled", True) or len(Q) == 0:
        return np.zeros(len(Q), dtype=bool)
    if filt.get("fused_rectangle"):
        return fused_rectangle_mask(Q, filt)
    b = filt["rear_axle_box"]
    return ((Q[:, 0] > b["x_min"]) & (Q[:, 0] < b["x_max"]) &
            (Q[:, 1] > b["y_min"]) & (Q[:, 1] < b["y_max"]))


def fused_rectangle_mask(Q: np.ndarray, filt: dict) -> np.ndarray:
    """Mask points already expressed in the stitched rear-axle frame."""
    Q = np.asarray(Q, dtype=float).reshape(-1, 3)
    rectangle = filt.get("fused_rectangle")
    if not filt.get("enabled", True) or not rectangle or len(Q) == 0:
        return np.zeros(len(Q), dtype=bool)
    inside = ((Q[:, 0] > rectangle["x_min"]) & (Q[:, 0] < rectangle["x_max"]) &
              (Q[:, 1] > rectangle["y_min"]) & (Q[:, 1] < rectangle["y_max"]))
    return inside & (Q[:, 2] > float(filt["minimum_z"]))


def lidar_filter_mask(Q: np.ndarray, filt: dict, topic: str) -> np.ndarray:
    if not filt.get("enabled", True) or len(Q) == 0:
        return np.zeros(len(Q), dtype=bool)
    inside = np.zeros(len(Q), dtype=bool)
    for r in filt["lidar_rectangles"].get(_topic_key(topic), []):
        inside |= ((Q[:, 0] > r["x_min"]) & (Q[:, 0] < r["x_max"]) &
                   (Q[:, 1] > r["y_min"]) & (Q[:, 1] < r["y_max"]))
    return inside & (Q[:, 2] > filt["minimum_z"])


def to_yaml_overrides(filt: dict) -> dict:
    """Flatten the calibration schema into ROS 2 parameter overrides."""
    bf = {"enabled": bool(filt.get("enabled", True)),
          "minimum_z": float(filt["minimum_z"])}
    if filt.get("fused_rectangle"):
        r = filt["fused_rectangle"]
        bf["fused_rectangle"] = [r["x_min"], r["y_min"], r["x_max"], r["y_max"]]
    else:
        for topic in sorted(set(LIDAR_TOPICS) | set(filt["lidar_rectangles"])):
            flat = []
            for r in filt["lidar_rectangles"].get(topic, []):
                flat += [r["x_min"], r["y_min"], r["x_max"], r["y_max"]]
            bf[topic] = flat
    b = filt["rear_axle_box"]
    return {"body_filter": bf,
            "precheck": {"body_box": [b["x_min"], b["x_max"], b["y_min"], b["y_max"]]}}


def stamp(filt: dict, source: str = "studio visual editor") -> dict:
    out = dict(filt)
    out["source"] = source
    out["updated"] = dt.datetime.now().astimezone().isoformat(timespec="seconds")
    return out
