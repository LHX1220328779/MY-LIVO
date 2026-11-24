"""Select the calibrated camera without changing the estimator's pinhole geometry.

The supplier's fish_camera stream on Xingye13 is already rectified. Its supplied
3x4 projection matrix and five zero coefficients describe that image, not an
equidistant raw sensor. Never infer an OpenCV fisheye model from a camera name.
"""
import argparse
import copy
import json
from pathlib import Path

import numpy as np

CAMERAS = {"midrange": "midrange_camera", "fisheye": "fish_camera"}


def selected_camera(calib):
    name = calib.get("camera_selection", "midrange")
    if name not in CAMERAS:
        raise ValueError(f"Unknown camera selection: {name}")
    return name


def camera_transform_key(calib):
    return CAMERAS[selected_camera(calib)] + "_to_rear_axle"


def available_cameras(calib):
    return [name for name, field in CAMERAS.items()
            if name == selected_camera(calib) or calib.get(field)
            or name in calib.get("camera_profiles", {})]


def fisheye_activation_issues(calib, closed_loop=None):
    """Evidence required to promote a fish-camera candidate; diagnostics remain runnable."""
    if selected_camera(calib) != "fisheye":
        return []
    issues = []
    stages, camera = set(), "midrange"
    for entry in calib.get("history", []):
        camera = entry.get("camera_selection", entry.get("metrics", {}).get("camera_selection", camera))
        if camera == "fisheye":
            stages.add(entry.get("stage"))
    missing = {"camtime", "camlidar", "camfocus", "camphoto"} - stages
    if missing:
        issues.append("鱼眼标定尚缺分阶段记录：" + ", ".join(sorted(missing)))
    photo = calib.get("refinement", {}).get("camera_photometric", {})
    coverage = photo.get("coverage", {})
    if (photo.get("camera") != "fisheye" or coverage.get("camera_segment_count", 0) < 2
            or coverage.get("frames", 0) < 50 or coverage.get("views", 0) < 3):
        issues.append("鱼眼联合标定需要至少两个相机数据包和多帧观测")
    validation = photo.get("validation", {})
    runs = validation.get("runs", [])
    if (validation.get("passed") is not True or validation.get("refitted") is not False or not runs
            or any(r.get("passed") is not True or r.get("camera") != "fisheye"
                   or r.get("refitted") is not False
                   or r.get("coverage", {}).get("camera_segment_count", 0) < 2
                   or r.get("coverage", {}).get("frames", 0) < 50
                   or r.get("coverage", {}).get("views", 0) < 3 for r in runs)):
        issues.append("缺少独立多包路段的固定参数验证，或验证未通过")
    expected = {"camera": calib.get("camera"),
                "transform_matrix": calib.get("transforms", {}).get(camera_transform_key(calib), {}).get("transform_matrix"),
                "camera_offset": calib.get("time_offsets", {}).get("camera")}
    if photo.get("validated_parameters") != expected:
        issues.append("当前相机参数与独立验证参数不一致或尚未验证")
    source = str(calib.get("time_offsets", {}).get("camera_source", "")).lower()
    if not source or any(word in source for word in ("unmeasured", "unverified", "default")):
        issues.append("鱼眼相机时间偏移尚未确认")
    cl = closed_loop or {}
    if (cl.get("verdict") != "pass" or cl.get("camera") != "fisheye"
            or cl.get("candidate", {}).get("calibration_id") != calib.get("identity", {}).get("id")):
        issues.append("当前鱼眼版本尚未通过实际 LIVO 建图闭环验证")
    return issues


def sync_camera_profile(calib):
    """Persist edits of the selected camera, leaving the other camera untouched."""
    name = selected_camera(calib)
    if "camera" in calib:
        offsets = calib.get("time_offsets", {})
        calib.setdefault("camera_profiles", {})[name] = {
            "camera": copy.deepcopy(calib["camera"]),
            "time_offset": offsets.get("camera", 0.0),
            "time_source": offsets.get("camera_source", "unmeasured"),
        }


def _matrix(values):
    m = np.asarray(values, dtype=float)
    if m.size == 18:
        m = m[2:]
    m = m.reshape(4, 4).copy()
    if not np.isfinite(m).all() or not np.allclose(m[3], [0, 0, 0, 1]):
        raise ValueError("Invalid camera rigid transform")
    u, _, vt = np.linalg.svd(m[:3, :3])
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    m[:3, :3] = r
    return m


def _factory_rig(calib, root):
    """Recover the imported rig and each axle correction, never LiDAR-only edits."""
    history = calib.get("history", [])
    imports = [h for h in history if h.get("stage") == "import"]
    if not imports:
        if history or calib.get("refinement"):
            raise ValueError("Missing factory lineage: cannot derive a new camera from refined LiDAR extrinsics")
        return calib, np.eye(4)
    # The first child points to the actual normalized import, including any
    # orthonormalization. Paths recorded on another host are not used.
    import_id = next((h.get("parent") for h in history if h.get("parent")), None)
    if import_id is None:
        return calib, np.eye(4)
    parts = Path(import_id).parts
    if len(parts) != 3 or any(p in ("..", ".") for p in parts):
        raise ValueError("Invalid factory lineage ID")
    path = root.joinpath(*parts, "calibration.json")
    if not path.is_file():
        raise ValueError(f"Factory calibration required to add this camera: {path}")
    factory = json.loads(path.read_text())
    correction = np.eye(4)
    for entry in history:
        if entry.get("stage") == "axle":
            values = entry.get("metrics", {}).get("rear_axle_alignment", {}).get("correction_matrix")
            if values is None:
                raise ValueError("Axle history is missing its rigid correction; cannot transfer camera extrinsics")
            correction = _matrix(values) @ correction
            lateral = entry.get("metrics", {}).get("lateral", {})
            if str(lateral.get("decision", "")).startswith("applied"):
                extra = np.eye(4)
                extra[0, 3] = float(lateral["correction_x_cm"]) / 100.0
                correction = extra @ correction
    return factory, correction


def select_camera(calib, name=None, *, calibration_root=None):
    out = copy.deepcopy(calib)
    current = selected_camera(out)
    name = name or current
    if name not in CAMERAS:
        raise ValueError(f"Unknown camera: {name}")
    sync_camera_profile(out)
    profiles = out.setdefault("camera_profiles", {})
    field = CAMERAS[name]
    key = field + "_to_rear_axle"
    if name not in profiles:
        entries = out.get(field)
        if not isinstance(entries, list) or not entries:
            raise ValueError(f"Calibration has no {field}[0]; supply its parameters in the same factory JSON")
        source = entries[0]
        p = source.get("camera_in_params", [])
        if len(p) != 12:
            raise ValueError(f"{field}[0].camera_in_params must contain a 3x4 projection matrix")
        camera = {"topic": f"/{field}/image_raw", "stream_topic": f"/{field}/ffmpeg",
                  "width": int(source.get("image_width", 1920)),
                  "height": int(source.get("image_height", 1080)),
                  "intrinsics": [float(p[i]) for i in (0, 5, 2, 6)],
                  "distortion": [float(v) for v in source.get("distcoeff", [0] * 5)],
                  "model": source.get("model", "pinhole"),
                  "source": f"{field}[0].camera_in_params (supplier rectified projection)"}
        profiles[name] = {"camera": camera, "time_offset": 0.0,
                          "time_source": "unmeasured; run camera-time for this camera"}
    if key not in out.setdefault("transforms", {}):
        root = Path(calibration_root) if calibration_root else Path(__file__).resolve().parents[2] / "config/carlibration"
        factory, correction = _factory_rig(out, root)
        ref = factory.get("calibration_reference_lidar") or factory.get("correlated_lidar_matrix_tf") or "transform_matrix_lidar_6"
        rear_lidar = next((e[ref] for e in factory.get("lidar", []) if ref in e), None)
        source = (factory.get(field) or out.get(field) or [{}])[0]
        if rear_lidar is None or "tf_lidar_to_camera" not in source:
            raise ValueError(f"Missing factory {ref} / {field}[0].tf_lidar_to_camera")
        transform = correction @ _matrix(rear_lidar) @ np.linalg.inv(_matrix(source["tf_lidar_to_camera"]))
        out["transforms"][key] = {"source_frame": field, "target_frame": "vehicle_rear_axle",
                                   "transform_semantics": "target_from_source",
                                   "derived_from": f"axle corrections * factory {ref} * inverse({field}[0].tf_lidar_to_camera)",
                                   "transform_matrix": transform.reshape(-1).tolist()}
    out["camera_selection"] = name
    out["camera"] = copy.deepcopy(profiles[name]["camera"])
    cam = out["camera"]
    cam.setdefault("model", "pinhole")
    cam.setdefault("stream_topic", f"/{field}/ffmpeg")
    other = CAMERAS["midrange" if name == "fisheye" else "fisheye"]
    for topic_key in ("topic", "stream_topic"):
        topic = cam.get(topic_key)
        if not isinstance(topic, str) or not topic.strip() or other in topic.strip("/").split("/"):
            raise ValueError(f"{name} camera has an invalid or mismatched {topic_key}: {topic}")
    if not np.isfinite(float(profiles[name]["time_offset"])):
        raise ValueError("Camera time offset must be finite")
    if cam["model"] not in ("pinhole", "rectified_pinhole"):
        raise ValueError("This LIVO input requires a rectified pinhole stream; raw fisheye images need a calibrated rectification first")
    intr = np.asarray(cam.get("intrinsics", []), float)
    dist = np.asarray(cam.get("distortion", []), float)
    if intr.shape != (4,) or not np.isfinite(intr).all() or np.any(intr[:2] <= 0):
        raise ValueError("Invalid camera intrinsics")
    if cam.get("width", 0) <= 0 or cam.get("height", 0) <= 0:
        raise ValueError("Camera image dimensions must be positive")
    if not (0 < intr[2] < cam["width"] and 0 < intr[3] < cam["height"]):
        raise ValueError("Camera principal point is outside the image")
    if dist.shape != (5,) or not np.isfinite(dist).all():
        raise ValueError("Camera distortion must have five finite pinhole coefficients")
    if name == "fisheye" and np.any(dist != 0):
        # Residual Brown distortion on an already rectified wide-angle image.
        # The mapper rectifies once into the same K before the unchanged VIO.
        import cv2
        fx, fy, cx, cy = intr
        K = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
        w, h = int(cam["width"]), int(cam["height"])
        mx, my = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_32FC1)
        if (not np.isfinite(mx).all() or not np.isfinite(my).all()
                or mx.min() < -1e-3 or my.min() < -1e-3 or mx.max() > w - 1 + 1e-3 or my.max() > h - 1 + 1e-3):
            raise ValueError("Residual camera rectification leaves the source image; a fully valid pinhole field of view is required")
        dxdu = np.diff(mx, axis=1)[:-1]
        dxdv = np.diff(mx, axis=0)[:, :-1]
        dydu = np.diff(my, axis=1)[:-1]
        dydv = np.diff(my, axis=0)[:, :-1]
        if np.any(dxdu * dydv - dxdv * dydu <= 0):
            raise ValueError("Residual Brown model folds inside the image; calibration rejected")
    out.setdefault("time_offsets", {})["camera"] = profiles[name]["time_offset"]
    out["time_offsets"]["camera_source"] = profiles[name]["time_source"]
    sync_camera_profile(out)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input", required=True)
    ap.add_argument("--camera", choices=tuple(CAMERAS))
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    result = select_camera(json.loads(Path(a.input).read_text()), a.camera)
    Path(a.output).write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"camera": selected_camera(result), "topic": result["camera"]["topic"],
                      "stream_topic": result["camera"]["stream_topic"], "output": a.output}))


if __name__ == "__main__":
    main()
