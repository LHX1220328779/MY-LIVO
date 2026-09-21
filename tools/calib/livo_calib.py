#!/usr/bin/env python3
"""MY-LIVO calibration workbench (offline, versioned).

One canonical calibration file per truck and version lives under
config/carlibration/<mine>/<truck>/<version>/calibration.json. It is the schema-5 file the
SLAM already reads (lidar->rear-axle matrices, camera->rear-axle transform) extended with the
camera model and the per-truck time offsets, so a truck change never requires editing the
algorithm yaml. Every stage creates a NEW version from a parent version and never modifies
inputs. Stages:

  import        normalise a supplier/factory perception JSON into version v001_import
  ground        static-start roll/pitch/z refinement of every LiDAR (scripts/refine_multi_lidar_extrinsics.py)
  driven        driven-window 6-DoF refinement (scripts/refine_multi_lidar_yaw.py); needs a LIO run
  camera-time   camera->IMU time offset from optical flow vs gyro
  camera-lidar  camera->rear-axle extrinsic refinement by LiDAR depth-edge / image-edge alignment
  from-precheck adopt the corrections reported by the SLAM static-start self-check
  list/show/activate/export/set  bookkeeping

Run inside the calibration container with the Ubuntu system python (ROS 2 Humble):
  /usr/bin/python3 tools/calib/livo_calib.py list
"""
import os
import sys

if sys.executable != "/usr/bin/python3":
    env = os.environ.copy()
    for v in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME", "PYTHONPATH"):
        env.pop(v, None)
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    env["AMENT_PREFIX_PATH"] = "/opt/ros/humble"
    env["PYTHONPATH"] = ("/opt/ros/humble/lib/python3.10/site-packages:"
                         "/opt/ros/humble/local/lib/python3.10/dist-packages")
    os.execve("/usr/bin/python3", ["/usr/bin/python3", *sys.argv], env)

import argparse
import copy
import datetime as dt
import json
import pathlib
import re
import shutil
import subprocess

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
CAL_ROOT = REPO / "config" / "carlibration"
SCRIPTS = REPO / "scripts"
LIDAR_KEYS = {"front_right_lidar": "transform_matrix_lidar_1", "back_lidar": "transform_matrix_lidar_2",
              "front_left_lidar": "transform_matrix_lidar_3", "front_lidar": "transform_matrix_lidar_6"}
DEFAULT_CAMERA = {"topic": "/midrange_camera/image_raw", "width": 1920, "height": 1080,
                  "intrinsics": [1642.570556640625, 1855.53173828125, 971.0083025529602, 534.2602337509306],
                  "distortion": [0.0, 0.0, 0.0, 0.0, 0.0]}
DEFAULT_TIME_OFFSETS = {"lidar": -0.125, "camera": 0.0, "rtk_position": 0.025, "ins_attitude": 0.0}


# ----------------------------------------------------------------------------- bookkeeping
def now():
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def load_registry():
    p = CAL_ROOT / "registry.json"
    return json.loads(p.read_text()) if p.exists() else {"active": {}}


def save_registry(reg):
    CAL_ROOT.mkdir(parents=True, exist_ok=True)
    reg["updated"] = now()
    (CAL_ROOT / "registry.json").write_text(json.dumps(reg, indent=2, ensure_ascii=False) + "\n")


def version_dir(vid):
    parts = vid.strip("/").split("/")
    if len(parts) != 3:
        sys.exit(f"version id must be <mine>/<truck>/<version>, got {vid}")
    d = CAL_ROOT.joinpath(*parts)
    if not (d / "calibration.json").exists():
        sys.exit(f"no calibration.json in {d}")
    return d


CREATED_DIRS = []


def next_version_dir(mine, truck, stage):
    base = CAL_ROOT / mine / truck
    base.mkdir(parents=True, exist_ok=True)
    numbers = [int(m.group(1)) for p in base.iterdir() for m in [re.match(r"v(\d{3})_", p.name)]
               if m and (p / "calibration.json").exists()]
    out = base / f"v{(max(numbers) + 1) if numbers else 1:03d}_{stage}"
    if out.exists():
        shutil.rmtree(out)  # leftovers of a failed attempt
    CREATED_DIRS.append(out)
    return out


def all_versions():
    out = []
    if not CAL_ROOT.exists():
        return out
    for meta in sorted(CAL_ROOT.glob("*/*/*/meta.json")):
        try:
            out.append(json.loads(meta.read_text()))
        except Exception as e:  # noqa: BLE001
            out.append({"id": str(meta.parent.relative_to(CAL_ROOT)), "error": str(e)})
    return out


def rpy_deg(m):
    from scipy.spatial.transform import Rotation as R
    return R.from_matrix(np.asarray(m)[:3, :3]).as_euler("xyz", degrees=True).round(3).tolist()


def summarize(calib):
    rows = []
    for name, key in LIDAR_KEYS.items():
        for e in calib.get("lidar", []):
            if key in e:
                m = np.asarray(e[key], float).reshape(4, 4)
                rows.append({"lidar": name, "key": key, "t": m[:3, 3].round(4).tolist(), "rpy_deg": rpy_deg(m)})
    cam = calib.get("transforms", {}).get("midrange_camera_to_rear_axle", {}).get("transform_matrix")
    out = {"lidars": rows}
    if cam:
        m = np.asarray(cam, float).reshape(4, 4)
        out["camera_to_rear"] = {"t": m[:3, 3].round(4).tolist(), "rpy_deg": rpy_deg(m)}
    out["camera"] = calib.get("camera")
    out["time_offsets"] = calib.get("time_offsets")
    return out


def finalize_version(out_dir, calib, *, mine, truck, stage, parent, note, inputs=None, metrics=None,
                     report_lines=None, activate=False):
    out_dir.mkdir(parents=True, exist_ok=True)
    vid = f"{mine}/{truck}/{out_dir.name}"
    calib["schema_version"] = 6
    calib["identity"] = {"id": vid, "mine": mine, "truck": truck, "version": out_dir.name, "stage": stage,
                         "parent": parent, "created": now()}
    calib.setdefault("history", []).append({"stage": stage, "created": now(), "parent": parent, "note": note,
                                            "inputs": inputs or {}, "metrics": metrics or {}})
    (out_dir / "calibration.json").write_text(json.dumps(calib, indent=2, ensure_ascii=False) + "\n")
    meta = {"id": vid, "mine": mine, "truck": truck, "version": out_dir.name, "stage": stage, "parent": parent,
            "created": now(), "note": note, "inputs": inputs or {}, "metrics": metrics or {},
            "summary": summarize(calib), "calibration_file": str(out_dir / "calibration.json")}
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    lines = [f"# {vid}", "", f"- stage: {stage}", f"- parent: {parent}", f"- created: {meta['created']}",
             f"- note: {note}", ""]
    if inputs:
        lines += ["## inputs", ""] + [f"- {k}: {v}" for k, v in inputs.items()] + [""]
    if metrics:
        lines += ["## metrics", "", "```", json.dumps(metrics, indent=2, ensure_ascii=False), "```", ""]
    lines += ["## summary", "", "```", json.dumps(meta["summary"], indent=2, ensure_ascii=False), "```", ""]
    if report_lines:
        lines += ["## stage report", "", "```"] + list(report_lines) + ["```", ""]
    (out_dir / "report.md").write_text("\n".join(lines))
    reg = load_registry()
    if activate or f"{mine}/{truck}" not in reg["active"]:
        reg["active"][f"{mine}/{truck}"] = vid
    save_registry(reg)
    print(json.dumps({"id": vid, "dir": str(out_dir), "metrics": metrics or {}}, ensure_ascii=False))
    return vid


def parent_info(vid):
    d = version_dir(vid)
    calib = json.loads((d / "calibration.json").read_text())
    ident = calib.get("identity", {})
    return d, calib, ident.get("mine") or vid.split("/")[0], ident.get("truck") or vid.split("/")[1]


# ----------------------------------------------------------------------------- import
def orthonormalize(m):
    u, _, vt = np.linalg.svd(m[:3, :3])
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = m[:3, 3]
    return out


def cmd_import(a):
    src = pathlib.Path(a.source).resolve()
    calib = json.loads(src.read_text())
    if "lidar" not in calib or not calib["lidar"]:
        sys.exit("source file has no lidar[] section (the SLAM needs schema-5 style lidar matrices)")
    notes = []
    for e in calib["lidar"]:
        for key in list(e.keys()):
            if key.startswith("transform_matrix_lidar_"):
                m = np.asarray(e[key], float)
                if m.size != 16:
                    sys.exit(f"{key} has {m.size} values, expected 16")
                m = m.reshape(4, 4)
                err = float(np.abs(m[:3, :3].T @ m[:3, :3] - np.eye(3)).max())
                if err > 1e-6:
                    e[key] = orthonormalize(m).reshape(-1).tolist()
                    notes.append(f"{key}: re-orthonormalised (max deviation {err:.1e})")
    tr = calib.setdefault("transforms", {})
    cam_tf = tr.get("midrange_camera_to_rear_axle", {}).get("transform_matrix")
    if cam_tf is None:
        sys.exit("source file lacks transforms.midrange_camera_to_rear_axle.transform_matrix")
    m = np.asarray(cam_tf, float)
    if m.size == 18:  # legacy export with two leading padding values
        m = m[2:]
        notes.append("camera transform: dropped two leading padding values (18 -> 16)")
    m = m.reshape(4, 4)
    err = float(np.abs(m[:3, :3].T @ m[:3, :3] - np.eye(3)).max())
    if err > 1e-6:
        m = orthonormalize(m)
        notes.append(f"camera transform re-orthonormalised (max deviation {err:.1e})")
    tr["midrange_camera_to_rear_axle"]["transform_matrix"] = m.reshape(-1).tolist()
    calib["lidar_transform_semantics"] = calib.get("lidar_transform_semantics", "lidar_to_body_rear_axle")

    camera = copy.deepcopy(DEFAULT_CAMERA)
    source = "defaults (wuhu truck29 yaml)"
    mc = calib.get("midrange_camera")
    if isinstance(mc, list) and mc and isinstance(mc[0], dict):
        p = mc[0].get("camera_in_params")
        if p and len(p) >= 12:
            camera["intrinsics"] = [float(p[0]), float(p[5]), float(p[2]), float(p[6])]
            source = "midrange_camera[0].camera_in_params"
        if mc[0].get("distcoeff"):
            camera["distortion"] = [float(v) for v in mc[0]["distcoeff"]][:5] + [0.0] * (5 - len(mc[0]["distcoeff"]))
        camera["width"] = int(mc[0].get("image_width", camera["width"]))
        camera["height"] = int(mc[0].get("image_height", camera["height"]))
    if a.intrinsics:
        camera["intrinsics"] = [float(v) for v in a.intrinsics]
        source = "command line"
    if a.distortion:
        camera["distortion"] = [float(v) for v in a.distortion]
    camera["source"] = source
    calib["camera"] = camera
    offsets = dict(DEFAULT_TIME_OFFSETS)
    if a.lidar_offset is not None:
        offsets["lidar"] = a.lidar_offset
    if a.camera_offset is not None:
        offsets["camera"] = a.camera_offset
    offsets["source"] = "import (verify with camera-time / LIO evaluation)"
    calib["time_offsets"] = offsets
    calib["calibration_type"] = calib.get("calibration_type", "") or "multi_lidar_to_rear_axle_manual"
    out_dir = next_version_dir(a.mine, a.truck, "import")
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "source.json").write_text(src.read_text())
    finalize_version(out_dir, calib, mine=a.mine, truck=a.truck, stage="import", parent=None,
                     note=a.note or f"imported from {src}", inputs={"source": str(src)},
                     metrics={"normalisation": notes}, activate=a.activate)


# ----------------------------------------------------------------------------- lidar stages
def run_script(cmd, log_path):
    print("+", " ".join(cmd), flush=True)
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=REPO)
        lines = []
        for line in proc.stdout:
            print(line, end="", flush=True)
            log.write(line)
            lines.append(line.rstrip("\n"))
        proc.wait()
    if proc.returncode != 0:
        sys.exit(f"stage failed with exit code {proc.returncode}; see {log_path}")
    return lines


def cmd_ground(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    out_dir = next_version_dir(mine, truck, "ground")
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_multi_lidar_extrinsics.py"), "--calibration",
           str(pdir / "calibration.json"), "--bag", a.bag, "--output", str(out_dir / "calibration.json"),
           "--report", str(out_dir / "stage_report.txt"), "--stationary", str(a.stationary[0]), str(a.stationary[1]),
           "--moving", str(a.moving[0]), str(a.moving[1])]
    lines = run_script(cmd, out_dir / "stage.log")
    calib = json.loads((out_dir / "calibration.json").read_text())
    metrics = {"corrections": calib.get("refinement", {}).get("corrections", {})}
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="ground", parent=a.id,
                     note=a.note or "static-start ground roll/pitch/z refinement",
                     inputs={"bag": a.bag, "stationary": list(a.stationary), "moving": list(a.moving)},
                     metrics=metrics, report_lines=lines, activate=a.activate)


def cmd_driven(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    out_dir = next_version_dir(mine, truck, "driven")
    out_dir.mkdir(parents=True, exist_ok=True)
    lidar_offset = parent_calib.get("time_offsets", {}).get("lidar", -0.125)
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_multi_lidar_yaw.py"), "--calibration",
           str(pdir / "calibration.json"), "--bag", a.bag, "--run", a.run, "--window", str(a.window[0]),
           str(a.window[1]), "--lidar-time-offset", str(lidar_offset), "--output",
           str(out_dir / "calibration.json"), "--report", str(out_dir / "stage_report.txt")]
    if a.vertical_only:
        cmd.append("--vertical-only")
    lines = run_script(cmd, out_dir / "stage.log")
    calib = json.loads((out_dir / "calibration.json").read_text())
    metrics = {k: v for k, v in calib.items() if k.startswith("refinement")}
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="driven", parent=a.id,
                     note=a.note or "driven-window 6-DoF refinement", inputs={"bag": a.bag, "run": a.run,
                     "window": list(a.window)}, metrics={"refinement": metrics}, report_lines=lines,
                     activate=a.activate)


def cmd_from_precheck(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    report = pathlib.Path(a.run_dir) / "Log" / "extrinsic_precheck.json"
    if not report.exists():
        sys.exit(f"missing {report}")
    pre = json.loads(report.read_text())
    calib = copy.deepcopy(parent_calib)
    topic_to_key = {f"/{k}": v for k, v in LIDAR_KEYS.items()}
    applied = {}
    for s in pre["sources"]:
        if not s.get("solved") or not s.get("within_gates"):
            continue
        key = topic_to_key.get(s["topic"])
        for e in calib["lidar"]:
            if key in e:
                e[key] = [float(v) for v in s["rear_from_lidar_after"]]
                applied[s["topic"]] = {"rot_x_deg": s["rot_x_deg"], "rot_y_deg": s["rot_y_deg"], "dz_cm": s["dz_cm"],
                                       "median_before_cm": s["median_before_cm"], "median_after_cm": s["median_after_cm"]}
    if not applied:
        sys.exit("precheck report contains no in-gate corrections")
    calib["calibration_type"] = calib.get("calibration_type", "") + "+static_precheck_roll_pitch_z"
    out_dir = next_version_dir(mine, truck, "precheck")
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="precheck", parent=a.id,
                     note=a.note or f"adopted SLAM static-start self-check from {a.run_dir}",
                     inputs={"run_dir": a.run_dir, "report": str(report)}, metrics={"applied": applied},
                     activate=a.activate)


# ----------------------------------------------------------------------------- camera time offset
def read_bag_topics(bag, wanted):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [w.strip("/") for w in wanted]
    topics = [t for t in types if any(t.strip("/") == w or t.strip("/").endswith("/" + w) for w in wanted)]
    if not topics:
        sys.exit(f"bag {bag} has none of the topics {wanted}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    msgs = {t: get_message(types[t]) for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in msgs:
            yield topic, deserialize_message(data, msgs[topic])


def decode_hevc(stream_path, scale, keep=None):
    """Decode an Annex-B HEVC stream with ffmpeg; yields (index, gray frame)."""
    import cv2  # noqa: F401
    w, h = int(1920 * scale), int(1080 * scale)
    cmd = ["ffmpeg", "-loglevel", "error", "-f", "hevc", "-i", str(stream_path), "-vf", f"scale={w}:{h}",
           "-pix_fmt", "gray", "-f", "rawvideo", "-"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10 ** 7)
    n = w * h
    k = 0
    while True:
        buf = proc.stdout.read(n)
        if len(buf) < n:
            break
        if keep is None or k in keep:
            yield k, np.frombuffer(buf, np.uint8).reshape(h, w)
        k += 1
    proc.wait()


def dump_camera_stream(bag, out_dir):
    stamps = []
    path = out_dir / "stream.h265"
    with open(path, "wb") as f:
        for topic, m in read_bag_topics(bag, ["midrange_camera/ffmpeg"]):
            stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            f.write(bytes(m.data))
    return np.array(stamps), path


def read_imu(bag):
    t, g = [], []
    for topic, m in read_bag_topics(bag, ["imu_data"]):
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    return np.array(t), np.array(g)


def estimate_camera_offset(bag, out_dir, fx, fy, scale=0.25):
    import cv2
    stamps, stream = dump_camera_stream(bag, out_dir)
    ti, gyro = read_imu(bag)
    if np.abs(gyro).max() > 20:  # deg/s stream
        gyro = np.radians(gyro)
    frames = [f for _, f in decode_hevc(stream, scale)]
    n = min(len(frames), len(stamps))
    frames, stamps = frames[:n], stamps[:n]
    h, w = frames[0].shape
    feat = dict(maxCorners=400, qualityLevel=0.01, minDistance=6, blockSize=5)
    lk = dict(winSize=(15, 15), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
    wx = np.full(n, np.nan)
    wy = np.full(n, np.nan)
    for k in range(1, n):
        p0 = cv2.goodFeaturesToTrack(frames[k - 1], mask=None, **feat)
        if p0 is None or len(p0) < 20:
            continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(frames[k - 1], frames[k], p0, None, **lk)
        ok = st.reshape(-1) == 1
        if ok.sum() < 20:
            continue
        a_, b_ = p0.reshape(-1, 2)[ok], p1.reshape(-1, 2)[ok]
        sel = a_[:, 1] < h * 0.6
        if sel.sum() < 15:
            sel = np.ones(len(a_), bool)
        d = b_[sel] - a_[sel]
        wx[k], wy[k] = np.median(d[:, 0]), np.median(d[:, 1])
    dt_img = float(np.median(np.diff(stamps)))
    yaw = wx / (fx * scale) / dt_img
    pitch = wy / (fy * scale) / dt_img
    t_img = stamps - dt_img / 2
    gz_s = np.convolve(gyro[:, 2], np.ones(3) / 3, mode="same")
    gx_s = np.convolve(gyro[:, 0], np.ones(3) / 3, mode="same")
    lags = np.arange(-0.4, 0.4001, 0.001)

    def ls_lag(tt, s, ref):
        m = np.isfinite(s)
        tt, s = tt[m], s[m]
        if len(tt) < 200:
            return None
        curve = []
        for lag in lags:
            g = np.interp(tt + lag, ti, ref)
            A = np.vstack([g, np.ones_like(g)]).T
            coef = np.linalg.lstsq(A, s, rcond=None)[0]
            r = s - A @ coef
            thr = np.percentile(np.abs(r), 95)
            curve.append(np.mean(r[np.abs(r) < thr] ** 2))
        curve = np.array(curve)
        i = int(np.argmin(curve))
        frac = 0.0
        if 0 < i < len(curve) - 1:
            y0, y1, y2 = curve[i - 1], curve[i], curve[i + 1]
            den = y0 - 2 * y1 + y2
            frac = 0.5 * (y0 - y2) / den if den > 0 else 0.0
        best = float(lags[i] + frac * (lags[1] - lags[0]))
        g = np.interp(tt + best, ti, ref)
        corr = float(np.corrcoef(s, g)[0, 1])
        good = np.abs(g) > 0.02
        gain = float(np.median(s[good] / g[good])) if good.sum() > 50 else float("nan")
        return {"lag_s": best, "corr": corr, "gain": gain, "n": int(len(tt)),
                "sse_ratio_zero_lag": float(curve[np.argmin(np.abs(lags))] / curve.min())}

    result = {"yaw": ls_lag(t_img, yaw, gz_s), "pitch": ls_lag(t_img, pitch, gx_s), "segments": []}
    t0 = t_img[0]
    for a0 in range(0, int(t_img[-1] - t0), 30):
        sel = (t_img - t0 >= a0) & (t_img - t0 < a0 + 30)
        if sel.sum() < 300:
            continue
        g = np.interp(t_img[sel], ti, gz_s)
        if np.std(g) < 0.01:
            result["segments"].append({"start_s": a0, "note": "insufficient yaw excitation"})
            continue
        r = ls_lag(t_img[sel], yaw[sel], gz_s)
        result["segments"].append({"start_s": a0, "lag_s": r["lag_s"] if r else None, "corr": r["corr"] if r else None})
    lag = result["yaw"]["lag_s"] if result["yaw"] else None
    spread = [s["lag_s"] for s in result["segments"] if s.get("lag_s") is not None]
    result["camera_offset_s"] = lag
    result["segment_spread_ms"] = float((max(spread) - min(spread)) * 1000) if len(spread) > 1 else None
    result["implied_fx_from_gain"] = float(abs(result["yaw"]["gain"]) * fx) if result["yaw"] else None
    np.savez(out_dir / "flow_series.npz", stamps=stamps, wx=wx, wy=wy)
    stream.unlink(missing_ok=True)
    return result


def cmd_camera_time(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    out_dir = next_version_dir(mine, truck, "camtime")
    out_dir.mkdir(parents=True, exist_ok=True)
    fx, fy = parent_calib.get("camera", DEFAULT_CAMERA)["intrinsics"][:2]
    result = estimate_camera_offset(pathlib.Path(a.bag), out_dir, fx, fy)
    if result["camera_offset_s"] is None:
        sys.exit("camera time offset could not be estimated (no usable flow)")
    calib = copy.deepcopy(parent_calib)
    offsets = calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))
    offsets["camera"] = round(result["camera_offset_s"], 4)
    offsets["camera_source"] = f"optical flow vs gyro on {a.bag} (corr {result['yaw']['corr']:.2f})"
    calib["calibration_type"] = calib.get("calibration_type", "") + "+camera_time_offset"
    (out_dir / "camera_time.json").write_text(json.dumps(result, indent=2) + "\n")
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="camtime", parent=a.id,
                     note=a.note or "camera->IMU time offset from optical flow vs gyro", inputs={"bag": a.bag},
                     metrics={"camera_offset_s": offsets["camera"], "yaw": result["yaw"], "pitch": result["pitch"],
                              "segments": result["segments"], "implied_fx_from_gain": result["implied_fx_from_gain"]},
                     activate=a.activate)


# ----------------------------------------------------------------------------- camera-LiDAR
def read_run_frames(run):
    run = pathlib.Path(run)
    lines = (run / "frames.csv").read_text().splitlines()
    t_li = np.array([float(v) for v in lines[1].split()[2:]])
    header = lines[3].split(",")
    rows = np.array([[float(v) for v in l.split(",")] for l in lines[4:] if l and not l.startswith("#")])
    col = {n: i for i, n in enumerate(header)}
    return {"stamp": rows[:, col["stamp"]], "offset": rows[:, col["offset_bytes"]].astype(np.int64),
            "count": rows[:, col["count"]].astype(np.int64),
            "pos": rows[:, [col["tx"], col["ty"], col["tz"]]],
            "quat": rows[:, [col["qx"], col["qy"], col["qz"], col["qw"]]], "t_li": t_li,
            "archive": np.memmap(run / "scans.bin", dtype="<f4", mode="r", offset=8).reshape(-1, 5)}


def camera_lidar_refine(calib, bag, run, out_dir, n_frames=12, window=(5.0, 60.0), accumulate=1.0,
                        scale=0.5, max_rot_deg=2.0, max_trans_m=0.15, solve_intrinsics=False):
    import cv2
    from scipy.optimize import minimize
    from scipy.spatial.transform import Rotation as R
    fr = read_run_frames(run)
    stamps, rot = fr["stamp"], R.from_quat(fr["quat"])
    speed = np.linalg.norm(np.gradient(fr["pos"], stamps, axis=0), axis=1)
    cam = calib.get("camera", DEFAULT_CAMERA)
    fx, fy, cx, cy = cam["intrinsics"]
    K = np.array([[fx * scale, 0, cx * scale], [0, fy * scale, cy * scale], [0, 0, 1]])
    dist = np.array(cam.get("distortion", [0] * 5), float)
    T_rear_cam = np.asarray(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    cam_offset = calib.get("time_offsets", {}).get("camera", 0.0)
    img_stamps, stream = dump_camera_stream(bag, out_dir)
    eff = img_stamps + cam_offset
    t0 = stamps[0]
    targets = []
    for tq in np.linspace(t0 + window[0], min(t0 + window[1], stamps[-1] - 1), n_frames):
        j = int(np.argmin(np.abs(stamps - tq)))
        if speed[j] > 8.0:
            continue
        k = int(np.argmin(np.abs(eff - stamps[j])))
        if abs(eff[k] - stamps[j]) > 0.02:
            continue
        targets.append((j, k))
    if len(targets) < 4:
        sys.exit("not enough usable image/scan pairs (need a LIVO run whose frames end at image times)")
    images = {k: img for k, img in decode_hevc(stream, scale, keep={k for _, k in targets})}
    stream.unlink(missing_ok=True)
    h, w = next(iter(images.values())).shape

    def T_world_imu(j):
        T = np.eye(4)
        T[:3, :3] = rot[j].as_matrix()
        T[:3, 3] = fr["pos"][j]
        return T

    T_imu_rear = np.eye(4)
    T_imu_rear[:3, 3] = fr["t_li"]
    frames_data = []
    for j, k in targets:
        sel = np.flatnonzero(np.abs(stamps - stamps[j]) <= accumulate)
        pts_world = []
        for i in sel:
            o, c = (fr["offset"][i] - 8) // 20, fr["count"][i]
            p = np.asarray(fr["archive"][o:o + c, :3], np.float64)
            p = p[np.isfinite(p).all(1)]
            if len(p) > 20000:
                p = p[:: max(1, len(p) // 20000)]
            Tw = T_world_imu(i) @ T_imu_rear
            pts_world.append(p @ Tw[:3, :3].T + Tw[:3, 3])
        pw = np.vstack(pts_world)
        T_rear_world = np.linalg.inv(T_world_imu(j) @ T_imu_rear)
        p_rear = pw @ T_rear_world[:3, :3].T + T_rear_world[:3, 3]
        # depth edges seen from the camera with the current extrinsic
        T_cam_rear = np.linalg.inv(T_rear_cam)
        pc = p_rear @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]
        front = pc[:, 2] > 1.0
        pc, pr = pc[front], p_rear[front]
        uv = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        inside = (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1)
        u, v, depth, pr = u[inside], v[inside], pc[inside, 2], pr[inside]
        order = np.argsort(-depth)  # nearest written last
        zbuf = np.full((h, w), np.inf)
        idx = np.full((h, w), -1, np.int64)
        zbuf[v[order], u[order]] = depth[order]
        idx[v[order], u[order]] = order
        edge = np.zeros((h, w), bool)
        for dy, dx in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            nb = np.roll(np.roll(zbuf, dy, 0), dx, 1)
            edge |= (np.isfinite(zbuf) & np.isfinite(nb) & (nb - zbuf > np.maximum(0.6, 0.05 * zbuf)))
        e_idx = idx[edge]
        e_idx = e_idx[e_idx >= 0]
        edge_pts = pr[e_idx]
        img = images[k]
        canny = cv2.Canny(cv2.GaussianBlur(img, (3, 3), 0), 60, 160)
        # keep only long edge chains: foliage/gravel texture produces short fragments that make
        # the distance field flat and uninformative
        n_cc, labels, stats, _ = cv2.connectedComponentsWithStats((canny > 0).astype(np.uint8), connectivity=8)
        long_cc = np.zeros(n_cc, bool)
        long_cc[1:] = np.maximum(stats[1:, cv2.CC_STAT_WIDTH], stats[1:, cv2.CC_STAT_HEIGHT]) >= 25
        canny = np.where(long_cc[labels], 255, 0).astype(np.uint8)
        dt_img = cv2.distanceTransform(255 - canny, cv2.DIST_L2, 3)
        dt_img = np.minimum(dt_img, 10.0)
        frames_data.append({"j": j, "k": k, "edge_pts": edge_pts, "dt": dt_img, "img": img, "canny": canny,
                            "speed": float(speed[j])})

    def score(delta, T_base=T_rear_cam, return_parts=False):
        rv, tv = np.asarray(delta[:3]), np.asarray(delta[3:6])
        Kd = K.copy()
        if len(delta) >= 8:  # focal scale factors
            Kd[0, 0] *= delta[6]
            Kd[1, 1] *= delta[7]
        P = np.eye(4)
        P[:3, :3] = R.from_rotvec(np.radians(rv)).as_matrix()
        P[:3, 3] = tv
        T = T_base @ P  # perturbation expressed in the camera frame
        T_cam_rear = np.linalg.inv(T)
        total, count, parts = 0.0, 0, []
        for f in frames_data:
            pc = f["edge_pts"] @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]
            pc = pc[pc[:, 2] > 1.0]
            if len(pc) == 0:
                parts.append(10.0)
                continue
            uv = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Kd, dist)[0].reshape(-1, 2)
            u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
            inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            vals = f["dt"][v[inside], u[inside]] if inside.any() else np.array([10.0])
            s = float(vals.mean()) + 10.0 * (1 - inside.mean())
            parts.append(s)
            total += s
            count += 1
        s = total / max(count, 1)
        pen = np.sum(np.maximum(0, np.abs(rv) - max_rot_deg) ** 2) * 10 + np.sum(np.maximum(0, np.abs(tv) - max_trans_m) ** 2) * 100
        return (s + pen, parts) if return_parts else s + pen

    before, before_parts = score(np.zeros(6), return_parts=True)
    # coarse rotation grid, then local 6-DoF refinement
    best, best_x = before, np.zeros(6)
    grid = np.arange(-1.0, 1.001, 0.1)
    for ry in grid:      # yaw-like about camera y
        for rx in grid:  # pitch-like about camera x
            s = score([rx, ry, 0, 0, 0, 0])
            if s < best:
                best, best_x = s, np.array([rx, ry, 0, 0, 0, 0.0])
    for rz in np.arange(-1.0, 1.001, 0.1):
        x = best_x.copy()
        x[2] = rz
        s = score(x)
        if s < best:
            best, best_x = s, x
    res = minimize(score, best_x, method="Nelder-Mead",
                   options={"xatol": 1e-3, "fatol": 1e-4, "maxiter": 1500, "initial_simplex": None})
    x = res.x if res.fun < best else best_x
    after, after_parts = score(x, return_parts=True)
    # focal-length scan: for each fx scale (fy fixed) re-optimise the rotation and record the score
    focal_scan = []
    for sf in (0.92, 0.96, 1.0, 1.04, 1.08, 1.12, 1.16):
        bx, bs = None, np.inf
        for ry in np.arange(-0.6, 0.601, 0.1):
            for rx in np.arange(-0.6, 0.601, 0.1):
                xx = np.array([x[0] + rx, x[1] + ry, x[2], x[3], x[4], x[5], sf, 1.0])
                sc = score(xx)
                if sc < bs:
                    bs, bx = sc, xx
        focal_scan.append({"fx_scale": sf, "fx": round(float(fx * sf), 1), "score_px": round(float(bs), 4),
                           "rot_deg": np.round(bx[:3], 3).tolist()})
    # focal-length hypothesis: re-optimise with fx, fy scale factors as free parameters
    x8 = np.concatenate([x, [1.0, 1.0]])
    res8 = minimize(score, x8, method="Nelder-Mead", options={"xatol": 1e-3, "fatol": 1e-4, "maxiter": 3000})
    intr = {"fx": round(float(fx * res8.x[6]), 1), "fy": round(float(fy * res8.x[7]), 1),
            "score_px": round(float(res8.fun), 4), "delta_rot_deg_cam_xyz": np.round(res8.x[:3], 4).tolist(),
            "delta_trans_m_cam_xyz": np.round(res8.x[3:6], 4).tolist()}
    if solve_intrinsics and res8.fun < after:
        x = res8.x
        after, after_parts = score(x, return_parts=True)
        K[0, 0] *= x[6]
        K[1, 1] *= x[7]
    P = np.eye(4)
    P[:3, :3] = R.from_rotvec(np.radians(x[:3])).as_matrix()
    P[:3, 3] = x[3:6]
    T_new = T_rear_cam @ P
    overlays = out_dir / "overlays"
    overlays.mkdir(exist_ok=True)
    for f in frames_data[:: max(1, len(frames_data) // 4)]:
        canvas = cv2.cvtColor(f["img"], cv2.COLOR_GRAY2BGR)
        canvas[f["canny"] > 0] = (0, 255, 255)
        for T, color in ((T_rear_cam, (0, 0, 255)), (T_new, (0, 255, 0))):
            Tc = np.linalg.inv(T)
            pc = f["edge_pts"] @ Tc[:3, :3].T + Tc[:3, 3]
            pc = pc[pc[:, 2] > 1.0]
            uv = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
            for uu, vv in uv:
                if 0 <= uu < w and 0 <= vv < h:
                    cv2.circle(canvas, (int(uu), int(vv)), 1, color, -1)
        cv2.imwrite(str(overlays / f"frame_{f['k']:05d}.jpg"), canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
    metrics = {"frames": len(frames_data), "edge_points": int(sum(len(f["edge_pts"]) for f in frames_data)),
               "score_before_px": round(before, 4), "score_after_px": round(after, 4),
               "delta_rot_deg_cam_xyz": np.round(x[:3], 4).tolist(), "delta_trans_m_cam_xyz": np.round(x[3:], 4).tolist(),
               "per_frame_before": np.round(before_parts, 3).tolist(), "per_frame_after": np.round(after_parts, 3).tolist(),
               "focal_hypothesis": intr, "focal_scan": focal_scan, "intrinsics_used": [round(float(K[0, 0] / scale), 2), round(float(K[1, 1] / scale), 2),
                                                             round(float(K[0, 2] / scale), 2), round(float(K[1, 2] / scale), 2)],
               "intrinsics_solved": bool(solve_intrinsics and res8.fun < after + 1e-9), "image_scale": scale,
               "frame_speeds_mps": [round(f["speed"], 2) for f in frames_data],
               "overlay_dir": str(overlays)}
    return T_new, metrics


def cmd_camera_lidar(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    out_dir = next_version_dir(mine, truck, "camlidar")
    out_dir.mkdir(parents=True, exist_ok=True)
    T_new, metrics = camera_lidar_refine(parent_calib, pathlib.Path(a.bag), a.run, out_dir, n_frames=a.frames,
                                         window=tuple(a.window), accumulate=a.accumulate,
                                         solve_intrinsics=a.solve_intrinsics)
    calib = copy.deepcopy(parent_calib)
    calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"] = T_new.reshape(-1).tolist()
    if metrics.get("intrinsics_solved"):
        calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))["intrinsics"] = metrics["intrinsics_used"]
        calib["camera"]["source"] = "livo_calib camera-lidar edge alignment (focal scale solved)"
    calib["transforms"]["midrange_camera_to_rear_axle"]["refined_by"] = "livo_calib camera-lidar depth-edge alignment"
    calib["calibration_type"] = calib.get("calibration_type", "") + "+camera_lidar_edge_refined"
    improved = metrics["score_after_px"] < metrics["score_before_px"]
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="camlidar", parent=a.id,
                     note=a.note or ("camera->rear extrinsic refined by depth-edge alignment" if improved else
                                     "camera-lidar stage ran but did NOT improve the edge score; inspect overlays"),
                     inputs={"bag": a.bag, "run": a.run, "frames": a.frames, "window": list(a.window)},
                     metrics=metrics, activate=a.activate and improved)


# ----------------------------------------------------------------------------- bookkeeping cmds
def cmd_list(a):
    reg = load_registry()
    rows = all_versions()
    if a.json:
        print(json.dumps({"active": reg.get("active", {}), "versions": rows}, indent=2, ensure_ascii=False))
        return
    for r in rows:
        act = "*" if reg.get("active", {}).get(f"{r.get('mine')}/{r.get('truck')}") == r.get("id") else " "
        print(f"{act} {r.get('id'):40s} {r.get('stage', ''):9s} {r.get('created', '')[:19]}  {r.get('note', '')[:60]}")


def cmd_show(a):
    d = version_dir(a.id)
    meta = json.loads((d / "meta.json").read_text()) if (d / "meta.json").exists() else {}
    print(json.dumps(meta, indent=2, ensure_ascii=False))


def cmd_activate(a):
    d = version_dir(a.id)
    reg = load_registry()
    mine, truck, _ = a.id.strip("/").split("/")
    reg["active"][f"{mine}/{truck}"] = a.id
    save_registry(reg)
    print(json.dumps({"active": reg["active"], "calibration_file": str(d / "calibration.json")}))


def cmd_export(a):
    d = version_dir(a.id)
    print(d / "calibration.json")


def cmd_set(a):
    """Create a child version with explicit camera/time-offset values (operator-entered)."""
    pdir, parent_calib, mine, truck = parent_info(a.id)
    calib = copy.deepcopy(parent_calib)
    changes = {}
    if a.camera_offset is not None:
        calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))["camera"] = a.camera_offset
        changes["time_offsets.camera"] = a.camera_offset
    if a.lidar_offset is not None:
        calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))["lidar"] = a.lidar_offset
        changes["time_offsets.lidar"] = a.lidar_offset
    if a.intrinsics:
        calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))["intrinsics"] = [float(v) for v in a.intrinsics]
        changes["camera.intrinsics"] = calib["camera"]["intrinsics"]
    if a.distortion:
        calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))["distortion"] = [float(v) for v in a.distortion]
        changes["camera.distortion"] = calib["camera"]["distortion"]
    if a.camera_rot_delta:
        from scipy.spatial.transform import Rotation as R
        tf = calib["transforms"]["midrange_camera_to_rear_axle"]
        T = np.asarray(tf["transform_matrix"], float).reshape(4, 4)
        P = np.eye(4)
        P[:3, :3] = R.from_euler("xyz", a.camera_rot_delta, degrees=True).as_matrix()
        tf["transform_matrix"] = (T @ P).reshape(-1).tolist()  # delta expressed in the camera frame
        changes["camera_rot_delta_deg_cam_xyz"] = list(a.camera_rot_delta)
    if not changes:
        sys.exit("nothing to set")
    out_dir = next_version_dir(mine, truck, "manual")
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="manual", parent=a.id,
                     note=a.note or "operator-entered values", metrics={"changes": changes}, activate=a.activate)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p, need_id=True):
        if need_id:
            p.add_argument("--id", required=True, help="parent version id <mine>/<truck>/<version>")
        p.add_argument("--note", default="")
        p.add_argument("--activate", action="store_true", help="make the new version active")

    p = sub.add_parser("import")
    p.add_argument("--source", required=True)
    p.add_argument("--mine", required=True)
    p.add_argument("--truck", required=True)
    p.add_argument("--intrinsics", nargs=4, type=float)
    p.add_argument("--distortion", nargs=5, type=float)
    p.add_argument("--lidar-offset", type=float)
    p.add_argument("--camera-offset", type=float)
    common(p, need_id=False)
    p.set_defaults(func=cmd_import)

    p = sub.add_parser("ground")
    p.add_argument("--bag", required=True)
    p.add_argument("--stationary", nargs=2, type=int, default=(2, 20))
    p.add_argument("--moving", nargs=2, type=int, default=(300, 40))
    common(p)
    p.set_defaults(func=cmd_ground)

    p = sub.add_parser("driven")
    p.add_argument("--bag", required=True)
    p.add_argument("--run", required=True, help="fusion run directory (Log/fusion/<stamp>) of a LIO round")
    p.add_argument("--window", nargs=2, type=float, default=(8.0, 40.0))
    p.add_argument("--vertical-only", action="store_true")
    common(p)
    p.set_defaults(func=cmd_driven)

    p = sub.add_parser("camera-time")
    p.add_argument("--bag", required=True, help="prepared bag containing midrange_camera/ffmpeg and imu_data")
    common(p)
    p.set_defaults(func=cmd_camera_time)

    p = sub.add_parser("camera-lidar")
    p.add_argument("--bag", required=True)
    p.add_argument("--run", required=True, help="fusion run directory of a LIVO round (frames end at image times)")
    p.add_argument("--frames", type=int, default=12)
    p.add_argument("--window", nargs=2, type=float, default=(5.0, 60.0))
    p.add_argument("--accumulate", type=float, default=1.0, help="seconds of scans accumulated around each image")
    p.add_argument("--solve-intrinsics", action="store_true",
                   help="also solve fx/fy scale factors and write them to camera.intrinsics when the score improves")
    common(p)
    p.set_defaults(func=cmd_camera_lidar)

    p = sub.add_parser("from-precheck")
    p.add_argument("--run-dir", required=True, help="results/<round> containing Log/extrinsic_precheck.json")
    common(p)
    p.set_defaults(func=cmd_from_precheck)

    p = sub.add_parser("set")
    p.add_argument("--camera-rot-delta", nargs=3, type=float, metavar=("RX", "RY", "RZ"),
                   help="rotate the camera extrinsic by these degrees (camera-frame xyz Euler); closed-loop tests")
    p.add_argument("--camera-offset", type=float)
    p.add_argument("--lidar-offset", type=float)
    p.add_argument("--intrinsics", nargs=4, type=float)
    p.add_argument("--distortion", nargs=5, type=float)
    common(p)
    p.set_defaults(func=cmd_set)

    p = sub.add_parser("list")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_list)
    p = sub.add_parser("show")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_show)
    p = sub.add_parser("activate")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_activate)
    p = sub.add_parser("export")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_export)
    a = ap.parse_args()
    try:
        a.func(a)
    finally:
        for d in CREATED_DIRS:  # a stage that failed must not leave a half-written version behind
            if d.exists() and not (d / "calibration.json").exists():
                shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    main()
