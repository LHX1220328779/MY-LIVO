#!/usr/bin/env python3
"""MY-LIVO calibration workbench (offline, versioned).

One canonical calibration file per truck and version lives under
config/carlibration/<mine>/<truck>/<version>/calibration.json. It is the schema-5 file the
SLAM already reads (lidar->rear-axle matrices, camera->rear-axle transform) extended with the
camera model and the per-truck time offsets, so a truck change never requires editing the
algorithm yaml. Every stage creates a NEW version from a parent version and never modifies
inputs. Stages (version suffix = stage; see STAGES for the operator-facing catalogue):

  import        normalise a supplier/factory perception JSON (schema-5 file or the raw supplier export with
                lidar[] + midrange_camera[0].tf_lidar_to_camera) into version v001_import; without explicit
                intrinsics the camera model is recovered from the supplier ground-distance curves (equation_*m)
  axle          reference-LiDAR -> rear-axle alignment from the recording's RTK/INS poses (rig correction for
                every LiDAR and the camera + LiDAR time shift; scripts/refine_rear_axle_extrinsic.py); no mapping run
  ground        static-start roll/pitch/z refinement of every LiDAR (scripts/refine_multi_lidar_extrinsics.py)
  driven        driven-window 6-DoF refinement (scripts/refine_multi_lidar_yaw.py); needs a LIO run
  gcons         rear-facing LiDAR tilt/z vs the multi-LiDAR ground consensus (scripts/refine_lidar_ground_consensus.py)
  camera-time   camera->IMU time offset from optical flow vs gyro
  camera-lidar  camera->rear-axle extrinsic refinement by LiDAR depth-edge / image-edge alignment
                (advisory: writes overlays/ images, activate only after inspecting them)
  from-precheck adopt the corrections reported by the SLAM static-start self-check
  windows       motion profile of a recording (INS odometry) -> stationary / driving windows for the stages and
                calibration rounds (a recording that starts with a long standstill must not be calibrated on it)
  list/show/validate/activate/export/set/delete  bookkeeping

Every --bag argument accepts the raw ROS 2 recording root (Perception_*/Camera_* segment folders)
or any single bag folder; nothing is converted or merged (tools/livo/dataset.py).

Run inside the calibration container with the Ubuntu system python (ROS 2 Humble):
  /usr/bin/python3 tools/calib/livo_calib.py list
"""
import os
import pathlib as _pathlib
import sys

sys.path.insert(0, str(_pathlib.Path(__file__).resolve().parents[1]))
try:
    from livo import rosenv as _rosenv
except ImportError:  # copied outside the repo
    sys.path.insert(0, "/home/project/MY-LIVO/tools")
    from livo import rosenv as _rosenv
_rosenv.ensure()

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
DEFAULT_TIME_OFFSETS = {"lidar": -0.125, "lidar_frontend": -0.125, "camera": 0.0,
                        "rtk_position": 0.025, "ins_attitude": 0.0}

# Operator-facing catalogue: the version suffix is the stage key. `mapping` tells which mapping
# run the stage needs beforehand; `advisory` stages must be inspected before activation.
STAGES = {
    "import": {"order": 1, "label": "导入出厂标定", "command": "import",
               "input": "厂家感知标定 JSON", "output": "LiDAR/相机外参基线 + 默认时间偏移",
               "needs": [], "mapping": None, "advisory": False, "seconds": 1},
    "axle": {"order": 2, "label": "后轴对齐（RTK/INS）", "command": "axle",
             "input": "原始录制（INS/RTK 位姿 + 前雷达，无需建图）",
             "output": "参考雷达→后轴 roll/pitch/yaw/x/y（整机刚体修正，所有雷达与相机同步）+ LiDAR 时间偏移",
             "needs": ["dataset"], "mapping": None, "advisory": False, "seconds": 90},
    "ground": {"order": 3, "label": "静止地面精修", "command": "ground",
               "input": "原始录制（起步前静止段）", "output": "每个 LiDAR 的 roll / pitch / z",
               "needs": ["dataset"], "mapping": None, "advisory": False, "seconds": 60},
    "driven": {"order": 4, "label": "行驶段 6-DoF 精修", "command": "driven",
               "input": "原始录制 + 一次 LIO 建图结果", "output": "四个 LiDAR 之间的 6-DoF 一致性",
               "needs": ["dataset", "run"], "mapping": "lio", "advisory": False, "seconds": 90},
    "gcons": {"order": 4.7, "label": "后向雷达地面一致性（tilt/z）", "command": "gcons",
              "input": "原始录制 + 一次 LIO 建图结果（当前外参，RTK 优化位姿；流水线取 RTK 关联时间轮次）",
              "output": "后向 LiDAR 相对全部雷达共识地面的 roll / pitch / z（无显著偏差则沿用父版本）",
              "needs": ["dataset", "run"], "mapping": "lio", "advisory": False, "seconds": 120},
    "rtktime": {"order": 4.5, "label": "RTK 位置关联时间", "command": "rtk-time",
                "input": "1–2 次 LIO 建图结果（流水线自动生成）",
                "output": "time_offsets.rtk_position（地图自洽：不同车速/方向下同一结构重合；两轮割线闭环）",
                "needs": ["run"], "mapping": "lio", "advisory": False, "seconds": 30},
    "camtime": {"order": 5, "label": "相机时间偏移", "command": "camera-time",
                "input": "原始录制（含相机）", "output": "time_offsets.camera（光流 vs 陀螺）",
                "needs": ["dataset", "camera"], "mapping": None, "advisory": False, "seconds": 120},
    "camlidar": {"order": 6, "label": "相机-LiDAR 偏航/横滚复核", "command": "camera-lidar",
                 "input": "原始录制（含相机）+ 一次 LIVO 建图结果", "output": "相机→后轴外参的偏航/横滚（竖直边缘对齐，俯仰不变；含 overlays/ 叠加图）",
                 "needs": ["dataset", "camera", "run"], "mapping": "livo", "advisory": True, "seconds": 180},
    "camfocus": {"order": 6, "label": "相机内参/畸变（光流×深度）", "command": "camera-focus",
                 "input": "原始录制（含相机）+ 一次 LIVO 建图结果", "output": "camera.intrinsics 的 fx/fy 与畸变 k1（连续帧光流 vs LiDAR 深度预测流；外参/时间偏移/卷帘一阶抵消）",
                 "needs": ["dataset", "camera", "run"], "mapping": "livo", "advisory": False, "seconds": 240},
    "camloop": {"order": 7, "label": "相机外参闭环标定", "command": "set",
                "input": "俯仰扫描的 2–3 次 LIVO 建图（原生里程尺度 vs RTK）", "output": "相机→后轴外参的俯仰角（尺度过零点，闭环验证）",
                "needs": [], "mapping": "livo", "advisory": False, "seconds": 900, "internal": True},
    "precheck": {"order": 8, "label": "采纳建图自检修正", "command": "from-precheck",
                 "input": "一次建图结果的 Log/extrinsic_precheck.json", "output": "自检门限内的 roll / pitch / z 修正",
                 "needs": ["run_dir"], "mapping": "any", "advisory": False, "seconds": 1},
    "manual": {"order": 9, "label": "手动设置", "command": "set",
               "input": "人工输入的时间偏移 / 内参 / 相机旋转修正", "output": "对应字段的新版本",
               "needs": [], "mapping": None, "advisory": False, "seconds": 1},
    "scan": {"order": 10, "label": "闭环扫描点（临时）", "command": "set",
             "input": "流水线自动生成：父版本 + 固定俯仰增量", "output": "仅用于一次扫描建图，不应激活",
             "needs": [], "mapping": None, "advisory": True, "seconds": 1, "internal": True},
}


def require_dataset(bag, *, camera=False, what="stage"):
    """Open a recording (raw split root or single bag) and check the topics the stage needs."""
    from livo.dataset import Dataset
    try:
        ds = Dataset.open(bag)
    except Exception as exc:  # noqa: BLE001
        sys.exit(f"{what}: recording not readable: {bag}: {exc}")
    info = ds.summary()
    if not info.get("lio_ready"):
        sys.exit(f"{what}: recording {bag} lacks LiDAR/IMU topics (has {sorted(ds.topics)})")
    if camera and not info.get("livo_ready"):
        sys.exit(f"{what}: recording {bag} has no camera topic (midrange_camera/ffmpeg); "
                 "use the raw recording root that contains the Camera_* segments")
    return ds, {"path": str(bag), "kind": info.get("kind"), "segments": len(info.get("segments", [])),
                "duration_s": info.get("duration_s"), "has_camera": bool(info.get("has_camera"))}


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
            m = json.loads(meta.read_text())
            # the directory is the identity: a copied meta.json must not alias (or 'activate') another version
            rel = str(meta.parent.relative_to(CAL_ROOT))
            if m.get("id") != rel:
                m["meta_id_mismatch"] = m.get("id")
                m["id"] = rel
            out.append(m)
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


def changes_vs_parent(calib, parent_calib):
    """Human-readable delta between two calibrations (cm / deg / s); empty dict when identical."""
    out = {}
    a, b = summarize(parent_calib), summarize(calib)
    rows_a = {r["lidar"]: r for r in a["lidars"]}
    for r in b["lidars"]:
        p = rows_a.get(r["lidar"])
        if not p:
            out.setdefault("lidars", {})[r["lidar"]] = "new"
            continue
        dt = [round((x - y) * 100.0, 2) for x, y in zip(r["t"], p["t"])]
        dr = [round(x - y, 3) for x, y in zip(r["rpy_deg"], p["rpy_deg"])]
        if any(abs(v) >= 0.01 for v in dt) or any(abs(v) >= 0.001 for v in dr):
            out.setdefault("lidars", {})[r["lidar"]] = {"dt_cm": dt, "drpy_deg": dr}
    if a.get("camera_to_rear") and b.get("camera_to_rear"):
        dt = [round((x - y) * 100.0, 2) for x, y in zip(b["camera_to_rear"]["t"], a["camera_to_rear"]["t"])]
        dr = [round(x - y, 3) for x, y in zip(b["camera_to_rear"]["rpy_deg"], a["camera_to_rear"]["rpy_deg"])]
        if any(abs(v) >= 0.01 for v in dt) or any(abs(v) >= 0.001 for v in dr):
            out["camera_to_rear"] = {"dt_cm": dt, "drpy_deg": dr}
    ta, tb = a.get("time_offsets") or {}, b.get("time_offsets") or {}
    for k in sorted(set(ta) | set(tb)):
        if isinstance(tb.get(k), (int, float)) or isinstance(ta.get(k), (int, float)):
            if ta.get(k) != tb.get(k):
                out.setdefault("time_offsets", {})[k] = {"from": ta.get(k), "to": tb.get(k)}
    ca, cb = (a.get("camera") or {}), (b.get("camera") or {})
    for k in ("intrinsics", "distortion"):
        if ca.get(k) != cb.get(k):
            out.setdefault("camera", {})[k] = {"from": ca.get(k), "to": cb.get(k)}
    return out


def lineage_of(vid, parent):
    chain = [vid]
    seen = set()
    while parent and parent not in seen:
        seen.add(parent)
        chain.insert(0, parent)
        meta_path = CAL_ROOT.joinpath(*parent.strip("/").split("/")) / "meta.json"
        if not meta_path.exists():
            break
        try:
            parent = json.loads(meta_path.read_text()).get("parent")
        except Exception:  # noqa: BLE001
            break
    return chain


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
    changes = {}
    if parent:
        try:
            changes = changes_vs_parent(calib, json.loads((version_dir(parent) / "calibration.json").read_text()))
        except SystemExit:
            changes = {}
    stage_info = STAGES.get(stage, {})
    meta = {"id": vid, "mine": mine, "truck": truck, "version": out_dir.name, "stage": stage,
            "stage_label": stage_info.get("label", stage), "advisory": bool(stage_info.get("advisory")),
            "parent": parent, "lineage": lineage_of(vid, parent),
            "created": now(), "note": note, "inputs": inputs or {}, "metrics": metrics or {},
            "changes_vs_parent": changes, "summary": summarize(calib),
            "calibration_file": str(out_dir / "calibration.json"),
            "overlays": sorted(p.name for p in (out_dir / "overlays").iterdir() if p.suffix in (".png", ".jpg")) if (out_dir / "overlays").is_dir() else []}
    (out_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    lines = [f"# {vid}", "", f"- stage: {stage} ({meta['stage_label']})", f"- parent: {parent}",
             f"- lineage: {' -> '.join(meta['lineage'])}", f"- created: {meta['created']}", f"- note: {note}", ""]
    if inputs:
        lines += ["## inputs", ""] + [f"- {k}: {v}" for k, v in inputs.items()] + [""]
    if changes:
        lines += ["## changes vs parent", "", "```", json.dumps(changes, indent=2, ensure_ascii=False), "```", ""]
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


def ground_equations(mc0):
    """Supplier ground-distance curves `midrange_camera[0].equation_<d>m`: (mantissa, exponent) pairs = coefficients
    (highest power first) of the image row of the flat-ground circle of radius d m as a polynomial in the image
    column. Returns {d: coefficients}, sorted by distance."""
    eqs = {}
    for key, val in (mc0 or {}).items():
        m = re.fullmatch(r"equation_(\d+(?:\.\d+)?)m", str(key))
        if not m or not isinstance(val, (list, tuple)):
            continue
        v = np.asarray(val, float).reshape(-1)
        if v.size < 4 or v.size % 2 or not np.isfinite(v).all():
            continue
        c = v.reshape(-1, 2)
        eqs[float(m.group(1))] = c[:, 0] * 10.0 ** c[:, 1]
    return dict(sorted(eqs.items()))


def intrinsics_from_ground_equations(eqs, T_rear_cam, width=1920, height=1080):
    """Recover the pinhole intrinsics the supplier used to draw its ground-distance curves.

    Each curve is the image of a flat-ground circle (radius d, centre (x0, y0) at height zg in the rear-axle frame)
    seen through the supplier's own camera extrinsic T_rear_cam, so the curves fix fx, fy, cx, cy (+ x0, y0, zg).
    fy and zg trade off weakly, so the square-pixel model is preferred whenever it fits as well as the free one.
    Checked on truck29 (whose yaml intrinsics are known): fx/cx/cy within 2.2 px, fy within 0.5 %.
    Returns (intrinsics [fx, fy, cx, cy] or None, info)."""
    from scipy.optimize import least_squares
    info = {"curves": len(eqs), "distances_m": [float(d) for d in eqs]}
    if len(eqs) < 4:
        info["reason"] = f"only {len(eqs)} ground-distance curves (need >= 4)"
        return None, info
    T = np.asarray(T_rear_cam, float).reshape(4, 4)
    T_cam_rear = np.linalg.inv(T)
    cols = np.arange(0.0, float(width), 40.0)
    ang = np.radians(np.linspace(-89.0, 89.0, 1601))
    targets = [(d, np.polyval(c, cols)) for d, c in eqs.items()]

    def rows(fx, fy, cx, cy, x0, y0, zg, d):
        P = np.vstack([x0 + d * np.sin(ang), y0 + d * np.cos(ang), np.full_like(ang, zg)])
        pc = T_cam_rear[:3, :3] @ P + T_cam_rear[:3, 3:4]
        ok = pc[2] > 0.5
        u, v = cx + fx * pc[0, ok] / pc[2, ok], cy + fy * pc[1, ok] / pc[2, ok]
        o = np.argsort(u)
        return np.interp(cols, u[o], v[o], left=np.nan, right=np.nan)

    def residuals(p, square):
        fx, fy, rest = (p[0], p[0], p[1:]) if square else (p[0], p[1], p[2:])
        out = []
        for d, ve in targets:
            m = (ve > 0) & (ve < height)
            r = rows(fx, fy, *rest, d)[m] - ve[m]
            out.append(np.where(np.isfinite(r), r, 200.0))
        return np.concatenate(out)

    f0 = 0.5 * width / np.tan(np.radians(30.0))
    cam_z = float(T[2, 3])
    fits = {}
    for square in (False, True):
        best = None
        for zg0 in (cam_z - 2.5, cam_z - 3.5, cam_z - 4.5):
            p0 = ([f0] if square else [f0, f0]) + [width / 2.0, height / 2.0, float(T[0, 3]), float(T[1, 3]), zg0]
            try:
                s = least_squares(residuals, p0, args=(square,), x_scale="jac", max_nfev=4000)
            except Exception:  # noqa: BLE001
                continue
            if best is None or s.cost < best.cost:
                best = s
        if best is None:
            continue
        p = [float(v) for v in best.x]
        if square:
            p = [p[0]] + p
        fits["square" if square else "free"] = {
            "intrinsics": [round(v, 2) for v in p[:4]], "rms_px": round(float(np.sqrt(np.mean(best.fun ** 2))), 3),
            "circle_centre_rear_m": [round(p[4], 3), round(p[5], 3)], "ground_z_rear_m": round(p[6], 3)}
    info["fits"] = fits
    if "free" not in fits:
        info["reason"] = "ground-curve fit did not converge"
        return None, info
    free, sq = fits["free"], fits.get("square")
    pick = sq if sq and sq["rms_px"] <= max(1.1 * free["rms_px"], free["rms_px"] + 0.02) else free
    fx, fy, cx, cy = pick["intrinsics"]
    h_cam = cam_z - pick["ground_z_rear_m"]
    info.update({"model": "square pixels (fx = fy)" if pick is sq else "independent fx / fy",
                 "rms_px": pick["rms_px"], "camera_height_m": round(h_cam, 3)})
    problems = []
    if pick["rms_px"] > 1.0:
        problems.append(f"rms {pick['rms_px']} px > 1 px")
    if not (0.3 * width < fx < 5 * width and 0.3 * width < fy < 5 * width):
        problems.append(f"focal length {fx:.0f}/{fy:.0f} px implausible")
    if not (0.25 * width < cx < 0.75 * width and 0.25 * height < cy < 0.75 * height):
        problems.append(f"principal point ({cx:.0f}, {cy:.0f}) far from the image centre")
    if not (0.5 < h_cam < 8.0):
        problems.append(f"camera height {h_cam:.2f} m above the curve ground implausible")
    if problems:
        info["reason"] = "; ".join(problems)
        return None, info
    return [fx, fy, cx, cy], info


# Intrinsics that agree with the truck29 defaults this closely are the same camera model: keep the yaml values
# (the curves determine fy only to ~0.5 %), otherwise the curve fit replaces the borrowed defaults.
SAME_CAMERA_TOL = {"focal_rel": 0.015, "centre_px": 15.0}


def supplier_camera_model(mc0, T_rear_cam, camera):
    """Camera intrinsics for a supplier file without camera_in_params: (intrinsics, source, check, note).
    intrinsics is None when the defaults stay (confirmed or no usable curves)."""
    eqs = ground_equations(mc0)
    default = DEFAULT_CAMERA["intrinsics"]
    if not eqs:
        return None, "defaults (wuhu truck29 yaml)", None, (
            "WARNING camera intrinsics: the supplier file has neither camera_in_params nor equation_*m curves; the "
            "truck29 defaults are used unverified - set the real intrinsics (livo_calib.py set --intrinsics)")
    fit, info = intrinsics_from_ground_equations(eqs, T_rear_cam, camera.get("width", 1920), camera.get("height", 1080))
    check = {"method": "supplier ground-distance curves (midrange_camera[0].equation_*m)", **info}
    if fit is None:
        return None, "defaults (wuhu truck29 yaml)", check, (
            f"WARNING camera intrinsics: truck29 defaults kept unverified, the ground-curve fit was rejected "
            f"({info.get('reason')}) - check the camera model before LIVO")
    dev = {"fx_pct": round(100 * (fit[0] / default[0] - 1), 2), "fy_pct": round(100 * (fit[1] / default[1] - 1), 2),
           "cx_px": round(fit[2] - default[2], 1), "cy_px": round(fit[3] - default[3], 1)}
    check["deviation_vs_truck29_defaults"] = dev
    same = (max(abs(dev["fx_pct"]), abs(dev["fy_pct"])) <= 100 * SAME_CAMERA_TOL["focal_rel"]
            and max(abs(dev["cx_px"]), abs(dev["cy_px"])) <= SAME_CAMERA_TOL["centre_px"])
    if same:
        return None, f"defaults (wuhu truck29 yaml), confirmed by the supplier ground-distance curves (rms {info['rms_px']} px)", \
            check, "camera intrinsics: truck29 defaults confirmed by the supplier ground-distance curves"
    return fit, f"supplier ground-distance curves equation_*m ({info['model']}, rms {info['rms_px']} px)", check, (
        f"camera intrinsics recovered from the supplier ground-distance curves: fx {fit[0]:.1f} fy {fit[1]:.1f} "
        f"cx {fit[2]:.1f} cy {fit[3]:.1f} (the truck29 defaults would be off by fx {dev['fx_pct']:+.1f} % "
        f"fy {dev['fy_pct']:+.1f} % cx {dev['cx_px']:+.0f} px cy {dev['cy_px']:+.0f} px)")


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
    raw_supplier = cam_tf is None
    if raw_supplier:
        # Raw supplier export: only camera<-reference-LiDAR (midrange_camera[0].tf_lidar_to_camera) is given.
        # camera->rear = rear<-reference * inverse(camera<-reference); the reference is the correlated LiDAR.
        ref_key = calib.get("calibration_reference_lidar") or calib.get("correlated_lidar_matrix_tf") or LIDAR_KEYS["front_lidar"]
        mc = calib.get("midrange_camera")
        tf = mc[0].get("tf_lidar_to_camera") if isinstance(mc, list) and mc and isinstance(mc[0], dict) else None
        ref = next((e[ref_key] for e in calib["lidar"] if ref_key in e), None)
        if tf is None or ref is None:
            sys.exit("source file has neither transforms.midrange_camera_to_rear_axle nor "
                     f"midrange_camera[0].tf_lidar_to_camera + lidar[].{ref_key}")
        tf = np.asarray(tf, float)
        if tf.size == 18:
            tf = tf[2:]
            notes.append("tf_lidar_to_camera: dropped two leading padding values (18 -> 16)")
        cam_from_ref = orthonormalize(tf.reshape(4, 4))
        cam_tf = (np.asarray(ref, float).reshape(4, 4) @ np.linalg.inv(cam_from_ref)).reshape(-1).tolist()
        tr["midrange_camera_to_rear_axle"] = {"source_frame": "midrange_camera", "target_frame": "vehicle_rear_axle",
                                             "transform_semantics": "target_from_source",
                                             "derived_from": f"{ref_key} * inverse(midrange_camera[0].tf_lidar_to_camera)"}
        calib["calibration_reference_lidar"] = ref_key
        notes.append(f"raw supplier export: camera->rear-axle derived from {ref_key} and tf_lidar_to_camera")
        notes.append("rig-to-rear-axle alignment of a supplier file is unverified: run the axle stage next")
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
    elif source.startswith("defaults") and isinstance(mc, list) and mc and isinstance(mc[0], dict):
        # Raw supplier exports carry no intrinsics, only ground-distance curves drawn with them. Borrowing the
        # truck29 model silently is wrong for any other camera (truck05: fx 1849 vs 1643, principal point 32 px off).
        fit, source, check, note = supplier_camera_model(mc[0], m, camera)
        if fit is not None:
            camera["intrinsics"] = fit
        if check is not None:
            camera["intrinsics_check"] = check
        notes.append(note)
    if a.distortion:
        camera["distortion"] = [float(v) for v in a.distortion]
    camera["source"] = source
    calib["camera"] = camera
    offsets = dict(DEFAULT_TIME_OFFSETS)
    if a.lidar_offset is not None:
        offsets["lidar"] = a.lidar_offset
        offsets["lidar_source"] = "operator import"
    else:
        offsets["lidar_source"] = "import default (unmeasured; run the axle stage to auto-measure)"
    if a.camera_offset is not None:
        offsets["camera"] = a.camera_offset
        offsets["camera_source"] = "operator import"
    offsets["source"] = "import (verify with camera-time / LIO evaluation)"
    calib["time_offsets"] = offsets
    calib["calibration_type"] = calib.get("calibration_type", "") or (
        "supplier_raw_lidar_to_rear_axle" if raw_supplier else "multi_lidar_to_rear_axle_manual")
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


def axle_lateral_pass(a, out_dir, ds_info, calib):
    """Lateral lever arm of the rig from the WHOLE recording, applied on top of the window solution.

    The window (Studio: the manual calibration window, typically 0-120 s of one-way driving) observes the lateral
    rig offset only through a few turns (truck29raw 20261013: x repeatability 3.6 cm, sqrt-info 38/m). A body-fixed
    lateral error separates opposite-direction passes by twice its value, so the round trips of a full recording
    observe it directly; truck29raw v019 had 8-10 cm between the ramp ascent and descent (fused map). Only x is
    taken: the forward offset of a round trip is indistinguishable from an along-track timing error (RTK
    association / INS latency; an unconstrained full-recording solve moved y by 21 cm and broke the along-track
    consistency of the fused map 3.9 -> 36 cm) and stays with the window solution and the rtk-time stage. Rotation and
    time offset are kept (the joint solve still estimates every DoF). Applied when converged, |x| <= 15 cm and the
    matches come from revisits (match time gap median >= 30 s); never aborts the stage."""
    dur = float((ds_info or {}).get("duration_s") or 0.0)
    w0, w1 = float(a.window[0]), float(a.window[1])
    if dur <= 0.0 or min(w1, dur) - max(w0, 0.0) >= dur - 60.0:
        return {"decision": "skipped (the window already covers the recording)"}
    tmp, rep = out_dir / "lateral_calibration.json", out_dir / "stage_report_lateral.txt"
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_rear_axle_extrinsic.py"), "--calibration", str(out_dir / "calibration.json"),
           "--bag", a.bag, "--output", str(tmp), "--report", str(rep), "--dof", "x", "--keep-time-offset", "--every", "6"]
    print("+", " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=REPO)
    (out_dir / "stage_lateral.log").write_text(proc.stdout)
    print(proc.stdout, end="", flush=True)
    if proc.returncode != 0 or not tmp.exists():
        return {"decision": f"skipped (lateral solve exit {proc.returncode})",
                "lines": [f"lateral lever arm (whole recording): skipped, solver exit {proc.returncode}"]}
    sol = json.loads(tmp.read_text()); tmp.unlink()
    meta = sol.get("refinement", {}).get("rear_axle_alignment", {})
    dx = float((meta.get("correction_translation_cm") or [0.0])[0])
    m = re.search(r"match time gap median ([\d.]+) s", proc.stdout); gap = float(m.group(1)) if m else 0.0
    out = {"correction_x_cm": round(dx, 2), "match_gap_median_s": gap, "converged": bool(meta.get("converged")),
           "residual_cm": meta.get("residual_cm"), "repeatability_halves": meta.get("repeatability_halves"),
           "scans_used": meta.get("scans_used"), "window_s": meta.get("window_s")}
    if out["converged"] and abs(dx) <= 15.0 and gap >= 30.0:
        calib["lidar"] = sol["lidar"]
        if sol.get("transforms"):
            calib["transforms"] = sol["transforms"]
        calib["calibration_type"] = str(calib.get("calibration_type", "")) + "+rtk_rear_axle_lateral"
        out["decision"] = f"applied x {dx:+.1f} cm"
        calib.setdefault("refinement", {})["rear_axle_lateral"] = dict(out)
        (out_dir / "calibration.json").write_text(json.dumps(calib, indent=1))
    else:
        out["decision"] = "not applied (" + ("not converged" if not out["converged"] else
                                             f"|x| {abs(dx):.1f} cm > 15" if abs(dx) > 15.0 else
                                             f"no round trips: match time gap median {gap:.0f} s < 30") + ")"
    out["lines"] = [f"lateral lever arm (whole recording, dof x, rotation/time kept): {out['decision']}; "
                    f"solved x {dx:+.1f} cm, match time gap median {gap:.0f} s"]
    return out


def cmd_axle(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, what="axle")
    out_dir = next_version_dir(mine, truck, "axle")
    out_dir.mkdir(parents=True, exist_ok=True)
    dof = a.dof
    if dof == "auto":
        # Only a parent this stage already aligned keeps its rotation. Any other import (raw supplier file or a
        # hand-tuned schema-5 export) has never been checked against RTK/INS: the old "hand-tuned = aligned" rule
        # left truck29 v001_import 0.14/0.15/0.23 deg (pitch/roll/yaw) off the INS although the solver measured it
        # with 0.002/0.001/0.05 deg half-to-half repeatability; the LiDAR/IMU tilt made native LIO drift 14.5 m in
        # 120 s (same code, aligned lineage: 2.1 m). On an aligned parent the solved rotation is ~0 anyway.
        ctype = str(parent_calib.get("calibration_type", ""))
        dof = "xy" if "rear_axle_alignment" in ctype else "rpy_xy"
        print(f"axle: --dof auto -> {dof} (parent calibration_type {ctype or '-'})", flush=True)
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_rear_axle_extrinsic.py"), "--calibration",
           str(pdir / "calibration.json"), "--bag", a.bag, "--output", str(out_dir / "calibration.json"),
           "--report", str(out_dir / "stage_report.txt"), "--window", str(a.window[0]), str(a.window[1]), "--dof", dof]
    if a.keep_time_offset:
        cmd.append("--keep-time-offset")
    lines = run_script(cmd, out_dir / "stage.log")
    calib = json.loads((out_dir / "calibration.json").read_text())
    # Front-end time base: time_offsets.lidar is the scan<->INS alignment this stage just
    # solved, but the SLAM front-end deskews against the raw IMU stream. On every truck
    # measured so far the INS odometry timeline runs 15-21 ms behind the raw IMU for the
    # same motion, so lidar_frontend = lidar + imu/ins clock lag (truck29 -0.1459 + 0.0209
    # = -0.125, truck05 -0.0267 + 0.0148 = -0.0119). A lidar_frontend inherited from the
    # import template of another truck desynchronises deskew by >100 ms on GPS-stamped
    # LiDARs (zongyang truck07: native LIO yaw drift 1.1 deg / 2.3 m position in 45 s).
    frontend_sync = {}
    if not a.keep_time_offset and not getattr(a, "no_frontend_sync", False):
        offsets = calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))
        solved = offsets.get("lidar")
        template = offsets.get("lidar_frontend")
        if template is None:
            # versions imported before lidar_frontend existed: compare against the default every new import
            # carries instead of replacing it with lidar + lag (truck29 v082: -0.1335 instead of the hardware
            # -0.125; the yaw-rate lag on this INS reads 12.5 ms on every truck, the 0.02 s tolerance absorbs it).
            # Written explicitly: without the key the mapper deskews with time_offsets.lidar (21 ms off on truck29).
            template = DEFAULT_TIME_OFFSETS["lidar_frontend"]
            offsets["lidar_frontend"] = template
            offsets["lidar_frontend_source"] = "axle: import default (parent version had no lidar_frontend)"
            (out_dir / "calibration.json").write_text(json.dumps(calib, indent=1))
        if isinstance(solved, (int, float)):
            lag, corr, yaw_rms = measure_imu_ins_clock_lag(a.bag)
            frontend_sync = {"imu_ins_lag_s": lag, "corr": round(corr, 4),
                             "yaw_rms_rad_s": round(yaw_rms, 4), "template": template}
            if lag is None or corr < 0.95 or yaw_rms < 0.02:
                frontend_sync["decision"] = "kept template (weak yaw excitation or low imu/ins correlation)"
            else:
                candidate = round(float(solved) + lag, 4)
                if template is None or abs(candidate - float(template)) > 0.02:
                    offsets["lidar_frontend"] = candidate
                    offsets["lidar_frontend_source"] = (
                        "axle: lidar %+.4f + imu/ins clock lag %+.4f (corr %.3f); template %s replaced"
                        % (float(solved), lag, corr, "none" if template is None else "%+.4f" % float(template)))
                    frontend_sync["decision"] = "replaced template with %+.4f" % candidate
                    (out_dir / "calibration.json").write_text(json.dumps(calib, indent=1))
                else:
                    frontend_sync["decision"] = "template confirmed (candidate %+.4f)" % candidate
            lines.append("frontend time base: %s; imu/ins lag %s, corr %.3f, yaw rms %.3f rad/s"
                         % (frontend_sync["decision"],
                            "n/a" if lag is None else "%+.1f ms" % (lag * 1000.0),
                            corr, yaw_rms))
    lateral = {"decision": "disabled (--no-lateral)"} if getattr(a, "no_lateral", False) else axle_lateral_pass(a, out_dir, ds_info, calib)
    lines += lateral.pop("lines", [])
    metrics = {"rear_axle_alignment": calib.get("refinement", {}).get("rear_axle_alignment", {}),
               "frontend_sync": frontend_sync, "lateral": lateral}
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="axle", parent=a.id,
                     note=a.note or "reference-LiDAR -> rear-axle alignment from RTK/INS poses (rig correction)",
                     inputs={"bag": a.bag, "dataset": ds_info, "window": list(a.window), "dof": dof,
                             "time_offset": "kept" if a.keep_time_offset else "estimated"},
                     metrics=metrics, report_lines=lines, activate=a.activate)


def cmd_ground(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, what="ground")
    out_dir = next_version_dir(mine, truck, "ground")
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_multi_lidar_extrinsics.py"), "--calibration",
           str(pdir / "calibration.json"), "--bag", a.bag, "--output", str(out_dir / "calibration.json"),
           "--report", str(out_dir / "stage_report.txt"), "--stationary", str(a.stationary[0]), str(a.stationary[1]),
           "--moving", str(a.moving[0]), str(a.moving[1])]
    lines = run_script(cmd, out_dir / "stage.log")
    calib = json.loads((out_dir / "calibration.json").read_text())
    refinement = calib.get("refinement", {})
    # quality: ok / warn per LiDAR (a LiDAR outside the refinement basin - mount moved since the parent file - is
    # re-initialised from its own ground plane and flagged; the stage never writes a regressed matrix)
    metrics = {"corrections": refinement.get("corrections", {}), "quality": refinement.get("quality", "ok")}
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="ground", parent=a.id,
                     note=a.note or "static-start ground roll/pitch/z refinement",
                     inputs={"bag": a.bag, "dataset": ds_info, "stationary": list(a.stationary),
                             "moving": list(a.moving)},
                     metrics=metrics, report_lines=lines, activate=a.activate)


def cmd_driven(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, what="driven")
    if not (pathlib.Path(a.run) / "frames.csv").exists():
        sys.exit(f"driven: {a.run} is not a fusion run directory (frames.csv missing); run a LIO round first")
    out_dir = next_version_dir(mine, truck, "driven")
    out_dir.mkdir(parents=True, exist_ok=True)
    # frames.csv / trajectory_optimized.txt are stamped in the front-end time base (header + lidar_frontend);
    # time_offsets.lidar is the INS-alignment offset of the fusion and would misplace every scan by the
    # difference (truck29: 20.9 ms = 13.6 cm at 6.5 m/s) on the pose timeline
    offsets = parent_calib.get("time_offsets", {})
    lidar_offset = offsets.get("lidar_frontend", offsets.get("lidar", -0.125))
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_multi_lidar_yaw.py"), "--calibration",
           str(pdir / "calibration.json"), "--bag", a.bag, "--run", a.run, "--window", str(a.window[0]),
           str(a.window[1]), "--lidar-time-offset", str(lidar_offset), "--output",
           str(out_dir / "calibration.json"), "--report", str(out_dir / "stage_report.txt")]
    if a.vertical_only:
        cmd.append("--vertical-only")
    if getattr(a, "pass1_poses", None):
        cmd += ["--pass1-poses", a.pass1_poses]
    lines = run_script(cmd, out_dir / "stage.log")
    calib = json.loads((out_dir / "calibration.json").read_text())
    metrics = {k: v for k, v in calib.items() if k.startswith("refinement")}
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="driven", parent=a.id,
                     note=a.note or "driven-window 6-DoF refinement", inputs={"bag": a.bag, "dataset": ds_info,
                     "run": a.run, "window": list(a.window)}, metrics={"refinement": metrics}, report_lines=lines,
                     activate=a.activate)


def cmd_gcons(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, what="gcons")
    if not ((pathlib.Path(a.run) / "frames.csv").exists() and (pathlib.Path(a.run) / "trajectory_optimized.txt").exists()):
        sys.exit(f"gcons: {a.run} is not an RTK-fused fusion run directory (frames.csv / trajectory_optimized.txt missing)")
    out_dir = next_version_dir(mine, truck, "gcons")
    out_dir.mkdir(parents=True, exist_ok=True)
    # frames.csv / trajectory_optimized.txt are stamped in the front-end time base (see cmd_driven)
    offsets = parent_calib.get("time_offsets", {})
    lidar_offset = offsets.get("lidar_frontend", offsets.get("lidar", -0.125))
    cmd = ["/usr/bin/python3", str(SCRIPTS / "refine_lidar_ground_consensus.py"), "--calibration",
           str(pdir / "calibration.json"), "--bag", a.bag, "--run", a.run, "--lidar-time-offset", str(lidar_offset),
           "--output", str(out_dir / "calibration.json"), "--report", str(out_dir / "stage_report.txt")]
    lines = run_script(cmd, out_dir / "stage.log")
    if not (out_dir / "calibration.json").exists():
        # no significant correction: the parent stays the result (the pipeline continues with it)
        shutil.rmtree(out_dir, ignore_errors=True)
        if out_dir in CREATED_DIRS:
            CREATED_DIRS.remove(out_dir)
        print(json.dumps({"id": a.id, "kept": True, "reason": "no significant ground-consensus correction"}, ensure_ascii=False))
        return
    calib = json.loads((out_dir / "calibration.json").read_text())
    metrics = {"ground_consensus": calib.get("refinement", {}).get("ground_consensus", {})}
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="gcons", parent=a.id,
                     note=a.note or "rear-facing LiDAR tilt/z from the multi-LiDAR ground consensus",
                     inputs={"bag": a.bag, "dataset": ds_info, "run": a.run}, metrics=metrics, report_lines=lines,
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


# ----------------------------------------------------------------------------- RTK position time association
# Fraction of an offset error that one round's map-consistency estimate recovers (the backend follows the
# native odometry through the accelerations that make the offset observable). Measured on truck28: 0.55-0.65.
# Only used to place the second (probe) round and as a fallback; the result comes from the two-round secant.
RTK_TIME_GAIN_PRIOR = 0.6
RTK_TIME_TOL_S = 0.0015     # |correction| below this: the parent is already map-consistent, nothing to do
RTK_TIME_MAX_SE_S = 0.006   # observability gate (jackknife standard error of one round's estimate)
RTK_TIME_SIGNIFICANCE = 2.5  # a correction must exceed this many standard errors to be acted on
RTK_TIME_MAX_STEP_S = 0.08  # plausibility bound on the total change


def _fusion_run(path):
    p = pathlib.Path(path)
    if (p / "frames.csv").exists():
        return p
    runs = sorted((p / "Log" / "fusion").glob("*/"))
    if not runs:
        sys.exit(f"rtk-time: {p} has no fusion run (Log/fusion/<stamp>/frames.csv)")
    return runs[-1]


def _run_calibration_file(run):
    for line in (run / "parameters.txt").read_text().splitlines():
        if line.startswith("multi_lidar.calibration_file="):
            return line.split("=", 1)[1].strip()
    return None


def estimate_rtk_time(run, log_path):
    """Map-consistency estimate of one round (scripts/estimate_rtk_time_offset.py), JSON dict."""
    json_path = log_path.with_suffix(".json")
    cmd = ["/usr/bin/python3", str(SCRIPTS / "estimate_rtk_time_offset.py"), "--run", str(run), "--json",
           str(json_path), "--max-se-ms", f"{1000 * RTK_TIME_MAX_SE_S:.2f}"]
    lines = run_script(cmd, log_path)
    return json.loads(json_path.read_text()), lines


def cmd_rtk_time(a):
    """Two-step closed loop, both steps automatic (the Studio LiDAR pipeline chains them):
      probe : --run R1 (round with the parent)  -> keep (unobservable / already consistent) or the
              rtk_position of a probe round placed at the predicted optimum
      final : --run R1 --run2 R2 (round with the probe value) -> secant zero of the two corrections ->
              new version (stage rtktime)."""
    pdir, parent_calib, mine, truck = parent_info(a.id)
    offsets = parent_calib.get("time_offsets") or {}
    rtk0 = float(offsets.get("rtk_position", DEFAULT_TIME_OFFSETS["rtk_position"]))
    run1 = _fusion_run(a.run)
    cal1 = _run_calibration_file(run1)
    if cal1 and pathlib.Path(cal1).resolve() != (pdir / "calibration.json").resolve():
        sys.exit(f"rtk-time: {run1} was mapped with {cal1}, not with the parent {a.id}")
    work = run1.parent.parent.parent  # round directory
    try:
        est1, lines1 = estimate_rtk_time(run1, work / "rtk_time_estimate.log")
    except SystemExit as exc:
        if a.run2:
            raise
        # an optional refinement must not fail the pipeline: keep the parent and say why
        why = f"estimate failed ({exc}): rtk_position of the parent kept"
        print(why, flush=True)
        print(json.dumps({"id": a.id, "decision": "keep", "reason": why, "probe_rtk_position_s": None}, ensure_ascii=False))
        return
    e1, d1, se1 = est1["run_effective_offset_s"], est1["correction_s"], est1["correction_se_s"]
    summary = {"parent": a.id, "parent_rtk_position_s": rtk0, "round1": {"run": str(run1), "effective_s": e1,
               "correction_s": d1, "se_s": se1, "observable": est1["observable"], "frames": est1["frames"],
               "informative_voxels": est1["informative_voxels"]}}
    if not a.run2:
        if not est1["observable"]:
            decision, why = "keep", (f"not observable on this round (se {1000 * se1:.1f} ms > {1000 * RTK_TIME_MAX_SE_S:.0f} ms "
                                     f"or no interior minimum): rtk_position of the parent kept")
        elif abs(d1) <= max(RTK_TIME_TOL_S, RTK_TIME_SIGNIFICANCE * se1):
            decision, why = "keep", (f"parent already map-consistent (correction {1000 * d1:+.1f} ms, "
                                     f"se {1000 * se1:.1f} ms)")
        else:
            decision, why = "probe", f"correction {1000 * d1:+.1f} ms: probe round at the predicted optimum"
        step = float(np.clip(d1 / RTK_TIME_GAIN_PRIOR, -RTK_TIME_MAX_STEP_S, RTK_TIME_MAX_STEP_S))
        out = {"id": a.id, "decision": decision, "reason": why, "probe_rtk_position_s": round(rtk0 + step, 4)
               if decision == "probe" else None, **summary}
        print(why, flush=True)
        print(json.dumps(out, ensure_ascii=False))
        return
    run2 = _fusion_run(a.run2)
    est2, lines2 = estimate_rtk_time(run2, run2.parent.parent.parent / "rtk_time_estimate.log")
    if abs(est2["run_effective_offset_s"] - est1["run_effective_offset_s"]) < 1e-4:
        sys.exit("rtk-time: both rounds ran with the same effective offset; the probe round must use the probe value")
    e2, d2, se2 = est2["run_effective_offset_s"], est2["correction_s"], est2["correction_se_s"]
    summary["round2"] = {"run": str(run2), "effective_s": e2, "correction_s": d2, "se_s": se2,
                         "observable": est2["observable"], "frames": est2["frames"],
                         "informative_voxels": est2["informative_voxels"]}
    gain = (d1 - d2) / (e2 - e1) if abs(e2 - e1) > 1e-4 else float("nan")
    if est1["observable"] and est2["observable"] and 0.25 <= gain <= 1.5:
        best, method = e1 + d1 / gain, f"secant of both rounds (measured gain {gain:.2f})"
        # uncertainty of the secant zero from the two jackknife errors
        se_best = float(np.hypot(se1 * abs(e2 - best), se2 * abs(e1 - best)) / max(abs(gain * (e2 - e1)), 1e-4))
    else:
        # no usable secant: each usable round predicts the optimum with the prior gain; combine them weighted by
        # their standard errors (a weak round still pulls a little instead of being thrown away)
        def usable(est, se):
            return est["observable"] or (np.isfinite(se) and se <= 2 * RTK_TIME_MAX_SE_S)
        preds = [(e + d / RTK_TIME_GAIN_PRIOR, se / RTK_TIME_GAIN_PRIOR, tag)
                 for est, e, d, se, tag in ((est1, e1, d1, se1, "first"), (est2, e2, d2, se2, "probe")) if usable(est, se)]
        if not preds:
            sys.exit("rtk-time: neither round is observable; parent kept")
        w = np.array([1.0 / max(sp, 1e-4) ** 2 for _, sp, _ in preds])
        best = float(np.dot(w, [p for p, _, _ in preds]) / w.sum())
        se_best = float(1.0 / np.sqrt(w.sum()))
        method = (f"{' + '.join(t for _, _, t in preds)} round{'s' if len(preds) > 1 else ''} with the prior gain "
                  f"{RTK_TIME_GAIN_PRIOR}, standard-error weighted (secant gain {gain:.2f} not usable)")
    best = float(np.clip(best, e1 - RTK_TIME_MAX_STEP_S, e1 + RTK_TIME_MAX_STEP_S))
    rtk_new = round(rtk0 + (best - e1), 4)
    summary.update(method=method, gain=None if gain != gain else round(gain, 3), best_effective_s=round(best, 5),
                   best_se_s=round(se_best, 5), rtk_position_s={"from": rtk0, "to": rtk_new})
    if a.dry_run:
        print(json.dumps({"id": a.id, "dry_run": True, **summary}, ensure_ascii=False))
        return
    calib = copy.deepcopy(parent_calib)
    to = calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))
    to["rtk_position"] = rtk_new
    to["rtk_position_source"] = (f"rtk-time: map consistency, {method}; effective {1000 * best:+.1f} ms "
                                 f"(se {1000 * se_best:.1f} ms)")
    out_dir = next_version_dir(mine, truck, "rtktime")
    out_dir.mkdir(parents=True, exist_ok=True)
    report = [f"round 1: effective {1000 * e1:+.2f} ms -> correction {1000 * d1:+.2f} ms (se {1000 * se1:.2f})",
              f"round 2: effective {1000 * e2:+.2f} ms -> correction {1000 * d2:+.2f} ms (se {1000 * se2:.2f})",
              f"{method}: best effective {1000 * best:+.2f} ms (se {1000 * se_best:.2f} ms); "
              f"rtk_position {rtk0:+.4f} -> {rtk_new:+.4f} s"]
    for line in report:
        print(line, flush=True)
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="rtktime", parent=a.id,
                     note=a.note or f"RTK position time association {rtk0:+.4f} -> {rtk_new:+.4f} s ({method})",
                     inputs={"run": str(run1), "run2": str(run2)}, metrics={"rtk_time": summary},
                     report_lines=report + [""] + lines1[-3:] + lines2[-3:], activate=a.activate)


# ----------------------------------------------------------------------------- camera time offset
def read_bag_topics(bag, wanted):
    """Time-ordered deserialized messages straight from the recording (raw split root or single bag)."""
    from livo.dataset import Dataset
    ds = Dataset.open(bag)
    topics = ds.resolve_topics(list(wanted))
    if not topics:
        sys.exit(f"recording {bag} has none of the topics {list(wanted)} (has {sorted(ds.topics)})")
    for topic, msg, _ts in ds.read_messages(topics):
        yield topic, msg


def measure_imu_ins_clock_lag(bag, max_lag=0.30, step=0.0025, max_duration=240.0):
    """Clock lag of the INS odometry timeline relative to the raw IMU stream (s).

    Correlates body yaw rate (imu_data gyro z) against the yaw rate differentiated from
    imu_data/odometry orientation. Positive lag means the INS stamps the same physical
    motion later than the raw IMU (measured: truck29 +21 ms, truck05 +15 ms, truck07 +15 ms).
    The scan<->IMU front-end offset is therefore time_offsets.lidar (scan<->INS) + lag.
    Returns (lag_s | None, corr, yaw_rms_rad_s).
    """
    imu_t, imu_gz, odo_t, odo_yaw = [], [], [], []
    t0 = None
    for topic, msg in read_bag_topics(bag, ("imu_data", "imu_data/odometry")):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t0 is None:
            t0 = stamp
        elif stamp - t0 > max_duration and len(imu_t) > 500 and len(odo_t) > 500:
            break
        if topic.lstrip("/") == "imu_data":
            imu_t.append(stamp)
            imu_gz.append(msg.angular_velocity.z)
        else:
            q = msg.pose.pose.orientation
            odo_t.append(stamp)
            odo_yaw.append(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    if len(imu_t) < 500 or len(odo_t) < 500:
        return None, 0.0, 0.0
    imu_t = np.asarray(imu_t); imu_gz = np.asarray(imu_gz)
    odo_t = np.asarray(odo_t); odo_yaw = np.unwrap(np.asarray(odo_yaw))
    odo_rate = np.gradient(odo_yaw, odo_t)
    lo = max(imu_t[0], odo_t[0]) + 1.0
    hi = min(imu_t[-1], odo_t[-1]) - 1.0
    if hi - lo < 20.0:
        return None, 0.0, 0.0
    grid = np.arange(lo, hi, 0.01)
    gi = np.interp(grid, imu_t, imu_gz)
    yaw_rms = float(np.sqrt(np.mean(gi * gi)))
    gi_c = gi - gi.mean()
    best_lag, best_corr = None, -1.0
    for lag in np.arange(-max_lag, max_lag + step / 2.0, step):
        go = np.interp(grid, odo_t + lag, odo_rate)
        go_c = go - go.mean()
        denom = np.sqrt(float((gi_c * gi_c).sum()) * float((go_c * go_c).sum()))
        if denom <= 0.0:
            continue
        corr = float((gi_c * go_c).sum()) / denom
        if corr > best_corr:
            best_lag, best_corr = float(lag), corr
    return best_lag, best_corr, yaw_rms


def decode_hevc(stream_path, scale, keep=None):
    """Decode an Annex-B HEVC stream with ffmpeg; yields (index, gray frame)."""
    import cv2  # noqa: F401
    w, h = int(1920 * scale), int(1080 * scale)
    cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-f", "hevc", "-i", str(stream_path), "-vf", f"scale={w}:{h}",
           "-pix_fmt", "gray", "-f", "rawvideo", "-"]
    # -nostdin + DEVNULL: ffmpeg otherwise reads interactive commands from the caller's stdin (it swallowed the
    # rest of a shell script piped through stdin once).
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, bufsize=10 ** 7)
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


def dump_camera_stream(bag, out_dir, duration=None, start=0.0):
    """Write the HEVC packets to one Annex-B stream. duration (s) caps the stream length from the
    first image: the optical-flow/gyro correlation reaches full accuracy with ~60 s of driving
    (measured spread <=1 ms vs the full recording), so the camera-time stage does not need to
    decode the whole video. start (s after the first image) skips a standstill head; the kept
    stream begins at a key frame so every written packet decodes (index == packet index)."""
    stamps = []
    path = out_dir / "stream.h265"
    t_first = None
    with open(path, "wb") as f:
        for topic, m in read_bag_topics(bag, ["midrange_camera/ffmpeg"]):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            t_first = t if t_first is None else t_first
            if start and not stamps and (t - t_first < start or not int(getattr(m, "flags", 0)) & 1):
                continue
            if duration is not None and stamps and t - stamps[0] > duration:
                break
            stamps.append(t)
            f.write(bytes(m.data))
    return np.array(stamps), path


def read_imu(bag):
    t, g = [], []
    for topic, m in read_bag_topics(bag, ["imu_data"]):
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    return np.array(t), np.array(g)


def estimate_camera_offset(bag, out_dir, fx, fy, scale=0.25, duration=None, start=0.0):
    import cv2
    stamps, stream = dump_camera_stream(bag, out_dir, duration=duration, start=start)
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


# Gates of the flow/gyro offset. truck29 (camera 7.6 m ahead of the rear axle, open road): segment spread 3.9-6.6 ms,
# flow/gyro gain 1.15-1.25. truck05 (same lever arm, pitched 10 deg down, slopes 8-15 m away): gain 2.0-2.4, spread
# 22.6 ms, estimate -35 ms, and LIVO diverged in 3 of 4 turns with it while 0 ms mapped repeatably (1.06 / 1.01 m).
# Lateral camera motion in turns (yaw rate x lever arm / depth) adds flow in phase with the gyro but with a
# time-varying weight, which shifts the lag; the gain exposes it.
CAMTIME_MAX_SPREAD_MS = 12.0
CAMTIME_MAX_GAIN = 1.6
CAMTIME_MIN_CORR = 0.8


def camera_time_quality(result):
    """Whether a camera-time estimate is trustworthy enough to overwrite the parent's offset."""
    problems = []
    yaw = result.get("yaw") or {}
    spread = result.get("segment_spread_ms")
    gain = abs(float(yaw.get("gain") or float("nan")))
    if spread is not None and spread > CAMTIME_MAX_SPREAD_MS:
        problems.append(f"30 s segments disagree by {spread:.1f} ms (limit {CAMTIME_MAX_SPREAD_MS:.0f} ms)")
    if np.isfinite(gain) and gain > CAMTIME_MAX_GAIN:
        problems.append(f"image yaw flow is {gain:.2f}x the gyro rate (limit {CAMTIME_MAX_GAIN}): lateral camera motion in "
                        "turns (near scene, camera far ahead of the rear axle) biases the lag")
    if float(yaw.get("corr") or 0.0) < CAMTIME_MIN_CORR:
        problems.append(f"flow/gyro correlation {float(yaw.get('corr') or 0.0):.2f} < {CAMTIME_MIN_CORR}")
    return {"ok": not problems, "problems": problems, "segment_spread_ms": spread,
            "gain": round(gain, 3) if np.isfinite(gain) else None}


def cmd_camera_time(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, camera=True, what="camera-time")
    out_dir = next_version_dir(mine, truck, "camtime")
    out_dir.mkdir(parents=True, exist_ok=True)
    fx, fy = parent_calib.get("camera", DEFAULT_CAMERA)["intrinsics"][:2]
    start = a.start
    if start is None:  # skip a standstill head: the flow/gyro correlation needs yaw excitation
        start = suggest_windows(motion_profile(a.bag))["camera_time_start_s"]
    result = estimate_camera_offset(pathlib.Path(a.bag), out_dir, fx, fy, duration=a.duration, start=start)
    result["start_s"] = start
    if result["camera_offset_s"] is None:
        sys.exit("camera time offset could not be estimated (no usable flow)")
    calib = copy.deepcopy(parent_calib)
    offsets = calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))
    quality = camera_time_quality(result)
    result["quality"] = quality
    if quality["ok"]:
        offsets["camera"] = round(result["camera_offset_s"], 4)
        offsets["camera_source"] = f"optical flow vs gyro on {a.bag} (corr {result['yaw']['corr']:.2f})"
        calib["calibration_type"] = calib.get("calibration_type", "") + "+camera_time_offset"
    print(f"quality: {'ok' if quality['ok'] else 'warn'}  estimate {1000 * result['camera_offset_s']:+.1f} ms"
          + ("" if quality["ok"] else f" NOT applied, parent value {offsets.get('camera')} kept ({'; '.join(quality['problems'])})"),
          flush=True)
    (out_dir / "camera_time.json").write_text(json.dumps(result, indent=2) + "\n")
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="camtime", parent=a.id,
                     note=a.note or "camera->IMU time offset from optical flow vs gyro",
                     inputs={"bag": a.bag, "dataset": ds_info, "duration": a.duration, "start_s": start},
                     metrics={"camera_offset_s": offsets["camera"], "estimated_offset_s": round(result["camera_offset_s"], 4),
                              "applied": quality["ok"], "quality": quality, "yaw": result["yaw"], "pitch": result["pitch"],
                              "segments": result["segments"], "implied_fx_from_gain": result["implied_fx_from_gain"]},
                     activate=a.activate)


# ----------------------------------------------------------------------------- motion windows
MOTION_CACHE = REPO / "results" / ".studio" / "motion"
MOVING_MPS = 0.5          # driving when the INS speed exceeds this for MOVING_HOLD_S
MOVING_HOLD_S = 3.0
STATIONARY_MPS = 0.05
ROUND_LEAD_S = 8.0        # standstill kept before the first motion: IMU initialisation of the mapper
MIN_SHIFT_S = 5.0         # recordings that start moving within ROUND_LEAD_S + this keep offset 0 (unchanged)


def motion_profile(bag, refresh=False):
    """1 Hz speed / yaw / distance profile of a recording from the INS odometry (plus IMU sample gaps), relative to
    the dataset start (the player's --start-offset origin). Cached per recording under results/.studio/motion/."""
    import hashlib
    from livo.dataset import Dataset
    from scipy.spatial.transform import Rotation as R
    ds = Dataset.open(bag)
    key = hashlib.sha1(f"{pathlib.Path(bag).resolve()}|{ds.start_ns}|{ds.end_ns}".encode()).hexdigest()[:16]
    cache = MOTION_CACHE / f"{key}.json"
    if cache.exists() and not refresh:
        try:
            return json.loads(cache.read_text())
        except Exception:  # noqa: BLE001
            pass
    t0 = ds.start_ns * 1e-9
    t, p, yaw, ti = [], [], [], []
    for topic, m, _ts in ds.read_messages(ds.resolve_topics(["imu_data/odometry", "imu_data"])):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t0
        if topic.endswith("odometry"):
            q = m.pose.pose.orientation
            t.append(st)
            p.append([m.pose.pose.position.x, m.pose.pose.position.y])
            yaw.append(R.from_quat([q.x, q.y, q.z, q.w]).as_euler("zyx")[0])
        else:
            ti.append(st)
    if len(t) < 100:
        sys.exit(f"windows: {bag} has no usable INS odometry (imu_data/odometry)")
    t, p, yaw = np.array(t), np.array(p), np.unwrap(np.array(yaw))
    step = np.r_[0.0, np.linalg.norm(np.diff(p, axis=0), axis=1)]
    dist = np.cumsum(step)
    secs = np.arange(0, int(np.ceil(t[-1])) + 1)
    d1 = np.interp(secs, t, dist)
    y1 = np.interp(secs, t, yaw)
    speed = np.r_[np.diff(d1), 0.0]
    dyaw = np.r_[np.abs(np.diff(y1)), 0.0]
    dti = np.diff(np.asarray(ti)) if len(ti) > 1 else np.array([])
    gaps = [{"t_s": round(float(ti[i]), 2), "gap_ms": round(float(1000 * dti[i]), 1)} for i in np.flatnonzero(dti > 0.025)]
    prof = {"bag": str(bag), "duration_s": round(float(t[-1]), 2), "path_m": round(float(dist[-1]), 1),
            "speed_mps": [round(float(v), 3) for v in speed], "yaw_rate_dps": [round(float(np.degrees(v)), 3) for v in dyaw],
            "imu_gaps": gaps, "imu_median_dt_ms": round(float(1000 * np.median(dti)), 2) if len(dti) else None}
    try:
        MOTION_CACHE.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(prof) + "\n")
    except OSError:
        pass
    return prof


def _straight(v, w, s, min_mps=1.5, max_dps=2.0):
    """4 s starting at second s are straight driving (ground stage moving frames)."""
    return s + 4 <= len(v) and float(v[s:s + 4].min()) >= min_mps and float(np.degrees(w[s:s + 4]).max()) <= max_dps


def suggest_windows(prof, round_duration=60.0):
    """Stationary / driving windows from a motion profile.

    round_start: every calibration round (LIO for the driven stage, LIVO for the camera loop) starts ROUND_LEAD_S
    before the standstill->driving transition whose round_duration window holds the most driving (distance + turns),
    so the mapper still initialises at rest. Recordings that start moving within the first ~13 s keep offset 0 (the
    behaviour validated on truck29). The ground stage gets its stationary frames from the standstill before that
    transition and its moving frames from a straight drive (the default 30-34 s when that is one, otherwise the
    straightest 4 s at >= 1.5 m/s after the first drive; front LiDAR frame indices at 10 Hz)."""
    v = np.asarray(prof["speed_mps"], float)
    w = np.radians(np.asarray(prof["yaw_rate_dps"], float))
    n = len(v)
    hold = int(MOVING_HOLD_S)
    moving = np.array([v[i] > MOVING_MPS and (v[i:i + hold] > 0.3).all() for i in range(n)])
    rest = v < 0.1
    # standstill -> driving transitions only (>= 4 s at rest right before): the round must start at rest
    starts = [i for i in range(5, n) if moving[i] and not moving[i - 1] and rest[i - 5:i - 1].all()]
    if moving[:1].any() or not starts:  # moving right away (or never resting): the validated offset-0 round
        starts = [0] + starts
    cands = []
    for s in starts:
        rs = max(0.0, s - ROUND_LEAD_S)
        if rs < MIN_SHIFT_S:
            rs = 0.0
        a, b = int(rs), int(min(n, rs + round_duration))
        score = float(v[a:b].sum() + 20.0 * np.degrees(w[a:b]).sum() / 90.0)  # 90 deg of turning ~ 20 m of driving
        cands.append({"motion_start_s": float(s), "round_start_s": rs, "driving_m": round(float(v[a:b].sum()), 1),
                      "turning_deg": round(float(np.degrees(w[a:b]).sum()), 1), "score": round(score, 1)})
    best = max(cands, key=lambda c: c["score"]) if cands else {"motion_start_s": 0.0, "round_start_s": 0.0,
                                                                  "driving_m": 0.0, "turning_deg": 0.0, "score": 0.0}
    ms, rs = best["motion_start_s"], best["round_start_s"]
    still = np.flatnonzero(v[: int(ms)] < STATIONARY_MPS) if ms > 0 else np.array([], int)
    stat_first = int(still[0]) if len(still) else 0
    stationary = (max(2, 10 * stat_first + 2), 20)
    moving_frames = (300, 40)  # validated default (30-34 s); must be straight driving: the back LiDAR is solved
    if not _straight(v, w, 30):  # per frame, and a hard turn (lever arm, deskew) biases its roll/pitch by ~1 deg
        straight = [s for s in range(int(ms), n - 4) if _straight(v, w, s)]
        if straight:
            s_best = min(straight, key=lambda s: (float(np.degrees(w[s:s + 4]).max()), s))
            moving_frames = (10 * s_best, 40)
    gaps = [g for g in prof.get("imu_gaps", []) if rs <= g["t_s"] <= rs + round_duration]
    return {"round_start_s": rs, "round_duration_s": round_duration, "motion_start_s": ms,
            "camera_time_start_s": max(0.0, ms - 2.0) if rs > 0 else 0.0,
            "ground_stationary_frames": list(stationary), "ground_moving_frames": list(moving_frames),
            "driving_m": best["driving_m"], "turning_deg": best["turning_deg"], "candidates": cands,
            "imu_gaps_in_round": gaps, "max_imu_gap_ms": max([g["gap_ms"] for g in prof.get("imu_gaps", [])], default=0.0),
            "stationary_head_s": float(ms) if rs > 0 else 0.0}


MANUAL_LEAD_MAX_S = 60.0  # a manual window starting mid-drive backs playback up to a standstill at most this far


def manual_window(prof, start_s, duration_s=None):
    """Stage/round windows inside an operator-chosen segment [start, start+duration] (seconds from the bag start).

    The calibration data stay inside the segment; only the playback of the mapping rounds may begin earlier:
    when the segment starts while driving, the round starts at the last standstill (<= MANUAL_LEAD_MAX_S before),
    so the mapper initialises at rest and has converged when the segment begins (lead_in_s). The driven stage then
    uses [lead_in_s, lead_in_s + duration] of the round. Ground stationary frames come from a standstill inside the
    segment, else the nearest one before it; moving frames from the straightest 4 s drive inside the segment."""
    v = np.asarray(prof["speed_mps"], float)
    w = np.radians(np.asarray(prof["yaw_rate_dps"], float))
    n = len(v)
    lo = int(min(max(0.0, float(start_s)), max(0, n - 1)))
    hi = n if not duration_s else int(min(n, lo + float(duration_s)))
    rest = v < 0.1
    warnings = []
    rs = float(lo)
    if not rest[lo:lo + 2].all():
        ends = [j for j in range(lo, max(3, int(lo - MANUAL_LEAD_MAX_S)) - 1, -1) if j >= 4 and rest[j - 4:j].all()]
        if ends:
            rs = float(max(0, ends[0] - 4))
        elif lo > 0:
            warnings.append(f"窗口起点在行驶中，前 {MANUAL_LEAD_MAX_S:.0f} s 内无静止段：建图将在运动中初始化")
    still_in = [i for i in range(lo, max(lo, hi - 2)) if v[i:i + 2].max() < STATIONARY_MPS]
    still_before = [i for i in range(lo - 2, -1, -1) if v[i:i + 2].max() < STATIONARY_MPS]
    if still_in:
        stationary = (10 * still_in[0] + 2, 20)
    elif still_before:
        stationary = (10 * still_before[0] + 2, 20)
        warnings.append(f"窗口内无静止段：地面静止帧取窗口前 {lo - still_before[0]} s 的静止段")
    else:
        stationary = (10 * lo + 2, 20)
        warnings.append("整段录包无静止段：ground 阶段的静止地面拟合不可信")
    straight = [t for t in range(lo, max(lo, hi - 4)) if _straight(v, w, t)]
    if 30 in straight:  # validated default (30-34 s) when it lies in the segment
        moving_frames = (300, 40)
    elif straight:
        t_best = min(straight, key=lambda t: (float(np.degrees(w[t:t + 4]).max()), t))
        moving_frames = (10 * t_best, 40)
    else:
        moving_frames = (10 * min(lo + 10, max(lo, hi - 4)), 40)
        warnings.append("窗口内无 4 s 直线匀速段：ground 阶段运动帧可能不在直线上")
    lead = float(lo) - rs
    return {"manual": True, "window_start_s": float(lo), "window_end_s": float(hi), "lead_in_s": lead,
            "round_start_s": rs, "round_duration_s": (hi - lo) + lead if duration_s else 0,
            "motion_start_s": float(lo), "camera_time_start_s": float(lo),
            "ground_stationary_frames": list(stationary), "ground_moving_frames": list(moving_frames),
            "driving_m": round(float(v[lo:hi].sum()), 1), "turning_deg": round(float(np.degrees(w[lo:hi]).sum()), 1),
            "warnings": warnings}


def cmd_windows(a):
    prof = motion_profile(a.bag, refresh=a.refresh)
    if a.start is not None:
        win = manual_window(prof, a.start, a.duration)
        for msg in win["warnings"]:
            print(f"warning: {msg}", flush=True)
    else:
        win = suggest_windows(prof, round_duration=a.round_duration)
    win.update({"bag": str(a.bag), "duration_s": prof["duration_s"], "path_m": prof["path_m"]})
    print(json.dumps(win, ensure_ascii=False))


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
                        scale=0.5, max_rot_deg=2.0, max_trans_m=0.15, solve_intrinsics=False, dof="yawroll"):
    """Align LiDAR depth edges with image edges.

    dof="yawroll" (default): only vertical structure edges (poles, building corners, trunks) are used and only
    the camera yaw (about camera y) and roll (about camera z) are solved. Horizontal edges are NOT usable:
    a 36-line LiDAR samples the top of an object up to one beam pitch below its true edge, so matching them
    to image edges pulls the solution in pitch by ~0.2 deg in a repeatable direction (v006/v015/v017 all
    did that and the closed loop rejected them). Camera pitch is determined by the closed-loop scan instead.
    dof="full": legacy behaviour (all edges, 6-DoF), kept for diagnostics only.
    """
    import cv2
    vertical_only = dof != "full"
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
        # vertical-only: compare horizontal neighbours only (azimuth resolution 0.1-0.2 deg is fine);
        # vertical neighbours would add the coarse ring-quantised object tops/bottoms
        neighbours = ((0, 1), (0, -1)) if vertical_only else ((0, 1), (1, 0), (0, -1), (-1, 0))
        for dy, dx in neighbours:
            nb = np.roll(np.roll(zbuf, dy, 0), dx, 1)
            edge |= (np.isfinite(zbuf) & np.isfinite(nb) & (nb - zbuf > np.maximum(0.6, 0.05 * zbuf)))
        e_idx = idx[edge]
        e_idx = e_idx[e_idx >= 0]
        edge_pts = pr[e_idx]
        img = images[k]
        blur = cv2.GaussianBlur(img, (3, 3), 0)
        canny = cv2.Canny(blur, 60, 160)
        if vertical_only:  # keep edge pixels whose gradient is mostly horizontal (i.e. vertical structures)
            gx = cv2.Sobel(blur, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(blur, cv2.CV_32F, 0, 1, ksize=3)
            canny = np.where(np.abs(gx) > 2.0 * np.abs(gy), canny, 0).astype(np.uint8)
        # keep only long edge chains: foliage/gravel texture produces short fragments that make
        # the distance field flat and uninformative
        n_cc, labels, stats, _ = cv2.connectedComponentsWithStats((canny > 0).astype(np.uint8), connectivity=8)
        long_cc = np.zeros(n_cc, bool)
        if vertical_only:
            long_cc[1:] = stats[1:, cv2.CC_STAT_HEIGHT] >= 25
        else:
            long_cc[1:] = np.maximum(stats[1:, cv2.CC_STAT_WIDTH], stats[1:, cv2.CC_STAT_HEIGHT]) >= 25
        canny = np.where(long_cc[labels], 255, 0).astype(np.uint8)
        dt_img = cv2.distanceTransform(255 - canny, cv2.DIST_L2, 3)
        dt_img = np.minimum(dt_img, 10.0)
        frames_data.append({"j": j, "k": k, "edge_pts": edge_pts, "dt": dt_img, "img": img, "canny": canny,
                            "speed": float(speed[j])})

    def score(delta, T_base=T_rear_cam, return_parts=False):
        rv, tv = np.asarray(delta[:3], float).copy(), np.asarray(delta[3:6], float).copy()
        if vertical_only:  # pitch and translation are not observable from vertical edges: keep them fixed
            rv[0] = 0.0
            tv[:] = 0.0
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
    rx_grid = [0.0] if vertical_only else grid
    for ry in grid:      # yaw-like about camera y
        for rx in rx_grid:  # pitch-like about camera x
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
    if vertical_only:
        x = np.array([0.0, x[1], x[2], 0.0, 0.0, 0.0])
    after, after_parts = score(x, return_parts=True)
    # focal-length scan: for each fx scale (fy fixed) re-optimise the rotation and record the score
    focal_scan = []
    for sf in (0.92, 0.96, 1.0, 1.04, 1.08, 1.12, 1.16):
        bx, bs = None, np.inf
        for ry in np.arange(-0.6, 0.601, 0.1):
            for rx in ([0.0] if vertical_only else np.arange(-0.6, 0.601, 0.1)):
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
    if solve_intrinsics and res8.fun < after and not vertical_only:
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
               "intrinsics_solved": bool(solve_intrinsics and not vertical_only and res8.fun < after + 1e-9), "image_scale": scale,
               "dof": "yaw+roll (vertical edges, pitch fixed)" if vertical_only else "6-DoF (all edges)",
               "frame_speeds_mps": [round(f["speed"], 2) for f in frames_data],
               "overlay_dir": str(overlays)}
    return T_new, metrics


def accumulate_rear(fr, rot, stamps, j, half_window):
    """Scans within +-half_window s of run frame j, expressed in the rear-axle frame of frame j."""
    from scipy.spatial.transform import Rotation as R  # noqa: F401
    T_imu_rear = np.eye(4)
    T_imu_rear[:3, 3] = fr["t_li"]

    def T_world_imu(i):
        T = np.eye(4)
        T[:3, :3] = rot[i].as_matrix()
        T[:3, 3] = fr["pos"][i]
        return T

    sel = np.flatnonzero(np.abs(stamps - stamps[j]) <= half_window)
    pts = []
    for i in sel:
        o, c = (fr["offset"][i] - 8) // 20, fr["count"][i]
        p = np.asarray(fr["archive"][o:o + c, :3], np.float64)
        p = p[np.isfinite(p).all(1)]
        Tw = T_world_imu(i) @ T_imu_rear
        pts.append(p @ Tw[:3, :3].T + Tw[:3, 3])
    pw = np.vstack(pts)
    T_rear_world = np.linalg.inv(T_world_imu(j) @ T_imu_rear)
    return pw @ T_rear_world[:3, :3].T + T_rear_world[:3, 3]


def find_poles(p_rear, cell=0.3, min_height=3.0, max_radius=0.22, max_range=45.0):
    """Thin vertical clusters (lamp / sign posts) in a rear-axle-frame cloud (z up).
    Returns [(bottom xyz, top xyz, radius_m, n_points)]."""
    r_xy = np.hypot(p_rear[:, 0], p_rear[:, 1])
    p = p_rear[(r_xy > 3.0) & (r_xy < max_range)]
    if len(p) < 100:
        return []
    ij = np.floor(p[:, :2] / cell).astype(np.int64)
    key = ij[:, 0] * 1_000_003 + ij[:, 1]
    order = np.argsort(key)
    key_s, p_s = key[order], p[order]
    bounds = np.flatnonzero(np.diff(key_s)) + 1
    starts = np.concatenate([[0], bounds])
    ends = np.concatenate([bounds, [len(key_s)]])
    poles = []
    for a, b in zip(starts, ends):
        if b - a < 12:
            continue
        q = p_s[a:b]
        z0, z1 = q[:, 2].min(), q[:, 2].max()
        if z1 - z0 < min_height:
            continue
        cx, cy = np.median(q[:, 0]), np.median(q[:, 1])
        # shaft points: everything above the lowest metre, within 0.6 m of the axis, taken from the full cloud
        near = p[(np.hypot(p[:, 0] - cx, p[:, 1] - cy) < 0.6) & (p[:, 2] > z0 + 1.0)]
        if len(near) < 12:
            continue
        axis = np.median(near[:, :2], axis=0)
        rad = np.hypot(near[:, 0] - axis[0], near[:, 1] - axis[1])
        if np.percentile(rad, 80) > max_radius:
            continue  # tree trunk / bush / wall
        # isolation: few points in the 0.6-1.8 m annulus above 1.5 m
        ann = p[(np.hypot(p[:, 0] - axis[0], p[:, 1] - axis[1]) < 1.8) & (p[:, 2] > z0 + 1.5)]
        if len(ann) > 2.0 * len(near) + 20:
            continue
        zt = np.percentile(near[:, 2], 98)
        if zt - z0 < min_height:
            continue
        if any(np.hypot(axis[0] - q0[0], axis[1] - q0[1]) < 1.5 for q0, _, _, _ in poles):
            continue
        poles.append((np.array([axis[0], axis[1], z0 + 0.5]), np.array([axis[0], axis[1], zt]),
                      float(np.percentile(rad, 80)), int(len(near))))
    return poles


def measure_pole_column(img, u_axis, v_rows, bar_px, search=40):
    """Sub-pixel image column of a thin vertical bar near the projected LiDAR axis.
    Per row a zero-mean bar template (either polarity) is slid over +-search px; the bar width is unknown
    (the LiDAR radius of an accumulated pole is inflated by pose noise), so several widths are tried and
    the one whose rows agree best is kept. Returns {"du","mad","rows"} (offset image - lidar, px) or {"reason"}."""
    h, w = img.shape
    widths = sorted({3, 5, 7, 10, 14, 19, max(3, int(round(bar_px)))})
    rows_ok = []
    for v, u0 in zip(v_rows, u_axis):
        v = int(round(v))
        lo, hi = int(round(u0)) - search - 60, int(round(u0)) + search + 61
        if v < 0 or v >= h or lo < 0 or hi > w:
            continue
        prof = img[v, lo:hi].astype(np.float32)
        if prof.std() < 4.0:
            continue
        rows_ok.append((prof, lo - u0, float(prof.std()), float(v)))
    if len(rows_ok) < 8:
        return {"reason": f"only {len(rows_ok)} textured rows inside the image"}
    best = None
    for bw in widths:
        half = bw // 2
        margin = max(3, bw)
        offs, resps, v_used = [], [], []
        for prof, base, std, v_row in rows_ok:
            cs = np.concatenate([[0.0], np.cumsum(prof)])
            uc = np.arange(margin + half, len(prof) - margin - half - 1)
            inner = (cs[uc + half + 1] - cs[uc - half]) / (2 * half + 1)
            left = (cs[uc - half] - cs[uc - half - margin]) / margin
            right = (cs[uc + half + 1 + margin] - cs[uc + half + 1]) / margin
            resp = np.abs(inner - 0.5 * (left + right)) - 0.5 * np.abs(left - right)
            # restrict to the +-search window around the LiDAR axis
            u_rel = uc + base
            mask = np.abs(u_rel) <= search
            if not mask.any():
                continue
            i = np.argmax(np.where(mask, resp, -np.inf))
            if resp[i] < 1.5 * std:
                continue
            offs.append(u_rel[i])
            resps.append(resp[i] / std)
            v_used.append(v_row)
        if len(offs) < 8:
            continue
        offs = np.array(offs)
        med = float(np.median(offs))
        mad = float(np.median(np.abs(offs - med)))
        score = len(offs) / (1.0 + mad)
        if best is None or score > best["score"]:
            best = {"du": med, "mad": mad, "rows": len(offs), "width": bw, "score": score,
                    "snr": round(float(np.median(resps)), 2),
                    "v_rows": [float(x) for x in v_used], "row_offs": [float(x) for x in offs]}
    if best is None:
        return {"reason": f"no bar response in {len(rows_ok)} rows"}
    if best["mad"] > 3.0:
        return {"reason": f"rows disagree (MAD {best['mad']:.1f} px over {best['rows']} rows, width {best['width']} px)"}
    return best


def pole_yaw_refine(calib, bag, run, out_dir, n_frames=30, window=(5.0, 115.0), accumulate=1.0, scale=1.0):
    """Camera yaw (rotation about camera y) from lamp / sign posts: the LiDAR pole axis is projected with the
    current extrinsic and compared with the pole's column in the image. Poles are the only structures in a
    mine road scene that both sensors see sharply; a 36-line LiDAR resolves them in azimuth (0.1-0.2 deg),
    while their horizontal image position is measured to ~1 px = 0.035 deg. Pitch is left to the closed loop."""
    import cv2
    from scipy.spatial.transform import Rotation as R
    fr = read_run_frames(run)
    stamps, rot = fr["stamp"], R.from_quat(fr["quat"])
    speed = np.linalg.norm(np.gradient(fr["pos"], stamps, axis=0), axis=1)
    cam = calib.get("camera", DEFAULT_CAMERA)
    fx, fy, cx, cy = cam["intrinsics"]
    K = np.array([[fx * scale, 0, cx * scale], [0, fy * scale, cy * scale], [0, 0, 1]])
    dist = np.array(cam.get("distortion", [0] * 5), float)
    T_rear_cam = np.asarray(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    T_cam_rear = np.linalg.inv(T_rear_cam)
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
    # find poles first so only frames that see a pole are decoded
    per_frame = []
    for j, k in targets:
        p_rear = accumulate_rear(fr, rot, stamps, j, accumulate / 2)
        poles = []
        for bottom, top, rad, n in find_poles(p_rear):
            seg = np.linspace(bottom, top, 40)
            pc = seg @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]
            if pc[:, 2].min() < 3.0:
                continue
            uv = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
            w_img, h_img = int(1920 * scale), int(1080 * scale)
            inside = (uv[:, 0] > 60) & (uv[:, 0] < w_img - 60) & (uv[:, 1] > 5) & (uv[:, 1] < h_img - 5)
            if inside.sum() < 20:
                continue
            poles.append({"bottom": bottom, "top": top, "radius": rad, "n": n, "uv": uv[inside],
                          "depth": float(np.median(pc[inside, 2]))})
        if poles:
            per_frame.append((j, k, poles))
    if not per_frame:
        stream.unlink(missing_ok=True)
        return None, {"error": "no lamp/sign posts found in the accumulated scans", "frames_checked": len(targets)}
    images = {k: img for k, img in decode_hevc(stream, scale, keep={k for _, k, _ in per_frame})}
    stream.unlink(missing_ok=True)
    # pixel shift produced by +0.1 deg about camera y (sign convention of `set --camera-rot-delta`)
    P = np.eye(4)
    P[:3, :3] = R.from_euler("xyz", [0, 0.1, 0], degrees=True).as_matrix()
    T_probe = np.linalg.inv(T_rear_cam @ P)

    def du_per_deg(pt_rear):
        a = cv2.projectPoints((pt_rear @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]).reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].ravel()
        b = cv2.projectPoints((pt_rear @ T_probe[:3, :3].T + T_probe[:3, 3]).reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].ravel()
        return (b[0] - a[0]) / 0.1

    obs, candidates = [], []
    dbg = out_dir / "overlays"
    dbg.mkdir(exist_ok=True)
    n_dbg = 0
    for j, k, poles in per_frame:
        img = images.get(k)
        if img is None:
            continue
        for pole in poles:
            uv = pole["uv"]
            rows = np.arange(int(np.ceil(uv[:, 1].min())), int(np.floor(uv[:, 1].max())))
            if len(rows) < 12:
                continue
            u_axis = np.interp(rows, uv[np.argsort(uv[:, 1]), 1], uv[np.argsort(uv[:, 1]), 0])
            bar_px = max(2.0, fx * scale * max(0.14, 2 * pole["radius"]) / pole["depth"])
            m = measure_pole_column(img, u_axis, rows, bar_px)
            if "du" not in m:
                candidates.append({"t": round(float(stamps[j] - t0), 1), "depth_m": round(pole["depth"], 1),
                                   "bar_px": round(bar_px, 1), "rows": int(len(rows)), "rejected": m["reason"]})
                continue
            du, mad, nrows = m["du"], m["mad"], m["rows"]
            pole["width_px"] = m["width"]
            mid = 0.5 * (pole["bottom"] + pole["top"])
            g = du_per_deg(mid)
            # roll (about camera z): a rolled camera slants the pole column, du grows linearly with the
            # image row; regress du(v) robustly and convert with a numeric probe (sign-convention safe).
            roll_obs = None
            v_rows = np.asarray(m.get("v_rows", ()), float)
            row_offs = np.asarray(m.get("row_offs", ()), float)
            if len(v_rows) >= 40 and float(v_rows.max() - v_rows.min()) >= 100.0:
                rng = np.random.default_rng(0)
                ii = rng.choice(len(v_rows), size=min(len(v_rows), 240), replace=False)
                vv, oo = v_rows[ii], row_offs[ii]
                dv = vv[:, None] - vv[None, :]
                do = oo[:, None] - oo[None, :]
                far = np.abs(dv) >= 30.0
                if far.sum() >= 50:
                    slope = float(np.median(do[far] / dv[far]))
                    Pz = np.eye(4)
                    Pz[:3, :3] = R.from_euler("xyz", [0, 0, 0.1], degrees=True).as_matrix()
                    Tz = np.linalg.inv(T_rear_cam @ Pz)
                    ends = np.vstack([pole["bottom"], pole["top"]])
                    pa = cv2.projectPoints((ends @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]).reshape(-1, 1, 3),
                                           np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
                    pb = cv2.projectPoints((ends @ Tz[:3, :3].T + Tz[:3, 3]).reshape(-1, 1, 3),
                                           np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
                    dv_ends = pa[1, 1] - pa[0, 1]
                    if abs(dv_ends) > 1.0:
                        slope_per_deg = (((pb[1, 0] - pa[1, 0]) - (pb[0, 0] - pa[0, 0])) / dv_ends) / 0.1
                        if abs(slope_per_deg) > 1e-4:
                            roll_obs = float(slope / slope_per_deg)
            obs.append({"t": round(float(stamps[j] - t0), 1), "image": int(k), "depth_m": round(pole["depth"], 1),
                        "height_m": round(float(pole["top"][2] - pole["bottom"][2] + 0.5), 1), "rows": int(nrows),
                        "du_px": round(du, 2), "mad_px": round(mad, 2), "yaw_deg": round(float(du / g), 4),
                        "bar_width_px": int(m["width"]), "snr": m["snr"], "speed_mps": round(float(speed[j]), 2),
                        "roll_deg": (round(roll_obs, 4) if roll_obs is not None else None),
                        "u_px": round(float(np.median(u_axis)), 1)})
            if n_dbg < 8:
                u_c = int(np.median(u_axis))
                v0, v1 = max(0, rows[0] - 20), min(img.shape[0], rows[-1] + 20)
                u0, u1 = max(0, u_c - 80), min(img.shape[1], u_c + 80)
                crop = cv2.cvtColor(img[v0:v1, u0:u1], cv2.COLOR_GRAY2BGR)
                for v, u in zip(rows, u_axis):
                    cv2.circle(crop, (int(round(u - u0)), int(v - v0)), 1, (0, 0, 255), -1)
                    cv2.circle(crop, (int(round(u + du - u0)), int(v - v0)), 1, (0, 255, 0), -1)
                crop = cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
                cv2.putText(crop, f"t+{stamps[j] - t0:.0f}s d={pole['depth']:.0f}m du={du:+.1f}px", (4, 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imwrite(str(dbg / f"pole_{n_dbg:02d}_t{int(stamps[j] - t0):03d}.jpg"), crop, [cv2.IMWRITE_JPEG_QUALITY, 88])
                n_dbg += 1
    metrics = {"method": "pole yaw (LiDAR pole axis vs image pole column)", "frames_checked": len(targets),
               "frames_with_poles": len(per_frame), "observations": obs, "rejected": candidates, "image_scale": scale}
    if len(obs) < 3:
        metrics["error"] = f"only {len(obs)} usable pole observations"
        return None, metrics
    yaws = np.array([o["yaw_deg"] for o in obs])
    med = float(np.median(yaws))
    mad = float(np.median(np.abs(yaws - med)))
    keep = np.abs(yaws - med) <= max(3 * mad, 0.05)
    yaw = float(np.median(yaws[keep]))
    rolls = np.array([o["roll_deg"] for o in obs if o.get("roll_deg") is not None], float)
    roll = 0.0
    if len(rolls) >= 3:
        r_med = float(np.median(rolls))
        r_mad = float(np.median(np.abs(rolls - r_med)))
        r_keep = np.abs(rolls - r_med) <= max(3 * r_mad, 0.05)
        if int(r_keep.sum()) >= 3 and r_mad <= 0.08:
            roll = float(np.median(rolls[r_keep]))
        metrics.update({"roll_delta_deg": round(roll, 4), "roll_mad_deg": round(r_mad, 4),
                        "roll_used": int(r_keep.sum()), "roll_observations": int(len(rolls))})
    else:
        metrics.update({"roll_delta_deg": 0.0, "roll_observations": int(len(rolls)),
                        "roll_note": "fewer than 3 poles with enough row span; roll kept"})
    metrics.update({"yaw_delta_deg": round(yaw, 4), "yaw_mad_deg": round(mad, 4), "used": int(keep.sum()),
                    "du_median_px": round(float(np.median([o["du_px"] for o in obs])), 2),
                    "delta_rot_deg_cam_xyz": [0.0, round(yaw, 4), round(roll, 4)], "delta_trans_m_cam_xyz": [0.0, 0.0, 0.0],
                    "overlay_dir": str(dbg)})
    P = np.eye(4)
    P[:3, :3] = R.from_euler("xyz", [0, yaw, roll], degrees=True).as_matrix()
    return T_rear_cam @ P, metrics


def camera_focus_refine(calib, bag, run, out_dir, n_pairs=36, window=(5.0, 115.0), scale=1.0):
    """Solve fx/fy scale and radial distortion k1 from optical flow against LiDAR depth + trajectory.

    Between two consecutive images the camera pose change is known from the run trajectory and each
    tracked feature's depth from the LiDAR points projected into the first image. The rotational part
    of the predicted flow scales directly with fx/fy and radial distortion bends it towards the image
    border, so the intrinsics are observable without any target. Differencing consecutive frames
    cancels (to first order) a constant extrinsic error, a camera time-offset error and the rolling
    shutter row skew, so this probe is orthogonal to the closed-loop pitch and the pole yaw. The
    36-line LiDAR edge sampling bias plays no role: only features inside smooth depth patches are used.
    """
    import cv2
    from scipy.spatial.transform import Rotation as R
    from scipy.optimize import least_squares
    fr = read_run_frames(run)
    stamps, rot = fr["stamp"], R.from_quat(fr["quat"])
    speed = np.linalg.norm(np.gradient(fr["pos"], stamps, axis=0), axis=1)
    cam = calib.get("camera", DEFAULT_CAMERA)
    fx0, fy0, cx0, cy0 = [v * scale for v in cam["intrinsics"]]
    dist0 = np.array(cam.get("distortion", [0] * 5), float)
    T_rear_cam = np.asarray(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    T_cam_rear = np.linalg.inv(T_rear_cam)
    T_imu_rear = np.eye(4)
    T_imu_rear[:3, 3] = fr["t_li"]
    cam_offset = calib.get("time_offsets", {}).get("camera", 0.0)
    img_stamps, stream = dump_camera_stream(bag, out_dir)
    eff = img_stamps + cam_offset
    t0 = stamps[0]
    pairs = []
    for tq in np.linspace(t0 + window[0], min(t0 + window[1], stamps[-1] - 1.0), n_pairs):
        k = int(np.argmin(np.abs(eff - tq)))
        if k + 1 >= len(eff) or eff[k + 1] - eff[k] > 0.35:
            continue
        j1 = int(np.argmin(np.abs(stamps - eff[k])))
        j2 = int(np.argmin(np.abs(stamps - eff[k + 1])))
        if abs(stamps[j1] - eff[k]) > 0.02 or abs(stamps[j2] - eff[k + 1]) > 0.02 or j1 == j2:
            continue
        if not (1.0 <= speed[j1] <= 8.5):
            continue
        pairs.append((k, j1, j2))
    if len(pairs) < 6:
        stream.unlink(missing_ok=True)
        return None, {"error": f"only {len(pairs)} usable consecutive image pairs"}
    keep = set()
    for k, _, _ in pairs:
        keep.update((k, k + 1))
    images = {k: img for k, img in decode_hevc(stream, scale, keep=keep)}
    stream.unlink(missing_ok=True)

    def T_world_cam(j):
        T = np.eye(4)
        T[:3, :3] = rot[j].as_matrix()
        T[:3, 3] = fr["pos"][j]
        return T @ T_imu_rear @ T_rear_cam

    K0 = np.array([[fx0, 0, cx0], [0, fy0, cy0], [0, 0, 1]])
    data, Ts, pair_meta = [], [], []
    for k, j1, j2 in pairs:
        im1, im2 = images.get(k), images.get(k + 1)
        if im1 is None or im2 is None:
            continue
        p_rear = accumulate_rear(fr, rot, stamps, j1, 0.45)
        pc = p_rear @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]
        pc = pc[(pc[:, 2] > 3.0) & (pc[:, 2] < 70.0)]
        if len(pc) < 500:
            continue
        uvz = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K0, dist0)[0].reshape(-1, 2)
        h, w = im1.shape
        gs = 6.0
        ok = (uvz[:, 0] >= 0) & (uvz[:, 0] < w) & (uvz[:, 1] >= 0) & (uvz[:, 1] < h)
        depth = {}
        for a_, b_, z_ in zip(np.round(uvz[ok, 0] / gs).astype(int), np.round(uvz[ok, 1] / gs).astype(int), pc[ok, 2]):
            key = (a_, b_)
            if key not in depth or z_ < depth[key]:
                depth[key] = z_
        feats = cv2.goodFeaturesToTrack(im1, maxCorners=500, qualityLevel=0.01, minDistance=10)
        if feats is None:
            continue
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(im1, im2, feats, None, winSize=(21, 21), maxLevel=3)
        back, st2, _ = cv2.calcOpticalFlowPyrLK(im2, im1, nxt, None, winSize=(21, 21), maxLevel=3)
        fb = np.linalg.norm(back - feats, axis=2).ravel()
        good = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 0.5)
        n_before = len(data)
        for f, g in zip(feats[good].reshape(-1, 2), nxt[good].reshape(-1, 2)):
            a_, b_ = int(round(f[0] / gs)), int(round(f[1] / gs))
            zs = [depth[(a_ + da, b_ + db)] for da in (-1, 0, 1) for db in (-1, 0, 1) if (a_ + da, b_ + db) in depth]
            if len(zs) < 4:
                continue
            zmed = float(np.median(zs))
            if (max(zs) - min(zs)) / zmed > 0.06:
                continue
            data.append((f[0], f[1], g[0], g[1], zmed, len(Ts)))
        T1, T2 = T_world_cam(j1), T_world_cam(j2)
        rel = np.linalg.inv(T2) @ T1
        Ts.append(rel)
        rot_deg = float(np.linalg.norm(R.from_matrix(rel[:3, :3]).as_rotvec())) * 57.2958
        pair_meta.append({"t": round(float(stamps[j1] - t0), 1), "speed": round(float(speed[j1]), 2),
                          "rot_deg": round(rot_deg, 3), "corrs": len(data) - n_before})
    D = np.asarray(data, float)
    if len(D) < 400:
        return None, {"error": f"only {len(D)} flow/depth correspondences", "pairs": pair_meta}
    u1, v1, u2, v2, Z, pidx = D[:, 0], D[:, 1], D[:, 2], D[:, 3], D[:, 4], D[:, 5].astype(int)
    Rs = np.array([T[:3, :3] for T in Ts])
    ts = np.array([T[:3, 3] for T in Ts])

    def residual(p, sel=None):
        sfx, sfy, k1 = p
        K = np.array([[fx0 * sfx, 0, cx0], [0, fy0 * sfy, cy0], [0, 0, 1]])
        dd = np.array([k1, 0, 0, 0, 0], float)
        m = np.ones(len(u1), bool) if sel is None else np.isin(pidx, sel)
        pts = np.stack([u1[m], v1[m]], 1).reshape(-1, 1, 2)
        xn = cv2.undistortPoints(pts, K, dd).reshape(-1, 2)
        Pc1 = np.stack([xn[:, 0] * Z[m], xn[:, 1] * Z[m], Z[m]], 1)
        Pc2 = np.einsum("nij,nj->ni", Rs[pidx[m]], Pc1) + ts[pidx[m]]
        pred = cv2.projectPoints(Pc2.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dd)[0].reshape(-1, 2)
        return np.concatenate([pred[:, 0] - u2[m], pred[:, 1] - v2[m]])

    res0 = residual([1.0, 1.0, 0.0])
    sol = least_squares(residual, [1.0, 1.0, 0.0], loss="huber", f_scale=1.0,
                        x_scale=[0.01, 0.01, 0.01], max_nfev=80)
    res1 = residual(sol.x)
    rng = np.random.default_rng(1)
    boots = []
    n_pair_used = len(Ts)
    for _ in range(40):
        sel = rng.choice(n_pair_used, max(4, int(0.7 * n_pair_used)), replace=False)
        try:
            b = least_squares(residual, sol.x, args=(sel,), loss="huber", f_scale=1.0,
                              x_scale=[0.01, 0.01, 0.01], max_nfev=40)
            boots.append(b.x)
        except Exception:
            pass
    boots = np.array(boots) if boots else sol.x.reshape(1, 3)
    lo, hi = np.percentile(boots, [2.5, 97.5], axis=0)
    metrics = {"method": "optical flow x LiDAR depth (consecutive frames; extrinsic/time/rolling-shutter cancel)",
               "pairs_used": n_pair_used, "correspondences": int(len(D)),
               "fx_scale": round(float(sol.x[0]), 5), "fy_scale": round(float(sol.x[1]), 5),
               "k1": round(float(sol.x[2]), 5),
               "ci95": {"fx_scale": [round(float(lo[0]), 5), round(float(hi[0]), 5)],
                         "fy_scale": [round(float(lo[1]), 5), round(float(hi[1]), 5)],
                         "k1": [round(float(lo[2]), 5), round(float(hi[2]), 5)]},
               "residual_px_median_before": round(float(np.median(np.abs(res0))), 3),
               "residual_px_median_after": round(float(np.median(np.abs(res1))), 3),
               "intrinsics_before": [v / scale for v in (fx0, fy0, cx0, cy0)],
               "intrinsics_after": [fx0 * sol.x[0] / scale, fy0 * sol.x[1] / scale, cx0 / scale, cy0 / scale],
               "distortion_after": [round(float(sol.x[2]), 5), 0.0, 0.0, 0.0, 0.0],
               "image_scale": scale, "pairs": pair_meta}
    return sol.x, metrics


def cmd_camera_focus(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, camera=True, what="camera-focus")
    if not (pathlib.Path(a.run) / "frames.csv").exists():
        sys.exit(f"camera-focus: {a.run} is not a fusion run directory (frames.csv missing); run a LIVO round first")
    out_dir = next_version_dir(mine, truck, "camfocus")
    out_dir.mkdir(parents=True, exist_ok=True)
    x, metrics = camera_focus_refine(parent_calib, pathlib.Path(a.bag), a.run, out_dir,
                                     n_pairs=a.pairs, window=tuple(a.window), scale=a.scale)
    calib = copy.deepcopy(parent_calib)
    applied = []
    if x is not None and a.apply:
        # ADVISORY by default: the pair bootstrap underestimates systematic depth-association bias.
        # Before --apply, cross-check fx against the static pole-azimuth test (camera-lidar --method
        # poles reports u_px per pole: an fx scale error s shows as du = (1/s-1)*(u-cx), i.e. tens of
        # px at |u-cx| > 500 for a 5+ % error). On truck29 this test bounded fx to <=1 % while the
        # flow solve reported -8 %: the flow numbers alone must not rewrite the intrinsics.
        ci = metrics["ci95"]
        def decisive(name, val, null, tol):
            l, h = ci[name]
            return abs(val - null) > tol and not (l <= null <= h)
        fx_ok = decisive("fx_scale", metrics["fx_scale"], 1.0, 0.008)
        fy_ok = decisive("fy_scale", metrics["fy_scale"], 1.0, 0.008)
        k1_ok = decisive("k1", metrics["k1"], 0.0, 0.006)
        if fx_ok or fy_ok or k1_ok:
            cam = calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))
            fx, fy, cx, cy = cam["intrinsics"]
            cam["intrinsics"] = [fx * (metrics["fx_scale"] if fx_ok else 1.0),
                                 fy * (metrics["fy_scale"] if fy_ok else 1.0), cx, cy]
            if k1_ok:
                cam["distortion"] = metrics["distortion_after"]
            cam["source"] = "livo_calib camera-focus (optical flow x LiDAR depth)"
            applied = [n for n, okf in (("fx", fx_ok), ("fy", fy_ok), ("k1", k1_ok)) if okf]
    metrics["applied"] = applied
    if x is None:
        note = f"camera-focus inconclusive: {metrics.get('error')}"
    elif applied:
        note = (f"camera intrinsics refined by flow x depth: " +
                ", ".join(applied) +
                f" (fx x{metrics['fx_scale']}, fy x{metrics['fy_scale']}, k1 {metrics['k1']:+.4f}; "
                f"residual {metrics['residual_px_median_before']} -> {metrics['residual_px_median_after']} px)")
    else:
        note = (f"camera intrinsics CONFIRMED by flow x depth (fx x{metrics['fx_scale']}, fy x{metrics['fy_scale']}, "
                f"k1 {metrics['k1']:+.4f} all within noise); nothing applied")
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="camfocus", parent=a.id, note=a.note or note,
                     inputs={"bag": a.bag, "dataset": ds_info, "run": a.run, "pairs": a.pairs,
                             "window": list(a.window)},
                     metrics=metrics, activate=a.activate and bool(applied))


def cmd_camera_lidar(a):
    pdir, parent_calib, mine, truck = parent_info(a.id)
    _, ds_info = require_dataset(a.bag, camera=True, what="camera-lidar")
    if not (pathlib.Path(a.run) / "frames.csv").exists():
        sys.exit(f"camera-lidar: {a.run} is not a fusion run directory (frames.csv missing); run a LIVO round first")
    out_dir = next_version_dir(mine, truck, "camlidar")
    out_dir.mkdir(parents=True, exist_ok=True)
    if a.method == "poles":
        T_new, metrics = pole_yaw_refine(parent_calib, pathlib.Path(a.bag), a.run, out_dir, n_frames=a.frames,
                                         window=tuple(a.window), accumulate=a.accumulate)
        calib = copy.deepcopy(parent_calib)
        metrics["max_delta_deg"] = a.max_delta
        ok = T_new is not None and metrics.get("used", 0) >= 3 and metrics.get("yaw_mad_deg", 9) <= 0.15
        too_big = T_new is not None and abs(metrics["yaw_delta_deg"]) > a.max_delta
        metrics["applied"] = bool(ok and not too_big)
        if metrics["applied"]:
            calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"] = T_new.reshape(-1).tolist()
            calib["transforms"]["midrange_camera_to_rear_axle"]["refined_by"] = "livo_calib camera-lidar pole yaw"
            calib["calibration_type"] = calib.get("calibration_type", "") + "+camera_pole_yaw"
            note = (f"camera yaw refined on {metrics['used']} pole observations: {metrics['yaw_delta_deg']:+.3f} deg "
                    f"(median shift {metrics['du_median_px']:+.1f} px, MAD {metrics['yaw_mad_deg']:.3f} deg)")
        elif too_big:
            note = f"pole yaw {metrics['yaw_delta_deg']:+.3f} deg exceeds ±{a.max_delta} deg: NOT applied, extrinsic kept"
        else:
            note = f"pole yaw not applied ({metrics.get('error') or 'inconsistent observations'}): extrinsic kept"
        finalize_version(out_dir, calib, mine=mine, truck=truck, stage="camlidar", parent=a.id, note=a.note or note,
                         inputs={"bag": a.bag, "dataset": ds_info, "run": a.run, "frames": a.frames, "window": list(a.window)},
                         metrics=metrics, activate=a.activate and metrics["applied"])
        return
    T_new, metrics = camera_lidar_refine(parent_calib, pathlib.Path(a.bag), a.run, out_dir, n_frames=a.frames,
                                         window=tuple(a.window), accumulate=a.accumulate,
                                         solve_intrinsics=a.solve_intrinsics, dof=a.dof)
    calib = copy.deepcopy(parent_calib)
    improved = metrics["score_after_px"] < metrics["score_before_px"]
    too_big = max(abs(v) for v in metrics["delta_rot_deg_cam_xyz"]) > a.max_delta
    metrics["applied"] = bool(improved and not too_big)
    metrics["max_delta_deg"] = a.max_delta
    if metrics["applied"]:
        calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"] = T_new.reshape(-1).tolist()
        calib["transforms"]["midrange_camera_to_rear_axle"]["refined_by"] = f"livo_calib camera-lidar edge alignment ({metrics['dof']})"
        calib["calibration_type"] = calib.get("calibration_type", "") + "+camera_lidar_edge_refined"
    if metrics.get("intrinsics_solved"):
        calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))["intrinsics"] = metrics["intrinsics_used"]
        calib["camera"]["source"] = "livo_calib camera-lidar edge alignment (focal scale solved)"
    if metrics["applied"]:
        note = (f"camera yaw/roll refined by vertical-edge alignment: delta cam xyz {metrics['delta_rot_deg_cam_xyz']} deg"
                if a.dof != "full" else "camera->rear extrinsic refined by depth-edge alignment (6-DoF)")
    elif too_big:
        note = f"camera-lidar solution {metrics['delta_rot_deg_cam_xyz']} deg exceeds ±{a.max_delta} deg: NOT applied, extrinsic kept"
    else:
        note = "camera-lidar stage ran but did NOT improve the edge score: extrinsic kept; inspect overlays"
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage="camlidar", parent=a.id,
                     note=a.note or note,
                     inputs={"bag": a.bag, "dataset": ds_info, "run": a.run, "frames": a.frames,
                             "window": list(a.window)},
                     metrics=metrics, activate=a.activate and metrics["applied"])


def render_overlays(calib, bag, run, out_dir, times, accumulate=1.0, scale=0.5, max_points=60000,
                    rot_delta=None, cam_offset=None, title=""):
    """Project accumulated scans of a LIVO run into the camera images at the requested run times."""
    import cv2
    from scipy.spatial.transform import Rotation as R
    fr = read_run_frames(run)
    stamps, rot = fr["stamp"], R.from_quat(fr["quat"])
    cam = calib.get("camera", DEFAULT_CAMERA)
    fx, fy, cx, cy = cam["intrinsics"]
    K = np.array([[fx * scale, 0, cx * scale], [0, fy * scale, cy * scale], [0, 0, 1]])
    dist = np.array(cam.get("distortion", [0] * 5), float)
    T_rear_cam = np.asarray(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    if rot_delta is not None and any(abs(v) > 0 for v in rot_delta):
        P = np.eye(4)
        P[:3, :3] = R.from_euler("xyz", rot_delta, degrees=True).as_matrix()
        T_rear_cam = T_rear_cam @ P
    if cam_offset is None:
        cam_offset = calib.get("time_offsets", {}).get("camera", 0.0)
    img_stamps, stream = dump_camera_stream(bag, out_dir)
    eff = img_stamps + cam_offset
    t0 = stamps[0]
    targets, index = [], []
    for t_req in times:
        tq = t0 + float(t_req)
        cand = np.flatnonzero(np.abs(stamps - tq) <= 1.0)
        if len(cand) == 0:
            index.append({"t": float(t_req), "error": "outside run"})
            continue
        best = None
        for j in cand[np.argsort(np.abs(stamps[cand] - tq))]:
            k = int(np.argmin(np.abs(eff - stamps[j])))
            if abs(eff[k] - stamps[j]) <= 0.02:
                best = (int(j), k)
                break
        if best is None:
            index.append({"t": float(t_req), "error": "no image within 20 ms of a frame"})
            continue
        targets.append((float(t_req), best[0], best[1]))
    if not targets:
        stream.unlink(missing_ok=True)
        return index
    images = {k: img for k, img in decode_hevc(stream, scale, keep={k for _, _, k in targets})}
    stream.unlink(missing_ok=True)
    T_imu_rear = np.eye(4)
    T_imu_rear[:3, 3] = fr["t_li"]
    T_cam_rear = np.linalg.inv(T_rear_cam)

    def T_world_imu(j):
        T = np.eye(4)
        T[:3, :3] = rot[j].as_matrix()
        T[:3, 3] = fr["pos"][j]
        return T

    speed = np.linalg.norm(np.gradient(fr["pos"], stamps, axis=0), axis=1)
    for t_req, j, k in targets:
        if k not in images:
            index.append({"t": t_req, "error": "frame not decoded"})
            continue
        sel = np.flatnonzero(np.abs(stamps - stamps[j]) <= accumulate / 2)
        pts_world = []
        for i in sel:
            o, c = (fr["offset"][i] - 8) // 20, fr["count"][i]
            p = np.asarray(fr["archive"][o:o + c, :3], np.float64)
            p = p[np.isfinite(p).all(1)]
            Tw = T_world_imu(i) @ T_imu_rear
            pts_world.append(p @ Tw[:3, :3].T + Tw[:3, 3])
        pw = np.vstack(pts_world)
        T_rear_world = np.linalg.inv(T_world_imu(j) @ T_imu_rear)
        p_rear = pw @ T_rear_world[:3, :3].T + T_rear_world[:3, 3]
        pc = p_rear @ T_cam_rear[:3, :3].T + T_cam_rear[:3, 3]
        pc = pc[pc[:, 2] > 1.0]
        if len(pc) > max_points:
            pc = pc[np.random.default_rng(0).choice(len(pc), max_points, replace=False)]
        img = images[k]
        h, w = img.shape
        canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        uv = cv2.projectPoints(pc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v, depth = u[inside], v[inside], pc[inside, 2]
        order = np.argsort(-depth)  # far first, near overwrites
        zbuf = np.full((h, w), np.inf)
        zbuf[v[order], u[order]] = depth[order]
        # colour by depth (near red -> far blue), only the nearest point per pixel is drawn
        keep = depth[order] <= zbuf[v[order], u[order]] + 1e-6
        uo, vo, do = u[order][keep], v[order][keep], depth[order][keep]
        norm = np.clip(1.0 - np.log1p(do) / np.log1p(60.0), 0, 1)
        colors = cv2.applyColorMap((norm * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
        canvas[vo, uo] = colors
        for dy, dx in ((0, 1), (1, 0)):  # 2x2 splat so points stay visible at half resolution
            vv, uu = np.clip(vo + dy, 0, h - 1), np.clip(uo + dx, 0, w - 1)
            canvas[vv, uu] = colors
        label = f"t=+{t_req:.1f}s  {speed[j]:.1f} m/s  {len(do)} pts  {title}"
        if rot_delta is not None and any(abs(v) > 0 for v in rot_delta):
            label += f"  delta cam xyz {tuple(round(float(v), 3) for v in rot_delta)} deg"
        cv2.putText(canvas, label, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        name = f"overlay_t{int(round(t_req)):03d}.jpg"
        cv2.imwrite(str(out_dir / name), canvas, [cv2.IMWRITE_JPEG_QUALITY, 88])
        index.append({"t": t_req, "file": name, "frame": j, "image": k, "points": int(len(do)),
                      "speed_mps": round(float(speed[j]), 2), "image_dt_ms": round(float((eff[k] - stamps[j]) * 1e3), 1)})
    return index


def cmd_overlay(a):
    pdir, calib, mine, truck = parent_info(a.id)
    require_dataset(a.bag, camera=True, what="overlay")
    run = pathlib.Path(a.run)
    if not (run / "frames.csv").exists() or not (run / "scans.bin").exists():
        sys.exit(f"overlay: {run} has no frames.csv/scans.bin (run a LIVO round with save_map)")
    out_dir = pathlib.Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    times = [float(v) for v in str(a.times).split(",") if v.strip()]
    index = render_overlays(calib, pathlib.Path(a.bag), run, out_dir, times, accumulate=a.accumulate, scale=a.scale,
                            max_points=a.max_points, rot_delta=a.camera_rot_delta, cam_offset=a.camera_offset,
                            title=a.id.split("/")[-1])
    doc = {"id": a.id, "run": str(run), "bag": str(a.bag), "camera_rot_delta_deg_cam_xyz": a.camera_rot_delta,
           "camera_offset": a.camera_offset, "frames": index, "created": now()}
    (out_dir / "index.json").write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(doc, ensure_ascii=False))


# ----------------------------------------------------------------------------- bookkeeping cmds
def cmd_list(a):
    reg = load_registry()
    rows = all_versions()
    ids = {r.get("id") for r in rows}
    for r in rows:
        r.setdefault("stage_label", STAGES.get(r.get("stage"), {}).get("label", r.get("stage")))
        r.setdefault("advisory", bool(STAGES.get(r.get("stage"), {}).get("advisory")))
        r["children"] = sorted(c.get("id") for c in rows if c.get("parent") == r.get("id"))
        r["active"] = reg.get("active", {}).get(f"{r.get('mine')}/{r.get('truck')}") == r.get("id")
        r["parent_missing"] = bool(r.get("parent")) and r.get("parent") not in ids
    if a.json:
        print(json.dumps({"active": reg.get("active", {}), "versions": rows, "stages": STAGES},
                         indent=2, ensure_ascii=False))
        return
    for r in rows:
        act = "*" if r["active"] else " "
        print(f"{act} {r.get('id'):40s} {r.get('stage', ''):9s} {r.get('created', '')[:19]}  {r.get('note', '')[:60]}")


def validate_calibration(calib, *, strict=False):
    """Structural + plausibility checks; returns {"ok", "errors", "warnings"}."""
    errors, warnings = [], []
    lidar = calib.get("lidar") or []
    if not lidar:
        errors.append("lidar[] missing")
    found = {}
    for e in lidar:
        for key, val in e.items():
            if key.startswith("transform_matrix_lidar_"):
                found[key] = val
    for name, key in LIDAR_KEYS.items():
        if key not in found:
            errors.append(f"{name} ({key}) missing")
            continue
        m = np.asarray(found[key], float)
        if m.size != 16:
            errors.append(f"{key}: {m.size} values, expected 16")
            continue
        m = m.reshape(4, 4)
        r = m[:3, :3]
        dev = float(np.abs(r.T @ r - np.eye(3)).max())
        if dev > 1e-5:
            errors.append(f"{key}: rotation not orthonormal (max deviation {dev:.1e})")
        if np.linalg.det(r) < 0:
            errors.append(f"{key}: rotation is a reflection (det < 0)")
        if float(np.linalg.norm(m[:3, 3])) > 20.0:
            errors.append(f"{key}: translation {m[:3, 3].round(3).tolist()} m implausible")
        rpy = rpy_deg(m)
        if abs(rpy[0]) > 15 or abs(rpy[1]) > 15:
            warnings.append(f"{name}: roll/pitch {rpy[:2]} deg exceed 15 deg")
        if not (-1.0 <= m[2, 3] <= 8.0):
            warnings.append(f"{name}: z {m[2, 3]:.3f} m outside [-1, 8]")
    cam_tf = (calib.get("transforms") or {}).get("midrange_camera_to_rear_axle", {}).get("transform_matrix")
    if cam_tf is None:
        errors.append("transforms.midrange_camera_to_rear_axle.transform_matrix missing")
    else:
        m = np.asarray(cam_tf, float)
        if m.size != 16:
            errors.append(f"camera transform has {m.size} values, expected 16")
        else:
            r = m.reshape(4, 4)[:3, :3]
            dev = float(np.abs(r.T @ r - np.eye(3)).max())
            if dev > 1e-5:
                errors.append(f"camera transform not orthonormal (max deviation {dev:.1e})")
    cam = calib.get("camera") or {}
    intr = cam.get("intrinsics")
    if not intr or len(intr) != 4:
        (errors if strict else warnings).append("camera.intrinsics missing (needed for LIVO)")
    else:
        fx, fy, cx, cy = [float(v) for v in intr]
        w, h = cam.get("width", 1920), cam.get("height", 1080)
        if not (200 <= fx <= 6000 and 200 <= fy <= 6000):
            errors.append(f"camera fx/fy {fx:.1f}/{fy:.1f} implausible")
        if not (0 < cx < w and 0 < cy < h):
            errors.append(f"camera principal point ({cx:.1f}, {cy:.1f}) outside {w}x{h}")
        truck = str((calib.get("identity") or {}).get("truck") or "")
        source = str(cam.get("source") or "")
        if source.startswith("defaults") and "confirmed" not in source and truck and not truck.startswith("truck29"):
            warnings.append("camera intrinsics are the truck29 defaults, unverified for this camera: derive them "
                            "(livo_calib.py set --intrinsics-from-supplier) or enter measured values")
    offs = calib.get("time_offsets") or {}
    for k in ("lidar", "camera"):
        v = offs.get(k)
        if not isinstance(v, (int, float)):
            (errors if strict else warnings).append(f"time_offsets.{k} missing")
        elif abs(v) > 1.0:
            errors.append(f"time_offsets.{k} = {v} s implausible")
    if offs.get("camera") == 0.0 and "camera_source" not in offs:
        warnings.append("time_offsets.camera is 0.0 and unverified (run the camera-time stage)")
    return {"ok": not errors, "errors": errors, "warnings": warnings}


def cmd_validate(a):
    if a.file:
        path = pathlib.Path(a.file)
        vid = str(path)
    else:
        path = version_dir(a.id) / "calibration.json"
        vid = a.id
    calib = json.loads(path.read_text())
    result = validate_calibration(calib, strict=a.strict)
    result["id"] = vid
    result["calibration_file"] = str(path)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not result["ok"]:
        sys.exit(1)


def cmd_delete(a):
    d = version_dir(a.id)
    reg = load_registry()
    mine, truck, _ = a.id.strip("/").split("/")
    if reg.get("active", {}).get(f"{mine}/{truck}") == a.id:
        sys.exit(f"{a.id} is the active version; activate another version first")
    children = [r.get("id") for r in all_versions() if r.get("parent") == a.id]
    if children:
        sys.exit(f"{a.id} has child versions {children}; delete them first")
    shutil.rmtree(d)
    print(json.dumps({"deleted": a.id, "dir": str(d)}))


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
        calib["time_offsets"]["camera_source"] = f"operator set (livo_calib.py set): {a.note}" if a.note else "operator set"
        changes["time_offsets.camera"] = a.camera_offset
    if a.lidar_offset is not None:
        calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))["lidar"] = a.lidar_offset
        changes["time_offsets.lidar"] = a.lidar_offset
    if getattr(a, "lidar_frontend_offset", None) is not None:
        calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))["lidar_frontend"] = a.lidar_frontend_offset
        changes["time_offsets.lidar_frontend"] = a.lidar_frontend_offset
    if getattr(a, "rtk_position_offset", None) is not None:
        calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))["rtk_position"] = a.rtk_position_offset
        changes["time_offsets.rtk_position"] = a.rtk_position_offset
    if getattr(a, "ins_attitude_offset", None) is not None:
        calib.setdefault("time_offsets", dict(DEFAULT_TIME_OFFSETS))["ins_attitude"] = a.ins_attitude_offset
        changes["time_offsets.ins_attitude"] = a.ins_attitude_offset
    if a.intrinsics:
        calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))["intrinsics"] = [float(v) for v in a.intrinsics]
        calib["camera"]["source"] = "operator entered (livo_calib.py set --intrinsics)"
        changes["camera.intrinsics"] = calib["camera"]["intrinsics"]
    elif a.intrinsics_from_supplier:
        root = lineage_of(a.id, json.loads((pdir / "meta.json").read_text()).get("parent"))[0]
        rdir = version_dir(root)
        if not (rdir / "source.json").exists():
            sys.exit(f"--intrinsics-from-supplier: lineage root {root} has no source.json (not an import)")
        supplier = json.loads((rdir / "source.json").read_text())
        mc = supplier.get("midrange_camera")
        mc0 = mc[0] if isinstance(mc, list) and mc and isinstance(mc[0], dict) else {}
        T_sup = json.loads((rdir / "calibration.json").read_text())["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"]
        cam = calib.setdefault("camera", copy.deepcopy(DEFAULT_CAMERA))
        fit, info = intrinsics_from_ground_equations(ground_equations(mc0), T_sup, cam.get("width", 1920), cam.get("height", 1080))
        if fit is None:
            sys.exit(f"--intrinsics-from-supplier: {info.get('reason')}")
        cam["intrinsics"] = fit
        cam["source"] = f"supplier ground-distance curves equation_*m of {root} ({info['model']}, rms {info['rms_px']} px)"
        cam["intrinsics_check"] = {"method": "supplier ground-distance curves (midrange_camera[0].equation_*m)", **info}
        changes["camera.intrinsics"] = fit
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
    for item in getattr(a, "lidar_translation_delta", None) or []:
        # operator correction of one LiDAR's lever arm, rear-axle frame (x right, y forward, z up), metres;
        # e.g. the back LiDAR's measured along-track map offset (guard metric "back LiDAR along-track offset")
        key, delta = item[0], item[1:]
        try:
            delta = [float(v) for v in delta]
        except ValueError:
            sys.exit(f"--lidar-translation-delta {' '.join(item)}: DX DY DZ must be numbers")
        entry = next((e for e in calib.get("lidar", []) if key in e), None)
        if entry is None:
            sys.exit(f"--lidar-translation-delta: {key} not in calibration lidar list "
                     f"({[k for e in calib.get('lidar', []) for k in e if k.startswith('transform_matrix')]})")
        T = np.asarray(entry[key], float).reshape(4, 4)
        T[:3, 3] += delta
        entry[key] = T.reshape(-1).tolist()
        changes[f"lidar_translation_delta.{key}"] = delta
    if not changes:
        sys.exit("nothing to set")
    stage = a.stage or "manual"
    if stage == "camloop":
        calib["transforms"]["midrange_camera_to_rear_axle"]["refined_by"] = "livo_calib closed-loop pitch scan (native odometry scale vs RTK)"
        calib["calibration_type"] = calib.get("calibration_type", "") + "+camera_closed_loop"
    metrics = {"changes": changes}
    if a.metrics_json:
        try:
            metrics.update(json.loads(a.metrics_json))
        except json.JSONDecodeError as exc:
            sys.exit(f"--metrics-json is not valid JSON: {exc}")
    out_dir = next_version_dir(mine, truck, stage)
    finalize_version(out_dir, calib, mine=mine, truck=truck, stage=stage, parent=a.id,
                     note=a.note or "operator-entered values", metrics=metrics, activate=a.activate)


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

    p = sub.add_parser("axle", help="reference-LiDAR -> rear-axle alignment from RTK/INS poses (no mapping run)")
    p.add_argument("--bag", required=True, help="recording: raw ROS 2 root (segment folders) or single bag folder")
    p.add_argument("--window", nargs=2, type=float, default=(0.0, 1e9),
                   help="seconds after the first reference scan (default: whole recording; turns improve x/y/roll)")
    p.add_argument("--dof", choices=("auto", "rpy_xy", "rpy", "xy"), default="auto",
                   help="auto (default): rpy_xy for a raw supplier import, xy (keep rotation, refine lever arm + time) "
                        "for an already aligned parent; rpy_xy / rpy / xy force the solved DoF")
    p.add_argument("--keep-time-offset", action="store_true", help="do not estimate/apply the LiDAR time shift")
    p.add_argument("--no-frontend-sync", action="store_true",
                   help="keep time_offsets.lidar_frontend untouched (skip the imu/ins clock-lag check)")
    p.add_argument("--no-lateral", action="store_true",
                   help="skip the whole-recording lateral lever-arm pass (default: run it when the window is shorter "
                        "than the recording)")
    common(p)
    p.set_defaults(func=cmd_axle)

    p = sub.add_parser("ground")
    p.add_argument("--bag", required=True, help="recording: raw ROS 2 root (segment folders) or single bag folder")
    p.add_argument("--stationary", nargs=2, type=int, default=(2, 20))
    p.add_argument("--moving", nargs=2, type=int, default=(300, 40))
    common(p)
    p.set_defaults(func=cmd_ground)

    p = sub.add_parser("gcons")
    p.add_argument("--bag", required=True, help="recording: raw ROS 2 root (segment folders) or single bag folder")
    p.add_argument("--run", required=True, help="fusion run directory (Log/fusion/<stamp>) of an RTK-fused LIO round "
                   "made with the parent's extrinsics")
    common(p)
    p.set_defaults(func=cmd_gcons)

    p = sub.add_parser("driven")
    p.add_argument("--bag", required=True, help="recording: raw ROS 2 root (segment folders) or single bag folder")
    p.add_argument("--run", required=True, help="fusion run directory (Log/fusion/<stamp>) of a LIO round")
    p.add_argument("--window", nargs=2, type=float, default=(8.0, 40.0))
    p.add_argument("--vertical-only", action="store_true")
    p.add_argument("--pass1-poses", choices=("native", "optimized"),
                   help="pass-1 pose source of the refinement (default native; optimized = RTK-optimized trajectory of the run)")
    common(p)
    p.set_defaults(func=cmd_driven)

    p = sub.add_parser("camera-time")
    p.add_argument("--bag", required=True, help="recording containing midrange_camera/ffmpeg and imu_data (raw root)")
    p.add_argument("--duration", type=float, default=None,
                   help="seconds of video used from the first image (default: all); ~60 s of driving is enough")
    p.add_argument("--start", type=float, default=None,
                   help="seconds skipped at the beginning (default: from the motion profile, i.e. the standstill "
                        "head is skipped; 0 = from the first image)")
    common(p)
    p.set_defaults(func=cmd_camera_time)

    p = sub.add_parser("rtk-time", help="RTK position time association from map self-consistency of LIO rounds")
    p.add_argument("--run", required=True, help="LIO round (or its fusion dir) mapped with the parent version")
    p.add_argument("--run2", help="probe round mapped with the probe rtk_position (final step: secant -> version)")
    p.add_argument("--dry-run", action="store_true", help="final step: print the result without creating a version")
    common(p)
    p.set_defaults(func=cmd_rtk_time)

    p = sub.add_parser("windows", help="motion profile -> stationary/driving windows for stages and rounds (JSON)")
    p.add_argument("--bag", required=True, help="recording: raw ROS 2 root (segment folders) or single bag folder")
    p.add_argument("--round-duration", type=float, default=60.0, help="length of a calibration round (s)")
    p.add_argument("--refresh", action="store_true", help="ignore the cached profile")
    p.add_argument("--start", type=float, help="operator-chosen segment start (s from the bag start)")
    p.add_argument("--duration", type=float, help="operator-chosen segment length (s); omitted = to the end")
    p.set_defaults(func=cmd_windows)

    p = sub.add_parser("camera-lidar")
    p.add_argument("--bag", required=True, help="recording containing midrange_camera/ffmpeg (raw root)")
    p.add_argument("--run", required=True, help="fusion run directory of a LIVO round (frames end at image times)")
    p.add_argument("--frames", type=int, default=30, help="image/scan pairs to inspect (poles are found in a subset)")
    p.add_argument("--window", nargs=2, type=float, default=(5.0, 115.0))
    p.add_argument("--accumulate", type=float, default=1.0, help="seconds of scans accumulated around each image")
    p.add_argument("--method", choices=("poles", "edges"), default="poles",
                   help="poles: camera yaw from lamp/sign posts (default); edges: generic depth-edge alignment (legacy)")
    p.add_argument("--solve-intrinsics", action="store_true",
                   help="also solve fx/fy scale factors and write them to camera.intrinsics when the score improves (dof=full only)")
    p.add_argument("--dof", choices=("yawroll", "full"), default="yawroll",
                   help="yawroll: vertical edges, solve camera yaw/roll only (pitch comes from the closed loop); full: legacy 6-DoF")
    p.add_argument("--max-delta", type=float, default=0.5, help="reject solutions whose rotation exceeds this (deg)")
    common(p)
    p.set_defaults(func=cmd_camera_lidar)

    p = sub.add_parser("from-precheck")
    p.add_argument("--run-dir", required=True, help="results/<round> containing Log/extrinsic_precheck.json")
    common(p)
    p.set_defaults(func=cmd_from_precheck)

    p = sub.add_parser("camera-focus", help="fx/fy/k1 from optical flow vs LiDAR depth + trajectory (no target)")
    p.add_argument("--bag", required=True, help="recording containing midrange_camera/ffmpeg (raw root)")
    p.add_argument("--run", required=True, help="fusion run directory of a LIVO round (frames end at image times)")
    p.add_argument("--pairs", type=int, default=36, help="consecutive image pairs to sample")
    p.add_argument("--window", nargs=2, type=float, default=(5.0, 115.0))
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--apply", action="store_true",
                   help="write decisive fx/fy/k1 into the new version (default: advisory report only; "
                        "cross-check fx with camera-lidar --method poles u_px first)")
    common(p)
    p.set_defaults(func=cmd_camera_focus)

    p = sub.add_parser("set")
    p.add_argument("--camera-rot-delta", nargs=3, type=float, metavar=("RX", "RY", "RZ"),
                   help="rotate the camera extrinsic by these degrees (camera-frame xyz Euler); closed-loop tests")
    p.add_argument("--camera-offset", type=float)
    p.add_argument("--lidar-translation-delta", nargs=4, action="append", metavar=("KEY", "DX", "DY", "DZ"),
                   help="shift one LiDAR extrinsic (e.g. transform_matrix_lidar_2) by DX DY DZ metres in the rear-axle "
                        "frame (x right, y forward, z up); repeatable")
    p.add_argument("--lidar-offset", type=float)
    p.add_argument("--lidar-frontend-offset", type=float,
                   help="scan<->IMU hardware offset consumed by the SLAM front-end (deskew/image pairing); "
                        "time_offsets.lidar keeps the INS alignment for the fusion")
    p.add_argument("--rtk-position-offset", type=float,
                   help="INS position association offset added to the (INS-aligned) scan time; the mapper reads it "
                        "from the calibration (time_offsets.rtk_position), yaml rtk.time_offset_sec is only the fallback")
    p.add_argument("--ins-attitude-offset", type=float, help="INS attitude association offset (time_offsets.ins_attitude)")
    p.add_argument("--intrinsics", nargs=4, type=float)
    p.add_argument("--intrinsics-from-supplier", action="store_true",
                   help="recover fx fy cx cy from the supplier ground-distance curves of the lineage's import")
    p.add_argument("--distortion", nargs=5, type=float)
    p.add_argument("--stage", choices=("manual", "camloop", "scan"), default="manual",
                   help="version suffix: manual (default), camloop (closed-loop pitch result), scan (temporary scan point)")
    p.add_argument("--metrics-json", help="extra metrics recorded in the version (JSON object)")
    common(p)
    p.set_defaults(func=cmd_set)

    p = sub.add_parser("overlay", help="project the LiDAR scans of a LIVO round into camera frames (visual check)")
    p.add_argument("--bag", required=True, help="recording containing midrange_camera/ffmpeg (raw root)")
    p.add_argument("--run", required=True, help="fusion run directory of a LIVO round (frames.csv + scans.bin)")
    p.add_argument("--out", required=True, help="output directory for overlay_*.jpg + index.json")
    p.add_argument("--times", default="20,60,100", help="seconds after run start, comma separated")
    p.add_argument("--camera-rot-delta", nargs=3, type=float, metavar=("RX", "RY", "RZ"),
                   help="preview an additional camera-frame rotation (deg) on top of the version's extrinsic")
    p.add_argument("--camera-offset", type=float, help="override time_offsets.camera (s) for the preview")
    p.add_argument("--accumulate", type=float, default=1.0)
    p.add_argument("--scale", type=float, default=0.5)
    p.add_argument("--max-points", type=int, default=60000)
    common(p)
    p.set_defaults(func=cmd_overlay)

    p = sub.add_parser("list")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_list)
    p = sub.add_parser("show")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_show)
    p = sub.add_parser("validate", help="structural/plausibility check of a version or a calibration file")
    p.add_argument("--id")
    p.add_argument("--file")
    p.add_argument("--strict", action="store_true", help="camera model / time offsets are required (LIVO)")
    p.set_defaults(func=cmd_validate)
    p = sub.add_parser("delete", help="remove a leaf version that is not active")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_delete)
    p = sub.add_parser("activate")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_activate)
    p = sub.add_parser("export")
    p.add_argument("--id", required=True)
    p.set_defaults(func=cmd_export)
    a = ap.parse_args()
    if a.cmd == "validate" and not (a.id or a.file):
        ap.error("validate needs --id or --file")
    try:
        a.func(a)
    finally:
        for d in CREATED_DIRS:  # a stage that failed must not leave a half-written version behind
            if d.exists() and not (d / "calibration.json").exists():
                shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    main()
