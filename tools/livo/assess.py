#!/usr/bin/env python3
"""Dataset assessment (数据评估) for MY-LIVO recordings.

One streaming pass over a recording (raw split root or single bag) produces a scored report of
every data property that limited calibration or mapping precision during the Wuhu truck29 /
truck05 work. Each metric carries a threshold (正常 / 关注 / 异常), the *remedy class* that says
whether the calibration pipeline can absorb the defect, and the project finding the threshold is
based on:

  fix = auto       the mapping self-check / merger / time-offset stages compensate it automatically
  fix = stage      an offline calibration stage (ground / driven / camtime / axle) solves it
  fix = procedure  needs a change of the recording or playback procedure (re-record, split, rate)
  fix = playback   only the playback settings need adjusting (split into segments, lower rate); not an alert
  fix = none       neither calibration nor procedure can recover the precision (hardware / source)

A metric that is not 正常 and whose remedy is `procedure` or `none` becomes an *alert*: the
precision loss cannot be rescued by calibration fine-tuning. The report is JSON (Studio) plus a
console table.

Message headers are decoded straight from the CDR bytes (no per-message deserialisation), so the
pass is I/O bound: truck29 (120 s, 3.5 GB) takes a few seconds, truck05 (720 s, 23 GB) about a
minute on a cold cache. Full point clouds are deserialised only for the static-start ground check
and one sampled slot every --sample-every seconds; camera frames are decoded for a handful of
sample windows.

CLI::

    /usr/bin/python3 tools/livo/assess.py <dataset> [--calibration calibration.json] [--json out.json]
        [--window START DURATION] [--sample-every 6] [--camera-samples 8] [--no-camera]
"""
import argparse
import re
import base64
import io
import json
import math
import os
import pathlib
import shutil
import struct
import subprocess
import sys
import tempfile
import time

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import numpy as np

if __package__ in (None, ""):  # executed as a file: make `livo` importable
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from livo import rosenv  # noqa: E402

rosenv.ensure()

from livo.dataset import Dataset, norm  # noqa: E402

SCHEMA = 2
LIDAR_KEYS = {"front_right_lidar": "transform_matrix_lidar_1", "back_lidar": "transform_matrix_lidar_2",
              "front_left_lidar": "transform_matrix_lidar_3", "front_lidar": "transform_matrix_lidar_6"}
REFERENCE_LIDAR = "front_lidar"
NS = 1_000_000_000

# ----------------------------------------------------------------------------- CDR field access
_HDR = struct.Struct("<iI")
_U32 = struct.Struct("<I")


def _align(off, n):
    """CDR alignment is relative to the start of the payload (after the 4-byte encapsulation)."""
    r = (off - 4) % n
    return off + ((n - r) % n)


def cdr_header(data):
    """(stamp_ns, offset after frame_id) for messages whose first field is std_msgs/Header."""
    sec, nsec = _HDR.unpack_from(data, 4)
    n = _U32.unpack_from(data, 12)[0]
    return sec * NS + nsec, 16 + n


def parse_pointcloud(data):
    ts, off = cdr_header(data)
    h, w = struct.unpack_from("<II", data, _align(off, 4))
    return ts, h * w


def parse_imu(data):
    ts, off = cdr_header(data)
    v = struct.unpack_from("<37d", data, _align(off, 8))
    return ts, v[13:16], v[25:28]


def parse_odometry(data):
    ts, off = cdr_header(data)
    off = _align(off, 4)
    n = _U32.unpack_from(data, off)[0]
    off = _align(off + 4 + n, 8)
    v = struct.unpack_from("<85d", data, off)
    # position 3, orientation 4, covariance 36, twist linear 3, angular 3, covariance 36
    return ts, v[0:3], v[3:7], v[43:46], (v[7], v[14], v[21])


def parse_ffmpeg(data):
    ts, off = cdr_header(data)
    off = _align(off, 4)
    w, h = struct.unpack_from("<II", data, off)
    off += 8
    n = _U32.unpack_from(data, off)[0]
    off = _align(off + 4 + n, 8)
    pts = struct.unpack_from("<Q", data, off)[0]
    flags, big = struct.unpack_from("<BB", data, off + 8)
    off = _align(off + 10, 4)
    n = _U32.unpack_from(data, off)[0]
    return ts, w, h, pts, flags, data[off + 4:off + 4 + n]


def _cdr_string(data, off):
    off = _align(off, 4)
    n = _U32.unpack_from(data, off)[0]
    return bytes(data[off + 4:off + 3 + n]) if n else b"", off + 4 + n


def parse_diag(data, keys=(b"ins_pos_mode", b"gps_num_sats")):
    """(stamp_ns, {key: value}) for diagnostic_msgs/DiagnosticArray, reading only the wanted keys."""
    ts, off = cdr_header(data)
    off = _align(off, 4)
    n_status = _U32.unpack_from(data, off)[0]
    off += 4
    found = {}
    for _ in range(n_status):
        off += 1  # level (byte)
        for _ in range(3):  # name, message, hardware_id
            _, off = _cdr_string(data, off)
        off = _align(off, 4)
        n_val = _U32.unpack_from(data, off)[0]
        off += 4
        for _ in range(n_val):
            k, off = _cdr_string(data, off)
            v, off = _cdr_string(data, off)
            if k in keys:
                found[k] = v
    return ts, found


def _diag_int(v):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return -1


def verify_parsers(samples):
    """Compare the byte-level parsers with rclpy on the first message of each type; return the
    set of parsers that agree (the others fall back to deserialize_message)."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    ok = set()
    for kind, (typ, data) in samples.items():
        try:
            m = deserialize_message(data, get_message(typ))
            st = m.header.stamp.sec * NS + m.header.stamp.nanosec
            if kind == "lidar":
                ts, n = parse_pointcloud(data)
                good = ts == st and n == m.width * m.height
            elif kind == "imu":
                ts, g, a = parse_imu(data)
                good = ts == st and abs(g[2] - m.angular_velocity.z) < 1e-12 and abs(a[2] - m.linear_acceleration.z) < 1e-12
            elif kind == "odom":
                ts, p, q, vb, c = parse_odometry(data)
                good = (ts == st and abs(p[0] - m.pose.pose.position.x) < 1e-9 and abs(q[3] - m.pose.pose.orientation.w) < 1e-12
                        and abs(vb[1] - m.twist.twist.linear.y) < 1e-12 and abs(c[0] - m.pose.covariance[0]) < 1e-12)
            elif kind == "camera":
                ts, w, h, pts, flags, payload = parse_ffmpeg(data)
                good = ts == st and w == m.width and flags == int(m.flags) and len(payload) == len(m.data)
            elif kind == "status":
                ts, found = parse_diag(data)
                ref = {item.key: item.value for st_ in m.status for item in st_.values}
                good = ts == st and all(found.get(k.encode(), b"").decode() == ref.get(k, "") for k in ("ins_pos_mode", "gps_num_sats"))
            else:
                good = False
            if good:
                ok.add(kind)
        except Exception:  # noqa: BLE001
            continue
    return ok


# ----------------------------------------------------------------------------- metric records
FIX_LABEL = {"auto": "标定/建图自动补偿", "stage": "需运行离线标定阶段", "procedure": "需调整采集或回放流程",
             "playback": "调整回放设置（分段 / 倍速）", "none": "标定无法补救"}
STATUS_ORDER = {"ok": 0, "info": 0, "na": 0, "warn": 1, "bad": 2}


def grade(value, ok, warn, higher=False):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "na"
    if higher:
        return "ok" if value >= ok else "warn" if value >= warn else "bad"
    return "ok" if value <= ok else "warn" if value <= warn else "bad"


def fmt_num(v, unit=""):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "—"
    if isinstance(v, (int, np.integer)):
        s = f"{int(v)}"
    else:
        a = abs(v)
        s = f"{v:.0f}" if a >= 100 else f"{v:.1f}" if a >= 10 else f"{v:.2f}"
    return f"{s} {unit}".strip()


class Report:
    def __init__(self):
        self.groups = []
        self._by_id = {}

    def group(self, gid, label, blurb=""):
        g = {"id": gid, "label": label, "blurb": blurb, "metrics": []}
        self.groups.append(g)
        self._by_id[gid] = g
        return gid

    def add(self, gid, mid, label, value, unit="", *, ok=None, warn=None, higher=False, status=None,
            fix="none", fix_bad=None, note="", basis="", plain="", display=None, limits=None, extra=None):
        """Register one metric. `fix` applies to 关注, `fix_bad` (default = fix) to 异常.

        `plain` is a short Chinese sentence (one or two lines) shown directly below the label
        in the data-source card. It must answer two questions in plain language:
            • 它在测什么？(what is being measured)
            • 多少算“好”，为什么重要？(what good looks like and why it matters)
        Keep it <50 characters per line. Engineers should not have to open the tooltip to
        decide whether this metric is acceptable. `note`/`basis` remain the technical detail.
        """
        if status is None:
            status = grade(value, ok, warn, higher) if ok is not None else "info"
        fixc = fix if status != "bad" else (fix_bad or fix)
        if limits is None and ok is not None:
            cmp_ok, cmp_w = ("≥", "≥") if higher else ("≤", "≤")
            if warn is None or warn == ok:
                limits = f"{cmp_ok} {fmt_num(ok, unit)} 正常 · 其余 异常"
            else:
                limits = f"{cmp_ok} {fmt_num(ok, unit)} 正常 · {cmp_w} {fmt_num(warn, unit)} 关注 · 其余 异常"
        if isinstance(value, (np.integer,)):
            value = int(value)
        elif isinstance(value, (np.floating,)):
            value = float(value)
        rec = {"id": mid, "label": label, "value": value,
               "unit": unit, "display": display if display is not None else fmt_num(value, unit),
               "status": status, "limits": limits or "", "fix": fixc if status in ("warn", "bad") else None,
               "fix_label": FIX_LABEL.get(fixc, "") if status in ("warn", "bad") else "",
               "alert": status in ("warn", "bad") and fixc in ("procedure", "none"),
               "plain": plain or "", "note": note, "basis": basis,
               "thresholds": ({"ok": float(ok), "warn": float(warn if warn is not None else ok), "higher": bool(higher)}
                              if ok is not None and isinstance(value, (int, float)) and not isinstance(value, bool) else None)}
        if extra:
            rec["extra"] = extra
        self._by_id[gid]["metrics"].append(rec)
        return rec

    def finish(self):
        counts = {"ok": 0, "warn": 0, "bad": 0, "info": 0, "na": 0}
        alerts, fixable = [], []
        for g in self.groups:
            worst = "ok"
            for m in g["metrics"]:
                counts[m["status"]] = counts.get(m["status"], 0) + 1
                if STATUS_ORDER.get(m["status"], 0) > STATUS_ORDER[worst]:
                    worst = m["status"]
                if m["alert"]:
                    alerts.append({"id": m["id"], "group": g["label"], "label": m["label"], "display": m["display"],
                                   "status": m["status"], "fix": m["fix"], "fix_label": m["fix_label"], "note": m["note"]})
                elif m["status"] in ("warn", "bad"):
                    fixable.append({"id": m["id"], "group": g["label"], "label": m["label"], "display": m["display"],
                                    "status": m["status"], "fix": m["fix"], "fix_label": m["fix_label"], "note": m["note"]})
            g["status"] = worst
        verdict = "bad" if counts["bad"] else "warn" if counts["warn"] else "ok"
        return verdict, counts, alerts, fixable


# ----------------------------------------------------------------------------- geometry helpers
def load_calibration(path):
    if not path:
        return None
    calib = json.loads(pathlib.Path(path).read_text())
    mats = {}
    for entry in calib.get("lidar", []):
        for topic, key in LIDAR_KEYS.items():
            if key in entry:
                mats[topic] = np.asarray(entry[key], float).reshape(4, 4)
    return {"path": str(path), "matrices": mats, "time_offsets": calib.get("time_offsets", {}),
            "type": calib.get("calibration_type", "")}


def vehicle_ids(ds, calib_path):
    """(mine, truck number) of the recording (segment names `<Kind>_<mine>_<NN>_...`) and of the calibration path."""
    data = set()
    for seg in ds.segments:
        m = re.match(r"^[A-Za-z]+_([a-z]+)_(\d+)_", getattr(seg, "name", ""))
        if m:
            data.add((m.group(1), int(m.group(2))))
    cal = None
    if calib_path:
        m = re.search(r"/([a-z]+)/truck(\d+)[a-z]*/", str(calib_path)) or re.search(r"truck(\d+)", str(calib_path))
        if m:
            cal = (m.group(1), int(m.group(2))) if m.re.groups == 2 else (None, int(m.group(1)))
    return data, cal


def cloud_xyz(data, max_range=32.0):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    msg = deserialize_message(data, PointCloud2)
    arr = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
    P = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)
    if max_range:
        P = P[(P * P).sum(1) < max_range * max_range]
    return P


def is_keyframe(payload):
    """HEVC Annex-B packet starts with VPS/SPS/PPS or an IRAP slice (NAL types 32-34 / 16-21)."""
    n = len(payload)
    i = 0
    for _ in range(6):
        i = payload.find(b"\x00\x00\x01", i)
        if i < 0 or i + 3 >= n:
            return False
        typ = (payload[i + 3] >> 1) & 0x3F
        if 16 <= typ <= 21 or 32 <= typ <= 34:
            return True
        i += 3
    return False


def slot_structure(d, mats):
    """Above-ground structure count of one sampled slot (all LiDARs, rear-axle frame, 4-30 m, body box removed)."""
    above = total = 0
    for s, P in d.items():
        if s not in mats:
            continue
        Qp = transform(P, mats[s])
        r = np.hypot(Qp[:, 0], Qp[:, 1])
        body = (np.abs(Qp[:, 0]) < 3.0) & (Qp[:, 1] > -9.0) & (Qp[:, 1] < 5.0)
        sel = (r > 4.0) & (r < 30.0) & ~body
        above += int(((Qp[:, 2] > 0.5) & sel).sum())
        total += int(sel.sum())
    return above, total


def transform(P, M):
    return P @ M[:3, :3].T + M[:3, 3]


def fit_ground(P, iters=(0.25, 0.12, 0.07, 0.05)):  # kept for standalone diagnostics
    """Robust near-horizontal plane z = a x + b y + c through the lowest band of points."""
    if len(P) < 200:
        return None
    z0 = np.percentile(P[:, 2], 30)
    sel = np.abs(P[:, 2] - z0) < 0.6
    a = b = 0.0
    c = float(np.median(P[sel, 2])) if sel.sum() > 50 else z0
    for thr in iters:
        r = P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)
        sel = np.abs(r) < thr
        if sel.sum() < 100:
            return None
        A = np.c_[P[sel, 0], P[sel, 1], np.ones(sel.sum())]
        sol, *_ = np.linalg.lstsq(A, P[sel, 2], rcond=None)
        a, b, c = (float(x) for x in sol)
    return {"a": a, "b": b, "c": c, "inliers": int(sel.sum()), "points": P[sel]}


def cell_ground(Q, cell=1.5, min_points=8):
    """Per 1.5 m cell: 20th-percentile height of the points in the ground band (rear-axle frame)."""
    Q = Q[(Q[:, 2] > -3.0) & (Q[:, 2] < -0.35)]
    if len(Q) < min_points:
        return {}
    keys = np.floor(Q[:, :2] / cell).astype(np.int64)
    k = keys[:, 0] * 1_000_003 + keys[:, 1]
    order = np.argsort(k, kind="stable")
    ks, zs, kk = k[order], Q[order, 2], keys[order]
    bounds = np.flatnonzero(np.diff(ks)) + 1
    starts, ends = np.r_[0, bounds], np.r_[bounds, len(ks)]
    out = {}
    for s0, e0 in zip(starts, ends):
        if e0 - s0 >= min_points:
            out[int(ks[s0])] = (float(np.percentile(zs[s0:e0], 20)), (kk[s0, 0] + 0.5) * cell, (kk[s0, 1] + 0.5) * cell, int(e0 - s0))
    return out


def ground_consistency(start_frames, mats, reference=REFERENCE_LIDAR):
    """Static-start ground of every LiDAR vs the reference LiDAR: per-cell ground heights are
    compared in the cells both LiDARs observe (like the mapper's extrinsic self-check), giving the
    relative tilt (slopes of the height difference) and height offset (median difference)."""
    out = {}
    if reference not in start_frames or reference not in mats:
        return out
    cells = {}
    for topic, frames in start_frames.items():
        if topic not in mats or not frames:
            continue
        P = np.concatenate(frames)
        rng = np.linalg.norm(P, axis=1)
        P = P[(rng > 3.0) & (rng < 30.0)]
        cells[topic] = cell_ground(transform(P, mats[topic]))
    ref = cells.get(reference, {})
    if len(ref) < 12:
        return out
    for topic, cg in cells.items():
        if topic == reference:
            continue
        common = [k for k in cg if k in ref]
        n_pts = sum(cg[k][3] for k in common)
        if len(common) < 12 or n_pts < 300:
            out[topic] = {"solved": False, "reason": f"与 {reference} 的静止地面重叠不足（{len(common)} 格）"}
            continue
        X = np.array([[cg[k][1], cg[k][2]] for k in common])
        dz = np.array([cg[k][0] - ref[k][0] for k in common])
        keep = np.ones(len(dz), bool)
        sol = np.zeros(3)
        for thr in (0.25, 0.10):
            A = np.c_[X[keep], np.ones(keep.sum())]
            sol, *_ = np.linalg.lstsq(A, dz[keep], rcond=None)
            res = dz - (X @ sol[:2] + sol[2])
            keep = np.abs(res) < thr
            if keep.sum() < 8:
                break
        rot_x = math.degrees(math.atan(sol[1]))   # height-difference slope along y (forward)
        rot_y = math.degrees(math.atan(sol[0]))   # slope along x (right)
        out[topic] = {"solved": True, "dz_cm": 100 * float(np.median(dz)), "rot_x_deg": rot_x, "rot_y_deg": rot_y,
                      "tilt_deg": math.hypot(rot_x, rot_y), "overlap_cells": int(len(common)), "overlap_points": int(n_pts),
                      "residual_cm": 100 * float(np.median(np.abs(dz - np.median(dz))))}
    return out


# ----------------------------------------------------------------------------- camera sampling
def decode_gray(packets, scale=0.25):
    """Decode Annex-B HEVC packets (starting at a key frame) with ffmpeg; returns the last gray frame."""
    w, h = int(1920 * scale), int(1080 * scale)
    with tempfile.NamedTemporaryFile(suffix=".h265", delete=False) as f:
        for p in packets:
            f.write(p)
        path = f.name
    try:
        cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", "-f", "hevc", "-i", path, "-vf", f"scale={w}:{h}",
               "-pix_fmt", "gray", "-f", "rawvideo", "-"]
        proc = subprocess.run(cmd, stdin=subprocess.DEVNULL, capture_output=True, timeout=60)
        buf = proc.stdout
        n = w * h
        frames = len(buf) // n
        if frames == 0:
            return None
        return np.frombuffer(buf[(frames - 1) * n:frames * n], np.uint8).reshape(h, w)
    except Exception:  # noqa: BLE001
        return None
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def image_metrics(img):
    import cv2
    lap = cv2.Laplacian(img, cv2.CV_64F)
    ok, jpg = cv2.imencode(".jpg", cv2.resize(img, (240, 135), interpolation=cv2.INTER_AREA), [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    return {"mean": float(img.mean()), "clipped_pct": 100 * float((img >= 250).mean()),
            "dark_pct": 100 * float((img <= 8).mean()), "texture": float(lap.var()),
            "thumb": "data:image/jpeg;base64," + base64.b64encode(jpg.tobytes()).decode() if ok else None}


# ----------------------------------------------------------------------------- main pass
def percentile(a, p):
    return float(np.percentile(a, p)) if len(a) else None


def plan_chunks(ds, jobs, start_ns=None, end_ns=None):
    """Split the recording into time-contiguous groups of whole segments (one streaming pass each).

    Raw split recordings interleave Perception_* and Camera_* segments of the same 30 s; a chunk holds all
    segments whose start falls into its time bin, so every topic stays time-ordered across chunks and the
    parent only has to de-duplicate the segment borders (same rule as Dataset.iter_messages)."""
    segs = [x for x in ds.segments if (end_ns is None or x.start_ns <= end_ns) and (start_ns is None or x.end_ns >= start_ns)]
    segs.sort(key=lambda x: (x.start_ns, x.name))
    if jobs <= 1 or len(segs) < 2:
        return [segs]
    t_lo, t_hi = min(x.start_ns for x in segs), max(x.end_ns for x in segs)
    n = max(1, min(jobs, len(segs)))
    edges = [t_lo + (t_hi - t_lo) * k // n for k in range(1, n)]
    chunks = [[] for _ in range(n)]
    for x in segs:
        k = sum(1 for e in edges if x.start_ns >= e)
        chunks[k].append(x)
    return [c for c in chunks if c]


def scan_chunk(job):
    """One streaming pass over a chunk of segments; returns raw collectors (picklable)."""
    t_start = time.time()
    a = job["args"]
    ds = Dataset.open(job["dataset"])
    if job["segments"] is not None:
        names = set(job["segments"])
        ds = Dataset(ds.root, [x for x in ds.segments if x.name in names])
    calib = load_calibration(job["calibration"]) if job["calibration"] else None
    lidar_topics = job["lidar_topics"]
    short = {t: t.split("/")[-1] for t in lidar_topics}
    imu_topic, odom_topic, status_topic, cam_topic = job["imu_topic"], job["odom_topic"], job["status_topic"], job["cam_topic"]
    wanted = [t for t in lidar_topics + [imu_topic, odom_topic, status_topic, cam_topic] if t and t in ds.topics]
    start_ns, end_ns, t0, span_s = job["start_ns"], job["end_ns"], job["t0"], job["span_s"]
    first = job["first"]

    lid_ts = {t: [] for t in lidar_topics}
    lid_rec = {t: [] for t in lidar_topics}
    lid_n = {t: [] for t in lidar_topics}
    lid_start = {short[t]: [] for t in lidar_topics}
    sample_slots = {}
    slots_eval = []
    imu_ts, imu_gyro, imu_rec = [], [], []
    odo = {"ts": [], "p": [], "q": [], "vb": [], "cov": []}
    st_mode, st_sats, st_ts = [], [], []
    cam_ts, cam_rec, cam_flags, cam_size = [], [], [], []
    cam_samples = []
    cam_windows = list(job["cam_windows"])
    cam_pending = None
    sample_every_ns = int(a["sample_every"] * NS)
    next_sample_ns = None
    verified = None
    samples_for_verify = {}
    counters = {"messages": 0, "bytes": 0}
    dropped = {}
    n_start_frames = 22
    if not wanted:
        return {"empty": True, "counters": counters, "dropped": dropped, "elapsed": 0.0}

    for topic, data, rec_ns in ds.iter_messages(wanted, start_ns=start_ns, end_ns=end_ns, stats=dropped):
        counters["messages"] += 1
        counters["bytes"] += len(data)
        if first and verified is None:
            kind = ("lidar" if topic in lid_ts else "imu" if topic == imu_topic else "odom" if topic == odom_topic
                    else "status" if topic == status_topic else "camera" if topic == cam_topic else None)
            if kind and kind not in samples_for_verify:
                samples_for_verify[kind] = (ds.topics[topic], bytes(data))
            need = {k for k, t in (("lidar", lidar_topics[0] if lidar_topics else None), ("imu", imu_topic), ("odom", odom_topic),
                                   ("status", status_topic), ("camera", cam_topic)) if t and t in ds.topics}
            if need <= set(samples_for_verify):
                verified = verify_parsers(samples_for_verify)
                missing = need - verified
                if missing:
                    print(f"[assess] byte-level parser mismatch for {sorted(missing)} — check message definitions", flush=True)
        if topic in lid_ts:
            try:
                ts, n = parse_pointcloud(data)
            except struct.error:
                continue
            lid_ts[topic].append(ts)
            lid_rec[topic].append(rec_ns - ts)
            lid_n[topic].append(n)
            sname = short[topic]
            if first and len(lid_start[sname]) < n_start_frames and len(lid_ts[topic]) > 2:
                lid_start[sname].append(cloud_xyz(data))
            if next_sample_ns is None:
                next_sample_ns = ts + 3 * NS
            if calib and ts >= next_sample_ns - 60_000_000:
                d = sample_slots.setdefault(next_sample_ns, {})
                if sname not in d:
                    d[sname] = cloud_xyz(data)
                if len(d) == len(lidar_topics) or ts >= next_sample_ns + NS:
                    above, total = slot_structure(d, calib["matrices"])
                    slots_eval.append({"t_s": round(next_sample_ns / NS - t0, 1), "above_ground": above, "points": total})
                    del sample_slots[next_sample_ns]
                    next_sample_ns += sample_every_ns
        elif topic == imu_topic:
            ts, g, _acc = parse_imu(data)
            imu_ts.append(ts)
            imu_gyro.append(g)
            imu_rec.append(rec_ns - ts)
        elif topic == odom_topic:
            ts, p, q, vb, cov = parse_odometry(data)
            odo["ts"].append(ts)
            odo["p"].append(p)
            odo["q"].append(q)
            odo["vb"].append(vb)
            odo["cov"].append(cov)
        elif topic == status_topic:
            try:
                ts, found = parse_diag(data)
            except (struct.error, IndexError):
                continue
            st_ts.append(ts)
            st_mode.append(_diag_int(found.get(b"ins_pos_mode")))
            st_sats.append(_diag_int(found.get(b"gps_num_sats")))
        elif topic == cam_topic:
            ts, w, h, pts, flags, payload = parse_ffmpeg(data)
            cam_ts.append(ts)
            cam_rec.append(rec_ns - ts)
            cam_flags.append(flags)
            cam_size.append(len(payload))
            if cam_pending is not None:
                cam_pending["packets"].append(bytes(payload))
                if len(cam_pending["packets"]) >= 4:
                    cam_samples.append(cam_pending)
                    cam_pending = None
            elif cam_windows and ts / NS >= cam_windows[0] and ((flags & 1) or is_keyframe(payload)):
                cam_pending = {"t": ts / NS - t0, "packets": [bytes(payload)], "w": w, "h": h}
                while cam_windows and cam_windows[0] <= ts / NS:
                    cam_windows.pop(0)
    if cam_pending and len(cam_pending["packets"]) >= 2:
        cam_samples.append(cam_pending)

    # camera samples are decoded here so the ffmpeg work runs inside the parallel workers
    cam_eval = []
    for smp in cam_samples:
        img = decode_gray(smp["packets"])
        if img is None:
            cam_eval.append({"t_s": round(smp["t"], 1), "decoded": False})
            continue
        m = image_metrics(img)
        m.update({"t_s": round(smp["t"], 1), "decoded": True})
        cam_eval.append(m)

    return {"empty": False, "lid_ts": lid_ts, "lid_rec": lid_rec, "lid_n": lid_n, "lid_start": lid_start if first else None,
            "slots_eval": slots_eval, "imu_ts": imu_ts, "imu_gyro": imu_gyro, "imu_rec": imu_rec, "odo": odo,
            "st_mode": st_mode, "st_sats": st_sats, "st_ts": st_ts, "cam_ts": cam_ts, "cam_rec": cam_rec,
            "cam_flags": cam_flags, "cam_size": cam_size, "cam_eval": cam_eval, "counters": counters, "dropped": dropped,
            "elapsed": time.time() - t_start}


def _dedup(chunks, key, *companions):
    """Concatenate per-chunk time-ordered lists; drop border repeats / backwards samples like iter_messages does.

    chunks: list of dicts; key: name of the timestamp list; companions: names of parallel lists.
    Returns (merged dict of lists, n_duplicate, n_backwards)."""
    out = {k: [] for k in (key,) + companions}
    dup = back = 0
    last = None
    for c in chunks:
        ts = c[key]
        if not ts:
            continue
        keep_from = 0
        if last is not None:
            while keep_from < len(ts) and ts[keep_from] <= last:
                if ts[keep_from] == last:
                    dup += 1
                else:
                    back += 1
                keep_from += 1
        if keep_from:
            out[key].extend(ts[keep_from:])
            for k in companions:
                out[k].extend(c[k][keep_from:])
        else:
            out[key].extend(ts)
            for k in companions:
                out[k].extend(c[k])
        if out[key]:
            last = out[key][-1]
    return out, dup, back


def run(a):
    t_wall = time.time()
    ds = Dataset.open(a.dataset)
    summary = ds.summary()
    calib = load_calibration(a.calibration)
    lidar_topics = [t for t in ds.lidar_topics]
    short = {t: t.split("/")[-1] for t in lidar_topics}
    imu_topic = next((t for t in ds.imu_topics), None)
    odom_topic = next((t for t in ds.topics if t.endswith("imu_data/odometry")), None)
    status_topic = next((t for t in ds.topics if t.endswith("imu_data/ins_status")), None)
    cam_topic = next((t for t in ds.camera_topics), None) if not a.no_camera else None

    start_ns = ds.start_ns + int(a.window[0] * NS) if a.window else None
    end_ns = (start_ns + int(a.window[1] * NS)) if a.window else None
    span_s = (a.window[1] if a.window else ds.duration_s) or 1.0
    t0 = (start_ns if start_ns else ds.start_ns) / NS
    cam_windows = []
    if cam_topic and a.camera_samples > 0:
        cam_windows = [t0 + (k + 0.5) / a.camera_samples * span_s for k in range(a.camera_samples)]

    # ------------------------------------------------------------------ streaming pass (parallel over segment chunks)
    jobs = a.jobs if a.jobs and a.jobs > 0 else max(1, min(8, (os.cpu_count() or 2) // 2))
    chunks = plan_chunks(ds, jobs, start_ns, end_ns)
    specs = []
    for i, chunk in enumerate(chunks):
        c_lo = min(x.start_ns for x in chunk) / NS
        c_hi = max(x.end_ns for x in chunk) / NS
        specs.append({"dataset": str(ds.root), "segments": None if len(chunks) == 1 else [x.name for x in chunk], "first": i == 0,
                      "calibration": a.calibration, "lidar_topics": lidar_topics, "imu_topic": imu_topic, "odom_topic": odom_topic,
                      "status_topic": status_topic, "cam_topic": cam_topic, "start_ns": start_ns, "end_ns": end_ns, "t0": t0,
                      "span_s": span_s, "cam_windows": [w for w in cam_windows if c_lo <= w <= c_hi or (i == 0 and w < c_lo)
                                                        or (i == len(chunks) - 1 and w > c_hi)],
                      "args": {"sample_every": a.sample_every}})
    print(f"[assess] {len(ds.segments)} segments → {len(specs)} parallel pass(es)", flush=True)
    if len(specs) == 1:
        results = [scan_chunk(specs[0])]
    else:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(len(specs)) as pool:
            results = []
            for i, r in enumerate(pool.imap(scan_chunk, specs)):
                results.append(r)
                print(f"[assess] chunk {i + 1}/{len(specs)} done: {r['counters']['messages']} messages, "
                      f"{r['counters']['bytes'] / 1e9:.2f} GB, {r['elapsed']:.1f} s", flush=True)
    results = [r for r in results if not r.get("empty")]
    if not results:
        raise SystemExit("[assess] no messages of the required topics in the requested range")

    counters = {"messages": sum(r["counters"]["messages"] for r in results), "bytes": sum(r["counters"]["bytes"] for r in results)}
    dropped = {"dropped_duplicate": sum(r["dropped"].get("dropped_duplicate", 0) for r in results),
               "dropped_backwards": sum(r["dropped"].get("dropped_backwards", 0) for r in results)}
    lid_ts, lid_rec, lid_n = {}, {}, {}
    for t in lidar_topics:
        m, d1, b1 = _dedup([{"ts": r["lid_ts"][t], "rec": r["lid_rec"][t], "n": r["lid_n"][t]} for r in results], "ts", "rec", "n")
        lid_ts[t], lid_rec[t], lid_n[t] = m["ts"], m["rec"], m["n"]
        dropped["dropped_duplicate"] += d1
        dropped["dropped_backwards"] += b1
    m, d1, b1 = _dedup([{"ts": r["imu_ts"], "g": r["imu_gyro"], "rec": r["imu_rec"]} for r in results], "ts", "g", "rec")
    imu_ts, imu_gyro, imu_rec = m["ts"], m["g"], m["rec"]
    dropped["dropped_duplicate"] += d1
    dropped["dropped_backwards"] += b1
    m, d1, b1 = _dedup([r["odo"] for r in results], "ts", "p", "q", "vb", "cov")
    odo = m
    dropped["dropped_duplicate"] += d1
    dropped["dropped_backwards"] += b1
    m, d1, b1 = _dedup([{"ts": r["st_ts"], "mode": r["st_mode"], "sats": r["st_sats"]} for r in results], "ts", "mode", "sats")
    st_ts, st_mode, st_sats = m["ts"], m["mode"], m["sats"]
    dropped["dropped_duplicate"] += d1
    dropped["dropped_backwards"] += b1
    m, d1, b1 = _dedup([{"ts": r["cam_ts"], "rec": r["cam_rec"], "flags": r["cam_flags"], "size": r["cam_size"]} for r in results],
                       "ts", "rec", "flags", "size")
    cam_ts, cam_rec, cam_flags, cam_size = m["ts"], m["rec"], m["flags"], m["size"]
    dropped["dropped_duplicate"] += d1
    dropped["dropped_backwards"] += b1
    lid_start = next((r["lid_start"] for r in results if r.get("lid_start") is not None), {short[t]: [] for t in lidar_topics})
    slots_eval = sorted((x for r in results for x in r["slots_eval"]), key=lambda x: x["t_s"])
    cam_eval = sorted((x for r in results for x in r["cam_eval"]), key=lambda x: x["t_s"])
    cam_samples = cam_eval  # evaluation only needs the decoded samples
    elapsed_pass = time.time() - t_wall
    print(f"[assess] pass finished: {counters['messages']} messages, {counters['bytes'] / 1e9:.2f} GB in {elapsed_pass:.1f} s "
          f"({len(specs)} worker(s))", flush=True)

    # ------------------------------------------------------------------ evaluation
    R = Report()
    g_int = R.group("integrity", "完整性与覆盖", "话题、分段覆盖与消息顺序：缺失或断裂的数据没有任何算法可以补回。")
    g_time = R.group("timing", "时间同步", "IMU / 多雷达 / 相机的时间戳质量：常数偏移可标定，随机抖动与断流不可标定。")
    g_ref = R.group("reference", "参考系统 RTK / INS", "融合与评估的参考：其自身噪声与内部矛盾会直接成为地图误差与评估误差的下限。")
    g_lid = R.group("lidar", "雷达几何与外参", "点云密度、遮挡、结构可观性与静止段外参一致性（对应建图自检门限）。")
    g_cam = R.group("camera", "相机", "帧率、时间戳抖动与图像内容（曝光、纹理）：决定 LIVO 视觉约束是否可靠。")
    g_route = R.group("route", "行驶工况与流程", "静止起步、直线段、转弯、坡度与规模：决定标定各阶段是否有足够的可观测性。")
    g_res = R.group("resources", "回放与资源", "磁盘与读取带宽：不足会造成回放丢帧，与算法无关。")

    # --- integrity
    missing = summary.get("missing_lio_topics", [])
    core_missing = [m for m in missing if m in ("imu_data", "imu_data/odometry") or m.endswith("lidar")]
    R.add(g_int, "topics", "必需话题", len(missing), "项缺失",
          status="bad" if core_missing or not lidar_topics else ("warn" if missing else "ok"),
          display="齐全" if not missing else "缺 " + ", ".join(missing), fix="none",
          plain="标定与建图要用的话题全在吗？缺核心话题（IMU/雷达/INS 里程计）= 录包直接作废。",
          note="LIO 需要 LiDAR + imu_data + imu_data/odometry；ins_status 提供固定解/卫星数，navsat_fix 可选。",
          basis="缺 INS 里程计时无法做后轴对齐与 RTK 融合。")
    R.add(g_int, "lidar_count", "LiDAR 路数", len(lidar_topics), "路", ok=4, warn=1, higher=True, fix="none",
          display=f"{len(lidar_topics)} 路（{', '.join(short.values())}）",
          plain="车上 4 路雷达是否都录到了。少一路覆盖就缺一块，多雷达互校验也跑不全。",
          note="车辆配置为 4 路（front / front_left / front_right / back）；少于 4 路时覆盖与多雷达一致性检查按实际路数进行。",
          basis="truck29 / truck05 均为 4 路。")
    gaps = summary.get("coverage_gaps", [])
    R.add(g_int, "coverage_gaps", "分段覆盖断裂", len(gaps), "处", ok=0, warn=0, fix="none",
          display="无" if not gaps else f"{len(gaps)} 处（最长 {max(x['gap_s'] for x in gaps):.1f} s）",
          plain="相邻分段之间是否有秒级空洞。空洞=这段没数据，地图会断一块，前端要重收敛。",
          note="同一话题集合的相邻分段之间 >0.5 s 的空洞；空洞期间没有点云与 IMU，地图在此处缺失且前端需要重新收敛。",
          basis="dataset.py 覆盖检查；录制软件分段切换应无缝（truck29: 8 段无缝）。")

    data_ids, cal_id = vehicle_ids(ds, a.calibration)
    if data_ids and cal_id:
        same = any(n == cal_id[1] and (cal_id[0] is None or m == cal_id[0]) for m, n in data_ids)
        dname = "、".join(f"{m} {n} 号车" for m, n in sorted(data_ids))
        cname = f"{cal_id[0] + ' ' if cal_id[0] else ''}{cal_id[1]} 号车"
        R.add(g_int, "vehicle_match", "数据车号 vs 标定车号", 0 if same else 1, "", status="ok" if same else "bad",
              fix="stage", display=f"一致（{dname}）" if same else f"数据 {dname} · 标定 {cname}",
              plain="录包来自哪台车、所选标定属于哪台车。不一致时外参和时间偏移都不适用，下面的地面倾角/高度差等外参检查会虚高；"
                    "先为这台车导入并标定，再评估和建图。",
              note="数据车号取自分段名（如 Camera_wuhu_28_…），标定车号取自标定路径（…/wuhu/truck29/…）。",
              basis="不同车的雷达安装不同：用 truck29 外参看 28 号车会出现数度倾角差、米级高度差。")
    dup, back = dropped.get("dropped_duplicate", 0), dropped.get("dropped_backwards", 0)
    dd_pct = 100.0 * (dup + back) / max(1, counters["messages"])
    R.add(g_int, "ordering", "重复 / 乱序消息", dd_pct, "%", ok=0.5, warn=2.0, fix="auto",
          display=f"{dup + back} 条（重复 {dup}，倒退 {back}；{dd_pct:.2f} %）",
          plain="消息时间戳是否单调。分段边界几条重复正常；成片乱序说明录制时钟错乱，无法自动修复。",
          note="在线合并按每话题单调时间戳丢弃重复与倒退消息；分段边界的少量重复无影响，成片乱序说明录制时钟异常。",
          basis="truck29: 0；truck05: 621 条分段边界重复（0.23 %）。")
    R.add(g_int, "camera_present", "相机话题", 1 if cam_topic or ds.has_camera else 0, "", status="ok" if ds.has_camera else "warn",
          display=("有：" + ds.camera_topics[0]) if ds.has_camera else "无（仅可 LIO）", fix="none",
          plain="有没有录相机数据。没相机=只能跑 LIO，地图无彩色，相机标定/闭环也无法做。",
          note="没有相机时只能运行 LIO 与 LiDAR 标定流水线，无法着色。", basis="Camera_* 分段包含 midrange_camera/ffmpeg。")

    # --- timing: IMU
    imu_ts_a = np.asarray(imu_ts, dtype=np.int64)
    if len(imu_ts_a) > 10:
        dti = np.diff(imu_ts_a) / 1e6  # ms
        med = float(np.median(dti))
        gaps_i = np.flatnonzero(dti > 20.0)
        R.add(g_time, "imu_rate", "IMU 采样间隔", med, "ms", ok=11.0, warn=21.0, fix="none",
              display=f"{med:.2f} ms（{1000 / med:.0f} Hz）",
              plain="IMU 频率是否稳定在 100 Hz（10 ms）。掉频会放大急转时的去畸变误差。",
              note="前端去畸变与预积分按 100 Hz 设计；更低的采样率会放大高角速率段的去畸变误差。", basis="truck29/05: 10.0 ms。")
        R.add(g_time, "imu_max_gap", "IMU 最大断点", float(dti.max()), "ms", ok=15.0, warn=100.0, fix="auto", fix_bad="none",
              plain="IMU 最长一次丢样本多久。≤100 ms 前端用插值桥接；更长=这一段建图会断/漂。",
              note="≤100 ms 由去畸变积分跨越（按两侧样本均值桥接）；更长的断流会中止或漂移本段建图。",
              basis="config: maximum_time_gap 0.1 s；修复前一次 20 ms 断点中止了约 50 轮中的 4 轮。")
        R.add(g_time, "imu_gaps", "IMU 断点数 (>20 ms)", int(len(gaps_i)), "次", ok=0, warn=5, fix="auto", fix_bad="none",
              display=f"{len(gaps_i)} 次" + (f"（首个 @ {(imu_ts_a[gaps_i[0]] / NS - t0):.1f} s）" if len(gaps_i) else ""),
              plain="整段录包里 IMU 出现多少次 >20 ms 的小断点。零星几个可桥接；频繁=IMU 链路在丢包。",
              note="每个断点都是一次前端桥接；频繁断点说明 IMU 链路丢包。", basis="truck29: 0–1 次；truck05: 见结果。")
    else:
        R.add(g_time, "imu_rate", "IMU 采样间隔", None, "ms", status="bad", display="无 IMU 数据", fix="none")

    # --- timing: LiDAR per topic
    lid_arrays = {t: np.asarray(v, dtype=np.int64) for t, v in lid_ts.items()}
    drop_share, per_lidar = {}, {}
    for t, arr in lid_arrays.items():
        if len(arr) < 5:
            per_lidar[short[t]] = {"frames": int(len(arr))}
            drop_share[short[t]] = 100.0
            continue
        d = np.diff(arr) / 1e6
        med = float(np.median(d))
        missing_frames = int(np.round(d[d > 1.5 * med] / med).sum() - (d > 1.5 * med).sum()) if med > 0 else 0
        share = 100 * missing_frames / max(1, len(arr) + missing_frames)
        drop_share[short[t]] = share
        lat = np.asarray(lid_rec[t]) / 1e6
        per_lidar[short[t]] = {"frames": int(len(arr)), "dt_median_ms": round(med, 2), "dt_max_ms": round(float(d.max()), 1),
                               "missing_frames": missing_frames, "missing_pct": round(share, 3),
                               "points_median": int(np.median(lid_n[t])), "points_p05": int(np.percentile(lid_n[t], 5)),
                               "record_latency_ms": round(float(np.median(lat)), 1),
                               "record_latency_drift_ms": round(float(np.median(lat[-200:]) - np.median(lat[:200])), 1) if len(lat) > 400 else 0.0}
    worst_drop = max(drop_share.values()) if drop_share else None
    worst_topic = max(drop_share, key=drop_share.get) if drop_share else ""
    R.add(g_time, "lidar_drops", "LiDAR 丢帧率（最差一路）", worst_drop, "%", ok=0.1, warn=1.0, fix="none",
          display=f"{worst_drop:.2f} %（{worst_topic}）" if worst_drop is not None else "—",
          plain="最差的一路雷达缺了多少帧。缺帧=合并帧里这路缺席，覆盖少一圈，无法补回。",
          note="由帧间隔 >1.5 倍周期推断的缺帧；缺帧处该雷达在合并帧中缺席，覆盖减少，无法补回。",
          basis="truck29: 0 %；truck05 回放丢帧属于回放侧（本项只看录制）。", extra=per_lidar)
    # scan-start dispersion across LiDARs
    span_ms = None
    spans = []
    if len(lid_arrays) >= 2:
        ref_t = lidar_topics[0]
        for t in lidar_topics:
            if t.endswith(REFERENCE_LIDAR):
                ref_t = t
        others = [t for t in lidar_topics if t != ref_t and len(lid_arrays[t]) > 5]
        ref_arr = lid_arrays[ref_t]
        cols = [ref_arr]
        for t in others:
            arr = lid_arrays[t]
            j = np.clip(np.searchsorted(arr, ref_arr), 1, len(arr) - 1)
            near = np.where(np.abs(arr[j] - ref_arr) < np.abs(arr[j - 1] - ref_arr), arr[j], arr[j - 1])
            cols.append(near)
        cols = np.stack(cols, 1)
        complete = np.all(np.abs(cols - cols[:, :1]) < 50_000_000, axis=1)
        if complete.sum() > 10:
            spans = (cols[complete].max(1) - cols[complete].min(1)) / 1e6
            span_ms = percentile(spans, 99)
    over1 = int((spans > 1.0).sum()) if len(spans) else 0
    over5 = int((spans > 5.0).sum()) if len(spans) else 0
    over20 = int((spans > 20.0).sum()) if len(spans) else 0
    status_span = grade(span_ms, 1.0, 20.0)
    if len(spans) and over20 > 0.001 * len(spans):
        status_span = "bad"
    R.add(g_time, "lidar_span", "多雷达扫描起始离散 P99", span_ms, "ms", ok=1.0, warn=20.0, status=status_span, fix="auto", fix_bad="none",
          display=(f"{span_ms:.2f} ms（最大 {float(spans.max()):.2f}；>1 ms {over1} 帧，>20 ms {over20} 帧 / {len(spans)}）" if span_ms is not None else "—"),
          plain="同一合并帧内四路雷达 header 时间差。<1 ms 好；持续 >20 ms 说明某路雷达时钟跑偏，去畸变无法修复。",
          note="同一合并帧内四路雷达 header 时间戳的极差。默认合帧容差 20 ms，逐点保留真实采样时刻做 IMU 去畸变；超限时丢弃未匹配的旧帧并继续。此补偿不能修复传感器时钟错误。",
          basis="truck29 <1 ms；truck05 抖动 1.0–1.3 ms；zongyang truck07 稳定相位差 P99 16.58 ms，原 5 ms 容差使建图零帧。",
          extra={"count": int(len(spans)), "over_1ms": over1, "over_5ms": over5, "over_20ms": over20, "max_ms": float(spans.max()) if len(spans) else None})
    # clock domains: record time - header time
    lat_med = {}
    for t in lidar_topics:
        if lid_rec[t]:
            lat_med[short[t]] = float(np.median(np.asarray(lid_rec[t]) / 1e6))
    if imu_rec:
        lat_med["imu"] = float(np.median(np.asarray(imu_rec) / 1e6))
    if cam_rec:
        lat_med["camera"] = float(np.median(np.asarray(cam_rec) / 1e6))
    worst_lat = max(abs(v) for v in lat_med.values()) if lat_med else None
    R.add(g_time, "clock_offset", "录制时间 − 传感器时间戳", worst_lat, "ms", ok=300.0, warn=1000.0, fix="none",
          display=" · ".join(f"{k} {v:+.0f}" for k, v in lat_med.items()) + " ms" if lat_med else "—",
          plain="各传感器时间戳和主机录制时刻的中位差。几百 ms 以内正常；秒级差异=时钟没同步，常数时间偏移无法覆盖漂移。",
          note="各传感器 header 时间戳与录制时间的中位差；LiDAR 以扫描起始计时约 +100…+200 ms 属正常。秒级差异表示传感器时钟未与主机同步，标定阶段的常数时间偏移无法覆盖漂移。",
          basis="truck29/05 各传感器均在 300 ms 内。", extra=lat_med)

    # --- timing: camera
    cam_ts_a = np.asarray(cam_ts, dtype=np.int64)
    if cam_topic and len(cam_ts_a) > 10:
        dtc = np.diff(cam_ts_a) / 1e6
        medc = float(np.median(dtc))
        jitter = float(np.percentile(np.abs(dtc - medc), 95))
        gaps_c = int((dtc > 2.5 * medc).sum())
        R.add(g_cam, "camera_rate", "相机帧间隔", medc, "ms", ok=40.0, warn=120.0, fix="none",
              display=f"{medc:.2f} ms（{1000 / medc:.1f} fps，{len(cam_ts_a)} 帧）",
              plain="相机帧率是否稳定在额定值（25/30 fps）。<10 fps 视觉约束太稀疏，LIVO 退化到 LIO。",
              note="LIVO 每帧图像做一次视觉更新；帧率低于 10 fps 时视觉约束稀疏。", basis="truck29/05: 30 fps。")
        R.add(g_cam, "camera_jitter", "相机时间戳抖动 P95", jitter, "ms", ok=1.0, warn=3.0, fix="none",
              display=f"{jitter:.2f} ms（最大 {float(np.abs(dtc - medc).max()):.1f} ms）",
              plain="帧到帧时间间隔稳不稳。常数偏移能标掉；逐帧抖直接变成投影误差（3 ms ≈ 2 cm），只能靠硬触发/PTP解决。",
              note="帧间隔相对中位数的偏差。常数偏移由 camtime 阶段标定；逐帧抖动直接进入投影误差（6.5 m/s × 3 ms ≈ 2 cm）并降低颜色对齐，只能靠硬件触发 / PTP 解决。",
              basis="truck29: 亚毫秒；truck05: ±ms 级抖动（相机常数偏移 −1.9 ms 已标定，抖动不可标定）。")
        R.add(g_cam, "camera_gaps", "相机断帧", gaps_c, "次", ok=0, warn=5, fix="none",
              plain="有没有长时间没画面。断帧期间没视觉更新，HEVC 还要等下一关键帧才能恢复。",
              note="帧间隔 >2.5 倍周期；断帧期间无视觉更新且 HEVC 需等下一关键帧。", basis="truck29: 0。")
    elif cam_topic:
        R.add(g_cam, "camera_rate", "相机帧间隔", None, "ms", status="bad", display="相机话题无数据", fix="none")

    # --- reference (RTK / INS)
    ref_stats = {}
    static_s = None
    moving_mask = None
    ots = np.asarray(odo["ts"], dtype=np.int64)
    if len(ots) > 200:
        from scipy.spatial.transform import Rotation as Rot
        t = (ots - ots[0]) / 1e9
        P = np.asarray(odo["p"])
        Q = np.asarray(odo["q"])
        VB = np.asarray(odo["vb"])
        COV = np.asarray(odo["cov"])
        qn = np.linalg.norm(Q, axis=1)
        good_q = np.abs(qn - 1) < 1e-3
        Qn = Q / np.where(qn[:, None] > 0, qn[:, None], 1)
        rot = Rot.from_quat(Qn)
        VW = rot.apply(VB)
        fwd = rot.apply(np.tile([0.0, 1.0, 0.0], (len(Qn), 1)))
        dt = np.diff(t)
        dP = np.diff(P, axis=0)
        speed_ins = np.linalg.norm(VW[:, :2], axis=1)
        # moving detection on 0.5 s position windows (duration-based: rate changes no longer shift the
        # window width; also keeps the 0.5 s consistency stats below aligned to a true 0.5 s)
        W_TARGET_S = 0.5
        # find the sample stride closest to W_TARGET_S using the median dt
        med_dt = float(np.median(dt[dt > 0])) if (dt > 0).any() else 0.01
        W = max(1, int(round(W_TARGET_S / med_dt)))
        if len(P) > W + 1:
            disp = np.linalg.norm(P[W:, :2] - P[:-W, :2], axis=1)
            tw = t[W:] - t[:-W]
            spd_w = disp / np.maximum(tw, 1e-3)
            moving_w = spd_w > 0.5
            first_move = int(np.argmax(moving_w)) if moving_w.any() else len(spd_w)
            static_s = float(t[first_move]) if moving_w.any() else float(t[-1])
            moving_mask = np.zeros(len(P), bool)
            idx = np.flatnonzero(moving_w)
            moving_mask[idx] = True
            moving_mask[np.minimum(idx + W, len(P) - 1)] = True
        else:
            static_s = 0.0
            moving_mask = np.ones(len(P), bool)
        valid = (dt > 0.004) & (dt < 0.03)
        gaps_o = int((dt > 0.05).sum())
        # position-stream artefacts that are not jumps: a held sample followed by a double step
        # (truck29: 1.7 % of samples) and a skipped stamp (truck05). Jumps are therefore measured on
        # 5-sample-smoothed positions over 0.1 s windows and clustered into events.
        vmid = 0.5 * (VW[1:] + VW[:-1])
        e_step = dP - vmid * dt[:, None]
        exy = np.linalg.norm(e_step[:, :2], axis=1) * 100
        mv = moving_mask[1:] & valid
        vdt = np.linalg.norm(vmid[:, :2], axis=1) * dt
        stale = (np.linalg.norm(dP[:, :2], axis=1) < 0.25 * vdt) & mv & (vdt > 0.01)
        stale_pct = 100 * float(stale.sum() / max(1, mv.sum()))
        P5 = P.copy()
        if len(P) > 5:
            P5[2:-2] = (P[:-4] + P[1:-3] + P[2:-2] + P[3:-1] + P[4:]) / 5.0
        cum = np.concatenate([np.zeros((1, 3)), np.cumsum(vmid * dt[:, None], axis=0)])
        S10 = 10
        ew10 = np.linalg.norm(((P5[S10:] - P5[:-S10]) - (cum[S10:] - cum[:-S10]))[:, :2], axis=1) * 100
        m10 = moving_mask[:-S10] & moving_mask[S10:]
        over = np.flatnonzero((ew10 > 4.0) & m10)
        events = []
        for i in over:
            if events and i - events[-1][1] <= 30:
                events[-1][1] = i
                events[-1][2] = max(events[-1][2], float(ew10[i]))
            else:
                events.append([int(i), int(i), float(ew10[i])])
        jumps = np.array([e[0] for e in events], dtype=int)
        jump_max = max((e[2] for e in events), default=0.0)
        per10 = 600.0 / max(float(dt[moving_mask[1:]].sum()), 1.0)
        # 0.5 s window consistency on 3-sample-smoothed positions (removes the hold/double-step artefact)
        Ps = P.copy()
        Ps[1:-1] = (P[:-2] + P[1:-1] + P[2:]) / 3.0
        exy_w = ez_w = np.array([])
        if len(P) > W + 1:
            dpw = Ps[W:] - Ps[:-W]
            ivw = cum[W:] - cum[:-W]
            e_w = dpw - ivw
            mw = moving_mask[:-W] & moving_mask[W:]
            gapfree = np.ones(len(mw), bool)
            for b in np.flatnonzero(~valid):
                gapfree[max(0, b - W):min(len(mw), b + 1)] = False
            mw &= gapfree
            exy_w = np.linalg.norm(e_w[mw, :2], axis=1) * 100
            ez_w = np.abs(e_w[mw, 2]) * 100
        sp_pos = np.linalg.norm(Ps[S10:, :2] - Ps[:-S10, :2], axis=1) / np.maximum(t[S10:] - t[:-S10], 1e-3)
        # heading vs track
        trk = P[S10:, :2] - P[:-S10, :2]
        fast = m10 & (sp_pos > 2.0)
        ang = np.array([])
        if fast.sum() > 20:
            f2 = fwd[:-S10][fast][:, :2]
            tr = trk[fast]
            ang = np.degrees(np.arctan2(f2[:, 0] * tr[:, 1] - f2[:, 1] * tr[:, 0], (f2 * tr).sum(1)))
        # grade / height consistency
        dh = np.linalg.norm(dP[:, :2], axis=1)
        fz = np.clip(fwd[:, 2], -0.5, 0.5)
        slope = fz / np.sqrt(1 - fz ** 2)
        H_pitch = np.concatenate([[0.0], np.cumsum(dh * 0.5 * (slope[1:] + slope[:-1]))])
        H_rtk = P[:, 2] - P[0, 2]
        D = np.concatenate([[0.0], np.cumsum(dh)])
        seg = 100.0
        diffs = []
        if D[-1] > 30.0:
            L = min(seg, D[-1])
            j = np.searchsorted(D, D + L)
            ok_j = j < len(D)
            i_idx = np.flatnonzero(ok_j)[::10]
            for i in i_idx:
                jj = j[i]
                diffs.append(((H_pitch[jj] - H_pitch[i]) - (H_rtk[jj] - H_rtk[i])) * 100 / L * 100)
        diffs = np.asarray(diffs) if diffs else np.array([])
        gh_bias = float(np.mean(diffs)) if len(diffs) else 0.0
        diffs = np.abs(diffs - gh_bias)
        grade_pct = np.abs(slope[moving_mask]) * 100 if moving_mask.any() else np.array([])
        cov_valid = float((COV[:, 0] > 0).mean())
        ref_stats = {"samples": int(len(P)), "path_m": float(D[-1]), "static_start_s": static_s,
                     "moving_s": float(dt[moving_mask[1:]].sum()), "speed_max_mps": float(np.percentile(speed_ins, 99.5)),
                     "speed_median_moving_mps": float(np.median(speed_ins[moving_mask])) if moving_mask.any() else 0.0,
                     "ins_gaps": gaps_o, "quaternion_bad": int((~good_q).sum()),
                     "step_xy_p99_cm": percentile(exy[mv], 99), "step_jumps": int(len(jumps)), "step_jumps_per_10min": len(jumps) * per10,
                     "step_jump_max_cm": jump_max, "win01_xy_p99_cm": percentile(ew10[m10], 99),
                     "stale_pct": stale_pct, "jump_times_s": [round(float(t[i]), 2) for i in jumps[:20]],
                     "win_xy_p50_cm": percentile(exy_w, 50), "win_xy_p95_cm": percentile(exy_w, 95),
                     "win_z_p50_cm": percentile(ez_w, 50), "win_z_p95_cm": percentile(ez_w, 95),
                     "heading_track_deg_median": float(np.median(ang)) if len(ang) else None,
                     "heading_track_deg_p95": percentile(np.abs(ang - np.median(ang)), 95) if len(ang) else None,
                     "grade_height_p95_cm_per_100m": percentile(diffs, 95), "grade_height_max_cm_per_100m": float(diffs.max()) if len(diffs) else None,
                     "grade_height_bias_cm_per_100m": gh_bias,
                     "grade_p99_pct": percentile(grade_pct, 99), "cov_valid_share": cov_valid}
        # metrics
        if st_mode:
            modes = np.asarray(st_mode)
            fix_share = 100 * float((modes == 4).mean())
            sats = np.asarray(st_sats)
            sats = sats[sats >= 0]
            R.add(g_ref, "rtk_fix", "RTK 固定解占比", fix_share, "%", ok=99.0, warn=95.0, higher=True, fix="none",
                  display=f"{fix_share:.2f} %（模式分布 {', '.join(f'{int(k)}:{int(v)}' for k, v in zip(*np.unique(modes, return_counts=True)))}）",
                  plain="RTK 是否全程固定解。浮点/单点期间参考位置差到分米级，融合会被带偏、评估也失真。",
                  note="ins_pos_mode=4 为 RTK 固定解；浮点/单点期间参考位置误差达分米级，融合会被拉偏且评估失真。",
                  basis="truck29/05 全程模式 4。")
            if len(sats):
                p10 = float(np.percentile(sats, 10))
                R.add(g_ref, "sats", "可用卫星数 P10", p10, "颗", ok=30.0, warn=20.0, higher=True, fix="none",
                      display=f"{p10:.0f} 颗（最少 {int(sats.min())}，中位 {int(np.median(sats))}）",
                      plain="跟踪的卫星数。≥30 颗时 RTK 逐历元抖动在 cm 级；<20 颗固定解虽在但噪声明显变大。",
                      note="卫星数直接决定 RTK 固定解的几何强度与逐历元抖动；≥30 颗时参考位置逐步一致性达 1 cm 级。",
                      basis="truck29: 32–42 颗，参考 0.5 s 一致性 P95 1.3 cm；truck05: 22–24 颗，P95 10 cm，|Δv| 26 cm/s。")
        else:
            R.add(g_ref, "rtk_fix", "RTK 固定解占比", None, "%", status="warn", display="无 ins_status", fix="none",
                  plain="录包里没有 ins_status 话题，无法判断固定解比例。参考精度只能靠位置-速度一致性间接估。",
                  note="没有 ins_status 就无法判断固定解与卫星数。")
        R.add(g_ref, "ins_gaps", "INS 里程计断点 (>50 ms)", gaps_o, "次", ok=0, warn=3, fix="auto", fix_bad="none",
              plain="INS 里程计出现多少次 >50 ms 断点。断点期间融合只能靠前端推算，频繁断点=约束变稀。",
              note="断点期间融合只能靠前端里程推算；频繁断点降低 RTK 约束密度。", basis="truck29/05: 0。")
        R.add(g_ref, "ref_step_jumps", "参考位置阶跃事件 (>4 cm / 0.1 s)", len(jumps) * per10, "次/10min", ok=5.0, warn=150.0, fix="none",
              display=f"{len(jumps)} 次（{len(jumps) * per10:.0f} 次/10 min，最大 {jump_max:.1f} cm）" + (f"，首个 @ {t[jumps[0]]:.1f} s" if len(jumps) else ""),
              plain="RTK 位置有没有突然跳一下（比 INS 速度积分多 >4 cm）。这种跳变原样出现在“误差”里，融合和评估都分不清它是跳还是真偏差。",
              note="0.1 s 内位置增量与 INS 速度积分之差 >4 cm（位置经 5 样本平滑，停顿/补步伪影已剔除）：参考位置相对自身速度流的阶跃，会原样出现在“误差”里，融合与评估都无法区分。",
              basis="truck29: 0 次；truck05: 72 次（每 3 s 一次 3–5 cm、最大 13.7 cm 的位置重整），397.24 s 单步 8.0 cm（INS 速度只允许 4.1 cm）→ 该处 xy 误差 10.1 cm。")
        R.add(g_ref, "ref_stale", "参考位置停顿样本", stale_pct, "%", ok=3.0, warn=10.0, fix="none",
              display=f"{stale_pct:.2f} %",
              plain="RTK 位置流有没有“卡住一拍、下一拍跳两步”的样本。偶尔出现对融合鲁棒，但评估曲线会带毛刺。",
              note="100 Hz 位置流中“保持上一值、下一样本补双步”的样本占比；单个停顿在 6.5 m/s 下等于 3 cm 的瞬时参考误差，融合对其鲁棒但评估曲线会带毛刺。",
              basis="truck29: 1.7 %（精度仍 6 cm）；truck05: 0.1 %。")
        R.add(g_ref, "ref_xy_consistency", "参考位置-速度一致性 xy P95 (0.5 s)", percentile(exy_w, 95), "cm", ok=3.0, warn=8.0, fix="none",
              display=f"{percentile(exy_w, 95) or 0:.2f} cm（P50 {percentile(exy_w, 50) or 0:.2f}）" if len(exy_w) else "—",
              plain="RTK 0.5 s 内位置差和速度积分对不对得上（平面）。这就是参考本身的逐历元噪声，融合和评估都没法再小。",
              note="0.5 s 内位置增量与速度积分之差（位置经 3 样本平滑）；反映 RTK 位置的逐历元抖动，是融合与评估都无法区分的参考噪声。",
              basis="truck29 P50/P95 0.6/2.2 cm；truck05 0.9/5.0 cm（卫星 22–24 颗）。")
        R.add(g_ref, "ref_z_consistency", "参考高程-速度一致性 z P95 (0.5 s)", percentile(ez_w, 95), "cm", ok=3.0, warn=12.0, fix="none",
              display=f"{percentile(ez_w, 95) or 0:.2f} cm（P50 {percentile(ez_w, 50) or 0:.2f}）" if len(ez_w) else "—",
              plain="同上，但测高程。RTK 高程噪声通常是平面 2–3 倍，直接决定坡道上 z 的精度下限。",
              note="高程方向的同类指标；RTK 高程噪声通常是平面的 2–3 倍，直接决定坡道上的 z 精度。", basis="truck29 0.29/1.37 cm；truck05 1.55/10.3 cm（P50/P95）。")
        if len(ang):
            R.add(g_ref, "heading_track", "INS 航向 vs 轨迹方向（中位）", float(np.median(ang)), "°", ok=1.0, warn=2.5, fix="stage",
                  display=f"{float(np.median(ang)):+.2f}°（P95 抖动 {percentile(np.abs(ang - np.median(ang)), 95):.2f}°）",
                  plain="车开起来（>2 m/s）时 INS 指的方向和轨迹方向差几度。常数偏角=INS→后轴的偏航安装角，由后轴对齐阶段标掉；抖动=参考噪声。",
                  note="行驶 >2 m/s 时车体前向轴与位置轨迹方向的夹角；常数项等价于 INS→后轴的偏航安装角，由后轴对齐阶段吸收；抖动项来自参考噪声。",
                  basis="axle 阶段（rpy_xy）求解参考雷达→后轴的整机旋转。")
        R.add(g_ref, "grade_height", "俯仰积分高程 vs RTK 高程 P95", percentile(diffs, 95), "cm/100m", ok=25.0, warn=45.0, fix="none",
              display=f"{percentile(diffs, 95):.1f} cm/100m（去常数偏差 {gh_bias:+.1f} 后；最大 {float(diffs.max()):.1f}）" if len(diffs) else "行驶距离不足",
              plain="按 INS 俯仰积分的爬坡高度和 RTK 报的高度差多少（每 100 m）。差得大=姿态与高程在坡道上互相矛盾，z 误差随坡度出现，标定消不掉。",
              note="按 INS 俯仰角积分的高程变化与 RTK 高程变化之差（每 100 m 行程，扣除常数俯仰安装偏差后）。差异越大，INS 姿态与 RTK 高程在坡道上越矛盾，融合的 z 误差随坡度出现，标定无法消除。",
              basis="truck29 P95 19 cm/100m（坡道 |z| ≤ 5 cm）；truck05 39 cm/100m（+6–8 % 坡 z +13…+18 cm，−6–7 % 坡 z −20…−34 cm）。")
        R.add(g_ref, "ins_cov", "INS 协方差字段", cov_valid * 100, "%", status="ok" if cov_valid > 0.5 else "info",
              display="有效" if cov_valid > 0.5 else "全 0（按固定权重融合）", fix="none",
              plain="INS 是否给了逐历元协方差。全 0=融合按配置里的固定 σ 加权，不按实时精度自适应；固定解质量好的数据没影响。",
              note="协方差全 0 时融合使用配置中的固定 σ，无法按逐历元精度加权；不影响固定解质量好的数据。",
              basis="truck29/05 均为全 0。")
        R.add(g_ref, "ins_quat", "INS 姿态四元数", int((~good_q).sum()), "条异常", ok=0, warn=10, fix="none",
              plain="INS 给的姿态四元数模长是否=1。异常姿态会污染去畸变和融合。",
              note="模长偏离 1 的四元数；异常姿态会污染去畸变与融合。", basis="truck29/05: 0。")
    else:
        R.add(g_ref, "rtk_fix", "INS 里程计", None, "", status="bad", display="无 imu_data/odometry", fix="none",
              plain="完全没有 INS 里程计话题。没它=RTK 融合、后轴对齐、精度评估全都没法做，录包作废。",
              note="没有 INS 里程计无法做 RTK 融合、后轴对齐与精度评估。")

    # --- LiDAR geometry
    pts_med = {k: v["points_median"] for k, v in per_lidar.items() if "points_median" in v}
    if pts_med:
        weakest = min(pts_med, key=pts_med.get)
        R.add(g_lid, "lidar_points", "每帧点数（最稀一路中位）", pts_med[weakest], "点", ok=5000, warn=1500, higher=True, fix="none",
              display=" · ".join(f"{k} {v / 1000:.0f}k" for k, v in pts_med.items()),
              plain="每路雷达一帧回来多少点。<1.5k 点/帧=该路太稀，行驶段 6-DoF 精修会缺匹配。",
              note="点数骤降表示遮挡、污损或回波异常；侧/后雷达低于 1.5k 点时行驶段 6-DoF 精修缺少匹配。", basis="truck29 各路点数为参照。",
              extra=pts_med)
        blocked = {}
        for lt in lidar_topics:
            arr = np.asarray(lid_n[lt])
            med = np.median(arr)
            blocked[short[lt]] = 100 * float((arr < 0.3 * med).mean()) if med > 0 else 100.0
        wb = max(blocked, key=blocked.get)
        R.add(g_lid, "lidar_blocked", "点数骤降帧占比（最差一路）", blocked[wb], "%", ok=0.0, warn=2.0, fix="none",
              display=f"{blocked[wb]:.2f} %（{wb}）",
              plain="有多少帧点数比自己的中位数低 70% 以上。多=雷达被挡、被糊、或镜头/线束有问题。",
              note="点数低于该路中位数 30 % 的帧：短时遮挡（车辆、扬尘）或雷达异常。", basis="truck29: 0 %。", extra=blocked)
    # sampled slots: vertical structure availability (needs calibration)
    if calib and slots_eval:
        arr = np.asarray([s["above_ground"] for s in slots_eval])
        open_share = 100 * float((arr < 300).mean()) if len(arr) else None
        R.add(g_lid, "vertical_structure", "结构缺失帧占比（30 m 内地面以上 <300 点）", open_share, "%", ok=10.0, warn=30.0, fix="none",
              display=f"{open_share:.1f} %（{len(arr)} 个采样帧，中位 {int(np.median(arr))} 点）" if len(arr) else "—",
              plain="多少帧 30 m 内几乎没电线杆/墙/设备（空场）。空场段 LiDAR 沿行进方向退化，行驶段精修靠 RTK 兜底。",
              note="开阔无结构路段：LiDAR 里程计沿行进方向退化、行驶段精修缺少竖直面匹配；只能依靠 RTK 约束。",
              basis="truck05 204 s 开阔料场：视觉/LiDAR 约束弱，LIO 仍 ≤3 cm 因 RTK 约束。", extra={"slots": slots_eval[:400]})
    elif not calib:
        R.add(g_lid, "vertical_structure", "结构缺失帧占比", None, "%", status="na", display="未提供标定文件", fix="none",
              plain="需要激活的标定把点云变到后轴坐标系才能统计结构；先做后轴对齐再评估。",
              note="需要激活的标定版本把点云变换到后轴坐标系。")
    # static-start ground consistency
    gc = {}
    if calib and static_s is not None and static_s >= 2.5:
        gc = ground_consistency(lid_start, calib["matrices"])
    solved = {k: v for k, v in gc.items() if v.get("solved")}
    if solved:
        wt = max(solved, key=lambda k: solved[k]["tilt_deg"])
        wz = max(solved, key=lambda k: abs(solved[k]["dz_cm"]))
        tilt = solved[wt]["tilt_deg"]
        dz = abs(solved[wz]["dz_cm"])
        st_t = "ok" if tilt <= 0.2 else "warn" if tilt <= 1.0 else "bad"
        st_z = "ok" if dz <= 4.0 else "warn" if dz <= 15.0 else "bad"
        R.add(g_lid, "extrinsic_tilt", "静止段雷达间地面倾角差（最大）", tilt, "°", status=st_t, fix="auto",
              fix_bad="stage",
              display=f"{tilt:.2f}°（{wt}：前后向 {solved[wt]['rot_x_deg']:+.2f}° 左右向 {solved[wt]['rot_y_deg']:+.2f}°）",
              limits="≤ 0.2° 正常（低于自检生效阈值）· ≤ 1.0° 关注（建图自检自动修正）· > 1° 需 ground 阶段重标（先按地面平面粗对齐再精修，数度的安装变动也能收敛）并经 driven 复核；> 3° 多为安装变动或标定文件不属于这台车（见“数据车号 vs 标定车号”）",
              plain="起步静止时各路雷达“看到的地面”和参考雷达差几度。≤0.2° 可接受；0.2–1° 起步自检会自动修正；>1° 需要 ground 阶段重标（地面阶段会先用平面粗对齐，几度的安装变动也能收敛），再由 driven 阶段复核。",
              note="各雷达静止地面相对参考雷达（front）地面的倾角差；对应 SLAM 起步静止外参自检的 roll/pitch 修正量。",
              basis="config: max_rotation_deg 1.0 / min_apply 0.2°；truck05 原始外参 0.3° 由 driven 阶段闭合。",
              extra={k: {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items()} for k, v in gc.items()})
        R.add(g_lid, "extrinsic_z", "静止段雷达间地面高度差（最大）", dz, "cm", status=st_z, fix="auto",
              fix_bad="stage",
              display=f"{dz:.1f} cm（{wz}，重叠 {solved[wz]['overlap_cells']} 格，残差 {solved[wz]['residual_cm']:.1f} cm）",
              limits="≤ 4 cm 正常 · ≤ 15 cm 关注（自检自动修正）· > 15 cm 需 ground 阶段重标；> 50 cm 多为安装变动或标定文件不属于这台车",
              plain="静止地面各路看出来的高度差几厘米。≤4 cm 好；4–15 cm 起步自检修；>15 cm 要 ground 阶段。",
              note="对应自检的 z 修正量（max_height 0.15 m / min_apply 0.04 m）。", basis="truck05 原始外参：后雷达 14 cm 沿车 + 6 cm z。")
    else:
        reason = "未提供标定文件" if not calib else ("起步无静止段（<2.5 s）" if static_s is not None and static_s < 2.5 else
                                             ("静止地面与参考雷达无重叠" if gc else "静止帧不足"))
        R.add(g_lid, "extrinsic_tilt", "静止段雷达间地面一致性", None, "", status="na", display=reason, fix="stage",
              plain="起步时没有 ≥2.5 s 静止或没给标定，地面互校算不出。这种数据只能靠行驶段 driven 解外参。",
              note="没有静止段时建图自检与 ground 阶段无法运行；外参只能由行驶段 driven 阶段求解。")

    # --- camera content
    if cam_topic and cam_eval:
        dec = [c for c in cam_eval if c["decoded"]]
        R.add(g_cam, "camera_decode", "HEVC 样例解码", len(dec), "帧", status="ok" if len(dec) == len(cam_eval) and dec else ("warn" if dec else "bad"),
              display=f"{len(dec)} / {len(cam_eval)} 个样例可解码", fix="none",
              plain="抽样的几段 HEVC 能不能正常解码。解不开=那段视频没法做视觉更新也没颜色。",
              note="每个样例从关键帧起解码 4 个包；解码失败的区间没有视觉更新与颜色。", basis="")
        if dec:
            clip = max(c["clipped_pct"] for c in dec)
            tex = min(c["texture"] for c in dec)
            means = [c["mean"] for c in dec]
            R.add(g_cam, "camera_clipping", "过曝像素占比（最差样例）", clip, "%", ok=3.0, warn=10.0, fix="procedure",
                  display=f"{clip:.1f} %（亮度均值 {min(means):.0f}–{max(means):.0f}）",
                  plain="过曝（纯白）像素占多少。过曝区没纹理，视觉约束失效，彩色地图还会被洗白；调曝光或避开顺光段。",
                  note="过曝区域没有纹理，视觉更新在此退化并可能把错误的亮度写进彩色地图；需调整曝光/增益或避开顺光时段。",
                  basis="truck05 204 s 顺光开阔料场的视觉瞬态（LIO ≤3 cm，LIVO 跳变）。")
            R.add(g_cam, "camera_texture", "图像纹理（最弱样例，Laplacian 方差）", tex, "", ok=40.0, warn=15.0, higher=True, fix="none",
                  display=f"{tex:.0f}（中位 {np.median([c['texture'] for c in dec]):.0f}）",
                  plain="画面里有没有角点/边缘等可跟踪的纹理。天空/平整料面/过曝=低纹理，视觉约束不可靠，LIVO 退回 LIO 水平。",
                  note="低纹理（天空/平整料面/过曝）时视觉约束不可靠；LIVO 会自动降低视觉权重但精度退回 LIO 水平，标定无法改善。",
                  basis="truck29 全程有灯柱/挡墙/设备纹理。")
            lo = sum(1 for m in means if m < 40 or m > 200)
            R.add(g_cam, "camera_exposure", "曝光异常样例", lo, "个", ok=0, warn=2, fix="procedure",
                  display=f"{lo} / {len(dec)}（均值 <40 或 >200）",
                  plain="样例里整幅过暗/过亮的有几张。整体偏暗/偏亮的图给不出稳定的光度残差。",
                  note="整体过暗/过亮的图像不能提供稳定的光度残差。", basis="")
    elif cam_topic:
        R.add(g_cam, "camera_decode", "HEVC 样例解码", 0, "帧", status="warn", display="未采到关键帧样例", fix="none",
              plain="抽到的样例里没遇到关键帧，解码不出来。大概率是关键帧间隔问题，换几个时刻重抽。")

    # --- route / procedure
    if ref_stats:
        R.add(g_route, "static_start", "起步前静止时长", static_s, "s", ok=3.0, warn=1.0, higher=True, fix="procedure",
              plain="车子开动前停了几秒。≥3 s 好（够 ground 阶段和起步自检用）；<1 s 静止段无法用。",
              note="静止外参自检需 ≥5 帧、ground 阶段需 ~2 s 静止；从静止开始录制。", basis="truck29/05 均从静止起步。")
        R.add(g_route, "path", "行驶里程 / 时长", ref_stats["path_m"], "m",
              status="ok" if ref_stats["path_m"] >= 200 else "warn", fix="procedure",
              display=f"{ref_stats['path_m']:.0f} m · 行驶 {ref_stats['moving_s']:.0f} s / 总 {span_s:.0f} s · 最高 {ref_stats['speed_max_mps'] * 3.6:.0f} km/h",
              plain="一共开了多远多久。标定需要 ≥200 m 行驶；里程越长 RTK 对时间偏移的约束越强。",
              note="行驶段精修与后轴对齐需要 ≥200 m 的行驶；里程越长 RTK 对时间偏移的约束越强。", basis="")
        # turns and straight windows from 1 Hz profile
        tt = (ots - ots[0]) / 1e9
        secs = np.arange(0, int(tt[-1]) + 1)
        yaw = np.unwrap(np.arctan2(fwd[:, 1], fwd[:, 0]))
        y1 = np.interp(secs, tt, yaw)
        d1 = np.interp(secs, tt, D)
        v1 = np.r_[np.diff(d1), 0.0]
        w1 = np.r_[np.abs(np.diff(y1)), 0.0]
        # count heading changes >= 60 deg accumulated over consecutive turning seconds
        turns, acc = 0, 0.0
        for k in range(len(secs)):
            if np.degrees(w1[k]) > 3.0 and v1[k] > 0.5:
                acc += np.degrees(w1[k])
            else:
                if acc >= 60:
                    turns += 1
                acc = 0.0
        if acc >= 60:
            turns += 1
        straight = 0
        k = 0
        while k + 4 <= len(v1):
            if v1[k:k + 4].min() >= 1.5 and np.degrees(w1[k:k + 4]).max() <= 2.0:
                straight += 1
                k += 4
            else:
                k += 1
        R.add(g_route, "turns", "转弯次数 (≥60°)", turns, "次", ok=2, warn=1, higher=True, fix="procedure",
              plain="录到了几次 ≥60° 的转弯。后轴对齐和相机时间偏移靠转弯段激发可观性；全程直线这些量标不准。",
              note="转弯段给出 LiDAR/相机时间偏移与后轴对齐的可观测性（直线段对时间偏移不敏感）。", basis="axle / camtime 阶段依赖角速率。")
        R.add(g_route, "straight_windows", "直线匀速窗口 (4 s, ≥1.5 m/s)", straight, "个", ok=3, warn=1, higher=True, fix="procedure",
              plain="有几段 ≥4 s 的直线匀速。ground 阶段取后雷达行驶帧、driven 阶段默认窗口都要用到直线段。",
              note="ground 阶段的后雷达行驶帧与 driven 阶段窗口选择需要直线段。", basis="livo_calib suggest_windows 规则。")
        kf = ref_stats["moving_s"] * 2.0
        R.add(g_route, "keyframes", "预计关键帧数（0.5 s 一帧）", kf, "帧", ok=1000, warn=1500, fix="playback", status="ok" if kf <= 1000 else "warn",
              display=f"≈{kf:.0f} 帧",
              plain="整段大概会进后端多少关键帧。这不是数据缺陷：1000 帧以内可整段实时回放；更多时在建图页设“起点 / 时长”分段回放或降倍速即可，否则末段可能丢帧。",
              note="后端 iSAM2 求解时间随关键帧数增长（P50 78 ms@481 → 298 ms@1286 帧）；超过 1000 帧建议分段回放或 --rate 0.7，否则末段传输丢帧。",
              basis="truck05 720 s：末段负载丢帧使 LIO 最大误差 8.3 → 13.5 cm。")
        gp = ref_stats["grade_p99_pct"]
        R.add(g_route, "grade", "坡度 P99", gp, "%", status="info", fix="none",
              plain="录到的最大坡度。坡度本身不是问题，要搭配“俯仰积分高程 vs RTK 高程”判断坡道上参考准不准。",
              note="坡度本身不是缺陷；结合“俯仰积分高程 vs RTK 高程”判断参考在坡道上的可信度。", basis="truck29 8.6 %；truck05 +8 % / −7 % 坡。")

    # --- resources
    size_gb = summary["size_bytes"] / 1e9
    need_gb = 0.5 * size_gb + 2.0
    try:
        results_dir = pathlib.Path(__file__).resolve().parents[2] / "results"
        free_gb = shutil.disk_usage(results_dir if results_dir.exists() else pathlib.Path.home()).free / 1e9
    except OSError:
        free_gb = None
    R.add(g_res, "disk", "结果盘可用空间", free_gb, "GB", ok=3 * need_gb, warn=need_gb, higher=True, fix="procedure",
          display=f"{free_gb:.0f} GB（一轮建图约需 {need_gb:.0f} GB：数据 {size_gb:.1f} GB）" if free_gb is not None else "—",
          plain="结果盘剩余空间够不够跑一轮。剩 <1 轮所需=跑到一半磁盘写满直接失败。",
          note="每轮建图保存地图、扫描存档与评估；磁盘写满会让轮次中途失败。", basis="truck05 一轮 10–12 GB；truck29 1–2 GB。")
    thr = counters["bytes"] / max(elapsed_pass, 1e-6) / 1e6
    bitrate = summary["size_bytes"] / max(summary["duration_s"], 1e-6) / 1e6
    R.add(g_res, "read_throughput", "读取带宽 vs 码率", thr / max(bitrate, 1e-6), "×", ok=2.0, warn=1.2, higher=True, fix="procedure",
          display=f"{thr:.0f} MB/s（码率 {bitrate:.0f} MB/s，{thr / max(bitrate, 1e-6):.1f}×）",
          plain="这次评估读数据的速度是数据码率的几倍。≥2× 实时回放稳；<1.2× 得用 --rate 0.7 降倍速，否则丢帧。",
          note="回放需要以 ≥1× 码率读取并同时运行建图；低于 1.2× 时请用 --rate 0.7。", basis="本次评估读取实测。")

    verdict, counts, alerts, fixable = R.finish()
    outlook = precision_outlook(verdict, alerts, fixable)
    doc = {"schema": SCHEMA, "generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "elapsed_s": round(time.time() - t_wall, 1),
           "dataset": {"path": summary["path"], "name": summary["name"], "kind": summary["kind"], "duration_s": summary["duration_s"],
                       "size_gb": summary["size_gb"], "segments": summary["segment_count"], "start_iso": summary["start_iso"],
                       "start_ns": summary["start_ns"], "end_ns": summary["end_ns"], "lidar_topics": [short[t] for t in lidar_topics],
                       "has_camera": summary["has_camera"]},
           "window": list(a.window) if a.window else None, "calibration": calib["path"] if calib else None,
           "verdict": verdict, "counts": counts, "alerts": alerts, "fixable": fixable, "outlook": outlook,
           "groups": R.groups, "reference": ref_stats, "camera_samples": cam_eval,
           "messages": counters["messages"], "bytes": counters["bytes"], "workers": len(specs), "pass_s": round(elapsed_pass, 1)}
    return doc


def precision_outlook(verdict, alerts, fixable):
    bad_alerts = [x for x in alerts if x["status"] == "bad"]
    mismatch = next((x for x in fixable if x["id"] == "vehicle_match"), None)
    if mismatch:
        rest = f" 另有 {len(alerts)} 项数据源问题标定无法补救，见下方列表。" if alerts else ""
        return {"level": "bad", "title": "所选标定不属于这台车",
                "text": f"{mismatch['display']}。外参、地面倾角/高度差等检查是按别的车的标定算出来的，数值没有参考意义；"
                        "先在标定页为这台车导入出厂标定并运行 LiDAR 标定流水线，再重新评估和建图。" + rest}
    if verdict == "ok":
        return {"level": "ok", "title": "数据质量达到 truck29 参照水平",
                "text": "所有指标在正常区间：标定流水线与建图自检可按常规流程运行，预期融合精度与 truck29 同级（xy 最大误差 ≤ 8.5 cm）。"}
    if not alerts:
        return {"level": "warn", "title": "发现的问题均可由标定/建图流程自动补救",
                "text": f"{len(fixable)} 项关注/异常都属于标定阶段或建图自检能吸收的常数误差：运行 LiDAR 标定流水线（必要时相机流水线）后再建图即可。"}
    if bad_alerts:
        return {"level": "bad", "title": f"{len(bad_alerts)} 项异常无法通过标定补救",
                "text": "这些缺陷来自数据源本身（参考系统、传感器同步、录制流程或场景），标定自动微调不能恢复精度；建议按“处理建议”修复硬件或重新采集后再标定。"
                        + (f" 另有 {len(alerts) - len(bad_alerts)} 项关注级问题会降低精度上限。" if len(alerts) > len(bad_alerts) else "")}
    return {"level": "warn", "title": f"{len(alerts)} 项关注级问题无法通过标定补救",
            "text": "标定与建图可以正常运行，但这些问题决定了精度的下限（参考噪声、同步抖动、场景纹理等）；相同算法与标定下的误差预期高于 truck29。"}


# ----------------------------------------------------------------------------- console output
def print_report(doc):
    ic = {"ok": "✓", "warn": "!", "bad": "✗", "info": "·", "na": "-"}
    d = doc["dataset"]
    print(f"\n数据评估  {d['name']}  [{d['kind']}]  {d['duration_s']} s  {d['size_gb']} GB  {d['segments']} 段  {d['start_iso']}")
    print(f"标定文件: {doc['calibration'] or '（无）'}   评估耗时 {doc['elapsed_s']} s")
    for g in doc["groups"]:
        print(f"\n[{ic[g['status']]}] {g['label']}")
        for m in g["metrics"]:
            tag = f"  ← {m['fix_label']}" if m["fix_label"] else ""
            alert = "  ⚠ 标定无法补救" if m["alert"] else ""
            print(f"   {ic[m['status']]} {m['label']:<34s} {m['display']}{tag}{alert}")
    o = doc["outlook"]
    print(f"\n结论 [{doc['verdict'].upper()}] {o['title']}\n{o['text']}")
    if doc["alerts"]:
        print("\n警示（标定无法补救）:")
        for x in doc["alerts"]:
            print(f"   {ic[x['status']]} {x['group']} · {x['label']}: {x['display']} — {x['fix_label']}")
    print()


def main(argv=None):
    ap = argparse.ArgumentParser(description="MY-LIVO 数据评估：一次流式读取，输出带阈值与补救判断的数据质量报告")
    ap.add_argument("dataset")
    ap.add_argument("--calibration", help="calibration.json（激活版本）；用于后轴坐标系下的地面/结构检查")
    ap.add_argument("--json", type=pathlib.Path, help="写出完整 JSON 报告")
    ap.add_argument("--window", type=float, nargs=2, metavar=("START", "DURATION"), help="只评估从数据起点 START s 开始的 DURATION s")
    ap.add_argument("--sample-every", type=float, default=6.0, help="点云采样间隔 (s)")
    ap.add_argument("--camera-samples", type=int, default=8)
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="不打印表格（只写 JSON）")
    ap.add_argument("--jobs", type=int, default=0, help="并行读取的进程数（0 = 自动：min(8, CPU/2)，1 = 串行）")
    a = ap.parse_args(argv)
    doc = run(a)
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        tmp = a.json.with_suffix(".tmp")
        tmp.write_text(json.dumps(doc, ensure_ascii=False, indent=1, default=float) + "\n")
        tmp.replace(a.json)
        print(f"[assess] report written: {a.json}", flush=True)
    if not a.quiet:
        print_report(doc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
