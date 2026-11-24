#!/usr/bin/env python3
"""Collect every number the round evaluation produced into one machine-readable file.

Usage (container): /usr/bin/python3 scripts/analysis/round_metrics.py <round_dir | round_name> [--print]
Writes <round>/evaluation/metrics.json (schema 1) from the artefacts that already exist:
  round.json / run.log, Log/fusion/<stamp>/{parameters.txt,trajectory_*.txt,frames.csv,diagnostics.csv},
  Log/extrinsic_precheck.json, launch.log, playback.log,
  evaluation/{trajectory.log,summary.txt,pole_*.log}.
Nothing is recomputed from point clouds; missing artefacts simply leave their section null.
The "headline" block is what the console lists; the other blocks feed the run detail panel.
"""
import csv, datetime, json, math, pathlib, re, sys

REPO = pathlib.Path("/home/project/MY-LIVO")
NUM = r"([-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?)"
M = NUM + r"\s?m"     # number immediately followed by the unit, e.g. 0.9337m
MM = NUM + r"\s?mm"


def fnum(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def rx(pattern, text, flags=re.M):
    m = re.search(pattern, text, flags)
    return m.groups() if m else None


def stats(values):
    vals = sorted(v for v in values if v is not None and math.isfinite(v))
    if not vals:
        return None
    def q(p):
        k = (len(vals) - 1) * p
        lo, hi = math.floor(k), math.ceil(k)
        return vals[lo] if lo == hi else vals[lo] + (vals[hi] - vals[lo]) * (k - lo)
    return {"median": q(0.5), "p95": q(0.95), "max": vals[-1], "n": len(vals)}


# ----------------------------------------------------------------------------- round identity
def read_round(rd):
    info = {"round": rd.name, "mode": None, "dataset": None, "calibration": None, "options": None,
            "calibration_source": None, "calibration_id": None, "calibration_id_source": None,
            "camera": None, "camera_stream_topic": None, "camera_image_topic": None,
            "start": None, "end": None, "status": None, "commit": None, "fusion_dir": None, "map_chunks": None}
    rj = rd / "round.json"
    if rj.is_file():
        try:
            j = json.loads(rj.read_text())
            for k in info:
                if k in j:
                    info[k] = j[k]
        except Exception:
            pass
    rl = rd / "run.log"
    if rl.is_file():
        t = rl.read_text(errors="replace")
        for k in ("mode", "dataset", "calibration", "calibration_source", "commit"):
            if not info.get(k):
                g = rx(r"^%s=(.+)$" % k, t)
                if g:
                    info[k] = g[0].strip()
    # The round-local snapshot is the calibration actually consumed by mapping.
    # Its parent version cannot be inferred from results/<round>/calibration.json.
    info["calibration_id"] = None
    info["calibration_id_source"] = None
    camera_from_snapshot = False
    candidates = [(rd / "calibration.json", "snapshot.identity.id")]
    for key in ("calibration", "calibration_source"):
        if info.get(key):
            path = pathlib.Path(info[key])
            if not path.is_absolute() and not path.is_file():
                path = rd / path
            candidates.append((path, f"{key}.identity.id"))
    for path, origin in candidates:
        try:
            cal = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        identity = cal.get("identity") or {}
        if not info["calibration_id"] and identity.get("id"):
            info["calibration_id"] = identity["id"]
            info["calibration_id_source"] = origin
        if info.get("mode") != "lio":
            camera = cal.get("camera") or {}
            if not camera_from_snapshot and origin != "calibration_source.identity.id":
                info["camera"] = cal.get("camera_selection", "midrange")
                info["camera_stream_topic"] = camera.get("stream_topic") or info["camera_stream_topic"]
                info["camera_image_topic"] = camera.get("topic") or info["camera_image_topic"]
                camera_from_snapshot = True
            else:
                info["camera"] = info["camera"] or cal.get("camera_selection", "midrange")
                info["camera_stream_topic"] = info["camera_stream_topic"] or camera.get("stream_topic")
                info["camera_image_topic"] = info["camera_image_topic"] or camera.get("topic")
    if not info["calibration_id"]:
        for key in ("calibration_source", "calibration"):
            parts = pathlib.Path(info[key]).parts if info.get(key) else ()
            if "carlibration" in parts:
                i = parts.index("carlibration")
                if len(parts) >= i + 5:
                    info["calibration_id"] = "/".join(parts[i + 1:i + 4])
                    info["calibration_id_source"] = f"{key}.version_path"
                    break
    return info


def fusion_dir(rd, info):
    fd = info.get("fusion_dir")
    if fd and pathlib.Path(fd).is_dir():
        return pathlib.Path(fd)
    cands = sorted((rd / "Log" / "fusion").glob("*/"), key=lambda p: p.stat().st_mtime) if (rd / "Log" / "fusion").is_dir() else []
    return cands[-1] if cands else None


# ----------------------------------------------------------------------------- parameters.txt
def read_parameters(fd):
    p = {}
    if fd and (fd / "parameters.txt").is_file():
        for line in (fd / "parameters.txt").read_text(errors="replace").splitlines():
            if "=" in line and not line.startswith("model="):
                k, v = line.split("=", 1)
                p.setdefault(k.strip(), v.strip())  # effective values are written first
    keep = ("camera.frame_divider", "lio.max_iterations", "vio.max_iterations", "vio.img_point_cov", "vio.patch_size",
            "vio.exposure_estimate_en", "time_offset.img_time_offset", "time_offset.lidar_time_offset",
            "rtk.time_offset_sec", "rtk.attitude_time_offset_sec", "rtk.attitude_enabled", "rtk.frontend_attitude_aiding",
            "multi_lidar.precheck.enabled", "multi_lidar.precheck.apply", "vio.ekf_update_en",
            "effective.lidar_time_offset_sec", "effective.img_time_offset_sec", "effective.rtk_reference_time_shift_sec",
            "effective.camera_mode", "effective.camera_frame_divider", "effective.camera_selection",
            "effective.camera_projection_model", "effective.camera_input_rectification", "effective.camera_image_topic",
            "calibration.id")
    out = {k: p[k] for k in keep if k in p}
    out["time_offset_sec"] = fnum(p.get("rtk.time_offset_sec"))
    out["attitude_time_offset_sec"] = fnum(p.get("rtk.attitude_time_offset_sec"))
    return out


# ----------------------------------------------------------------------------- trajectories / frames / diagnostics
def path_length(path):
    rows = [l.split() for l in path.read_text().splitlines() if l and not l.startswith("#")]
    t = [float(r[0]) for r in rows]
    xyz = [(float(r[1]), float(r[2]), float(r[3])) for r in rows]
    dist = 0.0
    for a, b in zip(xyz, xyz[1:]):
        dist += math.dist(a, b)
    return dist, t, xyz


def read_run_shape(fd):
    out = {"frames": None, "duration_s": None, "distance_m": None, "native_path_m": None, "optimized_path_m": None,
           "path_length_ratio": None, "frame_dt_median_s": None, "points_per_frame_median": None}
    if not fd:
        return out
    try:
        dist, t, xyz = path_length(fd / "trajectory_lio.txt")
        out["frames"] = len(t)
        out["duration_s"] = round(t[-1] - t[0], 2) if len(t) > 1 else 0.0
        out["native_path_m"] = round(dist, 2)
        try:
            dopt, _, _ = path_length(fd / "trajectory_optimized.txt")
            out["optimized_path_m"] = round(dopt, 2)
            out["distance_m"] = round(dopt, 1)
            # native odometry scale: >1 = native path longer than the RTK-fused path (scale error of the front end)
            out["path_length_ratio"] = round(dist / dopt, 4) if dopt > 1.0 else None
        except Exception:
            out["distance_m"] = round(dist, 1)
    except Exception:
        pass
    try:
        lines = [l for l in (fd / "frames.csv").read_text(errors="replace").splitlines() if l and not l.startswith("#")]
        hdr = lines[0].split(",")
        it, ic = hdr.index("stamp"), hdr.index("count")
        st, cnt = [], []
        for l in lines[1:]:
            c = l.split(",")
            st.append(float(c[it])); cnt.append(float(c[ic]))
        dts = [b - a for a, b in zip(st, st[1:])]
        s = stats(dts); c = stats(cnt)
        out["frame_dt_median_s"] = round(s["median"], 4) if s else None
        out["points_per_frame_median"] = int(c["median"]) if c else None
    except Exception:
        pass
    return out


def read_diagnostics(fd):
    out = None
    if not fd or not (fd / "diagnostics.csv").is_file():
        return out
    try:
        with open(fd / "diagnostics.csv") as fh:
            rows = list(csv.DictReader(l for l in fh if not l.startswith("#")))
        if not rows:
            return None
        def col(n):
            return [fnum(r.get(n)) for r in rows]
        last = rows[-1]
        out = {"rows": len(rows),
               "processing_ms": stats(col("processing_ms")),
               "publish_lag_max_s": max((v for v in col("publish_lag_sec") if v is not None), default=None),
               "speed_max_mps": max((v for v in col("frontend_speed_mps") if v is not None), default=None),
               "keyframes": fnum(last.get("keyframes")), "position_factors": fnum(last.get("position_factors")),
               "planes": fnum(last.get("planes")), "factors": fnum(last.get("factors")),
               }
        for k in ("processing_ms",):
            if out[k]:
                out[k] = {kk: round(vv, 2) for kk, vv in out[k].items()}
    except Exception:
        out = None
    return out


# ----------------------------------------------------------------------------- trajectory.log (evaluate_wuhu_lio.py)
def parse_series(block, label):
    g = rx(r"^all time-valid %s: n=(\d+), RMSE=%s, median=%s, P95=%s, max=%s, >10cm=%s%%, >15cm=%s%%, >20cm=%s%%, final=%s"
           % (label, M, M, M, M, NUM, NUM, NUM, M), block)
    if not g:
        return None
    return {"n": int(g[0]), "rmse_m": float(g[1]), "median_m": float(g[2]), "p95_m": float(g[3]), "max_m": float(g[4]),
            "gt10cm_pct": float(g[5]), "gt15cm_pct": float(g[6]), "gt20cm_pct": float(g[7]), "final_m": float(g[8])}


def parse_trajectory_log(path):
    if not path.is_file():
        return None, None, None
    txt = path.read_text(errors="replace")
    traj = {}
    for m in re.finditer(r"^(\S*trajectory_(lio|optimized)\.txt): poses=(\d+), time-valid=(\d+)", txt, re.M):
        kind = "native" if m.group(2) == "lio" else "optimized"
        block = txt[m.start():]
        eol = block.find("\n") + 1
        nxt = re.search(r"^\S*trajectory_(?:lio|optimized)\.txt: poses=", block[eol:], re.M)
        if nxt:
            block = block[:eol + nxt.start()]
        entry = {"poses": int(m.group(3)), "time_valid": int(m.group(4)),
                 "xy": parse_series(block, "XY"), "z": parse_series(block, "Z"), "d3": parse_series(block, "3D")}
        g = rx(r"Route-weighted 3D RTK consistency: assessed=%s, unassessed reference-polyline distance=%s, <=10cm=%s%%, >20cm=%s%%, accepted-sample max=%s"
               % (M, M, NUM, NUM, M), block)
        if g:
            entry["route"] = {"assessed_m": float(g[0]), "unassessed_m": float(g[1]), "le10cm_pct": float(g[2]),
                              "gt20cm_pct": float(g[3]), "accepted_max_m": float(g[4])}
        g = rx(r"Assessed-route position budget \(NOT map acceptance\): (PASS|FAIL)", block)
        entry["budget"] = g[0] if g else None
        imu = {}
        for lab, key in (("XY", "xy"), ("Z", "z"), ("3D", "d3")):
            g = rx(r"accepted INS-derived IMU consistency %s: n=(\d+), RMSE=%s, median=%s, P95=%s, max=%s.*?final=%s"
                   % (lab, M, M, M, M, M), block)
            if g:
                imu[key] = {"n": int(g[0]), "rmse_m": float(g[1]), "median_m": float(g[2]), "p95_m": float(g[3]), "max_m": float(g[4]), "final_m": float(g[5])}
        g = rx(r"IMU-reference route consistency: assessed=%s, <=10cm=%s%%, >20cm=%s%%" % (M, NUM, NUM), block)
        if g:
            imu["route"] = {"assessed_m": float(g[0]), "le10cm_pct": float(g[1]), "gt20cm_pct": float(g[2])}
        entry["imu"] = imu or None
        g = rx(r"INS attitude difference \(NOT attitude truth\): P95=%s\s?deg" % NUM, block)
        entry["attitude_diff_p95_deg"] = float(g[0]) if g else None
        g = rx(r"Backend ms: median=%s, P95=%s, max=%s" % (NUM, NUM, NUM), block)
        if g:
            entry["backend_ms"] = {"median": float(g[0]), "p95": float(g[1]), "max": float(g[2])}
        g = rx(r"Final online counters: keys=(\d+), position factors=(\d+)", block)
        if g:
            entry["online_counters"] = {"keys": int(g[0]), "position_factors": int(g[1])}
        traj[kind] = entry

    startup = {}
    g = rx(r"first %s reference-polyline metres" % NUM, txt)
    startup["length_m"] = float(g[0]) if g else None
    for key, lab in (("native", "native"), ("fused", "fused")):
        g = rx(r"^%s 3D error median/P95/max=\[%s, %s, %s\]" % (lab, NUM, NUM, NUM), txt)
        if g:
            startup[key] = {"median_m": float(g[0]), "p95_m": float(g[1]), "max_m": float(g[2])}
        g = rx(r"^%s IMU 3D difference median/P95/max=\[%s, %s, %s\]" % (lab, NUM, NUM, NUM), txt)
        if g:
            startup[key + "_imu"] = {"median_m": float(g[0]), "p95_m": float(g[1]), "max_m": float(g[2])}
    sides = []
    for m in re.finditer(r"^first (\d+) m opposite route sides=(\d+)/(\d+) samples; native/fused signed cross-track median=%s/%s m" % (NUM, NUM), txt, re.M):
        sides.append({"length_m": int(m.group(1)), "opposite": int(m.group(2)), "samples": int(m.group(3)),
                      "native_cross_track_m": float(m.group(4)), "fused_cross_track_m": float(m.group(5))})
    startup["sides"] = sides or None
    g = rx(r"distance to synchronized RTK--native XY segment P50/P95/max=\[%s, %s, %s\]" % (NUM, NUM, NUM), txt)
    if g:
        startup["segment_distance_m"] = {"p50": float(g[0]), "p95": float(g[1]), "max": float(g[2])}
    g = rx(r"correction tilt min/median/max=\[%s, %s, %s\] deg" % (NUM, NUM, NUM), txt)
    if g:
        startup["correction_tilt_deg"] = {"min": float(g[0]), "median": float(g[1]), "max": float(g[2])}
    g = rx(r"first fused rear error=\[%s, %s, %s\] m" % (NUM, NUM, NUM), txt)
    if g:
        startup["first_fused_rear_error_m"] = [float(x) for x in g]

    mp = {"thickness": [], "plane_offset": [], "jaccard": []}
    g = rx(r"Map SAME-point temporal surface comparison: sampled=(\d+), planar cells=(\d+), frame/point strides=(\d+)/(\d+)", txt)
    if g:
        mp["sampled_points"] = int(g[0]); mp["planar_cells"] = int(g[1]); mp["strides"] = [int(g[2]), int(g[3])]
    for m in re.finditer(r"^(\d+-\d+)m (all|ground-like|wall-like): cells=(\d+), plane RMS thickness median %s -> %s, increase P95=%s, within proposed regression bound=%s%%"
                         % (NUM, MM, MM, NUM), txt, re.M):
        mp["thickness"].append({"band_m": m.group(1), "class": m.group(2), "cells": int(m.group(3)), "median_before_mm": float(m.group(4)),
                                "median_after_mm": float(m.group(5)), "increase_p95_mm": float(m.group(6)), "within_bound_pct": float(m.group(7))})
    g = rx(r"Non-keyframe cross-scan surfaces: pairs=(\d+), matches=(\d+)", txt)
    if g:
        mp["cross_scan_pairs"] = int(g[0]); mp["cross_scan_matches"] = int(g[1])
    for m in re.finditer(r"^(\d+-\d+)m (ground-like|wall-like): n=(\d+), raw->optimized \|plane offset\| median=%s->%s, P95=%s->%s, optimized >5cm=%s%%"
                         % (NUM, MM, NUM, MM, NUM), txt, re.M):
        mp["plane_offset"].append({"band_m": m.group(1), "class": m.group(2), "n": int(m.group(3)), "median_raw_mm": float(m.group(4)),
                                   "median_opt_mm": float(m.group(5)), "p95_raw_mm": float(m.group(6)), "p95_opt_mm": float(m.group(7)),
                                   "opt_gt5cm_pct": float(m.group(8))})
    for m in re.finditer(r"^(\d+)cm occupancy Jaccard: pairs=(\d+), median raw->optimized %s%%->%s%%" % (NUM, NUM), txt, re.M):
        mp["jaccard"].append({"cell_cm": int(m.group(1)), "pairs": int(m.group(2)), "raw_pct": float(m.group(3)), "opt_pct": float(m.group(4))})
    for k in ("thickness", "plane_offset", "jaccard"):
        if not mp[k]:
            mp[k] = None
    return traj or None, startup if startup.get("native") else None, mp if (mp.get("thickness") or mp.get("plane_offset")) else None


# ----------------------------------------------------------------------------- summary.txt (round_summary.py)
def parse_summary(path):
    if not path.is_file():
        return None
    txt = path.read_text(errors="replace")
    if "Traceback" in txt and "[native] rear-axle" not in txt:
        return {"error": txt.strip().splitlines()[-1][:200]}
    out = {}
    g = rx(r"^reference=(.+)$", txt)
    out["reference"] = g[0].strip() if g else None
    g = rx(r"^frames=(\d+) duration=%s\s?s" % NUM, txt)
    if g:
        out["frames"] = int(g[0]); out["duration_s"] = float(g[1])
    for kind in ("native", "optimized"):
        e = {}
        g = rx(r"^\[%s\] rear-axle 3D vs RTK: median %s P95 %s max %s \| route <=10cm %s%% >20cm %s%%\s+-> budget (PASS|FAIL)" % (kind, NUM, NUM, NUM, NUM, NUM), txt)
        if g:
            e["rear"] = {"median_m": float(g[0]), "p95_m": float(g[1]), "max_m": float(g[2]), "route_le10cm_pct": float(g[3]),
                         "route_gt20cm_pct": float(g[4]), "budget": g[5]}
        g = rx(r"^\[%s\] IMU 3D vs INS-derived IMU: median %s P95 %s max %s \| route <=10cm %s%%" % (kind, NUM, NUM, NUM, NUM), txt)
        if g:
            e["imu"] = {"median_m": float(g[0]), "p95_m": float(g[1]), "max_m": float(g[2]), "route_le10cm_pct": float(g[3])}
        g = rx(r"^\[%s\] attitude - INS \(deg\): x\(pitch\) median %s P95 %s max %s \| y\(roll\) median %s P95 %s max %s \| z\(yaw\) median %s P95 %s max %s"
               % ((kind,) + (NUM,) * 9), txt)
        if g:
            v = [float(x) for x in g]
            e["attitude_deg"] = {"pitch": {"median": v[0], "p95": v[1], "max": v[2]}, "roll": {"median": v[3], "p95": v[4], "max": v[5]},
                                 "yaw": {"median": v[6], "p95": v[7], "max": v[8]}}
        out[kind] = e or None
    windows = []
    for m in re.finditer(r"^\s*(\d+)-\s*(\d+)\s+%s\s+%s\s+%s \|\s+%s\s+%s \|\|\s+%s\s+%s\s+%s \|\s+%s\s*$" % ((NUM,) * 9), txt, re.M):
        v = [float(x) for x in m.groups()]
        windows.append({"t0": int(v[0]), "t1": int(v[1]),
                        "opt": {"dpitch_deg": v[2], "droll_deg": v[3], "dyaw_deg": v[4], "rear_m": v[5], "imu_m": v[6]},
                        "native": {"dpitch_deg": v[7], "droll_deg": v[8], "dyaw_deg": v[9], "rear_m": v[10]}})
    out["windows"] = windows or None
    st = []
    for m in re.finditer(r"^startup first (\d+) m: rear-axle opposite side (\d+)/(\d+); native/opt signed cross-track median %s/%s m; \|opt\| median %s" % (NUM, NUM, NUM), txt, re.M):
        st.append({"length_m": int(m.group(1)), "opposite": int(m.group(2)), "samples": int(m.group(3)),
                   "native_cross_track_m": float(m.group(4)), "opt_cross_track_m": float(m.group(5)), "opt_abs_median_m": float(m.group(6))})
    out["startup"] = st or None
    g = rx(r"^correction knots=(\d+); per-knot rotation step deg: median %s P95 %s max %s" % (NUM, NUM, NUM), txt)
    if g:
        out["corrections"] = {"knots": int(g[0]), "rotation_step_deg": {"median": float(g[1]), "p95": float(g[2]), "max": float(g[3])}}
    g = rx(r"^native \|v\| vs INS speed by 10s: (.+)$", txt)
    if g:
        pairs = []
        for tok in g[0].split():
            a, b = tok.split("/")
            pairs.append([fnum(a), fnum(b)])
        out["speed_by_10s"] = pairs
    g = rx(r"^# installation_misalignment_deg\(body x,y,z\)\s+%s\s+%s\s+%s" % (NUM, NUM, NUM), txt)
    if g:
        out["installation_misalignment_deg"] = [float(x) for x in g]
    g = rx(r"^INS attitude factor residuals \(deg\): x median %s P95 %s max %s \| y median %s P95 %s max %s \| z median %s P95 %s max %s; n=(\d+)" % ((NUM,) * 9), txt)
    if g:
        v = [float(x) for x in g[:9]]
        out["attitude_factor_residuals_deg"] = {"x": {"median": v[0], "p95": v[1], "max": v[2]}, "y": {"median": v[3], "p95": v[4], "max": v[5]},
                                                "z": {"median": v[6], "p95": v[7], "max": v[8]}, "n": int(g[9])}
    pole = {"chunks": []}
    for m in re.finditer(r"^\s*map_(\d+)\s+(\d+)\s+\(\s*%s,\s*%s\)\s+%s\s+%s\s+tilt %s deg" % ((NUM,) * 5), txt, re.M):
        pole["chunks"].append({"chunk": int(m.group(1)), "shaft_points": int(m.group(2)), "axis_x": float(m.group(3)), "axis_y": float(m.group(4)),
                               "top_z_p99": float(m.group(5)), "z_max": float(m.group(6)), "tilt_deg": float(m.group(7))})
    g = rx(r"pole ring-centre separation over all epoch pairs \(cm, radius %s cm fixed\): median %s P95 %s max %s" % (NUM, NUM, NUM, NUM), txt)
    if g:
        pole["pair_separation_cm"] = {"median": float(g[1]), "p95": float(g[2]), "max": float(g[3]), "metric": "ring_fit"}
    g = rx(r"\[view-biased diagnostic\] pole axis XY separation over all epoch pairs \(cm\): median %s P95 %s max %s" % (NUM, NUM, NUM), txt)
    if g:
        pole["view_biased_pair_separation_cm"] = {"median": float(g[0]), "p95": float(g[1]), "max": float(g[2])}
    pairwise = []
    for m in re.finditer(r"^\s*(\d+)v(\d+)\s+%s\s+(%s|nan)\s+(%s|nan)\s*$" % (NUM, NUM, NUM), txt, re.M):
        pairwise.append({"a": int(m.group(1)), "b": int(m.group(2)), "axis_sep_cm": float(m.group(3)),
                         "equal_height_sep_cm": fnum(m.group(4)), "top_z_diff_cm": fnum(m.group(6))})
    if pairwise:
        pole["pairwise"] = pairwise
    if len(pole["chunks"]) >= 2 and "pair_separation_cm" not in pole:
        seps = []
        ch = pole["chunks"]
        for i in range(len(ch)):
            for j in range(i + 1, len(ch)):
                seps.append(math.hypot(ch[i]["axis_x"] - ch[j]["axis_x"], ch[i]["axis_y"] - ch[j]["axis_y"]) * 100.0)
        s = stats(seps)
        if s:
            pole["view_biased_pair_separation_cm"] = {"median": round(s["median"], 2), "p95": round(s["p95"], 2), "max": round(s["max"], 2), "pairs": s["n"]}
    out["pole"] = pole if pole["chunks"] else None
    if "Lamp pole scan skipped" in txt:
        out["pole_skipped"] = True
    return out


# ----------------------------------------------------------------------------- pole pair logs
def parse_pole_pairs(eval_dir):
    out = {}
    for f in sorted(eval_dir.glob("pole_*_*.log")):
        txt = f.read_text(errors="replace")
        key = f.stem[len("pole_"):]
        e = {}
        g = rx(r"map_(\d+) to map_(\d+) observed-surface NN median/P95=\[%s, %s\] m" % (NUM, NUM), txt)
        if g:
            e["nn_ab_m"] = {"median": float(g[2]), "p95": float(g[3])}
        ms = list(re.finditer(r"map_(\d+) to map_(\d+) observed-surface NN median/P95=\[%s, %s\] m" % (NUM, NUM), txt))
        if len(ms) >= 2:
            e["nn_ba_m"] = {"median": float(ms[1].group(3)), "p95": float(ms[1].group(4))}
        g = rx(r"equal-height XY surface separation median/P95/max=\[%s, %s, %s\] m" % (NUM, NUM, NUM), txt)
        if g:
            e["equal_height_xy_m"] = {"median": float(g[0]), "p95": float(g[1]), "max": float(g[2])}
        for m in re.finditer(r"map_(\d+): selected points=(\d+), shaft points=(\d+), shaft Z span=%s m" % NUM, txt):
            e.setdefault("chunks", []).append({"chunk": int(m.group(1)), "selected": int(m.group(2)), "shaft": int(m.group(3)), "span_m": float(m.group(4))})
        if e:
            out[key] = e
    return out or None


# ----------------------------------------------------------------------------- launch.log / playback.log / precheck
def parse_launch(rd):
    p = rd / "launch.log"
    if not p.is_file():
        return None
    out = {}
    try:
        with open(p, "rb") as fh:
            fh.seek(0, 2); size = fh.tell(); fh.seek(max(0, size - 400_000))
            tail = fh.read().decode("utf-8", "replace")
        head = p.open("rb").read(200_000).decode("utf-8", "replace")
    except Exception:
        return None
    g = rx(r"INS attitude factors: (\d+); estimated installation misalignment \[%s\s+%s\s+%s\] deg" % (NUM, NUM, NUM), tail)
    if g:
        out["attitude_factors"] = int(g[0]); out["installation_misalignment_deg"] = [float(x) for x in g[1:]]
    g = rx(r"Final keyframe max RTK residual %s m" % NUM, tail)
    if g:
        out["final_keyframe_max_rtk_residual_m"] = float(g[0])
    g = rx(r"RTK finalized (\d+) frames, (\d+) keys, (\d+) position factors", tail)
    if g:
        out["finalized_frames"] = int(g[0]); out["keys"] = int(g[1]); out["position_factors"] = int(g[2])
    out["vio_no_point_msgs"] = len(re.findall(r"\[ VIO \] No point", head)) if "[ VIO ]" in head else None
    out["warnings"] = len(re.findall(r"\[WARN\]", tail))
    out["errors"] = len(re.findall(r"\[ERROR\]", tail))
    return out or None


def parse_playback(rd):
    p = rd / "playback.log"
    if not p.is_file():
        return None
    txt = p.read_text(errors="replace")
    g = rx(r"\[player\] summary (\{.*\})", txt)
    if not g:
        return None
    try:
        j = json.loads(g[0])
    except Exception:
        return None
    return {k: j.get(k) for k in ("status", "messages", "elapsed_s", "max_lag_ms", "dropped_duplicate", "dropped_late", "per_topic") if k in j}


def parse_precheck(rd):
    p = rd / "Log" / "extrinsic_precheck.json"
    if not p.is_file():
        return None
    try:
        j = json.loads(p.read_text())
    except Exception:
        return None
    src = []
    for s in j.get("sources") or []:
        src.append({k: s.get(k) for k in ("topic", "reference", "solved", "within_gates", "significant", "applied",
                                          "median_before_cm", "median_after_cm", "rot_x_deg", "rot_y_deg", "dz_cm", "note")})
    return {"status": j.get("status"), "reference_topic": j.get("reference_topic"), "sources": src}


# ----------------------------------------------------------------------------- headline
def headline(traj, summ, startup, mp, prm, run=None):
    h = {"native": None, "optimized": None}
    for kind in ("native", "optimized"):
        e = {}
        s = (summ or {}).get(kind) or {}
        rear = s.get("rear")
        t = (traj or {}).get(kind) or {}
        d3, xy, z = t.get("d3"), t.get("xy"), t.get("z")
        if rear:
            e.update({"median_m": rear["median_m"], "p95_m": rear["p95_m"], "max_m": rear["max_m"],
                      "route_le10cm_pct": rear["route_le10cm_pct"], "route_gt20cm_pct": rear["route_gt20cm_pct"], "budget": rear["budget"]})
        elif d3:
            e.update({"median_m": d3["median_m"], "p95_m": d3["p95_m"], "max_m": d3["max_m"], "budget": t.get("budget")})
            if t.get("route"):
                e["route_le10cm_pct"] = t["route"]["le10cm_pct"]; e["route_gt20cm_pct"] = t["route"]["gt20cm_pct"]
        if d3:
            e["final_m"] = d3["final_m"]; e["rmse_m"] = d3["rmse_m"]
        if xy:
            e["final_xy_m"] = xy["final_m"]; e["median_xy_m"] = xy["median_m"]
        if z:
            e["final_z_m"] = z["final_m"]; e["median_z_m"] = z["median_m"]
        att = s.get("attitude_deg")
        if att:
            e["yaw_p95_deg"] = att["yaw"]["p95"]; e["pitch_p95_deg"] = att["pitch"]["p95"]; e["roll_p95_deg"] = att["roll"]["p95"]
        elif t.get("attitude_diff_p95_deg") is not None:
            e["attitude_p95_deg"] = t["attitude_diff_p95_deg"]
        win = (summ or {}).get("windows")
        if win:
            last = win[-1][ "opt" if kind == "optimized" else "native"]
            e["yaw_drift_end_deg"] = last["dyaw_deg"]; e["pitch_drift_end_deg"] = last["dpitch_deg"]
        if kind == "native" and run and run.get("path_length_ratio"):
            e["path_length_ratio"] = run["path_length_ratio"]
        h[kind] = e or None
    if startup:
        h["startup"] = {"length_m": startup.get("length_m"),
                        "native": startup.get("native"), "fused": startup.get("fused"),
                        "opposite_10m": next(({"opposite": s["opposite"], "samples": s["samples"]} for s in (startup.get("sides") or []) if s["length_m"] == 10), None)}
    if mp and mp.get("thickness"):
        g = next((t for t in mp["thickness"] if t["band_m"] == "5-20" and t["class"] == "ground-like"), None)
        if g:
            h["ground_5_20"] = {"median_before_mm": g["median_before_mm"], "median_after_mm": g["median_after_mm"],
                                "within_bound_pct": g["within_bound_pct"], "cells": g["cells"]}
        po = next((t for t in (mp.get("plane_offset") or []) if t["band_m"] == "5-20" and t["class"] == "ground-like"), None)
        if po:
            h["plane_offset_5_20"] = {"median_raw_mm": po["median_raw_mm"], "median_opt_mm": po["median_opt_mm"], "opt_gt5cm_pct": po["opt_gt5cm_pct"]}
    pole = (summ or {}).get("pole")
    if pole and pole.get("pair_separation_cm"):
        h["pole_pair_separation_cm"] = pole["pair_separation_cm"]
        h["pole_chunks"] = len(pole["chunks"])
    if summ and summ.get("installation_misalignment_deg"):
        h["installation_misalignment_deg"] = summ["installation_misalignment_deg"]
    if prm:
        h["time_offset_sec"] = prm.get("time_offset_sec")
    return h


def build(rd):
    info = read_round(rd)
    fd = fusion_dir(rd, info)
    ev = rd / "evaluation"
    prm = read_parameters(fd)
    if not info["calibration_id"] and prm.get("calibration.id"):
        info["calibration_id"] = prm["calibration.id"]
        info["calibration_id_source"] = "parameters.calibration.id"
    if info.get("mode") != "lio":
        info["camera"] = info["camera"] or prm.get("effective.camera_selection")
        info["camera_image_topic"] = info["camera_image_topic"] or prm.get("effective.camera_image_topic")
    traj, startup, mp = parse_trajectory_log(ev / "trajectory.log")
    summ = parse_summary(ev / "summary.txt")
    launch = parse_launch(rd)
    if summ and not summ.get("installation_misalignment_deg") and launch and launch.get("installation_misalignment_deg"):
        summ["installation_misalignment_deg"] = launch["installation_misalignment_deg"]
    doc = {
        "schema": 1,
        "generated": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "round": info["round"], "mode": info.get("mode"), "dataset": info.get("dataset"),
        "calibration": info.get("calibration"), "calibration_id": info.get("calibration_id"),
        "calibration_source": info.get("calibration_source"), "calibration_id_source": info.get("calibration_id_source"),
        "camera": info.get("camera"), "camera_stream_topic": info.get("camera_stream_topic"),
        "camera_image_topic": info.get("camera_image_topic"),
        "commit": info.get("commit"), "start": info.get("start"), "end": info.get("end"),
        "fusion_dir": str(fd) if fd else None,
        "parameters": prm,
        "run": read_run_shape(fd),
        "diagnostics": read_diagnostics(fd),
        "trajectory": traj,
        "summary": summ,
        "startup": startup,
        "map": mp,
        "pole_pairs": parse_pole_pairs(ev) if ev.is_dir() else None,
        "backend": launch,
        "playback": parse_playback(rd),
        "precheck": parse_precheck(rd),
    }
    doc["map_chunks"] = info.get("map_chunks") if info.get("map_chunks") is not None else (len(list(fd.glob("map_*.pcd"))) if fd else None)
    doc["headline"] = headline(traj, summ if isinstance(summ, dict) and "error" not in summ else None, startup, mp, prm, doc["run"])
    doc["evaluated"] = bool(traj)
    doc["complete"] = bool(traj and isinstance(summ, dict) and summ.get("native"))
    return doc


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    rd = pathlib.Path(args[0])
    if not rd.is_dir():
        rd = REPO / "results" / args[0]
    if not rd.is_dir():
        sys.exit(f"round directory not found: {args[0]}")
    doc = build(rd)
    out = rd / "evaluation" / "metrics.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, ensure_ascii=False, indent=1, allow_nan=False))
    h = doc["headline"]
    n, o = h.get("native") or {}, h.get("optimized") or {}
    print(f"metrics -> {out}")
    print(f"native: median {n.get('median_m')} max {n.get('max_m')} final {n.get('final_m')} (xy {n.get('final_xy_m')} z {n.get('final_z_m')}) yaw_end {n.get('yaw_drift_end_deg')}")
    print(f"optimized: median {o.get('median_m')} p95 {o.get('p95_m')} max {o.get('max_m')} budget {o.get('budget')}")
    if "--print" in sys.argv:
        print(json.dumps(doc, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
