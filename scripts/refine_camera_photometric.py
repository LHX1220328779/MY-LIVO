#!/usr/bin/env python3
"""Camera rotation + time offset (with an intrinsics check) from the multi-view photo-consistency of a LIVO round.

Why: the RGB map colours every LiDAR return from a camera view that sees it (look-back colouring); a return
is seen by many views 2-60 m apart at different image positions, and with the right camera model they all show
the same surface texture. The camera pipeline sets the camera yaw from pole columns, the pitch from the native
odometry scale (closed loop) and the time offset from optical flow vs gyro; none of these looks at where a
LiDAR return actually lands in the images. On truck29raw v042_camloop (20261013) the views agreed best with the
camera turned by pitch +0.29 / yaw -0.25 / roll +0.06 deg and its images 16 ms earlier: local-contrast
cross-view variance -29 % on held-out cells, the same correction measured on a separate stretch of the route
(-31 %), and a LIVO round with it kept the native odometry scale (path-length ratio 1.0001 -> 0.9999). The focal
lengths and radial distortion added <= 1 % on held-out cells (fx/fy within 0.6 % of the supplier model, k1 ~ 0):
the intrinsics are confirmed, not changed.

Method: kept returns of the round's frames (map_rows.bin, all in-cap returns when the round wrote no map chunks),
native world points (frames.csv poses, the mapper's own projection). Views = the round's images every 2 m of
camera travel (newest first, up to 10 per return, 40 px inside the border, not behind the nearest return of the
view's own scan), as local-contrast-normalised grey images. Cost = mean over returns of the variance of the
sampled LCN value across their views (capped). The camera model is perturbed by a rotation in the camera frame
(about the camera centre) and a time shift (camera pose taken dt later along the native trajectory) and fitted
with a coarse pitch/yaw grid + Powell on two interleaved halves of the returns (4 m checkerboard cells): each
half's fit is scored on the other half. The intrinsics check refits with fx/fy scales and k1 added.

Gate: the rotation/time correction is applied when both halves improve their held-out cost by >= 5 % and the
two halves agree (each rotation axis within 0.08 deg, dt within 6 ms); the applied values are the fit on all
returns. Intrinsics are applied only when they add >= 2 % on both held-out halves and the halves agree (focal
scales within 0.5 %, k1 within 0.02). Otherwise no file is written (the parent stays, exit 0, "kept").

Usage (container):
  /usr/bin/python3 scripts/refine_camera_photometric.py --calibration <parent calibration.json> --bag <dataset> \\
      --run <LIVO fusion run dir> --output <calibration.json> --report <txt> [--json <metrics.json>]
"""
import argparse
import json
import pathlib
import subprocess
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tools" / "calib"))
from livo import map_export as me  # noqa: E402
from livo_calib import dump_camera_stream  # noqa: E402
from scipy.optimize import minimize  # noqa: E402
from scipy.spatial.transform import Rotation as Rot, Slerp  # noqa: E402

W_IMG, H_IMG = 1920, 1080
CELL_PX = 8                      # occlusion grid
DELTA_M, SPAN_M, KMAX, MARGIN_PX = 2.0, 80.0, 10, 40
BLIND_M, TOL_ABS, TOL_REL = 2.0, 0.3, 0.04
ROWS_PER_FRAME, MAX_VIEWS, POINTS = 2500, 260, 50000
MIN_GAIN, AGREE_ROT_DEG, AGREE_DT_S = 0.05, 0.08, 0.006
INTR_MIN_GAIN, AGREE_FOCAL, AGREE_K1 = 0.02, 0.005, 0.02
NAMES = ["rx", "ry", "rz", "dt", "sfx", "sfy", "k1"]
SCALE = np.array([0.05, 0.05, 0.05, 0.005, 0.01, 0.01, 0.02])


def lcn(bgr):
    import cv2
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    g = cv2.GaussianBlur(g, (0, 0), 1.0)
    m = cv2.GaussianBlur(g, (0, 0), 6.0)
    d = g - m
    return (d / (np.sqrt(cv2.GaussianBlur(d * d, (0, 0), 6.0)) + 3.0)).astype(np.float16)


def collect(run, bag, calib, out_dir, window, log):
    """Views (LCN images, camera poses, velocities) and the (return, view) pairs seen by the look-back rule."""
    fx, fy, cx, cy = calib["camera"]["intrinsics"]
    T_rc = np.array(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    params = me.parameters(run)
    cap = float(params.get("rtk.save_map_max_range", 45.0))
    rgb_cap = float(params.get("rtk.save_rgb_max_range", cap))
    rgb_cap = cap if rgb_cap < 0 else rgb_cap
    cam_off = float(params.get("effective.img_time_offset_sec", 0.0))
    fr, lt, lr = me.read_frames(run)
    stamp = fr["stamp"]
    quat = np.column_stack([fr["qx"], fr["qy"], fr["qz"], fr["qw"]])
    raw_R = me.quat_to_rot(quat)
    raw_t = np.column_stack([fr["tx"], fr["ty"], fr["tz"]])
    slerp = Slerp(stamp, Rot.from_quat(quat))
    cnt = fr["count"].astype(np.int64)
    rows_path = run / "map_rows.bin"
    rows_mm = np.memmap(rows_path, dtype=np.uint8, mode="r") if rows_path.exists() else None
    rstart = 8 + np.concatenate([[0], np.cumsum((cnt + 7) // 8)[:-1]])
    gh, gw = H_IMG // CELL_PX + 1, W_IMG // CELL_PX + 1

    def cam_pose(t):
        t = min(max(t, stamp[0]), stamp[-1])
        R = slerp([t])[0].as_matrix()
        p = np.array([np.interp(t, stamp, raw_t[:, j]) for j in range(3)])
        Rn, tn = R @ lr, R @ lt + p
        return Rn @ T_rc[:3, :3], Rn @ T_rc[:3, 3] + tn

    def project(Rcw, tcw, X):
        C = X @ Rcw.T + tcw
        z = C[:, 2]
        zs = np.where(z > 0.1, z, np.nan)
        return fx * C[:, 0] / zs + cx, fy * C[:, 1] / zs + cy, z, np.linalg.norm(C, axis=1)

    def inside(u, v, m):
        return np.isfinite(u) & (u >= m) & (u < W_IMG - 1 - m) & (v >= m) & (v < H_IMG - 1 - m)

    t0 = stamp[0]
    sel = np.flatnonzero((stamp - t0 >= window[0]) & (stamp - t0 <= min(window[1], stamp[-1] - t0 - 1.0)))
    if len(sel) < 50:
        return None, f"only {len(sel)} frames in the window"
    img_stamps, stream = dump_camera_stream(bag, out_dir, t_from=stamp[sel[0]] - cam_off - 0.5,
                                            t_to=stamp[sel[-1]] - cam_off + 0.5)
    eff = img_stamps + cam_off
    want = {}
    for i in sel:
        k = int(np.argmin(np.abs(eff - stamp[i])))
        if abs(eff[k] - stamp[i]) < 0.004:
            want[k] = int(i)
    rng = np.random.default_rng(3)
    views, buf = [], []
    PTS, PX, PV, CELLS = [], [], [], []
    npts = 0
    proc = subprocess.Popen(["ffmpeg", "-nostdin", "-loglevel", "error", "-f", "hevc", "-i", str(stream),
                             "-pix_fmt", "bgr24", "-f", "rawvideo", "-"], stdout=subprocess.PIPE,
                            stdin=subprocess.DEVNULL, bufsize=10 ** 8)
    nbytes, k = W_IMG * H_IMG * 3, -1
    with open(run / "scans.bin", "rb") as scans:
        while len(views) < MAX_VIEWS:
            data = proc.stdout.read(nbytes)
            if len(data) < nbytes:
                break
            k += 1
            if k not in want:
                continue
            i = want[k]
            n = int(cnt[i])
            scans.seek(int(fr["offset_bytes"][i]))
            a = np.frombuffer(scans.read(n * 20), dtype="<f4").reshape(n, 5)
            bits = (np.unpackbits(rows_mm[rstart[i]:rstart[i] + (n + 7) // 8], bitorder="little")[:n].astype(bool)
                    if rows_mm is not None else np.ones(n, bool))
            p = a[:, :3].astype(np.float64)
            rv = np.linalg.norm(p, axis=1)
            m = np.isfinite(rv) & (rv <= cap)
            w = p[m] @ (raw_R[i] @ lr).T + (raw_R[i] @ lt + raw_t[i])
            bits, rv = bits[m], rv[m]
            R_wc, t_wc = cam_pose(stamp[i])
            Rcw = R_wc.T
            tcw = -Rcw @ t_wc
            u, v, z, _ = project(Rcw, tcw, w)
            okz = inside(u, v, 0) & (z > 0.1)
            grid = np.full(gh * gw, np.inf)
            np.minimum.at(grid, (v[okz] // CELL_PX).astype(np.int64) * gw + (u[okz] // CELL_PX).astype(np.int64), z[okz])
            Ra, ta = cam_pose(stamp[i] - 0.05)
            Rb, tb = cam_pose(stamp[i] + 0.05)
            cur = {"Rcw": Rcw, "tcw": tcw, "z": grid, "c": t_wc, "vel": (tb - ta) / 0.1,
                   "omega": Rot.from_matrix(Ra.T @ Rb).as_rotvec() / 0.1, "bgr": np.frombuffer(data, np.uint8).reshape(H_IMG, W_IMG, 3),
                   "id": None}
            if not buf or np.linalg.norm(cur["c"] - buf[-1]["c"]) >= DELTA_M:
                buf.append(cur)
            while buf and np.linalg.norm(buf[0]["c"] - cur["c"]) > SPAN_M:
                buf.pop(0)["bgr"] = None
            kept = np.flatnonzero(bits & (rv <= rgb_cap))
            if len(kept) > ROWS_PER_FRAME:
                kept = np.sort(rng.choice(kept, ROWS_PER_FRAME, replace=False))
            if not len(kept):
                continue
            X = w[kept]
            got = np.zeros(len(X), np.int64)
            for vw in buf[::-1]:
                todo = np.flatnonzero(got < KMAX)
                if not len(todo):
                    break
                uu, vv, zz, dd = project(vw["Rcw"], vw["tcw"], X[todo])
                ok = inside(uu, vv, MARGIN_PX) & (dd > BLIND_M) & (dd <= rgb_cap)
                kk = np.zeros(len(todo), np.int64)
                kk[ok] = (vv[ok] // CELL_PX).astype(np.int64) * gw + (uu[ok] // CELL_PX).astype(np.int64)
                ok &= zz <= vw["z"][kk] + np.maximum(TOL_ABS, TOL_REL * zz)
                if not ok.any():
                    continue
                if vw["id"] is None:
                    vw["id"] = len(views)
                    vw["img"] = lcn(vw["bgr"])
                    vw["bgr"] = None
                    views.append(vw)
                s_ = todo[ok]
                PX.append(npts + s_)
                PV.append(np.full(len(s_), vw["id"], np.int32))
                got[s_] += 1
            PTS.append(X)
            CELLS.append(((np.floor(X[:, 0] / 4) + np.floor(X[:, 1] / 4)).astype(np.int64) % 2).astype(np.int8))
            npts += len(X)
    proc.stdout.close()
    proc.kill()
    proc.wait()
    stream.unlink(missing_ok=True)
    if not views or not PX:
        return None, "no return is seen by two views"
    P, half = np.concatenate(PTS), np.concatenate(CELLS)
    px, pv = np.concatenate(PX), np.concatenate(PV)
    nview = np.bincount(px, minlength=len(P))
    cand = np.flatnonzero(nview >= 3)
    if len(cand) < 2000:
        return None, f"only {len(cand)} returns seen by >= 3 views"
    if len(cand) > POINTS:
        cand = np.sort(rng.choice(cand, POINTS, replace=False))
    remap = -np.ones(len(P), np.int64)
    remap[cand] = np.arange(len(cand))
    m = remap[px] >= 0
    px, pv = remap[px[m]], pv[m]
    P, half = P[cand], half[cand]
    RCW = np.stack([vw["Rcw"] for vw in views])
    TCW = np.stack([vw["tcw"] for vw in views])
    VEL = np.stack([vw["vel"] for vw in views])
    OMG = np.stack([vw["omega"] for vw in views])
    data = {"img": np.stack([vw["img"] for vw in views]), "px": px, "pv": pv, "half": half, "n": len(P),
            "C": np.einsum("nij,nj->ni", RCW[pv], P[px]) + TCW[pv], "V": np.einsum("nij,nj->ni", RCW[pv], VEL[pv]),
            "Wc": OMG[pv], "K": (fx, fy, cx, cy), "frames": len(want), "views": len(views)}
    log(f"collection: {len(want)} frames with an image, {len(views)} views, {len(P)} returns (>= 3 views), {len(px)} samples")
    return data, None


def make_score(data):
    img = data["img"].reshape(len(data["img"]), -1)
    px, pv, n = data["px"], data["pv"], data["n"]
    fx, fy, cx, cy = data["K"]

    def score(theta, part=None):
        rx, ry, rz, dt, sfx, sfy, k1 = theta
        C = data["C"] - data["V"] * dt
        C = C - np.cross(data["Wc"] * dt, C)
        if rx or ry or rz:
            C = C @ Rot.from_rotvec(np.radians([rx, ry, rz])).as_matrix().T
        x, y = C[:, 0] / C[:, 2], C[:, 1] / C[:, 2]
        dist = 1 + k1 * (x * x + y * y)
        u = np.clip(fx * (1 + sfx) * x * dist + cx, 0, W_IMG - 2.001)
        v = np.clip(fy * (1 + sfy) * y * dist + cy, 0, H_IMG - 2.001)
        a, b = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
        du, dv = (u - a).astype(np.float32), (v - b).astype(np.float32)
        off = b * W_IMG + a
        s = ((1 - du) * (1 - dv) * img[pv, off].astype(np.float32) + du * (1 - dv) * img[pv, off + 1].astype(np.float32)
             + (1 - du) * dv * img[pv, off + W_IMG].astype(np.float32) + du * dv * img[pv, off + W_IMG + 1].astype(np.float32))
        w = np.ones(len(px)) if part is None else (data["half"][px] == part).astype(np.float64)
        cnt = np.bincount(px, w, n)
        s1, s2 = np.bincount(px, w * s, n), np.bincount(px, w * s * s, n)
        good = cnt >= 3
        var = (s2[good] - s1[good] ** 2 / cnt[good]) / (cnt[good] - 1)
        return float(np.mean(np.minimum(var, 3.0)))
    return score


def fit(score, free, part, start=None):
    th0 = np.zeros(len(NAMES)) if start is None else start.copy()
    if start is None and "rx" in free and "ry" in free:   # coarse pitch / yaw grid: the cost is sharp (LCN at 1 px)
        best = (np.inf, None)
        for rx in np.arange(-0.4, 0.41, 0.1):
            for ry in np.arange(-0.4, 0.41, 0.1):
                th = np.zeros(len(NAMES))
                th[0], th[1] = rx, ry
                sc = score(th, part)
                if sc < best[0]:
                    best = (sc, th)
        th0 = best[1]
    fi = [NAMES.index(k) for k in free]
    base = th0[fi].copy()

    def f(z):
        th = th0.copy()
        th[fi] = base + z * SCALE[fi]
        return score(th, part)
    res = minimize(f, np.zeros(len(fi)), method="Powell", options={"xtol": 0.02, "ftol": 1e-5, "maxfev": 500})
    th = th0.copy()
    th[fi] = base + res.x * SCALE[fi]
    return th


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--json", help="metrics JSON (also written when the parent is kept)")
    ap.add_argument("--window", nargs=2, type=float, default=(3.0, 1e9), help="seconds after the run's first frame")
    a = ap.parse_args()
    t_start = time.time()
    calib = json.load(open(a.calibration))
    run = me.resolve_run(a.run)
    report = [f"calibration={a.calibration}", f"run={run}"]

    def log(line):
        report.append(line)
        print(line, flush=True)

    # the round must have been mapped with this camera model (the correction is relative to it)
    params = me.parameters(run)
    run_cal = params.get("multi_lidar.calibration_file")
    if run_cal and pathlib.Path(run_cal).exists():
        rc = json.load(open(run_cal))
        same = (np.allclose(rc["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"],
                            calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], atol=1e-9)
                and np.allclose(rc["camera"]["intrinsics"], calib["camera"]["intrinsics"])
                and abs(float((rc.get("time_offsets") or {}).get("camera", 0.0))
                        - float((calib.get("time_offsets") or {}).get("camera", 0.0))) < 1e-6)
        if not same:
            sys.exit(f"{run} was mapped with another camera model ({run_cal}); map a LIVO round with the parent first")
    out_dir = pathlib.Path(a.output).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    data, why = collect(run, pathlib.Path(a.bag), calib, out_dir, a.window, log)
    metrics = {"method": "multi-view photo-consistency of the round's returns (LCN cross-view variance)"}
    if data is None:
        log(f"not enough data: {why}; parent kept")
        finish(a, report, metrics, None)
        return
    score = make_score(data)
    zero = np.zeros(len(NAMES))
    base = [score(zero, p) for p in (0, 1)]
    halves = {}
    for p in (0, 1):
        th = fit(score, ["rx", "ry", "rz", "dt"], p)
        thi = fit(score, ["rx", "ry", "rz", "dt", "sfx", "sfy", "k1"], p, start=th)
        q = 1 - p
        halves[p] = {"theta": th, "theta_intr": thi,
                     "train_gain": 1 - score(th, p) / base[p], "test_gain": 1 - score(th, q) / base[q],
                     "test_gain_intr": 1 - score(thi, q) / base[q]}
        h = halves[p]
        log(f"half {p}: rotation+time rx {th[0]:+.3f} ry {th[1]:+.3f} rz {th[2]:+.3f} deg, dt {1000 * th[3]:+.1f} ms: "
            f"train {100 * h['train_gain']:+.1f} %, held-out {100 * h['test_gain']:+.1f} %; with intrinsics "
            f"(fx {100 * thi[4]:+.2f} %, fy {100 * thi[5]:+.2f} %, k1 {thi[6]:+.4f}) held-out {100 * h['test_gain_intr']:+.1f} %")
    d = np.abs(halves[0]["theta"] - halves[1]["theta"])
    gain_ok = min(halves[0]["test_gain"], halves[1]["test_gain"]) >= MIN_GAIN
    agree = bool(np.all(d[:3] <= AGREE_ROT_DEG) and d[3] <= AGREE_DT_S)
    extra = [halves[p]["test_gain_intr"] - halves[p]["test_gain"] for p in (0, 1)]
    di = np.abs(halves[0]["theta_intr"] - halves[1]["theta_intr"])
    intr_ok = bool(min(extra) >= INTR_MIN_GAIN and di[4] <= AGREE_FOCAL and di[5] <= AGREE_FOCAL and di[6] <= AGREE_K1)
    metrics.update({"frames": data["frames"], "views": data["views"], "returns": data["n"],
                    "cost_zero": [round(b, 5) for b in base],
                    "halves": [{"rot_deg": np.round(halves[p]["theta"][:3], 4).tolist(), "dt_s": round(float(halves[p]["theta"][3]), 5),
                                "train_gain": round(halves[p]["train_gain"], 4), "test_gain": round(halves[p]["test_gain"], 4),
                                "intrinsics": {"fx_scale": round(1 + float(halves[p]["theta_intr"][4]), 5),
                                               "fy_scale": round(1 + float(halves[p]["theta_intr"][5]), 5),
                                               "k1": round(float(halves[p]["theta_intr"][6]), 5),
                                               "test_gain": round(halves[p]["test_gain_intr"], 4)}} for p in (0, 1)],
                    "gate": {"min_test_gain": MIN_GAIN, "halves_agree": agree, "gain_ok": gain_ok,
                             "intrinsics_extra_gain": [round(e, 4) for e in extra], "intrinsics_ok": intr_ok}})
    if not (gain_ok and agree):
        log("rotation/time: " + ("held-out gain below 5 %" if not gain_ok else "halves disagree")
            + f" (rotation spread {np.round(d[:3], 3).tolist()} deg, dt spread {1000 * d[3]:.1f} ms); parent kept")
        log(f"intrinsics: {'not applied' if not intr_ok else 'would improve, but the rotation/time gate failed'}")
        finish(a, report, metrics, None)
        return
    free = ["rx", "ry", "rz", "dt"] + (["sfx", "sfy", "k1"] if intr_ok else [])
    th = fit(score, ["rx", "ry", "rz", "dt"], None)
    if intr_ok:
        th = fit(score, free, None, start=th)
    gain_all = 1 - score(th, None) / score(zero, None)
    T = np.array(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    Rd = Rot.from_rotvec(np.radians(th[:3])).as_matrix()
    T[:3, :3] = T[:3, :3] @ Rd.T                     # p_cam' = Rd p_cam about the camera centre
    out = json.loads(json.dumps(calib))
    out["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"] = [float(v) for v in T.reshape(-1)]
    offs = out.setdefault("time_offsets", {})
    old = float(offs.get("camera", 0.0))
    offs["camera"] = round(old + float(th[3]), 4)
    offs["camera_source"] = (f"camera-photo: multi-view photo-consistency, {old:+.4f} {1000 * th[3]:+.1f} ms "
                             f"(held-out gains {100 * halves[0]['test_gain']:.1f} / {100 * halves[1]['test_gain']:.1f} %)")
    if intr_ok:
        fx, fy, cx, cy = out["camera"]["intrinsics"]
        out["camera"]["intrinsics"] = [fx * (1 + th[4]), fy * (1 + th[5]), cx, cy]
        out["camera"]["distortion"] = [float(th[6]), 0.0, 0.0, 0.0, 0.0]
        out["camera"]["source"] = "camera-photo (multi-view photo-consistency)"
    out["calibration_type"] = str(calib.get("calibration_type", "")) + "+camera_photometric"
    applied = {"rot_delta_deg_cam_xyz": np.round(th[:3], 4).tolist(), "camera_time_offset_s": offs["camera"],
               "dt_s": round(float(th[3]), 5), "intrinsics": intr_ok, "cost_gain_all": round(gain_all, 4)}
    if intr_ok:
        applied.update({"fx_scale": round(1 + float(th[4]), 5), "fy_scale": round(1 + float(th[5]), 5), "k1": round(float(th[6]), 5)})
    out.setdefault("refinement", {})["camera_photometric"] = {**metrics, "applied": applied}
    metrics["applied"] = applied
    log(f"applied: camera rotation (camera frame) rx {th[0]:+.3f} ry {th[1]:+.3f} rz {th[2]:+.3f} deg, time offset "
        f"{old:+.4f} -> {offs['camera']:+.4f} s; intrinsics {'refined' if intr_ok else 'confirmed (unchanged)'}; "
        f"cost {100 * gain_all:+.1f} % on all returns ({time.time() - t_start:.0f} s)")
    finish(a, report, metrics, out)


def finish(a, report, metrics, out):
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    if out is not None:
        json.dump(out, open(a.output, "w"), indent=1)
        report.append(f"written {a.output}")
    else:
        report.append("result: parent kept (no calibration written)")
    pathlib.Path(a.report).write_text("\n".join(report) + "\n")
    if a.json:
        json.dump(metrics, open(a.json, "w"), indent=1, ensure_ascii=False)
    print(report[-1], flush=True)


if __name__ == "__main__":
    main()
