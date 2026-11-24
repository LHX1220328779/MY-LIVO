#!/usr/bin/env python3
"""Camera rotation + time offset (with an intrinsics check) and the camera flat field from the multi-view
photo-consistency of a LIVO round.

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
half's fit is scored on the other half. Three models fit rotation/time, rotation/time plus fx/fy,
and rotation/time plus fx/fy and a residual k1. Samples use raw encoded images and the complete
parent Brown distortion; an invalid projection gives its return the maximum cost instead of
sampling the clipped image border.

Gate: each model must itself improve both held-out costs by >= 5 % and agree between halves
(each rotation axis within 0.08 deg, dt within 6 ms, fitted focal scales within 0.5 %, fitted k1
within 0.02). A richer model must add >= 2 % on both held-out halves. A failed simpler model
does not reject an independently consistent richer model. Repeated --validation-run inputs
score the fixed candidate on separate recording windows without refitting; each half must
not worsen, and missing observations cannot count as a pass.

Flat field (camera.flat_field, used by the LIVO RGB colouring): the raw grey value of the same return in its views
= albedo of the return + exposure of the view + log flat field of the rectified image position (poly8 in x = (u - cx) / (W/2),
y = (v - cy) / (H/2), 0 at the principal point), alternating least squares on log grey. truck29raw: image edges
15-25 % darker, bottom row ~10 %; each 2 m band of look-back views otherwise leaves this as a ripple on the ground.
Written when both halves reduce their held-out cross-view variance by >= 5 % and their gain maps agree within 3 %
(fit on all returns, y range = the sampled one). No significant rotation/time, intrinsics or flat field: no file is
written (the parent stays, exit 0, "kept").

Usage (container):
  /usr/bin/python3 scripts/refine_camera_photometric.py --calibration <parent calibration.json> --bag <dataset> \\
      --run <LIVO fusion run dir> --output <calibration.json> --report <txt> [--json <metrics.json>] \\
      [--validation-run <independent completed round> ...]
"""
import argparse
import gc
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
from livo.camera import camera_transform_key, select_camera, selected_camera, sync_camera_profile  # noqa: E402
from livo_calib import dump_camera_stream  # noqa: E402
from scipy.optimize import minimize  # noqa: E402
from scipy.spatial.transform import Rotation as Rot, Slerp  # noqa: E402

W_IMG, H_IMG = 1920, 1080
CELL_PX = 8                      # occlusion grid
DELTA_M, SPAN_M, KMAX, MARGIN_PX = 2.0, 80.0, 10, 40
BLIND_M, TOL_ABS, TOL_REL = 2.0, 0.3, 0.04
ROWS_PER_FRAME, MAX_VIEWS, POINTS = 2500, 260, 50000
MIN_GAIN, AGREE_ROT_DEG, AGREE_DT_S = 0.05, 0.08, 0.006
FLAT_MIN_GAIN, AGREE_FLAT = 0.05, 0.03
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


def brown_pixels(C, K, D):
    """Raw-image Brown projection; D follows OpenCV [k1,k2,p1,p2,k3]."""
    fx, fy, cx, cy = K
    k1, k2, p1, p2, k3 = D
    z = np.where(C[:, 2] > 0.1, C[:, 2], np.nan)
    x, y = C[:, 0] / z, C[:, 1] / z
    r2 = x * x + y * y
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    return (fx * (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)) + cx,
            fy * (y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y) + cy)


def sample_positions(u, v):
    valid = np.isfinite(u) & np.isfinite(v) & (u >= 0) & (u < W_IMG - 1) & (v >= 0) & (v < H_IMG - 1)
    # Safe indexing only. The caller must mask or penalize invalid samples;
    # a candidate cannot gain by sampling a clipped image border.
    return np.where(valid, u, 0.0), np.where(valid, v, 0.0), valid


def collect(run, bag, calib, out_dir, window, log):
    """Views (LCN images, camera poses, velocities) and the (return, view) pairs seen by the look-back rule."""
    camera = calib["camera"]
    if (camera.get("width"), camera.get("height")) != (W_IMG, H_IMG):
        raise ValueError("camera-photo currently requires calibrated 1920x1080 images")
    fx, fy, cx, cy = calib["camera"]["intrinsics"]
    distortion = np.asarray(camera.get("distortion", [0] * 5), float)
    T_rc = np.array(calib["transforms"][camera_transform_key(calib)]["transform_matrix"], float).reshape(4, 4)
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
        u, v = brown_pixels(C, (fx, fy, cx, cy), distortion)
        return u, v, z, np.linalg.norm(C, axis=1)

    def inside(u, v, m):
        return np.isfinite(u) & np.isfinite(v) & (u >= m) & (u < W_IMG - 1 - m) & (v >= m) & (v < H_IMG - 1 - m)

    t0 = stamp[0]
    sel = np.flatnonzero((stamp - t0 >= window[0]) & (stamp - t0 <= min(window[1], stamp[-1] - t0 - 1.0)))
    if len(sel) < 50:
        return None, f"only {len(sel)} frames in the window"
    img_stamps, stream = dump_camera_stream(bag, out_dir, t_from=stamp[sel[0]] - cam_off - 0.5,
                                            t_to=stamp[sel[-1]] - cam_off + 0.5, camera=camera)
    if not len(img_stamps):
        return None, "selected camera stream has no decodable frames in the run window"
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
    nbytes, k, matched_frames = W_IMG * H_IMG * 3, -1, 0
    with open(run / "scans.bin", "rb") as scans:
        while len(views) < MAX_VIEWS:
            data = proc.stdout.read(nbytes)
            if len(data) < nbytes:
                break
            k += 1
            if k not in want:
                continue
            matched_frames += 1
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
                   "stamp": float(stamp[i]), "id": None}
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
                ok = inside(uu, vv, MARGIN_PX) & (zz > 0.1) & (dd > BLIND_M) & (dd <= rgb_cap)
                kk = np.zeros(len(todo), np.int64)
                kk[ok] = (vv[ok] // CELL_PX).astype(np.int64) * gw + (uu[ok] // CELL_PX).astype(np.int64)
                ok &= zz <= vw["z"][kk] + np.maximum(TOL_ABS, TOL_REL * zz)
                if not ok.any():
                    continue
                if vw["id"] is None:
                    import cv2
                    vw["id"] = len(views)
                    vw["img"] = lcn(vw["bgr"])
                    vw["gray"] = cv2.cvtColor(vw["bgr"], cv2.COLOR_BGR2GRAY)
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
    data = {"img": np.stack([vw["img"] for vw in views]), "gray": np.stack([vw["gray"] for vw in views]),
            "px": px, "pv": pv, "half": half, "n": len(P),
            "C": np.einsum("nij,nj->ni", RCW[pv], P[px]) + TCW[pv], "V": np.einsum("nij,nj->ni", RCW[pv], VEL[pv]),
            "Wc": OMG[pv], "K": (fx, fy, cx, cy), "D": distortion,
            "frames": matched_frames, "views": len(views),
            "view_stamps_s": [vw["stamp"] for vw in views],
            "stamp_range_s": [min(vw["stamp"] for vw in views), max(vw["stamp"] for vw in views)]}
    log(f"collection: {matched_frames} frames with an image, {len(views)} views, {len(P)} returns (>= 3 views), {len(px)} samples")
    return data, None


def make_score(data):
    img = data["img"].reshape(len(data["img"]), -1)
    px, pv, n = data["px"], data["pv"], data["n"]
    def score(theta, part=None):
        if not np.isfinite(theta).all() or min(1 + theta[4], 1 + theta[5]) <= 0:
            return 3.0
        u, v, valid = sample_positions(*project_samples(data, theta))
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
        invalid = np.bincount(px, w * (~valid), n)[good] > 0
        return float(np.mean(np.where(invalid, 3.0, np.minimum(var, 3.0))))
    return score


def fit(score, free, part, start=None, diagnostics=None):
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
    rounds = []
    x0 = np.zeros(len(fi))
    directions = None
    for attempt in range(4):
        options = {"xtol": 0.02, "ftol": 1e-5, "maxfev": 500}
        if directions is not None:
            options["direc"] = directions
        res = minimize(f, x0, method="Powell", options=options)
        rounds.append({"success": bool(res.success), "status": int(res.status), "message": str(res.message),
                       "nfev": int(res.nfev), "nit": int(res.nit), "cost": float(res.fun)})
        # Continue only an exhausted evaluation budget, retaining Powell's
        # learned directions. Other failures are not hidden by a blind retry.
        if res.success or int(res.status) != 1:
            break
        x0 = res.x
        directions = res.direc
    th = th0.copy()
    th[fi] = base + res.x * SCALE[fi]
    if diagnostics is not None:
        diagnostics.update({"success": bool(res.success), "status": int(res.status),
                            "message": str(res.message), "nfev": sum(r["nfev"] for r in rounds),
                            "continuations": len(rounds) - 1, "rounds": rounds})
    return th


def project_samples(data, theta, rectified=False):
    """Image position of every (return, view) sample under the camera model perturbed by theta (as in score)."""
    fx, fy, cx, cy = data["K"]
    rx, ry, rz, dt, sfx, sfy, k1 = theta
    C = data["C"] - data["V"] * dt
    C = C - np.cross(data["Wc"] * dt, C)
    if rx or ry or rz:
        C = C @ Rot.from_rotvec(np.radians([rx, ry, rz])).as_matrix().T
    D = np.array(data.get("D", np.zeros(5)), float).copy()
    D[0] += k1
    return brown_pixels(C, (fx * (1 + sfx), fy * (1 + sfy), cx, cy), np.zeros(5) if rectified else D)


def flat_terms(x, y):
    """poly8 log flat field terms (no constant: 0 at the principal point)."""
    x2 = x * x
    return np.column_stack([x2, x2 * x2, y, y * y, y ** 3, y ** 4, x2 * y, x2 * y * y])


def flat_field(data, theta, log):
    """Flat field from the raw grey values of the samples (see the module doc). Returns (calibration entry or None,
    metrics)."""
    fx, fy, cx, cy = data["K"]
    u, v, valid = sample_positions(*project_samples(data, theta))
    g = data["gray"].reshape(len(data["gray"]), -1)
    a, b = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
    du, dv = u - a, v - b
    off = b * W_IMG + a
    pv, px, n = data["pv"], data["px"], data["n"]
    grey = ((1 - du) * (1 - dv) * g[pv, off] + du * (1 - dv) * g[pv, off + 1]
            + (1 - du) * dv * g[pv, off + W_IMG] + du * dv * g[pv, off + W_IMG + 1])
    ok = valid & (grey > 8) & (grey < 247)             # neither black nor saturated
    L = np.log(grey + 1.0)
    ur, vr, rectified_valid = sample_positions(*project_samples(data, theta, rectified=True))
    ok &= rectified_valid
    x, y = (ur - cx) / (W_IMG / 2), (vr - cy) / (H_IMG / 2)
    F = flat_terms(x, y)
    nv = int(pv.max()) + 1
    half = data["half"][px]
    if any(np.sum(ok & (half == p)) < 2000 for p in (0, 1)):
        return None, {"applied": False, "reason": "not enough valid unsaturated flat-field samples"}

    def fit_part(mask, with_flat):
        c, e = np.zeros(F.shape[1]), np.zeros(nv)
        for _ in range(10):
            r = L - (F @ c if with_flat else 0) - e[pv]
            alb = np.bincount(px[mask], r[mask], n) / np.maximum(np.bincount(px[mask], minlength=n), 1)
            if with_flat:
                c = np.linalg.lstsq(F[mask], (L - alb[px] - e[pv])[mask], rcond=None)[0]
            r = L - alb[px] - (F @ c if with_flat else 0)
            e = np.bincount(pv[mask], r[mask], nv) / np.maximum(np.bincount(pv[mask], minlength=nv), 1)
            e -= np.median(e)
        return c, e

    def cost(mask, c, e):
        r = L - F @ c - e[pv]
        cnt = np.bincount(px[mask], minlength=n)
        s1, s2 = np.bincount(px[mask], r[mask], n), np.bincount(px[mask], r[mask] ** 2, n)
        good = cnt >= 3
        return float(np.mean(np.minimum((s2[good] - s1[good] ** 2 / cnt[good]) / (cnt[good] - 1), 1.0)))

    y_range = [float(np.percentile(y[ok], 1)), float(np.percentile(y[ok], 99))]
    gx, gy = np.meshgrid(np.linspace(-1, 1, 33), np.linspace(y_range[0], y_range[1], 19))
    G = flat_terms(gx.ravel(), gy.ravel())
    halves, maps = [], []
    for p in (0, 1):
        m, q = ok & (half == p), ok & (half == 1 - p)
        c, e = fit_part(m, True)
        c0, e0 = fit_part(m, False)
        gain = 1 - cost(q, c, e) / cost(q, np.zeros_like(c), e0)
        halves.append({"coefficients": np.round(c, 5).tolist(), "test_gain": round(gain, 4)})
        maps.append(np.exp(-(G @ c)))
        log(f"flat field half {p}: held-out cross-view variance {100 * gain:+.1f} %, darkest position "
            f"{100 * (np.exp((G @ c).min()) - 1):+.0f} % of the principal point")
    spread = float(np.max(np.abs(maps[0] - maps[1])))
    metrics = {"halves": halves, "gain_map_spread": round(spread, 4), "y_range": [round(r, 3) for r in y_range],
               "min_test_gain": FLAT_MIN_GAIN, "max_spread": AGREE_FLAT, "samples": int(ok.sum())}
    if min(h["test_gain"] for h in halves) < FLAT_MIN_GAIN or spread > AGREE_FLAT:
        log(f"flat field: not applied (held-out gains {[h['test_gain'] for h in halves]}, half spread {spread:.3f})")
        metrics["applied"] = False
        return None, metrics
    c, e = fit_part(ok, True)
    flat = {"model": "poly8", "coefficients": [round(float(v), 6) for v in c], "y_range": [round(r, 4) for r in y_range],
            "source": f"camera-photo: multi-view grey consistency (held-out {100 * halves[0]['test_gain']:.1f} / "
                      f"{100 * halves[1]['test_gain']:.1f} %)"}
    edge = np.exp(G @ c)
    log(f"flat field: applied (held-out {100 * halves[0]['test_gain']:.1f} / {100 * halves[1]['test_gain']:.1f} %, "
        f"half spread {100 * spread:.1f} %); image positions {100 * (edge.min() - 1):+.0f} .. "
        f"{100 * (edge.max() - 1):+.0f} % of the principal point")
    metrics["applied"] = True
    metrics["coefficients"] = flat["coefficients"]
    return flat, metrics


def require_run_camera(run, calib):
    """A correction is relative to the camera actually used for this archive."""
    params = me.parameters(run)
    if params.get("effective.camera_selection", "midrange") != selected_camera(calib):
        raise ValueError(f"{run} was mapped with another camera; map a LIVO round with the selected camera first")
    run_cal = params.get("multi_lidar.calibration_file")
    if selected_camera(calib) == "fisheye":
        if params.get("effective.camera_mode") != "livo" or not run_cal or not pathlib.Path(run_cal).is_file():
            raise ValueError("fisheye camera-photo requires a real LIVO archive and its calibration snapshot")
    if run_cal and pathlib.Path(run_cal).exists():
        rc = select_camera(json.load(open(run_cal)))
        same = (selected_camera(rc) == selected_camera(calib)
                and (rc["camera"]["width"], rc["camera"]["height"]) == (calib["camera"]["width"], calib["camera"]["height"])
                and rc["camera"]["topic"] == calib["camera"]["topic"]
                and np.allclose(rc["camera"].get("distortion", [0] * 5), calib["camera"].get("distortion", [0] * 5))
                and np.allclose(rc["transforms"][camera_transform_key(rc)]["transform_matrix"],
                            calib["transforms"][camera_transform_key(calib)]["transform_matrix"], atol=1e-9)
                and np.allclose(rc["camera"]["intrinsics"], calib["camera"]["intrinsics"])
                and abs(float((rc.get("time_offsets") or {}).get("camera", 0.0))
                        - float((calib.get("time_offsets") or {}).get("camera", 0.0))) < 1e-6)
        if not same:
            raise ValueError(f"{run} was mapped with another camera model ({run_cal}); map a LIVO round with the parent first")


def recording_coverage(bag, data, camera):
    from livo.dataset import Dataset
    ds = Dataset.open(bag)
    lo, hi = data["stamp_range_s"]
    used = [s for s in ds.segments if s.end_ns >= lo * 1e9 and s.start_ns <= hi * 1e9]
    segments = [s.name for s in used]
    topic = camera["stream_topic"].lstrip("/")
    view_stamps = np.asarray(data.get("view_stamps_s", [lo, hi])) * 1e9
    camera_segments = [s.name for s in used if topic in s.topics
                       and np.any((view_stamps >= s.start_ns) & (view_stamps <= s.end_ns))]
    return {"bag": str(bag), "relative_window_s": [lo - ds.start_ns * 1e-9, hi - ds.start_ns * 1e-9],
            "segments": segments, "segment_count": len(segments),
            "camera_topic": topic, "camera_segments": camera_segments, "camera_segment_count": len(camera_segments),
            "frames": data["frames"], "views": data["views"], "returns": data["n"]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--report", required=True)
    ap.add_argument("--json", help="metrics JSON (also written when the parent is kept)")
    ap.add_argument("--window", nargs=2, type=float, default=(3.0, 1e9), help="seconds after the run's first frame")
    ap.add_argument("--validation-run", action="append", default=[],
                    help="independent completed LIVO round with the same parent model; repeat for multiple windows/recordings")
    a = ap.parse_args()
    t_start = time.time()
    calib = select_camera(json.load(open(a.calibration)))
    run = me.resolve_run(a.run)
    report = [f"calibration={a.calibration}", f"run={run}"]

    def log(line):
        report.append(line)
        print(line, flush=True)

    require_run_camera(run, calib)
    out_dir = pathlib.Path(a.output).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    data, why = collect(run, pathlib.Path(a.bag), calib, out_dir, a.window, log)
    metrics = {"method": "multi-view photo-consistency of the round's returns (LCN cross-view variance)",
               "camera": selected_camera(calib)}
    if data is None:
        log(f"not enough data: {why}; parent kept")
        finish(a, report, metrics, None)
        return
    coverage = recording_coverage(pathlib.Path(a.bag), data, calib["camera"])
    metrics["coverage"] = coverage
    if selected_camera(calib) == "fisheye" and coverage["camera_segment_count"] < 2:
        metrics["coverage"]["passed"] = False
        log("fisheye training requires at least two recorded camera segments; parent kept")
        finish(a, report, metrics, None)
        return
    score = make_score(data)
    zero = np.zeros(len(NAMES))
    base = [score(zero, p) for p in (0, 1)]
    model_free = {"rotation_time": ["rx", "ry", "rz", "dt"],
                  "focal": ["rx", "ry", "rz", "dt", "sfx", "sfy"],
                  "focal_k1": list(NAMES)}
    models = {name: [] for name in model_free}
    for p in (0, 1):
        start = None
        for name, free in model_free.items():
            optimizer = {}
            th = fit(score, free, p, start=start, diagnostics=optimizer)
            start = th
            h = {"theta": th, "train_gain": 1 - score(th, p) / base[p],
                 "test_gain": 1 - score(th, 1 - p) / base[1 - p], "optimizer": optimizer}
            models[name].append(h)
            log(f"half {p} {name}: rotation {np.round(th[:3], 4).tolist()} deg, dt {1000 * th[3]:+.2f} ms, "
                f"fx/fy {100 * th[4]:+.3f}/{100 * th[5]:+.3f} %, delta k1 {th[6]:+.5f}; "
                f"train {100 * h['train_gain']:+.2f} %, held-out {100 * h['test_gain']:+.2f} %; "
                f"optimizer success={optimizer['success']}, nfev={optimizer['nfev']}, continuations={optimizer['continuations']}")
    summaries = {}
    selected = None
    previous = None
    for name, hs in models.items():
        spread = np.abs(hs[0]["theta"] - hs[1]["theta"])
        gain_ok = min(h["test_gain"] for h in hs) >= MIN_GAIN
        agree = bool(np.all(spread[:3] <= AGREE_ROT_DEG) and spread[3] <= AGREE_DT_S)
        if name != "rotation_time":
            agree = agree and bool(np.all(spread[4:6] <= AGREE_FOCAL))
        if name == "focal_k1":
            agree = agree and bool(spread[6] <= AGREE_K1)
        extra = ([hs[p]["test_gain"] - models[previous][p]["test_gain"] for p in (0, 1)]
                 if previous else [0.0, 0.0])
        extra_ok = previous is None or min(extra) >= INTR_MIN_GAIN
        converged = all(h["optimizer"]["success"] for h in hs)
        accepted = bool(gain_ok and agree and extra_ok and converged)
        summaries[name] = {"halves": [{"rot_deg": np.round(h["theta"][:3], 5).tolist(),
                                      "dt_s": round(float(h["theta"][3]), 6),
                                      "fx_scale": round(1 + float(h["theta"][4]), 6),
                                      "fy_scale": round(1 + float(h["theta"][5]), 6),
                                      "delta_k1": round(float(h["theta"][6]), 6),
                                      "train_gain": round(h["train_gain"], 5),
                                      "test_gain": round(h["test_gain"], 5), "optimizer": h["optimizer"]} for h in hs],
                           "spread": dict(zip(NAMES, np.round(spread, 6).tolist())),
                           "gate": {"gain_ok": gain_ok, "halves_agree": agree, "extra_gain_ok": extra_ok,
                                    "extra_gain": [round(e, 5) for e in extra], "comparison_model": previous,
                                    "converged": converged, "accepted": accepted}}
        log(f"{name} gate: accepted={accepted}, gain={gain_ok}, agreement={agree}, extra_gain={extra_ok}; "
            f"rotation spread {np.round(spread[:3], 4).tolist()} deg, dt {1000 * spread[3]:.2f} ms")
        if accepted:
            selected = name
        previous = name
    metrics.update({"frames": data["frames"], "views": data["views"], "returns": data["n"],
                    "training_run": str(run), "training_bag": str(a.bag),
                    "training_window_s": list(a.window), "training_stamp_range_s": data["stamp_range_s"],
                    "coverage": coverage,
                    "cost_zero": [round(b, 6) for b in base], "models": summaries,
                    "selected_model": selected,
                    "gate": {"min_test_gain": MIN_GAIN, "min_extra_gain": INTR_MIN_GAIN,
                             "max_rotation_spread_deg": AGREE_ROT_DEG, "max_time_spread_s": AGREE_DT_S,
                             "max_focal_spread": AGREE_FOCAL, "max_k1_spread": AGREE_K1}})
    if selected is None:
        log("all joint camera models failed their own held-out/repeatability gate; parent kept")
        flat, metrics["flat_field"] = flat_field(data, zero, log)
        if flat is None or a.validation_run or selected_camera(calib) == "fisheye":
            if a.validation_run or selected_camera(calib) == "fisheye":
                metrics["validation"] = {"passed": False, "reason": "no accepted geometric candidate to validate"}
            finish(a, report, metrics, None)
            return
        out = json.loads(json.dumps(calib))
        out["camera"]["flat_field"] = flat
        out["calibration_type"] = str(calib.get("calibration_type", "")) + "+camera_photometric"
        applied = {"rot_delta_deg_cam_xyz": [0.0, 0.0, 0.0],
                   "camera_time_offset_s": float((calib.get("time_offsets") or {}).get("camera", 0.0)),
                   "dt_s": 0.0, "intrinsics": False, "cost_gain_all": 0.0, "rotation_time": False, "flat_field": True}
        out.setdefault("refinement", {})["camera_photometric"] = {**metrics, "applied": applied}
        metrics["applied"] = applied
        finish(a, report, metrics, out)
        return
    if selected_camera(calib) == "fisheye" and not a.validation_run:
        metrics["validation"] = {"passed": False, "reason": "fisheye calibration requires independent validation-run data"}
        log("fisheye candidate is diagnostic only without independent multi-segment validation; parent kept")
        finish(a, report, metrics, None)
        return
    halves = models[selected]
    intr_ok = selected != "rotation_time"
    start = (halves[0]["theta"] + halves[1]["theta"]) / 2
    optimizer = {}
    th = fit(score, model_free[selected], None, start=start, diagnostics=optimizer)
    gain_all = 1 - score(th, None) / score(zero, None)
    full_gains = [1 - score(th, p) / base[p] for p in (0, 1)]
    metrics["combined_fit"] = {"theta": th.tolist(), "gains": full_gains, "optimizer": optimizer}
    limits = np.array([AGREE_ROT_DEG] * 3 + [AGREE_DT_S, AGREE_FOCAL, AGREE_FOCAL, AGREE_K1])
    full_agrees = all(np.all(np.abs(th - h["theta"]) <= limits) for h in halves)
    metrics["combined_fit"]["agrees_with_halves"] = bool(full_agrees)
    if not optimizer["success"] or not np.isfinite(th).all() or min(full_gains) < MIN_GAIN or not full_agrees:
        log("combined fit failed its cost gate; parent kept")
        finish(a, report, metrics, None)
        return
    T = np.array(calib["transforms"][camera_transform_key(calib)]["transform_matrix"], float).reshape(4, 4)
    Rd = Rot.from_rotvec(np.radians(th[:3])).as_matrix()
    T[:3, :3] = T[:3, :3] @ Rd.T                     # p_cam' = Rd p_cam about the camera centre
    out = json.loads(json.dumps(calib))
    out["transforms"][camera_transform_key(out)]["transform_matrix"] = [float(v) for v in T.reshape(-1)]
    offs = out.setdefault("time_offsets", {})
    old = float(offs.get("camera", 0.0))
    offs["camera"] = round(old + float(th[3]), 4)
    offs["camera_source"] = (f"camera-photo: multi-view photo-consistency, {old:+.4f} {1000 * th[3]:+.1f} ms "
                             f"(held-out gains {100 * halves[0]['test_gain']:.1f} / {100 * halves[1]['test_gain']:.1f} %)")
    if intr_ok:
        fx, fy, cx, cy = out["camera"]["intrinsics"]
        out["camera"]["intrinsics"] = [fx * (1 + th[4]), fy * (1 + th[5]), cx, cy]
        distortion = np.array(out["camera"].get("distortion", [0] * 5), float)
        distortion[0] += float(th[6])
        out["camera"]["distortion"] = distortion.tolist()
        out["camera"]["source"] = "camera-photo (multi-view photo-consistency)"
        out["camera"].pop("flat_field", None)  # old pixel-coordinate correction belongs to the old K/D
    sync_camera_profile(out)
    try:
        select_camera(out)  # includes finite/valid same-K rectification checks
    except ValueError as exc:
        metrics["camera_model_validity"] = {"passed": False, "reason": str(exc)}
        log(f"candidate camera model rejected: {exc}; parent kept")
        finish(a, report, metrics, None)
        return
    metrics["camera_model_validity"] = {"passed": True}
    out["calibration_type"] = str(calib.get("calibration_type", "")) + "+camera_photometric"
    flat, metrics["flat_field"] = flat_field(data, th, log)
    if a.validation_run:
        # Independent evaluation below validates geometry/time. Do not mix in a
        # newly fitted illumination correction without its own holdout check.
        metrics["flat_field"]["applied"] = False
        metrics["flat_field"]["note"] = "diagnostic only: independent runs validate projection/time, not a new flat field"
        flat = None
    if flat is not None:
        out["camera"]["flat_field"] = flat
    train_range = data["stamp_range_s"]
    del score, data
    gc.collect()
    validation = []
    for val_arg in a.validation_run:
        val_run = me.resolve_run(val_arg)
        entry = {"run": str(val_run), "passed": False, "camera": selected_camera(calib), "refitted": False}
        validation.append(entry)
        try:
            require_run_camera(val_run, calib)
            manifest_path = val_run.parent.parent.parent / "round.json"
            manifest = json.loads(manifest_path.read_text())
            if manifest.get("status") != "done":
                raise ValueError("validation round is not complete")
            val_bag = pathlib.Path(manifest["dataset"])
            val_dir = out_dir / f"validation_{len(validation):02d}"
            val_dir.mkdir(exist_ok=True)
            log(f"independent validation {len(validation)}: {val_run} (fixed candidate; no refit)")
            val_data, why = collect(val_run, val_bag, calib, val_dir, (3.0, 1e9), log)
            if val_data is None:
                raise ValueError(why)
            entry["coverage"] = recording_coverage(val_bag, val_data, calib["camera"])
            if selected_camera(calib) == "fisheye" and entry["coverage"]["camera_segment_count"] < 2:
                del val_data
                raise ValueError("fisheye validation requires at least two recorded camera segments")
            lo, hi = val_data["stamp_range_s"]
            if max(lo, train_range[0]) < min(hi, train_range[1]):
                del val_data
                raise ValueError("validation time span overlaps the training data")
            val_score = make_score(val_data)
            before = [val_score(zero, p) for p in (0, 1)]
            after = [val_score(th, p) for p in (0, 1)]
            gains = [1 - c / b for b, c in zip(before, after)]
            entry.update({"cost_parent": before, "cost_candidate": after, "gains": gains,
                          "passed": bool(np.isfinite(gains).all() and min(gains) >= 0)})
            log(f"independent validation gains {[round(100 * g, 3) for g in gains]} %, passed={entry['passed']}")
            del val_score, val_data
            gc.collect()
        except (ValueError, OSError, KeyError) as exc:
            entry["reason"] = str(exc)
            log(f"independent validation unavailable/rejected: {exc}")
    if a.validation_run:
        metrics["validation"] = {"passed": all(e["passed"] for e in validation),
                                 "min_gain": 0.0, "refitted": False, "runs": validation}
        if not metrics["validation"]["passed"]:
            log("independent validation failed; parent kept")
            finish(a, report, metrics, None)
            return
    applied = {"rot_delta_deg_cam_xyz": np.round(th[:3], 4).tolist(), "camera_time_offset_s": offs["camera"],
               "dt_s": round(float(th[3]), 5), "intrinsics": intr_ok, "cost_gain_all": round(gain_all, 4),
               "rotation_time": True, "flat_field": flat is not None}
    if intr_ok:
        applied.update({"fx_scale": round(1 + float(th[4]), 5), "fy_scale": round(1 + float(th[5]), 5), "delta_k1": round(float(th[6]), 5), "k1": out["camera"]["distortion"][0]})
    metrics["validated_parameters"] = {"camera": json.loads(json.dumps(out["camera"])),
                                       "transform_matrix": list(out["transforms"][camera_transform_key(out)]["transform_matrix"]),
                                       "camera_offset": offs["camera"]}
    out.setdefault("refinement", {})["camera_photometric"] = {**metrics, "applied": applied}
    metrics["applied"] = applied
    log(f"applied: camera rotation (camera frame) rx {th[0]:+.3f} ry {th[1]:+.3f} rz {th[2]:+.3f} deg, time offset "
        f"{old:+.4f} -> {offs['camera']:+.4f} s; intrinsics {'refined' if intr_ok else 'confirmed (unchanged)'}; "
        f"flat field {'written' if flat is not None else 'not applied'}; "
        f"cost {100 * gain_all:+.1f} % on all returns ({time.time() - t_start:.0f} s)")
    finish(a, report, metrics, out)


def finish(a, report, metrics, out):
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    if out is not None:
        sync_camera_profile(out)
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
