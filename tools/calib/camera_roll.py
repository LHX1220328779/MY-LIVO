#!/usr/bin/env python3
"""Camera roll (rotation about the optical axis) from the sky line: LiDAR vs image.

Every outdoor frame has a sky line - the top outline of trees, slopes, berms, buildings, poles against the sky -
that BOTH sensors see sharply: in the image it is the first bright->dark transition below the sky, in the LiDAR
the topmost return of each image column (nothing returns from the sky). A camera roll error rotates the LiDAR
projection about the principal point, so the vertical offset LiDAR-minus-image grows linearly across the image
width; its slope is the roll. Per frame the offset is solved with its own constant term, which absorbs camera
pitch, the principal-point row and the LiDAR ring sampling (the topmost return of an object lies up to one ring
spacing below its true top - an ANGULAR spacing, i.e. the same number of pixels on both sides of the image).
Frames are solved independently and combined robustly (median, MAD, time-ordered thirds).

This replaces the structure-edge roll solver on routes without poles/buildings (truck29raw 20261013: one usable
pole in 300 s; trees/slopes everywhere): the old filter deliberately discarded foliage, i.e. the whole sky line.
"""
import numpy as np


def image_skyline(img, col_step=4, min_contrast=30.0, run=6):
    """Sub-pixel sky-line row per column (columns u = col_step/2 + i*col_step); NaN where the column does not
    start in sky. The top rows give the sky level; the sky line is where the column first stays darker than the
    midpoint between sky and object for `run` rows."""
    import cv2
    g = cv2.GaussianBlur(img, (0, 0), 1.5).astype(np.float32)
    h, w = g.shape
    cols = np.arange(col_step // 2, w, col_step)
    out = np.full(len(cols), np.nan)
    sky = float(np.percentile(g[:max(6, h // 50)], 75))
    for i, u in enumerate(cols):
        col = g[:, max(0, u - 1):u + 2].mean(axis=1)
        if col[0] < sky - 0.5 * min_contrast:
            continue  # column starts on an object: no sky line visible in it
        dark = col < sky - min_contrast
        idx = np.flatnonzero(dark)
        if len(idx) == 0:
            continue
        # first row that starts a dark run of `run` rows
        v = None
        for j in idx:
            if j + run <= h and dark[j:j + run].all():
                v = int(j)
                break
        if v is None or v < 3:
            continue
        below = float(np.median(col[v:min(h, v + 3 * run)]))
        thr = 0.5 * (float(np.median(col[max(0, v - 3 * run):v])) + below)
        # sub-pixel crossing of the midpoint threshold just above v
        k = v
        while k > 0 and col[k - 1] < thr:
            k -= 1
        if k == 0 or col[k - 1] == col[k]:
            continue
        out[i] = (k - 1) + (col[k - 1] - thr) / (col[k - 1] - col[k])
    return cols, out


def lidar_skyline(points_cam, project, w, h, col_step=4, min_depth=4.0, max_depth=120.0):
    """Topmost projected return per column: (index of the point, v) for columns with returns."""
    z = points_cam[:, 2]
    ok = (z > min_depth) & (z < max_depth)
    pc = points_cam[ok]
    idx_all = np.flatnonzero(ok)
    uv = project(pc)
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    uv, idx_all = uv[inside], idx_all[inside]
    bins = (uv[:, 0] // col_step).astype(int)
    n = (w + col_step - 1) // col_step
    best_v = np.full(n, np.inf)
    best_i = np.full(n, -1)
    order = np.argsort(uv[:, 1])[::-1]  # write the topmost (smallest v) last
    best_v[bins[order]] = uv[order, 1]
    best_i[bins[order]] = idx_all[order]
    return best_i, best_v


def frame_roll(points_cam_top, cols, v_img, project_delta, cx, min_cols=40, min_span=0.35, w=None):
    """Roll (deg) of one frame: least-squares (Huber IRLS) of the LiDAR-minus-image offset with a free constant,
    the roll entering through the exact projection derivative. Returns (roll_deg, offset_px, rms_px, n) or None."""
    valid = np.isfinite(v_img) & np.isfinite(points_cam_top[:, 0])
    if valid.sum() < min_cols:
        return None
    u = cols[valid]
    if w is not None and (u.max() - u.min()) < min_span * w:
        return None
    p = points_cam_top[valid]
    v_img = v_img[valid]
    v0 = project_delta(p, 0.0)[:, 1]
    dv = (project_delta(p, 0.1)[:, 1] - v0) / 0.1  # px per deg of roll
    r0 = v0 - v_img
    keep = np.abs(r0 - np.median(r0)) < 25.0  # tree tops swaying, LiDAR FOV clipping, wrong sky line
    if keep.sum() < min_cols:
        return None
    A = np.c_[np.ones(keep.sum()), dv[keep]]
    y = -r0[keep]
    wts = np.ones(keep.sum())
    x = np.zeros(2)
    for _ in range(10):
        W = np.sqrt(wts)
        x = np.linalg.lstsq(A * W[:, None], y * W, rcond=None)[0]
        res = y - A @ x
        s = 1.4826 * np.median(np.abs(res)) + 1e-6
        wts = np.where(np.abs(res) <= 1.5 * s, 1.0, 1.5 * s / np.abs(res))
    res = y - A @ x
    return float(x[1]), float(-x[0]), float(np.sqrt(np.mean(res ** 2))), int(keep.sum())


def combine(rolls, times):
    """Robust combination of per-frame rolls: median, MAD, time-ordered thirds."""
    rolls, times = np.asarray(rolls, float), np.asarray(times, float)
    order = np.argsort(times)
    rolls = rolls[order]
    med = float(np.median(rolls))
    mad = float(np.median(np.abs(rolls - med)))
    keep = np.abs(rolls - med) <= max(3.0 * 1.4826 * mad, 0.05)
    est = float(np.median(rolls[keep]))
    thirds = [rolls[i * len(rolls) // 3:(i + 1) * len(rolls) // 3] for i in range(3)]
    groups = [float(np.median(t)) for t in thirds if len(t) >= 3]
    se = 1.2533 * 1.4826 * mad / np.sqrt(max(1, keep.sum()))
    return {"roll_deg": round(est, 4), "frames": int(len(rolls)), "frames_kept": int(keep.sum()),
            "mad_deg": round(mad, 4), "se_deg": round(float(se), 4),
            "group_roll_deg": [round(g, 4) for g in groups],
            "group_spread_deg": round(float(max(groups) - min(groups)), 4) if len(groups) >= 2 else None}


def scan_cost(frames, deltas, project_delta, col_step=4, w=1920, h=1080, cap_px=15.0):
    """Truncated-L1 sky-line misfit of every frame for every candidate roll (deg).

    frames: list of (points_cam, cols, v_img) with points_cam already restricted to returns near the LiDAR top.
    For each roll the LiDAR top envelope is re-selected from the rotated projection (self-consistent; no
    linearisation), the per-frame constant offset is the median residual. Returns cost[frame, delta]."""
    cost = np.full((len(frames), len(deltas)), np.nan)
    for fi, (pc, cols, v_img) in enumerate(frames):
        valid_img = np.isfinite(v_img)
        for di, d in enumerate(deltas):
            uv = project_delta(pc, d)
            inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
            uv = uv[inside]
            b = (uv[:, 0] // col_step).astype(int)
            top = np.full(len(cols), np.inf)
            np.minimum.at(top, np.clip(b, 0, len(cols) - 1), uv[:, 1])
            ok = valid_img & np.isfinite(top)
            if ok.sum() < 40:
                continue
            r = top[ok] - v_img[ok]
            r = r - np.median(r)
            cost[fi, di] = float(np.mean(np.minimum(np.abs(r), cap_px)))
    return cost


def scan_roll(cost, deltas):
    """Global roll (sum of per-frame costs, parabolic refinement) + per-frame argmins for spread/thirds."""
    good = np.isfinite(cost).all(axis=1)
    c = cost[good]
    if len(c) < 6:
        return None, {"error": f"only {len(c)} frames with a usable sky line"}
    total = c.mean(axis=0)
    i = int(np.argmin(total))
    est = float(deltas[i])
    if 0 < i < len(deltas) - 1:
        y0, y1, y2 = total[i - 1], total[i], total[i + 1]
        den = y0 - 2 * y1 + y2
        if den > 0:
            est += 0.5 * (y0 - y2) / den * float(deltas[1] - deltas[0])
    per = np.array([float(deltas[int(np.argmin(row))]) for row in c])
    thirds = [c[k * len(c) // 3:(k + 1) * len(c) // 3] for k in range(3)]
    groups = [float(deltas[int(np.argmin(t.mean(axis=0)))]) for t in thirds if len(t) >= 3]
    curvature = float((total[min(i + 2, len(total) - 1)] + total[max(i - 2, 0)] - 2 * total[i]) / 2)
    return est, {"roll_deg": round(est, 4), "frames": int(len(c)), "per_frame_mad_deg": round(float(np.median(np.abs(per - np.median(per)))), 4),
                 "group_roll_deg": [round(g, 4) for g in groups],
                 "group_spread_deg": round(float(max(groups) - min(groups)), 4) if len(groups) >= 2 else None,
                 "cost_px_at_zero": round(float(total[int(np.argmin(np.abs(deltas)))]), 4), "cost_px_at_min": round(float(total[i]), 4),
                 "curvature_px": round(curvature, 4),
                 "profile": [(round(float(d), 2), round(float(v), 4)) for d, v in zip(deltas, total)][::2]}
