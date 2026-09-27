#!/usr/bin/env python3
"""Camera <-> LiDAR/IMU time-offset check by LiDAR-edge / image-edge alignment (no lever-arm bias).

While the truck turns, the LiDAR geometry of the round (scans.bin placed with the round's
poses) is projected into the camera with a candidate time delta added to the image time; the
camera pose is the interpolated LIO pose at that time, so translation is modelled and a camera
far from the rotation centre does not bias the estimate (the weakness of optical flow vs gyro).
Depth discontinuities of the projected cloud (object silhouettes, berm tops, skyline) must fall
on image edges; the mean image-gradient magnitude at those pixels peaks at the right delta.
Each image gives a score curve; the curves are summed and the peak is refined parabolically.

The result is a DELTA to add to the camera offset the round already used
(effective.img_time_offset_sec in parameters.txt).

usage: camera_sync_check.py <fusion_dir> --bag DIR --calibration calibration.json [--frames 40]
       [--min-yaw-rate 4] [--span 0.15 --step 0.005] [--json out.json] [--out-dir DIR]
"""
import argparse, json, pathlib, sys, tempfile
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "calib"))
sys.path.insert(0, str(REPO / "tools"))
from livo_calib import dump_camera_stream, decode_hevc, read_run_frames  # noqa: E402


def read_params(fusion):
    p = {}
    f = pathlib.Path(fusion) / "parameters.txt"
    if f.exists():
        for line in f.read_text().splitlines():
            k, _, v = line.partition("=")
            p.setdefault(k, v)
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fusion_dir"); ap.add_argument("--bag", required=True); ap.add_argument("--calibration", required=True)
    ap.add_argument("--frames", type=int, default=40); ap.add_argument("--min-yaw-rate", type=float, default=4.0, help="deg/s")
    ap.add_argument("--scale", type=float, default=0.25, help="working image scale")
    ap.add_argument("--max-points", type=int, default=300000); ap.add_argument("--accumulate", type=float, default=1.2)
    ap.add_argument("--range", type=float, nargs=2, default=(3.0, 80.0))
    ap.add_argument("--span", type=float, default=0.15, help="search +- span [s]"); ap.add_argument("--step", type=float, default=0.005)
    ap.add_argument("--json", type=pathlib.Path); ap.add_argument("--out-dir", type=pathlib.Path)
    ap.add_argument("--save-overlays", action="store_true", help="write edge overlays at the best delta")
    a = ap.parse_args()
    import cv2
    from scipy.spatial.transform import Rotation as R, Slerp

    fusion = pathlib.Path(a.fusion_dir)
    params = read_params(fusion)
    cam_offset = float(params.get("effective.img_time_offset_sec", params.get("time_offset.img_time_offset", 0.0)))
    calib = json.loads(pathlib.Path(a.calibration).read_text())
    cam = calib["camera"]; fx, fy, cx, cy = cam["intrinsics"]; dist = np.array(cam.get("distortion", [0] * 5), float)
    s = a.scale
    K = np.array([[fx * s, 0, cx * s], [0, fy * s, cy * s], [0, 0, 1]])
    T_rear_cam = np.asarray(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    T_cam_rear = np.linalg.inv(T_rear_cam)

    fr = read_run_frames(fusion)
    stamps, pos, rot = fr["stamp"], fr["pos"], R.from_quat(fr["quat"])
    slerp = Slerp(stamps, rot)
    T_imu_rear = np.eye(4); T_imu_rear[:3, 3] = fr["t_li"]
    yaw_rate = np.gradient(np.unwrap(rot.as_euler("zyx")[:, 0]), stamps) * 180 / np.pi
    yaw_rate_s = np.convolve(yaw_rate, np.ones(5) / 5, mode="same")
    print(f"run {stamps[-1]-stamps[0]:.1f} s, {len(stamps)} frames, camera offset in effect {cam_offset:+.4f} s")

    out_dir = a.out_dir or pathlib.Path(tempfile.mkdtemp(prefix="camsync_"))
    out_dir.mkdir(parents=True, exist_ok=True)
    round_json = fusion.parent.parent.parent / "round.json"
    start_rel = 0.0
    if round_json.exists():
        rj = json.loads(round_json.read_text())
        start_rel = float((rj.get("options") or {}).get("start_offset") or (rj.get("playback") or {}).get("start_offset") or 0.0)
    img_stamps, stream = dump_camera_stream(pathlib.Path(a.bag), out_dir, duration=stamps[-1] - stamps[0] + 6.0,
                                            start=max(0.0, start_rel - 3.0))
    eff = img_stamps + cam_offset
    if eff[-1] < stamps[0] or eff[0] > stamps[-1]:
        sys.exit("camera stream does not overlap the run window (start offset?)")
    ok = (eff > stamps[0] + a.accumulate) & (eff < stamps[-1] - a.accumulate)
    yr_img = np.interp(eff, stamps, yaw_rate_s)
    cand = np.flatnonzero(ok & (np.abs(yr_img) > a.min_yaw_rate))
    if len(cand) == 0:
        sys.exit("no images during turns; lower --min-yaw-rate")
    step = max(1, len(cand) // a.frames)
    chosen = [int(k) for k in cand[::step]][:a.frames]
    images = {}
    for k, img in decode_hevc(stream, s, keep=set(chosen)):
        g = cv2.GaussianBlur(img, (0, 0), 1.0).astype(np.float32)
        gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3); gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
        mag = cv2.GaussianBlur(np.sqrt(gx * gx + gy * gy), (0, 0), 1.5)
        images[k] = mag / (mag.mean() + 1e-6)
    stream.unlink(missing_ok=True)
    print(f"{len(chosen)} images during turns (|yaw rate| > {a.min_yaw_rate} deg/s), decoded {len(images)}")

    deltas = np.arange(-a.span, a.span + a.step / 2, a.step)
    rng = np.random.default_rng(0)
    k5 = np.ones((5, 5), np.uint8)
    curves, results = [], []
    h = w = None
    for k in chosen:
        if k not in images:
            continue
        G = images[k]; h, w = G.shape
        t = eff[k]
        sel = np.flatnonzero(np.abs(stamps - t) <= a.accumulate / 2)
        pts = []
        for i in sel:
            o, c = (fr["offset"][i] - 8) // 20, fr["count"][i]
            p = np.asarray(fr["archive"][o:o + c, :3], np.float64)
            p = p[np.isfinite(p).all(1)]
            Tw = np.eye(4); Tw[:3, :3] = rot[i].as_matrix(); Tw[:3, 3] = pos[i]
            Tw = Tw @ T_imu_rear
            pts.append(p @ Tw[:3, :3].T + Tw[:3, 3])
        if not pts:
            continue
        pw = np.vstack(pts)
        if len(pw) > a.max_points:
            pw = pw[rng.choice(len(pw), a.max_points, replace=False)]
        scores = np.full(len(deltas), np.nan)
        best_edges = None
        for di, d in enumerate(deltas):
            tt = np.clip(t + d, stamps[0], stamps[-1])
            Tw = np.eye(4); Tw[:3, :3] = slerp([tt])[0].as_matrix()
            Tw[:3, 3] = [np.interp(tt, stamps, pos[:, j]) for j in range(3)]
            T_cam_world = T_cam_rear @ np.linalg.inv(Tw @ T_imu_rear)
            pc = pw @ T_cam_world[:3, :3].T + T_cam_world[:3, 3]
            z = pc[:, 2]
            inr = (z > a.range[0]) & (z < a.range[1])
            if inr.sum() < 1000:
                continue
            uv = cv2.projectPoints(pc[inr].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
            u = np.round(uv[:, 0]).astype(int); v = np.round(uv[:, 1]).astype(int); zz = z[inr]
            inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            if inside.sum() < 1000:
                continue
            depth = np.full((h, w), np.inf, np.float32)
            np.minimum.at(depth, (v[inside], u[inside]), zz[inside].astype(np.float32))
            valid = np.isfinite(depth)
            dz = np.where(valid, depth, 0).astype(np.float32)
            big = np.where(valid, depth, 1e6).astype(np.float32)
            near = cv2.erode(big, k5)                      # nearest surface in the 5x5 neighbourhood
            far = cv2.dilate(dz, k5)                       # farthest surface in the neighbourhood
            valid_d = cv2.dilate(valid.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
            # depth-ratio edges on the near side, and skyline (no returns two rows above)
            fg = valid & (far > 1.25 * near) & (depth <= 1.05 * near)
            above = np.zeros_like(valid_d); above[2:, :] = valid_d[:-2, :]
            sky = valid & ~above & (depth > 12.0)
            edges = fg | sky
            n = int(edges.sum())
            if n < 200:
                continue
            scores[di] = float(G[edges].mean())
            if a.save_overlays and (best_edges is None or scores[di] >= np.nanmax(scores)):
                best_edges = (edges.copy(), float(d))
        if np.isfinite(scores).sum() < len(deltas) * 0.8:
            continue
        j = int(np.nanargmax(scores)); best = deltas[j]
        if 0 < j < len(deltas) - 1 and np.isfinite(scores[j - 1]) and np.isfinite(scores[j + 1]):
            y0, y1, y2 = scores[j - 1], scores[j], scores[j + 1]
            den = y0 - 2 * y1 + y2
            if den < 0:
                best = deltas[j] + 0.5 * (y0 - y2) / den * a.step
        contrast = float((np.nanmax(scores) - np.nanmedian(scores)) / (np.nanmedian(scores) + 1e-6))
        results.append({"image": int(k), "t_rel_s": round(float(t - stamps[0]), 2), "yaw_rate_dps": round(float(yr_img[k]), 1),
                        "delta_ms": round(1000 * float(best), 1), "contrast": round(contrast, 3),
                        "edge": bool(j in (0, len(deltas) - 1))})
        curves.append(scores / np.nanmedian(scores))
        if a.save_overlays and best_edges is not None:
            canvas = cv2.cvtColor(np.clip(G * 40, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            canvas[best_edges[0]] = (0, 0, 255)
            cv2.imwrite(str(out_dir / f"edges_t{int(t - stamps[0]):03d}.jpg"), canvas)
    if not results:
        sys.exit("no usable images")
    C = np.nanmean(np.vstack(curves), axis=0)
    j = int(np.nanargmax(C)); agg = deltas[j]
    if 0 < j < len(deltas) - 1:
        y0, y1, y2 = C[j - 1], C[j], C[j + 1]
        den = y0 - 2 * y1 + y2
        if den < 0:
            agg = deltas[j] + 0.5 * (y0 - y2) / den * a.step
    good = [r for r in results if not r["edge"]]
    d = np.array([r["delta_ms"] for r in (good or results)])
    med = float(np.median(d)); mad = float(np.median(np.abs(d - med)) / 0.6745)
    # weighted by |yaw rate| (fast turns localise the peak better)
    wgt = np.array([abs(r["yaw_rate_dps"]) for r in (good or results)])
    order = np.argsort(d); cw = np.cumsum(wgt[order]); wmed = float(d[order][np.searchsorted(cw, cw[-1] / 2)])
    summary = {"camera_offset_in_effect_s": cam_offset, "images": len(results), "images_used": len(d),
               "delta_ms_median": round(med, 1), "delta_ms_weighted_median": round(wmed, 1), "delta_ms_mad": round(mad, 1),
               "delta_ms_joint": round(1000 * float(agg), 1), "joint_peak_contrast": round(float((np.nanmax(C) - np.nanmedian(C))), 4),
               "recommended_camera_offset_s": round(cam_offset + agg, 4), "per_image": results,
               "curve": {"delta_ms": [round(1000 * x, 1) for x in deltas], "score": [round(float(c), 4) for c in C]}}
    print(f"camera time delta: joint peak {1000*agg:+.1f} ms (contrast {summary['joint_peak_contrast']:.3f}); per-image median {med:+.1f} ms, "
          f"yaw-rate-weighted median {wmed:+.1f} ms, MAD {mad:.1f} ms ({len(d)}/{len(results)} images)"
          f" -> recommended camera offset {cam_offset + agg:+.4f} s (in effect {cam_offset:+.4f})")
    print("per image:", " ".join(f"{r['t_rel_s']:.0f}s@{r['yaw_rate_dps']:+.0f}:{r['delta_ms']:+.0f}{'*' if r['edge'] else ''}" for r in results))
    if a.json:
        a.json.write_text(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
