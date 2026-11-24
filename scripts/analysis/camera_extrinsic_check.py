#!/usr/bin/env python3
"""Camera<->LiDAR rotation consistency check from one mapping round (no targets needed).

Frames with little rotation (|yaw rate| < --max-yaw-rate, so the image time offset does not matter)
are scored on a (pitch, yaw) grid of extra camera rotations: score = mean normalised image gradient at
the projected LiDAR depth edges (same measure as camera_sync_check.py). The peak location is the
rotation the calibration is missing; the peak contrast tells whether the scene supports the estimate.
A calibrated camera peaks at (0, 0) within ~0.1 deg (1 px = 0.03 deg at f=1850).

usage: camera_extrinsic_check.py <fusion_dir> --bag <raw root> --calibration <calibration.json>
                                 [--frames 12] [--span 1.5] [--step 0.1] [--json out.json]
"""
import argparse, json, pathlib, sys, tempfile
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "calib"))
from livo_calib import dump_camera_stream, decode_hevc, read_run_frames  # noqa: E402
from livo.camera import camera_transform_key, select_camera  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fusion_dir"); ap.add_argument("--bag", required=True); ap.add_argument("--calibration", required=True)
    ap.add_argument("--frames", type=int, default=12); ap.add_argument("--max-yaw-rate", type=float, default=1.5)
    ap.add_argument("--min-speed", type=float, default=0.5, help="m/s; moving frames see structure ahead")
    ap.add_argument("--scale", type=float, default=0.5); ap.add_argument("--accumulate", type=float, default=0.6)
    ap.add_argument("--range", type=float, nargs=2, default=(3.0, 60.0)); ap.add_argument("--max-points", type=int, default=400000)
    ap.add_argument("--span", type=float, default=1.5, help="search +- span [deg]"); ap.add_argument("--step", type=float, default=0.1)
    ap.add_argument("--json", type=pathlib.Path); ap.add_argument("--out-dir", type=pathlib.Path)
    a = ap.parse_args()
    import cv2
    from scipy.spatial.transform import Rotation as R, Slerp

    fusion = pathlib.Path(a.fusion_dir)
    params = {}
    for line in (fusion / "parameters.txt").read_text().splitlines():
        k, _, v = line.partition("="); params.setdefault(k, v)
    cam_offset = float(params.get("effective.img_time_offset_sec", 0.0))
    calib = select_camera(json.loads(pathlib.Path(a.calibration).read_text()))
    cam = calib["camera"]; fx, fy, cx, cy = cam["intrinsics"]; dist = np.array(cam.get("distortion", [0] * 5), float)
    s = a.scale
    K = np.array([[fx * s, 0, cx * s], [0, fy * s, cy * s], [0, 0, 1]])
    T_rear_cam = np.asarray(calib["transforms"][camera_transform_key(calib)]["transform_matrix"], float).reshape(4, 4)

    fr = read_run_frames(fusion)
    stamps, pos, rot = fr["stamp"], fr["pos"], R.from_quat(fr["quat"])
    slerp = Slerp(stamps, rot)
    T_imu_rear = np.eye(4); T_imu_rear[:3, 3] = fr["t_li"]
    yaw_rate = np.gradient(np.unwrap(rot.as_euler("zyx")[:, 0]), stamps) * 180 / np.pi
    yaw_rate_s = np.convolve(yaw_rate, np.ones(5) / 5, mode="same")
    speed = np.linalg.norm(np.gradient(pos, stamps, axis=0), axis=1)
    out_dir = a.out_dir or pathlib.Path(tempfile.mkdtemp(prefix="camext_"))
    out_dir.mkdir(parents=True, exist_ok=True)
    round_json = fusion.parent.parent.parent / "round.json"
    start_rel = 0.0
    if round_json.exists():
        rj = json.loads(round_json.read_text())
        start_rel = float((rj.get("options") or {}).get("start_offset") or 0.0)
    img_stamps, stream = dump_camera_stream(pathlib.Path(a.bag), out_dir, duration=stamps[-1] - stamps[0] + 6.0,
                                           start=max(0.0, start_rel - 3.0), camera=cam)
    if not len(img_stamps):
        sys.exit("selected camera stream has no decodable frames in the run window")
    eff = img_stamps + cam_offset
    ok = (eff > stamps[0] + a.accumulate) & (eff < stamps[-1] - a.accumulate)
    yr = np.interp(eff, stamps, yaw_rate_s); sp = np.interp(eff, stamps, speed)
    cand = np.flatnonzero(ok & (np.abs(yr) < a.max_yaw_rate) & (sp > a.min_speed))
    if len(cand) == 0:
        sys.exit("no straight-driving images")
    step = max(1, len(cand) // a.frames)
    chosen = [int(k) for k in cand[::step]][:a.frames]
    images = {}
    for k, img in decode_hevc(stream, s, keep=set(chosen), camera=cam):
        g = cv2.GaussianBlur(img, (0, 0), 1.0).astype(np.float32)
        gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3); gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
        mag = cv2.GaussianBlur(np.sqrt(gx * gx + gy * gy), (0, 0), 1.5)
        images[k] = mag / (mag.mean() + 1e-6)
    stream.unlink(missing_ok=True)
    print(f"run {stamps[-1]-stamps[0]:.1f} s; {len(images)} straight-driving images (|yaw rate| < {a.max_yaw_rate} deg/s), camera offset {cam_offset:+.4f} s")

    grid = np.arange(-a.span, a.span + a.step / 2, a.step)
    rng = np.random.default_rng(0)
    k5 = np.ones((5, 5), np.uint8)
    acc = np.zeros((len(grid), len(grid))); cnt = np.zeros_like(acc)
    per_image = []
    for k in chosen:
        if k not in images:
            continue
        G = images[k]; h, w = G.shape; t = eff[k]
        sel = np.flatnonzero(np.abs(stamps - t) <= a.accumulate / 2)
        pts = []
        for i in sel:
            o, c = (fr["offset"][i] - 8) // 20, fr["count"][i]
            p = np.asarray(fr["archive"][o:o + c, :3], np.float64); p = p[np.isfinite(p).all(1)]
            Tw = np.eye(4); Tw[:3, :3] = rot[i].as_matrix(); Tw[:3, 3] = pos[i]; Tw = Tw @ T_imu_rear
            pts.append(p @ Tw[:3, :3].T + Tw[:3, 3])
        pw = np.vstack(pts)
        if len(pw) > a.max_points:
            pw = pw[rng.choice(len(pw), a.max_points, replace=False)]
        Tw = np.eye(4); Tw[:3, :3] = slerp([t])[0].as_matrix(); Tw[:3, 3] = [np.interp(t, stamps, pos[:, j]) for j in range(3)]
        T_cam0_world = np.linalg.inv(T_rear_cam) @ np.linalg.inv(Tw @ T_imu_rear)
        pc0 = pw @ T_cam0_world[:3, :3].T + T_cam0_world[:3, 3]
        sc = np.full((len(grid), len(grid)), np.nan)
        for ip, dp in enumerate(grid):
            for iy, dy in enumerate(grid):
                D = R.from_euler("xyz", [dp, dy, 0.0], degrees=True).as_matrix()  # camera x = pitch, y = yaw
                pc = pc0 @ D  # rotate the camera by D: p_cam' = D^T p_cam
                z = pc[:, 2]; inr = (z > a.range[0]) & (z < a.range[1])
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
                dz = np.where(valid, depth, 0).astype(np.float32); big = np.where(valid, depth, 1e6).astype(np.float32)
                near = cv2.erode(big, k5); far = cv2.dilate(dz, k5)
                valid_d = cv2.dilate(valid.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
                fg = valid & (far > 1.25 * near) & (depth <= 1.05 * near)
                above = np.zeros_like(valid_d); above[2:, :] = valid_d[:-2, :]
                sky = valid & ~above & (depth > 12.0)
                edges = fg | sky
                if edges.sum() < 200:
                    continue
                sc[ip, iy] = float(G[edges].mean())
        if np.isfinite(sc).sum() < 0.8 * sc.size:
            continue
        scn = sc / np.nanmedian(sc)
        j = np.unravel_index(np.nanargmax(scn), scn.shape)
        per_image.append({"image": int(k), "t_rel_s": round(float(t - stamps[0]), 1), "pitch_deg": round(float(grid[j[0]]), 2),
                          "yaw_deg": round(float(grid[j[1]]), 2), "contrast": round(float(np.nanmax(scn) - 1.0), 3)})
        m = np.isfinite(scn); acc[m] += scn[m]; cnt[m] += 1
    if not per_image:
        sys.exit("no usable images")
    C = acc / np.maximum(cnt, 1)
    j = np.unravel_index(np.nanargmax(C), C.shape)
    peak_p, peak_y = float(grid[j[0]]), float(grid[j[1]])
    # parabolic refinement along each axis
    def refine(vals, idx, step):
        if 0 < idx < len(vals) - 1 and np.isfinite(vals[idx - 1]) and np.isfinite(vals[idx + 1]):
            y0, y1, y2 = vals[idx - 1], vals[idx], vals[idx + 1]; den = y0 - 2 * y1 + y2
            if den < 0:
                return 0.5 * (y0 - y2) / den * step
        return 0.0
    peak_p += refine(C[:, j[1]], j[0], a.step); peak_y += refine(C[j[0], :], j[1], a.step)
    contrast = float(np.nanmax(C) - np.nanmedian(C))
    fpx = fx * np.pi / 180
    print(f"camera rotation residual (joint over {len(per_image)} images): pitch {peak_p:+.2f} deg, yaw {peak_y:+.2f} deg "
          f"(~{abs(peak_p)*fpx:.0f} / {abs(peak_y)*fpx:.0f} px at full resolution), peak contrast {contrast:.3f}")
    print("per image (pitch,yaw,contrast):", " ".join(f"{r['t_rel_s']:.0f}s:({r['pitch_deg']:+.1f},{r['yaw_deg']:+.1f},{r['contrast']:.2f})" for r in per_image))
    sp_ = np.array([r["pitch_deg"] for r in per_image]); sy_ = np.array([r["yaw_deg"] for r in per_image])
    print(f"per-image medians: pitch {np.median(sp_):+.2f} (MAD {np.median(np.abs(sp_-np.median(sp_)))/0.6745:.2f}) yaw {np.median(sy_):+.2f} (MAD {np.median(np.abs(sy_-np.median(sy_)))/0.6745:.2f}) deg")
    if a.json:
        a.json.write_text(json.dumps({"pitch_deg": round(peak_p, 3), "yaw_deg": round(peak_y, 3), "contrast": round(contrast, 4),
                                      "per_image": per_image, "grid_deg": [round(float(g), 2) for g in grid],
                                      "score": [[None if not np.isfinite(v) else round(float(v), 4) for v in row] for row in C]}, indent=1))


if __name__ == "__main__":
    main()
