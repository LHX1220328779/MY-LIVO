#!/usr/bin/env python3
"""Visual camera<->LiDAR alignment check of one mapping round: decodes a few camera images and draws the
LiDAR points of the surrounding scans (world frame, run poses) projected with the calibration's camera
intrinsics / camera->rear-axle transform and the image time offset in effect. Depth is colour coded
(near = red, far = blue). Misaligned silhouettes (berm tops, vehicles, poles) mean a wrong camera
extrinsic / intrinsic or image time offset; the coloured map inherits exactly this misalignment.

usage: camera_overlay_check.py <fusion_dir> --bag <raw root> --calibration <calibration.json>
                               --times 5 25 45 [--accumulate 0.4] [--scale 0.5] --out-dir DIR
"""
import argparse, json, pathlib, sys, tempfile
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "calib"))
from livo_calib import dump_camera_stream, decode_hevc, read_run_frames  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fusion_dir"); ap.add_argument("--bag", required=True); ap.add_argument("--calibration", required=True)
    ap.add_argument("--times", type=float, nargs="+", required=True, help="seconds after the first run frame")
    ap.add_argument("--accumulate", type=float, default=0.4); ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--range", type=float, nargs=2, default=(2.0, 60.0)); ap.add_argument("--out-dir", type=pathlib.Path, required=True)
    ap.add_argument("--camera-offset", type=float, help="override the image time offset in effect [s]")
    ap.add_argument("--rot-delta", type=float, nargs=3, default=(0, 0, 0), help="extra camera rotation (deg, xyz in camera frame) to test")
    a = ap.parse_args()
    import cv2
    from scipy.spatial.transform import Rotation as R, Slerp

    fusion = pathlib.Path(a.fusion_dir)
    params = {}
    for line in (fusion / "parameters.txt").read_text().splitlines():
        k, _, v = line.partition("="); params.setdefault(k, v)
    cam_offset = a.camera_offset if a.camera_offset is not None else float(params.get("effective.img_time_offset_sec", 0.0))
    calib = json.loads(pathlib.Path(a.calibration).read_text())
    cam = calib["camera"]; fx, fy, cx, cy = cam["intrinsics"]; dist = np.array(cam.get("distortion", [0] * 5), float)
    s = a.scale
    K = np.array([[fx * s, 0, cx * s], [0, fy * s, cy * s], [0, 0, 1]])
    T_rear_cam = np.asarray(calib["transforms"]["midrange_camera_to_rear_axle"]["transform_matrix"], float).reshape(4, 4)
    if any(a.rot_delta):
        D = np.eye(4); D[:3, :3] = R.from_euler("xyz", a.rot_delta, degrees=True).as_matrix(); T_rear_cam = T_rear_cam @ D
    T_cam_rear = np.linalg.inv(T_rear_cam)

    fr = read_run_frames(fusion)
    stamps, pos, rot = fr["stamp"], fr["pos"], R.from_quat(fr["quat"])
    slerp = Slerp(stamps, rot)
    T_imu_rear = np.eye(4); T_imu_rear[:3, 3] = fr["t_li"]
    a.out_dir.mkdir(parents=True, exist_ok=True)
    round_json = fusion.parent.parent.parent / "round.json"
    start_rel = 0.0
    if round_json.exists():
        rj = json.loads(round_json.read_text())
        start_rel = float((rj.get("options") or {}).get("start_offset") or 0.0)
    img_stamps, stream = dump_camera_stream(pathlib.Path(a.bag), a.out_dir, duration=stamps[-1] - stamps[0] + 6.0, start=max(0.0, start_rel - 3.0))
    eff = img_stamps + cam_offset
    want = {}
    for t_rel in a.times:
        t = stamps[0] + t_rel
        k = int(np.argmin(np.abs(eff - t)))
        want[k] = t_rel
    print(f"camera offset in effect {cam_offset:+.4f} s; images {sorted(want)}")
    for k, img in decode_hevc(stream, s, keep=set(want)):
        t = eff[k]; h, w = img.shape
        sel = np.flatnonzero(np.abs(stamps - t) <= a.accumulate / 2)
        pts = []
        for i in sel:
            o, c = (fr["offset"][i] - 8) // 20, fr["count"][i]
            p = np.asarray(fr["archive"][o:o + c, :3], np.float64); p = p[np.isfinite(p).all(1)]
            Tw = np.eye(4); Tw[:3, :3] = rot[i].as_matrix(); Tw[:3, 3] = pos[i]; Tw = Tw @ T_imu_rear
            pts.append(p @ Tw[:3, :3].T + Tw[:3, 3])
        pw = np.vstack(pts)
        Tw = np.eye(4); Tw[:3, :3] = slerp([t])[0].as_matrix(); Tw[:3, 3] = [np.interp(t, stamps, pos[:, j]) for j in range(3)]
        T_cam_world = T_cam_rear @ np.linalg.inv(Tw @ T_imu_rear)
        pc = pw @ T_cam_world[:3, :3].T + T_cam_world[:3, 3]
        z = pc[:, 2]; inr = (z > a.range[0]) & (z < a.range[1])
        uv = cv2.projectPoints(pc[inr].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        u = np.round(uv[:, 0]).astype(int); v = np.round(uv[:, 1]).astype(int); zz = z[inr]
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v, zz = u[inside], v[inside], zz[inside]
        order = np.argsort(-zz)  # far first, near drawn last
        col = cv2.applyColorMap((np.clip(1 - np.log(zz / a.range[0]) / np.log(a.range[1] / a.range[0]), 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET).reshape(-1, 3)
        out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for j in order:
            out[v[j], u[j]] = col[j]
        cv2.putText(out, f"t+{want[k]:.1f}s img {k} cam_off {cam_offset:+.4f}s rot_delta {a.rot_delta}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        path = a.out_dir / f"overlay_t{want[k]:05.1f}.jpg"
        cv2.imwrite(str(path), out, [cv2.IMWRITE_JPEG_QUALITY, 80])
        print("wrote", path, f"points {inside.sum()}")
    stream.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
