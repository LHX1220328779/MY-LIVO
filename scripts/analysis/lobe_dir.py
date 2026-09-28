#!/usr/bin/env python3
"""Are the far-range pole 'lobes' along the line of sight (range physics: mixed pixel / footprint) or
tangential (pose error)?  Per source and range bin: point-level along-LOS (a) and tangential (b) offsets from
the pole axis, and per-frame centroid tangential offset (attitude-error signature grows ~ range)."""
import sys, pathlib, numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import pole_xsection as px
run = pathlib.Path(sys.argv[1]); band = (float(sys.argv[2]), float(sys.argv[3])); poles_xy = [tuple(map(float, s.split(":"))) for s in sys.argv[4].split(",")]
if not (run / "frames.csv").exists():
    run = sorted((run / "Log" / "fusion").glob("*"))[-1]
fr = px.load_frames(run); px.LEVER = fr["lever"]
stamps = fr["stamp"]; counts = fr["count"].astype(np.int64); n = len(stamps)
nat = np.column_stack([stamps, fr["tx"], fr["ty"], fr["tz"], fr["qx"], fr["qy"], fr["qz"], fr["qw"]])
poses = px.pose_rows(run / "trajectory_optimized.txt", stamps)
names = px.source_names(run) or ["s0", "s1", "s2", "s3"]
from scipy.spatial.transform import Rotation
origins = Rotation.from_quat(poses[:, 4:8]).apply(px.LEVER) + poses[:, 1:4]
P = {k: [] for k in range(len(poles_xy))}
with open(run / "scans.bin", "rb") as f, open(run / "sources.bin", "rb") as fsrc:
    for i in range(n):
        near = [k for k, (cx, cy) in enumerate(poles_xy) if np.hypot(origins[i, 0] - cx, origins[i, 1] - cy) < 65]
        if not near: continue
        off, cnt = int(fr["offset_bytes"][i]), int(counts[i])
        f.seek(off); pts = np.frombuffer(f.read(cnt * 20), dtype="<f4").reshape(-1, 5).astype(np.float64)
        fsrc.seek(8 + (off - 8) // 20); src = np.frombuffer(fsrc.read(cnt), dtype=np.uint8)
        w = px.world(pts, poses[i])
        for k in near:
            cx, cy = poles_xy[k]
            dd = np.hypot(w[:, 0] - cx, w[:, 1] - cy)
            sel = dd < 1.0
            if not sel.any(): continue
            P[k].append((i, w[sel], src[sel], np.linalg.norm(pts[sel, :3], axis=1)))
for k, (cx, cy) in enumerate(poles_xy):
    if not P[k]: continue
    allw = np.concatenate([w for _, w, _, _ in P[k]]); gz = np.percentile(allw[:, 2], 3)
    # crisp reference: front_right (0) <= 15 m, band
    ref = np.concatenate([w[(s == 0) & (r <= 15)] for _, w, s, r in P[k]] or [np.zeros((0, 3))])
    ref = ref[(ref[:, 2] - gz > band[0]) & (ref[:, 2] - gz < band[1])]
    if len(ref) < 40:
        ref = np.concatenate([w[(r <= 20)] for _, w, s, r in P[k]]); ref = ref[(ref[:, 2] - gz > band[0]) & (ref[:, 2] - gz < band[1])]
    X, Y = ref[:, 0], ref[:, 1]; M = np.column_stack([X, Y, np.ones(len(X))]); sol = np.linalg.lstsq(M, X**2 + Y**2, rcond=None)[0]
    c = np.array([sol[0] / 2, sol[1] / 2]); R = np.sqrt(sol[2] + c @ c)
    print(f"\n== pole {k} at ({cx:.1f},{cy:.1f}) fitted R={R*100:.1f} cm, n_ref={len(ref)}")
    print("source        range   frames  pts | point |b| P50/P90 | along-LOS a P10/P50/P90 | per-frame centroid b: mean  P50|b|  P90|b| | per-frame centroid a: P50 P90")
    for s_id in range(4):
        for lo, hi in ((5, 15), (15, 25), (25, 35), (35, 45), (45, 60)):
            fa, fb, pa, pb = [], [], [], []
            for i, w, s, r in P[k]:
                m = (s == s_id) & (r >= lo) & (r < hi) & (w[:, 2] - gz > band[0]) & (w[:, 2] - gz < band[1]) & (np.hypot(w[:, 0] - c[0], w[:, 1] - c[1]) < 0.5)
                if m.sum() < 4: continue
                u = c - origins[i, :2]; dist = np.linalg.norm(u); u /= dist; v = np.array([-u[1], u[0]])
                d = w[m, :2] - c; a = d @ u; b = d @ v
                pa.append(a); pb.append(b); fa.append(np.median(a)); fb.append(np.mean(b))
            if len(fa) < 3: continue
            pa = np.concatenate(pa); pb = np.concatenate(pb); fa = np.array(fa); fb = np.array(fb)
            print(f"{names[s_id]:12s} {lo:2d}-{hi:2d} m  {len(fa):4d} {len(pa):6d} | {np.median(np.abs(pb))*100:4.1f}/{np.percentile(np.abs(pb),90)*100:4.1f} cm | {np.percentile(pa,10)*100:5.1f}/{np.median(pa)*100:5.1f}/{np.percentile(pa,90)*100:5.1f} cm | {fb.mean()*100:5.1f} {np.median(np.abs(fb))*100:4.1f} {np.percentile(np.abs(fb),90)*100:4.1f} cm | {np.median(fa)*100:5.1f} {np.percentile(fa,90)*100:5.1f} cm")
