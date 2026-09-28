#!/usr/bin/env python3
"""Per-source, per-frame pole centroid offset in the TRAVEL frame (fwd = velocity direction, right), relative to
the pole axis fitted from crisp front_right <=15 m returns.  A time offset of one source shows as fwd offset
proportional to speed; an extrinsic translation as a constant offset; a yaw error grows with range."""
import sys, pathlib, numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import pole_xsection as px
from scipy.spatial.transform import Rotation
run = pathlib.Path(sys.argv[1]); band = (float(sys.argv[2]), float(sys.argv[3])); poles_xy = [tuple(map(float, s.split(":"))) for s in sys.argv[4].split(",")]
if not (run / "frames.csv").exists():
    run = sorted((run / "Log" / "fusion").glob("*"))[-1]
fr = px.load_frames(run); px.LEVER = fr["lever"]
stamps = fr["stamp"]; counts = fr["count"].astype(np.int64); n = len(stamps)
poses = px.pose_rows(run / "trajectory_optimized.txt", stamps)
names = px.source_names(run) or ["s0", "s1", "s2", "s3"]
origins = Rotation.from_quat(poses[:, 4:8]).apply(px.LEVER) + poses[:, 1:4]
vel = np.gradient(poses[:, 1:3], stamps, axis=0); speed = np.linalg.norm(vel, axis=1)
rows = []  # pole, frame, source, range, fwd, right, npts, speed
with open(run / "scans.bin", "rb") as f, open(run / "sources.bin", "rb") as fsrc:
    for i in range(n):
        near = [k for k, (cx, cy) in enumerate(poles_xy) if np.hypot(origins[i, 0] - cx, origins[i, 1] - cy) < 40]
        if not near or speed[i] < 0.5: continue
        off, cnt = int(fr["offset_bytes"][i]), int(counts[i])
        f.seek(off); pts = np.frombuffer(f.read(cnt * 20), dtype="<f4").reshape(-1, 5).astype(np.float64)
        fsrc.seek(8 + (off - 8) // 20); src = np.frombuffer(fsrc.read(cnt), dtype=np.uint8)
        w = px.world(pts, poses[i]); rng = np.linalg.norm(pts[:, :3], axis=1)
        for k in near:
            cx, cy = poles_xy[k]
            sel = np.hypot(w[:, 0] - cx, w[:, 1] - cy) < 1.0
            if sel.sum() < 4: continue
            rows.append((k, i, w[sel], src[sel], rng[sel]))
fwd_dir = vel / np.maximum(speed, 1e-6)[:, None]
for k, (cx, cy) in enumerate(poles_xy):
    R_ = [r for r in rows if r[0] == k]
    if not R_: continue
    allw = np.concatenate([r[2] for r in R_]); gz = np.percentile(allw[:, 2], 3)
    ref = np.concatenate([r[2][(r[3] == 0) & (r[4] <= 15)] for r in R_]); ref = ref[(ref[:, 2] - gz > band[0]) & (ref[:, 2] - gz < band[1])]
    if len(ref) < 40:
        ref = np.concatenate([r[2][(r[4] <= 20)] for r in R_]); ref = ref[(ref[:, 2] - gz > band[0]) & (ref[:, 2] - gz < band[1])]
    X, Y = ref[:, 0], ref[:, 1]; M = np.column_stack([X, Y, np.ones(len(X))]); sol = np.linalg.lstsq(M, X**2 + Y**2, rcond=None)[0]
    c = np.array([sol[0] / 2, sol[1] / 2]); Rfit = np.sqrt(sol[2] + c @ c)
    print(f"\n== pole {k} ({cx:.1f},{cy:.1f}) R={Rfit*100:.1f} cm; travel-frame centroid offsets of the NEAR FACE points (|d-axis|<0.5 m, band {band}); axis from front_right<=15 m")
    print("source           range  frames  speed m/s | centroid fwd: mean  sd | right: mean  sd | LOS-corrected (offset minus expected near-face shift -0.8R*u): fwd mean | right mean")
    for s_id in range(4):
        for lo, hi in ((3, 10), (10, 15), (15, 25), (25, 40)):
            F, Rt, Fc, Rc, Sp = [], [], [], [], []
            for _, i, w, s, rg in R_:
                m = (s == s_id) & (rg >= lo) & (rg < hi) & (w[:, 2] - gz > band[0]) & (w[:, 2] - gz < band[1]) & (np.hypot(w[:, 0] - c[0], w[:, 1] - c[1]) < 0.5)
                if m.sum() < 4: continue
                d = w[m, :2].mean(0) - c
                u = c - origins[i, :2]; u /= np.linalg.norm(u)
                exp = -0.8 * Rfit * u  # expected centroid of a sampled near half-arc
                fd = fwd_dir[i]; rt = np.array([fd[1], -fd[0]])
                F.append(d @ fd); Rt.append(d @ rt); Fc.append((d - exp) @ fd); Rc.append((d - exp) @ rt); Sp.append(speed[i])
            if len(F) < 3: continue
            F, Rt, Fc, Rc, Sp = map(np.array, (F, Rt, Fc, Rc, Sp))
            print(f"{names[s_id]:16s} {lo:2d}-{hi:2d} m {len(F):5d}  {Sp.mean():4.1f}     | {F.mean()*100:6.1f} {F.std()*100:4.1f} | {Rt.mean()*100:6.1f} {Rt.std()*100:4.1f} | {Fc.mean()*100:6.1f} | {Rc.mean()*100:6.1f}")
