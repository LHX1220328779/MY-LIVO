#!/usr/bin/env python3
"""Along-track inter-source offset: planar near-vertical voxels whose normal is ALONG the travel direction
(sign faces, block ends, pole faces). Normal oriented to +travel; signed offset of each source's points along it,
relative to front_right (0).  Split by speed to separate a per-source time offset (offset ~ speed) from an
extrinsic translation (constant).  Usage: along_track.py <run> <t0> <t1> [max_range] [voxel]"""
import sys, pathlib, numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import pole_xsection as px
run = pathlib.Path(sys.argv[1]); t0, t1 = float(sys.argv[2]), float(sys.argv[3])
max_range = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0; vox = float(sys.argv[5]) if len(sys.argv) > 5 else 0.4
if not (run / "frames.csv").exists():
    run = sorted((run / "Log" / "fusion").glob("*"))[-1]
fr = px.load_frames(run); px.LEVER = fr["lever"]
stamps = fr["stamp"]; counts = fr["count"].astype(np.int64); n = len(stamps); t_rel = stamps - stamps[0]
poses = px.pose_rows(run / "trajectory_optimized.txt", stamps); names = px.source_names(run)
Rp = Rotation.from_quat(poses[:, 4:8]); vel = np.gradient(poses[:, 1:3], stamps, axis=0); speed = np.linalg.norm(vel, axis=1)
W, S, F = [], [], []
with open(run / "scans.bin", "rb") as f, open(run / "sources.bin", "rb") as fsrc:
    for i in range(n):
        if not (t0 <= t_rel[i] <= t1) or speed[i] < 0.3: continue
        off, cnt = int(fr["offset_bytes"][i]), int(counts[i])
        f.seek(off); pts = np.frombuffer(f.read(cnt * 20), dtype="<f4").reshape(-1, 5).astype(np.float64)
        fsrc.seek(8 + (off - 8) // 20); src = np.frombuffer(fsrc.read(cnt), dtype=np.uint8)
        rng = np.linalg.norm(pts[:, :3], axis=1); m = (rng > 3.0) & (rng < max_range)
        w = Rp[i].apply(pts[m, :3] + px.LEVER) + poses[i, 1:4]
        W.append(w.astype(np.float32)); S.append(src[m]); F.append(np.full(m.sum(), i, np.int32))
W = np.concatenate(W); S = np.concatenate(S); F = np.concatenate(F)
print(f"{run.name}: {len(W)} points, window {t0}-{t1} s, range 3-{max_range} m, voxel {vox} m")
key = np.floor(W / vox).astype(np.int64); kk = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
order = np.argsort(kk, kind="stable"); kk = kk[order]; bounds = np.flatnonzero(np.diff(kk)) + 1; starts = np.r_[0, bounds]; ends = np.r_[bounds, len(kk)]
rows = []  # source, signed offset along +travel vs front_right, speed, |n.t|, height of voxel above the IMU (m)
nvox = 0
for st, en in zip(starts, ends):
    if en - st < 25: continue
    idx = order[st:en]
    if len(np.unique(F[idx])) < 3: continue
    P = W[idx].astype(np.float64); c = P.mean(0); Q = P - c
    ev, evec = np.linalg.eigh(Q.T @ Q / len(P)); nrm = evec[:, 0]
    if abs(nrm[2]) > 0.35 or ev[1] < 2.0 * ev[0] or ev[1] < 0.08 ** 2: continue
    fi = F[idx]; t = np.median(vel[fi] / np.maximum(speed[fi], 1e-3)[:, None], 0); t /= np.linalg.norm(t)
    nt = nrm[:2] @ t / max(np.linalg.norm(nrm[:2]), 1e-6)
    if abs(nt) < 0.7: continue
    if nt < 0: nrm = -nrm
    nvox += 1
    d = Q @ nrm; srcs = S[idx]
    ms0 = srcs == 0
    if ms0.sum() < 8: continue
    ref = np.median(d[ms0]); sp = np.median(speed[fi])
    for s in (1, 2, 3):
        ms = srcs == s
        if ms.sum() >= 8 and len(np.unique(fi[ms])) >= 2:
            rows.append((s, np.median(d[ms]) - ref, sp, abs(nt), c[2] - np.median(poses[fi, 3])))
rows = np.array(rows)
print(f"along-travel-facing planar voxels: {nvox}; source-vs-front_right comparisons: {len(rows)}")
for s in (1, 2, 3):
    o = rows[rows[:, 0] == s]
    if len(o) < 5: continue
    print(f"{names[s]:17s}: n={len(o):4d} signed offset along +travel: median {np.median(o[:,1])*100:+5.1f} cm, IQR {np.percentile(o[:,1],25)*100:+5.1f}..{np.percentile(o[:,1],75)*100:+5.1f}")
    for lo, hi in ((0.3, 3.5), (3.5, 5.5), (5.5, 8.0)):
        m = (o[:, 2] >= lo) & (o[:, 2] < hi)
        if m.sum() >= 5:
            print(f"    speed {lo:3.1f}-{hi:3.1f} m/s (mean {o[m,2].mean():.1f}): n={m.sum():4d} median {np.median(o[m,1])*100:+5.1f} cm  IQR {np.percentile(o[m,1],25)*100:+5.1f}..{np.percentile(o[m,1],75)*100:+5.1f}  -> equivalent time offset {np.median(o[m,1])/o[m,2].mean()*1000:+5.1f} ms")
    for lo, hi in ((-3, -1.5), (-1.5, -0.5), (-0.5, 0.5), (0.5, 1.5), (1.5, 4)):
        m = (o[:, 4] >= lo) & (o[:, 4] < hi)
        if m.sum() >= 5:
            print(f"    height above IMU {lo:+4.1f}..{hi:+4.1f} m: n={m.sum():4d} median {np.median(o[m,1])*100:+5.1f} cm  IQR {np.percentile(o[m,1],25)*100:+5.1f}..{np.percentile(o[m,1],75)*100:+5.1f}")
    if len(o) > 30:
        A = np.column_stack([o[:, 4], np.ones(len(o))]); sl, ic = np.linalg.lstsq(A, o[:, 1], rcond=None)[0]
        print(f"    linear fit offset = {sl*100:+.2f} cm/m * height {ic*100:+.1f} cm  (pitch-error equivalent {np.degrees(sl):+.2f} deg)")
