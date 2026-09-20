#!/usr/bin/env python3
"""Viewpoint-independent multi-epoch consistency on the flat sign plate attached to the lamp pole
(z 48.3-49.7). Fit one line (XY) to all plate points, then report each epoch's median signed
perpendicular offset and spread; plate thickness across epochs = P90-P10 of the offsets.
Usage: sign_plane.py <run_dir> [<run_dir> ...]"""
import sys, pathlib, numpy as np
sys.path.insert(0, "/home/project/MY-LIVO/scripts")
from evaluate_wuhu_lio import read_pcd
cx0, cy0 = -10.7, -273.6
for run in [pathlib.Path(a) for a in sys.argv[1:]]:
    ep = {}
    for f in sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[1])):
        k = int(f.stem.split("_")[1])
        if k > 13: break
        xyz = np.asarray(read_pcd(f)[:, :3], dtype=float)
        m = (np.hypot(xyz[:, 0] - cx0, xyz[:, 1] - cy0) <= 0.9) & (xyz[:, 2] >= 48.3) & (xyz[:, 2] <= 49.7)
        # exclude the pole shaft itself (within 0.2 m of the pole axis)
        m &= np.hypot(xyz[:, 0] + 11.05, xyz[:, 1] + 273.5) > 0.25
        if m.sum() >= 30: ep[k] = xyz[m]
    allp = np.vstack(list(ep.values()))
    # robust line fit via PCA with iterative trimming
    P = allp[:, :2]
    for _ in range(4):
        c = P.mean(0); u, s, vt = np.linalg.svd(P - c, full_matrices=False); d = vt[0]; n = np.array([-d[1], d[0]])
        off = (P - c) @ n
        P = P[np.abs(off - np.median(off)) < 3 * (1.4826 * np.median(np.abs(off - np.median(off))) + 0.01)]
    c = P.mean(0); u, s, vt = np.linalg.svd(P - c, full_matrices=False); d = vt[0]; n = np.array([-d[1], d[0]])
    print(f"\n=== {run.parent.parent.parent.name}: plate points {len(allp)}, line dir=({d[0]:.3f},{d[1]:.3f}) normal=({n[0]:.3f},{n[1]:.3f})")
    rows = []
    for k in sorted(ep):
        off = (ep[k][:, :2] - c) @ n
        along = (ep[k][:, :2] - c) @ d
        rows.append((k, len(off), np.median(off), np.percentile(off, 90) - np.percentile(off, 10), along.min(), along.max()))
        print(f"  map_{k:<3d} n={len(off):4d}  median offset {100*np.median(off):+6.1f} cm  intra spread(P90-P10) {100*(np.percentile(off,90)-np.percentile(off,10)):5.1f} cm  along {along.min():+.2f}..{along.max():+.2f} m")
    med = np.array([r[2] for r in rows if r[1] >= 60])
    alloff = np.concatenate([(ep[k][:, :2] - c) @ n for k in ep])
    print(f"  epoch median offsets (n>=60): spread max-min {100*(med.max()-med.min()):.1f} cm, std {100*med.std():.1f} cm; whole-plate thickness P90-P10 {100*(np.percentile(alloff,90)-np.percentile(alloff,10)):.1f} cm, P95-P5 {100*(np.percentile(alloff,95)-np.percentile(alloff,5)):.1f} cm")

