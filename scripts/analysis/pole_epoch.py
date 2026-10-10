#!/usr/bin/env python3
"""Epoch-to-epoch consistency of one lamp pole (view-independent ring fit, per height slice).

For each 2 s epoch bin and each 1 m height slice of the straight shaft the returns on the pole
circumference are fitted with a fixed-radius circle (centre does not depend on which side of the
pole an epoch saw). Because the shaft leans (~0.06 m per m) each slice is compared only with
the SAME slice of the other epochs. Reported: spread of the ring centres over epochs
(RMS about the slice median, max pairwise) for the chosen trajectory file(s).
Usage: pole_epoch.py <round_dir> [--traj trajectory_lio.txt trajectory_optimized.txt] [--t0 4 --t1 28]
"""
import argparse, pathlib, sys
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from analysis.pole_xsection import load_frames, pose_rows  # noqa: E402
from evaluate_wuhu_lio import ring_fit  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--traj", nargs="+", default=["trajectory_lio.txt", "trajectory_optimized.txt"])
ap.add_argument("--centre", type=float, nargs=2, default=(-10.9, -273.55))
ap.add_argument("--zs", type=float, nargs=2, default=(44.5, 47.5))
ap.add_argument("--t0", type=float, default=0.0)
ap.add_argument("--t1", type=float, default=30.0)
ap.add_argument("--bin", type=float, default=2.0)
ap.add_argument("--radius", type=float, default=0.12)
ap.add_argument("--minpts", type=int, default=40)
ap.add_argument("-v", action="store_true")
ap.add_argument("--src", type=int, default=-1, help="restrict to one LiDAR source index")
a = ap.parse_args()
run = pathlib.Path(a.run)
if not (run / "frames.csv").exists():
    run = sorted((run / "Log" / "fusion").glob("*"))[-1]
fr = load_frames(run); lever = fr["lever"]; st = fr["stamp"]; cnt = fr["count"].astype(np.int64); t = st - st[0]
trs = {k: pose_rows(run / k, st) for k in a.traj}
raw = []
fsrc = open(run / "sources.bin", "rb")
with open(run / "scans.bin", "rb") as f:
    for i in range(len(st)):
        if t[i] < a.t0 or t[i] > a.t1: continue
        off, n = int(fr["offset_bytes"][i]), int(cnt[i]); f.seek(off)
        p = np.frombuffer(f.read(n * 20), "<f4").reshape(-1, 5).astype(np.float64)
        fsrc.seek(8 + (off - 8) // 20); srcs = np.frombuffer(fsrc.read(n), np.uint8)
        raw.append((i, p[:, :3] + lever, srcs))
slices = np.arange(a.zs[0], a.zs[1] - 1e-6, 1.0)
for name, tr in trs.items():
    rows = []
    for i, pl, sr in raw:
        if a.src >= 0:
            pl = pl[sr == a.src]
        w = Rotation.from_quat(tr[i, 4:8]).apply(pl) + tr[i, 1:4]
        m = (np.hypot(w[:, 0] - a.centre[0], w[:, 1] - a.centre[1]) < 0.7) & (w[:, 2] > a.zs[0]) & (w[:, 2] < a.zs[1])
        if m.any(): rows.append(np.column_stack([np.full(m.sum(), t[i]), w[m]]))
    D = np.concatenate(rows)
    C = {}  # (bin, slice) -> centre
    for b0 in np.arange(a.t0, a.t1, a.bin):
        for z0 in slices:
            S = D[(D[:, 0] >= b0) & (D[:, 0] < b0 + a.bin) & (D[:, 3] >= z0) & (D[:, 3] < z0 + 1)]
            if len(S) < a.minpts: continue
            xy = S[:, 1:3]; c0 = np.median(xy, 0); best = None
            for ang in range(0, 360, 45):
                r = ring_fit(xy, c0 + a.radius * np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))]), a.radius, band=0.04)
                if r and (best is None or r[2] > best[2]): best = r
            if best and best[3] >= 5 and best[1] < 0.035: C[(b0, z0)] = best[0]
    devs, pair = [], []
    for z0 in slices:
        ks = [k for k in C if k[1] == z0]
        if len(ks) < 3: continue
        P = np.array([C[k] for k in ks]); med = np.median(P, 0)
        d = np.linalg.norm(P - med, axis=1); devs += list(d)
        pair.append(max(np.linalg.norm(P[i] - P[j]) for i in range(len(P)) for j in range(i)))
        if a.v: print(f"   {name} z{z0:.1f}: n={len(ks)} " + " ".join(f"{k[0]:.0f}:({(C[k][0]-a.centre[0])*100:+.0f},{(C[k][1]-a.centre[1])*100:+.0f})" for k in ks))
    devs = np.array(devs) if devs else np.zeros(1)
    print(f"{name:26s} cells {len(devs):3d}  dev-from-median cm: rms {100*np.sqrt(np.mean(devs**2)):5.1f} P90 {100*np.percentile(devs,90):5.1f} max {100*devs.max():5.1f} | max pair/slice cm: " + " ".join(f"{100*p:.1f}" for p in pair))
