#!/usr/bin/env python3
"""Viewpoint-independent pole consistency: per export chunk (epoch), fit a circle of shared radius
to the shaft cross-section (z slices) and compare the fitted centres across epochs.
Usage: pole_circle.py <run_dir> [<run_dir> ...]"""
import sys, pathlib, numpy as np
sys.path.insert(0, "/home/project/MY-LIVO/scripts")
from evaluate_wuhu_lio import read_pcd
from scipy.optimize import least_squares
cx0, cy0, r0 = -11.05, -273.55, 0.8
SLICES = [(45.0, 46.0), (46.0, 47.0), (47.0, 48.0)]

def load(run):
    ep = {}
    for f in sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[1])):
        k = int(f.stem.split("_")[1])
        if k > 13: break
        xyz = np.asarray(read_pcd(f)[:, :3], dtype=float)
        m = (np.hypot(xyz[:, 0] - cx0, xyz[:, 1] - cy0) <= r0) & (xyz[:, 2] >= 44.5) & (xyz[:, 2] <= 48.5)
        ep[k] = xyz[m]
    return ep

def fit_free(P):
    def res(p): return np.hypot(P[:, 0] - p[0], P[:, 1] - p[1]) - p[2]
    c = np.median(P[:, :2], 0)
    s = least_squares(res, [c[0], c[1], 0.1], loss="soft_l1", f_scale=0.03)
    return s.x, np.sqrt(np.mean(s.fun ** 2))

def fit_fixed(P, r, init):
    def res(p): return np.hypot(P[:, 0] - p[0], P[:, 1] - p[1]) - r
    s = least_squares(res, init, loss="soft_l1", f_scale=0.03)
    return s.x, np.sqrt(np.mean(s.fun ** 2))

for run in [pathlib.Path(a) for a in sys.argv[1:]]:
    ep = load(run)
    print(f"\n=== {run.parent.parent.parent.name}")
    # shared radius from the densest epochs (>=300 points in slice)
    radii = []
    for k, P in ep.items():
        for zlo, zhi in SLICES:
            S = P[(P[:, 2] >= zlo) & (P[:, 2] < zhi)]
            if len(S) >= 250:
                x, rms = fit_free(S); radii.append(x[2])
                print(f"  free fit map_{k} z[{zlo},{zhi}] n={len(S)} centre=({x[0]:.3f},{x[1]:.3f}) radius={100*x[2]:.1f} cm rms={100*rms:.1f} cm")
    r = float(np.median(radii)) if radii else 0.1
    print(f"  shared radius = {100*r:.1f} cm (median of {len(radii)} free fits)")
    centres = {}
    for k in sorted(ep):
        P = ep[k]; cs = []
        for zlo, zhi in SLICES:
            S = P[(P[:, 2] >= zlo) & (P[:, 2] < zhi)]
            if len(S) < 40: continue
            c = np.median(S[:, :2], 0)
            # initial centre: push the arc centroid away from the viewer by ~0.64 r; unknown viewer -> start at centroid and let the fit move it
            x, rms = fit_fixed(S, r, c)
            cs.append((zlo, x[0], x[1], len(S), rms))
        if cs: centres[k] = cs
    print("  epoch  slice  n   centre x,y      rms(cm)")
    for k, cs in centres.items():
        for zlo, x, y, n, rms in cs:
            print(f"  map_{k:<3d} {zlo:5.1f} {n:4d}  ({x:8.3f},{y:8.3f})  {100*rms:5.1f}")
    # pairwise centre separation per slice, summarised
    keys = sorted(centres)
    seps = []
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            a = {c[0]: c for c in centres[keys[i]]}; b = {c[0]: c for c in centres[keys[j]]}
            common = set(a) & set(b)
            if not common: continue
            d = np.median([100 * np.hypot(a[z][1] - b[z][1], a[z][2] - b[z][2]) for z in common])
            seps.append((keys[i], keys[j], d))
    if seps:
        arr = np.array([s[2] for s in seps])
        print(f"  fixed-radius centre separation over {len(seps)} epoch pairs: median {np.median(arr):.1f} cm, P90 {np.percentile(arr,90):.1f}, max {arr.max():.1f}")
        # consecutive epochs and the 0v10 / 5v10 style pairs
        cons = [s[2] for s in seps if s[1] == s[0] + 1]
        print(f"  consecutive epochs: median {np.median(cons):.1f} cm; " + " ".join(f"{a}v{b}:{d:.1f}" for a, b, d in seps if (a, b) in [(0, 10), (5, 10), (0, 5), (6, 10), (7, 11), (2, 8)]))

