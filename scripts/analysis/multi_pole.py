#!/usr/bin/env python3
"""Multi-landmark pole consistency across export epochs (chunks), from the ACTUAL saved map.
1) find pole-like structures (thin, >=3.5 m tall, seen in >=6 chunks);
2) per pole and epoch, fit the shaft axis XY at a reference height from 0.5 m z-bin medians;
3) report separations between epochs whose vehicle->pole bearings differ by < 35 deg (limits
   the visible-arc/viewpoint bias of a thin cylinder), consecutive-epoch separations, and the
   per-pole spread. Usage: multi_pole.py <run_dir> [<run_dir> ...]"""
import sys, pathlib, numpy as np
sys.path.insert(0, "/home/project/MY-LIVO/scripts")
from evaluate_wuhu_lio import read_pcd
CH = 1000003

def decode(u):
    ix = u // CH; iy = u - ix * CH
    if iy > CH // 2: iy -= CH; ix += 1
    return ix * 0.5 + 0.25, iy * 0.5 + 0.25

def analyse(run, kmax=49):
    opt = np.loadtxt(run / "trajectory_optimized.txt")
    files = sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[1]))
    clouds = {}
    for f in files:
        k = int(f.stem.split("_")[1])
        if k > kmax: break
        clouds[k] = np.asarray(read_pcd(f)[:, :3], dtype=np.float32)
    # chunk -> vehicle position (median of frames in chunk: 1M points per chunk ~ frames in order)
    npts = np.array([len(clouds[k]) for k in sorted(clouds)])
    # map frames to chunks by cumulative point counts from frames.csv
    rows = np.array([[float(v) for v in l.split(",")] for l in (run / "frames.csv").read_text().splitlines()[4:]])
    chunk_of = []; size = 0; k = 0
    for c in rows[:, 3]:
        chunk_of.append(k); size += int(c)
        if size >= 1000000: k += 1; size = 0
    chunk_of = np.array(chunk_of)
    veh = {k: np.median(opt[chunk_of == k, 1:3], axis=0) for k in np.unique(chunk_of) if (chunk_of == k).sum() > 0}
    cells = {}
    for k, xyz in clouds.items():
        key = np.floor(xyz[:, :2] / 0.5).astype(np.int64); kk = key[:, 0] * CH + key[:, 1]
        order = np.argsort(kk, kind="stable"); kk = kk[order]; z = xyz[order, 2]
        uniq, start, count = np.unique(kk, return_index=True, return_counts=True)
        for u, s, c in zip(uniq, start, count):
            if c < 15: continue
            zc = z[s:s + c]; cells.setdefault(u, []).append((k, float(zc.min()), float(zc.max()), int(c)))
    cand = []
    for u, lst in cells.items():
        if len(lst) < 6: continue
        zmin = np.median([l[1] for l in lst]); zmax = np.median([l[2] for l in lst]); n = sum(l[3] for l in lst)
        if zmax - zmin >= 3.5 and n / (zmax - zmin) < 400:
            x, y = decode(u); cand.append((x, y, zmin, zmax, len(lst)))
    cand.sort(key=lambda c: -c[4]); poles = []
    for c in cand:
        if all(np.hypot(c[0] - m[0], c[1] - m[1]) > 2.0 for m in poles): poles.append(c)
    results = []
    for (px, py, zmin, zmax, nch) in poles[:12]:
        z_lo, z_hi = zmin + 1.0, min(zmin + 4.0, zmax - 0.5)
        if z_hi - z_lo < 1.5: continue
        zref = 0.5 * (z_lo + z_hi)
        axes = {}
        for k, xyz in clouds.items():
            m = (np.abs(xyz[:, 0] - px) < 1.0) & (np.abs(xyz[:, 1] - py) < 1.0) & (xyz[:, 2] >= z_lo) & (xyz[:, 2] <= z_hi)
            if m.sum() < 25: continue
            P = xyz[m].astype(float)
            # tighten around the densest XY (0.3 m) to drop arms/wires
            c0 = np.median(P[:, :2], 0); P = P[np.hypot(P[:, 0] - c0[0], P[:, 1] - c0[1]) < 0.35]
            if len(P) < 25: continue
            cents = []
            for lo in np.arange(z_lo, z_hi, 0.5):
                b = P[(P[:, 2] >= lo) & (P[:, 2] < lo + 0.5)]
                if len(b) >= 4: cents.append([np.median(b[:, 0]), np.median(b[:, 1]), lo + 0.25])
            if len(cents) < 3: continue
            cents = np.array(cents); mean = cents.mean(0); u, s, vt = np.linalg.svd(cents - mean); ax = vt[0]
            if abs(ax[2]) < 0.9: continue
            if ax[2] < 0: ax = -ax
            ref = mean + ax * (zref - mean[2]) / ax[2]
            if k in veh:
                axes[k] = (ref[:2], np.degrees(np.arctan2(py - veh[k][1], px - veh[k][0])), len(P), np.hypot(px - veh[k][0], py - veh[k][1]))
        keys = sorted(axes)
        if len(keys) < 4: continue
        sim, cons = [], []
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                a, b = axes[keys[i]], axes[keys[j]]
                dbear = abs((a[1] - b[1] + 180) % 360 - 180)
                sep = 100 * np.linalg.norm(a[0] - b[0])
                if dbear < 35 and a[3] < 45 and b[3] < 45: sim.append(sep)
                if keys[j] == keys[i] + 1: cons.append(sep)
        allxy = np.array([axes[k][0] for k in keys]); spread = 100 * np.percentile(np.linalg.norm(allxy - np.median(allxy, 0), axis=1), 90)
        results.append(dict(pole=(px, py), z=(zmin, zmax), epochs=len(keys), sim=np.median(sim) if sim else np.nan, nsim=len(sim), cons=np.median(cons) if cons else np.nan, spread=spread))
    return results

for run in [pathlib.Path(a) for a in sys.argv[1:]]:
    res = analyse(run)
    print(f"\n=== {run.parent.parent.parent.name}: {len(res)} poles")
    for r in res:
        print(f"  pole ({r['pole'][0]:7.2f},{r['pole'][1]:8.2f}) z {r['z'][0]:.1f}-{r['z'][1]:.1f} epochs={r['epochs']:2d} | similar-bearing sep median {r['sim']:5.1f} cm (n={r['nsim']:2d}) | consecutive {r['cons']:5.1f} cm | axis P90 spread {r['spread']:5.1f} cm")
    sim = [r['sim'] for r in res if np.isfinite(r['sim'])]; cons = [r['cons'] for r in res if np.isfinite(r['cons'])]; spr = [r['spread'] for r in res]
    print(f"  AGGREGATE over poles: similar-bearing sep median {np.median(sim):.1f} cm | consecutive-epoch sep median {np.median(cons):.1f} cm | axis P90 spread median {np.median(spr):.1f} cm")

