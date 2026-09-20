#!/usr/bin/env python3
"""Whole-run ground thickness from the ACTUAL saved map_*.pcd: per 1 m XY cell with enough
returns from >= 2 different chunks (i.e. observed at different times), the vertical spread
(P90-P10) of the lowest-surface points. Reported per time band of the run. Also usable on the
native reference per-frame PCDs. Usage: ground_thickness.py <run_dir> [<run_dir> ...]"""
import sys, pathlib, numpy as np
sys.path.insert(0, "/home/project/MY-LIVO/scripts")
from evaluate_wuhu_lio import read_pcd

def analyse(run):
    files = sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[1]))
    cells = {}  # key -> list of (chunk, z array)
    for f in files:
        k = int(f.stem.split("_")[1])
        xyz = np.asarray(read_pcd(f)[:, :3], dtype=np.float64)
        key = np.floor(xyz[:, :2]).astype(np.int64)
        kk = key[:, 0] * 1000003 + key[:, 1]
        order = np.argsort(kk, kind="stable"); kk = kk[order]; z = xyz[order, 2]
        uniq, start, count = np.unique(kk, return_index=True, return_counts=True)
        for u, s, c in zip(uniq, start, count):
            if c < 40: continue
            zc = z[s:s + c]
            # lowest surface: points within 0.5 m of the 5th percentile (rejects vehicles/berms above ground)
            lo = np.percentile(zc, 5); zc = zc[zc < lo + 0.5]
            if len(zc) < 30: continue
            cells.setdefault(u, []).append((k, zc))
    out = {}
    for u, lst in cells.items():
        if len(lst) < 2: continue
        chunks = [k for k, _ in lst]
        allz = np.concatenate([zc for _, zc in lst])
        spread = np.percentile(allz, 90) - np.percentile(allz, 10)
        # single-chunk spread (intra-chunk) for reference
        intra = np.median([np.percentile(zc, 90) - np.percentile(zc, 10) for _, zc in lst])
        out[u] = (min(chunks), max(chunks), spread, intra)
    return out

runs = [pathlib.Path(a) for a in sys.argv[1:]]
res = [analyse(r) for r in runs]
common = set.intersection(*[set(r) for r in res]) if len(res) > 1 else set(res[0])
print(f"multi-chunk ground cells common to all runs: {len(common)}")
bands = [(0, 10), (10, 20), (20, 30), (30, 40), (40, 50)]
print("chunk band (first chunk)   " + "   ".join(f"{r.parent.parent.parent.name[:28]:>28s}" for r in runs))
print("                           " + "   ".join(f"{'cells  P50/P90 spread(cm) | intra':>28s}" for _ in runs))
for lo, hi in bands:
    row = []
    for r in res:
        vals = np.array([[v[2], v[3]] for u, v in r.items() if u in common and lo <= v[0] < hi])
        if len(vals) == 0: row.append(f"{'-':>28s}"); continue
        row.append(f"{len(vals):5d} {100*np.median(vals[:,0]):6.1f}/{100*np.percentile(vals[:,0],90):6.1f} | {100*np.median(vals[:,1]):5.1f}")
    print(f"  map_{lo:02d}-map_{hi:02d}          " + "   ".join(row))
vals_all = [np.array([[v[2]] for u, v in r.items() if u in common]) for r in res]
print("  all                    " + "   ".join(f"{len(v):5d} {100*np.median(v):6.1f}/{100*np.percentile(v,90):6.1f}" for v in vals_all))

