#!/usr/bin/env python3
"""Find revisits in trajectory_optimized.txt: time pairs with |dt|>min_dt whose positions are within max_d."""
import sys, pathlib, numpy as np
run = pathlib.Path(sys.argv[1]); min_dt = float(sys.argv[2]); max_d = float(sys.argv[3])
fd = run if (run / "frames.csv").exists() else sorted((run / "Log" / "fusion").glob("*"))[-1]
rows = []
for line in open(fd / "trajectory_optimized.txt"):
    if line.startswith("#") or not line.strip(): continue
    p = line.split()
    try: rows.append([float(p[0]), float(p[1]), float(p[2]), float(p[3])])
    except ValueError: continue
a = np.array(rows); t = a[:, 0] - a[0, 0]; xy = a[:, 1:3]
print(f"{len(a)} poses, {t[-1]:.0f} s, path length {np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)):.0f} m")
# subsample to 1 Hz
idx = np.searchsorted(t, np.arange(0, t[-1], 1.0)); idx = np.unique(np.clip(idx, 0, len(t) - 1))
ts, ps = t[idx], xy[idx]
d = np.linalg.norm(ps[:, None, :] - ps[None, :, :], axis=2)
dt = np.abs(ts[:, None] - ts[None, :])
m = (d < max_d) & (dt > min_dt)
pairs = np.argwhere(np.triu(m))
if len(pairs) == 0:
    print("no revisits"); sys.exit()
# cluster by first time (10 s bins)
seen = {}
for i, j in pairs:
    key = (int(ts[i] // 20) * 20, int(ts[j] // 20) * 20)
    seen[key] = seen.get(key, 0) + 1
for (t1, t2), n in sorted(seen.items())[:40]:
    print(f"  revisit: t~{t1:4d}-{t1+20:4d} s and t~{t2:4d}-{t2+20:4d} s  ({n} pose pairs within {max_d} m)")
