#!/usr/bin/env python3
"""Per-LiDAR / per-time ring fit of one lamp pole from the archived scans of a round.

Streams scans.bin (+ sources.bin), transforms the returns near the pole with the NATIVE or the
OPTIMIZED trajectory, and fits a fixed-radius circle (view-independent centre) to the shaft
returns of every (time bin, source). Shows whether the sources/epochs agree and what the
optimized correction does to them.

Usage: pole_source_ring.py <round_dir> [--bin 2.0] [--radius 0.12] [--t0 0 --t1 40]
"""
import argparse, pathlib, sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from analysis.pole_xsection import load_frames, pose_rows  # noqa: E402
from evaluate_wuhu_lio import ring_fit  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--bin", type=float, default=2.0)
ap.add_argument("--radius", type=float, default=0.12)
ap.add_argument("--t0", type=float, default=0.0)
ap.add_argument("--t1", type=float, default=40.0)
ap.add_argument("--centre", type=float, nargs=2, default=(-10.75, -273.65))
ap.add_argument("--z", type=float, nargs=2, default=(45.5, 48.5))
ap.add_argument("--traj", default="trajectory_optimized.txt")
a = ap.parse_args()
run = pathlib.Path(a.run)
if not (run / "frames.csv").exists():
    run = sorted((run / "Log" / "fusion").glob("*"))[-1]
fr = load_frames(run)
lever = fr["lever"]
st, cnt = fr["stamp"], fr["count"].astype(np.int64)
t = st - st[0]
tr = pose_rows(run / a.traj, st)
names = []
try:
    import yaml
    cfg = yaml.safe_load((run.parents[2] / "config.yaml").read_text())
    names = [x.strip("/") for x in cfg["laser_mapping"]["ros__parameters"]["multi_lidar"]["topics"]]
except Exception:
    pass
cx, cy = a.centre
rows = []
with open(run / "scans.bin", "rb") as f, open(run / "sources.bin", "rb") as fs:
    for i in range(len(st)):
        if t[i] < a.t0 or t[i] > a.t1:
            continue
        off, n = int(fr["offset_bytes"][i]), int(cnt[i])
        f.seek(off); p = np.frombuffer(f.read(n * 20), "<f4").reshape(-1, 5).astype(np.float64)
        fs.seek(8 + (off - 8) // 20); s = np.frombuffer(fs.read(n), np.uint8)
        R = Rotation.from_quat(tr[i, 4:8])
        w = R.apply(p[:, :3] + lever) + tr[i, 1:4]
        m = (np.hypot(w[:, 0] - cx, w[:, 1] - cy) < 0.6) & (w[:, 2] > a.z[0]) & (w[:, 2] < a.z[1])
        if m.any():
            rows.append(np.column_stack([np.full(m.sum(), t[i]), w[m], s[m], np.linalg.norm(p[m, :3], axis=1), p[m, 4]]))
D = np.concatenate(rows)
print(f"{run.parent.parent.parent.name} {a.traj}: {len(D)} shaft returns, t {D[:,0].min():.1f}-{D[:,0].max():.1f}")
print(f"sources: {names}")
bins = np.arange(a.t0, a.t1 + a.bin, a.bin)
print("bin   src    n  range   centre_x  centre_y  rms(cm) cov")
res = {}
for b0 in bins[:-1]:
    sel = (D[:, 0] >= b0) & (D[:, 0] < b0 + a.bin)
    for src in [-1] + sorted(np.unique(D[sel, 4]).astype(int)):
        m = sel if src < 0 else sel & (D[:, 4] == src)
        if m.sum() < 25:
            continue
        xy = D[m, 1:3]
        c0 = np.median(xy, 0)
        best = None
        for ang in range(0, 360, 45):  # multistart on the ring: centre = near + r*dir
            init = c0 + a.radius * np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))])
            r = ring_fit(xy, init, a.radius, band=0.04)
            if r and (best is None or r[2] > best[2] or (r[2] == best[2] and r[1] < best[1])):
                best = r
        if best is None:
            continue
        res[(b0, src)] = best
        nm = "all" if src < 0 else (names[src] if src < len(names) else str(src))
        print(f"{b0:5.1f} {nm:>16s} {int(m.sum()):4d} {D[m,5].mean():5.1f}  {best[0][0]:9.3f} {best[0][1]:9.3f}  {best[1]*100:5.1f} {best[3]:2d}")
