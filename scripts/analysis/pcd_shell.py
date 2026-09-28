#!/usr/bin/env python3
"""Pole shell residual straight from saved map_N.pcd of one or two runs (band above local ground).
Free-radius Kasa circle fit on points within 25 cm of the median centre; residual |r-R| for all points within 45 cm.
Also writes side-by-side cross-sections (img/pcd_shell.png)."""
import sys, re, pathlib, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
runs = [pathlib.Path(a) for a in sys.argv[1].split(",")]; out = pathlib.Path(sys.argv[2])
lo, hi = float(sys.argv[3]), float(sys.argv[4])
poles = {1: (25.1, -329.2), 7: (-28.5, -253.8), 3: (65.3, -355.0), 4: (114.3, -385.5), 5: (193.7, -436.3)}
def fusion_dir(run):
    return run if (run / "frames.csv").exists() else sorted((run / "Log" / "fusion").glob("*"))[-1]
def read_pcd(fn):
    with open(fn, "rb") as f:
        hdr = b""
        while True:
            line = f.readline(); hdr += line
            if line.startswith(b"DATA"): break
        npts = int(re.search(rb"POINTS (\d+)", hdr).group(1)); nf = len(re.search(rb"FIELDS (.*)", hdr).group(1).split())
        return np.frombuffer(f.read(npts * 4 * nf), dtype="<f4").reshape(npts, nf)
def kasa(xy):
    A = np.column_stack([2 * xy, np.ones(len(xy))]); b = (xy ** 2).sum(1)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]; c = sol[:2]; R = np.sqrt(max(sol[2] + c @ c, 1e-9)); return c, R
fig, axes = plt.subplots(len(runs), len(poles), figsize=(4.2 * len(poles), 4.2 * len(runs)), squeeze=False)
for ri, run in enumerate(runs):
    d = fusion_dir(run); near = {k: [] for k in poles}; total = 0
    for fn in sorted(d.glob("map_*.pcd")):
        a = read_pcd(fn); total += len(a)
        for k, (cx, cy) in poles.items():
            m = (np.abs(a[:, 0] - cx) < 1.5) & (np.abs(a[:, 1] - cy) < 1.5)
            if m.any(): near[k].append(a[m])
    print(f"== {run.name}: {total} map points")
    for j, (k, parts) in enumerate(near.items()):
        A = np.concatenate(parts); gz = np.percentile(A[:, 2], 3); h = A[:, 2] - gz
        B = A[(h > lo) & (h < hi)]
        c0 = np.median(B[np.hypot(B[:, 0] - poles[k][0], B[:, 1] - poles[k][1]) < 0.4, :2], 0)
        core = B[np.hypot(B[:, 0] - c0[0], B[:, 1] - c0[1]) < 0.25]
        c, R = kasa(core[:, :2])
        r = np.hypot(B[:, 0] - c[0], B[:, 1] - c[1]); sel = r < 0.45; res = np.abs(r[sel] - R)
        print(f"pole {k}: n={sel.sum():5d} R={R*100:5.1f} cm  |r-R| P50/P90/P99 {np.percentile(res,50)*100:4.1f}/{np.percentile(res,90)*100:4.1f}/{np.percentile(res,99)*100:4.1f} cm  share |r-R|>6 cm {np.mean(res>0.06)*100:4.1f}%  >10 cm {np.mean(res>0.10)*100:4.1f}%")
        ax = axes[ri, j]; ax.scatter(B[sel, 0] - c[0], B[sel, 1] - c[1], s=3, c="k"); ax.add_patch(plt.Circle((0, 0), R, fill=False, color="r", ls="--"))
        ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.45, 0.45); ax.set_aspect("equal"); ax.set_title(f"{run.name}\npole {k} band {lo}-{hi} m, R={R*100:.0f} cm", fontsize=9)
fig.tight_layout(); fig.savefig(out / "pcd_shell.png", dpi=70); print("wrote", out / "pcd_shell.png")
