#!/usr/bin/env python3
"""Startup lamp-pole ghost metric (truck29, pole at (-10.95, -273.55), seen from 0 to ~30 s).

View-independent: per 0.5 m slice of the clean shaft (z 44.5-47.5) ONE circle is fitted to the returns of
all epochs; every return gets a signed radial residual. An epoch whose poses are displaced by d shows a
median signed residual of about -d on the side it sees, so the spread (max - min) of the 2 s epoch medians
is the ghost size free of the one-sided-arc bias of per-epoch centre fits.

  startup_pole_shell.py <round dir | fusion run dir> [--traj trajectory_optimized.txt ...] [--sources]
  -> "SHELL <traj>: P90 x cm | epoch-offset spread (max-min) y cm, rms z cm"

Reference (2026-10-11): native 7.3 / 8.4 cm; v069_driven fused 8.4 / 11.1 cm; v071 fused 6.8 / 6.0 cm."""
import sys, pathlib, argparse, numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from analysis.pole_xsection import load_frames, pose_rows
ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("--centre", type=float, nargs=2, default=(-10.95, -273.55))
ap.add_argument("--z", type=float, nargs=2, default=(44.5, 47.5)); ap.add_argument("--t1", type=float, default=30.0)
ap.add_argument("--traj", nargs="+", default=["trajectory_lio.txt", "trajectory_optimized.txt"])
ap.add_argument("--sources", action="store_true", help="per-LiDAR epoch residuals (source ids: topic order)")
a = ap.parse_args()
run = pathlib.Path(a.run); run = sorted((run / "Log" / "fusion").glob("*"))[-1] if not (run / "frames.csv").exists() else run
CX, CY = a.centre
fr = load_frames(run); lever = fr["lever"]; st = fr["stamp"]; cnt = fr["count"].astype(np.int64); t = st - st[0]
trs = {k: pose_rows(run / k, st) for k in a.traj}
out = {k: [] for k in trs}
fsrc = open(run / "sources.bin", "rb")
with open(run / "scans.bin", "rb") as f:
    for i in range(len(st)):
        if t[i] > a.t1: break
        off = int(fr["offset_bytes"][i]); f.seek(off); n = int(cnt[i])
        fsrc.seek(8 + (off - 8) // 20); srcs = np.frombuffer(fsrc.read(n), np.uint8).astype(float)
        p = np.frombuffer(f.read(n * 20), "<f4").reshape(-1, 5).astype(np.float64)[:, :3] + lever
        for k, tr in trs.items():
            row = tr[i]
            w = R.from_quat(row[4:8]).apply(p) + row[1:4]
            m = (np.hypot(w[:, 0] - CX, w[:, 1] - CY) < 0.6) & (w[:, 2] > a.z[0]) & (w[:, 2] < a.z[1])
            if m.any():
                out[k].append(np.column_stack([np.full(m.sum(), t[i]), w[m], srcs[m]]))


def kasa(xy):
    A = np.column_stack([2 * xy, np.ones(len(xy))]); b = (xy ** 2).sum(1)
    s = np.linalg.lstsq(A, b, rcond=None)[0]; c = s[:2]
    return c, float(np.sqrt(max(s[2] + c @ c, 1e-9)))


for k, L in out.items():
    D = np.concatenate(L); T = D.T
    res = np.full(len(D), np.nan)
    for z0 in np.arange(a.z[0], a.z[1] - 1e-6, 0.5):
        m = (T[3] >= z0) & (T[3] < z0 + 0.5)
        if m.sum() < 50: continue
        xy = D[m][:, 1:3]
        c, r = kasa(xy); keep = np.ones(len(xy), bool)
        for _ in range(10):
            d = np.hypot(*(xy - c).T) - r
            keep = np.abs(d) < max(3 * 1.4826 * np.median(np.abs(d[keep])), 0.03)
            c, r = kasa(xy[keep])
        res[m] = np.hypot(*(xy - c).T) - r
    ok = np.isfinite(res) & (np.abs(res) < 0.3)
    ar = np.abs(res[ok])
    print(f"== {k}: n={ok.sum()} |r-R| P50/P90 {100*np.percentile(ar,50):.1f}/{100*np.percentile(ar,90):.1f} cm")
    meds = []
    line = []
    for b0 in np.arange(0, a.t1, 2.0):
        m = ok & (T[0] >= b0) & (T[0] < b0 + 2)
        if m.sum() < 60: continue
        md = float(np.median(res[m])); meds.append(md); line.append(f"{b0:.0f}s:{100*md:+.1f}")
    meds = np.array(meds)
    print("  epoch median signed residual (cm): " + " ".join(line))
    print(f"SHELL {k}: P90 {100*np.percentile(ar,90):.1f} cm | epoch-offset spread (max-min) {100*(meds.max()-meds.min()):.1f} cm, rms {100*meds.std():.1f} cm")

    for src in (sorted(set(T[4].astype(int))) if a.sources else []):
        line = []
        for b0 in np.arange(0, a.t1, 2.0):
            m = ok & (T[0] >= b0) & (T[0] < b0 + 2) & (T[4] == src)
            if m.sum() < 30: continue
            line.append(f"{b0:.0f}s:{100*np.median(res[m]):+.1f}({m.sum()})")
        print(f"  src {src}: " + " ".join(line))
