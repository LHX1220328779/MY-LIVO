#!/usr/bin/env python3
"""Inter-LiDAR translation offset of static structure, per time window.

Both sources are placed in the world with the SAME trajectory file, so any translation that
best registers source A to source B (point-to-plane, translation only, Huber) is an inter-sensor
inconsistency (extrinsic / per-source timing), not a trajectory error. The offset is reported
in world axes and in the vehicle frame (right, forward, up) together with the vehicle speed,
so a timing error (offset parallel to the velocity, proportional to speed) can be told from an
extrinsic error (fixed in the body frame).
Usage: src_offset.py <round> A B [--t0 6 --t1 34 --win 4 --traj trajectory_lio.txt --rmin 6 --rmax 35]
(A, B = source indices of multi_lidar.topics: 0 front_right, 1 back, 2 front_left, 3 front)
"""
import argparse, pathlib, sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from analysis.pole_xsection import load_frames, pose_rows  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("A", type=int); ap.add_argument("B", type=int)
ap.add_argument("--t0", type=float, default=6); ap.add_argument("--t1", type=float, default=34)
ap.add_argument("--win", type=float, default=4.0)
ap.add_argument("--traj", default="trajectory_lio.txt")
ap.add_argument("--rmin", type=float, default=6.0); ap.add_argument("--rmax", type=float, default=35.0)
ap.add_argument("--stride", type=int, default=2)
a = ap.parse_args()
run = pathlib.Path(a.run)
if not (run / "frames.csv").exists():
    run = sorted((run / "Log" / "fusion").glob("*"))[-1]
fr = load_frames(run); lever = fr["lever"]; st = fr["stamp"]; cnt = fr["count"].astype(np.int64); t = st - st[0]
tr = pose_rows(run / a.traj, st)

def planes(Bp, Q, k=12, rad=0.6):
    tree = cKDTree(Bp)
    d, idx = tree.query(Q, k=k, distance_upper_bound=rad)
    ok = np.isfinite(d[:, -1])
    Q, idx = Q[ok], idx[ok]
    N = Bp[idx]; c = N.mean(1); X = N - c[:, None, :]
    cov = np.einsum("nki,nkj->nij", X, X) / k
    w, v = np.linalg.eigh(cov)
    planar = (w[:, 0] < 0.02 ** 2) & (w[:, 1] > 6 * w[:, 0] + 1e-6) & (w[:, 1] > 0.05 ** 2)
    return Q[planar], c[planar], v[planar][:, :, 0]

def solve(Q, c, n, iters=4, huber=0.05):
    tt = np.zeros(3)
    for _ in range(iters):
        r = np.einsum("ni,ni->n", Q + tt - c, n)
        wgt = np.where(np.abs(r) < huber, 1.0, huber / np.abs(r))
        H = (n * wgt[:, None]).T @ n; g = (n * (wgt * r)[:, None]).sum(0)
        tt -= np.linalg.solve(H + 1e-6 * np.eye(3), g)
    r = np.einsum("ni,ni->n", Q + tt - c, n)
    H = n.T @ n
    return tt, r, np.linalg.eigvalsh(H)

print(f"{run.parent.parent.parent.name} {a.traj}  offset of source {a.A} relative to source {a.B} (A shifted by +t registers on B); world (E,N,U) and vehicle (right,fwd,up) in cm")
print(" t0   n_pairs  speed  heading |  world dE dN dU        | vehicle dR dF dU   | along-vel  cross-vel | rms  conditioning(min eig/N)")
fsrc = open(run / "sources.bin", "rb")
with open(run / "scans.bin", "rb") as f:
    for b0 in np.arange(a.t0, a.t1, a.win):
        PA, PB = [], []
        ids = [i for i in range(len(st)) if b0 <= t[i] < b0 + a.win]
        for i in ids:
            off, n = int(fr["offset_bytes"][i]), int(cnt[i]); f.seek(off)
            p = np.frombuffer(f.read(n * 20), "<f4").reshape(-1, 5).astype(np.float64)
            fsrc.seek(8 + (off - 8) // 20); s = np.frombuffer(fsrc.read(n), np.uint8)
            rng = np.linalg.norm(p[:, :3], axis=1)
            ok = (rng > a.rmin) & (rng < a.rmax)
            w = Rotation.from_quat(tr[i, 4:8]).apply(p[:, :3] + lever) + tr[i, 1:4]
            PA.append(w[ok & (s == a.A)][::a.stride]); PB.append(w[ok & (s == a.B)][::a.stride])
        PA, PB = np.concatenate(PA), np.concatenate(PB)
        if len(PA) < 2000 or len(PB) < 2000:
            print(f"{b0:5.1f}  too few points ({len(PA)}, {len(PB)})"); continue
        Q, c, nrm = planes(PB, PA)
        if len(Q) < 800:
            print(f"{b0:5.1f}  too few planar pairs ({len(Q)})"); continue
        tt, r, ev = solve(Q, c, nrm)
        i0, i1 = ids[0], ids[-1]
        vel = (tr[i1, 1:3] - tr[i0, 1:3]) / (t[i1] - t[i0]); sp = np.linalg.norm(vel); hd = np.arctan2(vel[1], vel[0])
        fwd = np.array([np.cos(hd), np.sin(hd)]); rgt = np.array([np.sin(hd), -np.cos(hd)])
        veh = np.array([tt[:2] @ rgt, tt[:2] @ fwd, tt[2]])
        cross = tt[:2] @ np.array([-fwd[1], fwd[0]])
        print(f"{b0:5.1f} {len(Q):7d} {sp:6.2f} {np.degrees(hd):7.1f} | {tt[0]*100:6.1f} {tt[1]*100:6.1f} {tt[2]*100:6.1f} | {veh[0]*100:6.1f} {veh[1]*100:6.1f} {veh[2]*100:6.1f} | {veh[1]*100:7.1f} {cross*100:7.1f} | {np.sqrt(np.mean(r**2))*100:4.1f}  {ev[0]/len(Q):.2f}")
