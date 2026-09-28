#!/usr/bin/env python3
"""Ghosting metric on vertical planar structure (berms / walls): per 0.4 m voxel PCA of the merged map.

For planar, near-vertical voxels reports
  * intra-source thickness (std along the normal) for each source alone  -> pose / deskew / sensor noise
  * inter-source offset along the normal (mean of source A minus mean of the front LiDAR) -> calibration ghosts
  * the same split by observation range and by the travel-direction component of the normal.
Poses: native (frames.csv) or fused (trajectory_optimized.txt).
Usage: wall_ghost.py <run> <t0> <t1> [--fused] [--max-range 25]
"""
import argparse, pathlib, sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import pole_xsection as px  # noqa: E402



def load_window(run, fr, poses, Rp, t_rel, counts, t0, t1, min_range, max_range, step=1):
    W, F, RG = [], [], []
    with open(run / "scans.bin", "rb") as f:
        for i in range(0, len(t_rel), step):
            if not (t0 <= t_rel[i] <= t1):
                continue
            off, cnt = int(fr["offset_bytes"][i]), int(counts[i])
            f.seek(off); pts = np.frombuffer(f.read(cnt * 20), dtype="<f4").reshape(-1, 5).astype(np.float64)
            rng = np.linalg.norm(pts[:, :3], axis=1)
            m = (rng > min_range) & (rng < max_range)
            w = Rp[i].apply(pts[m, :3] + px.LEVER) + poses[i, 1:4]
            W.append(w.astype(np.float32)); F.append(np.full(m.sum(), i, np.int32)); RG.append(rng[m].astype(np.float32))
    return np.concatenate(W), np.concatenate(F), np.concatenate(RG)


def main():
    """Multi-pass ghost test: signed plane offset between two time windows (passes) in shared voxels."""
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("a0", type=float); ap.add_argument("a1", type=float)
    ap.add_argument("b0", type=float); ap.add_argument("b1", type=float)
    ap.add_argument("--fused", action="store_true"); ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--min-range", type=float, default=3.0); ap.add_argument("--voxel", type=float, default=0.4)
    ap.add_argument("--planarity", type=float, default=2.0); ap.add_argument("--step", type=int, default=1)
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    if not (run / "frames.csv").exists():
        run = sorted((run / "Log" / "fusion").glob("*"))[-1]
    fr = px.load_frames(run); px.LEVER = fr["lever"]
    stamps = fr["stamp"]; counts = fr["count"].astype(np.int64); t_rel = stamps - stamps[0]
    nat = np.column_stack([stamps, fr["tx"], fr["ty"], fr["tz"], fr["qx"], fr["qy"], fr["qz"], fr["qw"]])
    poses = px.pose_rows(run / "trajectory_optimized.txt", stamps) if a.fused else nat
    Rp = Rotation.from_quat(poses[:, 4:8])
    WA, FA, RA = load_window(run, fr, poses, Rp, t_rel, counts, a.a0, a.a1, a.min_range, a.max_range, a.step)
    WB, FB, RB = load_window(run, fr, poses, Rp, t_rel, counts, a.b0, a.b1, a.min_range, a.max_range, a.step)
    print(f"{run.name}: pass A {a.a0:.0f}-{a.a1:.0f} s {len(WA)} pts, pass B {a.b0:.0f}-{a.b1:.0f} s {len(WB)} pts, poses {'fused' if a.fused else 'native'}, range {a.min_range}-{a.max_range} m")
    W = np.concatenate([WA, WB]); P = np.r_[np.zeros(len(WA), np.int8), np.ones(len(WB), np.int8)]; F = np.concatenate([FA, FB])
    key = np.floor(W / a.voxel).astype(np.int64)
    kk = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
    order = np.argsort(kk, kind="stable"); kk = kk[order]
    bounds = np.flatnonzero(np.diff(kk)) + 1; starts = np.r_[0, bounds]; ends = np.r_[bounds, len(kk)]
    offs, thickA, thickB, thickAll, vert = [], [], [], [], []
    for st, en in zip(starts, ends):
        if en - st < 40: continue
        idx = order[st:en]; p = P[idx]
        if (p == 0).sum() < 15 or (p == 1).sum() < 15: continue
        if len(np.unique(F[idx][p == 0])) < 3 or len(np.unique(F[idx][p == 1])) < 3: continue
        pts = W[idx].astype(np.float64); c = pts.mean(0); q = pts - c
        ev, evec = np.linalg.eigh(q.T @ q / len(q))
        nrm = evec[:, 0]
        if abs(nrm[2]) > 0.35 or ev[1] < a.planarity * ev[0] or ev[1] < 0.08 ** 2:
            continue  # same voxel acceptance as wall_ghost.py (near-vertical planar patches)
        dA = q[p == 0] @ nrm; dB = q[p == 1] @ nrm
        offs.append(np.median(dB) - np.median(dA)); thickA.append(dA.std()); thickB.append(dB.std()); thickAll.append((q @ nrm).std())
    offs = np.abs(np.array(offs)) * 100; thickA = np.array(thickA) * 100; thickB = np.array(thickB) * 100; thickAll = np.array(thickAll) * 100
    if len(offs) == 0:
        print("no shared planar voxels"); return
    print(f"shared vertical planar voxels: {len(offs)}")
    print(f"  pass-to-pass |offset| along normal: median {np.median(offs):.1f} P90 {np.percentile(offs, 90):.1f} P99 {np.percentile(offs, 99):.1f} cm; share >5/>10/>20 cm: {np.mean(offs > 5) * 100:.0f}%/{np.mean(offs > 10) * 100:.0f}%/{np.mean(offs > 20) * 100:.0f}%")
    print(f"  thickness within pass A / pass B / both: median {np.median(thickA):.1f} / {np.median(thickB):.1f} / {np.median(thickAll):.1f} cm; P90 {np.percentile(thickA, 90):.1f} / {np.percentile(thickB, 90):.1f} / {np.percentile(thickAll, 90):.1f}")


if __name__ == "__main__":
    main()
