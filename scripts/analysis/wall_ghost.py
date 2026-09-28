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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("t0", type=float); ap.add_argument("t1", type=float)
    ap.add_argument("--fused", action="store_true"); ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--min-range", type=float, default=3.0); ap.add_argument("--voxel", type=float, default=0.4)
    ap.add_argument("--step", type=int, default=1)
    ap.add_argument("--planarity", type=float, default=3.0)
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    if not (run / "frames.csv").exists():
        run = sorted((run / "Log" / "fusion").glob("*"))[-1]
    fr = px.load_frames(run); px.LEVER = fr["lever"]
    stamps = fr["stamp"]; counts = fr["count"].astype(np.int64); n = len(stamps); t_rel = stamps - stamps[0]
    nat = np.column_stack([stamps, fr["tx"], fr["ty"], fr["tz"], fr["qx"], fr["qy"], fr["qz"], fr["qw"]])
    poses = px.pose_rows(run / "trajectory_optimized.txt", stamps) if a.fused else nat
    names = px.source_names(run)
    Rp = Rotation.from_quat(poses[:, 4:8])
    vel = np.gradient(poses[:, 1:3], stamps, axis=0)
    W, S, F, RG, TD = [], [], [], [], []
    import io
    has_src = (run / "sources.bin").exists()
    if not has_src: print("NOTE: no sources.bin (run predates source tags) -> all returns treated as one source; inter-source offsets not available")
    with open(run / "scans.bin", "rb") as f, (open(run / "sources.bin", "rb") if has_src else io.BytesIO(b"")) as fsrc:
        for i in range(0, n, a.step):
            if not (a.t0 <= t_rel[i] <= a.t1):
                continue
            off, cnt = int(fr["offset_bytes"][i]), int(counts[i])
            f.seek(off); pts = np.frombuffer(f.read(cnt * 20), dtype="<f4").reshape(-1, 5).astype(np.float64)
            if has_src:
                fsrc.seek(8 + (off - 8) // 20); src = np.frombuffer(fsrc.read(cnt), dtype=np.uint8)
            else:
                src = np.zeros(cnt, dtype=np.uint8)
            rng = np.linalg.norm(pts[:, :3], axis=1)
            m = (rng > a.min_range) & (rng < a.max_range)
            w = Rp[i].apply(pts[m, :3] + px.LEVER) + poses[i, 1:4]
            W.append(w.astype(np.float32)); S.append(src[m]); F.append(np.full(m.sum(), i, np.int32)); RG.append(rng[m].astype(np.float32))
            d = vel[i] / max(np.linalg.norm(vel[i]), 1e-3); TD.append(np.tile(d.astype(np.float32), (m.sum(), 1)))
    W = np.concatenate(W); S = np.concatenate(S); F = np.concatenate(F); RG = np.concatenate(RG); TD = np.concatenate(TD)
    print(f"{run.name}: {len(W)} points from {len(np.unique(F))} frames, poses {'fused' if a.fused else 'native'}, range {a.min_range}-{a.max_range} m")
    key = np.floor(W / a.voxel).astype(np.int64)
    kk = (key[:, 0] * 73856093) ^ (key[:, 1] * 19349663) ^ (key[:, 2] * 83492791)
    order = np.argsort(kk, kind="stable"); kk = kk[order]
    bounds = np.flatnonzero(np.diff(kk)) + 1; starts = np.r_[0, bounds]; ends = np.r_[bounds, len(kk)]
    thick = {s: [] for s in range(len(names))}; thick_all = []
    offs = {s: [] for s in range(len(names))}  # (offset along normal vs front_lidar, |n.travel|, range)
    off_pairs = {}
    nvox = 0
    for st, en in zip(starts, ends):
        if en - st < 40:
            continue
        idx = order[st:en]
        if len(np.unique(F[idx])) < 4:
            continue
        P = W[idx].astype(np.float64); c = P.mean(0); Q = P - c
        cov = Q.T @ Q / len(P); ev, evec = np.linalg.eigh(cov)
        nrm = evec[:, 0]
        if abs(nrm[2]) > 0.35 or ev[1] < a.planarity * ev[0] or ev[1] < (0.08) ** 2:
            continue
        nvox += 1
        d = Q @ nrm
        thick_all.append(np.std(d))
        srcs = S[idx]
        means = {}
        for s in np.unique(srcs):
            ms = srcs == s
            if ms.sum() >= 15 and len(np.unique(F[idx][ms])) >= 3:
                thick[int(s)].append(np.std(d[ms])); means[int(s)] = (d[ms].mean(), np.median(RG[idx][ms]))
        ref = names.index("front_lidar")
        tdir = np.abs(np.median(TD[idx], 0) @ nrm[:2]) / max(np.linalg.norm(nrm[:2]), 1e-6)
        if ref in means:
            for s, (mu, rg) in means.items():
                if s != ref:
                    offs[s].append((mu - means[ref][0], tdir, rg, means[ref][1]))
        for s1 in means:
            for s2 in means:
                if s1 < s2:
                    off_pairs.setdefault((s1, s2), []).append(means[s2][0] - means[s1][0])
    ta = np.array(thick_all) * 100
    hist = [(ta < 2).mean(), ((ta >= 2) & (ta < 4)).mean(), ((ta >= 4) & (ta < 7)).mean(), ((ta >= 7) & (ta < 12)).mean(), (ta >= 12).mean()]
    print(f"vertical planar voxels (planarity>={a.planarity}): {nvox}; thickness (std along normal) all sources: median {np.median(ta):.1f} cm, P90 {np.percentile(ta,90):.1f} cm; share <2/2-4/4-7/7-12/>12 cm: " + "/".join(f"{h*100:.0f}%" for h in hist))
    for s in range(len(names)):
        if thick[s]:
            print(f"  intra-source thickness {names[s]:17s}: n={len(thick[s]):5d} median {np.median(thick[s])*100:.1f} cm P90 {np.percentile(thick[s],90)*100:.1f} cm")
    print("inter-source offset along the plane normal, source minus front_lidar (signed cm; |offset| stats):")
    for s in range(len(names)):
        if offs[s]:
            o = np.array(offs[s])
            print(f"  {names[s]:17s}: n={len(o):5d} signed median {np.median(o[:,0])*100:+.1f}, |off| median {np.median(np.abs(o[:,0]))*100:.1f} P90 {np.percentile(np.abs(o[:,0]),90)*100:.1f} cm")
            for lab, m in (("normal ~ along travel (|n.t|>0.7)", o[:, 1] > 0.7), ("normal ~ across travel (|n.t|<0.3)", o[:, 1] < 0.3)):
                if m.sum() >= 10:
                    print(f"      {lab:36s}: n={m.sum():5d} |off| median {np.median(np.abs(o[m,0]))*100:.1f} P90 {np.percentile(np.abs(o[m,0]),90)*100:.1f} cm (signed median {np.median(o[m,0])*100:+.1f})")
            for lo, hi in ((0, 10), (10, 17), (17, 25)):
                m = (o[:, 2] >= lo) & (o[:, 2] < hi)
                if m.sum() >= 10:
                    print(f"      source range {lo:2d}-{hi:2d} m: n={m.sum():5d} |off| median {np.median(np.abs(o[m,0]))*100:.1f} P90 {np.percentile(np.abs(o[m,0]),90)*100:.1f} cm")
    print("pairwise |offset| medians (cm):")
    for (s1, s2), v in sorted(off_pairs.items()):
        v = np.array(v)
        if len(v) >= 10:
            print(f"  {names[s1]:17s} vs {names[s2]:17s}: n={len(v):5d} |off| median {np.median(np.abs(v))*100:.1f} P90 {np.percentile(np.abs(v),90)*100:.1f} signed median {np.median(v)*100:+.1f}")


if __name__ == "__main__":
    main()
