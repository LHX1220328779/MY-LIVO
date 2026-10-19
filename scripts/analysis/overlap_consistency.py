#!/usr/bin/env python3
"""Cross-pass consistency of the delivered map on revisited road sections.

For every revisit (a later scan within --radius m of an earlier one after >= --min-path m of driving) a local
cloud of each pass (scans within +-half m of path, projected exactly like the delivered map) is built and the
later pass is compared with the planes of the earlier pass: point-to-plane distance of every later-pass voxel
that lies on a planar patch of the earlier pass (<= 0.3 m away). Reported per class (ground: |n_z| > 0.9,
vertical: |n_z| < 0.3) as median / P90 in mm. This is the quantity a loop closure must reduce ("double" ground,
thick walls / poles where two passes overlap); the RTK error alone cannot show it.

    /usr/bin/python3 scripts/analysis/overlap_consistency.py results/<round> [--json out.json]
"""
import argparse
import json
import pathlib
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools" / "livo"))
import map_export as me  # noqa: E402


def local_cloud(scans, fr, lt, lr, corr, raw_R, raw_t, ids, max_range, voxel):
    pts = []
    for i in ids:
        cnt = int(fr["count"][i])
        scans.seek(int(fr["offset_bytes"][i]))
        a = np.frombuffer(scans.read(cnt * 20), dtype="<f4").reshape(cnt, 5)
        p = a[:, :3].astype(np.float64)
        rng = np.linalg.norm(p, axis=1)
        m = np.isfinite(rng) & (rng > 1.0) & (rng < max_range)
        p, off = p[m], a[m, 4].astype(np.float64)
        t = np.clip(fr["begin"][i] + off / 1000.0, fr["begin"][i], fr["stamp"][i])
        native = p @ (raw_R[i] @ lr).T + (raw_R[i] @ lt + raw_t[i])
        pts.append(me.apply_correction(native, t, corr))
    p = np.vstack(pts)
    key = np.floor(p / voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    out = np.zeros((len(cnt), 3))
    for k in range(3):
        out[:, k] = np.bincount(inv.ravel(), weights=p[:, k]) / cnt
    return out


def planes(cloud, k=12, max_radius=0.6):
    tree = cKDTree(cloud)
    d, idx = tree.query(cloud, k=k)
    ok = d[:, -1] < max_radius
    nb = cloud[idx]  # (N, k, 3)
    mu = nb.mean(1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb - mu, nb - mu) / k
    w, v = np.linalg.eigh(cov)
    planar = ok & (w[:, 0] < 0.02 * w.sum(1)) & (w[:, 1] > 0.1 * w[:, 2])
    return tree, v[:, :, 0], planar


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--radius", type=float, default=6.0)
    ap.add_argument("--min-path", type=float, default=60.0)
    ap.add_argument("--spacing", type=float, default=8.0, help="min path between evaluated revisits of a pass (m)")
    ap.add_argument("--half", type=float, default=10.0, help="local cloud extent along the path (m)")
    ap.add_argument("--scan-spacing", type=float, default=0.3)
    ap.add_argument("--max-range", type=float, default=30.0)
    ap.add_argument("--voxel", type=float, default=0.08)
    ap.add_argument("--max-pairs", type=int, default=60)
    ap.add_argument("--max-path", type=float, default=1e9,
                    help="max driven path between the two scans (with a small --min-path: same-pass noise floor)")
    ap.add_argument("--json")
    a = ap.parse_args()
    run = me.resolve_run(a.run)
    fr, lt, lr = me.read_frames(run)
    corr = me.read_corrections(run)
    raw_R = me.quat_to_rot(np.stack([fr["qx"], fr["qy"], fr["qz"], fr["qw"]], 1))
    raw_t = np.stack([fr["tx"], fr["ty"], fr["tz"]], 1)
    stamps = fr["stamp"]
    path = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(raw_t, axis=0), axis=1))]
    traj = np.loadtxt(run / "trajectory_optimized.txt")
    pos = traj[np.clip(np.searchsorted(traj[:, 0], stamps - 1e-6), 0, len(traj) - 1), 1:4]
    # revisit pairs (frame level), thinned along the later pass
    pairs, last = [], -1e18
    tree = cKDTree(pos[:, :2])
    for j in range(len(stamps)):
        if path[j] - last < a.spacing:
            continue
        cand = [i for i in tree.query_ball_point(pos[j, :2], a.radius)
                if a.min_path <= path[j] - path[i] <= a.max_path and abs(pos[j, 2] - pos[i, 2]) < 3.0]
        if cand:
            i = min(cand, key=lambda i: np.linalg.norm(pos[j, :2] - pos[i, :2]))
            pairs.append((i, j))
            last = path[j]
    if len(pairs) > a.max_pairs:
        pairs = [pairs[k * len(pairs) // a.max_pairs] for k in range(a.max_pairs)]

    def ids_around(c):
        sel = np.flatnonzero(np.abs(path - path[c]) <= a.half)
        keep, prev = [], -1e18
        for s in sel:
            if stamps[s] - prev >= a.scan_spacing:
                keep.append(s)
                prev = stamps[s]
        return keep

    res = {"ground": [], "vertical": [], "all": []}
    per_pair = []
    with open(run / "scans.bin", "rb") as scans:
        for i, j in pairs:
            A = local_cloud(scans, fr, lt, lr, corr, raw_R, raw_t, ids_around(i), a.max_range, a.voxel)
            B = local_cloud(scans, fr, lt, lr, corr, raw_R, raw_t, ids_around(j), a.max_range, a.voxel)
            tA, nA, planar = planes(A)
            d, idx = tA.query(B, k=1)
            m = (d < 0.3) & planar[idx]
            if m.sum() < 200:
                continue
            n = nA[idx[m]]
            dist = np.abs(np.einsum("ij,ij->i", n, B[m] - A[idx[m]]))
            nz = np.abs(n[:, 2])
            res["all"].append(dist)
            res["ground"].append(dist[nz > 0.9])
            res["vertical"].append(dist[nz < 0.3])
            per_pair.append({"a_stamp": float(stamps[i]), "b_stamp": float(stamps[j]),
                             "path_gap_m": round(float(path[j] - path[i]), 1),
                             "n": int(m.sum()), "median_mm": round(1000 * float(np.median(dist)), 2)})
    summary = {"run": str(run), "pairs": len(per_pair)}
    for k, parts in res.items():
        v = np.concatenate(parts) if parts else np.zeros(0)
        summary[k] = {"n": int(len(v)), "median_mm": round(1000 * float(np.median(v)), 2) if len(v) else None,
                      "p90_mm": round(1000 * float(np.percentile(v, 90)), 2) if len(v) else None}
    print(json.dumps(summary, ensure_ascii=False))
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps({"summary": summary, "pairs": per_pair}, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
