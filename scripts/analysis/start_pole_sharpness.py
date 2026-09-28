#!/usr/bin/env python3
"""Lamp-pole sharpness of the start segment: native LIO map vs fused (RTK-corrected) map.

The user-facing symptom is "the pole looks thick / doubled in the fused PCDs although pure LIO maps
it razor-sharp at the start". This tool measures exactly that from the run's own archive, without
any external reference:

  * fused points   = the saved map_*.pcd (what the user opens in CloudCompare)
  * native points  = the same returns re-projected with the native LIO pose of their frame
                     (T_native_i * T_fused_i^-1 applied per frame, so LiDAR/IMU conventions cancel)
  * poles          = thin vertical clusters (>= 3 m tall, <= 1 m wide) auto-detected on the fused
                     cloud of the first --seconds of the run
  * per pole       - radial P90 about its own PCA axis, native vs fused (thickness)
                     - "ghosting": spread over 1-s time bins of (fused - native) centroid shift,
                       i.e. how much the fusion correction moved between the passes that saw the pole
Usage: python3 scripts/analysis/start_pole_sharpness.py <fusion_dir> [--seconds 30] [--max-poles 12]
"""
import argparse, pathlib, sys
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))
from evaluate_wuhu_lio import read_pcd  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from scipy import ndimage  # noqa: E402


def load_frames(run):
    rows = [l for l in (run / "frames.csv").read_text().splitlines() if l and not l.startswith("#")]
    head = rows[0].split(",")
    data = np.array([[float(v) for v in r.split(",")] for r in rows[1:]])
    return {k: data[:, i] for i, k in enumerate(head)}


def pose_rows(path, stamps):
    tr = np.loadtxt(path, ndmin=2)
    if len(tr) == len(stamps) and np.max(np.abs(tr[:, 0] - stamps)) < 2e-3:
        return tr
    # fall back to nearest-stamp lookup
    idx = np.clip(np.searchsorted(tr[:, 0], stamps), 0, len(tr) - 1)
    return tr[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--max-poles", type=int, default=12)
    ap.add_argument("--min-height", type=float, default=3.0)
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--shaft-band", type=float, nargs=2, default=(1.0, 4.0), help="height band above ground used for the thickness metric")
    ap.add_argument("--max-native-p90", type=float, default=0.20, help="reject clusters whose native shaft radial P90 exceeds this (not a pole)")
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    if not (run / "frames.csv").exists():
        cands = sorted((run / "Log" / "fusion").glob("*"))
        if not cands:
            sys.exit(f"no fusion dir under {run}")
        run = cands[-1]
    fr = load_frames(run)
    stamps, counts = fr["stamp"], fr["count"].astype(np.int64)
    t_rel = stamps - stamps[0]
    n_frames = int(np.searchsorted(t_rel, a.seconds))
    offsets = np.r_[0, np.cumsum(counts)]
    need = offsets[n_frames]
    nat = pose_rows(run / "trajectory_lio.txt", stamps)
    fus = pose_rows(run / "trajectory_optimized.txt", stamps)
    # fused world points of the first n_frames frames from the saved chunks
    chunks = []
    got = 0
    for p in sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[-1])):
        c = np.asarray(read_pcd(p)[:, :3], dtype=np.float64)
        chunks.append(c)
        got += len(c)
        if got >= need:
            break
    if got < need:
        sys.exit(f"saved map has {got} points, need {need} for the first {a.seconds} s")
    fused = np.concatenate(chunks)[:need]
    frame_of = np.repeat(np.arange(n_frames), counts[:n_frames])
    # native re-projection: p_n = R_n R_f^T (p_f - t_f) + t_n, per frame
    native = np.empty_like(fused)
    for i in range(n_frames):
        s, e = offsets[i], offsets[i + 1]
        if e <= s:
            continue
        Rn = Rotation.from_quat(nat[i, 4:8]); Rf = Rotation.from_quat(fus[i, 4:8])
        native[s:e] = (Rn * Rf.inv()).apply(fused[s:e] - fus[i, 1:4]) + nat[i, 1:4]
    print(f"{run}\nfirst {a.seconds:.0f} s: {n_frames} frames, {need} returns, "
          f"speed at end {np.linalg.norm(np.diff(fus[max(n_frames-11,0):n_frames, 1:4], axis=0), axis=1).sum()/max(t_rel[n_frames-1]-t_rel[max(n_frames-11,0)],1e-3):.1f} m/s")

    # ---- pole detection on the fused cloud ----
    cell = 0.3
    xy_min = fused[:, :2].min(0) - 1.0
    ij = np.floor((fused[:, :2] - xy_min) / cell).astype(np.int64)
    shape = ij.max(0) + 2
    lin = ij[:, 0] * shape[1] + ij[:, 1]
    # local ground: 2 m cells, 5th percentile of z
    gcell = 2.0
    gij = np.floor((fused[:, :2] - xy_min) / gcell).astype(np.int64)
    gshape = gij.max(0) + 2
    glin = gij[:, 0] * gshape[1] + gij[:, 1]
    order = np.argsort(glin, kind="stable")
    gl_sorted = glin[order]
    uniq, start = np.unique(gl_sorted, return_index=True)
    ground = np.full(gshape[0] * gshape[1], np.nan)
    zs = fused[order, 2]
    for u, s0, e0 in zip(uniq, start, np.r_[start[1:], len(gl_sorted)]):
        ground[u] = np.percentile(zs[s0:e0], 5)
    # fill missing ground cells by nearest available
    gimg = ground.reshape(gshape)
    mask = np.isnan(gimg)
    if mask.any():
        idx = ndimage.distance_transform_edt(mask, return_distances=False, return_indices=True)
        gimg = gimg[tuple(idx)]
    h = fused[:, 2] - gimg[gij[:, 0], gij[:, 1]]
    elevated = (h > 1.5) & (h < 12.0)
    # per 0.3 m cell: count and z extent of elevated points
    cnt = np.bincount(lin[elevated], minlength=shape[0] * shape[1])
    zmax = np.full(shape[0] * shape[1], -np.inf); zmin = np.full(shape[0] * shape[1], np.inf)
    np.maximum.at(zmax, lin[elevated], h[elevated]); np.minimum.at(zmin, lin[elevated], h[elevated])
    ext = np.where(cnt > 0, zmax - zmin, 0.0)
    cand = ((cnt >= 40) & (ext >= a.min_height)).reshape(shape)
    lab, nlab = ndimage.label(cand, structure=np.ones((3, 3)))
    poles = []
    for k in range(1, nlab + 1):
        cells = np.argwhere(lab == k)
        diam = (cells.max(0) - cells.min(0) + 1).max() * cell
        if diam > 1.2:  # walls / trees / buildings
            continue
        members = np.isin(lin, cells[:, 0] * shape[1] + cells[:, 1]) & elevated
        if members.sum() < 150:
            continue
        # reject clusters embedded in a larger elevated structure (tree crown, wall): the 1.5 m ring
        # around the block must be mostly free of elevated returns
        c0 = cells.mean(0)
        i0, i1 = int(c0[0]) - 5, int(c0[0]) + 6
        j0, j1 = int(c0[1]) - 5, int(c0[1]) + 6
        block = cnt.reshape(shape)[max(i0, 0):i1, max(j0, 0):j1]
        inner = cnt.reshape(shape)[max(int(c0[0]) - 2, 0):int(c0[0]) + 3, max(int(c0[1]) - 2, 0):int(c0[1]) + 3]
        if block.sum() > 3 * inner.sum():
            continue
        poles.append((k, members, c0))
    if not poles:
        sys.exit("no pole-like clusters found in the start segment")
    # order by time of first observation
    poles.sort(key=lambda t: t_rel[frame_of[t[1]]].min())
    poles = poles[: a.max_poles]

    print(f"{len(poles)} pole-like clusters (>= {a.min_height} m tall, <= 1.2 m wide, >= 150 returns)")
    print(" id  first_seen  shaft_pts  height  |  shaft radial P90 native/fused (cm)  |  ghost spread native-vs-fused shift (cm) over 1-s bins  | bins")
    rat, ghosts, fusedP90, natP90, spread_n, spread_f = [], [], [], [], [], []
    hh = h  # height above local ground (fused frame; native differs by mm here)
    for k, m, c0 in poles:
        # shaft band only (below luminaire arms / sign plates) for the thickness metric
        shaft = m & (hh > a.shaft_band[0]) & (hh < a.shaft_band[1])
        if shaft.sum() < 60:
            continue
        P_f, P_n = fused[shaft], native[shaft]
        tt = t_rel[frame_of[shaft]]
        # PCA axis per cloud
        def radial_p90(P):
            mu = P.mean(0)
            u, s, vt = np.linalg.svd(P - mu, full_matrices=False)
            ax = vt[0]
            if abs(ax[2]) < 0.8:
                return np.nan
            d = (P - mu) - np.outer((P - mu) @ ax, ax)
            return np.percentile(np.linalg.norm(d, axis=1), 90)
        rn, rf = radial_p90(P_n), radial_p90(P_f)
        if not np.isfinite(rn) or rn > a.max_native_p90:
            continue
        # per-1-s-bin circle centres with a shared radius (removes the viewing-side bias of centroids):
        # the spread of centres is the pole ghosting a viewer sees in that map
        from scipy.optimize import least_squares

        def circle(P, r=None, init=None):
            c = np.median(P[:, :2], 0) if init is None else init
            if r is None:
                f = lambda q: np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]) - q[2]
                sol = least_squares(f, [c[0], c[1], 0.1], loss="soft_l1", f_scale=0.03)
                return sol.x[:2], float(np.clip(sol.x[2], 0.04, 0.3))
            f = lambda q: np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]) - r
            sol = least_squares(f, c, loss="soft_l1", f_scale=0.03)
            return sol.x, r

        def centre_spread(P, tt, r):
            # bins with >= 100 shaft returns (near range, what dominates the visual impression);
            # spread = P90 / max of |centre_i - weighted mean centre| (radius of the smear)
            b = np.floor(tt - tt.min()).astype(int)
            cs, ws = [], []
            for bi in np.unique(b):
                sel = b == bi
                if sel.sum() < 100:
                    continue
                cs.append(circle(P[sel], r=r)[0]); ws.append(sel.sum())
            cs, ws = np.array(cs), np.array(ws, dtype=float)
            if len(cs) < 2:
                return np.nan, np.nan
            cbar = (cs * ws[:, None]).sum(0) / ws.sum()
            d = np.linalg.norm(cs - cbar, axis=1)
            return np.percentile(d, 90), d.max()

        _, r_hat = circle(P_n)
        sp_n = centre_spread(P_n, tt, r_hat)
        sp_f = centre_spread(P_f, tt, r_hat)
        # ghosting: per 1-s bin, mean (fused - native) xy shift; spread = max pairwise distance
        b = np.floor(tt - tt.min()).astype(int)
        shifts = []
        for bi in np.unique(b):
            sel = b == bi
            if sel.sum() < 20:
                continue
            shifts.append((P_f[sel, :2] - P_n[sel, :2]).mean(0))
        shifts = np.array(shifts)
        if len(shifts) >= 2:
            dif = shifts[:, None, :] - shifts[None, :, :]
            ghost = np.linalg.norm(dif, axis=2).max()
        else:
            ghost = np.nan
        height = fused[m][:, 2].max() - fused[m][:, 2].min()
        print(f"{k:3d}  {tt.min():8.1f} s {shaft.sum():6d}  {height:5.1f} m |  {rn*100:6.1f} / {rf*100:6.1f}   |  {ghost*100:6.1f}   | {len(shifts):3d} | r {r_hat*100:4.1f} cm; centre spread P90/max native {sp_n[0]*100:5.1f}/{sp_n[1]*100:5.1f}  fused {sp_f[0]*100:5.1f}/{sp_f[1]*100:5.1f}")
        if np.isfinite(sp_n[0]) and np.isfinite(sp_f[0]):
            spread_n.append(sp_n[0]); spread_f.append(sp_f[0])
        if np.isfinite(rn) and np.isfinite(rf):
            natP90.append(rn); fusedP90.append(rf); rat.append(rf / max(rn, 1e-4))
        if np.isfinite(ghost):
            ghosts.append(ghost)
    if not natP90:
        sys.exit("no thin poles (native shaft P90 <= %.0f cm) among the clusters" % (a.max_native_p90 * 100))
    print("summary: radial P90 median native %.1f cm, fused %.1f cm (fused/native median ratio %.2f); "
          "ghost spread median %.1f cm, max %.1f cm over %d poles"
          % (np.median(natP90) * 100, np.median(fusedP90) * 100, np.median(rat), np.median(ghosts) * 100,
             np.max(ghosts) * 100, len(ghosts)))
    if spread_n:
        print("pole centre smear radius (P90 over 1-s bins with >=100 returns, shared radius) median over poles: native %.1f cm, fused %.1f cm"
              % (np.median(spread_n) * 100, np.median(spread_f) * 100))
    # correction smoothness in the same window: per-second change of (fused - native) rear-axle-free body shift
    d = fus[:n_frames, 1:4] - nat[:n_frames, 1:4]
    sec = np.floor(t_rel[:n_frames]).astype(int)
    per_sec = np.array([d[sec == s].mean(0) for s in np.unique(sec)])
    step = np.linalg.norm(np.diff(per_sec, axis=0), axis=1)
    print("correction (fused - native) translation change per second in this window (cm): median %.1f, P90 %.1f, max %.1f"
          % (np.median(step) * 100, np.percentile(step, 90) * 100, step.max() * 100))


if __name__ == "__main__":
    main()
