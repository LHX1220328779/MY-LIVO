#!/usr/bin/env python3
"""Cross-section anatomy of the start-segment lamp poles: what makes the pole look thick?

Decomposes the accumulated pole cloud of a run into
  * single-scan thickness (returns of ONE scan; what the sensors + multi-LiDAR extrinsics give)
  * within-scan structure by point time (offset_ms; the four LiDARs sweep the pole at different
    times inside the 100 ms scan, so offset clusters ~ LiDAR sources; their mutual shift = extrinsic
    / deskew consistency)
  * accumulated thickness over the whole run, native LIO poses vs fused (RTK-corrected) poses,
    split into passes (time clusters) -> pose drift / correction jitter between passes
and renders a PNG with the cross-sections so the numbers can be checked by eye.

Usage: pole_xsection.py <round or fusion dir> [--detect-seconds 30] [--poles 3] [--out PNG_PREFIX]
"""
import argparse, pathlib, struct, sys
import numpy as np

REPO = pathlib.Path("/home/project/MY-LIVO")
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "scripts" / "analysis"))
from evaluate_wuhu_lio import read_pcd  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from scipy import ndimage  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402


def load_frames(run):
    txt = (run / "frames.csv").read_text().splitlines()
    rows = [l for l in txt if l and not l.startswith("#")]
    head = rows[0].split(",")
    data = np.array([[float(v) for v in r.split(",")] for r in rows[1:]])
    out = {k: data[:, i] for i, k in enumerate(head)}
    lever = np.zeros(3)
    for l in txt:
        if l.startswith("# lidar_to_imu_translation"):
            lever = np.array([float(v) for v in l.split()[2:5]])
    out["lever"] = lever
    return out


def pose_rows(path, stamps):
    tr = np.loadtxt(path, ndmin=2)
    if len(tr) == len(stamps) and np.max(np.abs(tr[:, 0] - stamps)) < 2e-3:
        return tr
    idx = np.clip(np.searchsorted(tr[:, 0], stamps), 0, len(tr) - 1)
    return tr[idx]


def read_scans(run, fr, frame_ids):
    """native (rear-axle frame, deskewed to scan end) points of the given frames: dict id -> (N,5)"""
    out = {}
    with open(run / "scans.bin", "rb") as f:
        for i in frame_ids:
            off, n = int(fr["offset_bytes"][i]), int(fr["count"][i])
            f.seek(off)
            buf = f.read(n * 20)
            out[i] = np.frombuffer(buf, dtype="<f4").reshape(-1, 5).astype(np.float64)
    return out


LEVER = np.zeros(3)


def source_names(run):
    """multi_lidar.topics order of the round (results/<round>/config.yaml)"""
    try:
        import yaml
        cfg = yaml.safe_load((run.parents[2] / "config.yaml").read_text())
        return [t.strip("/") for t in cfg["laser_mapping"]["ros__parameters"]["multi_lidar"]["topics"]]
    except Exception as e:  # noqa: BLE001
        return []


def world(points, pose):
    R = Rotation.from_quat(pose[4:8])
    return R.apply(points[:, :3] + LEVER) + pose[1:4]


def circle(P, r=None):
    c = np.median(P[:, :2], 0)
    if r is None:
        f = lambda q: np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]) - q[2]
        sol = least_squares(f, [c[0], c[1], 0.1], loss="soft_l1", f_scale=0.03)
        return sol.x[:2], float(np.clip(sol.x[2], 0.04, 0.3))
    f = lambda q: np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]) - r
    sol = least_squares(f, c, loss="soft_l1", f_scale=0.03)
    return sol.x, r


def detect_poles(cloud, min_height=3.0, max_poles=6):
    cell = 0.3
    xy_min = cloud[:, :2].min(0) - 1.0
    ij = np.floor((cloud[:, :2] - xy_min) / cell).astype(np.int64)
    shape = ij.max(0) + 2
    lin = ij[:, 0] * shape[1] + ij[:, 1]
    gcell = 2.0
    gij = np.floor((cloud[:, :2] - xy_min) / gcell).astype(np.int64)
    gshape = gij.max(0) + 2
    glin = gij[:, 0] * gshape[1] + gij[:, 1]
    order = np.argsort(glin, kind="stable")
    gl_sorted = glin[order]
    uniq, start = np.unique(gl_sorted, return_index=True)
    ground = np.full(gshape[0] * gshape[1], np.nan)
    zs = cloud[order, 2]
    for u, s0, e0 in zip(uniq, start, np.r_[start[1:], len(gl_sorted)]):
        ground[u] = np.percentile(zs[s0:e0], 5)
    gimg = ground.reshape(gshape)
    mask = np.isnan(gimg)
    if mask.any():
        idx = ndimage.distance_transform_edt(mask, return_distances=False, return_indices=True)
        gimg = gimg[tuple(idx)]
    h = cloud[:, 2] - gimg[gij[:, 0], gij[:, 1]]
    elevated = (h > 1.5) & (h < 12.0)
    cnt = np.bincount(lin[elevated], minlength=shape[0] * shape[1])
    zmax = np.full(shape[0] * shape[1], -np.inf); zmin = np.full(shape[0] * shape[1], np.inf)
    np.maximum.at(zmax, lin[elevated], h[elevated]); np.minimum.at(zmin, lin[elevated], h[elevated])
    ext = np.where(cnt > 0, zmax - zmin, 0.0)
    cand = ((cnt >= 40) & (ext >= min_height)).reshape(shape)
    lab, nlab = ndimage.label(cand, structure=np.ones((3, 3)))
    poles = []
    for k in range(1, nlab + 1):
        cells = np.argwhere(lab == k)
        diam = (cells.max(0) - cells.min(0) + 1).max() * cell
        if diam > 1.2:
            continue
        members = np.isin(lin, cells[:, 0] * shape[1] + cells[:, 1]) & elevated
        if members.sum() < 150:
            continue
        c0 = cells.mean(0)
        block = cnt.reshape(shape)[max(int(c0[0]) - 5, 0):int(c0[0]) + 6, max(int(c0[1]) - 5, 0):int(c0[1]) + 6]
        inner = cnt.reshape(shape)[max(int(c0[0]) - 2, 0):int(c0[0]) + 3, max(int(c0[1]) - 2, 0):int(c0[1]) + 3]
        if block.sum() > 3 * inner.sum():
            continue
        shaft = members & (h > 1.0) & (h < 4.0)
        if shaft.sum() < 60:
            continue
        cxy, r = circle(cloud[shaft])
        gz = np.median(gimg[gij[shaft, 0], gij[shaft, 1]])
        poles.append(dict(centre=cxy, radius=r, ground_z=gz, n=int(members.sum()),
                          height=float(cloud[members][:, 2].max() - cloud[members][:, 2].min())))
    return poles


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--detect-seconds", type=float, default=30.0)
    ap.add_argument("--poles", type=int, default=3)
    ap.add_argument("--out", default="")
    ap.add_argument("--band", type=float, nargs=2, default=(1.0, 4.0))
    ap.add_argument("--crop", type=float, default=0.6, help="radius around the pole axis to collect returns")
    a = ap.parse_args()
    run = pathlib.Path(a.run)
    if not (run / "frames.csv").exists():
        run = sorted((run / "Log" / "fusion").glob("*"))[-1]
    fr = load_frames(run)
    global LEVER
    LEVER = fr["lever"]
    stamps, counts = fr["stamp"], fr["count"].astype(np.int64)
    t_rel = stamps - stamps[0]
    n = len(stamps)
    nat = np.column_stack([stamps, fr["tx"], fr["ty"], fr["tz"], fr["qx"], fr["qy"], fr["qz"], fr["qw"]])
    fus = pose_rows(run / "trajectory_optimized.txt", stamps)
    lio = pose_rows(run / "trajectory_lio.txt", stamps)
    # sanity: frames.csv transform vs trajectory_lio
    dd = np.linalg.norm(nat[:, 1:4] - lio[:, 1:4], axis=1)
    print(f"{run}: {n} frames, {t_rel[-1]:.1f} s; frames.csv pose vs trajectory_lio.txt max |dt| {dd.max()*100:.2f} cm")

    n_det = int(np.searchsorted(t_rel, a.detect_seconds))
    scans = read_scans(run, fr, range(n_det))
    cloud = np.concatenate([world(scans[i], nat[i]) for i in range(n_det)])
    poles = detect_poles(cloud)
    # order by distance from the first pose
    poles = [p for p in poles if p["radius"] <= 0.2]
    poles.sort(key=lambda p: -p["n"])
    poles = poles[: a.poles]
    for p in poles:
        print(f"  pole at ({p['centre'][0]:.1f},{p['centre'][1]:.1f}) r~{p['radius']*100:.0f} cm h {p['height']:.1f} m n {p['n']} dist from start {np.linalg.norm(p['centre'] - nat[0, 1:3]):.1f} m")
    print(f"{len(poles)} poles near the start (native frame of the first {a.detect_seconds:.0f} s)")
    del cloud

    # collect all returns near each pole over the whole run (streaming)
    coll = [dict(nat=[], fus=[], t=[], off=[], frame=[], rng=[], src=[]) for _ in poles]
    src_path = run / "sources.bin"
    have_src = src_path.exists()
    src_names = source_names(run) if have_src else []
    if have_src:
        print(f"  sources.bin present: LiDAR order {src_names}")
    fsrc = open(src_path, "rb") if have_src else None
    with open(run / "scans.bin", "rb") as f:
        for i in range(n):
            off, cnt = int(fr["offset_bytes"][i]), int(counts[i])
            f.seek(off)
            pts = np.frombuffer(f.read(cnt * 20), dtype="<f4").reshape(-1, 5).astype(np.float64)
            if have_src:
                fsrc.seek(8 + (off - 8) // 20)
                srcs = np.frombuffer(fsrc.read(cnt), dtype=np.uint8)
            else:
                srcs = np.full(cnt, 255, np.uint8)
            Rn = Rotation.from_quat(nat[i, 4:8]); tn = nat[i, 1:4]
            Rf = Rotation.from_quat(fus[i, 4:8]); tf = fus[i, 1:4]
            wn = Rn.apply(pts[:, :3] + LEVER) + tn
            for k, p in enumerate(poles):
                d = np.hypot(wn[:, 0] - p["centre"][0], wn[:, 1] - p["centre"][1])
                h = wn[:, 2] - p["ground_z"]
                sel = (d < a.crop) & (h > a.band[0]) & (h < a.band[1])
                if not sel.any():
                    continue
                coll[k]["nat"].append(wn[sel]); coll[k]["fus"].append(Rf.apply(pts[sel, :3] + LEVER) + tf)
                coll[k]["t"].append(np.full(sel.sum(), t_rel[i])); coll[k]["off"].append(pts[sel, 4])
                coll[k]["frame"].append(np.full(sel.sum(), i)); coll[k]["rng"].append(np.linalg.norm(pts[sel, :3], axis=1))
                coll[k]["src"].append(srcs[sel])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for k, (p, c) in enumerate(zip(poles, coll)):
        if not c["nat"]:
            continue
        N = np.concatenate(c["nat"]); F = np.concatenate(c["fus"]); T = np.concatenate(c["t"])
        O = np.concatenate(c["off"]); FR = np.concatenate(c["frame"]); RG = np.concatenate(c["rng"]); SRC = np.concatenate(c["src"])
        print(f"\n=== pole {k}: at ({p['centre'][0]:.1f},{p['centre'][1]:.1f}) r~{p['radius']*100:.0f} cm, height {p['height']:.1f} m, "
              f"{len(N)} shaft returns from {len(np.unique(FR))} frames, t {T.min():.1f}-{T.max():.1f} s, range {np.percentile(RG,5):.1f}-{np.percentile(RG,95):.1f} m")
        # passes = time clusters separated by > 3 s
        order = np.argsort(T)
        gaps = np.where(np.diff(T[order]) > 3.0)[0]
        pass_id = np.zeros(len(T), int)
        pid = np.zeros(len(order), int)
        for g in gaps:
            pid[g + 1:] += 1
        pass_id[order] = pid
        # global circle (native)
        cN, r = circle(N)
        cF, _ = circle(F, r=r)
        rad = lambda P, cc: np.hypot(P[:, 0] - cc[0], P[:, 1] - cc[1])
        rN, rF = rad(N, cN), rad(F, cF)
        def desc(rr):
            return f"P50 {np.percentile(rr,50)*100:5.1f} P90 {np.percentile(rr,90)*100:5.1f} P99 {np.percentile(rr,99)*100:5.1f} cm; outer diameter (2*P95) {2*np.percentile(rr,95)*100:5.1f} cm"
        print(f"  accumulated radius about the fitted axis: native {desc(rN)}")
        print(f"                                            fused  {desc(rF)}")
        # single-scan thickness: per frame with >= 25 returns, circle with the shared radius, radial P90 about own centre
        fr_ids, sc_p90, sc_cent, sc_t, sc_n = [], [], [], [], []
        for i in np.unique(FR):
            sel = FR == i
            if sel.sum() < 25:
                continue
            cc, _ = circle(N[sel], r=r)
            sc_p90.append(np.percentile(rad(N[sel], cc), 90)); sc_cent.append(cc); sc_t.append(T[sel][0]); sc_n.append(sel.sum()); fr_ids.append(i)
        sc_p90, sc_cent, sc_t, sc_n = map(np.array, (sc_p90, sc_cent, sc_t, sc_n))
        if len(sc_p90):
            print(f"  single-scan radial P90 about own centre (frames with >=25 returns, n={len(sc_p90)}): median {np.median(sc_p90)*100:.1f} cm, "
                  f"P90 {np.percentile(sc_p90,90)*100:.1f} cm  (a perfect pole gives ~{r*100:.0f} cm = the radius)")
            dc = np.linalg.norm(sc_cent - np.median(sc_cent, 0), axis=1)
            print(f"  scan-to-scan centre wander (native): median {np.median(dc)*100:.1f} cm, P90 {np.percentile(dc,90)*100:.1f} cm, max {dc.max()*100:.1f} cm")
        # per pass
        print("  passes (time clusters):")
        for q in np.unique(pass_id):
            sel = pass_id == q
            cn, _ = circle(N[sel], r=r); cf, _ = circle(F[sel], r=r)
            print(f"    pass {q}: t {T[sel].min():6.1f}-{T[sel].max():6.1f} s, {sel.sum():6d} returns, range {np.median(RG[sel]):5.1f} m | "
                  f"native centre offset {np.linalg.norm(cn-cN)*100:5.1f} cm, P90 radius {np.percentile(rad(N[sel],cn),90)*100:5.1f} | "
                  f"fused centre offset {np.linalg.norm(cf-cF)*100:5.1f} cm, P90 radius {np.percentile(rad(F[sel],cf),90)*100:5.1f}")
        # within-scan time structure: offset_ms clusters in the densest frame
        j = fr_ids[int(np.argmax(sc_n))] if len(sc_n) else None
        if j is not None:
            sel = FR == j
            o = O[sel]
            hist, edges = np.histogram(o, bins=np.arange(0, 101, 5))
            peaks = [(int(edges[m]), int(hist[m])) for m in range(len(hist)) if hist[m] > 0]
            print(f"  densest single scan (frame {j}, t={T[sel][0]:.1f} s, {sel.sum()} returns, range {np.median(RG[sel]):.1f} m): offset_ms histogram (5 ms bins, start:count) {peaks}")
            # cluster by offset gaps > 8 ms and report centres
            oo = np.sort(o); brk = np.where(np.diff(oo) > 8)[0]
            bounds = np.r_[oo[0], oo[brk + 1]]
            for b0 in bounds:
                m = (o >= b0) & (o < (oo[brk[np.searchsorted(bounds, b0, side='right') - 1]] if False else b0 + 100))
            groups = []
            start = oo[0]
            for b in brk:
                groups.append((start, oo[b])); start = oo[b + 1]
            groups.append((start, oo[-1]))
            for g0, g1 in groups:
                m = (o >= g0) & (o <= g1)
                if m.sum() < 8:
                    continue
                cc, _ = circle(N[sel][m], r=r)
                print(f"     offset {g0:5.1f}-{g1:5.1f} ms: {m.sum():4d} returns, centre offset from scan centre {np.linalg.norm(cc - sc_cent[int(np.argmax(sc_n))])*100:5.1f} cm, radial P90 {np.percentile(rad(N[sel][m], cc),90)*100:5.1f} cm")
        # ---- per-frame centre track: where does each scan put the pole, relative to the close-range solution?
        def track(P, poses, label):
            rows = []
            for i in np.unique(FR):
                sel = FR == i
                if sel.sum() < 15:
                    continue
                cc, _ = circle(P[sel], r=r)
                truck = poses[i, 1:3]
                los = p["centre"] - truck; rng_i = np.linalg.norm(los); los /= max(rng_i, 1e-6)
                i0, i1 = max(i - 2, 0), min(i + 2, n - 1)
                vel = (poses[i1, 1:3] - poses[i0, 1:3]) / max(poses[i1, 0] - poses[i0, 0], 1e-3)
                rows.append((i, T[sel][0], rng_i, cc[0], cc[1], los[0], los[1], vel[0], vel[1], sel.sum(),
                             np.percentile(rad(P[sel], cc), 90)))
            R_ = np.array(rows)
            if len(R_) < 3:
                return None
            close = R_[:, 2] < 15.0
            cref = np.median(R_[close, 3:5], 0) if close.sum() >= 3 else np.median(R_[:, 3:5], 0)
            d = R_[:, 3:5] - cref
            los = R_[:, 5:7]; tang = np.column_stack([-los[:, 1], los[:, 0]])
            spd = np.linalg.norm(R_[:, 7:9], axis=1); w = R_[:, 7:9] / np.maximum(spd, 1e-3)[:, None]
            along = np.sum(d * los, 1); cross = np.sum(d * tang, 1); travel = np.sum(d * w, 1)
            print(f"  {label} per-frame centre offsets vs the close-range (<15 m) solution, binned by range:")
            print("     range bin   frames  median|d|  along-LOS  cross-LOS  along-travel  speed  single-scan P90")
            for lo, hi in ((0, 10), (10, 15), (15, 20), (20, 30), (30, 45), (45, 90)):
                m = (R_[:, 2] >= lo) & (R_[:, 2] < hi)
                if m.sum() < 2:
                    continue
                print(f"     {lo:3d}-{hi:3d} m   {m.sum():5d}   {np.median(np.linalg.norm(d[m],axis=1))*100:6.1f}   {np.median(along[m])*100:+7.1f}    {np.median(cross[m])*100:+7.1f}     {np.median(travel[m])*100:+7.1f}     {np.median(spd[m]):4.1f}    {np.median(R_[m,10])*100:5.1f} cm")
            # time trend among frames closer than 30 m (LIO drift shows as a trend with time, not range)
            m = R_[:, 2] < 30.0
            if m.sum() >= 5:
                A = np.column_stack([R_[m, 1] - R_[m, 1].min(), np.ones(m.sum())])
                kx = np.linalg.lstsq(A, d[m, 0], rcond=None)[0][0]; ky = np.linalg.lstsq(A, d[m, 1], rcond=None)[0][0]
                print(f"     centre trend with time (<30 m frames, {m.sum()} frames over {R_[m,1].max()-R_[m,1].min():.1f} s): {np.hypot(kx,ky)*100:.2f} cm/s")
            return dict(R=R_, d=d, along=along, cross=cross, travel=travel, spd=spd, cref=cref)
        trN = track(N, nat, "native")
        trF = track(F, fus, "fused ")
        # ---- per physical LiDAR (sources.bin): does each LiDAR put the pole at the same place?
        per_src = {}
        if have_src and (SRC != 255).any():
            print("  per physical LiDAR (native poses; centre offsets relative to the all-source axis):")
            print("     source             returns  frames  range(med)  centre dx   dy   |d|   radial P90  single-scan P90(med)")
            for sidx in np.unique(SRC):
                m = SRC == sidx
                if m.sum() < 30:
                    continue
                cs, _ = circle(N[m], r=r)
                p90 = np.percentile(rad(N[m], cs), 90)
                # single-scan thickness of this source alone
                ss = []
                for i in np.unique(FR[m]):
                    mm = m & (FR == i)
                    if mm.sum() >= 15:
                        cc, _ = circle(N[mm], r=r); ss.append(np.percentile(rad(N[mm], cc), 90))
                name = src_names[sidx] if sidx < len(src_names) else str(sidx)
                per_src[sidx] = dict(centre=cs, n=int(m.sum()), p90=p90, name=name)
                print(f"     {name:18s} {m.sum():7d}  {len(np.unique(FR[m])):5d}   {np.median(RG[m]):6.1f} m    {(cs[0]-cN[0])*100:+5.1f} {(cs[1]-cN[1])*100:+5.1f} {np.linalg.norm(cs-cN)*100:5.1f}    {p90*100:5.1f} cm     {np.median(ss)*100 if ss else float('nan'):5.1f} cm ({len(ss)} scans)")
            # pairwise separation in scans where two sources see the pole simultaneously
            pairs = {}
            for i in np.unique(FR):
                mi = FR == i
                srcs_here = [sv for sv in np.unique(SRC[mi]) if (mi & (SRC == sv)).sum() >= 10]
                Rb = Rotation.from_quat(nat[i, 4:8])
                i0, i1 = max(i - 2, 0), min(i + 2, n - 1)
                vw = (nat[i1, 1:4] - nat[i0, 1:4]) / max(nat[i1, 0] - nat[i0, 0], 1e-3)
                vb = Rb.inv().apply(vw)
                for a_ in range(len(srcs_here)):
                    for b_ in range(a_ + 1, len(srcs_here)):
                        ca, _ = circle(N[mi & (SRC == srcs_here[a_])], r=r); cb, _ = circle(N[mi & (SRC == srcs_here[b_])], r=r)
                        dw = np.r_[cb - ca, 0.0]
                        db = Rb.inv().apply(dw)  # body frame: x right, y forward
                        pairs.setdefault((srcs_here[a_], srcs_here[b_]), []).append(np.r_[cb - ca, db[:2], np.linalg.norm(vb[:2]), T[mi][0]])
            for (sa, sb), v in pairs.items():
                v = np.array(v)
                na = src_names[sa] if sa < len(src_names) else str(sa); nb = src_names[sb] if sb < len(src_names) else str(sb)
                print(f"     same-scan separation {nb} - {na}: {len(v)} scans (t {v[:,5].min():.0f}-{v[:,5].max():.0f} s, speed {np.median(v[:,4]):.1f} m/s): world dx {np.median(v[:,0])*100:+5.1f} dy {np.median(v[:,1])*100:+5.1f} cm | "
                      f"BODY right {np.median(v[:,2])*100:+5.1f} forward {np.median(v[:,3])*100:+5.1f} cm (P25-P75 fwd {np.percentile(v[:,3],25)*100:+.1f}..{np.percentile(v[:,3],75)*100:+.1f}) | median |d| {np.median(np.linalg.norm(v[:,:2],axis=1))*100:5.1f} cm")
        # figure
        if a.out:
            fig, axs = plt.subplots(2, 2, figsize=(12, 12))
            lim = 0.45
            for ax, P, cc, title in ((axs[0, 0], N, cN, "native LIO poses, all passes (colour = time s)"),
                                     (axs[0, 1], F, cF, "fused poses, all passes (colour = time s)")):
                sc = ax.scatter(P[:, 0] - cc[0], P[:, 1] - cc[1], c=T, s=2, cmap="turbo")
                ax.add_patch(plt.Circle((0, 0), r, fill=False, color="k", lw=1))
                ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal"); ax.set_title(title); ax.grid(alpha=.3)
                plt.colorbar(sc, ax=ax, fraction=0.046)
            if j is not None:
                sel = FR == j
                cc = sc_cent[int(np.argmax(sc_n))]
                if have_src:
                    for sv in np.unique(SRC[sel]):
                        mm = sel & (SRC == sv)
                        axs[1, 0].scatter(N[mm, 0] - cc[0], N[mm, 1] - cc[1], s=8, label=src_names[sv] if sv < len(src_names) else str(sv))
                    axs[1, 0].legend(fontsize=8)
                    sc = None
                else:
                    sc = axs[1, 0].scatter(N[sel, 0] - cc[0], N[sel, 1] - cc[1], c=O[sel], s=6, cmap="viridis")
                axs[1, 0].add_patch(plt.Circle((0, 0), r, fill=False, color="k", lw=1))
                axs[1, 0].set_xlim(-lim, lim); axs[1, 0].set_ylim(-lim, lim); axs[1, 0].set_aspect("equal")
                axs[1, 0].set_title(f"one scan (frame {j}, t={T[sel][0]:.1f}s), colour = " + ("LiDAR source" if have_src else "point time in scan (ms)")); axs[1, 0].grid(alpha=.3)
                if sc is not None:
                    plt.colorbar(sc, ax=axs[1, 0], fraction=0.046)
            ax = axs[1, 1]
            bins = np.linspace(0, 0.45, 46)
            ax.hist(rN * 1, bins=bins, histtype="step", label="native accumulated", density=True)
            ax.hist(rF * 1, bins=bins, histtype="step", label="fused accumulated", density=True)
            if j is not None:
                sel = FR == j
                ax.hist(rad(N[sel], sc_cent[int(np.argmax(sc_n))]), bins=bins, histtype="step", label="single scan", density=True)
            ax.axvline(r, color="k", lw=1, label=f"fitted radius {r*100:.0f} cm")
            ax.set_xlabel("radial distance from pole axis (m)"); ax.legend(); ax.set_title("radial distribution")
            fig.suptitle(f"{run.parents[2].name} pole {k} ({p['centre'][0]:.1f},{p['centre'][1]:.1f}) h={p['height']:.1f} m")
            fig.tight_layout()
            fig.savefig(f"{a.out}_pole{k}.png", dpi=80)
            plt.close(fig)
            print(f"  wrote {a.out}_pole{k}.png")
            if per_src:
                fig, axs = plt.subplots(1, len(per_src), figsize=(5 * len(per_src), 5), squeeze=False)
                for ax, (sidx, info) in zip(axs[0], per_src.items()):
                    m = SRC == sidx
                    sc = ax.scatter(N[m, 0] - cN[0], N[m, 1] - cN[1], c=T[m], s=2, cmap="turbo")
                    ax.add_patch(plt.Circle((0, 0), r, fill=False, color="k", lw=1))
                    ax.plot([info["centre"][0] - cN[0]], [info["centre"][1] - cN[1]], "k+", ms=12, mew=2)
                    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal"); ax.grid(alpha=.3)
                    ax.set_title(f"{info['name']}: {info['n']} returns, P90 {info['p90']*100:.1f} cm, centre {np.linalg.norm(info['centre']-cN)*100:.1f} cm off")
                fig.suptitle(f"{run.parents[2].name} pole {k}: native map split by physical LiDAR (colour = time s)")
                fig.tight_layout(); fig.savefig(f"{a.out}_pole{k}_sources.png", dpi=80); plt.close(fig)
                print(f"  wrote {a.out}_pole{k}_sources.png")
            if trN is not None:
                fig, axs = plt.subplots(1, 3, figsize=(18, 5))
                for tr_, mk, lab in ((trN, "o", "native"), (trF, "x", "fused")):
                    if tr_ is None:
                        continue
                    axs[0].plot(tr_["R"][:, 2], tr_["along"] * 100, mk, ms=3, label=f"{lab} along-LOS")
                    axs[0].plot(tr_["R"][:, 2], tr_["cross"] * 100, mk, ms=3, label=f"{lab} cross-LOS", alpha=.6)
                    axs[1].plot(tr_["R"][:, 1], tr_["d"][:, 0] * 100, mk, ms=3, label=f"{lab} dx")
                    axs[1].plot(tr_["R"][:, 1], tr_["d"][:, 1] * 100, mk, ms=3, label=f"{lab} dy", alpha=.6)
                    axs[2].plot(tr_["R"][:, 2], tr_["R"][:, 10] * 100, mk, ms=3, label=lab)
                axs[0].set_xlabel("range (m)"); axs[0].set_ylabel("centre offset (cm)"); axs[0].set_title("scan centre offset vs range (ref = <15 m frames)"); axs[0].grid(alpha=.3); axs[0].legend(fontsize=8)
                ax2 = axs[1].twinx(); ax2.plot(trN["R"][:, 1], trN["spd"], "k-", lw=0.8, alpha=.5); ax2.set_ylabel("speed (m/s)")
                axs[1].set_xlabel("time (s)"); axs[1].set_ylabel("centre offset (cm)"); axs[1].set_title("scan centre offset vs time"); axs[1].grid(alpha=.3); axs[1].legend(fontsize=8)
                axs[2].axhline(r * 100, color="k", lw=1); axs[2].set_xlabel("range (m)"); axs[2].set_ylabel("radial P90 about own centre (cm)"); axs[2].set_title("single-scan thickness vs range"); axs[2].grid(alpha=.3); axs[2].legend(fontsize=8)
                fig.suptitle(f"{run.parents[2].name} pole {k} track")
                fig.tight_layout(); fig.savefig(f"{a.out}_pole{k}_track.png", dpi=80); plt.close(fig)
                print(f"  wrote {a.out}_pole{k}_track.png")


if __name__ == "__main__":
    main()
