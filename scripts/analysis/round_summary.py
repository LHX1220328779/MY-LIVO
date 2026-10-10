#!/usr/bin/env python3
"""Compact per-round summary: INS consistency (rear axle + IMU + attitude), native drift,
correction smoothness, and multi-epoch lamp-pole consistency from the ACTUAL saved map_*.pcd.

Usage (container): /usr/bin/python3 scripts/analysis/round_summary.py <fusion_run_dir> <output_dir>
Options: --reference <ins_odometry.csv> (default <output_dir>/reference_csv/ins_odometry.csv, dumped by
scripts/dump_bag_reference.py from the round's recording), --no-pole (skip map_*.pcd lamp-pole scan).
"""
import sys, pathlib, numpy as np
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path("/home/project/MY-LIVO")
sys.path.insert(0, str(REPO / "scripts"))
LEVER = np.array([0.904, -4.679, -1.699])  # IMU -> rear axle, body RFU
TIME_OFFSET = -0.1                          # default; overridden from the run's parameters.txt
POLE = dict(cx=-10.75, cy=-274.0, r=2.5, zmin=43.0, zmax=54.5, shaft=(45.5, 48.5), ref_z=47.0)


def interp_rot(tq, q, t):
    idx = np.clip(np.searchsorted(tq, t), 1, len(tq) - 1)
    a, b = q[idx - 1], q[idx]
    r = np.clip((t - tq[idx - 1]) / (tq[idx] - tq[idx - 1]), 0, 1)
    ra, rb = R.from_quat(a), R.from_quat(b)
    return ra * R.from_rotvec((ra.inv() * rb).as_rotvec() * r[:, None])


def interp_pos(tq, p, t):
    return np.stack([np.interp(t, tq, p[:, i]) for i in range(3)], 1)


def route_weighted(t, err, pos):
    """Distance-weighted fraction of route with |err|<=0.10 and max, using consecutive polyline segments."""
    seg = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    e = np.maximum(err[1:], err[:-1])
    total = seg.sum()
    return 100 * seg[e <= 0.10].sum() / total, 100 * seg[e > 0.20].sum() / total, err.max()


def stats(x):
    x = np.asarray(x)
    return f"median {np.median(x):.4f} P95 {np.percentile(x, 95):.4f} max {x.max():.4f}"


def parse_args(argv):
    pos, opts = [], {"reference": None, "pole": True}
    it = iter(argv)
    for a in it:
        if a == "--reference": opts["reference"] = pathlib.Path(next(it))
        elif a == "--no-pole": opts["pole"] = False
        elif a.startswith("--"): sys.exit(f"unknown option {a}")
        else: pos.append(a)
    if len(pos) != 2: sys.exit(__doc__)
    return pathlib.Path(pos[0]), pathlib.Path(pos[1]), opts


def find_reference(out, explicit):
    """Reference CSV: explicit path > <out>/reference_csv > legacy results/prepared/reference_csv."""
    cands = [explicit] if explicit else []
    cands += [out / "reference_csv" / "ins_odometry.csv", REPO / "results/prepared/reference_csv/ins_odometry.csv"]
    for c in cands:
        if c and c.is_file(): return c
    sys.exit("INS reference CSV not found (%s); run scripts/dump_bag_reference.py --bag <recording> --output %s"
             % (", ".join(str(c) for c in cands), out / "reference_csv"))


POLE_RING_RADIUS = 0.12  # m; fixed lamp-pole radius (see docs/mcp-pole-ghost-viewing-geometry.md)


def ring_fit(xy, centre, radius, band=0.03, iters=30):
    """Fixed-radius circle fit on the shaft returns (view-independent centre).

    Single-sided arcs have two near basins (real axis and the diameter-flip centre
    one radius behind the arc). Seed from centroid + prior + a ring of starts; pick
    the solution with the most inliers, breaking ties by rms then distance to prior.
    Returns (centre, n_inliers) or None.
    """
    xy = np.asarray(xy, dtype=float)
    centre = np.asarray(centre, dtype=float)
    if len(xy) < 20:
        return None

    def solve(init):
        c = np.asarray(init, dtype=float).copy()
        sel = None
        for _ in range(iters):
            d = xy - c
            rr = np.linalg.norm(d, axis=1)
            sel = np.abs(rr - radius) < band
            if sel.sum() < 20:
                return None
            ds = d[sel] / rr[sel, None]
            step, *_ = np.linalg.lstsq(-ds, -(rr[sel] - radius), rcond=None)
            c = c + step
            if np.linalg.norm(step) < 1e-6:
                break
        return c, int(sel.sum())

    c0 = np.median(xy, axis=0)
    starts = [c0, centre]
    for ang in range(0, 360, 30):
        starts.append(c0 + radius * np.array([np.cos(np.radians(ang)), np.sin(np.radians(ang))]))
    best = None
    for s0 in starts:
        r = solve(s0)
        if r is None:
            continue
        score = (-r[1], np.linalg.norm(r[0] - centre))
        if best is None or score < best[1]:
            best = (r, score)
    return best[0] if best else None


def main():
    run, out, opts = parse_args(sys.argv[1:]); out.mkdir(parents=True, exist_ok=True)
    ref = find_reference(out, opts["reference"])
    print(f"reference={ref}")
    ins = np.genfromtxt(ref, delimiter=",", names=True, usecols=range(14))
    params = {}
    for line in (run / "parameters.txt").read_text().splitlines():
        if "=" in line and not line.startswith("model="):
            k, v = line.split("=", 1); params.setdefault(k.strip(), v.strip())  # effective values come first
    global LEVER
    if params.get("effective.imu_to_rear_axle"):
        LEVER = np.array([float(v) for v in params["effective.imu_to_rear_axle"].split()])
    time_offset = float(params.get("rtk.time_offset_sec", TIME_OFFSET))
    att_offset = float(params.get("rtk.attitude_time_offset_sec", time_offset))
    print(f"time offsets: position {time_offset:+.3f} s, attitude {att_offset:+.3f} s (from parameters.txt)")
    tins = ins["stamp"] - time_offset
    tatt = ins["stamp"] - att_offset
    qins = np.stack([ins["qx"], ins["qy"], ins["qz"], ins["qw"]], 1)
    pins = np.stack([ins["x"], ins["y"], ins["z"]], 1)
    lio = np.loadtxt(run / "trajectory_lio.txt"); opt = np.loadtxt(run / "trajectory_optimized.txt")
    t0 = lio[0, 0]
    print(f"run={run}\nframes={len(opt)} duration={opt[-1,0]-t0:.1f}s")
    results = {}
    for name, traj in (("native", lio), ("optimized", opt)):
        t = traj[:, 0]; rq = R.from_quat(traj[:, 4:8]); ri = interp_rot(tatt, qins, t)
        att = (ri.inv() * rq).as_rotvec(degrees=True)
        rear = traj[:, 1:4] + rq.apply(LEVER)
        rear_err = np.linalg.norm(rear - interp_pos(tins, pins, t), axis=1)
        imu_ref = interp_pos(tins, pins, t) - ri.apply(LEVER)
        imu_err = np.linalg.norm(traj[:, 1:4] - imu_ref, axis=1)
        results[name] = (t, att, rear_err, imu_err)
        p10, p20, mx = route_weighted(t, rear_err, interp_pos(tins, pins, t))
        print(f"\n[{name}] rear-axle 3D vs RTK: {stats(rear_err)} | route <=10cm {p10:.2f}% >20cm {p20:.2f}%"
              f"  -> budget {'PASS' if p10 >= 80 and mx <= 0.20 else 'FAIL'}")
        print(f"[{name}] IMU 3D vs INS-derived IMU: {stats(imu_err)} | route <=10cm {route_weighted(t, imu_err, imu_ref)[0]:.2f}%")
        print(f"[{name}] attitude - INS (deg): x(pitch) {stats(np.abs(att[:,0]))} | y(roll) {stats(np.abs(att[:,1]))} | z(yaw) {stats(np.abs(att[:,2]))}")
    t, att, rear_err, imu_err = results["optimized"]
    tn, attn, rear_n, imu_n = results["native"]
    # per-frame error series for the console chart (optimized = delivered map trajectory), <= 600 rows
    step = max(1, len(t) // 600)
    with (out / "error_series.csv").open("w") as f:
        f.write("stamp,t_rel_s,opt_rear_m,opt_imu_m,native_rear_m,native_imu_m\n")
        for i in range(0, len(t), step):
            k = int(np.argmin(np.abs(tn - t[i])))
            f.write(f"{t[i]:.3f},{t[i]-t0:.2f},{rear_err[i]:.4f},{imu_err[i]:.4f},{rear_n[k]:.4f},{imu_n[k]:.4f}\n")
    print("\nwindow(s)  opt: dpitch droll dyaw | rear imu (med) || native: dpitch droll dyaw | rear")
    for w in range(0, 120, 10):
        m = (t - t0 >= w) & (t - t0 < w + 10); mn = (tn - t0 >= w) & (tn - t0 < w + 10)
        if m.any():
            print(f"{w:3d}-{w+10:3d} {np.median(att[m,0]):6.2f} {np.median(att[m,1]):6.2f} {np.median(att[m,2]):6.2f} | "
                  f"{np.median(rear_err[m]):.3f} {np.median(imu_err[m]):.3f} || {np.median(attn[mn,0]):6.2f} {np.median(attn[mn,1]):6.2f} {np.median(attn[mn,2]):6.2f} | {np.median(rear_n[mn]):7.3f}")
    # startup side check: signed cross-track of native/optimized rear axle vs RTK polyline, first 30 m
    pr = interp_pos(tins, pins, t)
    d = np.diff(pr, axis=0); heading = np.arctan2(d[:, 1], d[:, 0]); s = np.concatenate([[0], np.cumsum(np.linalg.norm(d, axis=1))])
    def cross_track(traj_rear):
        e = traj_rear[1:] - pr[1:]
        return -e[:, 0] * np.sin(heading) + e[:, 1] * np.cos(heading)
    rq_o = R.from_quat(opt[:, 4:8]); rq_n = R.from_quat(lio[:, 4:8])
    ri = interp_rot(tatt, qins, t)
    ct_o = cross_track(opt[:, 1:4] + rq_o.apply(LEVER)); ct_n = cross_track(lio[:, 1:4] + rq_n.apply(LEVER))
    for L in (10, 20, 30):
        m = (s[1:] <= L) & (s[1:] > 0.5)
        opp = np.sum(np.sign(ct_o[m]) != np.sign(ct_n[m]))
        print(f"startup first {L} m: rear-axle opposite side {opp}/{m.sum()}; native/opt signed cross-track median {np.median(ct_n[m]):+.4f}/{np.median(ct_o[m]):+.4f} m; |opt| median {np.median(np.abs(ct_o[m])):.4f}")
    # correction smoothness
    c = np.loadtxt(run / "corrections.txt")
    rc = R.from_quat(c[:, 4:8]); rel = (rc[:-1].inv() * rc[1:]).as_rotvec(degrees=True)
    print(f"\ncorrection knots={len(c)}; per-knot rotation step deg: {stats(np.linalg.norm(rel, axis=1))}")
    # native front-end speed sanity
    mat = run.parent.parent / "mat_out.txt"
    if mat.is_file():
        a = np.loadtxt(mat); v = np.linalg.norm(a[:, 7:10], axis=1)
        vins = np.linalg.norm(np.diff(pins, axis=0), axis=1) / np.diff(tins)
        print("native |v| vs INS speed by 10s:", " ".join(f"{np.median(v[(a[:,0]>=w)&(a[:,0]<w+10)]):.1f}/{np.median(vins[(tins[1:]-t0>=w)&(tins[1:]-t0<w+10)]):.1f}" for w in range(0, 120, 10) if ((a[:,0]>=w)&(a[:,0]<w+10)).any()))
        print("native acc bias (m/s^2) at 10s steps:", " ".join(f"[{a[i,13]:.2f},{a[i,14]:.2f},{a[i,15]:.2f}]" for i in [np.argmin(np.abs(a[:,0]-w)) for w in range(0,120,20)]))
    # attitude residual file
    ar = run / "attitude_residuals.csv"
    if ar.is_file():
        head = ar.read_text().splitlines()[0]
        r = np.genfromtxt(ar, delimiter=",", names=True, skip_header=1)
        if r.size:
            print(f"\n{head}\nINS attitude factor residuals (deg): x {stats(np.abs(r['rx_deg']))} | y {stats(np.abs(r['ry_deg']))} | z {stats(np.abs(r['rz_deg']))}; n={r.size}")
    # ---- pole multi-epoch consistency from actual chunks ----
    if not opts["pole"]:
        print("\nLamp pole scan skipped (--no-pole)"); return
    try:
        from evaluate_wuhu_lio import read_pcd
    except Exception as e:
        print("read_pcd import failed:", e); return
    print("\nLamp pole per-chunk (cylinder crop, shaft band %s): chunk  shaft_pts  axis_xy@%.1fm  top_z(P99)  z_max" % (POLE['shaft'], POLE['ref_z']))
    epochs = {}
    for f in sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[1])):
        k = int(f.stem.split("_")[1])
        xyz = np.asarray(read_pcd(f)[:, :3], dtype=float)
        m = (np.hypot(xyz[:, 0] - POLE['cx'], xyz[:, 1] - POLE['cy']) <= POLE['r']) & (xyz[:, 2] >= POLE['zmin']) & (xyz[:, 2] <= POLE['zmax'])
        crop = xyz[m]
        if len(crop) < 30: continue
        shaft = crop[(crop[:, 2] >= POLE['shaft'][0]) & (crop[:, 2] <= POLE['shaft'][1])]
        if len(shaft) < 40: continue
        # robust axis: per 10 cm z-bin centroid XY, then line fit (TLS) through centroids
        bins = np.arange(POLE['shaft'][0], POLE['shaft'][1] + 1e-6, 0.10)
        cents = []
        for lo in bins[:-1]:
            b = shaft[(shaft[:, 2] >= lo) & (shaft[:, 2] < lo + 0.10)]
            if len(b) >= 3: cents.append([np.median(b[:, 0]), np.median(b[:, 1]), lo + 0.05])
        cents = np.array(cents)
        if len(cents) < 5: continue
        mean = cents.mean(0); u, s_, vt = np.linalg.svd(cents - mean); axis = vt[0]
        if axis[2] < 0: axis = -axis
        ref = mean + axis * (POLE['ref_z'] - mean[2]) / axis[2]
        upper = crop[crop[:, 2] > POLE['shaft'][1]]
        top = np.percentile(upper[:, 2], 99) if len(upper) >= 20 else np.nan
        epochs[k] = dict(shaft_xy=shaft[:, :2], n=len(shaft), xy=ref[:2], top=top, zmax=crop[:, 2].max(), cents=cents, tilt=np.degrees(np.arccos(abs(axis[2]))))
        print(f"  map_{k:<3d} {len(shaft):6d}  ({ref[0]:8.3f},{ref[1]:8.3f})  {top:7.3f}  {crop[:,2].max():7.3f}  tilt {epochs[k]['tilt']:.2f} deg")
    keys = sorted(epochs)
    if len(keys) >= 2:
        print("pairwise: pair   axis_xy_sep(cm)  equal-height_median_sep(cm)  top_z_diff(cm)")
        seps = []
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                a, b = epochs[keys[i]], epochs[keys[j]]
                sep = 100 * np.linalg.norm(a['xy'] - b['xy'])
                # equal-height separation using shared bins
                za = {round(c[2], 3): c for c in a['cents']}; zb = {round(c[2], 3): c for c in b['cents']}
                common = sorted(set(za) & set(zb))
                eh = 100 * np.median([np.linalg.norm(za[z][:2] - zb[z][:2]) for z in common]) if common else np.nan
                dz = 100 * (a['top'] - b['top']) if np.isfinite(a['top']) and np.isfinite(b['top']) else np.nan
                seps.append(sep)
                print(f"  {keys[i]:>3d}v{keys[j]:<3d} {sep:8.2f} {eh:14.2f} {dz:14.2f}")
        print(f"[view-biased diagnostic] pole axis XY separation over all epoch pairs (cm): {stats(seps)}")
        # Headline pole consistency: fixed-radius ring centre per epoch. The per-height median above is
        # pulled toward the sensor side of each one-sided view, so it is not an axis estimate.
        centre = np.median(np.vstack([epochs[k]["shaft_xy"] for k in keys]), axis=0)
        rings = {}
        for k in keys:
            fit = ring_fit(epochs[k]["shaft_xy"], centre, POLE_RING_RADIUS)
            if fit is not None:
                rings[k] = fit[0]
        ring_seps = [100 * float(np.linalg.norm(rings[a] - rings[b]))
                     for i, a in enumerate(sorted(rings)) for b in sorted(rings)[i + 1:]]
        if ring_seps:
            print(f"pole ring-centre separation over all epoch pairs (cm, radius {POLE_RING_RADIUS*100:.0f} cm fixed): {stats(ring_seps)}")


if __name__ == "__main__":
    main()

