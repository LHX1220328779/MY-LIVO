#!/usr/bin/env python3
"""Compact per-round summary: INS consistency (rear axle + IMU + attitude), native drift,
correction smoothness, and multi-epoch lamp-pole consistency from the ACTUAL saved map_*.pcd.

Usage (container): /usr/bin/python3 scripts/analysis/round_summary.py <fusion_run_dir> <output_dir>
Reads the INS reference CSV dumped by scripts/dump_bag_reference.py (results/prepared/reference_csv).
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


def main():
    run = pathlib.Path(sys.argv[1]); out = pathlib.Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    ref = REPO / "results/prepared/reference_csv/ins_odometry.csv"
    ins = np.genfromtxt(ref, delimiter=",", names=True, usecols=range(14))
    params = {}
    for line in (run / "parameters.txt").read_text().splitlines():
        if "=" in line and not line.startswith("model="):
            k, v = line.split("=", 1); params[k.strip()] = v.strip()
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
        epochs[k] = dict(n=len(shaft), xy=ref[:2], top=top, zmax=crop[:, 2].max(), cents=cents, tilt=np.degrees(np.arccos(abs(axis[2]))))
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
        print(f"pole axis XY separation over all epoch pairs (cm): {stats(seps)}")


if __name__ == "__main__":
    main()

