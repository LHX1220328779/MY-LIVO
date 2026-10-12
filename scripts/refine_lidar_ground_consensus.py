#!/usr/bin/env python3
"""Ground-consensus tilt/z refinement of the rear-facing LiDAR(s), from a finished LIO run with the current
extrinsics (pipeline: the RTK time-association round, i.e. the driven calibration and RTK-optimized poses).

Why: the driven stage registers every LiDAR against the front (reference) LiDAR. The front LiDAR sees the ground
only >= 10 m ahead, so a LiDAR looking backwards meets it only cross-time (5-15 s apart) and at far, grazing
range; its tilt/z then follows the far-range view of a single sensor. truck29 v101..v106: the back LiDAR sat
0.33 deg / 5.7 cm off the ground seen by all other LiDARs (ground residual -3 cm at 3-6 m, +4 cm at 15-40 m
behind the truck), i.e. range-dependent ground layering wherever the back LiDAR re-observes a road section.
Corrected (v108, same data) the flat-road ground got thinner (P10-P90 median 5.7 -> 4.9 cm, native pure-LIO
5.5 -> 4.6 cm, thick flat cells -14 % / -35 %), the native drift fell from 8.6 to 2.7 m and the pole shells
became slightly tighter, while the RTK-fused accuracy stayed within its guard.

Method: the raw returns of every LiDAR (deskewed per point with the run's RTK-optimized poses) -> lowest-surface
returns per 0.5 m cell -> robust plane per flat cell (slope <= 6 deg) from ALL LiDARs (consensus) -> for each
rear-facing LiDAR (median ground return behind the rear axle) a robust least-squares rotation about x/y (about
the LiDAR position, rear-axle frame) + z translation, alternated with re-fitting the consensus planes. The side
LiDARs are deliberately left untouched: they overlap the reference at the same time and are solved by the driven
stage; their remaining far-rear ground bias is non-rigid (a rigid fit there did not survive a re-run).

Quality gate: correction <= 0.6 deg / 12 cm (fail), halves repeatability (warn), and only a correction that
moves the LiDAR's own ground by >= 0.8 cm RMS is applied - otherwise the parent calibration is kept and no file
is written (exit 0, "kept" in the report). Estimates from different fused runs of the same data scatter by up to
~0.1 deg / 2 cm (front and rear LiDARs see a ground cell seconds apart, the pose error in between enters the
estimate); no larger floor is applied because a 0.08 deg / 1.2 cm correction on truck28 still clearly improved the
map (flat ground cells >10 cm -40 %, native z drift 15 m -> 2 m over 1228 s).

Usage (container):
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/refine_lidar_ground_consensus.py --calibration <parent calibration.json> \
      --bag <dataset> --run <fusion_run_dir> --lidar-time-offset -0.125 --output <calibration.json> --report <txt>
"""
import argparse, json, pathlib, sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import refine_multi_lidar_yaw as rm  # noqa: E402  (ROS environment, Dataset, pose helpers, LiDAR keys)
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

GATE_MAX_ROT_DEG = 0.6
GATE_MAX_TRANS_M = 0.12
MIN_EFFECT_M = 0.008          # RMS change of the LiDAR's own ground returns below which the parent is kept
REPEAT_ROT_DEG, REPEAT_TRANS_M = 0.10, 0.015


def planes(W, inv, ncell, mask):
    """Robust plane per cell (3 reweighting passes) -> residual per point, cell slope [deg], cell support."""
    cx = np.bincount(inv, W[:, 0], ncell) / np.maximum(np.bincount(inv, minlength=ncell), 1)
    cy = np.bincount(inv, W[:, 1], ncell) / np.maximum(np.bincount(inv, minlength=ncell), 1)
    x = W[:, 0] - cx[inv]; y = W[:, 1] - cy[inv]; z = W[:, 2]
    use = mask.copy()
    for _ in range(3):
        w = use.astype(float)
        def B(v): return np.bincount(inv, v * w, ncell)
        n, sx, sy, sxx, sxy, syy = B(np.ones_like(x)), B(x), B(y), B(x * x), B(x * y), B(y * y)
        A = np.stack([np.stack([n, sx, sy], -1), np.stack([sx, sxx, sxy], -1), np.stack([sy, sxy, syy], -1)], 1)
        b = np.stack([B(z), B(x * z), B(y * z)], -1)
        coef = np.linalg.solve(A + np.eye(3) * 1e-6, b[..., None])[..., 0]
        res = z - (coef[inv, 0] + coef[inv, 1] * x + coef[inv, 2] * y)
        use = mask & (np.abs(res) < 0.12)
    slope = np.degrees(np.arctan(np.hypot(coef[:, 1], coef[:, 2])))
    return res, slope[inv], np.bincount(inv, mask.astype(float), ncell)[inv]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", required=True); ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True, help="fusion run dir with frames.csv + trajectory_optimized.txt")
    ap.add_argument("--lidar-time-offset", type=float, default=-0.125)
    ap.add_argument("--window", type=float, nargs=2, default=(0.0, 1e9), help="seconds after the first frame")
    ap.add_argument("--stride", type=int, default=4, help="use every n-th scan of each LiDAR")
    ap.add_argument("--min-range", type=float, default=4.0); ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--cell", type=float, default=0.5); ap.add_argument("--max-slope", type=float, default=6.0)
    ap.add_argument("--output", required=True); ap.add_argument("--report", required=True)
    a = ap.parse_args()

    calib = json.load(open(a.calibration))
    M = {}
    for entry in calib["lidar"]:
        for topic, k in rm.KEYS.items():
            if k in entry:
                M[topic] = np.array(entry[k], dtype=float).reshape(4, 4)
    run = pathlib.Path(a.run)
    lines = (run / "frames.csv").read_text().splitlines()
    lt = np.array([float(v) for v in lines[1].split()[2:]]); lr = np.array([float(v) for v in lines[2].split()[2:]]).reshape(3, 3)
    rows = np.array([[float(v) for v in l.split(",")] for l in lines[4:]])
    stamps = rows[:, 0]; t0 = stamps[0]
    poses = rm.load_optimized(run, stamps)
    report = [f"calibration={a.calibration}", f"run={run}"]
    if poses is None:
        report.append("no usable trajectory_optimized.txt (needs an RTK-fused LIO run): parent kept")
        finish(a, report, None, calib, {}); return
    t_end = min(a.window[1], stamps[-1] - t0 - 0.5)
    # vehicle forward axis in the rear-axle frame (from the motion; independent of the frame convention)
    vel = np.gradient(poses.pos, stamps, axis=0); moving = np.linalg.norm(vel, axis=1) > 1.0
    v_imu = np.median(poses.rot[moving].inv().apply(vel[moving]), axis=0) if moving.sum() > 20 else np.array([0, 1.0, 0])
    fwd = lr.T @ (v_imu / np.linalg.norm(v_imu))

    topics = list(rm.KEYS)
    Qs, Ts, Ss = [], [], []
    count = {t: 0 for t in topics}; used = {t: 0 for t in topics}
    dataset = rm.Dataset.open(a.bag)
    for topic, data, _ in dataset.iter_messages(topics):
        topic = topic.split("/")[-1]
        if topic not in M:
            continue
        msg = deserialize_message(data, PointCloud2)
        h = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + a.lidar_time_offset
        if h - t0 < max(a.window[0], 0.5):
            continue
        if h - t0 > t_end:
            if h - t0 > t_end + 1:
                break
            continue
        count[topic] += 1
        if count[topic] % a.stride:
            continue
        arr = pc2.read_points(msg, field_names=["x", "y", "z", "timestamp"], skip_nans=True)
        P = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(float)
        ts = np.asarray(arr["timestamp"], dtype=float)
        if ts.max() > 1e6: ts = ts - ts.min()
        if ts.max() > 1.0: ts = ts * 1e-3 if ts.max() < 1e3 else ts * 1e-9
        Q = P @ M[topic][:3, :3].T + M[topic][:3, 3]                      # rear-axle frame
        rng = np.linalg.norm(Q[:, :2] - M[topic][:2, 3], axis=1)
        excl = (Q[:, 0] > -2.8) & (Q[:, 0] < 2.8) & (Q[:, 1] > -4.5) & (Q[:, 1] < 9.5)
        keep = (rng > a.min_range) & (rng < a.max_range) & ~excl
        if keep.sum() < 200:
            continue
        Q, ts = Q[keep], ts[keep]
        low = Q[:, 2] < np.percentile(Q[:, 2], 40) + 1.5                  # coarse ground band of this scan
        Q, ts = Q[low], ts[low]
        sub = rm.voxel_down(Q, 0.2)
        Qs.append(Q[sub]); Ts.append(h + ts[sub]); Ss.append(np.full(len(sub), topics.index(topic), np.int8))
        used[topic] += 1
    if not Qs:
        report.append("no scans in the window: parent kept"); finish(a, report, None, calib, {}); return
    Q = np.vstack(Qs); T = np.concatenate(Ts); S = np.concatenate(Ss)
    rot, pos = poses.at(T)
    Rm = rot.as_matrix()
    report.append(f"window {a.window[0]:.1f}..{t_end:.1f} s, stride {a.stride}, scans " + ", ".join(f"{t}:{used[t]}" for t in topics)
                  + f"; ground-band returns {len(Q)}")

    def world(corr):
        q = Q.copy()
        for k, (rv, tt) in corr.items():
            m = S == k
            c = M[topics[k]][:3, 3]
            q[m] = (q[m] - c) @ R.from_rotvec(rv).as_matrix().T + c + tt
        return np.einsum("nij,nj->ni", Rm, q @ lr.T + lt) + pos

    # lowest surface per cell (P5) from the uncorrected cloud; keep returns near it
    W = world({})
    key = np.floor(W[:, :2] / a.cell).astype(np.int64)
    _, inv = np.unique(key[:, 0] * 4000037 + key[:, 1], return_inverse=True)
    o = np.lexsort((W[:, 2], inv)); _, st, ct = np.unique(inv[o], return_index=True, return_counts=True)
    p5 = W[o, 2][st + (ct * 0.05).astype(int)]
    keep = (W[:, 2] > p5[inv] - 0.25) & (W[:, 2] < p5[inv] + 0.40)
    Q, T, S, Rm, pos, key = Q[keep], T[keep], S[keep], Rm[keep], pos[keep], key[keep]
    _, inv = np.unique(key[:, 0] * 4000037 + key[:, 1], return_inverse=True); ncell = inv.max() + 1

    ref = topics.index(rm.REFERENCE)
    targets = []
    for k, t in enumerate(topics):
        if k == ref or not (S == k).any():
            continue
        along = np.median((Q[S == k]) @ fwd)
        report.append(f"{t}: ground returns {int((S == k).sum())}, median along-track position {along:+.1f} m "
                      f"({'rear-facing -> solved' if along < 0 else 'kept (overlaps the reference, driven stage)'})")
        if along < 0:
            targets.append(k)
    corr = {k: (np.zeros(3), np.zeros(3)) for k in targets}
    tmid = 0.5 * (T.min() + T.max())

    def solve(sel_time, corr_in, iters):
        corr_l = {k: (v[0].copy(), v[1].copy()) for k, v in corr_in.items()}
        stats = {}
        for _ in range(iters):
            Wc = world(corr_l)
            res, slope, cnt = planes(Wc, inv, ncell, np.ones(len(Q), bool))
            flat = (slope < a.max_slope) & (cnt >= 30) & (np.abs(res) < 0.30) & sel_time
            for k in targets:
                m = flat & (S == k)
                if m.sum() < 2000:
                    continue
                rv, tt = corr_l[k]; c = M[topics[k]][:3, 3]
                d = (Q[m] - c) @ R.from_rotvec(rv).as_matrix().T
                rowz = Rm[m][:, 2, :] @ lr
                J = np.hstack([np.cross(d, rowz)[:, :2], rowz[:, 2:3]])
                r = res[m]; x = np.zeros(3)
                for _ in range(4):
                    e = r + J @ x; w = np.minimum(1.0, 0.04 / np.maximum(np.abs(e), 1e-9))
                    x = -np.linalg.solve(J.T @ (J * w[:, None]) + np.eye(3) * 1e-3, (J * w[:, None]).T @ r)
                corr_l[k] = (rv + np.array([x[0], x[1], 0.0]), tt + np.array([0.0, 0.0, x[2]]))
                stats[k] = (int(m.sum()), float(np.median(np.abs(r))), float(np.median(np.abs(r + J @ x))),
                            float(np.sqrt(np.mean((J @ np.array([corr_l[k][0][0], corr_l[k][0][1], corr_l[k][1][2]])) ** 2))))
        return corr_l, stats

    if not targets:
        report.append("no rear-facing LiDAR: parent kept"); finish(a, report, None, calib, {}); return
    corr, stats0 = solve(np.ones(len(Q), bool), corr, 4)
    first, _ = solve(T < tmid, corr, 2)
    second, _ = solve(T >= tmid, corr, 2)
    # residual statistics before / after, per range band
    res0, sl0, c0 = planes(world({}), inv, ncell, np.ones(len(Q), bool))
    res1, sl1, c1 = planes(world(corr), inv, ncell, np.ones(len(Q), bool))
    rng = np.linalg.norm(Q[:, :2], axis=1)
    out_corr, quality, apply = {}, {}, {}
    for k in targets:
        t = topics[k]; rv, tt = corr[k]
        rot_deg = float(np.degrees(np.linalg.norm(rv))); dz = float(tt[2])
        rep_rot = float(np.degrees(np.linalg.norm(first[k][0] - second[k][0]))); rep_z = float(abs(first[k][1][2] - second[k][1][2]))
        effect = stats0.get(k, (0, 0, 0, 0.0))[3]
        level, reasons = "ok", []
        if rot_deg > GATE_MAX_ROT_DEG or abs(dz) > GATE_MAX_TRANS_M:
            level = "fail"; reasons.append(f"correction {rot_deg:.2f} deg / {100*dz:+.1f} cm beyond {GATE_MAX_ROT_DEG} deg / {100*GATE_MAX_TRANS_M:.0f} cm")
        if rep_rot > REPEAT_ROT_DEG or rep_z > REPEAT_TRANS_M:
            level = "warn" if level == "ok" else level; reasons.append(f"halves differ by {rep_rot:.3f} deg / {100*rep_z:.1f} cm")
        bands = []
        for lo, hi in ((4, 10), (10, 20), (20, 40)):
            m = (S == k) & (rng >= lo) & (rng < hi) & (sl0 < a.max_slope) & (c0 >= 30) & (np.abs(res0) < 0.3)
            m1 = (S == k) & (rng >= lo) & (rng < hi) & (sl1 < a.max_slope) & (c1 >= 30) & (np.abs(res1) < 0.3)
            if m.sum() > 300 and m1.sum() > 300:
                bands.append(f"{lo}-{hi} m {100*np.median(res0[m]):+.1f}->{100*np.median(res1[m1]):+.1f}")
        f0 = (S == k) & (sl0 < a.max_slope) & (c0 >= 30) & (np.abs(res0) < 0.3)
        f1 = (S == k) & (sl1 < a.max_slope) & (c1 >= 30) & (np.abs(res1) < 0.3)
        n = int(f1.sum())
        before = float(np.median(np.abs(res0[f0]))) if f0.any() else np.nan
        after = float(np.median(np.abs(res1[f1]))) if f1.any() else np.nan
        report.append(f"{t}: correction rotvec x {np.degrees(rv[0]):+.3f} y {np.degrees(rv[1]):+.3f} deg (about the LiDAR, rear-axle frame), "
                      f"z {100*dz:+.2f} cm; own-ground change RMS {100*effect:.1f} cm; flat-ground |res| median {100*before:.2f}->{100*after:.2f} cm "
                      f"(n={n}); signed median by range: {', '.join(bands)}; halves: {rep_rot:.3f} deg / {100*rep_z:.1f} cm  [quality: {level}"
                      f"{'; ' + '; '.join(reasons) if reasons else ''}]")
        quality[t] = {"level": level, "reasons": reasons, "rotation_deg": round(rot_deg, 3), "z_cm": round(100 * dz, 2),
                      "effect_rms_cm": round(100 * effect, 2), "halves_rot_deg": round(rep_rot, 3), "halves_z_cm": round(100 * rep_z, 2),
                      "median_abs_res_before_cm": round(100 * before, 2), "median_abs_res_after_cm": round(100 * after, 2)}
        out_corr[t] = {"rotvec_deg": np.degrees(rv).round(4).tolist(), "trans_cm": (100 * tt).round(2).tolist(),
                       "about": "lidar position in rear-axle frame", "quality": quality[t]}
        if level != "fail" and effect >= MIN_EFFECT_M:
            apply[t] = (rv, tt)
        elif level != "fail":
            report.append(f"{t}: change below {100*MIN_EFFECT_M:.1f} cm RMS -> kept")
    finish(a, report, apply, calib, {"corrections": out_corr, "run": str(run), "targets": [topics[k] for k in targets]}, M)


def finish(a, report, apply, calib, meta, M=None):
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    if not apply:
        report.append("result: parent calibration kept (no significant ground-consensus correction)")
        pathlib.Path(a.report).write_text("\n".join(report) + "\n"); print("\n".join(report))
        print(json.dumps({"kept": True, "refinement": meta}, ensure_ascii=False))
        return
    out = json.loads(json.dumps(calib))
    for entry in out["lidar"]:
        for topic, k in rm.KEYS.items():
            if k in entry and topic in apply:
                rv, tt = apply[topic]; c = M[topic][:3, 3]
                C = np.eye(4); C[:3, :3] = R.from_rotvec(rv).as_matrix(); C[:3, 3] = c - C[:3, :3] @ c + tt
                entry[k] = [float(v) for v in (C @ M[topic]).reshape(-1)]
    out["calibration_type"] = calib.get("calibration_type", "") + "+ground_consensus_tilt_z"
    out.setdefault("refinement", {})["ground_consensus"] = {**meta, "applied": sorted(apply)}
    report.append("result: applied to " + ", ".join(sorted(apply)))
    pathlib.Path(a.report).write_text("\n".join(report) + "\n"); print("\n".join(report))
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True); json.dump(out, open(a.output, "w"), indent=2)
    print("written", a.output)


if __name__ == "__main__":
    main()
