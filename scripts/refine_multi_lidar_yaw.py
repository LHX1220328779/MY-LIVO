#!/usr/bin/env python3
"""Stage-2 multi-LiDAR extrinsic refinement (all 6 DoF) using a driven window and the run's own
native LIO poses. Each LiDAR's raw returns are linearly deskewed with the per-frame poses and
accumulated in the world frame; every non-reference LiDAR is then registered (robust point-to-
plane ICP, correction applied in the vehicle frame about the LiDAR position) against the
reference LiDAR's accumulated cloud. Vertical structures (poles, plates, berms, walls) observe
yaw/xy; ground observes roll/pitch/z. Output: a new calibration JSON; inputs are never modified.

Usage (container):
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/refine_multi_lidar_yaw.py --calibration config/perception_truck29_refined.json \
      --bag results/prepared/wuhu_livo --run <fusion_run_dir> --window 8 40 \
      --lidar-time-offset -0.125 --output config/perception_truck29_refined_v2.json --report results/calibration_refinement/report_v2_yaw.txt
"""
import argparse, json, pathlib, numpy as np
import sys
from pathlib import Path as _LivoPath

for _cand in (_LivoPath(__file__).resolve().parents[1] / "tools", _LivoPath("/home/project/MY-LIVO/tools")):
    if (_cand / "livo" / "rosenv.py").exists():
        sys.path.insert(0, str(_cand))
        break
from livo import rosenv as _rosenv  # noqa: E402
_rosenv.ensure()
from livo.dataset import Dataset  # noqa: E402
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

KEYS = {"front_right_lidar": "transform_matrix_lidar_1", "back_lidar": "transform_matrix_lidar_2",
        "front_left_lidar": "transform_matrix_lidar_3", "front_lidar": "transform_matrix_lidar_6"}
REFERENCE = "front_lidar"


def voxel_down(P, size):
    key = np.floor(P / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return idx


class Target:
    def __init__(self, A, k=12):
        self.A = A; self.tree = cKDTree(A)
        d, idx = self.tree.query(A, k=k); nbr = A[idx]; c = nbr.mean(1)
        cov = np.einsum("nki,nkj->nij", nbr - c[:, None], nbr - c[:, None]) / k
        w, v = np.linalg.eigh(cov)
        self.normals = v[:, :, 0]
        self.planar = (w[:, 0] < 0.0025) & (w[:, 1] > 4 * w[:, 0] + 1e-9)

    def match(self, B, max_dist):
        dist, j = self.tree.query(B, distance_upper_bound=max_dist)
        ok = np.isfinite(dist); ok[ok] &= self.planar[j[ok]]
        return ok, self.normals[j[ok]], self.A[j[ok]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", required=True); ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True, help="fusion run dir with frames.csv (native poses)")
    ap.add_argument("--window", type=float, nargs=2, default=(8.0, 40.0), help="seconds after first frame")
    ap.add_argument("--lidar-time-offset", type=float, default=-0.125)
    ap.add_argument("--output", required=True); ap.add_argument("--report", required=True)
    ap.add_argument("--voxel", type=float, default=0.2); ap.add_argument("--max-range", type=float, default=60.0)
    ap.add_argument("--vertical-only", action="store_true", help="use only vertical planar matches (|n_z|<0.3): solves yaw/x/y, keeps roll/pitch/z")
    ap.add_argument("--prior-rot-deg", type=float, default=0.5); ap.add_argument("--prior-trans-m", type=float, default=0.10)
    a = ap.parse_args()
    calib = json.load(open(a.calibration))
    M = {}
    for entry in calib["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: M[topic] = np.array(entry[k], dtype=float).reshape(4, 4)
    run = pathlib.Path(a.run)
    lines = (run / "frames.csv").read_text().splitlines()
    lt = np.array([float(v) for v in lines[1].split()[2:]]); lr = np.array([float(v) for v in lines[2].split()[2:]]).reshape(3, 3)
    rows = np.array([[float(v) for v in l.split(",")] for l in lines[4:]])
    stamps = rows[:, 0]; begins = rows[:, 1]; t0 = stamps[0]
    rot = R.from_quat(rows[:, 7:11]); pos = rows[:, 4:7]

    def pose_at(t):
        j = np.clip(np.searchsorted(stamps, t), 1, len(stamps) - 1)
        ta, tb = stamps[j - 1], stamps[j]; r = np.clip((t - ta) / (tb - ta), 0, 1)
        ra, rb = rot[j - 1], rot[j]
        rr = ra * R.from_rotvec((ra.inv() * rb).as_rotvec() * r[:, None])
        return rr, pos[j - 1] * (1 - r)[:, None] + pos[j] * r[:, None]

    dataset = Dataset.open(a.bag)  # raw split recording or single bag, merged in time order
    topics = list(KEYS)
    body = {t: [] for t in topics}; world_ref = {t: [] for t in topics}
    n_used = {t: 0 for t in topics}
    for topic, data, _ in dataset.iter_messages(topics):
        topic = topic.split("/")[-1] if topic.split("/")[-1] in body else topic
        if topic not in body: continue
        msg = deserialize_message(data, PointCloud2)
        h = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + a.lidar_time_offset  # LIO time base (scan start)
        if h - t0 < a.window[0] - 0.2: continue
        if h - t0 > a.window[1]: 
            if all(n_used[t] > 0 for t in topics) and h - t0 > a.window[1] + 1: break
            continue
        arr = pc2.read_points(msg, field_names=["x", "y", "z", "timestamp"], skip_nans=True)
        P = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(float)
        ts = np.asarray(arr["timestamp"], dtype=float)
        if ts.max() > 1e6: ts = ts - ts.min()          # absolute -> relative
        if ts.max() > 1.0: ts = ts * 1e-3 if ts.max() < 1e3 else ts * 1e-9  # ms or ns
        r = np.linalg.norm(P, axis=1); keep = (r > 3.0) & (r < a.max_range)
        P, ts = P[keep], ts[keep]
        if len(P) < 100: continue
        sub = voxel_down(P, 0.1)
        P, ts = P[sub], ts[sub]
        Q = P @ M[topic][:3, :3].T + M[topic][:3, 3]                  # rear-axle frame
        excl = (Q[:, 0] > -2.8) & (Q[:, 0] < 2.8) & (Q[:, 1] > -4.5) & (Q[:, 1] < 9.5)
        Q, P, ts = Q[~excl], P[~excl], ts[~excl]
        B = Q @ lr.T + lt                                             # IMU body frame
        rr, pp = pose_at(h + ts)                                      # per-point pose (linear deskew)
        W = rr.apply(B) + pp
        body[topic].append((P, ts, h)); world_ref[topic].append(W); n_used[topic] += 1
    report = [f"calibration={a.calibration}", f"run={run}", f"window {a.window[0]}..{a.window[1]} s, lidar_time_offset {a.lidar_time_offset}",
              "scans used: " + ", ".join(f"{t}:{n_used[t]}" for t in topics)]
    ref_world = np.vstack(world_ref[REFERENCE]); ref_world = ref_world[voxel_down(ref_world, a.voxel)]
    target = Target(ref_world)
    report.append(f"reference {REFERENCE}: {len(ref_world)} world points")
    refined = {REFERENCE: M[REFERENCE].copy()}; corrections = {}
    for t in ("front_right_lidar", "front_left_lidar", "back_lidar"):
        if not body[t]: refined[t] = M[t].copy(); report.append(f"{t}: no scans"); continue
        center = M[t][:3, 3]
        # per point: rear-axle coordinates Q (with current calibration), and pose (rr, pp)
        Qs, Rs, Ps = [], [], []
        for P, ts, h in body[t]:
            Q = P @ M[t][:3, :3].T + M[t][:3, 3]
            rr, pp = pose_at(h + ts)
            Qs.append(Q); Rs.append(rr.as_matrix()); Ps.append(pp)
        Q = np.vstack(Qs); Rm = np.vstack(Rs); Pw = np.vstack(Ps)
        sub = voxel_down(Q, 0.15)   # subsample in vehicle frame keeps far structure
        Q, Rm, Pw = Q[sub], Rm[sub], Pw[sub]
        # Rm is (N*3,3) stacked; reshape
        Rm = Rm.reshape(-1, 3, 3)
        prior = np.diag([1 / np.radians(a.prior_rot_deg) ** 2] * 3 + [1 / a.prior_trans_m ** 2] * 3)
        active = np.array([not a.vertical_only, not a.vertical_only, True, True, True, not a.vertical_only])
        x = np.zeros(6); stats = None
        for it in range(25):
            Rc = R.from_rotvec(x[:3]).as_matrix()
            Qc = (Q - center) @ Rc.T + center + x[3:]
            Bc = Qc @ lr.T + lt
            W = np.einsum("nij,nj->ni", Rm, Bc) + Pw
            ok, n, q = target.match(W, 0.6)
            if a.vertical_only:
                keep = np.abs(n[:, 2]) < 0.3
                idx_ok = np.flatnonzero(ok); ok = np.zeros_like(ok); ok[idx_ok[keep]] = True; n, q = n[keep], q[keep]
            if ok.sum() < 300: stats = None; break
            r = np.einsum("ni,ni->n", n, W[ok] - q)
            w = np.minimum(1.0, 0.05 / np.maximum(np.abs(r), 1e-9))
            # d W / d(rotvec about center) = R_i * lr * (-[Qc-center]x) ; d W / d t = R_i * lr
            RL = np.einsum("nij,jk->nik", Rm[ok], lr)
            dq = Qc[ok] - center
            J_rot = -np.einsum("nij,njk->nik", RL, np.stack([np.zeros_like(dq[:, 0]), -dq[:, 2], dq[:, 1], dq[:, 2], np.zeros_like(dq[:, 0]), -dq[:, 0], -dq[:, 1], dq[:, 0], np.zeros_like(dq[:, 0])], 1).reshape(-1, 3, 3))
            J = np.hstack([np.einsum("ni,nij->nj", n, J_rot), np.einsum("ni,nij->nj", n, RL)])
            Jw = J * w[:, None]
            H = J.T @ Jw + prior; g = Jw.T @ r + prior @ x
            step = np.zeros(6); step[active] = -np.linalg.solve(H[np.ix_(active, active)], g[active])
            x = x + step
            gnd = np.abs(n[:, 2]) > 0.8; vert = np.abs(n[:, 2]) < 0.3
            stats = (it + 1, int(ok.sum()), float(np.median(np.abs(r))), int(gnd.sum()), float(np.median(np.abs(r[gnd]))) if gnd.any() else np.nan,
                     int(vert.sum()), float(np.median(np.abs(r[vert]))) if vert.any() else np.nan)
            if it == 0: first = stats
            if np.linalg.norm(step[:3]) < 1e-6 and np.linalg.norm(step[3:]) < 1e-5: break
        if stats is None: refined[t] = M[t].copy(); report.append(f"{t}: insufficient matches"); continue
        C = np.eye(4); C[:3, :3] = R.from_rotvec(x[:3]).as_matrix(); C[:3, 3] = center - C[:3, :3] @ center + x[3:]
        refined[t] = C @ M[t]
        corrections[t] = dict(rotvec_deg=np.degrees(x[:3]).tolist(), trans_cm=(100 * x[3:]).tolist(), about="lidar position in rear-axle frame")
        report.append(f"{t}: matches {first[1]}->{stats[1]} |res| median {100*first[2]:.2f}->{100*stats[2]:.2f} cm; ground {100*first[4]:.2f}->{100*stats[4]:.2f} (n={stats[3]}); vertical {100*first[6]:.2f}->{100*stats[6]:.2f} (n={stats[5]}); "
                      f"correction rot deg (x,y,z)=({np.degrees(x[0]):+.3f},{np.degrees(x[1]):+.3f},{np.degrees(x[2]):+.3f}) trans cm=({100*x[3]:+.1f},{100*x[4]:+.1f},{100*x[5]:+.1f}); iters {stats[0]}")
    out = json.loads(json.dumps(calib))
    for entry in out["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: entry[k] = [float(v) for v in refined[topic].reshape(-1)]
    tag = "+driven_window_vertical_yaw_xy_refinement" if a.vertical_only else "+driven_window_6dof_refinement"
    out["calibration_type"] = calib.get("calibration_type", "") + tag
    out.setdefault("refinement", {})["stage3_vertical" if a.vertical_only else "stage2_6dof"] = dict(reference=REFERENCE, corrections=corrections, run=str(run), window=list(a.window), prior=[a.prior_rot_deg, a.prior_trans_m])
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True); json.dump(out, open(a.output, "w"), indent=2)
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True); pathlib.Path(a.report).write_text("\n".join(report) + "\n")
    print("\n".join(report)); print("written", a.output)


if __name__ == "__main__":
    main()

