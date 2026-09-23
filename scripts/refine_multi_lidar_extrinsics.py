#!/usr/bin/env python3
"""Data-driven refinement of the manual multi-LiDAR extrinsics (lidar -> rear axle).

Observable degrees of freedom only. Ground overlap between LiDARs observes the relative
roll/pitch/height of each LiDAR; yaw and horizontal offsets are solved only if enough
vertical planar overlap exists (otherwise they are kept from the manual calibration).
Stationary start frames (no ego-motion, aggregated) are used for the side LiDARs; the back
LiDAR only overlaps the sides while driving, so it uses per-frame matches from a moving
segment (same scan, same vehicle pose; ground z is insensitive to the <0.5 m intra-scan
travel). The correction is a small rigid transform about the LiDAR's own position in the
rear-axle frame, applied on top of the manual matrix; the original JSON is never modified.

Usage (container):
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/refine_multi_lidar_extrinsics.py \
      --calibration /home/project/data/haibo/wuhu_livo/sensor-imu_calibration/perception_truck29.json \
      --bag results/prepared/wuhu_livo --output config/perception_truck29_refined.json \
      --report results/calibration_refinement/report.txt
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


def read_frames(bag, first, count):
    """Frames [first, first+count) of every LiDAR topic, read directly from the raw recording."""
    dataset = Dataset.open(bag)
    topics = list(KEYS)
    clouds = {t: [] for t in topics}; counts = {t: 0 for t in topics}
    for topic, data, _ in dataset.iter_messages(topics):
        topic = topic.split("/")[-1] if topic.split("/")[-1] in counts else topic
        if topic not in counts: continue
        counts[topic] += 1
        if counts[topic] <= first: continue
        if counts[topic] > first + count:
            if all(c > first + count for c in counts.values()): break
            continue
        msg = deserialize_message(data, PointCloud2)
        arr = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        clouds[topic].append(np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(float))
    return clouds  # list of frames per topic


def voxel_down(P, size):
    key = np.floor(P / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[np.sort(idx)]


def clean(Q, rmin=3.0, rmax=60.0):
    r = np.hypot(Q[:, 0], Q[:, 1])
    body = (Q[:, 0] > -2.8) & (Q[:, 0] < 2.8) & (Q[:, 1] > -4.5) & (Q[:, 1] < 9.5)
    return Q[(r > rmin) & (r < rmax) & ~body & np.isfinite(Q).all(1)]


def stationary_check(frames, rmin=3.0, rmax=60.0, voxel=0.2):
    """Median / P90 nearest-neighbour displacement between the first and the last frame of a LiDAR.
    Identical scenes from a standing vehicle differ only by the scan pattern (~0.1 m median here);
    any ego-motion adds its travel distance directly."""
    def prep(P):
        r = np.linalg.norm(P, axis=1); P = P[(r > rmin) & (r < rmax)]
        return voxel_down(P, voxel)
    A, B = prep(frames[0]), prep(frames[-1])
    d, _ = cKDTree(A).query(B)
    return {"median_m": float(np.median(d)), "p90_m": float(np.percentile(d, 90))}


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


def solve(pairs, center, solve_yaw_xy, iterations=25, max_dist=0.6, huber=0.05,
          prior_rot_deg=1.0, prior_trans_m=0.10):
    """pairs: list of (Target, B points). Correction: p' = Rc (p - center) + center + t."""
    rot = np.zeros(3); trans = np.zeros(3)
    active = np.array([True, True, solve_yaw_xy, solve_yaw_xy, solve_yaw_xy, True])  # rx, ry, rz, tx, ty, tz
    stats = None
    for it in range(iterations):
        Rm = R.from_rotvec(rot).as_matrix()
        H = np.zeros((6, 6)); g = np.zeros(6); rows = []
        for target, B in pairs:
            Bt = (B - center) @ Rm.T + center + trans
            ok, n, q = target.match(Bt, max_dist)
            if not ok.any(): continue
            p = Bt[ok]; r = np.einsum("ni,ni->n", n, p - q)
            if not solve_yaw_xy:
                keep = np.abs(n[:, 2]) > 0.8   # ground only observes rx, ry, tz
                p, n, q, r = p[keep], n[keep], q[keep], r[keep]
            w = np.minimum(1.0, huber / np.maximum(np.abs(r), 1e-9))
            J = np.hstack([np.cross(p - center, n), n])
            Jw = J * w[:, None]
            H += J.T @ Jw; g += Jw.T @ r
            rows.append(r)
        if not rows: return None
        r_all = np.concatenate(rows)
        if len(r_all) < 200: return None
        # weak prior on the correction (keeps unobservable directions at zero)
        prior = np.diag([1 / np.radians(prior_rot_deg) ** 2] * 3 + [1 / prior_trans_m ** 2] * 3)
        x = np.concatenate([rot, trans])
        H += prior; g += prior @ x
        Ha = H[np.ix_(active, active)]; ga = g[active]
        step = np.zeros(6); step[active] = -np.linalg.solve(Ha, ga)
        rot = rot + step[:3]; trans = trans + step[3:]
        stats = (it + 1, len(r_all), float(np.median(np.abs(r_all))))
        if np.linalg.norm(step[:3]) < 1e-6 and np.linalg.norm(step[3:]) < 1e-5: break
    return rot, trans, stats


def residuals(pairs, center, rot, trans, max_dist=0.6):
    Rm = R.from_rotvec(rot).as_matrix(); out = []
    for target, B in pairs:
        Bt = (B - center) @ Rm.T + center + trans
        ok, n, q = target.match(Bt, max_dist)
        r = np.einsum("ni,ni->n", n, Bt[ok] - q); out.append(np.column_stack([r, np.abs(n[:, 2])]))
    a = np.vstack(out); g = a[:, 1] > 0.8; v = a[:, 1] < 0.3
    return dict(n=len(a), med=float(np.median(np.abs(a[:, 0]))), n_ground=int(g.sum()),
                med_ground=float(np.median(np.abs(a[g, 0]))) if g.any() else np.nan,
                n_vertical=int(v.sum()), med_vertical=float(np.median(np.abs(a[v, 0]))) if v.any() else np.nan)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", required=True); ap.add_argument("--bag", required=True)
    ap.add_argument("--output", required=True); ap.add_argument("--report", required=True)
    ap.add_argument("--stationary", type=int, nargs=2, default=(2, 20), help="first frame, count (aggregated)")
    ap.add_argument("--moving", type=int, nargs=2, default=(300, 40), help="first frame, count (per-frame) for the back LiDAR")
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--min-vertical", type=int, default=400, help="vertical planar matches needed to solve yaw/xy")
    ap.add_argument("--max-still-shift", type=float, default=0.30,
                    help="max median scan displacement (m) between first and last stationary frame; a standing truck gives ~0.1 m from the scan pattern alone, 0.5 m/s of motion gives >0.5 m")
    a = ap.parse_args()
    calib = json.load(open(a.calibration))
    M = {}
    for entry in calib["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: M[topic] = np.array(entry[k], dtype=float).reshape(4, 4)
    to_rear = lambda t, P: P @ M[t][:3, :3].T + M[t][:3, 3]
    lines = [f"calibration={a.calibration}", f"bag={a.bag}", f"reference={REFERENCE}",
             f"stationary frames {a.stationary[0]}..{a.stationary[0]+a.stationary[1]} (aggregated) for side LiDARs; moving frames {a.moving[0]}..{a.moving[0]+a.moving[1]} (per frame) for back LiDAR"]
    stat = read_frames(a.bag, *a.stationary)
    # The aggregation below assumes the vehicle does not move during the stationary frames; a recording
    # that starts while driving silently biases roll/pitch/z (motion-blurred ground), so verify it.
    still = stationary_check(stat[REFERENCE])
    lines.append(f"stationary check ({REFERENCE}, first vs last frame of {a.stationary[1]}): scan displacement median "
                 f"{100*still['median_m']:.1f} cm / P90 {100*still['p90_m']:.1f} cm (limit {100*a.max_still_shift:.0f} cm) -> "
                 + ("stationary" if still["median_m"] <= a.max_still_shift else "MOVING"))
    if still["median_m"] > a.max_still_shift:
        msg = (f"ground stage aborted: frames {a.stationary[0]}..{a.stationary[0]+a.stationary[1]} are not stationary "
               f"(reference scan displacement {100*still['median_m']:.0f} cm). The stage needs ~{0.1*a.stationary[1]:.0f} s of standstill; "
               "use a recording that starts before the vehicle moves off, or point --stationary at a standstill.")
        lines.append(msg); pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(a.report).write_text("\n".join(lines) + "\n"); print("\n".join(lines)); sys.exit(3)
    agg = {t: voxel_down(clean(to_rear(t, np.vstack(v))), a.voxel) for t, v in stat.items()}
    for t in KEYS: lines.append(f"{t}: stationary aggregated points {len(agg[t])}")
    refined = {REFERENCE: M[REFERENCE].copy()}; corrections = {}
    ref_target = Target(agg[REFERENCE])
    side_targets = {}
    for t in ("front_right_lidar", "front_left_lidar"):
        center = M[t][:3, 3]
        pairs = [(ref_target, agg[t])]
        before = residuals(pairs, center, np.zeros(3), np.zeros(3))
        solve_yaw = before["n_vertical"] >= a.min_vertical
        out = solve(pairs, center, solve_yaw)
        if out is None: lines.append(f"{t}: insufficient overlap; manual matrix kept"); refined[t] = M[t].copy(); continue
        rot, trans, st = out
        after = residuals(pairs, center, rot, trans)
        C = np.eye(4); C[:3, :3] = R.from_rotvec(rot).as_matrix(); C[:3, 3] = center - C[:3, :3] @ center + trans
        refined[t] = C @ M[t]
        corrections[t] = dict(rotvec_deg=np.degrees(rot).tolist(), trans_cm=(100 * trans).tolist(), yaw_xy_solved=bool(solve_yaw), about="lidar position in rear-axle frame")
        lines.append(f"{t}: yaw/xy solved={solve_yaw}; matches {before['n']}->{after['n']}; |res| median {before['med']*100:.2f}->{after['med']*100:.2f} cm; "
                     f"ground {before['med_ground']*100:.2f}->{after['med_ground']*100:.2f} cm (n={after['n_ground']}); vertical {before['med_vertical']*100:.2f}->{after['med_vertical']*100:.2f} cm (n={after['n_vertical']}); "
                     f"correction rot deg (x,y,z)=({np.degrees(rot[0]):+.3f},{np.degrees(rot[1]):+.3f},{np.degrees(rot[2]):+.3f}) trans cm=({100*trans[0]:+.1f},{100*trans[1]:+.1f},{100*trans[2]:+.1f}); iters {st[0]}")
        Bt = (agg[t] - center) @ C[:3, :3].T + center + trans
        side_targets[t] = Target(Bt)
    # back LiDAR: per-frame matches against the (refined) side LiDARs in the moving segment
    t = "back_lidar"; center = M[t][:3, 3]
    mov = read_frames(a.bag, *a.moving)
    pairs = []
    for f in range(min(len(v) for v in mov.values())):
        B = voxel_down(clean(to_rear(t, mov[t][f])), a.voxel)
        for s in ("front_right_lidar", "front_left_lidar"):
            if s not in refined: continue
            Cs = refined[s] @ np.linalg.inv(M[s])   # correction of side lidar in rear frame
            S = clean(to_rear(s, mov[s][f])); S = S @ Cs[:3, :3].T + Cs[:3, 3]
            S = voxel_down(S, a.voxel)
            if len(S) > 500 and len(B) > 100: pairs.append((Target(S), B))
    if pairs:
        before = residuals(pairs, center, np.zeros(3), np.zeros(3))
        # Intra-scan ego-motion (~0.5 m per scan while driving) corrupts yaw/xy evidence from
        # vertical surfaces; only the ground-observed roll/pitch/height is trusted here.
        solve_yaw = False
        out = solve(pairs, center, solve_yaw)
        if out is None:
            lines.append(f"{t}: insufficient overlap; manual matrix kept"); refined[t] = M[t].copy()
        else:
            rot, trans, st = out; after = residuals(pairs, center, rot, trans)
            C = np.eye(4); C[:3, :3] = R.from_rotvec(rot).as_matrix(); C[:3, 3] = center - C[:3, :3] @ center + trans
            refined[t] = C @ M[t]
            corrections[t] = dict(rotvec_deg=np.degrees(rot).tolist(), trans_cm=(100 * trans).tolist(), yaw_xy_solved=bool(solve_yaw), about="lidar position in rear-axle frame")
            lines.append(f"{t}: (moving, per-frame vs refined sides) yaw/xy solved={solve_yaw}; matches {before['n']}->{after['n']}; |res| median {before['med']*100:.2f}->{after['med']*100:.2f} cm; "
                         f"ground {before['med_ground']*100:.2f}->{after['med_ground']*100:.2f} cm (n={after['n_ground']}); vertical n={after['n_vertical']}; "
                         f"correction rot deg (x,y,z)=({np.degrees(rot[0]):+.3f},{np.degrees(rot[1]):+.3f},{np.degrees(rot[2]):+.3f}) trans cm=({100*trans[0]:+.1f},{100*trans[1]:+.1f},{100*trans[2]:+.1f}); iters {st[0]}")
    else:
        lines.append(f"{t}: no moving-frame overlap; manual matrix kept"); refined[t] = M[t].copy()
    out = json.loads(json.dumps(calib))
    for entry in out["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: entry[k] = [float(v) for v in refined[topic].reshape(-1)]
    out["calibration_type"] = calib.get("calibration_type", "") + "+data_refined_roll_pitch_z"
    out["refinement"] = dict(reference=REFERENCE, corrections=corrections, source=str(a.bag),
                             stationary_frames=list(a.stationary), moving_frames=list(a.moving))
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(a.output, "w"), indent=2)
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    pathlib.Path(a.report).write_text("\n".join(lines) + "\n")
    print("\n".join(lines)); print("written", a.output)


if __name__ == "__main__":
    main()
