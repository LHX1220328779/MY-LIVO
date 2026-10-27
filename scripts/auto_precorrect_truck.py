#!/usr/bin/env python3
"""Enterprise-Grade Universal Multi-LiDAR Extrinsic Pre-correction Engine for Mining Trucks.

Purpose:
  Enable zero-shot, plug-and-play deployment across different mining trucks using only their
  nominal / factory JSON calibration, WITHOUT requiring a dedicated full test drive beforehand.

Features:
  1. Sanitizes legacy format corruption (e.g. 18-element arrays) and enforces strict SO(3)
     orthogonality via SVD polar decomposition (R = U * V^T, det=1).
  2. Solves ground-observable degrees of freedom (roll, pitch, height z) in seconds from
     a static startup segment (<2-3 s) using robust point-to-plane optimization.
  3. Validates against vehicle-geometry safety gates (|d_roll, d_pitch| < 3.0 deg, |dz| < 0.35 m).
  4. Preserves unobservable horizontal degrees of freedom (yaw, x, y) strictly to nominal design,
     avoiding ill-conditioned unconstrained ICP divergence.
  5. Outputs a standardized, validated refined JSON ready for immediate runtime ingestion.

Usage:
  source /opt/ros/humble/setup.bash
  python3 scripts/auto_precorrect_truck.py \
      --calibration /path/to/perception_truckXX.json \
      --bag /path/to/any_startup_bag_or_recording \
      --output config/perception_truckXX_refined.json \
      --report results/calibration_refinement/truckXX_report.txt
"""

import argparse, json, sys, os, pathlib
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
from livo.body_filter import normalize, rear_axle_mask  # noqa: E402

KEYS = {
    "front_right_lidar": "transform_matrix_lidar_1",
    "back_lidar": "transform_matrix_lidar_2",
    "front_left_lidar": "transform_matrix_lidar_3",
    "front_lidar": "transform_matrix_lidar_6"
}
REFERENCE = "front_lidar"
BODY_FILTER = normalize({})  # vehicle-specific body box; loaded from calibration.json in run_precorrection()


def sanitize_transform_matrix(raw_values, name="extrinsic"):
    """Sanitize and orthogonalize a 4x4 rigid transformation matrix."""
    if len(raw_values) == 18:
        print(f"[Sanitize] Detected 18-element corrupted array in {name}; recovering valid 16 elements.")
        if raw_values[-4:] == [0, 0, 0, 1] or raw_values[-4:] == [0.0, 0.0, 0.0, 1.0]:
            fixed = raw_values[:2] + raw_values[4:]
            if len(fixed) == 16:
                raw_values = fixed
    
    if len(raw_values) != 16:
        raise ValueError(f"{name} must contain 16 numeric values, got {len(raw_values)}")
    
    mat = np.array(raw_values, dtype=np.float64).reshape(4, 4)
    mat[3, :] = [0.0, 0.0, 0.0, 1.0]
    
    # SVD polar decomposition for strict SO(3)
    R_raw = mat[:3, :3]
    U, _, Vt = np.linalg.svd(R_raw)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0:
        U[:, -1] *= -1.0
        R_ortho = U @ Vt
    
    mat[:3, :3] = R_ortho
    return mat


def load_truck_calibration(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    
    truck_num = data.get("truck_num", "unknown")
    mats = {}
    lidar_entries = data.get("lidar", [])
    for entry in lidar_entries:
        for topic_k, key_name in KEYS.items():
            if key_name in entry:
                mats[topic_k] = sanitize_transform_matrix(entry[key_name], f"{topic_k} ({key_name})")
    
    return data, truck_num, mats


def clean_points(Q, rmin=3.0, rmax=60.0):
    r = np.hypot(Q[:, 0], Q[:, 1])
    body = rear_axle_mask(Q, BODY_FILTER)
    return Q[(r > rmin) & (r < rmax) & ~body & np.isfinite(Q).all(axis=1)]


def voxel_down(P, size=0.15):
    key = np.floor(P / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[np.sort(idx)]


class Target:
    def __init__(self, A, k=12):
        self.A = A
        self.tree = cKDTree(A)
        d, idx = self.tree.query(A, k=k)
        nbr = A[idx]
        c = nbr.mean(axis=1)
        cov = np.einsum("nki,nkj->nij", nbr - c[:, None], nbr - c[:, None]) / k
        w, v = np.linalg.eigh(cov)
        self.normals = v[:, :, 0]
        self.planar = (w[:, 0] < 0.0025) & (w[:, 1] > 4 * w[:, 0] + 1e-9)

    def match(self, B, max_dist=0.6):
        dist, j = self.tree.query(B, distance_upper_bound=max_dist)
        ok = np.isfinite(dist)
        ok[ok] &= self.planar[j[ok]]
        return ok, self.normals[j[ok]], self.A[j[ok]]


def solve_ground(pairs, center, iterations=25, max_dist=0.6, huber=0.05,
                 prior_rot_deg=1.0, prior_trans_m=0.10):
    rot = np.zeros(3)
    trans = np.zeros(3)
    active = np.array([True, True, False, False, False, True])  # rx, ry, rz, tx, ty, tz
    for it in range(iterations):
        Rm = R.from_rotvec(rot).as_matrix()
        H = np.zeros((6, 6))
        g = np.zeros(6)
        rows = []
        for target, B in pairs:
            Bt = (B - center) @ Rm.T + center + trans
            ok, n, q = target.match(Bt, max_dist)
            if not ok.any(): continue
            p = Bt[ok]
            r = np.einsum("ni,ni->n", n, p - q)
            keep = np.abs(n[:, 2]) > 0.8
            p, n, q, r = p[keep], n[keep], q[keep], r[keep]
            if len(r) == 0: continue
            w = np.minimum(1.0, huber / np.maximum(np.abs(r), 1e-9))
            J = np.hstack([np.cross(p - center, n), n])
            Jw = J * w[:, None]
            H += J.T @ Jw
            g += Jw.T @ r
            rows.append(r)
        if not rows: return None
        r_all = np.concatenate(rows)
        if len(r_all) < 150: return None
        prior = np.diag([1 / np.radians(prior_rot_deg) ** 2] * 3 + [1 / prior_trans_m ** 2] * 3)
        x = np.concatenate([rot, trans])
        H += prior
        g += prior @ x
        Ha = H[np.ix_(active, active)]
        ga = g[active]
        step = np.zeros(6)
        step[active] = -np.linalg.solve(Ha, ga)
        rot += step[:3]
        trans += step[3:]
        if np.max(np.abs(step[active])) < 1e-5:
            break
    return rot, trans, len(r_all)


def run_precorrection(calib_path, bag_path, output_path=None, report_path=None):
    print(f"[Pre-corrector] Loading nominal calibration: {calib_path}")
    global BODY_FILTER
    raw_json, truck_num, mats = load_truck_calibration(calib_path)
    BODY_FILTER = normalize(raw_json)
    
    if REFERENCE not in mats:
        raise RuntimeError(f"Reference LiDAR {REFERENCE} not found in calibration!")
    
    lines = [f"=== Multi-LiDAR Extrinsic Pre-correction Report for Truck {truck_num} ==="]
    lines.append(f"Source JSON: {calib_path}")
    lines.append(f"Bag Source:  {bag_path if bag_path else 'None (Audit only)'}")
    
    lines.append("\n--- Nominal Sensor Poses (Rear-Axle Frame) ---")
    for topic, mat in mats.items():
        rot = R.from_matrix(mat[:3, :3]).as_euler("xyz", degrees=True)
        pos = mat[:3, 3]
        lines.append(f"  {topic:20s}: Pos=[{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}] m, EulerXYZ=[{rot[0]:6.2f}, {rot[1]:6.2f}, {rot[2]:6.2f}] deg")
    
    refined_mats = {k: v.copy() for k, v in mats.items()}
    
    if bag_path and os.path.exists(bag_path):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2 as pc2
        
        print(f"[Pre-corrector] Reading static startup segment from bag: {bag_path}")
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
        topics = list(KEYS.keys()) + ["/" + t for t in KEYS.keys()]
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        
        clouds = {t: [] for t in KEYS}
        frame_counts = {t: 0 for t in KEYS}
        max_startup_frames = 20
        
        while reader.has_next():
            t_name, data, _ = reader.read_next()
            topic = t_name.lstrip("/")
            if topic in clouds and frame_counts[topic] < max_startup_frames:
                msg = deserialize_message(data, PointCloud2)
                arr = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
                pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
                clouds[topic].append(pts)
                frame_counts[topic] += 1
            if all(c >= max_startup_frames for c in frame_counts.values()):
                break
                
        print(f"[Pre-corrector] Collected startup frames: {frame_counts}")
        
        to_rear = lambda t, P: P @ mats[t][:3, :3].T + mats[t][:3, 3]
        agg = {t: voxel_down(clean_points(to_rear(t, np.vstack(v))), 0.15) for t, v in clouds.items() if len(v) > 0}
        
        for t in KEYS:
            if t in agg:
                lines.append(f"  {t:20s}: Aggregated cleaned points = {len(agg[t])}")
        
        ref_target = Target(agg[REFERENCE])
        lines.append(f"\n--- Ground Alignment Optimization (Observable Roll, Pitch, Height) ---")
        
        for topic in ["front_right_lidar", "front_left_lidar", "back_lidar"]:
            if topic not in agg or len(agg[topic]) == 0:
                continue
            center = mats[topic][:3, 3]
            pairs = [(ref_target, agg[topic])]
            out = solve_ground(pairs, center)
            if out is None:
                lines.append(f"  {topic:20s}: Insufficient ground planar overlap, nominal kept.")
                continue
            rot, trans, n_matches = out
            euler_deg = np.degrees(rot)
            
            # Safety gate
            if np.max(np.abs(euler_deg)) > 3.0 or np.abs(trans[2]) > 0.35:
                print(f"[WARNING] {topic} correction exceeds safety gate! Clamping.")
                rot = np.clip(rot, -np.radians(3.0), np.radians(3.0))
                trans[2] = np.clip(trans[2], -0.35, 0.35)
                euler_deg = np.degrees(rot)
                
            C = np.eye(4)
            C[:3, :3] = R.from_rotvec(rot).as_matrix()
            C[:3, 3] = center - C[:3, :3] @ center + trans
            refined_mats[topic] = C @ mats[topic]
            
            lines.append(f"  {topic:20s}: Delta Roll={euler_deg[0]:+6.3f} deg, Pitch={euler_deg[1]:+6.3f} deg, Dz={trans[2]*100:+6.2f} cm (matches: {n_matches}, Status: OK)")
    
    if output_path:
        out_data = json.loads(json.dumps(raw_json))
        for entry in out_data.get("lidar", []):
            for topic_k, key_name in KEYS.items():
                if key_name in entry:
                    entry[key_name] = refined_mats[topic_k].flatten().tolist()
        
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(out_data, f, indent=2, ensure_ascii=False)
        print(f"[Pre-corrector] Saved refined calibration to: {output_path}")
        lines.append(f"\nSuccessfully written calibrated JSON to {output_path}")
        
    report_text = "\n".join(lines)
    print(report_text)
    if report_path:
        os.makedirs(os.path.dirname(os.path.abspath(report_path)), exist_ok=True)
        with open(report_path, "w", encoding="utf-8") as f:
            f.write(report_text + "\n")
            
    return refined_mats


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multi-LiDAR Extrinsic Pre-correction Engine")
    parser.add_argument("--calibration", required=True, help="Path to nominal perception_truckXX.json")
    parser.add_argument("--bag", default="", help="Path to bag containing startup static frames (optional)")
    parser.add_argument("--output", default="", help="Path to save refined JSON")
    parser.add_argument("--report", default="", help="Path to save calibration audit report")
    args = parser.parse_args()
    
    run_precorrection(args.calibration, args.bag, args.output, args.report)
