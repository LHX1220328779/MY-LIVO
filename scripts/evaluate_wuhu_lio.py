#!/usr/bin/env python3
"""Report compact LIO-versus-standard-INS-odometry metrics and timing."""

import os
import sys

if sys.executable != "/usr/bin/python3":
    environment = os.environ.copy()
    for variable in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME", "PYTHONPATH"):
        environment.pop(variable, None)
    environment["PATH"] = (
        "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:"
        "/usr/sbin:/usr/bin:/sbin:/bin")
    environment["AMENT_PREFIX_PATH"] = "/opt/ros/humble"
    environment["PYTHONPATH"] = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/opt/ros/humble/local/lib/python3.10/dist-packages")
    os.execve("/usr/bin/python3", ["/usr/bin/python3", *sys.argv], environment)

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

RESULTS_ROOT = (Path(__file__).resolve().parents[1] / "results").resolve()
os.environ.setdefault("MPLCONFIGDIR", str(RESULTS_ROOT / ".cache/matplotlib"))


def result_path(path):
    resolved = Path(path).resolve()
    if not resolved.is_relative_to(RESULTS_ROOT):
        raise ValueError(f"Analysis output must be inside {RESULTS_ROOT}: {path}")
    return resolved


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def timing_summary(name, values):
    intervals = np.diff(np.asarray(values))
    if not len(intervals):
        return f"{name}: insufficient samples"
    return (
        f"{name}: count={len(values)}, dt median={np.median(intervals):.6f}s, "
        f"p99={np.percentile(intervals, 99):.6f}s, max={np.max(intervals):.6f}s, "
        f"non-increasing={np.count_nonzero(intervals <= 0)}")


def metric_summary(name, errors):
    if not len(errors):
        return f"{name}: no valid samples"
    return (
        f"{name}: n={len(errors)}, RMSE={np.sqrt(np.mean(errors ** 2)):.4f}m, "
        f"median={np.median(errors):.4f}m, P95={np.percentile(errors, 95):.4f}m, "
        f"max={np.max(errors):.4f}m, >10cm={np.mean(errors > 0.1):.2%}, "
        f">15cm={np.mean(errors > 0.15):.2%}, "
        f">20cm={np.mean(errors > 0.20):.2%}, "
        f"final={errors[-1]:.4f}m")


def route_budget(errors, reference_positions, segment_valid):
    """Distance-weighted budget on the reference polyline, never fitted poses.

    Error vectors vary linearly within each sampled segment. Integrate the
    part inside each error sphere exactly; averaging endpoint pass/fail flags
    is wrong at threshold crossings. Zero-distance stops carry no road weight.
    This is sampled reference consistency, not unobserved continuous truth.
    """
    lengths = np.linalg.norm(np.diff(reference_positions, axis=0), axis=1)
    eligible = lengths * np.asarray(segment_valid, dtype=bool)
    total = float(eligible.sum())
    start, delta = errors[:-1], np.diff(errors, axis=0)
    a = np.einsum('ij,ij->i', delta, delta)
    b = 2 * np.einsum('ij,ij->i', start, delta)
    fractions = []
    for radius in (0.10, 0.20):
        c = np.einsum('ij,ij->i', start, start) - radius * radius
        fraction = (c <= 0).astype(float)
        changing = a > 1e-24
        discriminant = b * b - 4 * a * c
        crosses = changing & (discriminant >= 0)
        fraction[changing] = 0
        root = np.sqrt(np.maximum(0, discriminant[crosses]))
        lo = (-b[crosses] - root) / (2 * a[crosses])
        hi = (-b[crosses] + root) / (2 * a[crosses])
        fraction[crosses] = np.maximum(0, np.minimum(1, hi) - np.maximum(0, lo))
        fractions.append(float(eligible @ fraction) / total if total > 0 else float('nan'))
    return total, fractions[0], fractions[1]


def quality_at(query, quality, args):
    """Same mode-4, freshness and stable-recovery contract at every query."""
    if not len(quality): return np.zeros(len(query), dtype=bool)
    fixed_since = np.full(len(quality), np.inf)
    for i, (stamp, mode, satellites) in enumerate(quality):
        if mode == 4 and satellites >= args.minimum_satellites:
            fixed_since[i] = (fixed_since[i - 1] if i and
                stamp - quality[i - 1, 0] <= args.status_max_age and
                np.isfinite(fixed_since[i - 1]) else stamp)
    qi = np.searchsorted(quality[:, 0], query + 1e-6, side="right") - 1
    selected = quality[np.maximum(qi, 0)]
    return ((qi >= 0) & (query - selected[:, 0] <= args.status_max_age) &
            (selected[:, 1] == 4) & (selected[:, 2] >= args.minimum_satellites) &
            (query - fixed_since[np.maximum(qi, 0)] >= args.recovery_stable_sec))


def load_reference(path, offset):
    bags = [path] if (path / "metadata.yaml").is_file() else [
        entry.parent for entry in sorted(path.glob("Perception_*/metadata.yaml"))]
    if not bags:
        raise ValueError(f"No perception bags at {path}")
    times, positions, quaternions, quality = [], [], [], []
    imu_header, imu_record, lidar_header, lidar_record = [], [], [], []
    topics = ["imu_data", "imu_data/odometry", "imu_data/ins_status", "front_lidar"]
    for bag in bags:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("", ""))
        reader.set_filter(rosbag2_py.StorageFilter(
            topics=topics + ["/" + topic for topic in topics]))
        while reader.has_next():
            topic, data, record_time = reader.read_next()
            topic = topic.lstrip("/")
            if topic == "imu_data":
                message = deserialize_message(data, Imu)
                imu_header.append(stamp_seconds(message.header.stamp))
                imu_record.append(record_time * 1e-9)
            elif topic == "imu_data/odometry":
                message = deserialize_message(data, Odometry)
                times.append(stamp_seconds(message.header.stamp) - offset)
                p, q = message.pose.pose.position, message.pose.pose.orientation
                positions.append([p.x, p.y, p.z])
                quaternions.append([q.x, q.y, q.z, q.w])
            elif topic == "imu_data/ins_status":
                message = deserialize_message(data, DiagnosticArray)
                values = {item.key: item.value
                          for status in message.status for item in status.values}
                try:
                    quality.append([stamp_seconds(message.header.stamp) - offset,
                                    int(values.get("ins_pos_mode", -1)),
                                    int(values.get("gps_num_sats", -1))])
                except ValueError:
                    continue
            elif topic == "front_lidar":
                message = deserialize_message(data, PointCloud2)
                lidar_header.append(stamp_seconds(message.header.stamp))
                lidar_record.append(record_time * 1e-9)
        del reader
    if len(times) < 2:
        raise ValueError("At least two INS rear-axle observations are required")
    for name, data in [("IMU header", imu_header), ("IMU record", imu_record),
                       ("INS header", times), ("LiDAR header", lidar_header),
                       ("LiDAR record", lidar_record)]:
        print(timing_summary(name, data))
    unique, indices = np.unique(times, return_index=True)
    return (unique, np.asarray(positions)[indices], np.asarray(quaternions)[indices],
            np.asarray(sorted(quality)).reshape(-1, 3))


def compare_trajectory(path, reference, args):
    trajectory = np.loadtxt(path, ndmin=2)
    if trajectory.shape[1] != 8 or not np.all(np.isfinite(trajectory)):
        raise ValueError(f"{path}: expected finite TUM timestamp xyz xyzw")
    if np.any(np.diff(trajectory[:, 0]) <= 0):
        raise ValueError(f"{path}: trajectory timestamps must increase")
    times, positions, quaternions, quality = reference
    query = trajectory[:, 0]
    right = np.searchsorted(times, query)
    right = np.clip(right, 0, len(times) - 1)
    left = np.maximum(0, right - 1)
    exact = np.abs(times[right] - query) < 1e-6
    left[exact] = right[exact]
    gaps = times[right] - times[left]
    valid = ((query >= times[0] - 1e-6) & (query <= times[-1] + 1e-6) &
             (gaps <= args.max_gap))
    print(f"\n{path}: poses={len(query)}, time-valid={valid.sum()}, "
          f"outside/gap={np.count_nonzero(~valid)}")
    if not valid.any():
        return
    ratio = np.divide(query - times[left], gaps, out=np.zeros_like(query), where=gaps > 1e-6)
    interpolated = positions[left] * (1 - ratio[:, None]) + positions[right] * ratio[:, None]
    predicted = trajectory[:, 1:4].copy()
    if args.rear_axle_to_imu:
        # Move each estimated physical IMU to the measured rear axle using
        # that estimator's own attitude, not the noisy INS orientation.
        predicted += Rotation.from_quat(trajectory[:, 4:8]).apply(args.imu_to_rear_axle)
    errors = predicted - interpolated
    source_quality = quality_at(times, quality, args)
    quality_valid = source_quality[left] & source_quality[right]
    for label, mask in [("all time-valid", valid),
                        ("accepted RTK quality", valid & quality_valid),
                        ("bad/missing RTK quality", valid & ~quality_valid)]:
        for axis, error in [("XY", np.linalg.norm(errors[:, :2], axis=1)),
                            ("Z", np.abs(errors[:, 2])),
                            ("3D", np.linalg.norm(errors, axis=1))]:
            print(metric_summary(f"{label} {axis}", error[mask]))
    # Do not bridge an outage between two apparently valid trajectory poses.
    # Check every source observation/gap, not only the two sampled endpoints.
    source_good = source_quality.copy()
    source_good[1:] &= np.diff(times) <= args.max_gap
    bad_prefix = np.r_[0, np.cumsum(~source_good)]
    eligible = (valid[:-1] & valid[1:] &
                (np.diff(query) <= args.max_trajectory_gap) &
                ((bad_prefix[right[1:] + 1] - bad_prefix[left[:-1]]) == 0))
    assessed_m, within10, within20 = route_budget(errors, interpolated, eligible)
    total_m = np.linalg.norm(np.diff(interpolated, axis=0), axis=1).sum()
    accepted_error = np.linalg.norm(errors[valid & quality_valid], axis=1)
    maximum = float(accepted_error.max()) if len(accepted_error) else float('nan')
    print(f"Route-weighted 3D RTK consistency: assessed={assessed_m:.3f}m, "
          f"unassessed reference-polyline distance={total_m-assessed_m:.3f}m, "
          f"<=10cm={within10:.2%}, >20cm={1-within20:.2%}, "
          f"accepted-sample max={maximum:.4f}m")
    print(f"Assessed-route position budget (NOT map acceptance): "
          f"{'PASS' if assessed_m > 0 and within10 >= .80 and maximum <= .20 else 'FAIL/UNASSESSED'}; "
          "requires >=80% distance within 10cm and all accepted samples within 20cm. "
          "Unassessed intervals are never counted as passing.")
    if args.rear_axle_to_imu:
        # RViz compares physical IMU paths, whereas GPSFactorArm observes the
        # rear axle. Moving each estimate to the rear axle can hide a large
        # IMU discrepancy compensated by its own erroneous attitude.
        # Match publishReferenceTrajectory(): convert each original INS
        # sample first, then interpolate its IMU position at the same time.
        imu_samples = positions - Rotation.from_quat(quaternions).apply(args.imu_to_rear_axle)
        imu_reference = imu_samples[left] * (1 - ratio[:, None]) + imu_samples[right] * ratio[:, None]
        imu_error = trajectory[:, 1:4] - imu_reference
        accepted = valid & quality_valid
        print("Above budget is REAR AXLE, not the RViz IMU trajectory. "
              "INS-derived IMU reference includes attitude/lever uncertainty; NOT independent position truth.")
        for axis, error in [("XY", np.linalg.norm(imu_error[:, :2], axis=1)),
                            ("Z", np.abs(imu_error[:, 2])),
                            ("3D", np.linalg.norm(imu_error, axis=1))]:
            print(metric_summary(f"accepted INS-derived IMU consistency {axis}", error[accepted]))
        imu_distance, imu_within10, imu_within20 = route_budget(imu_error, imu_reference, eligible)
        print(f"IMU-reference route consistency: assessed={imu_distance:.3f}m, "
              f"<=10cm={imu_within10:.2%}, >20cm={1-imu_within20:.2%}; "
              "rear-axle agreement alone is not blue-path or map acceptance.")
    closest = np.where(query - times[left] < times[right] - query, left, right)
    rotation_errors = (Rotation.from_quat(quaternions[closest]).inv() *
                       Rotation.from_quat(trajectory[:, 4:8])).magnitude()
    print(f"INS attitude difference (NOT attitude truth): "
          f"P95={np.rad2deg(np.percentile(rotation_errors[valid], 95)):.4f}deg")
    print("No trajectory alignment applied. INS consistency is NOT independent absolute accuracy.")
    diagnostics = path.parent / "diagnostics.csv"
    if diagnostics.is_file():
        data = np.genfromtxt(diagnostics, delimiter=",", names=True)
        timing = np.atleast_1d(data["processing_ms"])
        print(f"Backend ms: median={np.median(timing):.2f}, "
              f"P95={np.percentile(timing, 95):.2f}, max={np.max(timing):.2f}")
        print(f"Final online counters: keys={int(np.atleast_1d(data['keyframes'])[-1])}, "
              f"position factors={int(np.atleast_1d(data['position_factors'])[-1])}")



def route_error_components(positions, arc, errors):
    """Along/left-of-route errors; a 2 m chord defines axes, not a pose filter.

    Dotting two XY error vectors does NOT say which side of a road they occupy:
    an along-track lag can reverse that sign while both remain on the same side.
    No registration, trajectory projection or replacement is performed here.
    """
    before = np.column_stack([np.interp(arc - 1, arc, positions[:, i]) for i in range(2)])
    after = np.column_stack([np.interp(arc + 1, arc, positions[:, i]) for i in range(2)])
    tangent = after - before
    length = np.linalg.norm(tangent, axis=1)
    tangent = np.divide(tangent, length[:, None], out=np.full_like(tangent, np.nan),
                        where=length[:, None] > 1e-6)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    return np.column_stack((np.einsum("ij,ij->i", errors[:, :2], tangent),
                            np.einsum("ij,ij->i", errors[:, :2], normal)))


def inspect_startup(run, reference, lever, output):
    """Audit the SAME first 30 m in one frame, including the lever-arm change."""
    native = np.loadtxt(run / "trajectory_lio.txt", ndmin=2)
    fused = np.loadtxt(run / "trajectory_optimized.txt", ndmin=2)
    rotation, position = interpolate_poses(fused, native[:, 0])
    raw_rotation = Rotation.from_quat(native[:, 4:8])
    gps = np.column_stack([np.interp(native[:, 0], reference[0], reference[1][:, i]) for i in range(3)])
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(gps, axis=0), axis=1))]
    valid = (arc <= 30) & (native[:, 0] >= fused[0, 0]) & (native[:, 0] <= fused[-1, 0])
    valid &= (native[:, 0] >= reference[0][0]) & (native[:, 0] <= reference[0][-1])
    if not valid.any(): return
    lever = np.asarray(lever)
    raw_rear = native[:, 1:4] + raw_rotation.apply(lever)
    fused_rear = position + rotation.apply(lever)
    raw_error, fused_error = raw_rear - gps, fused_rear - gps
    correction = rotation * raw_rotation.inv()
    tilt = np.degrees(np.arccos(np.clip(correction.apply(np.tile([0, 0, 1.], (len(native), 1)))[:, 2], -1, 1)))
    raw_route = route_error_components(gps, arc, raw_error)
    fused_route = route_error_components(gps, arc, fused_error)
    imu_samples = reference[1] - Rotation.from_quat(reference[2]).apply(lever)
    imu_reference = np.column_stack([np.interp(native[:, 0], reference[0], imu_samples[:, i]) for i in range(3)])
    raw_imu_error, fused_imu_error = native[:, 1:4] - imu_reference, position - imu_reference
    raw_imu_route = route_error_components(imu_reference, arc, raw_imu_error)
    fused_imu_route = route_error_components(imu_reference, arc, fused_imu_error)
    side_valid = valid & np.isfinite(raw_route[:, 1]) & np.isfinite(fused_route[:, 1])
    opposite = raw_route[:, 1] * fused_route[:, 1] < 0
    # Also report the literal synchronized RTK--native XY segment. Alpha=0
    # is RTK, alpha=1 native. A value in [0,1] alone does not imply collinearity.
    squared = np.einsum("ij,ij->i", raw_error[:, :2], raw_error[:, :2])
    alpha = np.divide(np.einsum("ij,ij->i", raw_error[:, :2], fused_error[:, :2]), squared,
                      out=np.full(len(native), np.nan), where=squared > 1e-12)
    nearest_segment = np.nan_to_num(np.clip(alpha, 0, 1))[:, None] * raw_error[:, :2]
    segment_distance = np.linalg.norm(fused_error[:, :2] - nearest_segment, axis=1)
    output = result_path(output or run / "startup_analysis")
    output.mkdir(parents=True, exist_ok=True)
    data = np.column_stack((native[:, 0], arc, raw_error, fused_error,
                            position - native[:, 1:4], rotation.apply(lever) - raw_rotation.apply(lever), tilt,
                            raw_route, fused_route, alpha, segment_distance,
                            raw_imu_error, fused_imu_error, raw_imu_route, fused_imu_route))
    np.savetxt(output / "startup.csv", data[valid], delimiter=",", comments="", fmt="%.17g",
               header="stamp,arc_m,lio_error_x,lio_error_y,lio_error_z,fused_error_x,fused_error_y,fused_error_z,imu_shift_x,imu_shift_y,imu_shift_z,lever_shift_x,lever_shift_y,lever_shift_z,correction_tilt_deg,lio_along_m,lio_cross_m,fused_along_m,fused_cross_m,rtk_to_lio_segment_alpha,segment_distance_xy_m,lio_imu_error_x,lio_imu_error_y,lio_imu_error_z,fused_imu_error_x,fused_imu_error_y,fused_imu_error_z,lio_imu_along_m,lio_imu_cross_m,fused_imu_along_m,fused_imu_cross_m")
    lines = [f"run={run.resolve()}; first {arc[valid][-1]:.3f} reference-polyline metres; NO ALIGNMENT",
             "LIO/RTK consistency does not determine which has better absolute accuracy or map finesse.",
             "The following errors and segment checks use the REAR AXLE; RViz paths use the IMU."]
    for label, error in [("native", raw_error), ("fused", fused_error)]:
        lines.append(f"{label} 3D error median/P95/max={np.percentile(np.linalg.norm(error[valid], axis=1), [50,95,100]).tolist()} m")
    for distance in (10, 20, 30):
        selected = side_valid & (arc <= distance)
        if not selected.any(): continue
        lines.append(f"first {distance} m opposite route sides={np.count_nonzero(selected & opposite)}/{selected.sum()} samples; "
                     f"native/fused signed cross-track median={np.median(raw_route[selected,1]):.6f}/{np.median(fused_route[selected,1]):.6f} m")
    lines.append(f"distance to synchronized RTK--native XY segment P50/P95/max="
                 f"{np.percentile(segment_distance[valid], [50,95,100]).tolist()} m; "
                 "startup overshoot remains a regression to resolve, never corrected by clipping")
    lines.append(f"correction tilt min/median/max={np.percentile(tilt[valid], [0,50,100]).tolist()} deg")
    first = np.flatnonzero(valid)[0]
    lines.append(f"first fused rear error={fused_error[first].tolist()} m; IMU shift={data[first,8:11].tolist()} m; rotated lever shift={data[first,11:14].tolist()} m")
    lines.append("RViz IMU-reference check: INS attitude/lever uncertainty included, NOT independent truth.")
    for label, error in [("native IMU", raw_imu_error), ("fused IMU", fused_imu_error)]:
        lines.append(f"{label} 3D difference median/P95/max={np.percentile(np.linalg.norm(error[valid], axis=1), [50,95,100]).tolist()} m")
    for distance in (10, 20, 30):
        selected = valid & (arc <= distance) & np.isfinite(raw_imu_route[:, 1]) & np.isfinite(fused_imu_route[:, 1])
        if not selected.any(): continue
        opposite_imu = raw_imu_route[:, 1] * fused_imu_route[:, 1] < 0
        lines.append(f"first {distance} m IMU opposite route sides={np.count_nonzero(selected & opposite_imu)}/{selected.sum()} samples; "
                     f"native/fused IMU signed cross-track median={np.median(raw_imu_route[selected,1]):.6f}/{np.median(fused_imu_route[selected,1]):.6f} m")
    (output / "startup_summary.txt").write_text("\n".join(lines) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, panels = plt.subplots(2, 3, figsize=(16, 9))
    panels = panels.ravel()
    for label, trajectory in [("INS-derived IMU", imu_reference), ("native LIO IMU", native[:, 1:4]), ("fused IMU", position)]:
        q = trajectory[valid] - imu_reference[first]
        panels[0].plot(q[:, 0], q[:, 1], label=label)
    panels[0].set_aspect("equal"); panels[0].set_xlabel("East offset (m)"); panels[0].set_ylabel("North offset (m)")
    for label, error in [("native", raw_error), ("fused", fused_error)]:
        panels[1].plot(arc[valid], np.linalg.norm(error[valid], axis=1), label=label)
    panels[1].axhline(.1, color="gray", linestyle=":"); panels[1].axhline(.2, color="gray", linestyle="--")
    panels[1].set_xlabel("First route distance (m)"); panels[1].set_ylabel("Rear-axle 3D RTK difference (m)")
    panels[2].plot(arc[valid], tilt[valid], label="native-to-fused tilt")
    panels[2].set_xlabel("First route distance (m)"); panels[2].set_ylabel("Correction tilt (deg)")
    if (run / "history.csv").exists():
        history = np.atleast_1d(np.genfromtxt(run / "history.csv", names=True, delimiter=","))
        rows = history[np.abs(history["key_stamp"] - native[first, 0]) < 1e-5]
        panels[3].plot(rows["update_stamp"] - native[first, 0], rows["rtk_residual_m"], label="same first key")
    panels[3].set_xlabel("Later optimization time (s)"); panels[3].set_ylabel("Historical first-key RTK difference (m)")
    for label, components in [("native IMU", raw_imu_route), ("fused IMU", fused_imu_route)]:
        panels[4].plot(arc[valid], components[valid, 1], label=label)
        panels[5].plot(arc[valid], components[valid, 0], label=label)
    for panel, component in [(panels[4], "left-of-route"), (panels[5], "along-route")]:
        panel.axhline(0, color="gray", linestyle=":")
        panel.set_xlabel("First route distance (m)"); panel.set_ylabel(f"IMU signed {component} difference (m)")
    for panel in panels: panel.grid(alpha=.2); panel.legend()
    fig.suptitle("Startup: physical IMU paths / separate rear-axle residuals; no alignment or clipping")
    fig.tight_layout(); fig.savefig(output / "startup.png", dpi=160); plt.close(fig)
    print("\n".join(lines))


def correction_samples(raw, optimized, query):
    """Same SE(3) correction interpolation as the mapper, vectorized."""
    raw_rotation = Rotation.from_quat(raw[:, 4:8])
    rotation = Rotation.from_quat(optimized[:, 4:8]) * raw_rotation.inv()
    translation = optimized[:, 1:4] - rotation.apply(raw[:, 1:4])
    right = np.clip(np.searchsorted(raw[:, 0], query), 0, len(raw) - 1)
    left = np.maximum(0, right - 1)
    span = raw[right, 0] - raw[left, 0]
    ratio = np.clip(np.divide(query - raw[left, 0], span,
                             out=np.zeros_like(query), where=span > 0), 0, 1)
    a, b = rotation[left], rotation[right]
    omega = (a.inv() * b).as_rotvec()
    delta = a.inv().apply(translation[right] - translation[left])
    theta = np.linalg.norm(omega, axis=1)
    coefficient = np.full(len(theta), 1 / 12)
    valid = theta > 1e-5
    coefficient[valid] = (1 - 0.5 * theta[valid] / np.tan(theta[valid] / 2)) / theta[valid] ** 2
    velocity = delta - 0.5 * np.cross(omega, delta) + coefficient[:, None] * np.cross(
        omega, np.cross(omega, delta))
    w, v = omega * ratio[:, None], velocity * ratio[:, None]
    angle = theta * ratio
    c1, c2 = np.full(len(theta), 0.5), np.full(len(theta), 1 / 6)
    valid = angle > 1e-5
    c1[valid] = (1 - np.cos(angle[valid])) / angle[valid] ** 2
    c2[valid] = (angle[valid] - np.sin(angle[valid])) / angle[valid] ** 3
    shift = v + c1[:, None] * np.cross(w, v) + c2[:, None] * np.cross(w, np.cross(w, v))
    return a * Rotation.from_rotvec(w), translation[left] + a.apply(shift)


def interpolate_poses(trajectory, query):
    right = np.clip(np.searchsorted(trajectory[:, 0], query), 0, len(trajectory) - 1)
    left = np.maximum(0, right - 1)
    span = trajectory[right, 0] - trajectory[left, 0]
    u = np.clip(np.divide(query - trajectory[left, 0], span,
                         out=np.zeros_like(query), where=span > 0), 0, 1)
    a, b = Rotation.from_quat(trajectory[left, 4:8]), Rotation.from_quat(trajectory[right, 4:8])
    return a * Rotation.from_rotvec((a.inv() * b).as_rotvec() * u[:, None]), \
        (1 - u[:, None]) * trajectory[left, 1:4] + u[:, None] * trajectory[right, 1:4]


def read_pcd(path, with_header=False):
    """Read native/fused binary PCDs without filtering or changing points."""
    header = {}
    with path.open("rb") as stream:
        for _ in range(30):
            fields = stream.readline().decode("ascii").split()
            if fields and fields[0] != "#": header[fields[0]] = fields[1:]
            if fields[:1] == ["DATA"]:
                offset = stream.tell()
                break
        else: raise ValueError(f"Missing PCD header: {path}")
    count = len(header.get("FIELDS", []))
    if (header.get("FIELDS", [])[:4] != ["x", "y", "z", "intensity"] or
            header.get("TYPE") != ["F"] * count or header.get("SIZE") != ["4"] * count or
            header.get("COUNT") != ["1"] * count or header.get("DATA") != ["binary"]):
        raise ValueError(f"Expected binary float XYZI PCD: {path}")
    points = int(header["POINTS"][0])
    if path.stat().st_size != offset + points * count * 4:
        raise ValueError(f"PCD payload size disagrees with declared fields/point stride: {path}")
    cloud = np.memmap(path, mode="r", dtype="<f4", offset=offset,
                      shape=(points, count))[:, :4] if points else np.empty((0, 4), dtype="<f4")
    return (cloud, header) if with_header else cloud


def pcd_sections(directory, selection=None, reference=None, lever=(.904, -4.679, -1.699), output=None):
    """Inspect actual saved PCDs, without reprojecting or registering them.

    Sparse sampling selects representative ROIs only. All stored points in
    each section are then read, coloured by chunk/time, with no voxel filter.
    Passing a previous sections.csv repeats the exact world-space sections.
    """
    output = result_path(output if output is not None else directory)
    output.mkdir(parents=True, exist_ok=True)
    import csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    files = sorted(directory.glob("map_*.pcd"), key=lambda path: int(path.stem.split("_")[-1]))
    if not files:
        raise ValueError(f"No saved PCD chunks at {directory}")

    # A diagnostic comparator only: project identical RAW returns with the
    # recorded INS pose. Never use it to overwrite a fused pose or PCD.
    reference_sections = None
    if reference is not None:
        if not selection:
            raise ValueError("INS comparison requires the same existing section-selection CSV")
        translation = rotation = None
        with (directory / "frames.csv").open() as stream:
            for line in stream:
                if line.startswith("# lidar_to_imu_translation "):
                    translation = np.fromstring(line.split("translation ", 1)[1], sep=" ")
                if line.startswith("# lidar_to_imu_rotation_row_major "):
                    rotation = np.fromstring(line.split("row_major ", 1)[1], sep=" ").reshape(3, 3)
                if not line.startswith("#"):
                    index = np.atleast_1d(np.genfromtxt(stream, delimiter=",", names=line.strip().split(",")))
                    break
        with (directory / "scans.bin").open("rb") as stream:
            if stream.read(8) != b"LIVORTK2":
                raise ValueError("INS reference sections require original raw returns (LIVORTK2)")
        raw_points = np.memmap(directory / "scans.bin", dtype="<f4", mode="r", offset=8).reshape(-1, 5)
        times, positions, quaternions, _ = reference
        imu_positions = positions - Rotation.from_quat(quaternions).apply(lever)
        ins_trajectory = np.column_stack((times, imu_positions, quaternions))
        reference_sections = []

    sections = []
    if selection:
        with selection.open() as stream:
            for row in csv.DictReader(stream):
                sections.append((row["kind"], float(row["selection_offset_m"]),
                    np.array([float(row[k]) for k in ("x", "y", "z")]),
                    np.array([float(row[k]) for k in ("nx", "ny", "nz")]),
                    np.array([float(row[k]) for k in ("ux", "uy", "uz")])) )
    else:
        cells = {}
        trajectory = np.loadtxt(directory / "trajectory_optimized.txt", ndmin=2)
        route = cKDTree(trajectory[:, 1:3])
        for chunk, path in enumerate(files):
            points = np.asarray(read_pcd(path)[::8, :3], dtype=float)
            grid = np.floor(points / 2).astype(np.int64)
            order = np.lexsort((grid[:, 2], grid[:, 1], grid[:, 0]))
            starts = np.r_[0, np.flatnonzero(np.any(grid[order][1:] != grid[order][:-1], axis=1)) + 1, len(order)]
            for a, b in zip(starts[:-1], starts[1:]):
                if b - a < 25:
                    continue
                group = points[order[a:b]]
                eig, vec = np.linalg.eigh(np.cov(group.T, bias=True))
                if eig[0] > 0.04 ** 2 or eig[1] < 0.03 or eig[0] > .08 * eig[1]:
                    continue
                center, normal = group.mean(0), vec[:, 0]
                nearest = route.query(center[:2])[1]
                kind = "ground candidate" if abs(normal[2]) > .95 and center[2] < trajectory[nearest, 3] - 1.5 else \
                    "vertical surface" if abs(normal[2]) < .15 else None
                if kind:
                    key = (kind, *grid[order[a]])
                    cells.setdefault(key, []).append((chunk, center, normal, vec[:, 2]))
        candidates = {"ground candidate": [], "vertical surface": []}
        for key, entries in cells.items():
            if len(entries) < 3 or entries[-1][0] - entries[0][0] < 3:
                continue
            _, center, normal, tangent = entries[0]
            if any(abs(normal @ item[2]) < .99 for item in entries):
                continue
            offsets = [(item[1] - center) @ normal for item in entries]
            candidates[key[0]].append((np.ptp(offsets), center, normal, tangent))
        for kind, entries in candidates.items():
            entries.sort(key=lambda item: item[0])
            if not entries:
                print(f"PCD sections: insufficient repeatedly observed {kind}")
                continue
            for quantile in (.5, .9):
                item = entries[round(quantile * (len(entries) - 1))]
                if not any(np.linalg.norm(item[1] - old[2]) < 2 for old in sections):
                    sections.append((kind, *item))
    if not sections:
        raise ValueError("No cross-chunk planar sections found; specify surveyed/visually selected ROIs")
    prefix = "ins_sections" if reference is not None else "pcd_sections"
    with (output / f"{prefix}.csv").open("w") as stream:
        rows = csv.writer(stream)
        rows.writerow(("kind", "selection_offset_m", "x", "y", "z", "nx", "ny", "nz", "ux", "uy", "uz"))
        for kind, offset, center, normal, tangent in sections:
            rows.writerow((kind, offset, *center, *normal, *tangent))
    if reference_sections is not None:
        reference_sections = [[] for _ in sections]
        chunk, accumulated = 0, 0
        for record in index:
            start = (int(record["offset_bytes"]) - 8) // 20
            scan = raw_points[start:start + int(record["count"])]
            query = np.minimum(record["stamp"], record["begin"] + np.floor(scan[:, 4].astype(float) + .5) * .001)
            r, t = interpolate_poses(ins_trajectory, query)
            world = r.apply(scan[:, :3].astype(float) @ rotation.T + translation) + t
            for i, (_, _, center, normal, tangent) in enumerate(sections):
                selected = world[np.all(np.abs(world - center) < 2, axis=1)] - center
                x, y, h = selected @ tangent, selected @ np.cross(normal, tangent), selected @ normal
                keep = (abs(x) < 1) & (abs(y) < .5) & (abs(h) < .6)
                if keep.any(): reference_sections[i].append((chunk, np.column_stack((x[keep], h[keep]))))
            accumulated += int(record["count"])
            if accumulated >= 1000000: chunk += 1; accumulated = 0
    fig, axes = plt.subplots(len(sections), 1, figsize=(12, 3.2 * len(sections)), squeeze=False)
    with (output / f"{prefix}_measurements.csv").open("w") as stream:
        rows = csv.writer(stream)
        rows.writerow(("section", "chunk", "points", "normal_q05_m", "normal_median_m", "normal_q95_m"))
        for number, (kind, offset, center, normal, tangent) in enumerate(sections):
            ax = axes[number, 0]
            across = np.cross(normal, tangent)
            pooled, chunk_medians = [], []
            for chunk, path in enumerate(files):
                if reference_sections is not None:
                    parts = [points for group, points in reference_sections[number] if group == chunk]
                    if not parts: continue
                    x, height = np.concatenate(parts).T
                else:
                    points = read_pcd(path)[:, :3]
                    # Cheap prefilter; final section is an oriented 2 m x 1 m
                    # slab with +/-0.6 m normal extent.
                    mask = np.all(np.abs(points - center) < 2, axis=1)
                    delta = np.asarray(points[mask], dtype=float) - center
                    x, y, height = delta @ tangent, delta @ across, delta @ normal
                    keep = (abs(x) < 1) & (abs(y) < .5) & (abs(height) < .6)
                    x, height = x[keep], height[keep]
                if not len(height):
                    continue
                q05, median, q95 = np.percentile(height, [5, 50, 95])
                rows.writerow((number, chunk, len(height), q05, median, q95))
                pooled.append(height)
                if len(height) >= 30:
                    chunk_medians.append(median)
                ax.scatter(x, 100 * height, s=.6, alpha=.55,
                           color=plt.cm.turbo(chunk / max(1, len(files) - 1)), rasterized=True)
            if not pooled:
                ax.set_title(f"{kind}: no saved points in this fixed ROI")
                continue
            width = np.ptp(np.percentile(np.concatenate(pooled), [5, 95]))
            spacing = np.ptp(chunk_medians) if len(chunk_medians) > 1 else float("nan")
            print(f"{prefix} section {number}: {kind}, center={center}, points={sum(map(len, pooled))}, "
                  f"pooled Q95-Q05={100*width:.2f}cm, chunk median span={100*spacing:.2f}cm")
            ax.set_title(f"{kind} | center {np.round(center, 2)} | pooled 90% thickness {100*width:.1f} cm")
            ax.set_xlabel("Position along SAME reference plane (m); colour: early blue -> late red")
            ax.set_ylabel("Normal displacement (cm)")
            ax.grid(alpha=.2)
    title = "INS-projected raw-return comparator (NOT truth)" if reference is not None else "ACTUAL saved PCD sections"
    fig.suptitle(title + ": no registration, no per-chunk plane removal, no smoothing", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(output / f"{prefix}.png", dpi=160)
    plt.close(fig)


def inspect_chunk(run, chunk, reference, lever, output):
    """Actual chunk geometry and first-online versus final pose consistency."""
    import csv
    output = result_path(output or run / f"chunk_{chunk}_analysis")
    output.mkdir(parents=True, exist_ok=True)
    frames = np.genfromtxt(run / "frames.csv", names=True, delimiter=",", skip_header=3)
    offsets = np.r_[0, np.cumsum(frames["count"].astype(np.int64))]
    start = sum(len(read_pcd(run / f"map_{i}.pcd")) for i in range(chunk))
    cloud = read_pcd(run / f"map_{chunk}.pcd")
    begin, end = np.searchsorted(offsets, [start, start + len(cloud)])
    if offsets[begin] != start or offsets[end] != start + len(cloud):
        raise ValueError("Actual PCD boundary does not follow whole-scan export")
    stamps = frames["stamp"][begin:end]
    point_frame = np.searchsorted(offsets, start + np.arange(len(cloud)), side="right") - 1
    times, positions, quaternions, _ = reference
    gps = np.column_stack([np.interp(stamps, times, positions[:, i]) for i in range(3)])
    route = float(np.linalg.norm(np.diff(gps, axis=0), axis=1).sum())
    summary = [f"run={run.resolve()}; ACTUAL map_{chunk}.pcd, {len(cloud)} points",
               f"source frames=[{begin},{end}), stamps=[{stamps[0]:.9f},{stamps[-1]:.9f}], reference route={route:.3f} m",
               "No registration, per-frame plane removal, or point displacement. No absolute thickness acceptance threshold."]
    diagnostics = np.genfromtxt(run / "diagnostics.csv", names=True, delimiter=",")
    diag = diagnostics[(diagnostics["stamp"] >= stamps[0]) & (diagnostics["stamp"] <= stamps[-1])]
    if len(diag):
        for field in ("rtk_residual_m", "latest_key_rtk_residual_m", "unoptimized_tail_rtk_residual_m", "publish_lag_sec"):
            if field in diag.dtype.names:
                values = diag[field][np.isfinite(diag[field])]
                if len(values): summary.append(f"online {field} median/P95/max={np.percentile(values, [50,95,100]).tolist()}")
        if "published_stamp" not in diag.dtype.names:
            summary.append("LEGACY online rtk_residual_m refers to the last KEY, not necessarily the displayed current scan.")
    keys = np.genfromtxt(run / "keyframes.csv", names=True, delimiter=",")
    keys = keys[(keys["stamp"] >= stamps[0]) & (keys["stamp"] <= stamps[-1])]
    history = np.genfromtxt(run / "history.csv", names=True, delimiter=",")
    with (output / "key_error_evolution.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(("key_stamp", "first_online_rtk_m", "final_rtk_m", "max_logged_rtk_m"))
        for key in keys:
            rows = history[abs(history["key_stamp"] - key["stamp"]) < 1e-5]
            writer.writerow((key["stamp"], rows["rtk_residual_m"][0] if len(rows) else np.nan,
                             key["rtk_residual_m"], np.max(rows["rtk_residual_m"]) if len(rows) else np.nan))
    if len(keys): summary.append(f"FINAL key RTK residuals={keys['rtk_residual_m'].tolist()} m")
    quality_path = run.parents[1] / "lio_geometry.csv"
    if quality_path.is_file():
        quality = np.genfromtxt(quality_path, names=True, delimiter=",")
        quality = quality[(quality["stamp"] >= stamps[0]) & (quality["stamp"] <= stamps[-1])]
        if len(quality):
            info = np.zeros((len(quality), 6, 6))
            for i in range(6):
                for j in range(i, 6): info[:, i, j] = info[:, j, i] = quality[f"body_h{i}{j}"]
            info[:, :3, :] /= 30; info[:, :, :3] /= 30
            info /= np.maximum(1, quality["effective_points"])[:, None, None]
            eigenvalues, eigenvectors = np.linalg.eigh(info)
            summary.append(f"native effective points median={np.median(quality['effective_points'])}; point-plane RMS median={np.median(quality['point_plane_rms_m']):.6f} m")
            summary.append(f"metric-normalized information eigenvalues median={np.median(eigenvalues, axis=0).tolist()}; weakest-direction squared components [roll,pitch,yaw,x,y,z]={np.median(eigenvectors[:,:,0]**2, axis=0).tolist()}")

    # Ground CANDIDATES near this chunk's route, not the whole sloped mine.
    # A common pooled plane supplies only section coordinates; each original
    # frame keeps its displacement from that same plane. Do not flatten scans.
    center = gps.mean(axis=0)
    xyz = np.asarray(cloud[:, :3], dtype=float)
    keep = (np.linalg.norm(xyz[:, :2] - center[:2], axis=1) < 30) & (xyz[:, 2] < center[2] + .5)
    ids = np.flatnonzero(keep)
    cells = np.floor(xyz[ids, :2] / 2).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    order = np.argsort(inverse, kind="stable")
    breaks = np.r_[0, np.flatnonzero(np.diff(inverse[order])) + 1, len(order)]
    sections = []
    for first, last in zip(breaks[:-1], breaks[1:]):
        if last - first < 100: continue
        selected = ids[order[first:last]]
        if len(np.unique(point_frame[selected])) < 5: continue
        patch = xyz[selected]; mean = patch.mean(axis=0)
        eig, vec = np.linalg.eigh(np.cov(patch.T, bias=True))
        if abs(vec[2, 0]) < .9 or eig[1] < .01: continue
        normal, tangent = vec[:, 0], vec[:, 2]
        normal *= 1 if normal[2] >= 0 else -1
        height = (patch - mean) @ normal
        width = np.ptp(np.percentile(height, [5,95]))
        sections.append((width, selected, mean, normal, tangent, height))
    with (output / "ground_cells.csv").open("w") as stream:
        writer = csv.writer(stream); writer.writerow(("x", "y", "z", "points", "frames", "pooled_q90_m"))
        for width, ids, mean, _, _, _ in sections:
            writer.writerow((*mean, len(ids), len(np.unique(point_frame[ids])), width))
    if sections and route <= 30:
        sections.sort(key=lambda section: section[0])
        # Explicit median / 90th-percentile candidates; keep the full table so
        # choosing a thin individual surface cannot masquerade as a good map.
        selected_sections = [sections[round(q * (len(sections) - 1))] for q in (.5, .9)]
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, panels = plt.subplots(2, 2, figsize=(13, 8))
        with (output / "ground_frame_residuals.csv").open("w") as stream:
            writer = csv.writer(stream); writer.writerow(("section", "frame_stamp", "points", "q05_m", "median_m", "q95_m"))
            for section, (width, ids, mean, normal, tangent, height) in enumerate(selected_sections):
                frame_ids = point_frame[ids]; x = (xyz[ids] - mean) @ tangent
                panels[section, 0].scatter(x, height * 100, c=frames["stamp"][frame_ids]-stamps[0], s=1, cmap="turbo")
                medians, widths, sample_times = [], [], []
                for frame in np.unique(frame_ids):
                    h = height[frame_ids == frame]
                    if len(h) < 10: continue
                    q = np.percentile(h, [5,50,95]); writer.writerow((section, frames["stamp"][frame], len(h), *q))
                    sample_times.append(frames["stamp"][frame] - stamps[0]); medians.append(q[1]); widths.append(q[2] - q[0])
                panels[section, 1].plot(sample_times, np.asarray(medians)*100, ".-", label="per-frame median, same plane")
                panels[section, 1].plot(sample_times, np.asarray(widths)*100, ".-", label="per-frame Q95-Q05")
                panels[section, 0].set_title(f"Ground candidate {np.round(mean,2)}; pooled width {100*width:.2f} cm")
                panels[section, 0].set_xlabel("Common-plane tangent (m), colour: time")
                panels[section, 1].set_xlabel("Scan time from chunk start (s)"); panels[section, 1].legend()
                for panel in panels[section]: panel.set_ylabel("Normal displacement / width (cm)"); panel.grid(alpha=.2)
                summary.append(f"ground candidate {mean.tolist()}: pooled width={width:.6f} m, median per-frame width={np.median(widths) if widths else np.nan:.6f} m, frame median span={np.ptp(medians) if medians else np.nan:.6f} m")
        fig.suptitle("ACTUAL chunk: many scans, one common plane per ROI, no scan alignment")
        fig.tight_layout(); fig.savefig(output / "ground_time_sections.png", dpi=160); plt.close(fig)
    else:
        summary.append("Ground sections UNASSESSED: no supported candidate or chunk exceeds 30 m reference route.")
    text = "\n".join(summary) + "\n"; (output / "chunk_summary.txt").write_text(text); print(text)


def read_manual_cloud(path):
    """XYZ from a PCD or a single CloudCompare v5.0 point-cloud root.

    Follow ccObject/ccHObject/ccGenericPointCloud serialization, not a scan
    for plausible float triples. Reject unhandled layouts instead of guessing.
    https://github.com/CloudCompare/CloudCompare/tree/master/libs/qCC_db/src
    """
    import struct
    if path.suffix.lower() == ".pcd": return np.asarray(read_pcd(path)[:, :3], dtype=float)
    with path.open("rb") as stream:
        def unpack(fmt):
            size = struct.calcsize(fmt)
            data = stream.read(size)
            if len(data) != size: raise ValueError(f"Truncated CloudCompare file: {path}")
            return struct.unpack(fmt, data)
        signature = stream.read(4)
        if signature != b"CCB2" or unpack("<I")[0] != 50:
            raise ValueError(f"Unsupported CloudCompare layout; export selected cloud as PCD: {path}")
        kind, _ = unpack("<QI")
        size = unpack(">I")[0]  # Qt QString is UTF-16 big-endian.
        name = stream.read(size).decode("utf-16-be")
        if kind != 0x101:
            raise ValueError(f"{path}: '{name}' is not a point cloud (class {kind:#x})")
        _, metadata_count = unpack("<II")
        if metadata_count: raise ValueError("Manual cloud contains unsupported Qt metadata; export PCD")
        flags = unpack("<6B")
        if flags[-1]: stream.read(3)  # overridden display colour, not geometry
        if unpack("<B")[0]: raise ValueError("Unapplied GL transform in manual cloud; export PCD")
        unpack("<B")  # display name
        shift_scale = np.asarray(unpack("<4d"))
        if unpack("<B")[0]:
            components, count = unpack("<BI")
            if components != 1: raise ValueError("Invalid CloudCompare visibility table")
            stream.seek(count, 1)
        unpack("<B")  # display point size
        components, count = unpack("<BI")
        if components != 3 or count > path.stat().st_size // 12:
            raise ValueError("Invalid CloudCompare XYZ array")
        data = stream.read(count * 12)
        if len(data) != count * 12: raise ValueError("Truncated CloudCompare XYZ array")
        xyz = np.frombuffer(data, dtype="<f4").reshape(-1, 3).astype(float)
        if not np.isfinite(shift_scale).all() or shift_scale[3] <= 0:
            raise ValueError("Invalid CloudCompare global shift/scale")
        xyz = xyz / shift_scale[3] - shift_scale[:3]
        if not np.isfinite(xyz).all(): raise ValueError("Nonfinite manual cloud coordinates")
        return xyz


def audit_deskew_surfaces(run, reference, lever, output, args):
    """Causal, fixed-support velocity-only deskew diagnostic; NEVER a map output.

    Reuse exactly the source voxel returns of saved geometric constraints.
    No new association, ROI, optimizer weight or pose correction. INS attitude
    only converts reference velocity for this diagnostic, not a production prior.
    """
    output = result_path(output or run.parents[2] / "deskew_surface_audit")
    output.mkdir(parents=True, exist_ok=True)
    with (run / "scans.bin").open("rb") as stream:
        if stream.read(8) != b"LIVORTK3": raise ValueError("Requires native scan-end deskew archive")
    frames = np.genfromtxt(run / "frames.csv", names=True, delimiter=",", skip_header=3)
    raw = np.loadtxt(run / "trajectory_lio.txt")
    refined = np.loadtxt(run / "trajectory_optimized.txt")
    constraints = np.genfromtxt(run / "geometry_constraints.csv", names=True, delimiter=",")
    if "is_line" not in constraints.dtype.names: raise ValueError("Requires explicit geometric feature kind")
    if "source_body_sqrt_00" in constraints.dtype.names:
        raise ValueError("Legacy velocity diagnostic cannot reconstruct shared-support surface moments; refusing an inconsistent counterfactual")
    pre = np.loadtxt(run.parents[1] / "mat_pre.txt")[-len(frames):]
    offset = np.median(frames["stamp"] - pre[:, 0])
    if np.max(abs(pre[:, 0] + offset - frames["stamp"])) > .001:
        raise ValueError("Cannot associate native IMU propagation log with actual scans")
    # mat_pre Euler is radians times 57.3, NOT exact degrees. These are the
    # actual propagated endpoint rotation/velocity used by UndistortPcl.
    propagated_r = Rotation.from_euler("xyz", pre[:, 1:4] / 57.3)
    t = frames["stamp"]
    lo, hi = np.maximum(t - .25, reference[0][0]), np.minimum(t + .25, reference[0][-1])
    if not np.all(quality_at(lo, reference[3], args) & quality_at(hi, reference[3], args)):
        raise ValueError("This diagnostic requires qualified reference velocity brackets; do not bridge outages")
    ins = np.column_stack((reference[0], reference[1] - Rotation.from_quat(reference[2]).apply(lever), reference[2]))
    ins_r, _ = interpolate_poses(ins, t)
    velocity = (interpolate_poses(ins, hi)[1] - interpolate_poses(ins, lo)[1]) / (hi - lo)[:, None]
    delta_v = ins_r.inv().apply(velocity) - propagated_r.inv().apply(pre[:, 7:10])
    archive = np.memmap(run / "scans.bin", dtype="<f4", mode="r", offset=8).reshape(-1, 5)
    raw_r, raw_p = interpolate_poses(raw, t)
    opt_r, opt_p = interpolate_poses(refined, t)
    updated = np.column_stack([constraints[name] for name in constraints.dtype.names])
    local_centers, local_normals = {}, {}
    audits = []
    header = (run / "frames.csv").read_text().splitlines()[:3]
    extrinsic = np.fromstring(header[1].split("translation ")[1], sep=" ")
    extrinsic_r = np.fromstring(header[2].split("row_major ")[1], sep=" ").reshape(3, 3)
    for stamp in np.unique(np.r_[constraints["a_stamp"], constraints["b_stamp"]]):
        i = int(np.argmin(abs(t - stamp)))
        if abs(t[i] - stamp) > 1e-5: raise ValueError("Geometry stamp is not an actual scan")
        begin = (int(frames[i]["offset_bytes"]) - 8) // 20
        source = np.asarray(archive[begin:begin + int(frames[i]["count"])], dtype=float)
        # Wuhu's archived virtual RFU LiDAR has identity rotation and this
        # physical IMU lever. Verify the actual contract before applying it.
        point = source[:, :3] @ extrinsic_r.T + extrinsic
        dt = frames[i]["begin"] + source[:, 4] * .001 - stamp
        if dt.min() < -.11 or dt.max() > 1e-5: raise ValueError("Unexpected point-to-end interval")
        centers, normals, changed_centers = [], [], []
        for width in (2., 1.):
            _, group = np.unique(np.floor(point / width).astype(np.int64), axis=0, return_inverse=True)
            count = np.bincount(group)
            mean = np.column_stack([np.bincount(group, weights=point[:, j]) / count for j in range(3)])
            mean_t = np.bincount(group, weights=dt) / count
            var_t = np.bincount(group, weights=dt * dt) / count - mean_t * mean_t
            cov_pt = np.column_stack([np.bincount(group, weights=point[:, j] * dt) / count for j in range(3)]) - mean * mean_t[:, None]
            covariance = np.empty((len(count), 3, 3))
            for u in range(3):
                for v in range(3):
                    covariance[:, u, v] = np.bincount(group, weights=point[:, u] * point[:, v]) / count - mean[:, u] * mean[:, v]
            eig, _ = np.linalg.eigh(covariance)
            linear = (eig[:, 1] < .01) & (eig[:, 2] >= .1) & (eig[:, 1] <= .01) & (eig[:, 1] <= .05 * eig[:, 2])
            planar = (eig[:, 1] >= .01) & (eig[:, 0] <= .05 ** 2) & (eig[:, 0] <= .12 * eig[:, 1])
            supported = (count >= 8) & (linear | planar)
            # Exact moments of p' = p + delta_v * dt, for the SAME returns.
            covariance += cov_pt[:, :, None] * delta_v[i][None, None, :] + delta_v[i][None, :, None] * cov_pt[:, None, :]
            covariance += var_t[:, None, None] * np.outer(delta_v[i], delta_v[i])[None, :, :]
            _, directions = np.linalg.eigh(covariance)
            normals.append(np.where(linear[:, None], directions[:, :, 2], directions[:, :, 0])[supported])
            centers.append(mean[supported]); changed_centers.append((mean + mean_t[:, None] * delta_v[i])[supported])
        centers, normals, changed_centers = map(np.concatenate, (centers, normals, changed_centers))
        tree = cKDTree(centers)
        maximum = 0.
        for side, name in enumerate(("a_stamp", "b_stamp")):
            ids = np.flatnonzero(abs(constraints[name] - stamp) < 1e-5)
            xyz = updated[ids, 2 + 3 * side:5 + 3 * side]
            old_local = opt_r[i].inv().apply(xyz - opt_p[i])
            distance, selected = tree.query(old_local)
            if len(distance): maximum = max(maximum, float(distance.max()))
            if maximum > 1e-5: raise ValueError(f"Cannot reproduce saved source patch at {stamp}: {maximum} m")
            changed = changed_centers[selected]
            updated[ids, 2 + 3 * side:5 + 3 * side] = opt_r[i].apply(changed) + opt_p[i]
            for row, value in zip(ids, changed): local_centers[row, side] = value
            if side == 0:
                direction = normals[selected]
                updated[ids, 8:11] = opt_r[i].apply(direction)
                for row, value in zip(ids, direction): local_normals[row] = value
        audits.append([stamp, *delta_v[i], np.linalg.norm(delta_v[i]) * np.ptp(dt), maximum])
    # Physical offsets and intrinsic roughness may both change. Keep every
    # correspondence and its ORIGINAL sigma, isolating the input-model effect.
    for j, row in enumerate(updated):
        a = int(np.argmin(abs(t - row[0]))); b = int(np.argmin(abs(t - row[1])))
        normal = local_normals[j]
        native_delta = raw_r[a].inv().apply(raw_r[b].apply(local_centers[j, 1]) + raw_p[b] - raw_p[a]) - local_centers[j, 0]
        world_delta = row[5:8] - row[2:5]
        row[12] = np.linalg.norm(world_delta - row[8:11] * np.dot(row[8:11], world_delta)) if row[14] else np.dot(row[8:11], world_delta)
        row[13] = np.linalg.norm(native_delta - normal * np.dot(normal, native_delta)) if row[14] else np.dot(normal, native_delta)
    np.savetxt(output / "geometry_constraints.csv", updated, delimiter=",", fmt="%.17g", header=",".join(constraints.dtype.names), comments="")
    np.savetxt(output / "scan_velocity.csv", audits, delimiter=",", fmt="%.17g", comments="",
               header="stamp,delta_v_body_x,delta_v_body_y,delta_v_body_z,scan_velocity_displacement_span_m,source_center_reconstruction_error_m")
    print(f"Fixed-support velocity diagnostic: {len(audits)} scans, {len(updated)} unchanged correspondences. NOT an optimized map or production calibration.")


def compare_pole_chunks(directory, ground_triangle, height, chunks, output, manual_crops=None,
                        cylinder=(-10.75, -274.0, 2.5, 43.0, 54.5), shaft_z=(45.5, 48.5)):
    """Extract the verified pole region from ACTUAL saved chunks.

    No search for another object, registration, per-chunk alignment or point
    correction. Inadequate vertical support is UNASSESSED, never a small error.
    """
    import csv
    triangle = np.asarray(ground_triangle, dtype=float).reshape(3, 3) if ground_triangle is not None else None
    if triangle is not None:
        if not np.isfinite(triangle).all() or not np.isfinite(height) or height <= 0:
            raise ValueError("Finite ground triangle and positive prism height required")
        edges = (triangle[1:, :2] - triangle[0, :2]).T
        if abs(np.linalg.det(edges)) < 1e-6: raise ValueError("Ground triangle must have nonzero horizontal area")
        inverse = np.linalg.inv(edges)
    if (not np.isfinite(cylinder).all() or cylinder[2] <= 0 or cylinder[3] >= cylinder[4] or
            not np.isfinite(shaft_z).all() or shaft_z[0] >= shaft_z[1]):
        raise ValueError("Finite cylinder and ordered shaft Z bounds required")
    output = result_path(output or directory / "pole_pair")
    output.mkdir(parents=True, exist_ok=True)
    regions, patches = [], []
    summary = [f"run={directory.resolve()}",
               f"ground triangle={triangle.tolist()}, height={height}" if triangle is not None else
               f"world-Z cylinder [cx,cy,radius,z_min,z_max]={list(cylinder)}",
               f"chunks={list(chunks)}; NO ALIGNMENT; shaft-only scoring Z={list(shaft_z)} m"]
    if manual_crops:
        summary[1] = "Manual selections override the spatial crop. Exact source returns."
    for ordinal, chunk in enumerate(chunks):
        source = directory / f"map_{int(chunk)}.pcd"
        cloud, header = read_pcd(source, with_header=True)
        xyz = np.asarray(cloud[:, :3], dtype=float)
        if triangle is not None:
            barycentric = (xyz[:, :2] - triangle[0, :2]) @ inverse.T
            above = xyz[:, 2] - (triangle[0, 2] + barycentric @ (triangle[1:, 2] - triangle[0, 2]))
            mask = (barycentric.min(axis=1) >= -1e-8) & (barycentric.sum(axis=1) <= 1 + 1e-8)
            mask &= (above >= 0) & (above <= height)
        else:
            mask = np.linalg.norm(xyz[:, :2] - cylinder[:2], axis=1) <= cylinder[2]
            mask &= (xyz[:, 2] >= cylinder[3]) & (xyz[:, 2] <= cylinder[4])
        if manual_crops:
            selected = read_manual_cloud(manual_crops[ordinal])
            if not len(selected): raise ValueError("Manual pole selection is empty")
            distance, indices = cKDTree(xyz).query(selected)
            if np.max(distance) > 1e-6:
                raise ValueError(f"Manual crop is not an unchanged subset of {source}: max difference {max(distance)} m")
            summary.append(f"manual={manual_crops[ordinal].resolve()}; {len(selected)} points; original source XYZ max difference={max(distance)} m")
            mask = np.zeros(len(cloud), dtype=bool); mask[indices] = True
        # The manually verified straight shaft is distinct from the base and
        # horizontal fittings. Keep the complete crop, score this declared
        # height band; never select points by a small fitting residual.
        fit_mask = mask & (xyz[:, 2] >= shaft_z[0]) & (xyz[:, 2] <= shaft_z[1])
        crop = np.asarray(cloud[mask, :4], dtype="<f4")
        # Runtime analysis products only; original PCD files are never changed.
        destination = output / f"prism_map_{chunk}.pcd"
        viewpoint = " ".join(header.get("VIEWPOINT", ["0", "0", "0", "1", "0", "0", "0"]))
        with destination.open("wb") as stream:
            stream.write(("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n"
                          "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                          f"WIDTH {len(crop)}\nHEIGHT 1\nVIEWPOINT {viewpoint}\n"
                          f"POINTS {len(crop)}\nDATA binary\n").encode("ascii"))
            stream.write(crop.tobytes())
        # Explicit source-row provenance and metric coordinates, not a digest.
        # The cropped output must be an unmodified subset of the actual PCD,
        # never points re-created from a different trajectory or map version.
        reloaded = read_pcd(destination)
        coordinate_change = float(np.max(np.abs(reloaded[:, :3] - cloud[mask, :3]))) if len(crop) else 0.
        if coordinate_change != 0 or not np.array_equal(reloaded[:, 3], cloud[mask, 3]):
            raise ValueError(f"Cropped PCD changed actual source coordinates/intensity: {source}")
        np.savetxt(output / f"source_rows_map_{chunk}.csv",
                   np.column_stack((np.flatnonzero(mask), crop)), delimiter=",", comments="",
                   fmt=["%d", "%.9f", "%.9f", "%.9f", "%.9f"],
                   header="source_row_zero_based,x,y,z,intensity")
        summary.append(f"source={source.resolve()}; points={len(cloud)}; VIEWPOINT={viewpoint}; cropped/source XYZ maximum difference={coordinate_change} m")
        regions.append(xyz[mask])
        # Retain the base in the exported prism/plot; exclude the ground
        # itself from an axis fit. This is not a residual-dependent filter.
        patch = xyz[fit_mask]
        patches.append(patch)
        span = float(np.ptp(patch[:, 2])) if len(patch) else 0.
        summary.append(f"map_{chunk}: selected points={len(crop)}, shaft points={len(patch)}, shaft Z span={span:.6f} m")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, panels = plt.subplots(1, 3, figsize=(14, 7))
    outline = triangle[[0, 1, 2, 0]] if triangle is not None else None
    for panel, dims in zip(panels, [(0, 1), (0, 2), (1, 2)]):
        for chunk, points in zip(chunks, regions):
            panel.scatter(points[:, dims[0]], points[:, dims[1]], s=2, alpha=.6, label=f"map_{chunk}")
        if outline is not None and not manual_crops: panel.plot(outline[:, dims[0]], outline[:, dims[1]], "k--", linewidth=.7)
        if dims[1] == 2 and outline is not None and not manual_crops:
            panel.plot(outline[:, dims[0]], outline[:, 2] + height, "k--", linewidth=.7)
            panel.set_ylim(triangle[:, 2].min() - .1, triangle[:, 2].max() + height + .1)
        if dims[1] == 2:
            for z in shaft_z: panel.axhline(z, color="gray", linestyle=":", linewidth=.7)
        panel.set_xlabel(f"World {'XYZ'[dims[0]]} (m)")
        panel.set_ylabel(f"World {'XYZ'[dims[1]]} (m)")
        panel.set_aspect("equal"); panel.grid(alpha=.2)
    panels[0].legend()
    fig.suptitle("Manual pole selections: actual saved returns, no registration" if manual_crops else
                 "Pole region: actual saved returns; dotted Z band = straight shaft scoring")
    fig.tight_layout(); fig.savefig(output / "pole_pair.png", dpi=160); plt.close(fig)

    supported = True
    for chunk, patch in zip(chunks, patches):
        if len(patch) < 20 or np.ptp(patch[:, 2]) < 2 or len(np.unique(np.floor(patch[:, 2] / .25))) < 8:
            supported = False
            summary.append(f"map_{chunk}: insufficient height coverage for a pole axis")
            continue
        eigenvalues, eigenvectors = np.linalg.eigh(np.cov(patch.T))
        if abs(eigenvectors[2, -1]) < .85 or eigenvalues[1] > .1 * eigenvalues[2]:
            supported = False
            summary.append(f"map_{chunk}: prism is not supported as one slender vertical structure")
    rows = []
    if supported:
        for a, b in ((0, 1), (1, 0)):
            distances, _ = cKDTree(patches[b]).query(patches[a])
            summary.append(f"map_{chunks[a]} to map_{chunks[b]} observed-surface NN median/P95={np.percentile(distances,[50,95]).tolist()} m; sampling/occlusion included")
        low = max(p[:, 2].min() for p in patches)
        high = min(p[:, 2].max() for p in patches)
        # A lamp has a base and horizontal fittings: a single line fit to the
        # entire crop measures its shape, not pose accuracy. Do not report it.
        for bottom in np.arange(low, high - .249, .25):
            slices = [p[(p[:, 2] >= bottom) & (p[:, 2] < bottom + .25), :2] for p in patches]
            if min(map(len, slices)) < 4: continue
            centers = [np.median(p, axis=0) for p in slices]
            rows.append((bottom + .125, len(slices[0]), len(slices[1]), *centers[0], *centers[1],
                         np.linalg.norm(centers[0] - centers[1])))
        if rows:
            summary.append(f"equal-height XY surface separation median/P95/max={np.percentile(np.asarray(rows)[:, -1], [50,95,100]).tolist()} m")
        else:
            supported = False
            summary.append("No sufficiently populated equal-height slices")
    with (output / "pole_slices.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(("z_m", "count_a", "count_b", "surface_x_a", "surface_y_a",
                         "surface_x_b", "surface_y_b", "same_height_xy_separation_m"))
        writer.writerows(rows)
    summary.append("Observed surface differences include sampling, occlusion and physical radius; no whole-lamp line-fit accuracy claim.")
    summary.append("Pole comparison: MEASURED, not an automatic acceptance." if supported else
                   "Pole comparison: UNASSESSED. The selected region does not support a valid pole-pair residual.")
    text = "\n".join(summary) + "\n"
    (output / "pole_summary.txt").write_text(text)
    print(text)


def compare_native_windows(native, run, output, cutoff_ns, window_length, deskew_ablation=False):
    """Paired actual-PCD shape comparison, not an absolute-position score.

    Windows/ROIs depend ONLY on native LIO, before the requested cutoff.
    A single rigid transform per whole window removes its arbitrary global
    pose. Never align individual scans, blocks or surfaces independently.
    """
    import csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if not 20 <= window_length <= 30: raise ValueError("Window length must be in [20, 30] m")
    output = result_path(output or run / "native_comparison")
    output.mkdir(parents=True, exist_ok=True)
    poses = np.loadtxt(native / "Log/pcd/lidar_poses.txt")
    poses = poses[poses[:, 0] < cutoff_ns / 1e9]
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(poses[:, 1:4], axis=0), axis=1))]
    windows = []
    start = int(np.searchsorted(arc, 2.0))
    while start + 10 < len(poses):
        end = min(int(np.searchsorted(arc, arc[start] + window_length)),
                  int(np.searchsorted(poses[:, 0], poses[start, 0] + 12)), len(poses) - 1)
        while arc[end] - arc[start] > 30 or poses[end, 0] - poses[start, 0] > 12: end -= 1
        if end - start < 10 or arc[end] - arc[start] < 20: break
        windows.append((start, end))
        start = end + 1
    frames = np.genfromtxt(run / "frames.csv", delimiter=",", names=True, skip_header=3)
    fused_poses = np.loadtxt(run / "trajectory_optimized.txt") if deskew_ablation else None
    offsets = np.r_[0, np.cumsum(frames["count"].astype(np.int64))]
    chunks = [read_pcd(p) for p in sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[-1]))]
    chunk_ends = np.cumsum([len(c) for c in chunks])
    if not len(chunks) or chunk_ends[-1] != offsets[-1]:
        raise ValueError("Actual saved PCD point count does not match the raw frame index")
    with (output / "windows.csv").open("w") as stream, (output / "sections.csv").open("w") as metrics:
        rows, scores = csv.writer(stream), csv.writer(metrics)
        rows.writerow(("window", "begin", "end", "native_arc_m", "matched_frames", "expected_frames", "status"))
        scores.writerow(("window", "surface", "x", "y", "z", "paired_points", "native_q90_m", "fused_q90_m", "difference_m", "ratio",
                         "native_deskew_fused_pose_q90_m", "fused_deskew_native_pose_q90_m",
                         "paired_normal_change_q90_m", "paired_tangent_change_q90_m"))
        for number, (begin, end) in enumerate(windows):
            a_parts, b_parts, time_parts, c_parts, d_parts = [], [], [], [], []
            unpaired_returns = 0
            for k in range(begin, end + 1):
                stamp = poses[k, 0]
                i = int(np.argmin(abs(frames["stamp"] - stamp)))
                if abs(frames[i]["stamp"] - stamp) > 2e-6: continue
                cloud = read_pcd(native / "Log/pcd" / f"{stamp:.6f}.pcd")
                # Native IMU deskew removes ONLY the already-consumed prefix
                # of overlapping scan envelopes; point order is retained.
                stop, count = offsets[i + 1], len(cloud)
                first = stop - count
                if first < offsets[i]:
                    print(f"native window {number}: source count differs at {stamp:.9f}; whole window will be unscored")
                    unpaired_returns += 1
                    continue
                c = int(np.searchsorted(chunk_ends, first, side="right"))
                previous = chunk_ends[c - 1] if c else 0
                fused = chunks[c][first - previous:min(stop, chunk_ends[c]) - previous]
                if stop > chunk_ends[c]: fused = np.concatenate((fused, chunks[c + 1][:stop - chunk_ends[c]]))
                if not np.array_equal(cloud[:, 3], fused[:, 3]):
                    print(f"native window {number}: source-return order differs at {stamp:.9f}; whole window will be unscored")
                    unpaired_returns += 1
                    continue
                a_parts.append(np.asarray(cloud[:, :3], dtype=float))
                b_parts.append(np.asarray(fused[:, :3], dtype=float))
                time_parts.append(np.full(count, stamp - poses[begin, 0]))
                if deskew_ablation:
                    rn, rf = Rotation.from_quat(poses[k, 4:8]), Rotation.from_quat(fused_poses[i, 4:8])
                    c_parts.append((rf * rn.inv()).apply(a_parts[-1] - poses[k, 1:4]) + fused_poses[i, 1:4])
                    d_parts.append((rn * rf.inv()).apply(b_parts[-1] - fused_poses[i, 1:4]) + poses[k, 1:4])
            complete = len(a_parts) == end - begin + 1
            rows.writerow((number, f"{poses[begin,0]:.9f}", f"{poses[end,0]:.9f}", arc[end] - arc[begin],
                           len(a_parts), end - begin + 1,
                           "complete" if complete else "unpaired source returns" if unpaired_returns else "missing frames"))
            if not complete:
                print(f"native window {number}: missing frames {len(a_parts)}/{end-begin+1}; not scored")
                continue
            a, b, times = np.concatenate(a_parts), np.concatenate(b_parts), np.concatenate(time_parts)
            # Rigid Procrustes on paired returns; no ICP/rematching and no
            # per-scan correction. It cannot remove internal map distortion.
            variants = [a, b]
            labels = ["Native LIO", "Fused"]
            if deskew_ablation:
                variants.extend((np.concatenate(c_parts), np.concatenate(d_parts)))
                labels.extend(("Native deskew + fused poses", "Fused deskew + native poses"))
            ma = a[::8].mean(axis=0)
            for j in range(1, len(variants)):
                points = variants[j]; mb = points[::8].mean(axis=0)
                u, _, vt = np.linalg.svd((points[::8] - mb).T @ (a[::8] - ma))
                rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
                variants[j] = (points - mb) @ rotation + ma
            sample = a[::4]
            cells, inverse, counts = np.unique(np.floor(sample).astype(np.int64), axis=0,
                                               return_inverse=True, return_counts=True)
            order = np.argsort(inverse, kind="stable"); starts = np.r_[0, np.cumsum(counts)]
            candidates = {"ground": [], "vertical": []}
            for cell in np.flatnonzero(counts >= 60):
                ids = order[starts[cell]:starts[cell+1]]
                points = sample[ids]; center = points.mean(axis=0)
                eig, vectors = np.linalg.eigh(np.cov(points.T))
                if eig[1] < .01 or eig[0] > .15 * eig[1] or np.ptp(times[::4][ids]) < 1: continue
                normal = vectors[:, 0]
                kind = "ground" if abs(normal[2]) > .85 else "vertical" if abs(normal[2]) < .3 else None
                if kind: candidates[kind].append((eig[0], cell, center, normal, vectors[:, 2]))
            chosen = []
            for kind, options in candidates.items():
                options.sort(key=lambda x: x[0])
                for quantile in ((.5, .9) if kind == "ground" else (.5,)):
                    if options: chosen.append((kind, options[round(quantile * (len(options)-1))]))
            if not chosen: print(f"native window {number}: no repeated surface support"); continue
            fig, axes = plt.subplots(len(chosen), len(variants), figsize=(6*len(variants), 3*len(chosen)), squeeze=False)
            for section, (kind, (_, cell, center, normal, tangent)) in enumerate(chosen):
                # The SAME return IDs define both sections, even if fused
                # points have moved outside the native voxel (do not hide them).
                mask = np.all(np.floor(a).astype(np.int64) == cells[cell], axis=1)
                widths = []
                for column, variant in enumerate(variants):
                    points = variant[mask]
                    height = (points - center) @ normal
                    width = np.ptp(np.percentile(height, [5, 95])); widths.append(width)
                    axes[section, column].scatter((points-center) @ tangent, height*100,
                        c=times[mask], cmap="turbo", s=.7, alpha=.5, rasterized=True)
                    axes[section, column].set_title(f"{labels[column]} {kind}: {width*100:.2f} cm")
                    axes[section, column].set_xlabel("Position along common native surface (m)")
                    axes[section, column].set_ylabel("Normal displacement (cm)")
                low = min(ax.get_ylim()[0] for ax in axes[section]); high = max(ax.get_ylim()[1] for ax in axes[section])
                left = min(ax.get_xlim()[0] for ax in axes[section]); right = max(ax.get_xlim()[1] for ax in axes[section])
                for ax in axes[section]:
                    ax.set_ylim(low, high); ax.set_xlim(left, right); ax.grid(alpha=.2)
                # Paired displacement removes the shared natural roughness.
                # Its spread measures added shape change, not absolute error;
                # no per-frame alignment or centimetre acceptance threshold.
                deformation = variants[1][mask] - a[mask]
                normal_change = np.ptp(np.percentile(deformation @ normal, [5, 95]))
                tangent_change = np.ptp(np.percentile(deformation @ tangent, [5, 95]))
                scores.writerow((number, kind, *center, int(mask.sum()), *widths[:2],
                                 widths[1]-widths[0], widths[1]/max(widths[0], 1e-9),
                                 *(widths[2:] if deskew_ablation else ("", "")), normal_change, tangent_change))
                print(f"native window {number} {kind}: arc={arc[end]-arc[begin]:.2f}m "
                      f"widths cm={[round(100*w, 2) for w in widths]}; no absolute-thickness threshold")
            fig.suptitle(f"SAME returns, {poses[begin,0]:.3f}–{poses[end,0]:.3f}; {arc[end]-arc[begin]:.2f} m; one rigid window alignment")
            fig.tight_layout(); fig.savefig(output / f"window_{number:02d}.png", dpi=160); plt.close(fig)


def compare_map(directory, frame_stride, point_stride, reference, lever):
    if frame_stride < 1 or point_stride < 1:
        raise ValueError("Map sampling strides must be positive")
    raw = np.loadtxt(directory / "trajectory_lio.txt", ndmin=2)
    optimized = np.loadtxt(directory / "trajectory_optimized.txt", ndmin=2)
    if raw.shape != optimized.shape or not np.array_equal(raw[:, 0], optimized[:, 0]):
        raise ValueError("Map comparison requires matching trajectory timestamps")
    translation, rotation = None, None
    with (directory / "frames.csv").open() as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("Missing frame archive index")
            if line.startswith("# lidar_to_imu_translation "):
                translation = np.fromstring(line.split("translation ", 1)[1], sep=" ")
            if line.startswith("# lidar_to_imu_rotation_row_major "):
                rotation = np.fromstring(line.split("row_major ", 1)[1], sep=" ").reshape(3, 3)
            if not line.startswith("#"):
                names = line.strip().split(",")
                index = np.atleast_1d(np.genfromtxt(stream, delimiter=",", names=names))
                break
    if translation is None or rotation is None or len(index) != len(raw):
        raise ValueError("Archive calibration/index does not match trajectory")
    with (directory / "scans.bin").open("rb") as stream:
        version = stream.read(8)
        if version not in (b"LIVORTK1", b"LIVORTK2", b"LIVORTK3"):
            raise ValueError("Unsupported point archive")
    continuous = np.loadtxt(directory / "trajectory_imu.txt", ndmin=2) if version == b"LIVORTK2" else None
    points = np.memmap(directory / "scans.bin", dtype="<f4", mode="r", offset=8).reshape(-1, 5)
    before, after, stamps, ranges = [], [], [], []
    for i in range(0, len(index), frame_stride):
        record = index[i]
        start = (int(record["offset_bytes"]) - 8) // 20
        scan = points[start:start + int(record["count"]):point_stride]
        local = scan[:, :3].astype(np.float64) @ rotation.T + translation
        distance = np.linalg.norm(local, axis=1)
        valid = np.isfinite(local).all(axis=1) & (distance >= 5) & (distance <= 70)
        scan, local, distance = scan[valid], local[valid], distance[valid]
        world = Rotation.from_quat(raw[i, 4:8]).apply(local) + raw[i, 1:4]
        # Mapper evaluates correction on its millisecond grid.
        query = np.minimum(record["stamp"], record["begin"] + np.floor(scan[:, 4].astype(np.float64) + 0.5) * 0.001)
        if version == b"LIVORTK3":
            # Native-deskew archive: mapper applies one final rigid scan-end
            # pose, never a point-time correction to already-deskewed returns.
            refined = Rotation.from_quat(optimized[i, 4:8]).apply(local) + optimized[i, 1:4]
        elif continuous is None:
            r, t = correction_samples(raw, optimized, query)
            refined = r.apply(world) + t
        else:
            r, t = interpolate_poses(continuous, query)
            refined = r.apply(local) + t
            r0, t0 = interpolate_poses(raw, query)
            world = r0.apply(local) + t0
        before.append(world)
        after.append(refined)
        stamps.append(np.full(len(world), record["stamp"]))
        ranges.append(distance)
    before, after = np.concatenate(before), np.concatenate(after)
    stamps, ranges = np.concatenate(stamps), np.concatenate(ranges)
    cell = np.floor(before).astype(np.int64)
    order = np.lexsort((cell[:, 2], cell[:, 1], cell[:, 0]))
    cell = cell[order]
    starts = np.r_[0, np.flatnonzero(np.any(cell[1:] != cell[:-1], axis=1)) + 1, len(cell)]
    measurements = []
    for start, end in zip(starts[:-1], starts[1:]):
        if end - start < 30:
            continue
        ids = order[start:end]
        if np.ptp(stamps[ids]) < 5 or len(np.unique(stamps[ids])) < 3:
            continue
        source, target = before[ids], after[ids]
        eig, vectors = np.linalg.eigh(np.cov(source.T, bias=True))
        if eig[1] < 0.003 or eig[0] > 0.01 or eig[0] > 0.12 * eig[1]:
            continue
        baseline = np.sqrt(max(0, eig[0]))
        changed = np.sqrt(max(0, np.linalg.eigvalsh(np.cov(target.T, bias=True))[0]))
        measurements.append([baseline, changed, np.median(ranges[ids]), abs(vectors[2, 0])])
    measurements = np.asarray(measurements).reshape(-1, 4)
    print(f"\nMap SAME-point temporal surface comparison: sampled={len(before)}, "
          f"planar cells={len(measurements)}, frame/point strides={frame_stride}/{point_stride}")
    print("Cells selected in native LIO map; this is a regression diagnostic, NOT independent geometry truth.")
    if continuous is not None:
        print("LIVORTK2 stores raw returns: native comparison uses endpoint pose interpolation, NOT exact native IMU deskew.")
    for low, high in [(5, 20), (20, 40), (40, 70)]:
        for label, mask in [("all", np.ones(len(measurements), dtype=bool)),
                            ("ground-like", measurements[:, 3] > 0.9),
                            ("wall-like", measurements[:, 3] < 0.2)]:
            chosen = measurements[mask & (measurements[:, 2] >= low) & (measurements[:, 2] < high)]
            if not len(chosen):
                print(f"{low}-{high}m {label}: insufficient temporal planar cells")
                continue
            change = chosen[:, 1] - chosen[:, 0]
            threshold = np.maximum(0.005, 0.1 * chosen[:, 0])
            print(f"{low}-{high}m {label}: cells={len(chosen)}, plane RMS thickness median "
                  f"{np.median(chosen[:, 0])*1000:.2f} -> {np.median(chosen[:, 1])*1000:.2f}mm, "
                  f"increase P95={np.percentile(change, 95)*1000:.2f}mm, "
                  f"within proposed regression bound={np.mean(change <= threshold):.2%}")

    # Additional check independent of the native-map voxel grouping. Hold out
    # non-keyframes and form correspondences once from rough INS poses, then
    # score exactly those same physical surface candidates for both outputs.
    # INS is only a search seed (0.8 m); it is not geometry ground truth.
    keys = np.atleast_1d(np.genfromtxt(directory / "keyframes.csv", delimiter=",", names=True))
    is_key = np.isin(np.round(raw[:, 0], 6), np.round(keys["stamp"], 6))
    times, positions, quaternions, _ = reference
    qi = np.clip(np.searchsorted(times, raw[:, 0]), 0, len(times) - 1)
    seed_r = Rotation.from_quat(quaternions[qi])
    seed_p = positions[qi] - seed_r.apply(lever)
    raw_r, opt_r = Rotation.from_quat(raw[:, 4:8]), Rotation.from_quat(optimized[:, 4:8])
    cache, scan_points = {}, {}

    def planes(i):
        if i in cache:
            return cache[i]
        record = index[i]
        start = (int(record["offset_bytes"]) - 8) // 20
        scan = points[start:start + int(record["count"])]
        local = scan[:, :3].astype(float) @ rotation.T + translation
        if continuous is not None:
            query = np.minimum(record["stamp"], record["begin"] + np.floor(scan[:, 4].astype(np.float64) + 0.5) * 0.001)
            rp, tp = interpolate_poses(continuous, query)
            local = opt_r[i].inv().apply(rp.apply(local) + tp - optimized[i, 1:4])
        distance = np.linalg.norm(local, axis=1)
        local = local[np.isfinite(local).all(axis=1) & (distance > 5) & (distance < 70)]
        scan_points[i] = local[::point_stride]
        _, inv, counts = np.unique(np.floor(local).astype(np.int64), axis=0, return_inverse=True, return_counts=True)
        centers = np.zeros((len(counts), 3)); np.add.at(centers, inv, local)
        centers /= counts[:, None]
        cov = np.zeros((len(counts), 3, 3))
        for axis in range(3):
            for other in range(3):
                np.add.at(cov[:, axis, other], inv, local[:, axis] * local[:, other])
        cov /= counts[:, None, None]
        cov -= centers[:, :, None] * centers[:, None, :]
        eig, vectors = np.linalg.eigh(cov)
        keep = (counts >= 6) & (eig[:, 1] > 0.003) & (eig[:, 0] < 0.0025) & (eig[:, 0] < 0.12 * eig[:, 1])
        result = centers[keep], vectors[keep, :, 0]
        cache[i] = result
        return result

    offsets = []
    occupancy = {width: [[], []] for width in (0.02, 0.05, 0.10)}
    pair_count = 0
    candidates = np.flatnonzero(~is_key)[::10]
    for b in candidates:
        previous = candidates[(raw[b, 0] - raw[candidates, 0] >= 5)]
        if not len(previous):
            continue
        distance = np.linalg.norm(seed_p[previous] - seed_p[b], axis=1)
        nearest = np.argmin(abs(distance - 30))
        if abs(distance[nearest] - 30) > 10:
            continue
        a = previous[nearest]
        ca, na = planes(a); cb, nb = planes(b)
        if len(ca) < 10 or len(cb) < 10:
            continue
        seeded = seed_r[a].inv().apply(seed_r[b].apply(cb) + seed_p[b] - seed_p[a])
        dist, match = cKDTree(ca).query(seeded, k=1)
        normals = seed_r[a].inv().apply(seed_r[b].apply(nb))
        good = (dist < 0.8) & (abs(np.einsum("ij,ij->i", normals, na[match])) > 0.97)
        if good.sum() < 10:
            continue
        aa, nn, bb = ca[match[good]], na[match[good]], cb[good]
        errors = []
        pa, pb = scan_points[a], scan_points[b]
        pa = pa[cKDTree(aa).query(pa, k=1)[0] < 0.7]
        pb = pb[cKDTree(bb).query(pb, k=1)[0] < 0.7]
        count = min(len(pa), len(pb))
        if count:
            pa = pa[np.linspace(0, len(pa) - 1, count, dtype=int)]
            pb = pb[np.linspace(0, len(pb) - 1, count, dtype=int)]
        for pipeline, (rotations, translations) in enumerate([(raw_r, raw[:, 1:4]), (opt_r, optimized[:, 1:4])]):
            relative = rotations[a].inv().apply(rotations[b].apply(bb) + translations[b] - translations[a])
            errors.append(abs(np.einsum("ij,ij->i", nn, relative - aa)))
            if count >= 100:
                other = rotations[a].inv().apply(rotations[b].apply(pb) + translations[b] - translations[a])
                for width, measurements in occupancy.items():
                    phase_scores = []
                    for phase in (0.0, 0.5):
                        cells_a = np.unique(np.floor(pa / width + phase).astype(np.int64), axis=0)
                        cells_b = np.unique(np.floor(other / width + phase).astype(np.int64), axis=0)
                        union = len(np.unique(np.concatenate((cells_a, cells_b)), axis=0))
                        phase_scores.append((len(cells_a) + len(cells_b) - union) / union)
                    measurements[pipeline].append(np.mean(phase_scores))
        offsets.append(np.column_stack((*errors, np.maximum(np.linalg.norm(aa, axis=1), np.linalg.norm(bb, axis=1)),
                                         abs(seed_r[a].apply(nn)[:, 2]))))
        pair_count += 1
    offsets = np.concatenate(offsets) if offsets else np.empty((0, 4))
    print(f"\nNon-keyframe cross-scan surfaces: pairs={pair_count}, matches={len(offsets)}, "
          "time separation >=5s, vehicle baseline 20..40m, fixed INS-seeded correspondences")
    print("Surface offsets, not truth accuracy; centroid/end-pose approximation excludes within-scan deformation.")
    for low, high in [(5, 20), (20, 40), (40, 70)]:
        for label, mask in [("ground-like", offsets[:, 3] > 0.9), ("wall-like", offsets[:, 3] < 0.2)]:
            values = offsets[mask & (offsets[:, 2] >= low) & (offsets[:, 2] < high)]
            if len(values):
                print(f"{low}-{high}m {label}: n={len(values)}, raw->optimized |plane offset| "
                      f"median={np.median(values[:, 0])*1000:.1f}->{np.median(values[:, 1])*1000:.1f}mm, "
                      f"P95={np.percentile(values[:, 0],95)*1000:.1f}->{np.percentile(values[:, 1],95)*1000:.1f}mm, "
                      f"optimized >5cm={np.mean(values[:, 1] > 0.05):.2%}")
    print("3D occupancy auxiliary: same surface ROI, balanced point count, two grid phases; occlusion/sampling still matter.")
    for width, scores in occupancy.items():
        if scores[0]:
            print(f"{width*100:.0f}cm occupancy Jaccard: pairs={len(scores[0])}, "
                  f"median raw->optimized {np.median(scores[0]):.2%}->{np.median(scores[1]):.2%}; NOT an accuracy certificate")


def fault_relay(args):
    """Corrupt only external references during actual-data replay; never edit a bag."""
    import csv
    import rclpy
    from rclpy.node import Node
    if not args.fault_outage or not args.quality_log:
        raise ValueError("--fault-relay requires --fault-outage START END and --quality-log")
    args.quality_log = result_path(args.quality_log)
    args.quality_log.parent.mkdir(parents=True, exist_ok=True)
    for start, end in args.fault_outage:
        if not 0 <= start < end:
            raise ValueError("Each outage must have 0 <= START < END")
    rclpy.init()
    node = Node("rtk_failure_replay")
    status_publisher = node.create_publisher(DiagnosticArray, "/validation/ins_status", 1000)
    odom_publisher = node.create_publisher(Odometry, "/validation/odometry", 1000)
    origin = None
    previous = None
    with args.quality_log.open("w", buffering=1) as output:
        rows = csv.writer(output)
        rows.writerow(("stamp", "mode", "satellites"))

        def failed(stamp):
            nonlocal origin
            if origin is None:
                origin = stamp
                node.get_logger().info(f"Fault replay reference origin: {origin:.9f}")
            elapsed = stamp - origin
            for start, end in args.fault_outage:
                if start <= elapsed < end:
                    return True
                if end <= elapsed < end + args.fault_recovery_flicker_sec:
                    return int((elapsed - end) / 0.1) % 2 == 0
            return False

        def status_callback(message):
            nonlocal previous
            stamp = stamp_seconds(message.header.stamp)
            bad = failed(stamp)
            mode, satellites = -1, -1
            for diagnostic in message.status:
                for value in diagnostic.values:
                    if value.key == "ins_pos_mode":
                        if bad:
                            value.value = "1"
                        mode = int(value.value)
                    if value.key == "gps_num_sats":
                        satellites = int(value.value)
            status_publisher.publish(message)
            rows.writerow((f"{stamp:.9f}", mode, satellites))
            if mode != previous:
                node.get_logger().info(f"Injected quality at {stamp:.9f}: mode {mode}")
                previous = mode

        def odom_callback(message):
            if failed(stamp_seconds(message.header.stamp)):
                message.pose.pose.position.x += 100
                message.pose.pose.position.y -= 100
                message.pose.pose.position.z += 40
                message.twist.twist.linear.x += 100
                message.pose.pose.orientation.x = 1.0
                message.pose.pose.orientation.y = message.pose.pose.orientation.z = 0.0
                message.pose.pose.orientation.w = 0.0
            odom_publisher.publish(message)

        node.create_subscription(DiagnosticArray, "/imu_data/ins_status", status_callback, 1000)
        node.create_subscription(Odometry, "/imu_data/odometry", odom_callback, 1000)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path,
                        default=Path("/home/project/data/haibo/wuhu_livo/ros2bag"))
    parser.add_argument("--trajectory", type=Path, nargs="+",
                        default=[RESULTS_ROOT / "Log/result/wuhu_truck29.txt"])
    parser.add_argument("--rear-axle-to-imu", action="store_true",
                        help="trajectory is physical IMU; compare its predicted rear axle to INS")
    parser.add_argument("--imu-to-rear-axle", type=float, nargs=3,
                        default=(0.904, -4.679, -1.699), metavar=("X", "Y", "Z"))
    parser.add_argument("--time-offset-sec", type=float, default=0.0,
                        help="subtract from INS timestamps, matching rtk.time_offset_sec")
    parser.add_argument("--max-gap", type=float, default=0.05)
    parser.add_argument("--max-trajectory-gap", type=float, default=0.20,
                        help="Do not score road length across missing 10Hz trajectory samples")
    parser.add_argument("--status-max-age", type=float, default=0.1)
    parser.add_argument("--accepted-modes", type=int, nargs="+", default=[4])
    parser.add_argument("--minimum-satellites", type=int, default=12)
    parser.add_argument("--recovery-stable-sec", type=float, default=0.5)
    parser.add_argument("--quality-log", type=Path,
                        help="read recorded relay quality for evaluation; write it in --fault-relay mode")
    parser.add_argument("--fault-relay", action="store_true",
                        help="publish /validation references with bad RTK injected; raw IMU/LiDAR untouched")
    parser.add_argument("--fault-outage", nargs=2, type=float, action="append", metavar=("START", "END"),
                        help="bad RTK interval in seconds from first reference; repeat for multiple outages")
    parser.add_argument("--fault-recovery-flicker-sec", type=float, default=0.6)
    parser.add_argument("--map-run", type=Path,
                        help="fusion result directory with scans.bin; compare identical surface samples")
    parser.add_argument("--pcd-sections", type=Path,
                        help="inspect ACTUAL saved PCD sections in this run; skips trajectory/bag metrics")
    parser.add_argument("--pole-pair-run", type=Path,
                        help="inspect a selected roadside pole in two ACTUAL saved PCD chunks")
    parser.add_argument("--pole-manual-crops", type=Path, nargs=2,
                        help="two manual PCD/CloudCompare v5.0 cloud crops; exact source subsets, overrides prism")
    parser.add_argument("--pole-cylinder", type=float, nargs=5, default=(-10.75, -274.0, 2.5, 43.0, 54.5),
                        help="world-Z search cylinder: center X/Y, radius, minimum/maximum Z (metres)")
    parser.add_argument("--pole-shaft-z", type=float, nargs=2, default=(45.5, 48.5),
                        help="declared straight shaft height band; exclude base/fittings from numerical scoring")
    parser.add_argument("--pole-ground-triangle", type=float, nargs=9,
                        help="three XYZ ground vertices defining the exact pole extraction prism")
    parser.add_argument("--pole-height-m", type=float, default=8.0)
    parser.add_argument("--chunk-analysis-run", type=Path,
                        help="inspect one actual multi-scan chunk, its online/final errors and ground bands")
    parser.add_argument("--chunk-index", type=int, default=35)
    parser.add_argument("--pole-chunks", type=int, nargs=2, default=(0, 10))
    parser.add_argument("--section-selection", type=Path,
                        help="reuse an existing pcd_sections.csv for identical world-space ROIs")
    parser.add_argument("--analysis-output", type=Path,
                        help="Analysis output inside results/; defaults to the input run directory")
    parser.add_argument("--native-reference", type=Path,
                        help="Native LIO run with dense per-frame Log/pcd timestamped PCDs; requires --map-run")
    parser.add_argument("--reference-cutoff-ns", type=int, default=1785910280200745820,
                        help="Exclude native-reference frames at/after this Unix-nanosecond timestamp")
    parser.add_argument("--window-length-m", type=float, default=20.0,
                        help="Native comparison arc length, 20–30 m (also capped at 12 s)")
    parser.add_argument("--deskew-ablation", action="store_true",
                        help="Separate scan deskew from inter-frame pose errors in native-window comparisons")
    parser.add_argument("--deskew-surface-audit-run", type=Path,
                        help="diagnose fixed source surfaces under reference-based scan velocity; no map or production update")
    parser.add_argument("--reference-sections", action="store_true",
                        help="diagnostic INS projection of raw returns in SAME selected sections, not saved fused PCDs")
    parser.add_argument("--map-frame-stride", type=int, default=5)
    parser.add_argument("--map-point-stride", type=int, default=4)
    args = parser.parse_args()
    ros_logs = result_path(os.environ.get("ROS_LOG_DIR") or RESULTS_ROOT / "roslog")
    ros_logs.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(ros_logs)
    if args.chunk_analysis_run:
        inspect_chunk(args.chunk_analysis_run, args.chunk_index,
                      load_reference(args.bag, args.time_offset_sec), args.imu_to_rear_axle, args.analysis_output)
        return
    if args.pole_pair_run:
        compare_pole_chunks(args.pole_pair_run, args.pole_ground_triangle, args.pole_height_m,
                            args.pole_chunks, args.analysis_output, args.pole_manual_crops, args.pole_cylinder, args.pole_shaft_z)
        return
    if args.native_reference:
        if not args.map_run: parser.error("--native-reference requires --map-run")
        compare_native_windows(args.native_reference, args.map_run, args.analysis_output,
                               args.reference_cutoff_ns, args.window_length_m, args.deskew_ablation)
        return
    if args.pcd_sections:
        reference = load_reference(args.bag, args.time_offset_sec) if args.reference_sections else None
        pcd_sections(args.pcd_sections, args.section_selection, reference, args.imu_to_rear_axle, args.analysis_output)
        return
    if args.fault_relay:
        fault_relay(args)
        return
    if args.max_gap <= 0 or args.status_max_age <= 0 or args.max_trajectory_gap <= 0:
        parser.error("Time bounds must be positive")
    if args.accepted_modes != [4]:
        parser.error("Only mode 4 may provide an accepted RTK reference")
    reference = load_reference(args.bag, args.time_offset_sec)
    if args.deskew_surface_audit_run:
        audit_deskew_surfaces(args.deskew_surface_audit_run, reference, args.imu_to_rear_axle, args.analysis_output, args)
        return
    if args.quality_log:
        quality = np.atleast_1d(np.genfromtxt(args.quality_log, delimiter=",", names=True))
        reference = (*reference[:3], np.column_stack((quality["stamp"] - args.time_offset_sec,
                                                      quality["mode"], quality["satellites"])))
    for trajectory in args.trajectory:
        compare_trajectory(trajectory, reference, args)
        if (args.rear_axle_to_imu and trajectory.name == "trajectory_optimized.txt" and
                (trajectory.parent / "trajectory_lio.txt").is_file()):
            audit_output = args.analysis_output / trajectory.parent.name / "startup_analysis" if args.analysis_output else None
            inspect_startup(trajectory.parent, reference, args.imu_to_rear_axle, audit_output)
    if args.map_run:
        compare_map(args.map_run, args.map_frame_stride, args.map_point_stride, reference, args.imu_to_rear_axle)


if __name__ == "__main__":
    main()
