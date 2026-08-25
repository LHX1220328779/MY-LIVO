#!/usr/bin/env python3
"""Audit every global keyframe against the time-interpolated RTK solution."""

import argparse
import bisect
import csv
import math
import os
from pathlib import Path


def active_mapping_processes():
    project_root = str(Path(__file__).resolve().parent.parent)
    active = []
    for process in Path("/proc").glob("[0-9]*"):
        try:
            command = (process / "cmdline").read_bytes().replace(
                b"\0", b" ").decode(errors="replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if "fastlivo_mapping" in command and project_root in command and \
                int(process.name) != os.getpid():
            active.append(int(process.name))
    return sorted(active)


def read_csv(path, required):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not required.issubset(reader.fieldnames or ()):
            missing = sorted(required - set(reader.fieldnames or ()))
            raise RuntimeError(f"{path} is missing columns: {missing}")
        rows = []
        for line, row in enumerate(reader, start=2):
            if not all(row.get(name, "") for name in required):
                print(f"warning: ignored incomplete row {line} in {path}")
                continue
            rows.append(row)
        return rows


def vector(row, prefix):
    return tuple(float(row[prefix + axis]) for axis in ("x", "y", "z"))


def quaternion(row, prefix):
    q = tuple(float(row[prefix + axis]) for axis in ("x", "y", "z", "w"))
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise RuntimeError("invalid quaternion in trajectory input")
    return tuple(value / norm for value in q)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(value, factor):
    return tuple(factor * x for x in value)


def norm(value):
    return math.sqrt(sum(x * x for x in value))


def wrap_radians(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def angular_distance_degrees(a, b):
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def slerp(a, b, alpha):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-x for x in b)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        mixed = add(scale(a, 1.0 - alpha), scale(b, alpha))
        length = math.sqrt(sum(x * x for x in mixed))
        return tuple(x / length for x in mixed)
    angle = math.acos(dot)
    denominator = math.sin(angle)
    return add(scale(a, math.sin((1.0 - alpha) * angle) / denominator),
               scale(b, math.sin(alpha * angle) / denominator))


def percentile(values, percentage):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * percentage / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return (ordered[lower] * (upper - position) +
            ordered[upper] * (position - lower))


def interpolate_rtk(rows, timestamps, timestamp, maximum_gap):
    upper = bisect.bisect_left(timestamps, timestamp)
    if upper == 0 or upper == len(rows):
        raise RuntimeError(f"RTK does not bracket keyframe time {timestamp:.9f}")
    lower = upper - 1
    lower_time = timestamps[lower]
    upper_time = timestamps[upper]
    gap = upper_time - lower_time
    if gap <= 0.0 or gap > maximum_gap:
        raise RuntimeError(
            f"RTK interpolation gap {gap:.6f}s at {timestamp:.9f}s")
    alpha = (timestamp - lower_time) / gap
    lower_position = vector(rows[lower], "")
    upper_position = vector(rows[upper], "")
    position = add(scale(lower_position, 1.0 - alpha),
                   scale(upper_position, alpha))
    orientation = slerp(quaternion(rows[lower], "q"),
                        quaternion(rows[upper], "q"), alpha)
    return position, orientation, gap, min(timestamp - lower_time,
                                           upper_time - timestamp)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory", type=Path,
        default=Path("Log/backend/global_trajectory.csv"))
    parser.add_argument(
        "--rtk-solutions", type=Path,
        default=Path("Log/backend/rtk_solutions.csv"))
    parser.add_argument(
        "--output", type=Path,
        default=Path("Log/backend/global_rtk_accuracy.csv"))
    parser.add_argument("--maximum-interpolation-gap", type=float,
                        default=0.05)
    parser.add_argument("--position-target", type=float, default=0.30)
    parser.add_argument("--yaw-target-deg", type=float, default=0.50)
    parser.add_argument("--enforce-target", action="store_true")
    parser.add_argument("--allow-live-logs", action="store_true")
    args = parser.parse_args()

    active = active_mapping_processes()
    if active and not args.allow_live_logs:
        raise RuntimeError(
            "fastlivo_mapping is still writing the audit files (PID " +
            ",".join(str(pid) for pid in active) + "); stop it first")

    trajectory = read_csv(args.trajectory, {
        "id", "timestamp", "local_tx", "local_ty", "local_tz",
        "local_qx", "local_qy", "local_qz", "local_qw", "global_tx",
        "global_ty", "global_tz", "global_qx", "global_qy", "global_qz",
        "global_qw"})
    solutions = read_csv(args.rtk_solutions, {
        "timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"})
    if not trajectory or len(solutions) < 2:
        raise RuntimeError("trajectory or RTK solution log is empty")
    solution_times = [float(row["timestamp"]) for row in solutions]
    if any(right <= left for left, right in zip(solution_times,
                                                 solution_times[1:])):
        raise RuntimeError("RTK solution timestamps are not strictly ordered")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    position_errors = []
    yaw_errors = []
    attitude_errors = []
    cumulative_distance = 0.0
    previous_local = None
    previous_correction = None
    previous_gradient = None
    previous_distance = 0.0
    first_position_failure = None
    first_yaw_failure = None
    rows_out = []
    maximum_endpoint_distance = 0.0

    for expected_id, row in enumerate(trajectory):
        keyframe_id = int(row["id"])
        if keyframe_id != expected_id:
            raise RuntimeError("trajectory keyframe IDs are not contiguous")
        timestamp = float(row["timestamp"])
        local_position = vector(row, "local_t")
        global_position = vector(row, "global_t")
        local_orientation = quaternion(row, "local_q")
        global_orientation = quaternion(row, "global_q")
        rtk_position, rtk_orientation, gap, endpoint = interpolate_rtk(
            solutions, solution_times, timestamp,
            args.maximum_interpolation_gap)
        maximum_endpoint_distance = max(maximum_endpoint_distance, endpoint)
        if previous_local is not None:
            cumulative_distance += norm(sub(local_position, previous_local))
        correction = sub(global_position, local_position)
        position_vector = sub(global_position, rtk_position)
        position_error = norm(position_vector)
        yaw_error = abs(math.degrees(wrap_radians(
            yaw(global_orientation) - yaw(rtk_orientation))))
        attitude_error = angular_distance_degrees(global_orientation,
                                                  rtk_orientation)
        gradient = (0.0, 0.0, 0.0)
        gradient_norm = 0.0
        curvature_norm = 0.0
        if previous_correction is not None:
            interval = cumulative_distance - previous_distance
            if interval > 1.0e-9:
                gradient = scale(sub(correction, previous_correction),
                                 1.0 / interval)
                gradient_norm = norm(gradient)
                if previous_gradient is not None:
                    curvature_norm = (
                        norm(sub(gradient, previous_gradient)) / interval)
                previous_gradient = gradient
        position_errors.append(position_error)
        yaw_errors.append(yaw_error)
        attitude_errors.append(attitude_error)
        if position_error > args.position_target and first_position_failure is None:
            first_position_failure = (keyframe_id, timestamp, cumulative_distance,
                                      position_error)
        if yaw_error > args.yaw_target_deg and first_yaw_failure is None:
            first_yaw_failure = (keyframe_id, timestamp, cumulative_distance,
                                 yaw_error)
        rows_out.append({
            "id": keyframe_id, "timestamp": timestamp,
            "distance_m": cumulative_distance,
            "rtk_x": rtk_position[0], "rtk_y": rtk_position[1],
            "rtk_z": rtk_position[2],
            "global_x": global_position[0], "global_y": global_position[1],
            "global_z": global_position[2],
            "error_x": position_vector[0], "error_y": position_vector[1],
            "error_z": position_vector[2], "position_error_m": position_error,
            "yaw_error_deg": yaw_error,
            "attitude_error_deg": attitude_error,
            "correction_x": correction[0], "correction_y": correction[1],
            "correction_z": correction[2],
            "correction_gradient_m_per_m": gradient_norm,
            "correction_curvature_per_m": curvature_norm,
            "rtk_interpolation_gap_sec": gap,
            "rtk_endpoint_distance_sec": endpoint,
        })
        previous_local = local_position
        previous_correction = correction
        previous_distance = cumulative_distance

    fieldnames = list(rows_out[0])
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows_out)
    temporary.replace(args.output)

    def summary(name, values, target):
        within = 100.0 * sum(value <= target for value in values) / len(values)
        print(f"{name}: p50={percentile(values, 50):.4f}, "
              f"p95={percentile(values, 95):.4f}, "
              f"p99={percentile(values, 99):.4f}, "
              f"max={max(values):.4f}, last={values[-1]:.4f}, "
              f"within_target={within:.1f}%")

    print(f"full trajectory: keyframes={len(rows_out)}, "
          f"duration={float(trajectory[-1]['timestamp']) - float(trajectory[0]['timestamp']):.3f}s, "
          f"path={cumulative_distance:.1f}m, "
          f"RTK_endpoint_max={maximum_endpoint_distance * 1000.0:.3f}ms")
    summary("position [m]", position_errors, args.position_target)
    summary("yaw [deg]", yaw_errors, args.yaw_target_deg)
    summary("full attitude [deg]", attitude_errors, args.yaw_target_deg)
    if first_position_failure:
        keyframe_id, timestamp, distance, error = first_position_failure
        print(f"first position target violation: KF={keyframe_id}, "
              f"dt={timestamp - float(trajectory[0]['timestamp']):.3f}s, "
              f"path={distance:.1f}m, error={error:.4f}m")
    if first_yaw_failure:
        keyframe_id, timestamp, distance, error = first_yaw_failure
        print(f"first yaw target violation: KF={keyframe_id}, "
              f"dt={timestamp - float(trajectory[0]['timestamp']):.3f}s, "
              f"path={distance:.1f}m, error={error:.4f}deg")
    print(f"per-keyframe audit: {args.output}")

    passed = max(position_errors) <= args.position_target and \
        max(yaw_errors) <= args.yaw_target_deg
    if args.enforce_target and not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"global/RTK trajectory analysis failed: {error}")
