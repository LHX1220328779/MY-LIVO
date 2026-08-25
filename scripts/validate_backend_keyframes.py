#!/usr/bin/env python3
"""Validate backend keyframe invariants and optional raw-LIO pose equality."""

import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from statistics import median


REQUIRED_COLUMNS = {
    "id", "timestamp", "trigger", "tx", "ty", "tz",
    "qx", "qy", "qz", "qw", "cloud_points",
    "lio_observability_valid", "downsampled_features",
    "effective_features", "effective_feature_ratio",
    "mean_abs_residual_m", "translation_info_min",
    "translation_info_mid", "translation_info_max",
    "translation_condition_ratio", "translation_weak_x",
    "translation_weak_y", "translation_weak_z", "rotation_info_min",
    "rotation_info_mid", "rotation_info_max", "rotation_condition_ratio",
    "rotation_weak_x", "rotation_weak_y", "rotation_weak_z",
}


def fail(message):
    raise SystemExit(f"keyframe validation failed: {message}")


def load_keyframes(path):
    if not path.is_file():
        fail(f"CSV does not exist: {path}")
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_COLUMNS - set(reader.fieldnames or ())
        if missing:
            fail(f"CSV is missing columns: {sorted(missing)}")
        rows = list(reader)
    if not rows:
        fail("CSV contains no keyframes")

    parsed = []
    for row_index, row in enumerate(rows, start=2):
        try:
            keyframe = {
                "id": int(row["id"]),
                "timestamp": float(row["timestamp"]),
                "trigger": row["trigger"],
                "position": tuple(float(row[name]) for name in ("tx", "ty", "tz")),
                "quaternion": tuple(float(row[name]) for name in ("qx", "qy", "qz", "qw")),
                "cloud_points": int(row["cloud_points"]),
                "observability_valid": int(row["lio_observability_valid"]),
                "downsampled_features": int(row["downsampled_features"]),
                "effective_features": int(row["effective_features"]),
                "effective_feature_ratio": float(row["effective_feature_ratio"]),
                "mean_abs_residual_m": float(row["mean_abs_residual_m"]),
                "translation_info": tuple(float(row[name]) for name in (
                    "translation_info_min", "translation_info_mid",
                    "translation_info_max")),
                "translation_condition_ratio": float(
                    row["translation_condition_ratio"]),
                "translation_weak": tuple(float(row[name]) for name in (
                    "translation_weak_x", "translation_weak_y",
                    "translation_weak_z")),
                "rotation_info": tuple(float(row[name]) for name in (
                    "rotation_info_min", "rotation_info_mid",
                    "rotation_info_max")),
                "rotation_condition_ratio": float(
                    row["rotation_condition_ratio"]),
                "rotation_weak": tuple(float(row[name]) for name in (
                    "rotation_weak_x", "rotation_weak_y",
                    "rotation_weak_z")),
            }
        except ValueError as error:
            fail(f"invalid numeric value on CSV row {row_index}: {error}")
        values = (
            keyframe["timestamp"], *keyframe["position"],
            *keyframe["quaternion"],
        )
        if not all(math.isfinite(value) for value in values):
            fail(f"non-finite value on CSV row {row_index}")
        if keyframe["cloud_points"] <= 0:
            fail(f"non-positive cloud size on CSV row {row_index}")
        if keyframe["observability_valid"] not in (0, 1):
            fail(f"invalid observability flag on CSV row {row_index}")
        if keyframe["downsampled_features"] < 0 or \
                not 0 <= keyframe["effective_features"] <= \
                keyframe["downsampled_features"]:
            fail(f"invalid LIO feature counts on CSV row {row_index}")
        observability_values = (
            keyframe["effective_feature_ratio"],
            keyframe["mean_abs_residual_m"], *keyframe["translation_info"],
            keyframe["translation_condition_ratio"],
            *keyframe["translation_weak"], *keyframe["rotation_info"],
            keyframe["rotation_condition_ratio"], *keyframe["rotation_weak"])
        if not all(math.isfinite(value) for value in observability_values):
            fail(f"non-finite LIO observability on CSV row {row_index}")
        if keyframe["observability_valid"] and (
                not 0 <= keyframe["translation_condition_ratio"] <= 1 or
                not 0 <= keyframe["rotation_condition_ratio"] <= 1):
            fail(f"invalid LIO condition ratio on CSV row {row_index}")
        quaternion_norm = math.sqrt(sum(value * value for value in keyframe["quaternion"]))
        if abs(quaternion_norm - 1.0) > 1.0e-6:
            fail(f"non-unit quaternion on CSV row {row_index}: norm={quaternion_norm}")
        parsed.append(keyframe)
    return parsed


def validate_sequence(keyframes):
    if keyframes[0]["id"] != 0:
        fail(f"first keyframe id is {keyframes[0]['id']}, expected 0")
    if "first" not in keyframes[0]["trigger"].split("|"):
        fail("keyframe 0 does not have the first trigger")

    for expected_id, keyframe in enumerate(keyframes):
        if keyframe["id"] != expected_id:
            fail(
                f"keyframe id discontinuity: got {keyframe['id']}, "
                f"expected {expected_id}")
        if expected_id and keyframe["timestamp"] <= keyframes[expected_id - 1]["timestamp"]:
            fail(f"timestamps are not strictly increasing at keyframe {expected_id}")
        if expected_id and "first" in keyframe["trigger"].split("|"):
            fail(f"keyframe {expected_id} unexpectedly has the first trigger")


def load_trajectory(path):
    poses = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, start=1):
            values = line.split()
            if not values:
                continue
            if len(values) < 8:
                fail(f"trajectory row {line_number} has fewer than 8 columns")
            try:
                poses.append(tuple(float(value) for value in values[:8]))
            except ValueError as error:
                fail(f"invalid trajectory row {line_number}: {error}")
    if not poses:
        fail(f"trajectory contains no poses: {path}")
    return poses


def compare_trajectory(keyframes, trajectory, timestamp_tolerance):
    trajectory_index = 0
    maximum_time_error = 0.0
    maximum_position_error = 0.0
    maximum_angle_error_deg = 0.0
    for keyframe in keyframes:
        timestamp = keyframe["timestamp"]
        while (trajectory_index + 1 < len(trajectory) and
               abs(trajectory[trajectory_index + 1][0] - timestamp) <=
               abs(trajectory[trajectory_index][0] - timestamp)):
            trajectory_index += 1
        pose = trajectory[trajectory_index]
        time_error = abs(pose[0] - timestamp)
        if time_error > timestamp_tolerance:
            fail(
                f"no raw LIO pose near keyframe {keyframe['id']}: "
                f"time error={time_error:.9g}s")
        position_error = math.sqrt(sum(
            (left - right) ** 2
            for left, right in zip(keyframe["position"], pose[1:4])))
        quaternion_dot = abs(sum(
            left * right
            for left, right in zip(keyframe["quaternion"], pose[4:8])))
        quaternion_norm_product = math.sqrt(
            sum(value * value for value in keyframe["quaternion"]) *
            sum(value * value for value in pose[4:8]))
        if quaternion_norm_product < 1.0e-12:
            fail(f"invalid raw LIO quaternion near keyframe {keyframe['id']}")
        quaternion_dot /= quaternion_norm_product
        quaternion_dot = min(1.0, max(0.0, quaternion_dot))
        angle_error_deg = math.degrees(2.0 * math.acos(quaternion_dot))
        maximum_time_error = max(maximum_time_error, time_error)
        maximum_position_error = max(maximum_position_error, position_error)
        maximum_angle_error_deg = max(maximum_angle_error_deg, angle_error_deg)

    # The existing FAST-LIVO2 trajectory writer uses six decimal places.
    if maximum_position_error > 5.0e-5:
        fail(
            "keyframe pose differs from raw LIO trajectory: "
            f"max position error={maximum_position_error:.9g}m")
    if maximum_angle_error_deg > 0.01:
        fail(
            "keyframe orientation differs from raw LIO trajectory: "
            f"max angle error={maximum_angle_error_deg:.9g}deg")
    return maximum_time_error, maximum_position_error, maximum_angle_error_deg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--csv", type=Path,
        default=Path("/home/project/MY-LIVO2.0/Log/backend/keyframes.csv"))
    parser.add_argument(
        "--trajectory", type=Path,
        help="optional FAST-LIVO2 Log/result trajectory for pose equality check")
    parser.add_argument(
        "--timestamp-tolerance", type=float, default=1.0e-5)
    args = parser.parse_args()

    keyframes = load_keyframes(args.csv)
    validate_sequence(keyframes)
    timestamps = [keyframe["timestamp"] for keyframe in keyframes]
    intervals = [right - left for left, right in zip(timestamps, timestamps[1:])]
    trigger_counts = Counter()
    for keyframe in keyframes:
        trigger_counts.update(keyframe["trigger"].split("|"))

    print(
        f"keyframes: count={len(keyframes)}, "
        f"duration={timestamps[-1] - timestamps[0]:.3f}s")
    if intervals:
        print(
            f"keyframe dt: min={min(intervals):.3f}s, "
            f"median={median(intervals):.3f}s, max={max(intervals):.3f}s")
    print(
        "cloud points: "
        f"min={min(item['cloud_points'] for item in keyframes)}, "
        f"median={median(item['cloud_points'] for item in keyframes):.0f}, "
        f"max={max(item['cloud_points'] for item in keyframes)}")
    observable = [item for item in keyframes if item["observability_valid"]]
    if not observable:
        fail("no keyframe has valid LIO observability")
    print(
        "LIO observability: "
        f"valid={len(observable)}/{len(keyframes)}, "
        f"effective_features={min(item['effective_features'] for item in observable)}.."
        f"{max(item['effective_features'] for item in observable)}, "
        f"translation_condition_p50={median(item['translation_condition_ratio'] for item in observable):.6g}, "
        f"rotation_condition_p50={median(item['rotation_condition_ratio'] for item in observable):.6g}")
    print("triggers: " + ", ".join(
        f"{name}={count}" for name, count in sorted(trigger_counts.items())))

    if args.trajectory:
        if not args.trajectory.is_file():
            fail(f"trajectory does not exist: {args.trajectory}")
        comparison = compare_trajectory(
            keyframes, load_trajectory(args.trajectory),
            args.timestamp_tolerance)
        print(
            "raw LIO equality: "
            f"max_dt={comparison[0]:.9g}s, "
            f"max_position={comparison[1]:.9g}m, "
            f"max_angle={comparison[2]:.9g}deg")
    print("backend keyframe validation passed")


if __name__ == "__main__":
    main()
