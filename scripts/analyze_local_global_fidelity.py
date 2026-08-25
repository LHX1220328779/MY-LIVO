#!/usr/bin/env python3
"""Measure how much the global layer deforms the local-SLAM trajectory."""

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


def vec(row, prefix):
    return tuple(float(row[prefix + axis]) for axis in ("x", "y", "z"))


def quat(row, prefix):
    value = tuple(float(row[prefix + axis]) for axis in ("x", "y", "z", "w"))
    length = math.sqrt(sum(item * item for item in value))
    if length < 1.0e-12 or not math.isfinite(length):
        raise RuntimeError("invalid trajectory quaternion")
    return tuple(item / length for item in value)


def sub(left, right):
    return tuple(a - b for a, b in zip(left, right))


def norm(value):
    return math.sqrt(sum(item * item for item in value))


def qconj(value):
    return (-value[0], -value[1], -value[2], value[3])


def qmul(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate(rotation, value):
    pure = (value[0], value[1], value[2], 0.0)
    result = qmul(qmul(rotation, pure), qconj(rotation))
    return result[:3]


def angle_deg(rotation):
    return math.degrees(2.0 * math.acos(min(1.0, abs(rotation[3]))))


def relative(pose_a, pose_b):
    position_a, rotation_a = pose_a
    position_b, rotation_b = pose_b
    inverse = qconj(rotation_a)
    return rotate(inverse, sub(position_b, position_a)), qmul(inverse, rotation_b)


def percentile(values, percentage):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    location = (len(ordered) - 1) * percentage / 100.0
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    return (ordered[lower] * (upper - location) +
            ordered[upper] * (location - lower))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path,
                        default=Path("Log/backend/global_trajectory.csv"))
    parser.add_argument("--output", type=Path,
                        default=Path("Log/backend/local_global_fidelity.csv"))
    parser.add_argument("--scales", type=float, nargs="+",
                        default=(5.0, 20.0, 100.0))
    parser.add_argument("--segment-break-translation-m", type=float,
                        default=1.0)
    parser.add_argument("--segment-break-rotation-deg", type=float,
                        default=1.0)
    parser.add_argument("--allow-live-logs", action="store_true")
    args = parser.parse_args()

    active = active_mapping_processes()
    if active and not args.allow_live_logs:
        raise RuntimeError(
            "fastlivo_mapping is still writing the audit files (PID " +
            ",".join(str(pid) for pid in active) + "); stop it first")

    required = {
        "id", "timestamp", "local_tx", "local_ty", "local_tz",
        "local_qx", "local_qy", "local_qz", "local_qw", "global_tx",
        "global_ty", "global_tz", "global_qx", "global_qy", "global_qz",
        "global_qw"}
    with args.trajectory.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not required.issubset(reader.fieldnames or ()):
            raise RuntimeError("global trajectory schema is incomplete")
        source = list(reader)
    if len(source) < 2:
        raise RuntimeError("global trajectory has fewer than two poses")

    local = []
    global_poses = []
    distance = [0.0]
    segment_ids = [0]
    boundaries = []
    for expected, row in enumerate(source):
        if int(row["id"]) != expected:
            raise RuntimeError("trajectory IDs are not contiguous")
        local.append((vec(row, "local_t"), quat(row, "local_q")))
        global_poses.append((vec(row, "global_t"), quat(row, "global_q")))
        if expected:
            distance.append(distance[-1] + norm(sub(
                local[-1][0], local[-2][0])))
            local_step = relative(local[-2], local[-1])
            global_step = relative(global_poses[-2], global_poses[-1])
            step_translation_error = norm(sub(
                global_step[0], local_step[0]))
            step_rotation_error = angle_deg(qmul(
                qconj(local_step[1]), global_step[1]))
            is_boundary = step_translation_error > \
                args.segment_break_translation_m or \
                step_rotation_error > args.segment_break_rotation_deg
            segment_ids.append(segment_ids[-1] + int(is_boundary))
            if is_boundary:
                boundaries.append((expected, step_translation_error,
                                   step_rotation_error))

    rows = []
    summaries = {}
    for scale_m in args.scales:
        translation_errors = []
        rotation_errors = []
        scale_errors = []
        for begin in range(len(source) - 1):
            end = bisect.bisect_left(distance, distance[begin] + scale_m,
                                     lo=begin + 1)
            if end >= len(source):
                continue
            if segment_ids[begin] != segment_ids[end]:
                continue
            local_relative = relative(local[begin], local[end])
            global_relative = relative(global_poses[begin], global_poses[end])
            translation_error = norm(sub(global_relative[0], local_relative[0]))
            rotation_error = angle_deg(qmul(
                qconj(local_relative[1]), global_relative[1]))
            local_length = norm(local_relative[0])
            global_length = norm(global_relative[0])
            scale_error = abs(global_length / local_length - 1.0) \
                if local_length > 1.0e-6 else math.nan
            translation_errors.append(translation_error)
            rotation_errors.append(rotation_error)
            if math.isfinite(scale_error):
                scale_errors.append(scale_error)
            rows.append({
                "target_scale_m": scale_m,
                "start_id": begin,
                "end_id": end,
                "segment_id": segment_ids[begin],
                "path_span_m": distance[end] - distance[begin],
                "relative_translation_error_m": translation_error,
                "relative_rotation_error_deg": rotation_error,
                "relative_length_scale_error": scale_error,
            })
        summaries[scale_m] = (translation_errors, rotation_errors,
                              scale_errors)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(args.output)

    print(f"local/global fidelity: keyframes={len(source)}, "
          f"local_path={distance[-1]:.1f}m, "
          f"segment_boundaries={len(boundaries)}")
    if boundaries:
        print("global segment jumps: " + ", ".join(
            f"KF{keyframe}:{translation:.3f}m/{rotation:.3f}deg"
            for keyframe, translation, rotation in boundaries))
    for scale_m, values in summaries.items():
        translation, rotation, scale_error = values
        print(
            f"{scale_m:g}m relative deformation: "
            f"translation p50/p95/max={percentile(translation, 50):.4f}/"
            f"{percentile(translation, 95):.4f}/{max(translation):.4f}m, "
            f"rotation p50/p95/max={percentile(rotation, 50):.4f}/"
            f"{percentile(rotation, 95):.4f}/{max(rotation):.4f}deg, "
            f"length-scale p95/max={100.0 * percentile(scale_error, 95):.2f}/"
            f"{100.0 * max(scale_error):.2f}%")
    print(f"per-window audit: {args.output}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"local/global fidelity analysis failed: {error}")
