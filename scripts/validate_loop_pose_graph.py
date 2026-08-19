#!/usr/bin/env python3
"""Validate robust loop factors, final trajectory, and anti-collapse invariants."""

import argparse
import csv
import math
import statistics
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


def read_csv(path: Path) -> Tuple[Sequence[str], List[Dict[str, str]]]:
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames or [], list(reader)


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def qn(q: Sequence[float]) -> Tuple[float, float, float, float]:
    length = norm(q)
    if abs(length - 1.0) > 1.0e-5:
        raise RuntimeError("encountered a non-unit quaternion")
    return tuple(value / length for value in q)


def qc(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = qn(q)
    return -x, -y, -z, w


def qm(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return qn((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def rotate(q: Sequence[float], p: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z, w = qn(q)
    px, py, pz = p
    return (
        (1 - 2 * (y*y + z*z))*px + 2*(x*y - z*w)*py + 2*(x*z + y*w)*pz,
        2*(x*y + z*w)*px + (1 - 2*(x*x + z*z))*py + 2*(y*z - x*w)*pz,
        2*(x*z - y*w)*px + 2*(y*z + x*w)*py + (1 - 2*(x*x + y*y))*pz,
    )


def compose(a: Pose, b: Pose) -> Pose:
    rotated = rotate(a[1], b[0])
    return tuple(x + y for x, y in zip(a[0], rotated)), qm(a[1], b[1])


def inverse(a: Pose) -> Pose:
    rotation = qc(a[1])
    return tuple(-v for v in rotate(rotation, a[0])), rotation


def between(a: Pose, b: Pose) -> Pose:
    return compose(inverse(a), b)


def pose_error(measurement: Pose, prediction: Pose) -> Tuple[float, float]:
    error = between(measurement, prediction)
    angle = math.degrees(2 * math.acos(max(-1.0, min(1.0, abs(qn(error[1])[3])))))
    return norm(error[0]), angle


def pose(row: Dict[str, str], prefix: str) -> Pose:
    t = tuple(float(row[f"{prefix}_{axis}"]) for axis in ("tx", "ty", "tz"))
    q = tuple(float(row[f"{prefix}_{axis}"]) for axis in ("qx", "qy", "qz", "qw"))
    if not all(math.isfinite(v) for v in (*t, *q)):
        raise RuntimeError("non-finite pose")
    return t, qn(q)


def percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered)-1, math.ceil(fraction*len(ordered))-1)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--loops", type=Path, default=Path("Log/backend/loop_factors.csv"))
    parser.add_argument("--verification", type=Path, default=Path("Log/backend/loop_verification.csv"))
    parser.add_argument("--trajectory", type=Path, default=Path("Log/backend/optimized_trajectory.csv"))
    parser.add_argument("--keyframes", type=Path, default=Path("Log/backend/keyframes.csv"))
    parser.add_argument("--rtk-factors", type=Path,
                        default=Path("Log/backend/rtk_factors.csv"))
    parser.add_argument("--loop-additional-update-steps", type=int, default=2)
    parser.add_argument("--maximum-position-deformation", type=float, default=5.0)
    parser.add_argument("--maximum-angle-deformation-deg", type=float, default=10.0)
    parser.add_argument("--maximum-odometry-edge-translation-error", type=float, default=0.50)
    parser.add_argument("--maximum-odometry-edge-angle-error-deg", type=float, default=5.0)
    parser.add_argument(
        "--maximum-relative-cost-increase", type=float, default=0.01,
        help=("Allowed iSAM2 robust-cost fluctuation caused by incremental "
              "relinearization; the inserted loop residual must still decrease."),
    )
    args = parser.parse_args()

    loop_fields, loops = read_csv(args.loops)
    verification_fields, verification = read_csv(args.verification)
    trajectory_fields, trajectory = read_csv(args.trajectory)
    keyframe_fields, keyframes = read_csv(args.keyframes)
    required_loop = {
        "historical_id", "current_id", "robust_kernel", "robust_delta",
        "translation_sigma_m", "rotation_sigma_deg", "initial_cost", "final_cost",
        "residual_translation_before_m", "residual_rotation_before_deg",
        "residual_translation_after_m", "residual_rotation_after_deg",
        "maximum_pose_correction_m", "maximum_pose_correction_deg",
        "optimization_time_ms", "variables_relinearized", "variables_reeliminated",
        "measurement_tx", "measurement_ty", "measurement_tz", "measurement_qx",
        "measurement_qy", "measurement_qz", "measurement_qw",
    }
    required_verification = {
        "current_id", "candidate_id", "accepted", "measurement_tx", "measurement_ty",
        "measurement_tz", "measurement_qx", "measurement_qy", "measurement_qz", "measurement_qw",
    }
    required_trajectory = {
        "id", "timestamp", "raw_tx", "raw_ty", "raw_tz", "raw_qx", "raw_qy", "raw_qz", "raw_qw",
        "opt_tx", "opt_ty", "opt_tz", "opt_qx", "opt_qy", "opt_qz", "opt_qw",
        "position_delta_m", "angle_delta_deg",
    }
    for fields, required, name in (
        (loop_fields, required_loop, "loop-factor"),
        (verification_fields, required_verification, "verification"),
        (trajectory_fields, required_trajectory, "trajectory"),
    ):
        missing = sorted(required.difference(fields))
        if missing:
            raise RuntimeError(f"{name} CSV is missing columns: {missing}")
    accepted = {
        (int(row["current_id"]), int(row["candidate_id"])): row
        for row in verification if int(row["accepted"]) == 1
    }
    if len(loops) != len(accepted) or not loops:
        raise RuntimeError(f"loop factors/accepted decisions differ: {len(loops)} != {len(accepted)}")
    if len(trajectory) != len(keyframes):
        raise RuntimeError("optimized trajectory/keyframe count differs")

    raw_poses, optimized_poses = [], []
    max_position_delta = 0.0
    max_angle_delta = 0.0
    for index, (row, keyframe) in enumerate(zip(trajectory, keyframes)):
        if int(row["id"]) != index or int(keyframe["id"]) != index:
            raise RuntimeError("trajectory/keyframe IDs are not contiguous")
        raw, optimized = pose(row, "raw"), pose(row, "opt")
        keyframe_t = tuple(float(keyframe[a]) for a in ("tx", "ty", "tz"))
        keyframe_q = tuple(float(keyframe[a]) for a in ("qx", "qy", "qz", "qw"))
        input_error = pose_error((keyframe_t, keyframe_q), raw)
        if input_error[0] > 1.0e-9 or input_error[1] > 1.0e-5:
            raise RuntimeError(f"trajectory raw input differs at keyframe {index}")
        delta = pose_error(raw, optimized)
        if abs(delta[0] - float(row["position_delta_m"])) > 1.0e-8:
            raise RuntimeError(f"position delta is inconsistent at keyframe {index}")
        if abs(delta[1] - float(row["angle_delta_deg"])) > 1.0e-5:
            raise RuntimeError(f"angle delta is inconsistent at keyframe {index}")
        max_position_delta = max(max_position_delta, delta[0])
        max_angle_delta = max(max_angle_delta, delta[1])
        raw_poses.append(raw)
        optimized_poses.append(optimized)
    if max_position_delta > args.maximum_position_deformation:
        raise RuntimeError("loop graph position deformation exceeds anti-collapse bound")
    if max_angle_delta > args.maximum_angle_deformation_deg:
        raise RuntimeError("loop graph angle deformation exceeds anti-collapse bound")

    maximum_odom_translation_error = 0.0
    maximum_odom_angle_error = 0.0
    for index in range(1, len(trajectory)):
        raw_edge = between(raw_poses[index-1], raw_poses[index])
        optimized_edge = between(optimized_poses[index-1], optimized_poses[index])
        error = pose_error(raw_edge, optimized_edge)
        maximum_odom_translation_error = max(maximum_odom_translation_error, error[0])
        maximum_odom_angle_error = max(maximum_odom_angle_error, error[1])
    if maximum_odom_translation_error > args.maximum_odometry_edge_translation_error:
        raise RuntimeError("an optimized odometry edge indicates trajectory folding")
    if maximum_odom_angle_error > args.maximum_odometry_edge_angle_error_deg:
        raise RuntimeError("an optimized odometry edge has an excessive rotation change")

    initial_loop_errors, final_loop_errors, loop_times, chi2_values = [], [], [], []
    seen = set()
    for row in loops:
        historical, current = int(row["historical_id"]), int(row["current_id"])
        pair = (current, historical)
        if pair not in accepted or pair in seen or not 0 <= historical < current < len(trajectory):
            raise RuntimeError(f"invalid or duplicate loop factor {historical}<-{current}")
        seen.add(pair)
        if row["robust_kernel"] not in {"cauchy", "huber"} or float(row["robust_delta"]) <= 0:
            raise RuntimeError(f"loop factor {pair} has no valid robust kernel")
        measurement = pose(row, "measurement")
        verified = pose(accepted[pair], "measurement")
        measurement_error = pose_error(verified, measurement)
        if measurement_error[0] > 1.0e-8 or measurement_error[1] > 1.0e-5:
            raise RuntimeError(f"loop factor {pair} changed the verified measurement")
        initial_prediction = between(raw_poses[historical], raw_poses[current])
        final_prediction = between(optimized_poses[historical], optimized_poses[current])
        initial_error = pose_error(measurement, initial_prediction)
        final_error = pose_error(measurement, final_prediction)
        initial_loop_errors.append(initial_error[0])
        final_loop_errors.append(final_error[0])
        sigma_t = float(row["translation_sigma_m"])
        sigma_r = math.radians(float(row["rotation_sigma_deg"]))
        chi2_values.append((final_error[0]/sigma_t)**2 + (math.radians(final_error[1])/sigma_r)**2)
        before_logged = (float(row["residual_translation_before_m"]), float(row["residual_rotation_before_deg"]))
        after_logged = (float(row["residual_translation_after_m"]), float(row["residual_rotation_after_deg"]))
        if any(not math.isfinite(v) or v < 0 for v in (*before_logged, *after_logged)):
            raise RuntimeError(f"loop factor {pair} has invalid residual diagnostics")
        initial_cost, final_cost = float(row["initial_cost"]), float(row["final_cost"])
        if not math.isfinite(initial_cost) or not math.isfinite(final_cost):
            raise RuntimeError(f"loop update {pair} has a non-finite robust cost")
        # iSAM2's errorBefore/errorAfter are evaluated around successive local
        # linearization points.  With a robust kernel, reweighting can therefore
        # cause a small non-monotonic fluctuation even when the new factor is
        # fitted better.  Bound that fluctuation and independently require the
        # newly inserted loop's normalized residual to decrease.
        allowed_cost = initial_cost * (1.0 + args.maximum_relative_cost_increase) + 1e-8
        if final_cost > allowed_cost:
            raise RuntimeError(
                f"loop update {pair} increased total robust cost by more than "
                f"{100.0*args.maximum_relative_cost_increase:.2f}%"
            )
        before_chi2 = (
            (before_logged[0] / sigma_t) ** 2
            + (math.radians(before_logged[1]) / sigma_r) ** 2
        )
        after_chi2 = (
            (after_logged[0] / sigma_t) ** 2
            + (math.radians(after_logged[1]) / sigma_r) ** 2
        )
        if after_chi2 > before_chi2 + 1.0e-9:
            raise RuntimeError(f"loop update {pair} increased its factor residual")
        maximum_work = len(trajectory) * (1 + args.loop_additional_update_steps)
        if int(row["variables_relinearized"]) > maximum_work or int(row["variables_reeliminated"]) > maximum_work:
            raise RuntimeError(f"loop update {pair} has impossible iSAM2 work counters")
        elapsed = float(row["optimization_time_ms"])
        if not math.isfinite(elapsed) or elapsed < 0:
            raise RuntimeError(f"loop update {pair} has invalid runtime")
        loop_times.append(elapsed)
    if set(accepted) != seen:
        raise RuntimeError("some accepted verification decisions have no loop factor")
    if sum(final_loop_errors) > sum(initial_loop_errors) + 1.0e-6:
        raise RuntimeError("final aggregate loop translation residual did not decrease")

    print(
        f"loop pose graph: nodes={len(trajectory)}, odometry_factors={len(trajectory)-1}, "
        f"loop_factors={len(loops)}, robust=enabled"
    )
    print(
        "trajectory deformation: "
        f"max_position={max_position_delta:.4f}m, max_angle={max_angle_delta:.4f}deg; "
        f"max_odom_edge_error={maximum_odom_translation_error:.4f}m/"
        f"{maximum_odom_angle_error:.4f}deg"
    )
    print(
        "loop residual translation: "
        f"raw_median={statistics.median(initial_loop_errors):.4f}m, "
        f"optimized_median={statistics.median(final_loop_errors):.4f}m, "
        f"optimized_max={max(final_loop_errors):.4f}m"
    )
    print(
        "loop chi2/runtime: "
        f"chi2_median={statistics.median(chi2_values):.3f}, chi2_max={max(chi2_values):.3f}, "
        f"time_median={statistics.median(loop_times):.3f}ms, p95={percentile(loop_times,.95):.3f}ms"
    )
    rtk_count = 0
    if args.rtk_factors.is_file():
        with args.rtk_factors.open(newline="", encoding="utf-8") as stream:
            rtk_count = sum(1 for _ in csv.DictReader(stream))
    print(f"loop pose-graph validation passed; joint_rtk_factors={rtk_count}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError, IndexError) as error:
        print(f"loop pose-graph validation failed: {error}")
        raise SystemExit(1)
