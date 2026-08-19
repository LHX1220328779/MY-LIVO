#!/usr/bin/env python3
"""Validate loop-candidate gating, ranking, transforms, and CSV accounting."""

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


DETECTION_COLUMNS = {
    "current_id",
    "current_timestamp",
    "checked",
    "history_keyframes",
    "eligible_history",
    "nearby_history",
    "candidates",
}
CANDIDATE_COLUMNS = {
    "current_id",
    "candidate_id",
    "current_timestamp",
    "candidate_timestamp",
    "id_separation",
    "time_separation_sec",
    "planar_distance_m",
    "height_difference_m",
    "translation_distance_m",
    "initial_tx",
    "initial_ty",
    "initial_tz",
    "initial_qx",
    "initial_qy",
    "initial_qz",
    "initial_qw",
}
GRAPH_COLUMNS = {
    "id",
    "timestamp",
    "opt_tx",
    "opt_ty",
    "opt_tz",
    "opt_qx",
    "opt_qy",
    "opt_qz",
    "opt_qw",
}


def read_csv(path: Path) -> Tuple[Sequence[str], List[Dict[str, str]]]:
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames or [], list(reader)


def require_columns(fields: Sequence[str], required: set, description: str) -> None:
    missing = sorted(required.difference(fields))
    if missing:
        raise RuntimeError(f"{description} CSV is missing columns: {missing}")


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def quaternion_conjugate(q: Sequence[float]) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate(q: Sequence[float], point: Sequence[float]) -> Tuple[float, float, float]:
    vector_q = (point[0], point[1], point[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(q, vector_q), quaternion_conjugate(q)
    )
    return rotated[:3]


def quaternion_angle_deg(left: Sequence[float], right: Sequence[float]) -> float:
    left_norm = norm(left)
    right_norm = norm(right)
    if abs(left_norm - 1.0) > 1.0e-5 or abs(right_norm - 1.0) > 1.0e-5:
        raise RuntimeError("encountered a non-unit quaternion")
    dot = abs(sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def close(actual: float, expected: float, tolerance: float, description: str) -> None:
    if not math.isfinite(actual) or abs(actual - expected) > tolerance:
        raise RuntimeError(
            f"{description}: got {actual:.12g}, expected {expected:.12g}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--detections",
        type=Path,
        default=Path("Log/backend/loop_detection.csv"),
    )
    parser.add_argument(
        "--candidates",
        type=Path,
        default=Path("Log/backend/loop_candidates.csv"),
    )
    parser.add_argument(
        "--graph", type=Path, default=Path("Log/backend/pose_graph.csv")
    )
    parser.add_argument(
        "--loop-factors", type=Path, default=Path("Log/backend/loop_factors.csv"),
        help="when non-empty, validate candidate-time pose snapshots rather than "
             "incorrectly comparing them with the final optimized graph",
    )
    parser.add_argument("--check-interval", type=int, default=20)
    parser.add_argument("--minimum-id-separation", type=int, default=50)
    parser.add_argument("--minimum-time-separation", type=float, default=30.0)
    parser.add_argument("--maximum-planar-distance", type=float, default=20.0)
    parser.add_argument("--maximum-height-difference", type=float, default=5.0)
    parser.add_argument("--candidate-id-separation", type=int, default=20)
    parser.add_argument("--maximum-candidates", type=int, default=3)
    args = parser.parse_args()

    if args.check_interval <= 0 or args.minimum_id_separation <= 0:
        raise RuntimeError("keyframe interval/separation arguments must be positive")
    if args.minimum_time_separation < 0.0:
        raise RuntimeError("minimum time separation must be non-negative")
    if args.maximum_planar_distance <= 0.0:
        raise RuntimeError("maximum planar distance must be positive")
    if args.maximum_height_difference < 0.0:
        raise RuntimeError("maximum height difference must be non-negative")
    if args.candidate_id_separation <= 0 or args.maximum_candidates <= 0:
        raise RuntimeError("candidate separation/count arguments must be positive")

    detection_fields, detection_rows = read_csv(args.detections)
    candidate_fields, candidate_rows = read_csv(args.candidates)
    graph_fields, graph_rows = read_csv(args.graph)
    require_columns(detection_fields, DETECTION_COLUMNS, "loop detection")
    require_columns(candidate_fields, CANDIDATE_COLUMNS, "loop candidate")
    require_columns(graph_fields, GRAPH_COLUMNS, "pose graph")
    graph_is_dynamic = False
    if args.loop_factors.is_file():
        with args.loop_factors.open(newline="", encoding="utf-8") as stream:
            graph_is_dynamic = any(True for _ in csv.DictReader(stream))
    if len(detection_rows) != len(graph_rows):
        raise RuntimeError(
            "loop-detection/pose-graph row count differs: "
            f"{len(detection_rows)} != {len(graph_rows)}"
        )
    if len(graph_rows) < 2:
        raise RuntimeError("at least two keyframes are required")

    poses = []
    for index, row in enumerate(graph_rows):
        if int(row["id"]) != index:
            raise RuntimeError(f"pose graph expected id {index}, got {row['id']}")
        timestamp = float(row["timestamp"])
        translation = tuple(float(row[name]) for name in ("opt_tx", "opt_ty", "opt_tz"))
        quaternion = tuple(
            float(row[name]) for name in ("opt_qx", "opt_qy", "opt_qz", "opt_qw")
        )
        if not all(math.isfinite(value) for value in (timestamp, *translation, *quaternion)):
            raise RuntimeError(f"pose graph row {index} contains a non-finite value")
        if abs(norm(quaternion) - 1.0) > 1.0e-5:
            raise RuntimeError(f"pose graph row {index} has a non-unit quaternion")
        poses.append((timestamp, translation, quaternion))

    candidates_by_current: Dict[int, List[Dict[str, str]]] = defaultdict(list)
    pairs = set()
    for row_index, row in enumerate(candidate_rows):
        current_id = int(row["current_id"])
        candidate_id = int(row["candidate_id"])
        if not (0 <= candidate_id < current_id < len(graph_rows)):
            raise RuntimeError(
                f"candidate row {row_index} has invalid pair {candidate_id}->{current_id}"
            )
        pair = (current_id, candidate_id)
        if pair in pairs:
            raise RuntimeError(f"duplicate loop candidate pair {pair}")
        pairs.add(pair)
        candidates_by_current[current_id].append(row)

    checked_count = 0
    eligible_total = 0
    nearby_total = 0
    maximum_initial_position_error = 0.0
    maximum_initial_angle_error = 0.0
    has_checked = False
    last_checked_id = 0

    for current_id, (summary, pose) in enumerate(zip(detection_rows, poses)):
        timestamp, current_t, current_q = pose
        if int(summary["current_id"]) != current_id:
            raise RuntimeError(
                f"loop detection expected id {current_id}, got {summary['current_id']}"
            )
        close(float(summary["current_timestamp"]), timestamp, 1.0e-9,
              f"row {current_id} timestamp")
        if int(summary["history_keyframes"]) != current_id:
            raise RuntimeError(f"row {current_id} history count must equal its ID")

        expected_checked = current_id >= args.minimum_id_separation and (
            not has_checked or current_id - last_checked_id >= args.check_interval
        )
        checked = bool(int(summary["checked"]))
        if checked != expected_checked:
            raise RuntimeError(
                f"row {current_id} checked={int(checked)}, expected {int(expected_checked)}"
            )
        if checked:
            has_checked = True
            last_checked_id = current_id
            checked_count += 1

        eligible = []
        nearby = []
        if checked:
            for historical_id in range(current_id):
                historical_time, historical_t, _ = poses[historical_id]
                if current_id - historical_id < args.minimum_id_separation:
                    continue
                if timestamp - historical_time < args.minimum_time_separation:
                    continue
                dx = current_t[0] - historical_t[0]
                dy = current_t[1] - historical_t[1]
                dz = current_t[2] - historical_t[2]
                eligible.append(historical_id)
                planar = math.hypot(dx, dy)
                height = abs(dz)
                if (
                    planar <= args.maximum_planar_distance
                    and height <= args.maximum_height_difference
                ):
                    nearby.append((planar, historical_id, height, norm((dx, dy, dz))))

        if int(summary["eligible_history"]) != len(eligible):
            raise RuntimeError(f"row {current_id} eligible-history count is inconsistent")
        logged = candidates_by_current.get(current_id, [])
        if int(summary["candidates"]) != len(logged):
            raise RuntimeError(f"row {current_id} candidate count is inconsistent")

        # Once the first loop factor is inserted, historical optimized poses
        # continue to move.  pose_graph.csv stores the *final* estimates, while
        # each candidate row intentionally stores the transform and distances
        # at detection time.  Recomputing an old search against the final graph
        # is therefore invalid.  In this mode validate every invariant that is
        # independent of later graph updates and the candidate snapshot's own
        # rigid-transform consistency.
        if graph_is_dynamic:
            reported_nearby = int(summary["nearby_history"])
            if not 0 <= reported_nearby <= len(eligible):
                raise RuntimeError(f"row {current_id} nearby-history count is impossible")
            if len(logged) > min(args.maximum_candidates, reported_nearby):
                raise RuntimeError(f"row {current_id} selected too many candidates")
            eligible_total += len(eligible)
            nearby_total += reported_nearby
            previous_planar = -math.inf
            selected_ids = []
            for row in logged:
                candidate_id = int(row["candidate_id"])
                candidate_time = poses[candidate_id][0]
                close(float(row["current_timestamp"]), timestamp, 1.0e-9,
                      f"candidate {candidate_id}->{current_id} current time")
                close(float(row["candidate_timestamp"]), candidate_time, 1.0e-9,
                      f"candidate {candidate_id}->{current_id} historical time")
                if int(row["id_separation"]) != current_id - candidate_id:
                    raise RuntimeError(f"candidate {candidate_id}->{current_id} ID gap is wrong")
                close(float(row["time_separation_sec"]), timestamp - candidate_time,
                      1.0e-9, f"candidate {candidate_id}->{current_id} time gap")
                planar = float(row["planar_distance_m"])
                height = float(row["height_difference_m"])
                distance = float(row["translation_distance_m"])
                logged_initial_t = tuple(
                    float(row[name]) for name in ("initial_tx", "initial_ty", "initial_tz")
                )
                logged_initial_q = tuple(
                    float(row[name])
                    for name in ("initial_qx", "initial_qy", "initial_qz", "initial_qw")
                )
                if not all(math.isfinite(value) for value in (
                    planar, height, distance, *logged_initial_t, *logged_initial_q
                )):
                    raise RuntimeError(f"candidate {candidate_id}->{current_id} is non-finite")
                if planar < 0 or planar > args.maximum_planar_distance + 1.0e-9:
                    raise RuntimeError(f"candidate {candidate_id}->{current_id} violates planar gate")
                if height < 0 or height > args.maximum_height_difference + 1.0e-9:
                    raise RuntimeError(f"candidate {candidate_id}->{current_id} violates height gate")
                close(math.hypot(planar, height), distance, 1.0e-8,
                      f"candidate {candidate_id}->{current_id} distance components")
                close(norm(logged_initial_t), distance, 1.0e-8,
                      f"candidate {candidate_id}->{current_id} transform distance")
                close(norm(logged_initial_q), 1.0, 1.0e-5,
                      f"candidate {candidate_id}->{current_id} quaternion norm")
                if planar + 1.0e-12 < previous_planar:
                    raise RuntimeError(f"row {current_id} candidates are not distance-ranked")
                if any(abs(candidate_id - other) < args.candidate_id_separation
                       for other in selected_ids):
                    raise RuntimeError(f"row {current_id} candidates violate ID diversity")
                previous_planar = planar
                selected_ids.append(candidate_id)
            continue

        if int(summary["nearby_history"]) != len(nearby):
            raise RuntimeError(f"row {current_id} nearby-history count is inconsistent")
        eligible_total += len(eligible)
        nearby_total += len(nearby)

        nearby.sort(key=lambda item: (item[0], item[1]))
        selected = []
        for item in nearby:
            if any(
                abs(item[1] - previous[1]) < args.candidate_id_separation
                for previous in selected
            ):
                continue
            selected.append(item)
            if len(selected) >= args.maximum_candidates:
                break

        expected_ids = [item[1] for item in selected]
        logged_ids = [int(row["candidate_id"]) for row in logged]
        if logged_ids != expected_ids:
            raise RuntimeError(
                f"row {current_id} candidates {logged_ids}, expected {expected_ids}"
            )

        for row, expected in zip(logged, selected):
            planar, candidate_id, height, distance = expected
            candidate_time, candidate_t, candidate_q = poses[candidate_id]
            close(float(row["current_timestamp"]), timestamp, 1.0e-9,
                  f"candidate {candidate_id}->{current_id} current time")
            close(float(row["candidate_timestamp"]), candidate_time, 1.0e-9,
                  f"candidate {candidate_id}->{current_id} historical time")
            if int(row["id_separation"]) != current_id - candidate_id:
                raise RuntimeError(f"candidate {candidate_id}->{current_id} ID gap is wrong")
            close(float(row["time_separation_sec"]), timestamp - candidate_time,
                  1.0e-9, f"candidate {candidate_id}->{current_id} time gap")
            close(float(row["planar_distance_m"]), planar, 1.0e-9,
                  f"candidate {candidate_id}->{current_id} planar distance")
            close(float(row["height_difference_m"]), height, 1.0e-9,
                  f"candidate {candidate_id}->{current_id} height difference")
            close(float(row["translation_distance_m"]), distance, 1.0e-9,
                  f"candidate {candidate_id}->{current_id} 3D distance")

            inverse_candidate_q = quaternion_conjugate(candidate_q)
            delta_t = tuple(a - b for a, b in zip(current_t, candidate_t))
            expected_initial_t = rotate(inverse_candidate_q, delta_t)
            expected_initial_q = quaternion_multiply(inverse_candidate_q, current_q)
            logged_initial_t = tuple(
                float(row[name]) for name in ("initial_tx", "initial_ty", "initial_tz")
            )
            logged_initial_q = tuple(
                float(row[name])
                for name in ("initial_qx", "initial_qy", "initial_qz", "initial_qw")
            )
            if not all(
                math.isfinite(value) for value in (*logged_initial_t, *logged_initial_q)
            ):
                raise RuntimeError(
                    f"candidate {candidate_id}->{current_id} has a non-finite transform"
                )
            position_error = norm(
                tuple(a - b for a, b in zip(logged_initial_t, expected_initial_t))
            )
            angle_error = quaternion_angle_deg(logged_initial_q, expected_initial_q)
            maximum_initial_position_error = max(
                maximum_initial_position_error, position_error
            )
            maximum_initial_angle_error = max(maximum_initial_angle_error, angle_error)
            if position_error > 1.0e-8 or angle_error > 1.0e-5:
                raise RuntimeError(
                    f"candidate {candidate_id}->{current_id} initial transform is inconsistent"
                )

    if checked_count == 0:
        raise RuntimeError("no loop-candidate checks were executed")
    if not candidate_rows:
        raise RuntimeError("no loop candidates were produced")

    distances = [float(row["planar_distance_m"]) for row in candidate_rows]
    print(
        f"loop detector: keyframes={len(graph_rows)}, checks={checked_count}, "
        f"eligible_history={eligible_total}, nearby_history={nearby_total}, "
        f"candidates={len(candidate_rows)}"
    )
    print(
        "candidate distance: "
        f"min={min(distances):.3f}m, median={statistics.median(distances):.3f}m, "
        f"max={max(distances):.3f}m"
    )
    print(
        ("candidate snapshot self-consistency: dynamic_graph=1"
         if graph_is_dynamic else
         "initial transform equality: "
         f"max_position={maximum_initial_position_error:.9g}m, "
         f"max_angle={maximum_initial_angle_error:.9g}deg")
    )
    print("loop-candidate validation passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError) as error:
        print(f"loop-candidate validation failed: {error}")
        raise SystemExit(1)
