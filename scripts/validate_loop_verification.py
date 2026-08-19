#!/usr/bin/env python3
"""Validate high-precision loop gates and neighboring-loop cycle consistency."""

import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


def read_csv(path: Path) -> Tuple[Sequence[str], List[Dict[str, str]]]:
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames or [], list(reader)


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def q_normalize(q: Sequence[float]) -> Tuple[float, float, float, float]:
    length = norm(q)
    if length <= 0.0:
        raise RuntimeError("zero quaternion")
    return tuple(value / length for value in q)


def q_conjugate(q: Sequence[float]) -> Tuple[float, float, float, float]:
    q = q_normalize(q)
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return q_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate(q: Sequence[float], point: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z, w = q_normalize(q)
    # Expanded q * [p, 0] * conjugate(q), without normalizing the point quaternion.
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    px, py, pz = point
    return (
        (1.0 - 2.0 * (yy + zz)) * px + 2.0 * (xy - wz) * py + 2.0 * (xz + wy) * pz,
        2.0 * (xy + wz) * px + (1.0 - 2.0 * (xx + zz)) * py + 2.0 * (yz - wx) * pz,
        2.0 * (xz - wy) * px + 2.0 * (yz + wx) * py + (1.0 - 2.0 * (xx + yy)) * pz,
    )


Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


def compose(a: Pose, b: Pose) -> Pose:
    rotated = rotate(a[1], b[0])
    return (
        tuple(x + y for x, y in zip(a[0], rotated)),
        q_multiply(a[1], b[1]),
    )


def inverse(pose: Pose) -> Pose:
    rotation = q_conjugate(pose[1])
    return tuple(-value for value in rotate(rotation, pose[0])), rotation


def between(a: Pose, b: Pose) -> Pose:
    return compose(inverse(a), b)


def pose_error(a: Pose, b: Pose) -> Tuple[float, float]:
    error = between(a, b)
    angle = math.degrees(
        2.0 * math.acos(max(-1.0, min(1.0, abs(q_normalize(error[1])[3]))))
    )
    return norm(error[0]), angle


def pose_from_row(row: Dict[str, str], prefix: str) -> Pose:
    translation = tuple(float(row[f"{prefix}_{axis}"]) for axis in ("tx", "ty", "tz"))
    rotation = tuple(float(row[f"{prefix}_{axis}"]) for axis in ("qx", "qy", "qz", "qw"))
    if not all(math.isfinite(value) for value in (*translation, *rotation)):
        raise RuntimeError("non-finite pose")
    return translation, q_normalize(rotation)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verification", type=Path, default=Path("Log/backend/loop_verification.csv"))
    parser.add_argument("--registrations", type=Path, default=Path("Log/backend/loop_registrations.csv"))
    parser.add_argument("--graph", type=Path, default=Path("Log/backend/pose_graph.csv"))
    parser.add_argument("--minimum-converged-levels", type=int, default=4)
    parser.add_argument("--minimum-probability", type=float, default=0.35)
    parser.add_argument("--maximum-fitness", type=float, default=0.09)
    parser.add_argument("--minimum-overlap", type=float, default=0.90)
    parser.add_argument("--maximum-overlap-rmse", type=float, default=0.32)
    parser.add_argument("--maximum-translation-correction", type=float, default=5.0)
    parser.add_argument("--maximum-rotation-correction", type=float, default=10.0)
    parser.add_argument("--current-id-window", type=int, default=40)
    parser.add_argument("--candidate-id-window", type=int, default=40)
    parser.add_argument("--maximum-neighbor-translation-error", type=float, default=0.50)
    parser.add_argument("--maximum-neighbor-rotation-error", type=float, default=2.0)
    parser.add_argument("--minimum-consistent-neighbors", type=int, default=1)
    args = parser.parse_args()

    verification_fields, decisions = read_csv(args.verification)
    registration_fields, registrations = read_csv(args.registrations)
    graph_fields, graph_rows = read_csv(args.graph)
    required_verification = {
        "current_id", "candidate_id", "accepted", "reject_reason",
        "consistent_neighbors", "best_neighbor_translation_error_m",
        "best_neighbor_rotation_error_deg", "measurement_tx", "measurement_ty",
        "measurement_tz", "measurement_qx", "measurement_qy", "measurement_qz",
        "measurement_qw",
    }
    required_registration = {
        "current_id", "candidate_id", "status", "converged", "converged_levels",
        "transformation_probability", "fitness_score_m2", "overlap",
        "overlap_rmse_m", "correction_translation_m", "correction_angle_deg",
        "final_tx", "final_ty", "final_tz", "final_qx", "final_qy", "final_qz", "final_qw",
    }
    required_graph = {"id", "opt_tx", "opt_ty", "opt_tz", "opt_qx", "opt_qy", "opt_qz", "opt_qw"}
    for fields, required, name in (
        (verification_fields, required_verification, "verification"),
        (registration_fields, required_registration, "registration"),
        (graph_fields, required_graph, "graph"),
    ):
        missing = sorted(required.difference(fields))
        if missing:
            raise RuntimeError(f"{name} CSV is missing columns: {missing}")
    if len(decisions) != len(registrations):
        raise RuntimeError(
            f"verification/registration row count differs: {len(decisions)} != {len(registrations)}"
        )

    graph = []
    for index, row in enumerate(graph_rows):
        if int(row["id"]) != index:
            raise RuntimeError("pose graph IDs are not contiguous")
        graph.append(pose_from_row(row, "opt"))

    registration_by_pair = {}
    individual_reasons = {}
    for row in registrations:
        pair = (int(row["current_id"]), int(row["candidate_id"]))
        if pair in registration_by_pair:
            raise RuntimeError(f"duplicate registration pair {pair}")
        registration_by_pair[pair] = row
        reason = "none"
        if row["status"] != "completed":
            reason = "registration_failed"
        elif int(row["converged"]) != 1:
            reason = "not_converged"
        elif int(row["converged_levels"]) < args.minimum_converged_levels:
            reason = "insufficient_converged_levels"
        elif float(row["transformation_probability"]) < args.minimum_probability:
            reason = "probability_too_low"
        elif float(row["fitness_score_m2"]) > args.maximum_fitness:
            reason = "fitness_too_high"
        elif float(row["overlap"]) < args.minimum_overlap:
            reason = "overlap_too_low"
        elif float(row["overlap_rmse_m"]) > args.maximum_overlap_rmse:
            reason = "overlap_rmse_too_high"
        elif float(row["correction_translation_m"]) > args.maximum_translation_correction:
            reason = "translation_sanity"
        elif float(row["correction_angle_deg"]) > args.maximum_rotation_correction:
            reason = "rotation_sanity"
        individual_reasons[pair] = reason

    valid_pairs = [pair for pair, reason in individual_reasons.items() if reason == "none"]
    consistency: Dict[Tuple[int, int], List[Tuple[float, float, Tuple[int, int]]]] = {
        pair: [] for pair in valid_pairs
    }
    for query_pair in valid_pairs:
        query_current, query_historical = query_pair
        query_z = pose_from_row(registration_by_pair[query_pair], "final")
        for reference_pair in valid_pairs:
            if query_pair == reference_pair:
                continue
            reference_current, reference_historical = reference_pair
            if not 0 < abs(query_current - reference_current) <= args.current_id_window:
                continue
            if abs(query_historical - reference_historical) > args.candidate_id_window:
                continue
            reference_z = pose_from_row(registration_by_pair[reference_pair], "final")
            predicted = compose(
                compose(
                    between(graph[query_historical], graph[reference_historical]),
                    reference_z,
                ),
                between(graph[reference_current], graph[query_current]),
            )
            translation_error, rotation_error = pose_error(predicted, query_z)
            if (
                translation_error <= args.maximum_neighbor_translation_error
                and rotation_error <= args.maximum_neighbor_rotation_error
            ):
                consistency[query_pair].append(
                    (translation_error, rotation_error, reference_pair)
                )

    expected_accepted = {
        pair
        for pair, neighbors in consistency.items()
        if len(neighbors) >= args.minimum_consistent_neighbors
    }
    decision_by_pair = {}
    rejection_counts = Counter()
    accepted_translation_errors = []
    accepted_rotation_errors = []
    for row in decisions:
        pair = (int(row["current_id"]), int(row["candidate_id"]))
        if pair not in registration_by_pair or pair in decision_by_pair:
            raise RuntimeError(f"unknown or duplicate verification pair {pair}")
        decision_by_pair[pair] = row
        measurement = pose_from_row(row, "measurement")
        final = pose_from_row(registration_by_pair[pair], "final")
        translation_error, rotation_error = pose_error(final, measurement)
        if translation_error > 1.0e-8 or rotation_error > 1.0e-5:
            raise RuntimeError(f"verification pair {pair} changed its NDT measurement")

        accepted = bool(int(row["accepted"]))
        expected = pair in expected_accepted
        if accepted != expected:
            raise RuntimeError(f"verification pair {pair} accepted={accepted}, expected={expected}")
        if accepted:
            if row["reject_reason"] != "none":
                raise RuntimeError(f"accepted pair {pair} has a reject reason")
            if int(row["consistent_neighbors"]) < args.minimum_consistent_neighbors:
                raise RuntimeError(f"accepted pair {pair} lacks neighbor evidence")
            best_t = float(row["best_neighbor_translation_error_m"])
            best_r = float(row["best_neighbor_rotation_error_deg"])
            if not (0.0 <= best_t <= args.maximum_neighbor_translation_error):
                raise RuntimeError(f"accepted pair {pair} has invalid consistency translation")
            if not (0.0 <= best_r <= args.maximum_neighbor_rotation_error):
                raise RuntimeError(f"accepted pair {pair} has invalid consistency rotation")
            accepted_translation_errors.append(best_t)
            accepted_rotation_errors.append(best_r)
        else:
            expected_reason = individual_reasons[pair]
            if expected_reason == "none":
                expected_reason = "no_consistent_neighbor"
            if row["reject_reason"] != expected_reason:
                raise RuntimeError(
                    f"pair {pair} reason={row['reject_reason']}, expected={expected_reason}"
                )
            rejection_counts[row["reject_reason"]] += 1

    if set(decision_by_pair) != set(registration_by_pair):
        raise RuntimeError("verification decisions do not cover every registration")
    if not expected_accepted:
        raise RuntimeError("no loop passed high-precision verification")

    print(
        f"loop verification: registrations={len(registrations)}, "
        f"individually_valid={len(valid_pairs)}, accepted={len(expected_accepted)}, "
        f"rejected={len(registrations) - len(expected_accepted)}"
    )
    print(f"reject reasons: {dict(sorted(rejection_counts.items()))}")
    print(
        "accepted neighbor consistency: "
        f"translation={min(accepted_translation_errors):.4f}..{max(accepted_translation_errors):.4f}m, "
        f"rotation={min(accepted_rotation_errors):.4f}..{max(accepted_rotation_errors):.4f}deg"
    )
    print("loop-verification validation passed; graph-factor checks are handled downstream")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError, IndexError) as error:
        print(f"loop-verification validation failed: {error}")
        raise SystemExit(1)
