#!/usr/bin/env python3
"""Validate multi-resolution NDT accounting and geometric result fields."""

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


SUMMARY_COLUMNS = {
    "current_id",
    "candidate_id",
    "status",
    "converged",
    "converged_levels",
    "configured_levels",
    "levels_executed",
    "raw_source_points",
    "raw_target_points",
    "final_source_points",
    "final_target_points",
    "transformation_probability",
    "fitness_score_m2",
    "overlap",
    "overlap_rmse_m",
    "initial_tx",
    "initial_ty",
    "initial_tz",
    "initial_qx",
    "initial_qy",
    "initial_qz",
    "initial_qw",
    "final_tx",
    "final_ty",
    "final_tz",
    "final_qx",
    "final_qy",
    "final_qz",
    "final_qw",
    "correction_translation_m",
    "correction_angle_deg",
    "registration_time_ms",
}
LEVEL_COLUMNS = {
    "current_id",
    "candidate_id",
    "level",
    "resolution_m",
    "voxel_leaf_size_m",
    "source_points",
    "target_points",
    "converged",
    "iterations",
    "transformation_probability",
    "fitness_score_m2",
    "elapsed_ms",
}
CANDIDATE_COLUMNS = {
    "current_id",
    "candidate_id",
    "initial_tx",
    "initial_ty",
    "initial_tz",
    "initial_qx",
    "initial_qy",
    "initial_qz",
    "initial_qw",
}
VALID_STATUSES = {
    "completed",
    "invalid_input",
    "insufficient_source_points",
    "insufficient_target_points",
    "ndt_exception",
    "nonfinite_result",
}


def read_csv(path: Path) -> Tuple[Sequence[str], List[Dict[str, str]]]:
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames or [], list(reader)


def require_columns(fields: Sequence[str], required: set, name: str) -> None:
    missing = sorted(required.difference(fields))
    if missing:
        raise RuntimeError(f"{name} CSV is missing columns: {missing}")


def finite_values(row: Dict[str, str], names: Sequence[str], context: str) -> List[float]:
    values = [float(row[name]) for name in names]
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"{context} contains a non-finite value")
    return values


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def quaternion_angle_deg(left: Sequence[float], right: Sequence[float]) -> float:
    left_norm = norm(left)
    right_norm = norm(right)
    if abs(left_norm - 1.0) > 1.0e-5 or abs(right_norm - 1.0) > 1.0e-5:
        raise RuntimeError("encountered a non-unit quaternion")
    dot = abs(sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)
    return ordered[index]


def close(actual: float, expected: float, tolerance: float, context: str) -> None:
    if abs(actual - expected) > tolerance:
        raise RuntimeError(
            f"{context}: got {actual:.12g}, expected {expected:.12g}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--registrations",
        type=Path,
        default=Path("Log/backend/loop_registrations.csv"),
    )
    parser.add_argument(
        "--levels",
        type=Path,
        default=Path("Log/backend/loop_registration_levels.csv"),
    )
    parser.add_argument(
        "--candidates",
        type=Path,
        default=Path("Log/backend/loop_candidates.csv"),
    )
    parser.add_argument(
        "--resolutions", type=float, nargs="+", default=[10.0, 5.0, 2.0, 1.0]
    )
    parser.add_argument("--voxel-leaf-ratio", type=float, default=0.25)
    parser.add_argument("--minimum-voxel-leaf", type=float, default=0.50)
    parser.add_argument("--maximum-iterations", type=int, default=40)
    parser.add_argument("--minimum-source-points", type=int, default=100)
    parser.add_argument("--minimum-target-points", type=int, default=300)
    parser.add_argument("--overlap-distance", type=float, default=1.0)
    parser.add_argument(
        "--allow-failed",
        action="store_true",
        help="report failed jobs without making validation fail",
    )
    args = parser.parse_args()

    if not args.resolutions or any(
        not math.isfinite(value) or value <= 0.0 for value in args.resolutions
    ):
        raise RuntimeError("resolutions must be finite and positive")
    if any(
        args.resolutions[index] <= args.resolutions[index + 1]
        for index in range(len(args.resolutions) - 1)
    ):
        raise RuntimeError("resolutions must be strictly coarse-to-fine")
    if args.maximum_iterations <= 0:
        raise RuntimeError("maximum iterations must be positive")

    summary_fields, summaries = read_csv(args.registrations)
    level_fields, levels = read_csv(args.levels)
    candidate_fields, candidates = read_csv(args.candidates)
    require_columns(summary_fields, SUMMARY_COLUMNS, "registration")
    require_columns(level_fields, LEVEL_COLUMNS, "registration-level")
    require_columns(candidate_fields, CANDIDATE_COLUMNS, "candidate")
    if not candidates:
        raise RuntimeError("candidate CSV is empty")
    if len(summaries) != len(candidates):
        raise RuntimeError(
            "registration/candidate row count differs: "
            f"{len(summaries)} != {len(candidates)}; the worker may not have drained"
        )

    levels_by_pair: Dict[Tuple[int, int], List[Dict[str, str]]] = defaultdict(list)
    for row_index, row in enumerate(levels):
        pair = (int(row["current_id"]), int(row["candidate_id"]))
        if int(row["level"]) != len(levels_by_pair[pair]):
            raise RuntimeError(f"level order is not contiguous for pair {pair}")
        levels_by_pair[pair].append(row)

    completed_rows = []
    converged_rows = []
    failed_statuses: Dict[str, int] = defaultdict(int)
    maximum_initial_position_error = 0.0
    maximum_initial_angle_error = 0.0

    for row_index, (summary, candidate) in enumerate(zip(summaries, candidates)):
        pair = (int(summary["current_id"]), int(summary["candidate_id"]))
        expected_pair = (int(candidate["current_id"]), int(candidate["candidate_id"]))
        if pair != expected_pair:
            raise RuntimeError(
                f"registration row {row_index} pair {pair}, expected {expected_pair}"
            )
        status = summary["status"]
        if status not in VALID_STATUSES:
            raise RuntimeError(f"registration pair {pair} has unknown status {status!r}")
        configured_levels = int(summary["configured_levels"])
        if configured_levels != len(args.resolutions):
            raise RuntimeError(f"registration pair {pair} configured level count differs")
        executed = int(summary["levels_executed"])
        pair_levels = levels_by_pair.get(pair, [])
        if executed != len(pair_levels):
            raise RuntimeError(f"registration pair {pair} level accounting differs")

        summary_initial_t = finite_values(
            summary, ["initial_tx", "initial_ty", "initial_tz"], f"pair {pair}"
        )
        summary_initial_q = finite_values(
            summary,
            ["initial_qx", "initial_qy", "initial_qz", "initial_qw"],
            f"pair {pair}",
        )
        candidate_initial_t = finite_values(
            candidate, ["initial_tx", "initial_ty", "initial_tz"], f"pair {pair}"
        )
        candidate_initial_q = finite_values(
            candidate,
            ["initial_qx", "initial_qy", "initial_qz", "initial_qw"],
            f"pair {pair}",
        )
        initial_position_error = norm(
            [a - b for a, b in zip(summary_initial_t, candidate_initial_t)]
        )
        initial_angle_error = quaternion_angle_deg(
            summary_initial_q, candidate_initial_q
        )
        maximum_initial_position_error = max(
            maximum_initial_position_error, initial_position_error
        )
        maximum_initial_angle_error = max(maximum_initial_angle_error, initial_angle_error)
        if initial_position_error > 1.0e-8 or initial_angle_error > 1.0e-5:
            raise RuntimeError(f"registration pair {pair} changed its candidate initial pose")

        level_converged = 0
        for level_index, level in enumerate(pair_levels):
            resolution = float(level["resolution_m"])
            expected_resolution = args.resolutions[level_index]
            close(resolution, expected_resolution, 1.0e-12,
                  f"pair {pair} level {level_index} resolution")
            expected_leaf = max(
                args.minimum_voxel_leaf, expected_resolution * args.voxel_leaf_ratio
            )
            close(float(level["voxel_leaf_size_m"]), expected_leaf, 1.0e-12,
                  f"pair {pair} level {level_index} voxel leaf")
            source_points = int(level["source_points"])
            target_points = int(level["target_points"])
            iterations = int(level["iterations"])
            if source_points < 0 or target_points < 0:
                raise RuntimeError(f"pair {pair} has a negative point count")
            if iterations < 0 or iterations > args.maximum_iterations + 1:
                raise RuntimeError(f"pair {pair} has an impossible iteration count")
            probability, fitness, elapsed = finite_values(
                level,
                ["transformation_probability", "fitness_score_m2", "elapsed_ms"],
                f"pair {pair} level {level_index}",
            )
            if fitness < 0.0 or elapsed < 0.0:
                raise RuntimeError(f"pair {pair} has negative NDT metrics")
            level_converged += int(level["converged"])

        if int(summary["converged_levels"]) != level_converged:
            raise RuntimeError(f"registration pair {pair} converged-level count differs")
        registration_time = float(summary["registration_time_ms"])
        if not math.isfinite(registration_time) or registration_time < 0.0:
            raise RuntimeError(f"registration pair {pair} has invalid runtime")

        if status != "completed":
            failed_statuses[status] += 1
            continue

        if executed != len(args.resolutions):
            raise RuntimeError(f"completed registration pair {pair} skipped NDT levels")
        raw_source = int(summary["raw_source_points"])
        raw_target = int(summary["raw_target_points"])
        final_source = int(summary["final_source_points"])
        final_target = int(summary["final_target_points"])
        if raw_source < args.minimum_source_points or final_source < args.minimum_source_points:
            raise RuntimeError(f"registration pair {pair} source cloud is too small")
        if raw_target < args.minimum_target_points or final_target < args.minimum_target_points:
            raise RuntimeError(f"registration pair {pair} target cloud is too small")

        probability, fitness, overlap, rmse = finite_values(
            summary,
            [
                "transformation_probability",
                "fitness_score_m2",
                "overlap",
                "overlap_rmse_m",
            ],
            f"registration pair {pair}",
        )
        if fitness < 0.0 or not 0.0 <= overlap <= 1.0:
            raise RuntimeError(f"registration pair {pair} has invalid quality metrics")
        if rmse < 0.0 or rmse > args.overlap_distance + 1.0e-6:
            raise RuntimeError(f"registration pair {pair} overlap RMSE is invalid")
        last_level = pair_levels[-1]
        close(probability, float(last_level["transformation_probability"]), 1.0e-12,
              f"pair {pair} final probability")
        close(fitness, float(last_level["fitness_score_m2"]), 1.0e-12,
              f"pair {pair} final fitness")
        if int(summary["converged"]) != int(last_level["converged"]):
            raise RuntimeError(f"registration pair {pair} final convergence differs")
        if final_source != int(last_level["source_points"]) or final_target != int(
            last_level["target_points"]
        ):
            raise RuntimeError(f"registration pair {pair} final point counts differ")

        final_t = finite_values(
            summary, ["final_tx", "final_ty", "final_tz"], f"pair {pair}"
        )
        final_q = finite_values(
            summary,
            ["final_qx", "final_qy", "final_qz", "final_qw"],
            f"pair {pair}",
        )
        correction_translation = norm(
            [a - b for a, b in zip(final_t, summary_initial_t)]
        )
        correction_angle = quaternion_angle_deg(summary_initial_q, final_q)
        close(
            float(summary["correction_translation_m"]),
            correction_translation,
            1.0e-7,
            f"pair {pair} correction translation",
        )
        close(
            float(summary["correction_angle_deg"]),
            correction_angle,
            1.0e-5,
            f"pair {pair} correction angle",
        )
        completed_rows.append(summary)
        if int(summary["converged"]):
            converged_rows.append(summary)

    extra_pairs = set(levels_by_pair).difference(
        (int(row["current_id"]), int(row["candidate_id"])) for row in summaries
    )
    if extra_pairs:
        raise RuntimeError(f"level CSV has pairs absent from summaries: {sorted(extra_pairs)}")
    if failed_statuses and not args.allow_failed:
        raise RuntimeError(f"registration jobs failed: {dict(failed_statuses)}")
    if not completed_rows:
        raise RuntimeError("no registration job completed")
    if not converged_rows:
        raise RuntimeError("no final NDT level converged")

    runtimes = [float(row["registration_time_ms"]) for row in completed_rows]
    overlaps = [float(row["overlap"]) for row in completed_rows]
    fitnesses = [float(row["fitness_score_m2"]) for row in completed_rows]
    corrections = [float(row["correction_translation_m"]) for row in completed_rows]
    print(
        f"loop registration: candidates={len(candidates)}, completed={len(completed_rows)}, "
        f"final_converged={len(converged_rows)}, failures={sum(failed_statuses.values())}"
    )
    print(
        "NDT runtime: "
        f"median={statistics.median(runtimes):.1f}ms, "
        f"p95={percentile(runtimes, 0.95):.1f}ms, max={max(runtimes):.1f}ms"
    )
    print(
        "raw quality ranges: "
        f"overlap={min(overlaps):.3f}..{max(overlaps):.3f}, "
        f"fitness={min(fitnesses):.4g}..{max(fitnesses):.4g}m^2, "
        f"translation_correction={min(corrections):.3f}..{max(corrections):.3f}m"
    )
    print(
        "candidate/registration initial equality: "
        f"max_position={maximum_initial_position_error:.9g}m, "
        f"max_angle={maximum_initial_angle_error:.9g}deg"
    )
    print(
        "loop-registration structural validation passed; quality thresholds are "
        "intentionally deferred to Loop Verification"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError, IndexError) as error:
        print(f"loop-registration validation failed: {error}")
        raise SystemExit(1)
