#!/usr/bin/env python3
"""Validate status-gated, keyframe-driven robust RTK position fusion."""

import argparse
import csv
import math
import statistics
from pathlib import Path


def read(path):
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames or [], list(reader)


def vector(row, names):
    return tuple(float(row[name]) for name in names)


def norm(values):
    return math.sqrt(sum(value*value for value in values))


def distance(a, b):
    return norm(tuple(x-y for x, y in zip(a, b)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status", type=Path,
                        default=Path("Log/backend/rtk_status.csv"))
    parser.add_argument("--solutions", type=Path,
                        default=Path("Log/backend/rtk_solutions.csv"))
    parser.add_argument("--queries", type=Path,
                        default=Path("Log/backend/rtk_queries.csv"))
    parser.add_argument("--decisions", type=Path,
                        default=Path("Log/backend/rtk_decisions.csv"))
    parser.add_argument("--factors", type=Path,
                        default=Path("Log/backend/rtk_factors.csv"))
    parser.add_argument("--keyframes", type=Path,
                        default=Path("Log/backend/keyframes.csv"))
    parser.add_argument("--trajectory", type=Path,
                        default=Path("Log/backend/optimized_trajectory.csv"))
    parser.add_argument("--required-mode", type=int, default=4)
    parser.add_argument("--minimum-factor-dt", type=float, default=2.0)
    parser.add_argument("--maximum-factor-dt", type=float, default=10.0)
    parser.add_argument("--minimum-factor-distance", type=float, default=5.0)
    parser.add_argument("--maximum-factor-solution-ratio", type=float,
                        default=0.02)
    parser.add_argument("--maximum-trajectory-deformation", type=float,
                        default=5.0)
    args = parser.parse_args()

    status_fields, statuses = read(args.status)
    solution_fields, solutions = read(args.solutions)
    query_fields, queries = read(args.queries)
    decision_fields, decisions = read(args.decisions)
    factor_fields, factors = read(args.factors)
    _, keyframes = read(args.keyframes)
    _, trajectory = read(args.trajectory)
    requirements = [
        (status_fields, {"timestamp", "parsed", "ins_pos_mode"}, "status"),
        (solution_fields, {"timestamp", "x", "y", "z", "sigma_x", "sigma_y", "sigma_z"}, "solution"),
        (query_fields, {"timestamp", "accepted", "reason", "lower_timestamp", "upper_timestamp", "alpha", "x", "y", "z"}, "query"),
        (decision_fields, {"keyframe_id", "timestamp", "decision", "factor_added", "measurement_x", "measurement_y", "measurement_z", "innovation_chi2"}, "decision"),
        (factor_fields, {"keyframe_id", "robust_kernel", "robust_delta", "sigma_x", "sigma_y", "sigma_z", "measurement_x", "measurement_y", "measurement_z", "innovation_before_x", "innovation_before_y", "innovation_before_z", "innovation_after_x", "innovation_after_y", "innovation_after_z", "optimization_time_ms"}, "factor"),
    ]
    for fields, required, name in requirements:
        if not required.issubset(fields):
            raise RuntimeError(f"{name} CSV is missing {sorted(required-set(fields))}")
    if not statuses or not solutions or not keyframes or not factors:
        raise RuntimeError("RTK fusion logs contain no usable data")
    if len(queries) != len(keyframes) or len(decisions) != len(keyframes):
        raise RuntimeError("RTK query/decision cadence is not one per keyframe")
    if len(trajectory) != len(keyframes):
        raise RuntimeError("final trajectory/keyframe count differs")

    status_timeline = []
    parsed_statuses = 0
    mode4_statuses = 0
    for row in statuses:
        timestamp = float(row["timestamp"])
        parsed = int(row["parsed"]) == 1
        if not math.isfinite(timestamp):
            raise RuntimeError("non-finite status timestamp")
        mode = int(row["ins_pos_mode"]) if parsed else None
        parsed_statuses += parsed
        mode4_statuses += mode == args.required_mode
        status_timeline.append((timestamp, mode))
    if any(status_timeline[i][0] < status_timeline[i-1][0]
           for i in range(1, len(status_timeline))):
        raise RuntimeError("status timestamps moved backwards")

    solution_times = [float(row["timestamp"]) for row in solutions]
    if any(solution_times[i] <= solution_times[i-1]
           for i in range(1, len(solution_times))):
        raise RuntimeError("RTK solution timestamps are not strictly increasing")
    for row in solutions:
        if any(not math.isfinite(float(row[name])) for name in
               ("x", "y", "z", "sigma_x", "sigma_y", "sigma_z")):
            raise RuntimeError("RTK solution contains non-finite data")
        if any(float(row[name]) <= 0 for name in ("sigma_x", "sigma_y", "sigma_z")):
            raise RuntimeError("RTK covariance floor was not applied")

    factors_by_id = {}
    accepted_decisions = []
    for index, (query, decision, keyframe) in enumerate(
            zip(queries, decisions, keyframes)):
        if int(decision["keyframe_id"]) != index or int(keyframe["id"]) != index:
            raise RuntimeError("RTK decision/keyframe IDs are not contiguous")
        if abs(float(query["timestamp"])-float(keyframe["timestamp"])) > 1e-9 or \
           abs(float(decision["timestamp"])-float(keyframe["timestamp"])) > 1e-9:
            raise RuntimeError(f"RTK query timestamp differs at keyframe {index}")
        factor_added = int(decision["factor_added"]) == 1
        if factor_added != (decision["decision"] == "accepted"):
            raise RuntimeError(f"RTK decision/factor flag differs at keyframe {index}")
        if factor_added:
            if int(query["accepted"]) != 1 or query["reason"] != "accepted":
                raise RuntimeError(f"factor {index} used an invalid RTK bracket/status")
            accepted_decisions.append(decision)
        elif decision["decision"] == "accepted":
            raise RuntimeError(f"accepted RTK decision {index} has no factor")
    for row in factors:
        keyframe_id = int(row["keyframe_id"])
        if keyframe_id in factors_by_id or not 0 <= keyframe_id < len(keyframes):
            raise RuntimeError("duplicate or invalid RTK factor keyframe")
        factors_by_id[keyframe_id] = row
        if row["robust_kernel"] not in {"cauchy", "huber"} or float(row["robust_delta"]) <= 0:
            raise RuntimeError(f"RTK factor {keyframe_id} is not robust")
        if any(float(row[name]) <= 0 for name in ("sigma_x", "sigma_y", "sigma_z")):
            raise RuntimeError(f"RTK factor {keyframe_id} has invalid covariance")
        decision = decisions[keyframe_id]
        measurement = vector(row, ("measurement_x", "measurement_y", "measurement_z"))
        decision_measurement = vector(decision, ("measurement_x", "measurement_y", "measurement_z"))
        if distance(measurement, decision_measurement) > 1e-9:
            raise RuntimeError(f"RTK factor {keyframe_id} changed its observation")
    if set(factors_by_id) != {int(row["keyframe_id"]) for row in accepted_decisions}:
        raise RuntimeError("accepted RTK decisions and graph factors differ")
    if len(factors) > len(keyframes) or len(factors)/len(solutions) > args.maximum_factor_solution_ratio:
        raise RuntimeError("RTK factors scale with high-rate solution messages")

    ordered_factors = sorted(factors, key=lambda row: int(row["keyframe_id"]))
    for previous, current in zip(ordered_factors, ordered_factors[1:]):
        previous_id, current_id = int(previous["keyframe_id"]), int(current["keyframe_id"])
        dt = float(keyframes[current_id]["timestamp"])-float(keyframes[previous_id]["timestamp"])
        movement = distance(
            vector(previous, ("measurement_x", "measurement_y", "measurement_z")),
            vector(current, ("measurement_x", "measurement_y", "measurement_z")))
        if dt + 1e-9 < args.minimum_factor_dt:
            raise RuntimeError("RTK factor minimum time spacing was violated")
        if dt < args.maximum_factor_dt - 1e-9 and movement + 1e-9 < args.minimum_factor_distance:
            raise RuntimeError("RTK factor has neither distance nor max-time trigger")

    before = [norm(vector(row, ("innovation_before_x", "innovation_before_y", "innovation_before_z"))) for row in factors]
    after = [norm(vector(row, ("innovation_after_x", "innovation_after_y", "innovation_after_z"))) for row in factors]
    runtimes = [float(row["optimization_time_ms"]) for row in factors]
    if sum(after) > sum(before) + 1e-6:
        raise RuntimeError("RTK factor updates increased aggregate position residual")
    maximum_deformation = max(float(row["position_delta_m"]) for row in trajectory)
    if maximum_deformation > args.maximum_trajectory_deformation:
        raise RuntimeError("joint Loop+RTK graph exceeded trajectory deformation bound")

    decision_counts = {}
    for row in decisions:
        decision_counts[row["decision"]] = decision_counts.get(row["decision"], 0)+1
    print(f"RTK input: status={len(statuses)} (parsed={parsed_statuses}, mode4={mode4_statuses}), "
          f"solutions={len(solutions)}, keyframe_queries={len(queries)}")
    print(f"RTK factors: count={len(factors)}, solution_ratio={len(factors)/len(solutions):.6f}, "
          f"decisions={decision_counts}")
    print(f"RTK residual: before_median={statistics.median(before):.4f}m, "
          f"after_median={statistics.median(after):.4f}m, after_max={max(after):.4f}m")
    print(f"RTK optimization: median={statistics.median(runtimes):.3f}ms, "
          f"max={max(runtimes):.3f}ms, trajectory_deformation={maximum_deformation:.4f}m")
    print("keyframe-driven status-gated RTK fusion validation passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError, IndexError, ZeroDivisionError) as error:
        print(f"RTK fusion validation failed: {error}")
        raise SystemExit(1)
