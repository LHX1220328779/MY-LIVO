#!/usr/bin/env python3
"""Validate Phase-1 RTK/LIO timing, lever-arm, covariance and yaw diagnostics."""

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


def value(row, name):
    text = row[name]
    if text == "":
        raise RuntimeError(f"missing {name} in an available observation")
    result = float(text)
    if not math.isfinite(result):
        raise RuntimeError(f"non-finite {name}")
    return result


def vector(row, prefix):
    return tuple(value(row, f"{prefix}_{axis}") for axis in "xyz")


def norm(vector_value):
    return math.sqrt(sum(component * component for component in vector_value))


def subtract(left, right):
    return tuple(a - b for a, b in zip(left, right))


def add(left, right):
    return tuple(a + b for a, b in zip(left, right))


def distance(left, right):
    return norm(subtract(left, right))


def percentile(values, fraction):
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    alpha = position - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def require_close(actual, expected, tolerance, message):
    if abs(actual - expected) > tolerance:
        raise RuntimeError(
            f"{message}: actual={actual:.12g}, expected={expected:.12g}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--diagnostics", type=Path,
        default=Path("Log/backend/rtk_diagnostics.csv"))
    parser.add_argument(
        "--initial-alignment", type=Path,
        default=Path("Log/backend/rtk_initial_alignment.csv"))
    parser.add_argument(
        "--keyframes", type=Path,
        default=Path("Log/backend/keyframes.csv"))
    parser.add_argument("--required-mode", type=int, default=4)
    args = parser.parse_args()

    fields, rows = read(args.diagnostics)
    alignment_fields, alignment_rows = read(args.initial_alignment)
    _, keyframes = read(args.keyframes)
    required = {
        "keyframe_id", "timestamp", "observation_available", "query_reason",
        "health", "decision", "factor_added", "lower_timestamp",
        "upper_timestamp", "lower_dt", "upper_dt", "interpolation_gap",
        "interpolation_alpha", "lower_ins_pos_mode", "upper_ins_pos_mode",
        "rtk_raw_x", "rtk_raw_y", "rtk_raw_z", "lever_correction_x",
        "lever_correction_y", "lever_correction_z", "rtk_x", "rtk_y",
        "rtk_z", "reported_variance_x", "reported_variance_y",
        "reported_variance_z", "effective_sigma_x", "effective_sigma_y",
        "effective_sigma_z", "lio_x", "lio_y", "lio_z", "lio_yaw_deg",
        "backend_before_x", "backend_before_y", "backend_before_z",
        "backend_after_x", "backend_after_y", "backend_after_z",
        "rtk_yaw_deg", "lio_minus_rtk_raw_norm", "lio_minus_rtk_norm",
        "backend_before_minus_rtk_norm", "backend_after_minus_rtk_norm",
        "lio_minus_rtk_yaw_deg", "graph_update_translation_m",
        "graph_update_yaw_deg",
    }
    if not required.issubset(fields):
        raise RuntimeError(
            f"diagnostic CSV is missing {sorted(required - set(fields))}")
    required_alignment = {
        "initialization_timestamp", "first_sample_timestamp",
        "last_sample_timestamp", "sample_count", "x", "y", "z", "qx",
        "qy", "qz", "qw", "roll_deg", "pitch_deg", "yaw_deg",
        "position_rms_m", "position_max_m", "orientation_max_deg",
    }
    if not required_alignment.issubset(alignment_fields):
        raise RuntimeError(
            "initial-alignment CSV does not contain the Phase-1 schema")
    if len(alignment_rows) != 1:
        raise RuntimeError("expected exactly one initial-alignment row")
    if not rows or len(rows) != len(keyframes):
        raise RuntimeError(
            f"diagnostic/keyframe row count differs: {len(rows)} != "
            f"{len(keyframes)}")

    alignment = alignment_rows[0]
    sample_count = int(alignment["sample_count"])
    first_sample = value(alignment, "first_sample_timestamp")
    last_sample = value(alignment, "last_sample_timestamp")
    initialization_time = value(alignment, "initialization_timestamp")
    if sample_count <= 0 or first_sample > last_sample + 1e-12 or \
            last_sample > initialization_time + 1e-12:
        raise RuntimeError("invalid initial-alignment sampling interval")
    for name in (
            "x", "y", "z", "qx", "qy", "qz", "qw", "roll_deg",
            "pitch_deg", "yaw_deg", "position_rms_m", "position_max_m",
            "orientation_max_deg"):
        value(alignment, name)

    available = 0
    decisions = {}
    endpoint_offsets = []
    interpolation_gaps = []
    lio_residuals = []
    lio_raw_residuals = []
    lio_yaw_residuals = []
    graph_updates = []
    graph_yaw_updates = []
    reported_zero_axes = 0
    for index, (row, keyframe) in enumerate(zip(rows, keyframes)):
        if int(row["keyframe_id"]) != index or int(keyframe["id"]) != index:
            raise RuntimeError("diagnostic/keyframe IDs are not contiguous")
        timestamp = value(row, "timestamp")
        require_close(timestamp, float(keyframe["timestamp"]), 1e-9,
                      f"keyframe {index} timestamp mismatch")
        decisions[row["decision"]] = decisions.get(row["decision"], 0) + 1
        factor_added = int(row["factor_added"]) == 1
        if factor_added != (row["decision"] == "accepted"):
            raise RuntimeError(f"keyframe {index} factor decision is inconsistent")

        lio = vector(row, "lio")
        backend_before = vector(row, "backend_before")
        backend_after = vector(row, "backend_after")
        update = distance(backend_before, backend_after)
        require_close(value(row, "graph_update_translation_m"), update,
                      1e-8, f"keyframe {index} graph update mismatch")
        graph_updates.append(update)
        graph_yaw_updates.append(abs(value(row, "graph_update_yaw_deg")))

        observation_available = int(row["observation_available"]) == 1
        if not observation_available:
            if row["query_reason"] == "accepted":
                raise RuntimeError(
                    f"keyframe {index} accepted query has no observation")
            continue
        available += 1
        if row["query_reason"] != "accepted" or value(row, "health") != 1.0:
            raise RuntimeError(f"keyframe {index} available RTK is not healthy")
        if int(row["lower_ins_pos_mode"]) != args.required_mode or \
                int(row["upper_ins_pos_mode"]) != args.required_mode:
            raise RuntimeError(f"keyframe {index} used a non-FIX bracket")

        lower = value(row, "lower_timestamp")
        upper = value(row, "upper_timestamp")
        if lower > timestamp + 1e-12 or upper < timestamp - 1e-12:
            raise RuntimeError(f"keyframe {index} RTK does not bracket timestamp")
        lower_dt = timestamp - lower
        upper_dt = upper - timestamp
        gap = upper - lower
        require_close(value(row, "lower_dt"), lower_dt, 1e-9,
                      f"keyframe {index} lower timestamp delta mismatch")
        require_close(value(row, "upper_dt"), upper_dt, 1e-9,
                      f"keyframe {index} upper timestamp delta mismatch")
        require_close(value(row, "interpolation_gap"), gap, 1e-9,
                      f"keyframe {index} interpolation gap mismatch")
        expected_alpha = lower_dt / gap if gap > 0 else 0.0
        require_close(value(row, "interpolation_alpha"), expected_alpha,
                      1e-8, f"keyframe {index} interpolation alpha mismatch")
        endpoint_offsets.append(max(lower_dt, upper_dt))
        interpolation_gaps.append(gap)

        raw_rtk = vector(row, "rtk_raw")
        lever = vector(row, "lever_correction")
        rtk = vector(row, "rtk")
        if distance(add(raw_rtk, lever), rtk) > 1e-8:
            raise RuntimeError(
                f"keyframe {index} raw RTK + lever arm != corrected RTK")
        raw_residual = distance(lio, raw_rtk)
        corrected_residual = distance(lio, rtk)
        require_close(value(row, "lio_minus_rtk_raw_norm"), raw_residual,
                      1e-8, f"keyframe {index} raw RTK residual mismatch")
        require_close(value(row, "lio_minus_rtk_norm"), corrected_residual,
                      1e-8, f"keyframe {index} corrected RTK residual mismatch")
        require_close(value(row, "backend_before_minus_rtk_norm"),
                      distance(backend_before, rtk), 1e-8,
                      f"keyframe {index} backend-before residual mismatch")
        require_close(value(row, "backend_after_minus_rtk_norm"),
                      distance(backend_after, rtk), 1e-8,
                      f"keyframe {index} backend-after residual mismatch")
        lio_raw_residuals.append(raw_residual)
        lio_residuals.append(corrected_residual)
        lio_yaw_residuals.append(abs(value(row, "lio_minus_rtk_yaw_deg")))
        for axis in "xyz":
            reported = value(row, f"reported_variance_{axis}")
            effective_sigma = value(row, f"effective_sigma_{axis}")
            if reported == 0.0:
                reported_zero_axes += 1
            if reported < 0.0 or effective_sigma <= 0.0:
                raise RuntimeError(
                    f"keyframe {index} has invalid covariance diagnostics")

    if available == 0:
        raise RuntimeError("no timestamp- and status-valid RTK observations")
    print(
        f"initial alignment: samples={sample_count}, "
        f"duration={last_sample-first_sample:.3f}s, "
        f"position_rms={value(alignment, 'position_rms_m'):.4f}m, "
        f"position_max={value(alignment, 'position_max_m'):.4f}m, "
        f"orientation_max={value(alignment, 'orientation_max_deg'):.4f}deg")
    print(
        f"RTK diagnostics: keyframes={len(rows)}, available={available}, "
        f"decisions={decisions}, receiver_zero_covariance_axes="
        f"{reported_zero_axes}/{3*available}")
    print(
        "time interpolation: "
        f"endpoint_p95={percentile(endpoint_offsets, .95)*1000:.3f}ms, "
        f"gap_p95={percentile(interpolation_gaps, .95)*1000:.3f}ms, "
        f"endpoint_max={max(endpoint_offsets)*1000:.3f}ms")
    print(
        "pure-LIO/RTK disagreement: "
        f"position_p50={statistics.median(lio_residuals):.4f}m, "
        f"p95={percentile(lio_residuals, .95):.4f}m, "
        f"max={max(lio_residuals):.4f}m, "
        f"yaw_p95={percentile(lio_yaw_residuals, .95):.4f}deg")
    print(
        "lever-arm effect: "
        f"raw_residual_p50={statistics.median(lio_raw_residuals):.4f}m -> "
        f"corrected_residual_p50={statistics.median(lio_residuals):.4f}m")
    print(
        "legacy graph immediate update: "
        f"translation_p95={percentile(graph_updates, .95):.4f}m, "
        f"translation_max={max(graph_updates):.4f}m, "
        f"yaw_p95={percentile(graph_yaw_updates, .95):.4f}deg, "
        f"yaw_max={max(graph_yaw_updates):.4f}deg")
    print("Phase-1 RTK fusion diagnostics validation passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError, IndexError,
            ZeroDivisionError) as error:
        print(f"RTK diagnostics validation failed: {error}")
        raise SystemExit(1)
