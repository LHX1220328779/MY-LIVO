#!/usr/bin/env python3
"""Validate keyframe-driven RTK input and its active fusion architecture."""

import argparse
import csv
import math
import os
import statistics
from pathlib import Path


def read(path, allow_incomplete_tail=False):
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = reader.fieldnames or []
    incomplete_rows = [
        row_number
        for row_number, row in enumerate(rows, start=2)
        if None in row or any(value is None for value in row.values())
    ]
    if incomplete_rows:
        if (allow_incomplete_tail and len(incomplete_rows) == 1 and
                incomplete_rows[0] == len(rows) + 1):
            print(f"warning: ignored incomplete high-rate tail row in {path}")
            rows.pop()
        else:
            raise RuntimeError(
                f"{path} has an incomplete row at line {incomplete_rows[0]}; "
                "the producer did not finish flushing this run")
    return fieldnames, rows


def vector(row, names):
    return tuple(float(row[name]) for name in names)


def norm(values):
    return math.sqrt(sum(value*value for value in values))


def distance(a, b):
    return norm(tuple(x-y for x, y in zip(a, b)))


def quaternion_angle_deg(left, right):
    left_norm = norm(left)
    right_norm = norm(right)
    if left_norm < 1e-12 or right_norm < 1e-12:
        raise RuntimeError("pose provenance contains a zero quaternion")
    dot = abs(sum(a*b for a, b in zip(left, right)) /
              (left_norm*right_norm))
    return math.degrees(2*math.acos(max(-1.0, min(1.0, dot))))


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
    parser.add_argument("--global-observations", type=Path,
                        default=Path("Log/backend/global_rtk_observations.csv"))
    parser.add_argument("--global-trajectory", type=Path,
                        default=Path("Log/backend/global_trajectory.csv"))
    parser.add_argument("--correction-feasibility", type=Path,
                        default=Path("Log/backend/correction_feasibility.csv"))
    parser.add_argument("--correction-field", type=Path,
                        default=Path("Log/backend/correction_field.csv"))
    parser.add_argument(
        "--regularized-field", type=Path,
        default=Path("Log/backend/regularized_correction_field.csv"))
    parser.add_argument(
        "--elastic-acceptance", type=Path,
        default=Path("Log/backend/elastic_segment_acceptance.csv"))
    parser.add_argument("--velocity-guard", type=Path,
                        default=Path("Log/backend/rtk_velocity_guard.csv"))
    parser.add_argument("--relocalization-monitor", type=Path,
                        default=Path("Log/backend/rigid_relocalization.csv"))
    parser.add_argument("--recovery-monitor", type=Path,
                        default=Path(
                            "Log/backend/recovery_relocalization.csv"))
    parser.add_argument("--frontend-segments", type=Path,
                        default=Path("Log/backend/frontend_segments.csv"))
    parser.add_argument("--frontend-restart-supervisor", type=Path,
                        default=Path(
                            "Log/backend/frontend_restart_supervisor.csv"))
    parser.add_argument("--field-knot-spacing", type=float, default=15.0)
    parser.add_argument("--elastic-soft-radius", type=float, default=0.10)
    parser.add_argument("--elastic-full-radius", type=float, default=0.28)
    parser.add_argument("--elastic-minimum-stiffness", type=float,
                        default=0.05)
    parser.add_argument("--elastic-maximum-stiffness", type=float,
                        default=1.0)
    parser.add_argument("--regularized-elastic-soft-radius", type=float,
                        default=0.20)
    parser.add_argument("--regularized-elastic-full-radius", type=float,
                        default=1.00)
    parser.add_argument("--regularized-elastic-minimum-stiffness", type=float,
                        default=0.0)
    parser.add_argument("--regularized-elastic-maximum-stiffness", type=float,
                        default=1.0)
    parser.add_argument("--maximum-planar-field-gradient", type=float,
                        default=0.005)
    parser.add_argument("--maximum-vertical-field-gradient", type=float,
                        default=0.003)
    parser.add_argument("--maximum-field-planar-update", type=float,
                        default=0.05)
    parser.add_argument("--maximum-field-vertical-update", type=float,
                        default=0.03)
    parser.add_argument("--maximum-field-yaw-update-deg", type=float,
                        default=0.15)
    parser.add_argument("--maximum-regularized-planar-gradient", type=float,
                        default=0.20)
    parser.add_argument("--maximum-regularized-vertical-gradient", type=float,
                        default=0.18)
    parser.add_argument("--acceptance-minimum-observations", type=int,
                        default=6)
    parser.add_argument("--acceptance-minimum-path", type=float, default=30.0)
    parser.add_argument("--acceptance-maximum-position", type=float,
                        default=0.30)
    parser.add_argument("--acceptance-maximum-yaw-deg", type=float,
                        default=0.50)
    parser.add_argument("--acceptance-maximum-post-velocity", type=float,
                        default=0.80)
    parser.add_argument("--acceptance-maximum-gradient", type=float,
                        default=0.30)
    parser.add_argument("--acceptance-maximum-yaw-gradient", type=float,
                        default=0.20)
    parser.add_argument("--acceptance-maximum-outlier-fraction", type=float,
                        default=0.25)
    parser.add_argument("--velocity-low-pass-time-constant", type=float,
                        default=2.0)
    parser.add_argument("--receiver-velocity-low-pass-time-constant",
                        type=float, default=0.25)
    parser.add_argument("--maximum-receiver-position-difference-error",
                        type=float, default=1.5)
    parser.add_argument("--velocity-correction-time-constant", type=float,
                        default=0.8)
    parser.add_argument("--velocity-minimum-dt", type=float, default=0.8)
    parser.add_argument("--velocity-maximum-dt", type=float, default=1.2)
    parser.add_argument("--velocity-minimum-distance", type=float,
                        default=0.5)
    parser.add_argument("--velocity-evidence-max-age", type=float,
                        default=1.5)
    parser.add_argument("--maximum-planar-velocity-correction", type=float,
                        default=2.0)
    parser.add_argument("--maximum-vertical-velocity-correction", type=float,
                        default=0.75)
    parser.add_argument("--maximum-planar-velocity-acceleration", type=float,
                        default=0.30)
    parser.add_argument("--maximum-vertical-velocity-acceleration",
                        type=float, default=0.15)
    parser.add_argument("--recovery-tracking-time-constant", type=float,
                        default=0.35)
    parser.add_argument("--recovery-tracking-planar-acceleration",
                        type=float, default=1.50)
    parser.add_argument("--recovery-tracking-vertical-acceleration",
                        type=float, default=0.75)
    parser.add_argument("--emergency-restart-velocity-error", type=float,
                        default=2.0)
    parser.add_argument("--emergency-restart-observations", type=int,
                        default=3)
    parser.add_argument("--relocalization-velocity-error", type=float,
                        default=0.8)
    parser.add_argument("--relocalization-minimum-observations", type=int,
                        default=4)
    parser.add_argument("--relocalization-minimum-path", type=float,
                        default=30.0)
    parser.add_argument("--relocalization-scale-error", type=float,
                        default=0.05)
    parser.add_argument("--relocalization-planar-rms", type=float,
                        default=0.35)
    parser.add_argument("--relocalization-vertical-rms", type=float,
                        default=0.50)
    parser.add_argument("--recovery-minimum-path", type=float, default=30.0)
    parser.add_argument("--recovery-scale-error", type=float, default=0.05)
    parser.add_argument("--recovery-planar-rms", type=float, default=0.75)
    parser.add_argument("--recovery-vertical-rms", type=float, default=1.0)
    parser.add_argument("--restart-structural-rejections", type=int,
                        default=3)
    parser.add_argument("--restart-minimum-rejection-span", type=float,
                        default=15.0)
    parser.add_argument("--frontend-segment-minimum-seed-points", type=int,
                        default=1000)
    parser.add_argument("--maximum-automatic-restarts", type=int, default=1)
    parser.add_argument("--diagnostic-minimum-condition-ratio", type=float,
                        default=0.01)
    parser.add_argument("--diagnostic-minimum-scale-error", type=float,
                        default=0.03)
    parser.add_argument("--diagnostic-minimum-nonyaw-rotation-deg",
                        type=float, default=1.0)
    parser.add_argument("--diagnostic-maximum-similarity-rms", type=float,
                        default=0.40)
    parser.add_argument("--required-mode", type=int, default=4)
    parser.add_argument("--minimum-factor-dt", type=float, default=2.0)
    parser.add_argument("--maximum-factor-dt", type=float, default=10.0)
    parser.add_argument("--minimum-factor-distance", type=float, default=5.0)
    parser.add_argument("--maximum-factor-solution-ratio", type=float,
                        default=0.02)
    parser.add_argument("--maximum-trajectory-deformation", type=float,
                        default=None,
                        help="optional absolute raw/optimized displacement "
                             "limit; disabled by default because correcting "
                             "long-term LIO drift can legitimately exceed 5 m")
    parser.add_argument("--allow-live-logs", action="store_true",
                        help="diagnostic escape hatch; fixed snapshots are "
                             "required for acceptance")
    args = parser.parse_args()

    active_producers = active_mapping_processes()
    if active_producers and not args.allow_live_logs:
        raise RuntimeError(
            "fastlivo_mapping is still running (PID " +
            ",".join(str(pid) for pid in active_producers) +
            "); stop the launch process before validating its CSV snapshot")

    # Status and RTK solution files are high-rate input audits.  A process
    # interrupted during its final write may leave one unusable tail row;
    # dropping only that row is safe.  Keyframe-rate contract files below are
    # intentionally strict and must be complete.
    status_fields, statuses = read(args.status, allow_incomplete_tail=True)
    solution_fields, solutions = read(
        args.solutions, allow_incomplete_tail=True)
    query_fields, queries = read(args.queries)
    decision_fields, decisions = read(args.decisions)
    factor_fields, factors = read(args.factors)
    keyframe_fields, keyframes = read(args.keyframes)
    trajectory_fields, trajectory = read(args.trajectory)
    requirements = [
        (status_fields, {"timestamp", "parsed", "ins_pos_mode"}, "status"),
        (solution_fields, {"timestamp", "x", "y", "z", "sigma_x", "sigma_y", "sigma_z"}, "solution"),
        (query_fields, {"timestamp", "accepted", "reason", "lower_timestamp", "upper_timestamp", "alpha", "x", "y", "z"}, "query"),
        (decision_fields, {"keyframe_id", "timestamp", "decision", "factor_added", "measurement_x", "measurement_y", "measurement_z", "innovation_chi2"}, "decision"),
        (keyframe_fields, {
            "id", "timestamp", "tx", "ty", "tz", "qx", "qy", "qz",
            "qw"}, "keyframe"),
        (trajectory_fields, {
            "id", "timestamp", "raw_tx", "raw_ty", "raw_tz", "raw_qx",
            "raw_qy", "raw_qz", "raw_qw", "opt_tx", "opt_ty", "opt_tz",
            "opt_qx", "opt_qy", "opt_qz", "opt_qw",
            "position_delta_m"}, "trajectory"),
    ]
    for fields, required, name in requirements:
        if not required.issubset(fields):
            raise RuntimeError(f"{name} CSV is missing {sorted(required-set(fields))}")
    if not statuses or not solutions or not keyframes:
        raise RuntimeError("RTK fusion logs contain no usable data")
    if (len(queries) != len(keyframes) or
            len(decisions) != len(keyframes) or
            len(trajectory) != len(keyframes)):
        raise RuntimeError(
            "incomplete or mixed-run backend logs: "
            f"keyframes={len(keyframes)}, queries={len(queries)}, "
            f"decisions={len(decisions)}, trajectory={len(trajectory)}; "
            "rebuild and rerun so all keyframe-rate files come from the "
            "same mapping process")

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
    # The buffer deliberately accepts equal timestamps as an updated sample
    # and replaces the in-memory endpoint.  The raw audit CSV still records
    # both receiver messages, so only backward time is invalid here.
    if any(solution_times[i] < solution_times[i-1]
           for i in range(1, len(solution_times))):
        raise RuntimeError("RTK solution timestamps moved backwards")
    for row in solutions:
        if any(not math.isfinite(float(row[name])) for name in
               ("x", "y", "z", "sigma_x", "sigma_y", "sigma_z")):
            raise RuntimeError("RTK solution contains non-finite data")
        if any(float(row[name]) <= 0 for name in ("sigma_x", "sigma_y", "sigma_z")):
            raise RuntimeError("RTK covariance floor was not applied")
    receiver_solution_fields = {
        "velocity_valid", "velocity_x", "velocity_y", "velocity_z",
        "velocity_sigma_x", "velocity_sigma_y", "velocity_sigma_z",
        "reported_velocity_variance_x",
        "reported_velocity_variance_y",
        "reported_velocity_variance_z",
    }
    have_receiver_solution_schema = receiver_solution_fields.issubset(
        solution_fields)
    if receiver_solution_fields.intersection(solution_fields) and not \
            have_receiver_solution_schema:
        raise RuntimeError("receiver-velocity solution audit is partial")
    if have_receiver_solution_schema:
        for row in solutions:
            velocity_valid = int(row["velocity_valid"]) == 1
            velocity_names = (
                "velocity_x", "velocity_y", "velocity_z",
                "velocity_sigma_x", "velocity_sigma_y", "velocity_sigma_z")
            if velocity_valid and (
                    any(not math.isfinite(float(row[name]))
                        for name in velocity_names) or
                    any(float(row[name]) <= 0.0 for name in
                        ("velocity_sigma_x", "velocity_sigma_y",
                         "velocity_sigma_z"))):
                raise RuntimeError(
                    "receiver velocity/covariance is invalid")

    # The separated architecture leaves the legacy RTK factor file
    # header-only. RTK may change C(s), but never the local graph trajectory.
    if not factors and args.global_observations.is_file() and \
            args.global_trajectory.is_file():
        observation_fields, observations = read(args.global_observations)
        global_fields, global_trajectory = read(args.global_trajectory)
        required_observation = {
            "keyframe_id", "timestamp", "health", "x", "y", "z",
            "sigma_x", "sigma_y", "sigma_z",
        }
        required_global = {
            "id", "timestamp", "local_tx", "local_ty", "local_tz",
            "local_qx", "local_qy", "local_qz", "local_qw",
            "global_tx", "global_ty", "global_tz", "global_qx",
            "global_qy", "global_qz", "global_qw",
            "correction_translation_m", "correction_angle_deg",
            "map_eligible",
        }
        if not required_observation.issubset(observation_fields):
            raise RuntimeError("global RTK observation CSV is incomplete")
        if not required_global.issubset(global_fields):
            raise RuntimeError("global trajectory CSV is incomplete")
        if len(global_trajectory) != len(keyframes):
            raise RuntimeError("global trajectory/keyframe cadence differs")
        accepted = [row for row in decisions
                    if row["decision"] == "accepted_global_observation"]
        if not accepted or len(accepted) != len(observations):
            raise RuntimeError("selected global RTK observations differ from decisions")
        if any(int(row["factor_added"]) != 0 for row in decisions):
            raise RuntimeError("RTK still added a factor to the local graph")
        if any(row["decision"] == "innovation_gate" for row in decisions):
            raise RuntimeError("healthy RTK was rejected by local innovation")
        accepted_ids = {int(row["keyframe_id"]) for row in accepted}
        observation_ids = {int(row["keyframe_id"]) for row in observations}
        if accepted_ids != observation_ids or len(observation_ids) != len(observations):
            raise RuntimeError("global RTK observations have duplicate or wrong IDs")

        maximum_raw_input_error = 0.0
        maximum_raw_input_angle = 0.0
        maximum_local_input_error = 0.0
        maximum_local_input_angle = 0.0
        maximum_global_translation = 0.0
        maximum_global_angle = 0.0
        local_positions = []
        global_positions = []
        map_eligibility = []
        for index, (keyframe, local_row, global_row) in enumerate(
                zip(keyframes, trajectory, global_trajectory)):
            if int(global_row["id"]) != index or int(local_row["id"]) != index:
                raise RuntimeError("global trajectory IDs are not contiguous")
            if abs(float(global_row["timestamp"])-float(keyframe["timestamp"])) > 1e-9:
                raise RuntimeError("global trajectory timestamp differs")
            local_from_graph = vector(
                local_row, ("opt_tx", "opt_ty", "opt_tz"))
            local_rotation_from_graph = vector(
                local_row, ("opt_qx", "opt_qy", "opt_qz", "opt_qw"))
            local_from_global = vector(
                global_row, ("local_tx", "local_ty", "local_tz"))
            local_rotation_from_global = vector(
                global_row,
                ("local_qx", "local_qy", "local_qz", "local_qw"))
            global_position = vector(
                global_row, ("global_tx", "global_ty", "global_tz"))
            eligible = int(global_row["map_eligible"])
            if eligible not in (0, 1):
                raise RuntimeError("global-map eligibility is not binary")
            map_eligibility.append(eligible)
            raw_from_keyframe = vector(keyframe, ("tx", "ty", "tz"))
            raw_rotation_from_keyframe = vector(
                keyframe, ("qx", "qy", "qz", "qw"))
            raw_from_graph = vector(
                local_row, ("raw_tx", "raw_ty", "raw_tz"))
            raw_rotation_from_graph = vector(
                local_row, ("raw_qx", "raw_qy", "raw_qz", "raw_qw"))
            local_positions.append(local_from_global)
            global_positions.append(global_position)
            maximum_raw_input_error = max(
                maximum_raw_input_error,
                distance(raw_from_keyframe, raw_from_graph))
            maximum_raw_input_angle = max(
                maximum_raw_input_angle,
                quaternion_angle_deg(raw_rotation_from_keyframe,
                                     raw_rotation_from_graph))
            maximum_local_input_error = max(
                maximum_local_input_error,
                distance(local_from_graph, local_from_global))
            maximum_local_input_angle = max(
                maximum_local_input_angle,
                quaternion_angle_deg(local_rotation_from_graph,
                                     local_rotation_from_global))
            maximum_global_translation = max(
                maximum_global_translation,
                distance(local_from_global, global_position))
            maximum_global_angle = max(
                maximum_global_angle,
                abs(float(global_row["correction_angle_deg"])))
        if maximum_raw_input_error > 1e-8 or \
                maximum_raw_input_angle > 1e-5:
            raise RuntimeError(
                "local graph raw input differs from immutable keyframe odometry")
        if maximum_local_input_error > 1e-8 or \
                maximum_local_input_angle > 1e-5:
            raise RuntimeError("global layer input differs from local SLAM output")

        maximum_planar_gradient = 0.0
        maximum_vertical_gradient = 0.0
        emergency_segment_starts = set()
        if args.frontend_segments.is_file():
            preliminary_segment_fields, preliminary_segments = read(
                args.frontend_segments)
            if "restart_kind" in preliminary_segment_fields:
                emergency_segment_starts = {
                    int(row["trigger_keyframe_id"])+1
                    for row in preliminary_segments
                    if row["restart_kind"] == "velocity"}
        corrections = [tuple(global_value-local_value
                             for global_value, local_value in zip(global_pose, local_pose))
                       for local_pose, global_pose in
                       zip(local_positions, global_positions)]
        for index in range(1, len(corrections)):
            # A recovery segment uses one constant 4DOF frame transform, not
            # C(s). Its boundary jump and carried yaw must therefore not be
            # interpreted as elastic-field spatial gradient.
            if emergency_segment_starts and \
                    index >= min(emergency_segment_starts):
                continue
            ds = distance(local_positions[index-1], local_positions[index])
            if ds <= 1e-9:
                continue
            delta = tuple(current-previous for current, previous in
                          zip(corrections[index], corrections[index-1]))
            maximum_planar_gradient = max(
                maximum_planar_gradient, norm(delta[:2])/ds)
            maximum_vertical_gradient = max(
                maximum_vertical_gradient, abs(delta[2])/ds)

        decision_counts = {}
        for row in decisions:
            decision_counts[row["decision"]] = decision_counts.get(row["decision"], 0)+1
        print(f"RTK input: status={len(statuses)} (parsed={parsed_statuses}, "
              f"mode4={mode4_statuses}), solutions={len(solutions)}, "
              f"keyframe_queries={len(queries)}")
        print(f"global layer: local_graph_rtk_factors=0, "
              f"selected_observations={len(observations)}, decisions={decision_counts}")
        print(f"pose provenance: keyframe/raw_graph="
              f"{maximum_raw_input_error:.3g}m/"
              f"{maximum_raw_input_angle:.3g}deg")
        print(f"local/global isolation: max_local_input_error="
              f"{maximum_local_input_error:.3g}m/"
              f"{maximum_local_input_angle:.3g}deg, global_correction="
              f"{maximum_global_translation:.4f}m/{maximum_global_angle:.4f}deg")
        global_rtk_errors = [
            distance(
                global_positions[int(row["keyframe_id"])],
                vector(row, ("x", "y", "z")))
            for row in observations]
        sorted_global_rtk_errors = sorted(global_rtk_errors)
        global_rtk_p95 = sorted_global_rtk_errors[
            int(0.95 * (len(sorted_global_rtk_errors)-1))]
        print("global/RTK accuracy (diagnostic, not final acceptance): "
              f"median={statistics.median(global_rtk_errors):.3f}m, "
              f"p95={global_rtk_p95:.3f}m, "
              f"max={max(global_rtk_errors):.3f}m, "
              f"last={global_rtk_errors[-1]:.3f}m")
        first_relocalization = None
        monitor_rows = []
        if args.correction_feasibility.is_file():
            monitor_fields, monitor_rows = read(args.correction_feasibility)
            required_monitor = {
                "keyframe_id", "regime", "elastic_correction_allowed",
                "residual_m", "baseline_keyframe_id", "baseline_m",
                "correction_gradient_m_per_m", "local_x", "local_y",
                "local_z", "rtk_x", "rtk_y", "rtk_z",
            }
            if not required_monitor.issubset(monitor_fields):
                raise RuntimeError("correction-feasibility CSV is incomplete")
            if len(monitor_rows) != len(observations):
                raise RuntimeError(
                    "correction monitor/selected observation cadence differs")
            rows_by_id = {int(row["keyframe_id"]): row for row in monitor_rows}
            if set(rows_by_id) != observation_ids or \
                    len(rows_by_id) != len(monitor_rows):
                raise RuntimeError("correction monitor IDs differ or duplicate")
            latched = False
            monitor_counts = {}
            maximum_gradient = 0.0
            for row in monitor_rows:
                row_id = int(row["keyframe_id"])
                regime = row["regime"]
                monitor_counts[regime] = monitor_counts.get(regime, 0)+1
                if regime not in {
                        "warmup", "elastic", "degraded",
                        "relocalization_required"}:
                    raise RuntimeError("unknown correction regime")
                local = vector(row, ("local_x", "local_y", "local_z"))
                rtk = vector(row, ("rtk_x", "rtk_y", "rtk_z"))
                residual = tuple(a-b for a, b in zip(local, rtk))
                if abs(norm(residual)-float(row["residual_m"])) > 1e-8:
                    raise RuntimeError("correction residual log is inconsistent")
                gradient = float(row["correction_gradient_m_per_m"])
                maximum_gradient = max(maximum_gradient, gradient)
                baseline_id_text = row["baseline_keyframe_id"]
                if baseline_id_text:
                    baseline_id = int(baseline_id_text)
                    if baseline_id not in rows_by_id or baseline_id >= row_id:
                        raise RuntimeError("correction baseline ID is invalid")
                    baseline = rows_by_id[baseline_id]
                    baseline_residual = tuple(a-b for a, b in zip(
                        vector(baseline, ("local_x", "local_y", "local_z")),
                        vector(baseline, ("rtk_x", "rtk_y", "rtk_z"))))
                    expected = distance(residual, baseline_residual) / \
                        float(row["baseline_m"])
                    if abs(expected-gradient) > 1e-8:
                        raise RuntimeError(
                            "correction gradient log is inconsistent")
                if regime == "relocalization_required":
                    latched = True
                    if first_relocalization is None:
                        first_relocalization = row_id
                elif latched:
                    raise RuntimeError("relocalization request was not latched")
                if latched and int(row["elastic_correction_allowed"]) != 0:
                    raise RuntimeError(
                        "elastic correction remained enabled after latch")
            print(f"correction feasibility: states={monitor_counts}, "
                  f"max_gradient={maximum_gradient:.4f}m/m, "
                  f"first_relocalization_kf={first_relocalization}")
        expected_quarantine_start = first_relocalization
        actual_quarantine_start = next(
            (index for index, eligible in enumerate(map_eligibility)
             if not eligible), None)
        if actual_quarantine_start != expected_quarantine_start:
            raise RuntimeError(
                "global-map quarantine does not start at relocalization latch")
        acceptance_rows = []
        trusted_resume = None
        if args.elastic_acceptance.is_file():
            acceptance_fields, acceptance_rows = read(
                args.elastic_acceptance)
            required_acceptance = {
                "keyframe_id", "timestamp", "decision",
                "window_start_keyframe_id", "window_size", "valid_samples",
                "valid_ratio", "window_path_m",
                "position_residual_m", "yaw_residual_deg",
                "post_velocity_error_mps", "field_gradient_m_per_m",
                "yaw_gradient_deg_per_m", "accepted", "state_changed",
                "trusted_start_keyframe_id",
            }
            if not required_acceptance.issubset(acceptance_fields):
                raise RuntimeError(
                    "elastic-segment acceptance CSV is incomplete")
            acceptance_ids = [int(row["keyframe_id"])
                              for row in acceptance_rows]
            if any(current <= previous for previous, current in
                   zip(acceptance_ids, acceptance_ids[1:])):
                raise RuntimeError(
                    "elastic acceptance IDs are duplicate or unordered")
            accepted_rows = [row for row in acceptance_rows
                             if int(row["state_changed"]) == 1]
            if len(accepted_rows) > 1:
                raise RuntimeError(
                    "elastic recovery segment was accepted more than once")
            for row in acceptance_rows:
                decision = row["decision"]
                window_size = int(row["window_size"])
                valid_samples = int(row["valid_samples"])
                valid_ratio = float(row["valid_ratio"])
                if not 0 <= valid_samples <= window_size or (
                        window_size > 0 and abs(
                            valid_ratio-valid_samples/window_size) > 1e-9):
                    raise RuntimeError(
                        "elastic-acceptance robust window is inconsistent")
                valid = (
                    float(row["position_residual_m"]) <=
                        args.acceptance_maximum_position+1e-9 and
                    float(row["yaw_residual_deg"]) <=
                        args.acceptance_maximum_yaw_deg+1e-9 and
                    float(row["post_velocity_error_mps"]) <=
                        args.acceptance_maximum_post_velocity+1e-9 and
                    float(row["field_gradient_m_per_m"]) <=
                        args.acceptance_maximum_gradient+1e-9 and
                    float(row["yaw_gradient_deg_per_m"]) <=
                        args.acceptance_maximum_yaw_gradient+1e-9)
                if decision == "rejected" and valid:
                    raise RuntimeError(
                        "valid elastic-acceptance observation was rejected")
                if decision in {"monitoring", "accepted"} and not valid:
                    raise RuntimeError(
                        "invalid elastic-acceptance observation entered window")
                if decision == "accepted":
                    if int(row["accepted"]) != 1 or \
                            int(row["state_changed"]) != 1 or \
                            int(row["window_size"]) < \
                                args.acceptance_minimum_observations or \
                            valid_ratio+1e-9 < 1.0- \
                                args.acceptance_maximum_outlier_fraction or \
                            float(row["window_path_m"]) + 1e-9 < \
                                args.acceptance_minimum_path:
                        raise RuntimeError(
                            "elastic segment accepted before its full window")
                    trusted_resume = int(row["trusted_start_keyframe_id"])
                elif int(row["state_changed"]) != 0:
                    raise RuntimeError(
                        "non-acceptance row changed elastic state")
        if actual_quarantine_start is not None:
            if trusted_resume is None:
                expected_eligibility = [
                    int(index < actual_quarantine_start)
                    for index in range(len(map_eligibility))]
            else:
                if trusted_resume <= actual_quarantine_start:
                    raise RuntimeError(
                        "elastic acceptance did not preserve failed-tail isolation")
                expected_eligibility = [
                    int(index < actual_quarantine_start or
                        index >= trusted_resume)
                    for index in range(len(map_eligibility))]
            if map_eligibility != expected_eligibility:
                raise RuntimeError(
                    "global-map quarantine/accepted segment boundary differs")
        print(f"global map quarantine: start_kf={actual_quarantine_start}, "
              f"trusted={sum(map_eligibility)}, "
              f"quarantined={len(map_eligibility)-sum(map_eligibility)}, "
              f"trusted_resume_kf={trusted_resume}")
        if args.correction_field.is_file():
            field_fields, field_rows = read(args.correction_field)
            required_field = {
                "keyframe_id", "regime", "decision", "distance_m",
                "knot_count", "field_changed", "frozen",
                "correction_x", "correction_y", "correction_z",
                "correction_yaw_deg", "residual_before_m",
                "residual_after_m", "planar_step_m", "vertical_step_m",
                "yaw_step_deg", "elastic_distance_m",
                "elastic_stiffness", "radial_force_proxy_m",
            }
            if not required_field.issubset(field_fields):
                raise RuntimeError("correction-field CSV is incomplete")
            if len(field_rows) != len(observations):
                raise RuntimeError(
                    "correction field/selected observation cadence differs")
            field_ids = [int(row["keyframe_id"]) for row in field_rows]
            if field_ids != [int(row["keyframe_id"])
                             for row in observations]:
                raise RuntimeError("correction field observation IDs differ")
            if monitor_rows and any(
                    field["regime"] != monitor["regime"]
                    for field, monitor in zip(field_rows, monitor_rows)):
                raise RuntimeError("monitor and correction-field regimes differ")

            field_counts = {}
            previous_knots = 0
            previous_update_distance = None
            frozen = False
            elastic_stiffness_values = []
            for row in field_rows:
                decision = row["decision"]
                field_counts[decision] = field_counts.get(decision, 0)+1
                if decision not in {
                        "disabled", "anchor", "monitor_warmup",
                        "knot_spacing", "updated", "frozen_relocalization"}:
                    raise RuntimeError("unknown correction-field decision")
                elastic_distance = float(row["elastic_distance_m"])
                stiffness = float(row["elastic_stiffness"])
                force_proxy = float(row["radial_force_proxy_m"])
                if elastic_distance <= args.elastic_soft_radius:
                    expected_stiffness = args.elastic_minimum_stiffness
                elif elastic_distance >= args.elastic_full_radius:
                    expected_stiffness = args.elastic_maximum_stiffness
                else:
                    x = (elastic_distance-args.elastic_soft_radius) / \
                        (args.elastic_full_radius-args.elastic_soft_radius)
                    smooth = x*x*x*(10+x*(-15+6*x))
                    expected_stiffness = args.elastic_minimum_stiffness + \
                        (args.elastic_maximum_stiffness-
                         args.elastic_minimum_stiffness)*smooth
                if abs(stiffness-expected_stiffness) > 1e-9:
                    raise RuntimeError(
                        "radial elastic stiffness is inconsistent")
                if abs(elastic_distance-float(row["residual_before_m"])) > 1e-9:
                    raise RuntimeError(
                        "elastic distance did not use corrected global pose")
                if abs(force_proxy-stiffness*elastic_distance) > 1e-9:
                    raise RuntimeError(
                        "radial elastic force proxy is inconsistent")
                elastic_stiffness_values.append(stiffness)
                knot_count = int(row["knot_count"])
                if knot_count < previous_knots or knot_count > previous_knots+1:
                    raise RuntimeError("correction-field knot count jumped")
                if decision == "updated":
                    if int(row["field_changed"]) != 1 or \
                            knot_count != previous_knots+1:
                        raise RuntimeError("correction-field update flag differs")
                elif decision == "anchor":
                    if int(row["field_changed"]) != 0 or \
                            knot_count != previous_knots+1:
                        raise RuntimeError("correction-field anchor is invalid")
                elif int(row["field_changed"]) != 0:
                    raise RuntimeError("non-update changed the correction field")
                if decision == "anchor":
                    anchor = vector(row, (
                        "correction_x", "correction_y", "correction_z"))
                    if norm(anchor) > 1e-10 or \
                            abs(float(row["correction_yaw_deg"])) > 1e-10:
                        raise RuntimeError("correction-field gauge is not anchored")
                    previous_update_distance = float(row["distance_m"])
                if decision == "updated":
                    distance_m = float(row["distance_m"])
                    if previous_update_distance is None or \
                            distance_m-previous_update_distance+1e-8 < \
                            args.field_knot_spacing:
                        raise RuntimeError("correction knots violate spacing")
                    previous_update_distance = distance_m
                    if float(row["planar_step_m"]) > \
                            args.maximum_field_planar_update+1e-8 or \
                            float(row["vertical_step_m"]) > \
                            args.maximum_field_vertical_update+1e-8 or \
                            float(row["yaw_step_deg"]) > \
                            args.maximum_field_yaw_update_deg+1e-8:
                        raise RuntimeError("correction knot update exceeded limit")
                if row["regime"] == "relocalization_required":
                    frozen = True
                if frozen and (decision != "frozen_relocalization" or
                               int(row["frozen"]) != 1):
                    raise RuntimeError(
                        "correction field was not frozen after relocalization")
                previous_knots = knot_count

            if not args.regularized_field.is_file() and (
                    maximum_planar_gradient >
                    args.maximum_planar_field_gradient+2e-5 or
                    maximum_vertical_gradient >
                    args.maximum_vertical_field_gradient+2e-5):
                raise RuntimeError(
                    "global trajectory exceeded correction-field gradient")
            field_role = "shadow feasibility field" if \
                args.regularized_field.is_file() else "bounded correction field"
            print(f"{field_role}: decisions={field_counts}, "
                  f"knots={previous_knots}, max_spatial_gradient="
                  f"{maximum_planar_gradient:.5f}/"
                  f"{maximum_vertical_gradient:.5f}m/m, elastic_stiffness="
                  f"{min(elastic_stiffness_values):.3f}.."
                  f"{max(elastic_stiffness_values):.3f}")
        elif maximum_global_translation > 1e-8 or maximum_global_angle > 1e-5:
            raise RuntimeError("global correction exists without field audit")
        else:
            print("correction field: legacy Phase-2 identity output")
        guard_rows = []
        velocity_restart_transition_ids = set()
        have_recovery_tracking_schema = False
        if args.velocity_guard.is_file():
            guard_fields, guard_rows = read(args.velocity_guard)
            required_guard = {
                "keyframe_id", "timestamp", "regime", "decision",
                "baseline_available", "active", "state_changed",
                "correction_applied", "interval_sec", "raw_rtk_vx",
                "raw_rtk_vy", "raw_rtk_vz", "filtered_rtk_vx",
                "filtered_rtk_vy", "filtered_rtk_vz", "lio_before_vx",
                "lio_before_vy", "lio_before_vz", "lio_after_vx",
                "lio_after_vy", "lio_after_vz", "velocity_error_mps",
                "speed_ratio", "applied_gain", "correction_x", "correction_y",
                "correction_z",
            }
            if not required_guard.issubset(guard_fields):
                raise RuntimeError("RTK velocity-guard CSV is incomplete")
            emergency_guard_fields = {
                "emergency_restart_required",
                "emergency_restart_state_changed",
                "emergency_restart_evidence",
            }
            have_emergency_restart_schema = \
                emergency_guard_fields.issubset(guard_fields)
            if emergency_guard_fields.intersection(guard_fields) and not \
                    have_emergency_restart_schema:
                raise RuntimeError(
                    "RTK velocity emergency-restart audit is partial")
            have_recovery_tracking_schema = \
                "recovery_tracking" in guard_fields
            have_receiver_velocity_schema = "velocity_source" in guard_fields
            receiver_query_fields = {
                "velocity_valid", "velocity_x", "velocity_y", "velocity_z",
                "velocity_sigma_x", "velocity_sigma_y", "velocity_sigma_z",
                "reported_velocity_variance_x",
                "reported_velocity_variance_y",
                "reported_velocity_variance_z",
            }
            if have_receiver_velocity_schema and not \
                    receiver_query_fields.issubset(query_fields):
                raise RuntimeError(
                    "receiver-velocity query audit fields are incomplete")
            guard_ids = [int(row["keyframe_id"]) for row in guard_rows]
            if not guard_rows or any(
                    current <= previous for previous, current in
                    zip(guard_ids, guard_ids[1:])):
                raise RuntimeError(
                    "velocity-guard IDs are empty, duplicate or unordered")
            decisions_by_id = {
                int(row["keyframe_id"]): row for row in decisions}
            if any(keyframe_id not in decisions_by_id or
                   not decisions_by_id[keyframe_id]["measurement_x"]
                   for keyframe_id in guard_ids):
                raise RuntimeError(
                    "velocity guard used a keyframe without valid RTK")
            monitor_by_id = {
                int(row["keyframe_id"]): row for row in monitor_rows}
            expected_regime = "warmup"
            expected_regime_by_id = {}
            for keyframe_id in range(len(keyframes)):
                if keyframe_id in monitor_by_id:
                    expected_regime = monitor_by_id[keyframe_id]["regime"]
                expected_regime_by_id[keyframe_id] = expected_regime
            if monitor_rows and any(
                    row["regime"] != expected_regime_by_id[row_id]
                    for row, row_id in zip(guard_rows, guard_ids)):
                raise RuntimeError(
                    "velocity guard did not use the latest global regime")

            guard_counts = {}
            maximum_velocity_error = 0.0
            maximum_velocity_correction = 0.0
            first_activation = None
            expected_filtered = None
            guard_intervals = []
            velocity_source_counts = {}
            recovery_tracking_updates = 0
            emergency_evidence = 0
            emergency_completed = False
            for index, row in enumerate(guard_rows):
                row_id = int(row["keyframe_id"])
                observation = decisions_by_id[row_id]
                decision = row["decision"]
                guard_counts[decision] = guard_counts.get(decision, 0)+1
                velocity_source = row.get(
                    "velocity_source", "position_difference")
                velocity_source_counts[velocity_source] = \
                    velocity_source_counts.get(velocity_source, 0)+1
                recovery_tracking = have_recovery_tracking_schema and \
                    int(row["recovery_tracking"]) == 1
                recovery_tracking_updates += recovery_tracking
                if velocity_source not in {
                        "unavailable", "receiver_twist",
                        "position_difference"}:
                    raise RuntimeError("unknown RTK velocity source")
                if decision not in {
                        "warmup", "monitoring", "activated", "corrected",
                        "recovered", "healthy_vertical_aiding"}:
                    raise RuntimeError("unknown velocity-guard decision")
                if index == 0:
                    if int(row["baseline_available"]) != 0:
                        raise RuntimeError("velocity guard first row has baseline")
                    if have_emergency_restart_schema and any(
                            int(row[name]) != 0 for name in
                            emergency_guard_fields):
                        raise RuntimeError(
                            "velocity guard warmup requested a restart")
                    continue
                if int(row["baseline_available"]) != 1:
                    raise RuntimeError("velocity guard lost its baseline")
                previous_observation = decisions_by_id[guard_ids[index-1]]
                dt = float(observation["timestamp"]) - \
                    float(previous_observation["timestamp"])
                if abs(dt-float(row["interval_sec"])) > 1e-8 or dt <= 0:
                    raise RuntimeError("velocity-guard interval is inconsistent")
                guard_intervals.append(dt)
                movement = distance(
                    vector(observation, (
                        "measurement_x", "measurement_y", "measurement_z")),
                    vector(previous_observation, (
                        "measurement_x", "measurement_y", "measurement_z")))
                if dt+1e-9 < args.velocity_minimum_dt or (
                        dt < args.velocity_maximum_dt-1e-9 and
                        movement+1e-9 < args.velocity_minimum_distance):
                    raise RuntimeError(
                        "velocity guard violated its independent cadence")
                expected_raw = tuple((current-previous)/dt
                                     for current, previous in zip(
                                         vector(observation, (
                                             "measurement_x", "measurement_y",
                                             "measurement_z")),
                                         vector(previous_observation,
                                                ("measurement_x",
                                                 "measurement_y",
                                                 "measurement_z"))))
                filter_time_constant = \
                    args.velocity_low_pass_time_constant
                if have_receiver_velocity_schema:
                    query = queries[row_id]
                    receiver_available = int(query["accepted"]) == 1 and \
                        int(query["velocity_valid"]) == 1
                    receiver_raw = vector(
                        query, ("velocity_x", "velocity_y", "velocity_z")) \
                        if receiver_available else None
                    receiver_consistent = receiver_available and distance(
                        receiver_raw, expected_raw) <= \
                        args.maximum_receiver_position_difference_error+1e-8
                    expected_source = "receiver_twist" if \
                        receiver_consistent else "position_difference"
                    if velocity_source != expected_source:
                        raise RuntimeError(
                            "RTK velocity source arbitration differs")
                if velocity_source == "receiver_twist":
                    expected_raw = receiver_raw
                    filter_time_constant = \
                        args.receiver_velocity_low_pass_time_constant
                elif velocity_source != "position_difference":
                    raise RuntimeError(
                        "available velocity baseline has no source")
                logged_raw = vector(
                    row, ("raw_rtk_vx", "raw_rtk_vy", "raw_rtk_vz"))
                if distance(expected_raw, logged_raw) > 1e-8:
                    raise RuntimeError("velocity guard changed RTK velocity")
                if expected_filtered is None:
                    expected_filtered = expected_raw
                else:
                    alpha = 1-math.exp(
                        -dt/filter_time_constant)
                    expected_filtered = tuple(
                        old+alpha*(raw-old) for old, raw in
                        zip(expected_filtered, expected_raw))
                logged_filtered = vector(row, (
                    "filtered_rtk_vx", "filtered_rtk_vy",
                    "filtered_rtk_vz"))
                if distance(expected_filtered, logged_filtered) > 1e-8:
                    raise RuntimeError("velocity low-pass is inconsistent")
                before = vector(
                    row, ("lio_before_vx", "lio_before_vy", "lio_before_vz"))
                after = vector(
                    row, ("lio_after_vx", "lio_after_vy", "lio_after_vz"))
                correction = vector(
                    row, ("correction_x", "correction_y", "correction_z"))
                gain = float(row["applied_gain"])
                correction_time_constant = \
                    args.recovery_tracking_time_constant if \
                    recovery_tracking else \
                    args.velocity_correction_time_constant
                expected_gain = 1-math.exp(
                    -dt/correction_time_constant)
                if int(row["active"]) == 1 and decision != "recovered":
                    if abs(gain-expected_gain) > 1e-8:
                        raise RuntimeError(
                            "velocity correction gain is not time-scaled")
                elif abs(gain) > 1e-12:
                    raise RuntimeError(
                        "inactive velocity guard has a correction gain")
                if distance(after, tuple(value+delta for value, delta in
                                         zip(before, correction))) > 1e-9:
                    raise RuntimeError("velocity correction arithmetic differs")
                expected_correction = tuple(
                    gain*(target-value) for target, value in
                    zip(logged_filtered, before))
                expected_planar_norm = norm(expected_correction[:2])
                if expected_planar_norm > \
                        args.maximum_planar_velocity_correction:
                    planar_scale = args.maximum_planar_velocity_correction / \
                        expected_planar_norm
                    expected_correction = (
                        expected_correction[0]*planar_scale,
                        expected_correction[1]*planar_scale,
                        expected_correction[2])
                expected_correction = (
                    expected_correction[0], expected_correction[1],
                    max(-args.maximum_vertical_velocity_correction,
                        min(args.maximum_vertical_velocity_correction,
                            expected_correction[2])))
                if have_receiver_velocity_schema:
                    planar_acceleration = \
                        args.recovery_tracking_planar_acceleration if \
                        recovery_tracking else \
                        args.maximum_planar_velocity_acceleration
                    vertical_acceleration = \
                        args.recovery_tracking_vertical_acceleration if \
                        recovery_tracking else \
                        args.maximum_vertical_velocity_acceleration
                    planar_limit = min(
                        args.maximum_planar_velocity_correction,
                        planar_acceleration*dt)
                    expected_planar_norm = norm(expected_correction[:2])
                    if expected_planar_norm > planar_limit:
                        planar_scale = planar_limit / expected_planar_norm
                        expected_correction = (
                            expected_correction[0]*planar_scale,
                            expected_correction[1]*planar_scale,
                            expected_correction[2])
                    vertical_limit = min(
                        args.maximum_vertical_velocity_correction,
                        vertical_acceleration*dt)
                    expected_correction = (
                        expected_correction[0], expected_correction[1],
                        max(-vertical_limit, min(
                            vertical_limit, expected_correction[2])))
                if distance(expected_correction, correction) > 1e-8:
                    raise RuntimeError(
                        "velocity correction gain/limits are inconsistent")
                planar_correction = norm(correction[:2])
                maximum_velocity_correction = max(
                    maximum_velocity_correction,
                    norm(correction))
                if planar_correction > \
                        args.maximum_planar_velocity_correction+1e-8 or \
                        abs(correction[2]) > \
                        args.maximum_vertical_velocity_correction+1e-8:
                    raise RuntimeError("velocity correction exceeded its bound")
                error_before = distance(before, logged_filtered)
                error_after = distance(after, logged_filtered)
                if abs(error_before-float(row["velocity_error_mps"])) > 1e-8:
                    raise RuntimeError("velocity error log is inconsistent")
                if error_after > error_before+1e-10:
                    raise RuntimeError("velocity guard increased disagreement")
                applied = int(row["correction_applied"]) == 1
                if applied != (norm(correction) > 1e-12):
                    raise RuntimeError("velocity correction flag differs")
                if applied and int(row["active"]) != 1:
                    raise RuntimeError("inactive velocity guard changed LIO")
                if decision == "activated" and first_activation is None:
                    first_activation = int(row["keyframe_id"])
                maximum_velocity_error = max(
                    maximum_velocity_error, error_before)
                if have_emergency_restart_schema:
                    restart_changed = int(
                        row["emergency_restart_state_changed"]) == 1
                    restart_required = int(
                        row["emergency_restart_required"]) == 1
                    evidence = (not emergency_completed and
                                row["regime"] ==
                                "relocalization_required" and
                                int(row["active"]) == 1 and
                                velocity_source == "receiver_twist" and
                                error_after+1e-10 >=
                                args.emergency_restart_velocity_error)
                    emergency_evidence = emergency_evidence+1 \
                        if evidence else 0
                    expected_changed = (not emergency_completed and
                                        emergency_evidence >=
                                        args.emergency_restart_observations)
                    if restart_changed != expected_changed or \
                            restart_required != expected_changed or \
                            int(row["emergency_restart_evidence"]) != \
                            emergency_evidence:
                        raise RuntimeError(
                            "velocity emergency-restart latch is inconsistent")
                    if restart_changed:
                        velocity_restart_transition_ids.add(row_id)
                        emergency_completed = True
                        emergency_evidence = 0
            guard_dt_median = statistics.median(guard_intervals) \
                if guard_intervals else 0.0
            print(f"RTK velocity guard: updates={len(guard_rows)}, "
                  f"median_dt={guard_dt_median:.3f}s, "
                  f"decisions={guard_counts}, "
                  f"sources={velocity_source_counts}, "
                  f"first_activation_kf={first_activation}, "
                  f"max_error={maximum_velocity_error:.3f}m/s, "
                  f"max_correction={maximum_velocity_correction:.3f}m/s, "
                  f"recovery_tracking_updates={recovery_tracking_updates}, "
                  f"emergency_restart_kf="
                  f"{min(velocity_restart_transition_ids) if velocity_restart_transition_ids else None}")

        if args.regularized_field.is_file():
            regularized_fields, regularized_rows = read(
                args.regularized_field)
            required_regularized = {
                "keyframe_id", "timestamp", "segment_id",
                "recovery_segment", "decision", "distance_m",
                "knot_count", "elastic_distance_m", "elastic_stiffness",
                "radial_force_proxy_m",
                "correction_x", "correction_y", "correction_z",
                "correction_yaw_deg", "gradient_m_per_m",
                "yaw_gradient_deg_per_m", "curvature_per_m",
                "residual_after_m", "elastic_probation",
                "position_observation",
            }
            if not required_regularized.issubset(regularized_fields):
                raise RuntimeError(
                    "regularized correction-field CSV is incomplete")
            regularized_ids = [
                int(row["keyframe_id"]) for row in regularized_rows]
            if not regularized_rows or any(
                    current <= previous for previous, current in
                    zip(regularized_ids, regularized_ids[1:])):
                raise RuntimeError(
                    "regularized correction-field IDs are empty, duplicate "
                    "or unordered")
            valid_rtk_ids = [
                int(row["keyframe_id"]) for row in decisions
                if row["measurement_x"]]
            if regularized_ids != valid_rtk_ids:
                raise RuntimeError(
                    "regularized field/keyframe RTK cadence differs")
            position_ids = [
                int(row["keyframe_id"]) for row in regularized_rows
                if int(row["position_observation"]) == 1]
            if guard_rows and position_ids != [
                    int(row["keyframe_id"]) for row in guard_rows]:
                raise RuntimeError(
                    "regularized position/velocity-guard cadence differs")

            regularized_counts = {}
            segment_knots = {}
            maximum_regularized_gradient = 0.0
            maximum_regularized_yaw_gradient = 0.0
            maximum_regularized_curvature = 0.0
            maximum_regularized_residual = 0.0
            for row in regularized_rows:
                decision = row["decision"]
                if decision not in {
                        "accepted", "status_rejected", "spacing_rejected",
                        "position_hold", "alignment_rejected",
                        "translation_fallback", "frozen_quarantine",
                        "provisional_recovery"}:
                    raise RuntimeError(
                        "unknown regularized correction-field decision")
                regularized_counts[decision] = \
                    regularized_counts.get(decision, 0)+1
                segment_id = int(row["segment_id"])
                position_observation = int(row["position_observation"])
                if position_observation not in (0, 1):
                    raise RuntimeError(
                        "regularized position-observation flag is not binary")
                knot_count = int(row["knot_count"])
                previous_knots = segment_knots.get(segment_id, 0)
                if decision in {"accepted", "alignment_rejected",
                                "translation_fallback"}:
                    if knot_count != previous_knots+1:
                        raise RuntimeError(
                            "regularized correction-field knot count differs")
                    segment_knots[segment_id] = knot_count
                elif decision == "provisional_recovery":
                    if int(row["recovery_segment"]) != 1 or knot_count != 0:
                        raise RuntimeError(
                            "provisional elastic row escaped quarantine")
                elif decision == "frozen_quarantine":
                    if int(row["recovery_segment"]) != 0 or knot_count != 0:
                        raise RuntimeError(
                            "frozen initial segment changed its field")
                elif knot_count != previous_knots:
                    raise RuntimeError(
                        "rejected regularized observation changed its field")
                stiffness = float(row["elastic_stiffness"])
                if decision in {"accepted", "alignment_rejected",
                                "translation_fallback"}:
                    elastic_distance = float(row["elastic_distance_m"])
                    if elastic_distance <= args.regularized_elastic_soft_radius:
                        expected_stiffness = \
                            args.regularized_elastic_minimum_stiffness
                    elif elastic_distance >= \
                            args.regularized_elastic_full_radius:
                        expected_stiffness = \
                            args.regularized_elastic_maximum_stiffness
                    else:
                        x = (elastic_distance-
                             args.regularized_elastic_soft_radius) / \
                            (args.regularized_elastic_full_radius-
                             args.regularized_elastic_soft_radius)
                        smooth = x*x*x*(10+x*(-15+6*x))
                        expected_stiffness = \
                            args.regularized_elastic_minimum_stiffness + \
                            (args.regularized_elastic_maximum_stiffness-
                             args.regularized_elastic_minimum_stiffness)*smooth
                    if abs(stiffness-expected_stiffness) > 1e-9:
                        raise RuntimeError(
                            "production radial elastic stiffness is inconsistent")
                    force_proxy = float(row["radial_force_proxy_m"])
                    if abs(force_proxy-stiffness*elastic_distance) > 1e-9:
                        raise RuntimeError(
                            "production radial elastic force proxy is inconsistent")
                numeric = [float(row[name]) for name in (
                    "distance_m", "elastic_distance_m", "elastic_stiffness",
                    "radial_force_proxy_m",
                    "correction_x", "correction_y", "correction_z",
                    "correction_yaw_deg", "gradient_m_per_m",
                    "yaw_gradient_deg_per_m", "curvature_per_m",
                    "residual_after_m")]
                if any(not math.isfinite(value) for value in numeric):
                    raise RuntimeError(
                        "regularized correction-field contains non-finite data")
                if any(value < -1e-12 for value in (
                        numeric[0], numeric[1], numeric[2], numeric[3],
                        numeric[8], numeric[9], numeric[10], numeric[11])):
                    raise RuntimeError(
                        "regularized correction-field magnitude is negative")
                maximum_regularized_gradient = max(
                    maximum_regularized_gradient, numeric[8])
                maximum_regularized_yaw_gradient = max(
                    maximum_regularized_yaw_gradient, numeric[9])
                maximum_regularized_curvature = max(
                    maximum_regularized_curvature, numeric[10])
                maximum_regularized_residual = max(
                    maximum_regularized_residual, numeric[11])
            print(
                "production C2 elastic field: "
                f"decisions={regularized_counts}, "
                f"segments={len(segment_knots)}, "
                f"knots={sum(segment_knots.values())}, "
                f"position_knots={len(position_ids)}, "
                f"max_gradient={maximum_regularized_gradient:.4f}m/m, "
                f"yaw_gradient={maximum_regularized_yaw_gradient:.4f}deg/m, "
                f"curvature={maximum_regularized_curvature:.5f}/m, "
                f"observation_residual_max={maximum_regularized_residual:.3f}m")
        segment_rows = []
        if args.relocalization_monitor.is_file():
            segment_by_trigger = {}
            if args.frontend_segments.is_file():
                segment_fields, segment_rows = read(args.frontend_segments)
                required_segment = {
                    "segment_id", "trigger_keyframe_id",
                    "request_timestamp", "restart_timestamp", "reason",
                    "evidence_span_m", "seed_points", "deleted_voxels",
                    "map_voxels_after", "pose_delta_m",
                    "rotation_delta_deg", "velocity_delta_mps",
                    "bias_g_delta", "bias_a_delta", "gravity_delta",
                    "gate_acknowledged",
                }
                if not required_segment.issubset(segment_fields):
                    raise RuntimeError("frontend-segment CSV is incomplete")
                recovery_segment_fields = {
                    "restart_kind", "target_velocity_x",
                    "target_velocity_y", "target_velocity_z",
                    "rtk_anchor_x", "rtk_anchor_y", "rtk_anchor_z",
                }
                have_recovery_segment_schema = \
                    recovery_segment_fields.issubset(segment_fields)
                if recovery_segment_fields.intersection(segment_fields) and \
                        not have_recovery_segment_schema:
                    raise RuntimeError(
                        "frontend recovery-segment audit is partial")
                for expected_id, segment in enumerate(segment_rows, start=1):
                    segment_id = int(segment["segment_id"])
                    trigger_id = int(segment["trigger_keyframe_id"])
                    if segment_id != expected_id or \
                            trigger_id in segment_by_trigger:
                        raise RuntimeError(
                            "frontend segment IDs are not unique/contiguous")
                    if float(segment["restart_timestamp"]) + 1e-9 < \
                            float(segment["request_timestamp"]):
                        raise RuntimeError(
                            "frontend segment predates its request")
                    if int(segment["seed_points"]) < \
                            args.frontend_segment_minimum_seed_points or \
                            int(segment["deleted_voxels"]) <= 0 or \
                            int(segment["map_voxels_after"]) <= 0:
                        raise RuntimeError(
                            "frontend segment was not safely seeded")
                    restart_kind = segment.get(
                        "restart_kind", "structural")
                    if restart_kind not in {"structural", "velocity"}:
                        raise RuntimeError(
                            "frontend segment restart kind is unknown")
                    preserved = (
                        "pose_delta_m", "rotation_delta_deg",
                        "bias_g_delta", "bias_a_delta", "gravity_delta")
                    if any(abs(float(segment[name])) > 1e-10
                           for name in preserved):
                        raise RuntimeError(
                            "frontend segment changed the continuous state")
                    if restart_kind == "structural" and \
                            abs(float(segment["velocity_delta_mps"])) > 1e-10:
                        raise RuntimeError(
                            "structural frontend segment changed velocity")
                    if restart_kind == "velocity":
                        if not have_recovery_segment_schema or \
                                segment["reason"] != "velocity_divergence":
                            raise RuntimeError(
                                "velocity recovery segment lacks its audit")
                        guard = next((row for row in guard_rows
                                      if int(row["keyframe_id"]) ==
                                      trigger_id), None)
                        if guard is None or trigger_id not in \
                                velocity_restart_transition_ids:
                            raise RuntimeError(
                                "velocity segment lacks a guard transition")
                        target = vector(segment, (
                            "target_velocity_x", "target_velocity_y",
                            "target_velocity_z"))
                        filtered = vector(guard, (
                            "filtered_rtk_vx", "filtered_rtk_vy",
                            "filtered_rtk_vz"))
                        before_reset = vector(guard, (
                            "lio_after_vx", "lio_after_vy",
                            "lio_after_vz"))
                        if distance(target, filtered) > 1e-8 or abs(
                                float(segment["velocity_delta_mps"])-
                                distance(target, before_reset)) > 1e-8:
                            raise RuntimeError(
                                "frontend propagation velocity reset differs")
                        anchor = vector(segment, (
                            "rtk_anchor_x", "rtk_anchor_y", "rtk_anchor_z"))
                        measurement = vector(decisions_by_id[trigger_id], (
                            "measurement_x", "measurement_y",
                            "measurement_z"))
                        if distance(anchor, measurement) > 1e-8:
                            raise RuntimeError(
                                "emergency global anchor differs from RTK")
                        start_id = trigger_id+1
                        if start_id < len(global_positions):
                            local_increment = distance(
                                local_positions[start_id],
                                local_positions[trigger_id])
                            global_increment = distance(
                                global_positions[start_id],
                                global_positions[trigger_id])
                            if args.regularized_field.is_file():
                                allowed_increment_change = local_increment * \
                                    math.hypot(
                                        args.maximum_regularized_planar_gradient,
                                        args.maximum_regularized_vertical_gradient)
                                if abs(global_increment-local_increment) > \
                                        allowed_increment_change+1e-7:
                                    raise RuntimeError(
                                        "quarantined recovery boundary exceeded "
                                        "the production elastic-gradient bound")
                            elif abs(
                                    distance(global_positions[start_id], anchor)-
                                    local_increment) > 1e-7:
                                raise RuntimeError(
                                    "emergency global segment distorted its "
                                    "first local increment")
                            if map_eligibility[start_id] != 0:
                                raise RuntimeError(
                                    "unvalidated recovery segment escaped "
                                    "global-map quarantine")
                    if int(segment["gate_acknowledged"]) != 1:
                        raise RuntimeError(
                            "frontend segment did not restart its gate")
                    segment_by_trigger[trigger_id] = segment
            supervisor_rows = []
            if args.frontend_restart_supervisor.is_file():
                supervisor_fields, supervisor_rows = read(
                    args.frontend_restart_supervisor)
                required_supervisor = {
                    "request_id", "trigger_keyframe_id", "timestamp",
                    "action", "reason", "evidence_span_m",
                    "geometry_failure", "condition_ratio",
                    "similarity_scale", "similarity_rms_m",
                }
                if not required_supervisor.issubset(supervisor_fields):
                    raise RuntimeError(
                        "frontend-restart supervisor CSV is incomplete")
                for expected_id, event in enumerate(
                        supervisor_rows, start=1):
                    if int(event["request_id"]) != expected_id:
                        raise RuntimeError(
                            "frontend-restart request IDs are not contiguous")
                    if event["action"] not in {"scheduled", "suppressed"}:
                        raise RuntimeError(
                            "frontend-restart supervisor action is unknown")
            reloc_fields, reloc_rows = read(args.relocalization_monitor)
            required_reloc = {
                "keyframe_id", "timestamp", "regime", "decision",
                "guard_active", "post_velocity_error_mps", "window_size",
                "local_path_length_m", "rtk_path_length_m",
                "path_scale_ratio", "planar_rms_m", "vertical_rms_m",
                "transform_x", "transform_y", "transform_z",
                "transform_yaw_deg", "ready", "state_changed",
                "cumulative_distance_m", "frontend_restart_required",
                "restart_state_changed",
                "consecutive_structural_rejections",
                "structural_rejection_span_m", "restart_reason",
            }
            if not required_reloc.issubset(reloc_fields):
                raise RuntimeError("rigid-relocalization CSV is incomplete")
            diagnostic_reloc = {
                "full_rotation_observable", "geometry_condition_ratio",
                "se3_nonyaw_rotation_deg", "se3_rms_m",
                "similarity_scale", "similarity_rms_m",
                "geometry_failure",
            }
            have_geometry_diagnostics = diagnostic_reloc.issubset(
                reloc_fields)
            if diagnostic_reloc.intersection(reloc_fields) and not \
                    have_geometry_diagnostics:
                raise RuntimeError(
                    "rigid-relocalization geometry diagnostics are partial")
            if len(reloc_rows) != len(observations):
                raise RuntimeError(
                    "relocalization/selected observation cadence differs")
            reloc_ids = [int(row["keyframe_id"]) for row in reloc_rows]
            observation_ids_ordered = [int(row["keyframe_id"])
                                       for row in observations]
            if reloc_ids != observation_ids_ordered:
                raise RuntimeError("relocalization observation IDs differ")
            if monitor_rows and any(
                    reloc["regime"] != monitor["regime"]
                    for reloc, monitor in zip(reloc_rows, monitor_rows)):
                raise RuntimeError(
                    "monitor and relocalization regimes differ")
            if not guard_rows:
                raise RuntimeError(
                    "relocalization audit requires velocity-guard rows")

            reloc_counts = {}
            first_ready = None
            first_restart = None
            maximum_window_path = 0.0
            minimum_tested_scale = math.inf
            maximum_tested_scale = -math.inf
            minimum_diagnostic_scale = math.inf
            maximum_diagnostic_scale = -math.inf
            minimum_condition_ratio = math.inf
            maximum_condition_ratio = -math.inf
            geometry_failure_counts = {}
            ready_latched = False
            restart_latched = False
            structural_rejections = 0
            first_structural_distance = 0.0
            restart_reason = "monitoring"
            guard_cursor = -1
            previous_reloc_id = None
            for row in reloc_rows:
                row_id = int(row["keyframe_id"])
                if previous_reloc_id in segment_by_trigger:
                    restart_latched = False
                    structural_rejections = 0
                    first_structural_distance = 0.0
                    restart_reason = "monitoring"
                    ready_latched = False
                restart_transition_this_row = False
                while guard_cursor+1 < len(guard_rows) and int(
                        guard_rows[guard_cursor+1]["keyframe_id"]) <= row_id:
                    guard_cursor += 1
                guard = guard_rows[guard_cursor] \
                    if guard_cursor >= 0 else None
                decision = row["decision"]
                reloc_counts[decision] = reloc_counts.get(decision, 0)+1
                if decision not in {
                        "monitoring", "waiting_velocity",
                        "collecting_baseline", "scale_mismatch",
                        "fit_rejected", "candidate", "ready",
                        "restart_required"}:
                    raise RuntimeError("unknown relocalization decision")
                evidence_age = math.inf if guard is None else \
                    float(row["timestamp"])-float(guard["timestamp"])
                evidence_fresh = guard is not None and \
                    evidence_age >= -1e-9 and evidence_age <= \
                    args.velocity_evidence_max_age+1e-9
                expected_active = evidence_fresh and \
                    int(guard["active"]) == 1
                if (int(row["guard_active"]) == 1) != expected_active:
                    raise RuntimeError(
                        "relocalization velocity-guard state differs")
                baseline_available = evidence_fresh and \
                    int(guard["baseline_available"]) == 1
                if baseline_available:
                    after = vector(guard, (
                        "lio_after_vx", "lio_after_vy", "lio_after_vz"))
                    filtered = vector(guard, (
                        "filtered_rtk_vx", "filtered_rtk_vy",
                        "filtered_rtk_vz"))
                    expected_error = distance(after, filtered)
                else:
                    expected_error = 0.0
                if abs(expected_error-
                       float(row["post_velocity_error_mps"])) > 1e-8:
                    raise RuntimeError(
                        "relocalization post-velocity error differs")

                window_size = int(row["window_size"])
                local_path = float(row["local_path_length_m"])
                rtk_path = float(row["rtk_path_length_m"])
                scale = float(row["path_scale_ratio"])
                planar_rms = float(row["planar_rms_m"])
                vertical_rms = float(row["vertical_rms_m"])
                cumulative_distance = float(row["cumulative_distance_m"])
                maximum_window_path = max(maximum_window_path, local_path)
                if local_path > 1e-12:
                    if abs(scale-rtk_path/local_path) > 1e-8:
                        raise RuntimeError(
                            "relocalization path-scale ratio differs")
                    if min(local_path, rtk_path) >= \
                            args.relocalization_minimum_path:
                        minimum_tested_scale = min(minimum_tested_scale, scale)
                        maximum_tested_scale = max(maximum_tested_scale, scale)

                regime = row["regime"]
                velocity_stable = baseline_available and expected_error <= \
                    args.relocalization_velocity_error+1e-10
                enough_baseline = window_size >= \
                    args.relocalization_minimum_observations and min(
                        local_path, rtk_path) >= \
                    args.relocalization_minimum_path-1e-8
                scale_valid = abs(scale-1.0) <= \
                    args.relocalization_scale_error+1e-10
                fit_valid = planar_rms <= \
                    args.relocalization_planar_rms+1e-10 and \
                    vertical_rms <= args.relocalization_vertical_rms+1e-10
                structural_failure = velocity_stable and enough_baseline and (
                    not scale_valid or not fit_valid)
                if have_geometry_diagnostics:
                    condition_ratio = float(
                        row["geometry_condition_ratio"])
                    nonyaw_rotation_deg = float(
                        row["se3_nonyaw_rotation_deg"])
                    se3_rms = float(row["se3_rms_m"])
                    similarity_scale = float(row["similarity_scale"])
                    similarity_rms = float(row["similarity_rms_m"])
                    diagnostic_values = (
                        condition_ratio, nonyaw_rotation_deg, se3_rms,
                        similarity_scale, similarity_rms)
                    if any(not math.isfinite(value)
                           for value in diagnostic_values) or \
                            condition_ratio < 0.0 or similarity_scale <= 0.0:
                        raise RuntimeError(
                            "relocalization geometry diagnostic is invalid")
                    observable = condition_ratio >= \
                        args.diagnostic_minimum_condition_ratio
                    if (int(row["full_rotation_observable"]) == 1) != \
                            observable:
                        raise RuntimeError(
                            "relocalization observability flag differs")
                    if velocity_stable and enough_baseline and \
                            similarity_rms > se3_rms+1e-8:
                        raise RuntimeError(
                            "diagnostic similarity fit is worse than SE(3)")
                    geometry_failure = row["geometry_failure"]
                    if structural_failure:
                        similarity_explains = similarity_rms <= \
                            args.diagnostic_maximum_similarity_rms
                        has_scale_error = abs(similarity_scale-1.0) >= \
                            args.diagnostic_minimum_scale_error
                        has_nonyaw_rotation = observable and \
                            nonyaw_rotation_deg >= \
                            args.diagnostic_minimum_nonyaw_rotation_deg
                        if not similarity_explains:
                            expected_failure = "nonrigid"
                        elif has_scale_error and not observable:
                            expected_failure = \
                                "systematic_scale_weak_geometry"
                        elif has_scale_error and has_nonyaw_rotation:
                            expected_failure = "scale_and_attitude"
                        elif has_scale_error:
                            expected_failure = "systematic_scale"
                        elif not observable:
                            expected_failure = "weak_geometry"
                        elif has_nonyaw_rotation:
                            expected_failure = "attitude_misalignment"
                        else:
                            expected_failure = "nonrigid"
                        if geometry_failure != expected_failure:
                            raise RuntimeError(
                                "relocalization geometry classification "
                                f"differs at KF {row_id}")
                        geometry_failure_counts[geometry_failure] = \
                            geometry_failure_counts.get(
                                geometry_failure, 0)+1
                        minimum_diagnostic_scale = min(
                            minimum_diagnostic_scale, similarity_scale)
                        maximum_diagnostic_scale = max(
                            maximum_diagnostic_scale, similarity_scale)
                        minimum_condition_ratio = min(
                            minimum_condition_ratio, condition_ratio)
                        maximum_condition_ratio = max(
                            maximum_condition_ratio, condition_ratio)
                    elif geometry_failure != "none":
                        raise RuntimeError(
                            "non-structural row reports a geometry failure")
                if decision == "monitoring" and \
                        regime == "relocalization_required":
                    raise RuntimeError(
                        "relocalization trigger remained in monitoring")
                if decision == "waiting_velocity" and (
                        velocity_stable or window_size != 0):
                    raise RuntimeError(
                        "velocity wait gate is inconsistent")
                if decision == "collecting_baseline" and (
                        not velocity_stable or enough_baseline):
                    raise RuntimeError(
                        "relocalization baseline gate is inconsistent")
                if decision == "scale_mismatch" and (
                        not enough_baseline or scale_valid):
                    raise RuntimeError(
                        "relocalization scale gate is inconsistent")
                if decision == "fit_rejected" and (
                        not enough_baseline or not scale_valid or fit_valid):
                    raise RuntimeError(
                        "relocalization fit gate is inconsistent")
                if decision in {"candidate", "ready"} and (
                        not enough_baseline or not scale_valid or
                        not fit_valid):
                    raise RuntimeError(
                        "invalid rigid 4DOF candidate was accepted")

                row_restart = int(
                    row["frontend_restart_required"]) == 1
                row_restart_changed = int(row["restart_state_changed"]) == 1
                if not restart_latched:
                    if structural_failure:
                        if structural_rejections == 0:
                            first_structural_distance = cumulative_distance
                        structural_rejections += 1
                        restart_reason = "scale_mismatch" if not \
                            scale_valid else "fit_rejected"
                        rejection_span = (cumulative_distance-
                                          first_structural_distance)
                        if (structural_rejections >=
                                args.restart_structural_rejections and
                                rejection_span >=
                                args.restart_minimum_rejection_span-1e-8):
                            restart_latched = True
                            restart_transition_this_row = True
                            if first_restart is None:
                                first_restart = row_id
                    else:
                        structural_rejections = 0
                        first_structural_distance = 0.0
                        restart_reason = "monitoring"
                        rejection_span = 0.0
                else:
                    if structural_failure:
                        structural_rejections += 1
                        restart_reason = "scale_mismatch" if not \
                            scale_valid else "fit_rejected"
                    rejection_span = (cumulative_distance-
                                      first_structural_distance)
                if int(row["consecutive_structural_rejections"]) != \
                        structural_rejections:
                    raise RuntimeError(
                        f"structural-rejection counter is inconsistent at "
                        f"KF {row_id}: logged="
                        f"{row['consecutive_structural_rejections']}, "
                        f"expected={structural_rejections}")
                if abs(float(row["structural_rejection_span_m"])-
                       rejection_span) > 1e-8:
                    raise RuntimeError(
                        "structural-rejection span is inconsistent")
                if row_restart != restart_latched:
                    raise RuntimeError(
                        "frontend restart request latch is inconsistent")
                if row_restart_changed != restart_transition_this_row:
                    raise RuntimeError(
                        "frontend restart transition is inconsistent")
                if restart_latched:
                    if decision != "restart_required":
                        raise RuntimeError(
                            "latched frontend restart was not enforced")
                    if row["restart_reason"] not in {
                            "scale_mismatch", "fit_rejected"}:
                        raise RuntimeError(
                            "frontend restart lacks a structural reason")
                    if row["restart_reason"] != restart_reason:
                        raise RuntimeError(
                            f"frontend restart reason is inconsistent at "
                            f"KF {row_id}")
                elif decision == "restart_required":
                    raise RuntimeError(
                        "frontend restart decision was not latched")

                row_ready = int(row["ready"]) == 1
                if row_ready and not ready_latched:
                    ready_latched = True
                    first_ready = int(row["keyframe_id"])
                    if int(row["state_changed"]) != 1:
                        raise RuntimeError(
                            "relocalization ready transition was not marked")
                elif int(row["state_changed"]) != 0:
                    raise RuntimeError(
                        "duplicate relocalization state transition")
                if ready_latched and not row_ready:
                    raise RuntimeError("relocalization readiness was not latched")
                previous_reloc_id = row_id
            restart_transition_ids = {
                int(row["keyframe_id"]) for row in reloc_rows
                if int(row["restart_state_changed"]) == 1}
            all_restart_transition_ids = restart_transition_ids | \
                velocity_restart_transition_ids
            if set(segment_by_trigger) - all_restart_transition_ids:
                raise RuntimeError(
                    "frontend segment lacks a matching restart request")
            for trigger_id, segment in segment_by_trigger.items():
                if segment.get("restart_kind", "structural") == "velocity":
                    continue
                reloc = next(row for row in reloc_rows
                             if int(row["keyframe_id"]) == trigger_id)
                if abs(float(segment["request_timestamp"])-
                       float(reloc["timestamp"])) > 1e-8 or \
                        segment["reason"] != reloc["restart_reason"] or \
                        abs(float(segment["evidence_span_m"])-float(
                            reloc["structural_rejection_span_m"])) > 1e-8:
                    raise RuntimeError(
                        "frontend segment/request evidence differs")
            if args.frontend_restart_supervisor.is_file():
                supervisor_by_trigger = {
                    int(row["trigger_keyframe_id"]): row
                    for row in supervisor_rows}
                if (len(supervisor_by_trigger) != len(supervisor_rows) or
                        set(supervisor_by_trigger) !=
                        all_restart_transition_ids):
                    raise RuntimeError(
                        "frontend-restart supervisor/request transitions "
                        "differ")
                scheduled_triggers = {
                    trigger for trigger, event in supervisor_by_trigger.items()
                    if event["action"] == "scheduled"}
                suppressed_triggers = {
                    trigger for trigger, event in supervisor_by_trigger.items()
                    if event["action"] == "suppressed"}
                if len(scheduled_triggers) > \
                        args.maximum_automatic_restarts:
                    raise RuntimeError(
                        "automatic frontend restarts exceeded the limit")
                if scheduled_triggers != set(segment_by_trigger):
                    raise RuntimeError(
                        "scheduled frontend restart was not executed")
                if suppressed_triggers.intersection(segment_by_trigger):
                    raise RuntimeError(
                        "suppressed frontend restart was executed")
                have_suppressed = False
                reloc_by_id = {
                    int(row["keyframe_id"]): row for row in reloc_rows}
                for event in supervisor_rows:
                    trigger_id = int(event["trigger_keyframe_id"])
                    if event["action"] == "suppressed":
                        have_suppressed = True
                    elif have_suppressed:
                        raise RuntimeError(
                            "automatic restart resumed after suppression")
                    if event["reason"] == "velocity_divergence":
                        if trigger_id not in velocity_restart_transition_ids or \
                                event["geometry_failure"] != \
                                "velocity_divergence":
                            raise RuntimeError(
                                "velocity restart-supervisor evidence differs")
                        continue
                    reloc = reloc_by_id[trigger_id]
                    if abs(float(event["timestamp"])-
                           float(reloc["timestamp"])) > 1e-8 or \
                            event["reason"] != reloc["restart_reason"] or \
                            abs(float(event["evidence_span_m"])-float(
                                reloc["structural_rejection_span_m"])) > 1e-8:
                        raise RuntimeError(
                            "restart-supervisor evidence differs")
                    if have_geometry_diagnostics:
                        diagnostic_pairs = (
                            ("condition_ratio",
                             "geometry_condition_ratio"),
                            ("similarity_scale", "similarity_scale"),
                            ("similarity_rms_m", "similarity_rms_m"))
                        if event["geometry_failure"] != \
                                reloc["geometry_failure"] or any(
                                    abs(float(event[left])-
                                        float(reloc[right])) > 1e-8
                                    for left, right in diagnostic_pairs):
                            raise RuntimeError(
                                "restart-supervisor geometry evidence differs")
            tested_scale = "n/a" if minimum_tested_scale == math.inf else \
                f"{minimum_tested_scale:.3f}..{maximum_tested_scale:.3f}"
            print(f"rigid 4DOF relocalization gate: decisions={reloc_counts}, "
                  f"first_ready_kf={first_ready}, "
                  f"first_restart_kf={first_restart}, "
                  f"max_window_path={maximum_window_path:.1f}m, "
                  f"tested_scale={tested_scale}")
            if have_geometry_diagnostics and geometry_failure_counts:
                print(
                    "relocalization geometry diagnosis: "
                    f"failures={geometry_failure_counts}, "
                    f"condition={minimum_condition_ratio:.6f}.."
                    f"{maximum_condition_ratio:.6f}, similarity_scale="
                    f"{minimum_diagnostic_scale:.4f}.."
                    f"{maximum_diagnostic_scale:.4f}")
            if segment_rows:
                print(f"frontend local segments: count={len(segment_rows)}, "
                      f"triggers={sorted(segment_by_trigger)}, "
                      f"seed_points={min(int(row['seed_points']) for row in segment_rows)}.."
                      f"{max(int(row['seed_points']) for row in segment_rows)}")
            if supervisor_rows:
                supervisor_counts = {}
                for event in supervisor_rows:
                    supervisor_counts[event["action"]] = \
                        supervisor_counts.get(event["action"], 0)+1
                print("frontend restart supervisor: "
                      f"decisions={supervisor_counts}, "
                      f"limit={args.maximum_automatic_restarts}")
        if args.recovery_monitor.is_file():
            recovery_fields, recovery_rows = read(args.recovery_monitor)
            required_recovery = {
                "keyframe_id", "timestamp", "decision", "window_size",
                "window_start_keyframe_id", "local_path_length_m",
                "rtk_path_length_m", "path_scale_ratio", "planar_rms_m",
                "vertical_rms_m", "transform_x", "transform_y",
                "transform_z", "transform_yaw_deg", "ready",
                "state_changed", "post_velocity_error_mps",
            }
            if not required_recovery.issubset(recovery_fields):
                raise RuntimeError("recovery-relocalization CSV is incomplete")
            velocity_segments = [
                row for row in segment_rows
                if row.get("restart_kind", "structural") == "velocity"]
            if recovery_rows and not velocity_segments:
                raise RuntimeError(
                    "recovery monitor ran without a dynamic segment")
            if velocity_segments:
                trigger_id = int(velocity_segments[-1][
                    "trigger_keyframe_id"])
                expected_ids = [
                    int(row["keyframe_id"]) for row in guard_rows
                    if int(row["keyframe_id"]) > trigger_id]
                recovery_ids = [int(row["keyframe_id"])
                                for row in recovery_rows]
                if recovery_ids != expected_ids[:len(recovery_ids)]:
                    raise RuntimeError(
                        "recovery monitor did not follow the 1 Hz guard")
            ready_rows = [row for row in recovery_rows
                          if int(row["state_changed"]) == 1]
            if len(ready_rows) > 1 or any(
                    int(row["ready"]) != 1 for row in ready_rows):
                raise RuntimeError(
                    "recovery reanchor transition is inconsistent")
            if velocity_segments and have_recovery_tracking_schema:
                trigger_id = int(velocity_segments[-1][
                    "trigger_keyframe_id"])
                for guard in guard_rows:
                    guard_id = int(guard["keyframe_id"])
                    if guard_id <= trigger_id:
                        continue
                    # Rigid-ready starts an elastic probation; it no longer
                    # releases the velocity support that made the fit valid.
                    expected_tracking = True
                    if (int(guard["recovery_tracking"]) == 1) != \
                            expected_tracking:
                        raise RuntimeError(
                            "recovery velocity tracking ended before "
                            "elastic-segment acceptance")
            recovery_counts = {}
            for row in recovery_rows:
                recovery_counts[row["decision"]] = \
                    recovery_counts.get(row["decision"], 0)+1
            if ready_rows:
                fit = ready_rows[0]
                start_id = int(fit["window_start_keyframe_id"])
                end_id = int(fit["keyframe_id"])
                if start_id <= 0 or start_id > end_id:
                    raise RuntimeError(
                        "recovery fit window boundary is invalid")
                yaw = math.radians(float(fit["transform_yaw_deg"]))
                cosine, sine = math.cos(yaw), math.sin(yaw)
                translation = vector(
                    fit, ("transform_x", "transform_y", "transform_z"))
                if args.regularized_field.is_file():
                    field_at_ready = next((
                        row for row in regularized_rows
                        if int(row["keyframe_id"]) == end_id), None)
                    if field_at_ready is None or \
                            int(field_at_ready["recovery_segment"]) != 1 or \
                            field_at_ready["decision"] != "accepted" or \
                            int(field_at_ready["elastic_probation"]) != 1 or \
                            int(field_at_ready["position_observation"]) != 1:
                        raise RuntimeError(
                            "stable 4DOF recovery did not enter audited "
                            "elastic probation")
                    if float(field_at_ready["residual_after_m"]) > 0.30:
                        raise RuntimeError(
                            "recovery reanchor exceeded the RTK outer bound")
                else:
                    maximum_reanchor_error = 0.0
                    for index in range(start_id, len(global_positions)):
                        local = local_positions[index]
                        predicted = (
                            cosine*local[0]-sine*local[1]+translation[0],
                            sine*local[0]+cosine*local[1]+translation[1],
                            local[2]+translation[2])
                        maximum_reanchor_error = max(
                            maximum_reanchor_error,
                            distance(predicted, global_positions[index]))
                    if maximum_reanchor_error > 1e-7:
                        raise RuntimeError(
                            "stable 4DOF recovery transform was not applied")
                print("high-rate recovery 4DOF: "
                      f"decisions={recovery_counts}, "
                      f"window={start_id}..{end_id}, "
                      f"path={float(fit['local_path_length_m']):.1f}/"
                      f"{float(fit['rtk_path_length_m']):.1f}m, "
                      f"scale={float(fit['path_scale_ratio']):.4f}, "
                      f"rms={float(fit['planar_rms_m']):.3f}/"
                      f"{float(fit['vertical_rms_m']):.3f}m")
            elif recovery_rows:
                maximum_stable_path = max(
                    min(float(row["local_path_length_m"]),
                        float(row["rtk_path_length_m"]))
                    for row in recovery_rows)
                maximum_recovery_velocity_error = max(
                    float(row["post_velocity_error_mps"])
                    for row in recovery_rows)
                tested = [row for row in recovery_rows if min(
                    float(row["local_path_length_m"]),
                    float(row["rtk_path_length_m"])) >=
                    args.recovery_minimum_path]
                if maximum_stable_path < args.recovery_minimum_path:
                    blocker = "insufficient_stable_baseline"
                elif tested and all(abs(
                        float(row["path_scale_ratio"])-1.0) >
                        args.recovery_scale_error for row in tested):
                    blocker = "scale_mismatch"
                elif tested and all(
                        float(row["planar_rms_m"]) >
                        args.recovery_planar_rms or
                        float(row["vertical_rms_m"]) >
                        args.recovery_vertical_rms for row in tested):
                    blocker = "fit_rejected"
                else:
                    blocker = "awaiting_consecutive_candidate"
                print("high-rate recovery 4DOF: "
                      f"decisions={recovery_counts}, ready=None, "
                      f"max_stable_path={maximum_stable_path:.1f}m, "
                      f"max_velocity_error="
                      f"{maximum_recovery_velocity_error:.3f}m/s, "
                      f"blocker={blocker}")
        print("separated local/global RTK architecture validation passed")
        return 0

    if not factors:
        raise RuntimeError("neither legacy RTK factors nor global-layer observations exist")

    required_factor_fields = {
        "keyframe_id", "robust_kernel", "robust_delta", "sigma_x",
        "sigma_y", "sigma_z", "measurement_x", "measurement_y",
        "measurement_z", "innovation_before_x", "innovation_before_y",
        "innovation_before_z", "innovation_after_x", "innovation_after_y",
        "innovation_after_z", "optimization_time_ms",
    }
    if not required_factor_fields.issubset(factor_fields):
        raise RuntimeError(
            "factor CSV is missing " +
            str(sorted(required_factor_fields-set(factor_fields))))

    factors_by_id = {}
    accepted_decisions = []
    for index, (query, decision, keyframe, pose) in enumerate(
            zip(queries, decisions, keyframes, trajectory)):
        if (int(decision["keyframe_id"]) != index or
                int(keyframe["id"]) != index or int(pose["id"]) != index):
            raise RuntimeError(
                "RTK decision/keyframe/trajectory IDs are not contiguous")
        if abs(float(query["timestamp"])-float(keyframe["timestamp"])) > 1e-9 or \
           abs(float(decision["timestamp"])-float(keyframe["timestamp"])) > 1e-9 or \
           abs(float(pose["timestamp"])-float(keyframe["timestamp"])) > 1e-9:
            raise RuntimeError(
                f"query/decision/trajectory timestamp differs at keyframe {index}")
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
    corrections = [
        tuple(opt-raw for opt, raw in zip(
            vector(row, ("opt_tx", "opt_ty", "opt_tz")),
            vector(row, ("raw_tx", "raw_ty", "raw_tz"))))
        for row in trajectory
    ]
    local_correction_steps = [
        distance(previous, current)
        for previous, current in zip(corrections, corrections[1:])
    ]
    maximum_local_correction_step = max(local_correction_steps, default=0.0)
    if (args.maximum_trajectory_deformation is not None and
            maximum_deformation > args.maximum_trajectory_deformation):
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
          f"max={max(runtimes):.3f}ms, trajectory_deformation={maximum_deformation:.4f}m, "
          f"maximum_local_correction_step={maximum_local_correction_step:.4f}m")
    print("keyframe-driven status-gated RTK fusion validation passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, TypeError, KeyError, IndexError,
            ZeroDivisionError) as error:
        print(f"RTK fusion validation failed: {error}")
        raise SystemExit(1)
