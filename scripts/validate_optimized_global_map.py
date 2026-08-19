#!/usr/bin/env python3
"""Validate optimized-map rebuilds, final PCD, and map->odom algebra."""

import argparse
import csv
import math
from pathlib import Path


def rows(path: Path):
    if not path.is_file():
        raise RuntimeError(f"file does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames or [], list(reader)


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def qn(q):
    length = norm(q)
    if abs(length - 1.0) > 1.0e-5:
        raise RuntimeError("non-unit quaternion")
    return tuple(value / length for value in q)


def qm(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return qn((
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ))


def qc(q):
    x, y, z, w = qn(q)
    return -x, -y, -z, w


def rotate(q, p):
    x, y, z, w = qn(q)
    px, py, pz = p
    return (
        (1-2*(y*y+z*z))*px + 2*(x*y-z*w)*py + 2*(x*z+y*w)*pz,
        2*(x*y+z*w)*px + (1-2*(x*x+z*z))*py + 2*(y*z-x*w)*pz,
        2*(x*z-y*w)*px + 2*(y*z+x*w)*py + (1-2*(x*x+y*y))*pz,
    )


def compose(a, b):
    rotated = rotate(a[1], b[0])
    return tuple(x+y for x, y in zip(a[0], rotated)), qm(a[1], b[1])


def inverse(pose):
    rotation = qc(pose[1])
    return tuple(-v for v in rotate(rotation, pose[0])), rotation


def error(a, b):
    delta = compose(inverse(a), b)
    angle = math.degrees(2*math.acos(max(-1, min(1, abs(delta[1][3])))))
    return norm(delta[0]), angle


def pose(row, prefix):
    return (
        tuple(float(row[f"{prefix}_{axis}"]) for axis in ("tx", "ty", "tz")),
        qn(tuple(float(row[f"{prefix}_{axis}"])
                 for axis in ("qx", "qy", "qz", "qw"))),
    )


def pcd_point_count(path: Path):
    if not path.is_file() or path.stat().st_size < 100:
        raise RuntimeError(f"final PCD is missing or empty: {path}")
    points = None
    data = None
    with path.open("rb") as stream:
        for _ in range(100):
            line = stream.readline().decode("ascii", errors="strict").strip()
            if line.startswith("POINTS "):
                points = int(line.split()[1])
            if line.startswith("DATA "):
                data = line.split()[1]
                break
    if points is None or data not in {"binary", "binary_compressed"}:
        raise RuntimeError("PCD header is incomplete or not binary")
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--updates", type=Path,
                        default=Path("Log/backend/global_map_updates.csv"))
    parser.add_argument("--pcd", type=Path,
                        default=Path("Log/backend/optimized_global_map.pcd"))
    parser.add_argument("--trajectory", type=Path,
                        default=Path("Log/backend/optimized_trajectory.csv"))
    parser.add_argument("--keyframes", type=Path,
                        default=Path("Log/backend/keyframes.csv"))
    parser.add_argument("--maximum-build-time-ms", type=float, default=3000.0)
    args = parser.parse_args()

    fields, updates = rows(args.updates)
    trajectory_fields, trajectory = rows(args.trajectory)
    _, keyframes = rows(args.keyframes)
    required = {"revision", "reason", "keyframes", "input_points",
                "output_points", "build_time_ms", "min_x", "min_y", "min_z",
                "max_x", "max_y", "max_z"}
    if not required.issubset(fields):
        raise RuntimeError(f"map update CSV is missing {sorted(required-set(fields))}")
    if not updates or not trajectory or len(trajectory) != len(keyframes):
        raise RuntimeError("map/trajectory/keyframe logs are incomplete")
    if not {"raw_tx", "raw_qw", "opt_tx", "opt_qw"}.issubset(trajectory_fields):
        raise RuntimeError("optimized trajectory columns are incomplete")

    tiled_fields = {"build_mode", "processed_keyframes", "updated_tiles",
                    "total_tiles"}
    tiled = tiled_fields.issubset(fields)
    times = []
    build_modes = {}
    previous_keyframes = 0
    for index, row in enumerate(updates, 1):
        if int(row["revision"]) != index:
            raise RuntimeError("map revisions are not contiguous")
        keyframe_count = int(row["keyframes"])
        input_points = int(row["input_points"])
        output_points = int(row["output_points"])
        elapsed = float(row["build_time_ms"])
        bounds = [float(row[name]) for name in
                  ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")]
        if not 0 < keyframe_count <= len(keyframes):
            raise RuntimeError(f"map revision {index} has an invalid keyframe count")
        if not 0 < output_points <= input_points:
            raise RuntimeError(f"map revision {index} has invalid point accounting")
        if not math.isfinite(elapsed) or not 0 <= elapsed <= args.maximum_build_time_ms:
            raise RuntimeError(f"map revision {index} exceeded the build-time gate")
        if not all(math.isfinite(value) for value in bounds):
            raise RuntimeError(f"map revision {index} has invalid bounds")
        if any(bounds[axis] > bounds[axis+3] for axis in range(3)):
            raise RuntimeError(f"map revision {index} has reversed bounds")
        if tiled:
            mode = row["build_mode"]
            processed = int(row["processed_keyframes"])
            updated_tiles = int(row["updated_tiles"])
            total_tiles = int(row["total_tiles"])
            if mode not in {"full", "incremental"}:
                raise RuntimeError(f"map revision {index} has an invalid build mode")
            if mode == "full" and processed != keyframe_count:
                raise RuntimeError(f"full map revision {index} skipped keyframes")
            if mode == "incremental":
                if row["reason"] != "periodic":
                    raise RuntimeError("only periodic map updates may be incremental")
                if processed != keyframe_count - previous_keyframes or processed <= 0:
                    raise RuntimeError(
                        f"incremental map revision {index} processed the wrong keyframes")
            if not 0 < updated_tiles <= total_tiles:
                raise RuntimeError(f"map revision {index} has invalid tile accounting")
            build_modes[mode] = build_modes.get(mode, 0) + 1
        times.append(elapsed)
        previous_keyframes = keyframe_count
    final = updates[-1]
    if final["reason"] != "final" or int(final["keyframes"]) != len(keyframes):
        raise RuntimeError("the last map rebuild is not the complete final graph")
    pcd_points = pcd_point_count(args.pcd)
    if pcd_points != int(final["output_points"]):
        raise RuntimeError("PCD point count differs from final map diagnostics")

    latest = trajectory[-1]
    raw = pose(latest, "raw")
    optimized = pose(latest, "opt")
    map_to_odom = compose(optimized, inverse(raw))
    recomposed = compose(map_to_odom, raw)
    composition_error = error(optimized, recomposed)
    if composition_error[0] > 1.0e-9 or composition_error[1] > 1.0e-6:
        raise RuntimeError("map->odom does not compose to the optimized body pose")
    correction = error(raw, optimized)

    ordered = sorted(times)
    median = ordered[len(ordered)//2]
    print(f"optimized global map: revisions={len(updates)}, keyframes={len(keyframes)}, "
          f"points={int(final['input_points'])}->{pcd_points}")
    if tiled:
        print(f"tiled map builds: {build_modes}, final_tiles={final['total_tiles']}")
    print(f"map build time: median={median:.1f}ms, max={max(times):.1f}ms; "
          f"final_pcd={args.pcd.stat().st_size/1024/1024:.2f}MiB")
    print(f"map->odom latest correction: position={correction[0]:.4f}m, "
          f"angle={correction[1]:.4f}deg, composition_error={composition_error[0]:.3g}m")
    print("optimized global-map and map->odom validation passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, KeyError, IndexError) as error:
        print(f"optimized global-map validation failed: {error}")
        raise SystemExit(1)
