#!/usr/bin/env python3
"""Recompute camera-to-IMU from lidar-to-IMU and lidar-to-camera transforms."""

import argparse
import json
from pathlib import Path


def matrix(values):
    if len(values) != 16:
        raise ValueError("transform_matrix must contain 16 row-major values")
    return [list(map(float, values[row * 4:(row + 1) * 4])) for row in range(4)]


def multiply(left, right):
    return [[sum(left[row][k] * right[k][column] for k in range(4))
             for column in range(4)] for row in range(4)]


def rigid_inverse(transform):
    rotation = [row[:3] for row in transform[:3]]
    rotation_t = [[rotation[column][row] for column in range(3)] for row in range(3)]
    translation = [transform[row][3] for row in range(3)]
    inverse_translation = [-sum(rotation_t[row][k] * translation[k] for k in range(3))
                           for row in range(3)]
    return [rotation_t[row] + [inverse_translation[row]] for row in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def flatten(transform):
    return [value for row in transform for value in row]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("calibration", type=Path)
    parser.add_argument("--output", type=Path, help="defaults to updating the input file")
    args = parser.parse_args()

    document = json.loads(args.calibration.read_text(encoding="utf-8"))
    transforms = document["transforms"]
    imu_from_lidar = matrix(transforms["front_lidar_to_imu"]["transform_matrix"])
    camera_from_lidar = matrix(
        transforms["front_lidar_to_midrange_camera"]["transform_matrix"])
    imu_from_camera = multiply(imu_from_lidar, rigid_inverse(camera_from_lidar))
    transforms["midrange_camera_to_imu"]["transform_matrix"] = flatten(imu_from_camera)

    destination = args.output or args.calibration
    destination.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n",
                           encoding="utf-8")
    print(f"updated {destination}: midrange_camera -> imu")


if __name__ == "__main__":
    main()
