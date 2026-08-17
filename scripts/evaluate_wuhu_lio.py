#!/usr/bin/env python3
"""Report compact LIO-versus-standard-INS-odometry metrics and timing."""

import os
import sys

if sys.executable != "/usr/bin/python3":
    environment = os.environ.copy()
    for variable in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME", "PYTHONPATH"):
        environment.pop(variable, None)
    environment["PATH"] = (
        "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:"
        "/usr/sbin:/usr/bin:/sbin:/bin")
    environment["AMENT_PREFIX_PATH"] = "/opt/ros/humble"
    environment["PYTHONPATH"] = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/opt/ros/humble/local/lib/python3.10/dist-packages")
    os.execve("/usr/bin/python3", ["/usr/bin/python3", *sys.argv], environment)

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def timing_summary(name, values):
    intervals = np.diff(np.asarray(values))
    if not len(intervals):
        return f"{name}: insufficient samples"
    return (
        f"{name}: count={len(values)}, dt median={np.median(intervals):.6f}s, "
        f"p99={np.percentile(intervals, 99):.6f}s, max={np.max(intervals):.6f}s")


def metric_summary(name, errors):
    return (
        f"{name}: RMSE={np.sqrt(np.mean(errors ** 2)):.3f}m, "
        f"median={np.median(errors):.3f}m, "
        f"P95={np.percentile(errors, 95):.3f}m, "
        f"final={errors[-1]:.3f}m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag", type=Path,
        default=Path("/home/project/data/haibo/wuhu_livo/ros2bag_my_livo"))
    parser.add_argument(
        "--trajectory", type=Path,
        default=Path("/home/project/MY-LIVO1.0/Log/result/wuhu_truck29.txt"))
    parser.add_argument(
        "--rear-axle-to-imu", action="store_true",
        help="compare against the physical-IMU reference used by the matching launch switch")
    parser.add_argument(
        "--imu-to-rear-axle", type=float, nargs=3,
        default=(0.904, -4.679, -1.699), metavar=("X", "Y", "Z"),
        help="IMU-origin to rear-axle lever arm in RFU metres")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))

    imu_header_times, imu_record_times = [], []
    reference_times, reference_positions = [], []
    reference_quaternions = []
    lidar_header_times, lidar_record_times = [], []
    while reader.has_next():
        topic, data, record_time = reader.read_next()
        if topic == "imu_data":
            message = deserialize_message(data, Imu)
            imu_header_times.append(stamp_seconds(message.header.stamp))
            imu_record_times.append(record_time * 1.0e-9)
        elif topic == "imu_data/odometry":
            message = deserialize_message(data, Odometry)
            reference_times.append(stamp_seconds(message.header.stamp))
            reference_positions.append([
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ])
            reference_quaternions.append([
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ])
        elif topic == "front_lidar":
            message = deserialize_message(data, PointCloud2)
            lidar_header_times.append(stamp_seconds(message.header.stamp))
            lidar_record_times.append(record_time * 1.0e-9)

    trajectory = np.loadtxt(args.trajectory, ndmin=2)
    lio_times = trajectory[:, 0]
    lio_positions = trajectory[:, 1:4]
    lio_quaternions = trajectory[:, 4:8]
    if not reference_times:
        raise SystemExit(
            "imu_data/odometry is missing; the standard Imu topic does not "
            "contain an RTK/INS reference position")
    reference_times = np.asarray(reference_times)
    reference_positions = np.asarray(reference_positions)
    reference_quaternions = np.asarray(reference_quaternions)
    if args.rear_axle_to_imu:
        lever = np.asarray(args.imu_to_rear_axle)
        corrected_positions = []
        for position, quaternion in zip(
                reference_positions, reference_quaternions):
            x, y, z, w = quaternion / np.linalg.norm(quaternion)
            rotation = np.array([
                [1.0 - 2.0 * (y * y + z * z),
                 2.0 * (x * y - z * w),
                 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w),
                 1.0 - 2.0 * (x * x + z * z),
                 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w),
                 2.0 * (y * z + x * w),
                 1.0 - 2.0 * (x * x + y * y)],
            ])
            corrected_positions.append(position - rotation @ lever)
        reference_positions = np.asarray(corrected_positions)

    valid = (lio_times >= reference_times[0]) & (lio_times <= reference_times[-1])
    lio_times = lio_times[valid]
    lio_positions = lio_positions[valid]
    lio_quaternions = lio_quaternions[valid]
    interpolated = np.column_stack([
        np.interp(lio_times, reference_times, reference_positions[:, axis])
        for axis in range(3)
    ])
    direct_errors = np.linalg.norm(lio_positions - interpolated, axis=1)

    source_center = np.mean(lio_positions, axis=0)
    target_center = np.mean(interpolated, axis=0)
    covariance = (lio_positions - source_center).T @ (interpolated - target_center)
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1, :] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = target_center - rotation @ source_center
    aligned = (rotation @ lio_positions.T).T + translation
    aligned_errors = np.linalg.norm(aligned - interpolated, axis=1)

    reference_indices = np.searchsorted(reference_times, lio_times)
    reference_indices = np.clip(reference_indices, 1, len(reference_times) - 1)
    before = reference_indices - 1
    choose_before = (
        np.abs(lio_times - reference_times[before]) <=
        np.abs(reference_times[reference_indices] - lio_times))
    reference_indices[choose_before] = before[choose_before]
    quaternion_dots = np.abs(np.sum(
        lio_quaternions * reference_quaternions[reference_indices], axis=1))
    orientation_errors = np.rad2deg(
        2.0 * np.arccos(np.clip(quaternion_dots, 0.0, 1.0)))

    print(timing_summary("IMU header", imu_header_times))
    print(timing_summary("IMU record", imu_record_times))
    print(timing_summary("INS odometry header", reference_times))
    print(timing_summary("LiDAR header", lidar_header_times))
    print(timing_summary("LiDAR record", lidar_record_times))
    print(f"LIO poses: count={len(lio_times)}, span={lio_times[-1] - lio_times[0]:.3f}s")
    print(metric_summary("absolute mine-frame position", direct_errors))
    print(metric_summary("SE(3)-aligned position", aligned_errors))
    print(
        f"absolute orientation: RMSE={np.sqrt(np.mean(orientation_errors ** 2)):.3f}deg, "
        f"median={np.median(orientation_errors):.3f}deg, "
        f"P95={np.percentile(orientation_errors, 95):.3f}deg, "
        f"final={orientation_errors[-1]:.3f}deg")


if __name__ == "__main__":
    main()
