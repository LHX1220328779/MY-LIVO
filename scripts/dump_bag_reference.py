#!/usr/bin/env python3
"""Dump INS odometry / status / raw IMU from the prepared Wuhu MCAP into CSV for offline analysis.

Usage (inside the calibration container):
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/dump_bag_reference.py --bag results/prepared/wuhu_livo --output results/prepared/reference_csv
"""
import argparse, csv, pathlib
import sys
from pathlib import Path as _LivoPath

for _cand in (_LivoPath(__file__).resolve().parents[1] / "tools", _LivoPath("/home/project/MY-LIVO/tools")):
    if (_cand / "livo" / "rosenv.py").exists():
        sys.path.insert(0, str(_cand))
        break
from livo import rosenv as _rosenv  # noqa: E402
_rosenv.ensure()
from livo.dataset import Dataset  # noqa: E402
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--span", nargs=2, type=float, default=None,
                    help="absolute start/end time (s) of the round: only that part (+-10 s) of the recording is read")
    a = ap.parse_args()
    out = pathlib.Path(a.output)
    out.mkdir(parents=True, exist_ok=True)
    dataset = Dataset.open(a.bag)
    topics = ["imu_data", "imu_data/odometry", "imu_data/ins_status"]
    f_odom = open(out / "ins_odometry.csv", "w", newline="")
    f_stat = open(out / "ins_status.csv", "w", newline="")
    f_imu = open(out / "imu_raw.csv", "w", newline="")
    w_odom = csv.writer(f_odom); w_stat = csv.writer(f_stat); w_imu = csv.writer(f_imu)
    w_odom.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "cxx", "cyy", "czz", "vx", "vy", "vz", "child_frame"])
    w_stat.writerow(["stamp", "ins_pos_mode", "gps_num_sats", "extra"])
    w_imu.writerow(["stamp", "gx", "gy", "gz", "ax", "ay", "az"])
    n = [0, 0, 0]
    start_ns = end_ns = None
    if a.span:
        start_ns = max(dataset.start_ns, int((a.span[0] - 10.0) * 1e9))
        end_ns = int((a.span[1] + 10.0) * 1e9)
    for topic, data, _ in dataset.iter_messages(topics, start_ns, end_ns):
        if topic == "imu_data/odometry":
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            c = m.pose.covariance
            v = m.twist.twist.linear
            w_odom.writerow([f"{stamp_seconds(m.header.stamp):.9f}", p.x, p.y, p.z, q.x, q.y, q.z, q.w, c[0], c[7], c[14], v.x, v.y, v.z, m.child_frame_id])
            n[0] += 1
        elif topic == "imu_data/ins_status":
            m = deserialize_message(data, DiagnosticArray)
            values = {item.key: item.value for status in m.status for item in status.values}
            extra = ";".join(f"{k}={v}" for k, v in sorted(values.items()) if k not in ("ins_pos_mode", "gps_num_sats"))
            w_stat.writerow([f"{stamp_seconds(m.header.stamp):.9f}", values.get("ins_pos_mode", ""), values.get("gps_num_sats", ""), extra])
            n[1] += 1
        elif topic == "imu_data":
            m = deserialize_message(data, Imu)
            g, acc = m.angular_velocity, m.linear_acceleration
            w_imu.writerow([f"{stamp_seconds(m.header.stamp):.9f}", g.x, g.y, g.z, acc.x, acc.y, acc.z])
            n[2] += 1
    for f in (f_odom, f_stat, f_imu):
        f.close()
    print("odometry", n[0], "status", n[1], "imu", n[2], "->", out)


if __name__ == "__main__":
    main()

