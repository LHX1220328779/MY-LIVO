#!/usr/bin/env python3
"""Camera stream / recording quality audit: header-stamp regularity of midrange_camera/ffmpeg, duplicate
messages per topic (same recording stamp), receive-vs-header latency, HEVC keyframe cadence.
usage: camera_stream_audit.py <bag root> [--duration S] [--start S]"""
import argparse, pathlib, sys, collections
import numpy as np
REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from livo import rosenv  # noqa: E402
rosenv.ensure()
from livo.dataset import Dataset  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("bag"); ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--start", type=float, default=0.0)
    a = ap.parse_args()
    ds = Dataset.open(a.bag)
    topics = [t for t in ds.topics if t in ("midrange_camera/ffmpeg", "imu_data", "front_lidar", "back_lidar", "front_left_lidar", "front_right_lidar")]
    start_ns = ds.start_ns + int(a.start * 1e9)
    end_ns = None if a.duration is None else start_ns + int(a.duration * 1e9)
    last_rx = {}; dup = collections.Counter(); rx_int = collections.defaultdict(list)
    cam_hdr, cam_rx, cam_key, cam_size = [], [], [], []
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    cam_type = get_message(ds.topics["midrange_camera/ffmpeg"])
    for topic, data, ts in ds.iter_messages(topics, start_ns, end_ns, stats={}):
        # iter_messages already drops exact duplicates; count near-duplicates (< 1 ms apart) instead
        prev = last_rx.get(topic)
        if prev is not None:
            d = (ts - prev) / 1e6
            rx_int[topic].append(d)
            if d < 1.0:
                dup[topic] += 1
        last_rx[topic] = ts
        if topic == "midrange_camera/ffmpeg":
            m = deserialize_message(data, cam_type)
            cam_hdr.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); cam_rx.append(ts / 1e9)
            cam_key.append(bool(getattr(m, "flags", 0) & 1)); cam_size.append(len(m.data))
    print(f"{a.bag}: window {a.start:.0f}s +{a.duration if a.duration else 'all'}")
    for t in topics:
        v = np.array(rx_int[t])
        if len(v) == 0:
            continue
        print(f"  {t:24s} n={len(v)+1:6d} recv interval median {np.median(v):6.2f} ms  p1 {np.percentile(v,1):6.2f}  p99 {np.percentile(v,99):7.2f}  max {v.max():8.2f}  <1ms pairs {dup[t]}")
    h = np.array(cam_hdr); rx = np.array(cam_rx); dh = np.diff(h) * 1000
    print(f"  camera header stamps: n={len(h)} interval median {np.median(dh):.2f} ms p1 {np.percentile(dh,1):.2f} p99 {np.percentile(dh,99):.2f} max {dh.max():.1f}; "
          f"non-monotonic {np.sum(dh<=0)}; gaps >50 ms {np.sum(dh>50)}")
    lat = (rx - h) * 1000
    print(f"  camera recv - header latency: median {np.median(lat):.1f} ms p5 {np.percentile(lat,5):.1f} p95 {np.percentile(lat,95):.1f} (jitter p95-p5 {np.percentile(lat,95)-np.percentile(lat,5):.1f} ms)")
    k = np.array(cam_key); sz = np.array(cam_size)
    if k.any():
        ki = np.flatnonzero(k)
        print(f"  HEVC keyframes: {k.sum()} (every {np.median(np.diff(ki)) if len(ki)>1 else 0:.0f} packets); packet size median {np.median(sz)/1024:.1f} kB, keyframe median {np.median(sz[k])/1024:.1f} kB")
    else:
        print(f"  HEVC: no keyframe flag decoded; packet size median {np.median(sz)/1024:.1f} kB, p99 {np.percentile(sz,99)/1024:.1f} kB")
    # fraction of camera header intervals that are exact multiples of 33.33 ms vs irregular
    irregular = np.sum(np.abs(dh - np.round(dh / 33.333) * 33.333) > 3)
    print(f"  camera header intervals off the 33.3 ms grid by >3 ms: {irregular} ({100*irregular/len(dh):.2f}%)")


if __name__ == "__main__":
    main()
