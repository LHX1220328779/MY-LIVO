#!/usr/bin/env python3
"""Build one indexed, time-ordered MY-LIVO bag from split source MCAPs."""

import os
import sys

# Containers used for this project may put Conda or /usr/local Python ahead of
# Ubuntu's interpreter.  rosbag2_py is an ABI-bound Humble extension, so always
# re-enter through the Ubuntu 22.04 system Python with those overrides removed.
if sys.executable != "/usr/bin/python3":
    clean_environment = os.environ.copy()
    for variable in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME", "PYTHONPATH"):
        clean_environment.pop(variable, None)
    clean_environment["PATH"] = (
        "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:"
        "/usr/sbin:/usr/bin:/sbin:/bin")
    clean_environment["AMENT_PREFIX_PATH"] = "/opt/ros/humble"
    clean_environment["PYTHONPATH"] = (
        "/opt/ros/humble/lib/python3.10/site-packages:"
        "/opt/ros/humble/local/lib/python3.10/dist-packages")
    os.execve("/usr/bin/python3", ["/usr/bin/python3", *sys.argv], clean_environment)

import argparse
import heapq
from pathlib import Path

import rosbag2_py


DEFAULT_TOPICS = {
    "front_lidar",
    "imu_data",
    "imu_data/odometry",
    "midrange_camera/ffmpeg",
}


def next_on_topic(reader, selected_topic, previous_timestamp):
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == selected_topic:
            if timestamp < previous_timestamp:
                raise RuntimeError(
                    f"source MCAP moved backwards on {selected_topic}")
            previous_timestamp = timestamp
            return (topic, data, timestamp), previous_timestamp
    return None, previous_timestamp


def main():
    parser = argparse.ArgumentParser(
        description="Merge split Wuhu MCAP bags into one indexed runtime bag")
    parser.add_argument(
        "--bag-root", type=Path,
        default=Path("/home/project/data/haibo/wuhu_livo/ros2bag"))
    parser.add_argument(
        "--output", type=Path,
        default=Path("/home/project/data/haibo/wuhu_livo/ros2bag_my_livo"))
    parser.add_argument(
        "--lio-only", action="store_true",
        help="omit the camera topic (smaller, but cannot run LIVO mode)")
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(
            f"output already exists: {args.output}; choose a new path or remove it explicitly")

    metadata_files = sorted(args.bag_root.glob("*/metadata.yaml"))
    source_bags = [path.parent for path in metadata_files
                   if path.parent.name.startswith(("Perception_", "Camera_"))]
    if not source_bags:
        raise SystemExit(f"no source bags found below {args.bag_root}")

    selected_topics = set(DEFAULT_TOPICS)
    if args.lio_only:
        selected_topics.remove("midrange_camera/ffmpeg")

    # The source recorder wrote different topics slightly out of timestamp
    # order and its MCAPs have no message index.  Give every selected topic an
    # independent sequential reader; each topic is monotonic, and the heap
    # then performs an exact global merge without loading image/point-cloud
    # payloads into memory.
    readers = []
    reader_topics = []
    previous_topic_timestamps = []
    topic_metadata = {}
    heap = []
    sequence = 0
    for source_bag in source_bags:
        probe = rosbag2_py.SequentialReader()
        probe.open(
            rosbag2_py.StorageOptions(uri=str(source_bag), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""))
        available = {
            metadata.name: metadata
            for metadata in probe.get_all_topics_and_types()
            if metadata.name in selected_topics
        }
        del probe
        for selected_topic, metadata in available.items():
            topic_metadata[selected_topic] = metadata
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(source_bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
            readers.append(reader)
            reader_topics.append(selected_topic)
            previous_topic_timestamps.append(-1)
            reader_index = len(readers) - 1
            item, previous_topic_timestamps[reader_index] = next_on_topic(
                reader, selected_topic,
                previous_topic_timestamps[reader_index])
            if item is not None:
                topic, data, timestamp = item
                heapq.heappush(
                    heap, (timestamp, sequence, reader_index, topic, data))
                sequence += 1

    missing = selected_topics - set(topic_metadata)
    if missing:
        raise SystemExit(f"selected topics missing from source bags: {sorted(missing)}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""))
    for name in sorted(topic_metadata):
        writer.create_topic(topic_metadata[name])

    counts = {topic: 0 for topic in selected_topics}
    last_topic_timestamp = {topic: -1 for topic in selected_topics}
    while heap:
        timestamp, _, reader_index, topic, data = heapq.heappop(heap)
        # Exact boundary duplicates are redundant. A backwards timestamp is a
        # hard data error and must never be passed to FAST-LIVO2.
        if timestamp < last_topic_timestamp[topic]:
            raise RuntimeError(f"timestamp moved backwards on {topic}")
        if timestamp > last_topic_timestamp[topic]:
            writer.write(topic, data, timestamp)
            last_topic_timestamp[topic] = timestamp
            counts[topic] += 1

        item, previous_topic_timestamps[reader_index] = next_on_topic(
            readers[reader_index], reader_topics[reader_index],
            previous_topic_timestamps[reader_index])
        if item is not None:
            next_topic, next_data, next_timestamp = item
            heapq.heappush(
                heap, (next_timestamp, sequence, reader_index, next_topic, next_data))
            sequence += 1

    del writer
    print(f"prepared {args.output}")
    for topic in sorted(counts):
        print(f"  {topic}: {counts[topic]}")


if __name__ == "__main__":
    main()
