#!/usr/bin/env python3
"""Direct access to MY-LIVO recordings without any conversion step.

A *dataset* is any of:

* a raw recording directory as written by the truck (``<root>/Perception_*/`` and
  ``<root>/Camera_*/`` ROS 2 bag folders, each with ``metadata.yaml`` + ``*.mcap``);
* a single ROS 2 bag folder (``metadata.yaml`` directly inside), e.g. a legacy
  ``results/prepared/*`` runtime bag;
* a folder whose sub-folders (depth <= 2) are such bags.

``Dataset.open(path)`` only reads ``metadata.yaml`` files (no ROS needed), so the Studio can
describe a recording instantly. ``Dataset.iter_messages()`` performs an exact global time-ordered
merge across all segments with a lazily-opened rosbag2 reader per segment, guarantees per-topic
monotonic timestamps (duplicates at segment borders are dropped) and never writes anything to
disk. It replaces ``scripts/prepare_wuhu_bag.py`` for playback, calibration and evaluation.

Verified on the Wuhu truck29 recording: every source MCAP is already globally time ordered
(cross-topic disorder 0 ms) and consecutive segments are contiguous (exactly one sample period
between the last and first message of each topic), so streaming needs no reorder buffer.

CLI::

    python3 -m livo.dataset <path> [--json] [--check]
"""
import heapq
import json
import os
import pathlib
import sys
import time

import yaml

if __package__ in (None, ""):  # executed as a file: make `livo` importable
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

LIO_TOPICS = ("front_left_lidar", "back_lidar", "front_right_lidar", "front_lidar",
              "imu_data", "imu_data/odometry", "imu_data/ins_status", "imu_data/navsat_fix")
CAMERA_STREAM_TOPICS = {"midrange": "midrange_camera/ffmpeg", "fisheye": "fish_camera/ffmpeg"}
CAMERA_TOPICS = (CAMERA_STREAM_TOPICS["midrange"],)
PRESETS = {"lio": LIO_TOPICS, "livo": LIO_TOPICS + CAMERA_TOPICS, "all": None}
LIDAR_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_TYPE = "sensor_msgs/msg/Imu"
CAMERA_TYPES = ("ffmpeg_image_transport_msgs/msg/FFMPEGPacket", "sensor_msgs/msg/Image",
                "sensor_msgs/msg/CompressedImage")
_SKIP_DIR_MARKERS = (".build.",)


def norm(topic):
    return topic.lstrip("/")


class Segment:
    """One ROS 2 bag folder (metadata.yaml + storage files)."""

    def __init__(self, path):
        self.path = pathlib.Path(path)
        with open(self.path / "metadata.yaml", encoding="utf-8") as stream:
            info = yaml.safe_load(stream)["rosbag2_bagfile_information"]
        self.storage_id = info.get("storage_identifier", "mcap")
        self.start_ns = int(info["starting_time"]["nanoseconds_since_epoch"])
        self.duration_ns = int(info["duration"]["nanoseconds"])
        self.end_ns = self.start_ns + self.duration_ns
        self.message_count = int(info.get("message_count", 0))
        self.topics = {}
        self.counts = {}
        for entry in info.get("topics_with_message_count", []):
            meta = entry["topic_metadata"]
            name = norm(meta["name"])
            self.topics[name] = meta["type"]
            self.counts[name] = int(entry.get("message_count", 0))
        self.files = [self.path / rel for rel in info.get("relative_file_paths", [])]
        self.size_bytes = 0
        for f in self.files:
            try:
                self.size_bytes += f.stat().st_size
            except OSError:
                pass

    @property
    def name(self):
        return self.path.name

    def to_dict(self):
        return {"name": self.name, "path": str(self.path), "start_ns": self.start_ns, "end_ns": self.end_ns,
                "duration_s": round(self.duration_ns / 1e9, 3), "messages": self.message_count,
                "topics": dict(self.counts), "size_bytes": self.size_bytes,
                "files": [f.name for f in self.files], "storage": self.storage_id}


class Dataset:
    def __init__(self, root, segments):
        self.root = pathlib.Path(root)
        self.segments = sorted(segments, key=lambda s: (s.start_ns, s.name))
        self.topics = {}
        self.counts = {}
        for seg in self.segments:
            for name, typ in seg.topics.items():
                self.topics.setdefault(name, typ)
                self.counts[name] = self.counts.get(name, 0) + seg.counts.get(name, 0)
        self.start_ns = min(s.start_ns for s in self.segments)
        self.end_ns = max(s.end_ns for s in self.segments)
        self.size_bytes = sum(s.size_bytes for s in self.segments)

    # ------------------------------------------------------------------ discovery
    @classmethod
    def open(cls, path):
        root = pathlib.Path(path).expanduser().resolve()
        if not root.exists():
            raise FileNotFoundError(f"dataset path does not exist: {root}")
        if root.is_file():
            root = root.parent
        segments = []
        if (root / "metadata.yaml").is_file():
            segments.append(Segment(root))
        else:
            for meta in sorted(list(root.glob("*/metadata.yaml")) + list(root.glob("*/*/metadata.yaml"))):
                folder = meta.parent
                if any(m in folder.name for m in _SKIP_DIR_MARKERS):
                    continue
                try:
                    segments.append(Segment(folder))
                except Exception:  # noqa: BLE001  (unreadable metadata -> ignore that folder)
                    continue
        if not segments:
            raise FileNotFoundError(f"no ROS 2 bag (metadata.yaml) found at or below {root}")
        return cls(root, segments)

    # ------------------------------------------------------------------ description
    @property
    def name(self):
        return self.root.name

    @property
    def duration_s(self):
        return (self.end_ns - self.start_ns) / 1e9

    @property
    def kind(self):
        if len(self.segments) == 1 and self.segments[0].path == self.root:
            return "single_bag"
        return "raw_split"

    def topics_of_type(self, type_names):
        if isinstance(type_names, str):
            type_names = (type_names,)
        return [n for n, t in self.topics.items() if t in type_names]

    @property
    def lidar_topics(self):
        return self.topics_of_type(LIDAR_TYPE)

    @property
    def imu_topics(self):
        return self.topics_of_type(IMU_TYPE)

    @property
    def camera_topics(self):
        return self.topics_of_type(CAMERA_TYPES)

    @property
    def has_camera(self):
        return bool(self.camera_topics)

    def resolve_topics(self, wanted):
        """Map requested names (preset name, list, or None=all) to topics present in the dataset."""
        if wanted is None:
            return sorted(self.topics)
        if isinstance(wanted, str):
            if wanted in PRESETS:
                wanted = PRESETS[wanted]
                if wanted is None:
                    return sorted(self.topics)
            else:
                wanted = [wanted]
        out = []
        for w in wanted:
            w = norm(w)
            for name in self.topics:
                if name == w or name.endswith("/" + w):
                    if name not in out:
                        out.append(name)
        return out

    def missing_topics(self, wanted):
        have = set(self.resolve_topics(wanted))
        return [norm(w) for w in (PRESETS.get(wanted, wanted) if isinstance(wanted, str) else wanted) or []
                if not any(h == norm(w) or h.endswith("/" + norm(w)) for h in have)]

    def summary(self):
        lidars = self.lidar_topics
        gaps = []
        # gaps between consecutive segments carrying the same topic set (coverage check)
        by_topics = {}
        for seg in self.segments:
            by_topics.setdefault(tuple(sorted(seg.topics)), []).append(seg)
        for key, segs in by_topics.items():
            for a, b in zip(segs, segs[1:]):
                gap = (b.start_ns - a.end_ns) / 1e9
                if gap > 0.5:
                    gaps.append({"after": a.name, "before": b.name, "gap_s": round(gap, 3)})
        return {
            "path": str(self.root), "name": self.name, "kind": self.kind,
            "segments": [s.to_dict() for s in self.segments], "segment_count": len(self.segments),
            "start_ns": self.start_ns, "end_ns": self.end_ns, "duration_s": round(self.duration_s, 2),
            "start_iso": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.start_ns / 1e9)),
            "size_bytes": self.size_bytes, "size_gb": round(self.size_bytes / 1e9, 2),
            "topics": {n: {"type": t, "messages": self.counts.get(n, 0)} for n, t in sorted(self.topics.items())},
            "lidar_topics": lidars, "imu_topics": self.imu_topics, "camera_topics": self.camera_topics,
            "camera_streams": {name: matches[0] for name, topic in CAMERA_STREAM_TOPICS.items()
                               if (matches := self.resolve_topics([topic]))},
            "has_camera": self.has_camera, "has_imu": bool(self.imu_topics), "has_lidar": bool(lidars),
            "has_ins": any(n.endswith("imu_data/odometry") for n in self.topics),
            "lio_ready": bool(lidars) and bool(self.imu_topics),
            "livo_ready": bool(lidars) and bool(self.imu_topics) and self.has_camera,
            "missing_lio_topics": self.missing_topics("lio"),
            "coverage_gaps": gaps,
        }

    # ------------------------------------------------------------------ reading
    def iter_messages(self, topics=None, start_ns=None, end_ns=None, stats=None):
        """Yield (topic, serialized_bytes, timestamp_ns) in exact global time order.

        topics: preset name / list / None. start_ns/end_ns: absolute nanosecond bounds.
        stats (optional dict) receives counters: yielded, dropped_duplicate, dropped_backwards.
        """
        import rosbag2_py  # ROS-dependent import kept local so describing a dataset needs no ROS

        selected = set(self.resolve_topics(topics))
        if not selected:
            raise ValueError(f"none of the requested topics exist in {self.root}: {topics}")
        pending = [s for s in self.segments if any(t in selected for t in s.topics)
                   and (end_ns is None or s.start_ns <= end_ns) and (start_ns is None or s.end_ns >= start_ns)]
        pending.sort(key=lambda s: s.start_ns)
        heap = []      # (timestamp, seq, reader_index, topic, data)
        readers = []
        seq = 0
        last_ts = {}
        counters = stats if stats is not None else {}
        counters.update({"yielded": 0, "dropped_duplicate": 0, "dropped_backwards": 0, "segments_opened": 0})

        def open_segment(seg):
            nonlocal seq
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(seg.path), storage_id=seg.storage_id),
                        rosbag2_py.ConverterOptions("", ""))
            names = [t for t in seg.topics if t in selected]
            reader.set_filter(rosbag2_py.StorageFilter(topics=names + ["/" + t for t in names]))
            if start_ns is not None and start_ns > seg.start_ns:
                try:  # jump into the segment instead of reading and discarding its head
                    reader.seek(int(start_ns))
                except Exception:  # noqa: BLE001 - storage plugins without seek: sequential skip below
                    pass
            readers.append(reader)
            counters["segments_opened"] += 1
            idx = len(readers) - 1
            push_next(idx)

        def push_next(idx):
            nonlocal seq
            reader = readers[idx]
            while reader.has_next():
                topic, data, ts = reader.read_next()
                topic = norm(topic)
                if topic not in selected:
                    continue
                if start_ns is not None and ts < start_ns:
                    continue
                heapq.heappush(heap, (ts, seq, idx, topic, data))
                seq += 1
                return
            readers[idx] = None  # exhausted; release the file handles

        while pending or heap:
            # A segment cannot contain messages earlier than its recorded start, so it is
            # opened only once the merge front reaches it (bounded number of open files).
            while pending and (not heap or pending[0].start_ns <= heap[0][0]):
                open_segment(pending.pop(0))
            if not heap:
                break
            ts, _, idx, topic, data = heapq.heappop(heap)
            push_next(idx)
            if end_ns is not None and ts > end_ns:
                break
            prev = last_ts.get(topic)
            if prev is not None:
                if ts == prev:
                    counters["dropped_duplicate"] += 1
                    continue
                if ts < prev:
                    counters["dropped_backwards"] += 1
                    continue
            last_ts[topic] = ts
            counters["yielded"] += 1
            yield topic, data, ts

    def read_messages(self, topics, start_ns=None, end_ns=None, stats=None):
        """Yield (topic, deserialized_message, timestamp_ns) for the requested topics."""
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        selected = self.resolve_topics(topics)
        types = {t: get_message(self.topics[t]) for t in selected}
        for topic, data, ts in self.iter_messages(selected, start_ns, end_ns, stats):
            yield topic, deserialize_message(data, types[topic]), ts

    def check(self):
        """Read everything once and report ordering statistics (no output written)."""
        stats = {}
        t0 = time.time()
        n = 0
        first = None
        last = None
        for _, _, ts in self.iter_messages(None, stats=stats):
            n += 1
            first = ts if first is None else first
            last = ts
        stats.update({"messages": n, "read_seconds": round(time.time() - t0, 2),
                      "first_ns": first, "last_ns": last,
                      "span_s": round((last - first) / 1e9, 3) if first is not None else None})
        return stats


def open_dataset(path):
    return Dataset.open(path)


def describe(path):
    """Summary dict or {"error": ...}; never raises (Studio helper)."""
    try:
        return Dataset.open(path).summary()
    except Exception as exc:  # noqa: BLE001
        return {"path": str(path), "error": str(exc)}


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Describe / verify a MY-LIVO dataset (raw split bags or single bag)")
    ap.add_argument("path")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--check", action="store_true", help="read every message once and verify ordering")
    a = ap.parse_args(argv)
    ds = Dataset.open(a.path)
    s = ds.summary()
    if a.check:
        from livo import rosenv
        rosenv.ensure()
        s["check"] = ds.check()
    if a.json:
        print(json.dumps(s, ensure_ascii=False, indent=2))
        return
    print(f"{s['name']}  [{s['kind']}]  {s['duration_s']} s  {s['size_gb']} GB  {s['segment_count']} segment(s)  start {s['start_iso']}")
    for name, info in s["topics"].items():
        print(f"  {name:32s} {info['type']:48s} {info['messages']}")
    print(f"  LIO ready: {s['lio_ready']}   LIVO ready: {s['livo_ready']}   camera: {s['has_camera']}   INS: {s['has_ins']}")
    if s["coverage_gaps"]:
        print("  coverage gaps:", s["coverage_gaps"])
    if "check" in s:
        print("  check:", s["check"])


if __name__ == "__main__":
    main()
