#!/usr/bin/env python3
"""Real-time ROS 2 playback of a MY-LIVO dataset straight from the raw recording.

Replaces ``ros2 bag play`` + the ``results/prepared`` merge step: all segments of a recording
(Perception_* / Camera_* folders, or one bag) are merged on the fly in exact time order and
re-published with their original serialized payloads (zero deserialization), a ``/clock`` for
``use_sim_time`` nodes, real-time pacing at ``--rate`` and a clean stop on SIGINT/SIGTERM.
Per-topic timestamps are guaranteed monotonic, which the mapper relies on (it treats a
backwards IMU/LiDAR/image stamp as fatal).

    /usr/bin/python3 -m livo.player /home/project/data/haibo/wuhu_livo/ros2bag --preset livo
    /usr/bin/python3 -m livo.player <dataset> --preset lio --start-offset 45 --duration 30 --rate 0.5

Progress lines (``[player] t=12.0/120.1 s 10% msgs=5600 lag=2 ms``) go to stdout so that the
Studio can display playback progress; the process name/cmdline contains ``livo.player`` for
targeted signalling (``pkill -INT -f livo.player``).
"""
import argparse
import json
import os
import pathlib
import signal
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from livo import rosenv  # noqa: E402
from livo.dataset import Dataset, PRESETS  # noqa: E402

CLOCK_TOPIC = "/clock"


def log(msg):
    print(f"[player] {msg}", flush=True)


class Player:
    def __init__(self, args):
        self.args = args
        self.stop_requested = False
        self.dataset = Dataset.open(args.dataset)
        topics = args.topics if args.topics else args.preset
        self.topics = self.dataset.resolve_topics(topics)
        if not self.topics:
            raise SystemExit(f"dataset has none of the requested topics ({topics}): {self.dataset.root}")
        if args.preset == "livo" and not args.topics and not self.dataset.has_camera:
            raise SystemExit("preset 'livo' requested but the dataset has no camera topic; "
                             "use --preset lio or a recording that contains the camera")
        missing = self.dataset.missing_topics(topics) if not args.topics else []
        if missing:
            log(f"warning: topics missing from the dataset: {missing}")
        self.start_ns = self.dataset.start_ns + int(args.start_offset * 1e9)
        if self.start_ns >= self.dataset.end_ns:
            raise SystemExit("--start-offset is beyond the end of the dataset")
        self.end_ns = None if args.duration is None else self.start_ns + int(args.duration * 1e9)
        self.span_ns = (min(self.end_ns, self.dataset.end_ns) if self.end_ns else self.dataset.end_ns) - self.start_ns

    # ------------------------------------------------------------------ ROS setup
    def setup_ros(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rosidl_runtime_py.utilities import get_message
        from rosgraph_msgs.msg import Clock
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init(args=None)
        self.rclpy = rclpy
        self.node = rclpy.create_node("livo_player")
        reliable = self.args.qos == "reliable"

        def make_qos(type_name):
            # Writer history depth per message class. A RELIABLE writer with KEEP_LAST(10) still loses
            # samples: when the reader stalls > max_blocking_time (100 ms) the oldest samples are
            # overwritten. That produced an exact 0.100 s hole in a clean 100 Hz IMU stream (10 samples)
            # at t=537 s of a 720 s round while the mapper was busy, and the strict deskew aborted the
            # run (results/mcp_t05_colour_full, 2026-09-27). Small high-rate topics get a deep history;
            # point clouds stay shallow to bound memory (1.5 MB each).
            # Images/HEVC packets deliberately keep the shallow history: when the decoder (republish)
            # falls behind, late packets must be dropped, otherwise the images reach the mapper seconds
            # late, the LIVO frame cadence collapses and the fusion aborts ("Invalid LIO pose or scan
            # interval", observed with depth 300 at t=93 s of the same 720 s round).
            # Point clouds: depth 20 is the validated value (720 s truck05 round completed). Depth 60
            # made the mapper 2-3x slower (reliable repair traffic on 1.5 MB samples) and the multi-LiDAR
            # merger lost its 1 ms scan-start alignment and never recovered ("Dropping unmatched ...
            # scan-start span 97 ms exceeds 1.0 ms" forever) - see docs/mcp-colour-only-and-time-offset-audit.md.
            if "PointCloud2" in type_name:
                depth = 20
            elif "Image" in type_name or "FFMPEG" in type_name or "CompressedImage" in type_name:
                depth = 10
            else:
                depth = 3000
            return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.pubs = {}
        for name in self.topics:
            type_name = self.dataset.topics[name]
            msg_type = get_message(type_name)
            self.pubs[name] = self.node.create_publisher(msg_type, "/" + name, make_qos(type_name))
        # Flow control: follow the mapper's progress (its odometry header stamp) and hold the
        # playback when the data time runs more than --max-lead seconds ahead of it. Without this a
        # 720 s colour-only round fell 48 s behind between t=250 and 450 s (LIO 80-95 ms/frame), the
        # mapper's reader histories overflowed (LiDAR 20 s, IMU 50 s) and the run aborted on an IMU gap
        # that does not exist in the recording (results/mcp_t05_colour_full_* 2026-09-27).
        self.mapper_progress = None
        self.pace_sub = None
        if self.args.pace_topic:
            from nav_msgs.msg import Odometry
            pace_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

            def on_progress(msg):
                self.mapper_progress = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.pace_sub = self.node.create_subscription(Odometry, self.args.pace_topic, on_progress, pace_qos)
        self.clock_msg = Clock()
        self.clock_pub = None
        if self.args.clock:
            clock_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                   reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            self.clock_pub = self.node.create_publisher(Clock, CLOCK_TOPIC, clock_qos)

    def publish_clock(self, ts):
        if self.clock_pub is None:
            return
        self.clock_msg.clock.sec = int(ts // 1_000_000_000)
        self.clock_msg.clock.nanosec = int(ts % 1_000_000_000)
        self.clock_pub.publish(self.clock_msg)

    def wait_for_subscribers(self):
        timeout = self.args.wait_subscribers
        if timeout <= 0:
            return
        key = [t for t in self.topics if t in self.dataset.imu_topics or t in self.dataset.lidar_topics] or list(self.topics)
        deadline = time.monotonic() + timeout
        log(f"waiting up to {timeout:.0f} s for subscribers on {len(key)} sensor topic(s) ...")
        while time.monotonic() < deadline and not self.stop_requested:
            counts = {t: self.pubs[t].get_subscription_count() for t in key}
            if all(c > 0 for c in counts.values()):
                log("all sensor topics have subscribers: " + ", ".join(f"{t}={c}" for t, c in counts.items()))
                return
            time.sleep(0.2)
        counts = {t: self.pubs[t].get_subscription_count() for t in key}
        log("proceeding without full subscriber set: " + ", ".join(f"{t}={c}" for t, c in counts.items()))

    # ------------------------------------------------------------------ main loop
    def run(self):
        a = self.args
        self.setup_ros()
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)
        log(f"dataset {self.dataset.root} [{self.dataset.kind}] {len(self.dataset.segments)} segment(s), "
            f"{self.dataset.duration_s:.1f} s, topics {self.topics}")
        log(f"start_offset={a.start_offset:.3f} s duration={'all' if a.duration is None else f'{a.duration:.1f} s'} "
            f"rate={a.rate} clock={'on' if a.clock else 'off'} qos={a.qos}")
        self.publish_clock(self.start_ns)
        self.wait_for_subscribers()
        if a.delay > 0 and not self.stop_requested:
            time.sleep(a.delay)
        stats = {}
        rate = a.rate
        msgs = 0
        per_topic = {}
        first_ts = None
        wall0 = None
        last_clock = None
        last_progress = time.monotonic()
        clock_period = int(1e9 / a.clock_hz)
        max_lag = 0.0
        paced_total = 0.0
        t_begin = time.monotonic()
        try:
            for topic, data, ts in self.dataset.iter_messages(self.topics, self.start_ns, self.end_ns, stats):
                if self.stop_requested:
                    break
                if first_ts is None:
                    first_ts = ts
                    wall0 = time.monotonic()
                    last_clock = ts - clock_period
                target = wall0 + (ts - first_ts) / 1e9 / rate
                now = time.monotonic()
                if target > now:
                    if self.pace_sub is not None:
                        self.rclpy.spin_once(self.node, timeout_sec=target - now)
                        now = time.monotonic()
                        if target > now:
                            time.sleep(target - now)
                    else:
                        time.sleep(target - now)
                else:
                    lag = now - target
                    if lag > max_lag:
                        max_lag = lag
                    if self.pace_sub is not None and msgs % 20 == 0:
                        self.rclpy.spin_once(self.node, timeout_sec=0.0)
                if self.pace_sub is not None and self.mapper_progress is not None:
                    hold0 = None
                    while not self.stop_requested and ts / 1e9 - self.mapper_progress > a.max_lead:
                        if hold0 is None:
                            hold0 = time.monotonic()
                            prog0 = self.mapper_progress
                        elif time.monotonic() - hold0 > a.hold_timeout and self.mapper_progress == prog0:
                            # The mapper is not consuming what it already has (e.g. the multi-LiDAR merger
                            # lost its alignment and drops every frame); holding forever would deadlock.
                            log(f"mapper made no progress for {a.hold_timeout:.0f} s while held at t="
                                f"{(ts - self.start_ns) / 1e9:.1f} s; releasing the playback")
                            self.mapper_progress = None
                            break
                        self.rclpy.spin_once(self.node, timeout_sec=0.05)
                    if hold0 is not None:
                        held = time.monotonic() - hold0
                        paced_total += held
                        wall0 += held  # do not rush to catch up afterwards
                if last_clock is not None and ts - last_clock >= clock_period:
                    self.publish_clock(ts)
                    last_clock = ts
                self.pubs[topic].publish(data)
                msgs += 1
                per_topic[topic] = per_topic.get(topic, 0) + 1
                now = time.monotonic()
                if now - last_progress >= a.progress_interval:
                    last_progress = now
                    done = (ts - self.start_ns) / max(self.span_ns, 1)
                    lead = "" if self.mapper_progress is None else f" mapper_lead={ts / 1e9 - self.mapper_progress:.1f} s"
                    log(f"t={(ts - self.start_ns) / 1e9:.1f}/{self.span_ns / 1e9:.1f} s {done * 100:.0f}% "
                        f"msgs={msgs} lag={max(0.0, now - target) * 1000:.0f} ms{lead}")
            if first_ts is not None and self.clock_pub is not None and not self.stop_requested:
                self.publish_clock(ts)
        finally:
            status = "interrupted" if self.stop_requested else "completed"
            elapsed = time.monotonic() - t_begin
            summary = {"status": status, "messages": msgs, "per_topic": per_topic, "elapsed_s": round(elapsed, 2),
                       "max_lag_ms": round(max_lag * 1000, 1), "paced_s": round(paced_total, 1),
                       **{k: v for k, v in stats.items()}}
            log(f"{status}: {msgs} messages in {elapsed:.1f} s (max lag {max_lag * 1000:.0f} ms, "
                f"held for the mapper {paced_total:.1f} s, "
                f"dropped duplicate {stats.get('dropped_duplicate', 0)}, backwards {stats.get('dropped_backwards', 0)})")
            print("[player] summary " + json.dumps(summary), flush=True)
            if a.hold > 0 and not self.stop_requested:
                time.sleep(a.hold)  # let reliable subscribers drain the last samples
            self.node.destroy_node()
            self.rclpy.shutdown()
        return 0

    def _on_signal(self, signum, _frame):
        if not self.stop_requested:
            log(f"signal {signum}: stopping playback")
        self.stop_requested = True


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="raw recording root (Perception_*/Camera_* folders) or one bag folder")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="livo",
                    help="topic set: lio (LiDAR+IMU+INS), livo (lio + camera), all")
    ap.add_argument("--topics", nargs="*", help="explicit topic list (overrides --preset)")
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--start-offset", type=float, default=0.0, help="seconds skipped at the beginning")
    ap.add_argument("--duration", type=float, default=None, help="seconds of data to play (default: all)")
    ap.add_argument("--clock", dest="clock", action="store_true", default=True)
    ap.add_argument("--no-clock", dest="clock", action="store_false")
    ap.add_argument("--clock-hz", type=float, default=40.0)
    ap.add_argument("--qos", choices=("reliable", "sensor"), default="reliable")
    ap.add_argument("--wait-subscribers", type=float, default=8.0, help="seconds to wait for sensor subscribers (0 = do not wait)")
    ap.add_argument("--delay", type=float, default=1.0, help="extra settle time before the first message")
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to keep the node alive after the last message")
    ap.add_argument("--progress-interval", type=float, default=5.0)
    ap.add_argument("--pace-topic", default="/aft_mapped_to_init",
                    help="mapper odometry topic used for flow control ('' = free-running playback)")
    ap.add_argument("--max-lead", type=float, default=3.0,
                    help="hold the playback when the data time is more than this many seconds ahead of the mapper")
    ap.add_argument("--hold-timeout", type=float, default=20.0,
                    help="release a hold when the mapper has made no progress for this long (deadlock guard)")
    ap.add_argument("--dry-run", action="store_true", help="describe what would be played and exit")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.dry_run:
        ds = Dataset.open(args.dataset)
        topics = ds.resolve_topics(args.topics if args.topics else args.preset)
        print(json.dumps({"dataset": str(ds.root), "kind": ds.kind, "segments": [s.name for s in ds.segments],
                          "duration_s": round(ds.duration_s, 2), "topics": topics,
                          "has_camera": ds.has_camera}, ensure_ascii=False, indent=2))
        return 0
    rosenv.ensure()
    return Player(args).run()


if __name__ == "__main__":
    sys.exit(main())
