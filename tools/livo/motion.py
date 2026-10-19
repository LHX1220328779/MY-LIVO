#!/usr/bin/env python3
"""Motion profile of a recording and long-standstill detection (shared by the player and livo_calib).

``motion_profile(path)`` builds a 1 Hz speed / yaw-rate / distance profile from the INS odometry
(``imu_data/odometry``) plus the raw IMU sample gaps, relative to the dataset start (the player's
``--start-offset`` origin). It is cached per recording under ``results/.studio/motion/`` (same cache
and format as the calibration tool has always used).

``standstill_skips(profile, ...)`` turns the profile into playback skip windows: every standstill of
at least ``min_s`` seconds that begins after the first drive keeps ``head_s`` seconds at its start
and ``tail_s`` seconds before the vehicle moves again at the normal rate; only the middle part is
fast-forwarded by the player (IMU/INS complete, LiDAR thinned; LIO rounds only), so the
estimator state, the IMU integration and the RTK/INS association stay continuous and the first
seconds of motion are always processed at full rate.

    python3 -m livo.motion <dataset> [--min-s 30] [--json]
"""
import hashlib
import json
import pathlib
import sys

if __package__ in (None, ""):  # executed as a file: make `livo` importable
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

REPO = pathlib.Path(__file__).resolve().parents[2]
MOTION_CACHE = REPO / "results" / ".studio" / "motion"

# standstill: INS speed below STANDSTILL_MPS and yaw rate below STANDSTILL_DPS (1 Hz profile)
STANDSTILL_MPS = 0.05
STANDSTILL_DPS = 0.5
FIRST_DRIVE_MPS = 0.3


def motion_profile(bag, refresh=False):
    """1 Hz speed / yaw / distance profile of a recording from the INS odometry (plus IMU sample gaps)."""
    import numpy as np
    from livo.dataset import Dataset
    ds = Dataset.open(bag)
    key = hashlib.sha1(f"{pathlib.Path(bag).resolve()}|{ds.start_ns}|{ds.end_ns}".encode()).hexdigest()[:16]
    cache = MOTION_CACHE / f"{key}.json"
    if cache.exists() and not refresh:
        try:
            return json.loads(cache.read_text())
        except Exception:  # noqa: BLE001
            pass
    from scipy.spatial.transform import Rotation as R
    t0 = ds.start_ns * 1e-9
    t, p, yaw, ti = [], [], [], []
    for topic, m, _ts in ds.read_messages(ds.resolve_topics(["imu_data/odometry", "imu_data"])):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t0
        if topic.endswith("odometry"):
            q = m.pose.pose.orientation
            t.append(st)
            p.append([m.pose.pose.position.x, m.pose.pose.position.y])
            yaw.append(R.from_quat([q.x, q.y, q.z, q.w]).as_euler("zyx")[0])
        else:
            ti.append(st)
    if len(t) < 100:
        raise SystemExit(f"motion profile: {bag} has no usable INS odometry (imu_data/odometry)")
    t, p, yaw = np.array(t), np.array(p), np.unwrap(np.array(yaw))
    step = np.r_[0.0, np.linalg.norm(np.diff(p, axis=0), axis=1)]
    dist = np.cumsum(step)
    secs = np.arange(0, int(np.ceil(t[-1])) + 1)
    d1 = np.interp(secs, t, dist)
    y1 = np.interp(secs, t, yaw)
    speed = np.r_[np.diff(d1), 0.0]
    dyaw = np.r_[np.abs(np.diff(y1)), 0.0]
    dti = np.diff(np.asarray(ti)) if len(ti) > 1 else np.array([])
    gaps = [{"t_s": round(float(ti[i]), 2), "gap_ms": round(float(1000 * dti[i]), 1)} for i in np.flatnonzero(dti > 0.025)]
    prof = {"bag": str(bag), "duration_s": round(float(t[-1]), 2), "path_m": round(float(dist[-1]), 1),
            "speed_mps": [round(float(v), 3) for v in speed], "yaw_rate_dps": [round(float(np.degrees(v)), 3) for v in dyaw],
            "imu_gaps": gaps, "imu_median_dt_ms": round(float(1000 * np.median(dti)), 2) if len(dti) else None}
    try:
        MOTION_CACHE.mkdir(parents=True, exist_ok=True)
        cache.write_text(json.dumps(prof) + "\n")
    except OSError:
        pass
    return prof


def standstill_runs(prof, speed_mps=STANDSTILL_MPS, yaw_dps=STANDSTILL_DPS):
    """[(start_s, end_s)] standstill runs of the 1 Hz profile (second i covers [i, i+1))."""
    v = prof["speed_mps"]
    w = prof.get("yaw_rate_dps") or [0.0] * len(v)
    runs, start = [], None
    for i in range(len(v) + 1):
        still = i < len(v) and v[i] < speed_mps and w[i] < yaw_dps
        if still and start is None:
            start = i
        elif not still and start is not None:
            runs.append((float(start), float(i)))
            start = None
    return runs


def standstill_skips(prof, min_s=30.0, head_s=3.0, tail_s=5.0, start_s=0.0, end_s=None,
                     speed_mps=STANDSTILL_MPS, yaw_dps=STANDSTILL_DPS, min_skip_s=10.0):
    """Fast-forward windows [(from_s, to_s)] (seconds from the dataset start) inside the playback span.

    Only standstills that begin after the first drive qualify: the initial standstill is the IMU
    initialisation / static extrinsic check / gravity datum of the mapper and is always played."""
    v = prof["speed_mps"]
    first_drive = next((i for i, s in enumerate(v) if s >= FIRST_DRIVE_MPS), None)
    if first_drive is None:
        return []
    lo = max(float(start_s), float(first_drive))
    hi = float(prof.get("duration_s") or len(v)) if end_s is None else float(end_s)
    out = []
    for a, b in standstill_runs(prof, speed_mps, yaw_dps):
        if b - a < min_s or a < first_drive:
            continue
        f, t = max(a + head_s, lo + head_s), min(b - tail_s, hi)
        if t - f >= min_skip_s:
            out.append((round(f, 3), round(t, 3)))
    return out


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("--min-s", type=float, default=30.0)
    ap.add_argument("--head-s", type=float, default=3.0)
    ap.add_argument("--tail-s", type=float, default=5.0)
    ap.add_argument("--refresh", action="store_true")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    prof = motion_profile(a.dataset, refresh=a.refresh)
    runs = standstill_runs(prof)
    skips = standstill_skips(prof, a.min_s, a.head_s, a.tail_s)
    doc = {"dataset": a.dataset, "duration_s": prof["duration_s"], "path_m": prof["path_m"],
           "standstills": [r for r in runs if r[1] - r[0] >= 5], "skips": skips,
           "skipped_s": round(sum(t - f for f, t in skips), 1)}
    if a.json:
        print(json.dumps(doc, ensure_ascii=False))
    else:
        print(f"{a.dataset}: {prof['duration_s']:.0f} s, {prof['path_m']:.0f} m")
        for r in doc["standstills"]:
            print(f"  standstill {r[0]:7.1f} - {r[1]:7.1f} s ({r[1] - r[0]:.0f} s)")
        for f, t in skips:
            print(f"  skip       {f:7.1f} - {t:7.1f} s ({t - f:.0f} s)")
        print(f"  fast-forwarded {doc['skipped_s']:.0f} s of {prof['duration_s']:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
