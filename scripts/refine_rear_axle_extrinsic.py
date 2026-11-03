#!/usr/bin/env python3
"""Reference-LiDAR -> rear-axle alignment (rig correction) from the recording's own RTK/INS poses.

Why this stage exists: ground, driven and the SLAM precheck refine every LiDAR RELATIVE to the
reference LiDAR (front_lidar). Nothing observed the reference itself against the vehicle frame, so a
supplier file whose rig-to-rear-axle alignment is off (truck29 raw file: ~2.3/0.4/1.2 deg, 14/18 cm)
stayed off, and the pipeline only worked when started from a hand-tuned file.

Method: automates the manual check "stitch the front-LiDAR scans along the RTK trajectory and tune
xyz/rpy until the map is sharp". Every reference scan is deskewed per point with the INS rear-axle pose
(imu_data/odometry, RTK-fixed samples only) at the point time and accumulated in the world frame. The
correction C (rotation about the LiDAR position + horizontal translation, in the rear-axle frame) and the
LiDAR time shift are solved jointly by robust Gauss-Newton on cross-time point-to-plane residuals: every
sampled point is compared with the local plane of the returns recorded more than --gap s away, and both
sides move with C (symmetric Jacobian), so the result does not depend on the starting value.
  pitch  ground seen from different ranges (straight driving suffices)
  yaw    roadside structure seen from different ranges ("lateral stretch")
  roll   ground/structure seen from different headings (turns, round trips)
  x, y   heading changes (turns); y is separated from the time shift by speed changes
  z      unobservable (shifts the whole map vertically): kept from the parent
  time   turn/body-pitch dynamics and speed changes
C is applied to EVERY LiDAR and to the camera, so the relative calibration of the parent (ground/driven/
camera stages) is preserved: the same command aligns a raw supplier file or fine-tunes a refined version.
Precision is reported as the spread of two independent half-recording solutions.

DoF policy (livo_calib.py axle --dof auto): every import that this stage has not aligned yet gets rpy_xy (raw
supplier files are off by degrees, the hand-tuned truck29 v001 export by 0.2 deg, enough for 14.5 m of native
LIO drift in 120 s). A parent already aligned by a previous axle version gets xy: the
rotation is kept and only the lever arm and the LiDAR time shift are applied (always solved jointly with the
rotation: with a fixed rotation that is 0.2 deg off, the two half-recording lever arms differ by 30-55 cm). On truck29 the full-rotation
solution is INS-consistent to 0.1 deg and improves trajectory/ground/attitude metrics, but in LIVO it shifts
the single-pole multi-epoch consistency 9 -> 12-13 cm, while xy on the hand-tuned lineage improves all
trajectory metrics with the pole unchanged (docs/mcp-raw-extrinsic-rear-axle-alignment.md).

Usage (container):
  /usr/bin/python3 scripts/refine_rear_axle_extrinsic.py --calibration <parent calibration.json> \\
      --bag /home/project/data/haibo/wuhu_livo/ros2bag --output <calibration.json> --report <stage_report.txt>
"""
import argparse
import json
import multiprocessing as mp
import os
import pathlib
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path as _LivoPath

import numpy as np

for _cand in (_LivoPath(__file__).resolve().parents[1] / "tools", _LivoPath("/home/project/MY-LIVO/tools")):
    if (_cand / "livo" / "rosenv.py").exists():
        sys.path.insert(0, str(_cand))
        break
from livo import rosenv as _rosenv  # noqa: E402
_rosenv.ensure()
from livo.dataset import Dataset  # noqa: E402
from livo.body_filter import normalize, rear_axle_mask  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from scipy.spatial.transform import Rotation as R, Slerp  # noqa: E402

REFERENCE_TOPIC, REFERENCE_KEY = "front_lidar", "transform_matrix_lidar_6"
INS_TOPIC, STATUS_TOPIC = "imu_data/odometry", "imu_data/ins_status"
RTK_FIXED = 4                       # ins_pos_mode of an RTK-fixed INS solution
BODY_FILTER = normalize({})  # vehicle-specific body box; loaded from calibration.json in main()
COARSE_ITERS, MAX_ITERS = 8, 24
# quality gate: a correction beyond these limits is not a calibration but a wrong reference/recording
GATE_MAX_ROT_DEG, GATE_MAX_TRANS_M, GATE_MAX_DT_S = 6.0, 0.6, 0.08
GATE_MAX_ABS_DT_S = 0.25   # |final time offset| beyond this is not a calibration but a wrong recording
MIN_USABLE_SCANS = 5
SHORT_RECORDING_S = 90.0
PROFILE = tuple(np.arange(-0.18, 0.0001, 0.03))  # candidate time-shift baselines for the auto-detect
WARN_SPLIT_ROT_DEG, WARN_SPLIT_TRANS_M, WARN_SPLIT_DT_S = 0.2, 0.2, 0.01
QUERY_WORKERS = -1  # KD-tree query threads of solve() (-1 = all cores; reduced inside parallel workers)
_FORK = {}


def parallel_map(fn, items, jobs):
    """[fn(x) for x in items] in forked worker processes (fn and the arrays it reads are inherited, not pickled).

    solve() is dominated by single-threaded numpy work (KD-tree build, batched 3x3 eigen-decompositions), so the
    independent solves of this stage (time-offset candidates, repeatability halves) ran one after another on one
    core: 7 candidates x 14 iterations were ~70 % of the stage. Same seeds and data -> same results as sequential."""
    items = list(items)
    jobs = max(1, min(int(jobs), len(items)))
    if jobs == 1:
        return [fn(x) for x in items]
    _FORK.update(fn=fn, query_workers=max(1, (os.cpu_count() or 1) // jobs))
    with ProcessPoolExecutor(jobs, mp_context=mp.get_context("fork")) as pool:
        return list(pool.map(_forked_call, items))


def default_jobs():
    """<= 8 workers and ~1.5 GB of available memory each: a forked solve holds its own KD-trees and neighbour
    arrays (truck29 180 s window: 7 workers 1.2 GB each on top of the 2.2 GB parent)."""
    try:
        avail_gb = next(int(l.split()[1]) for l in open("/proc/meminfo") if l.startswith("MemAvailable:")) / 1e6
    except (OSError, StopIteration, ValueError):
        avail_gb = 8.0
    return max(1, min(8, os.cpu_count() or 1, int((avail_gb - 2.0) / 1.5)))


def _forked_call(item):
    global QUERY_WORKERS
    QUERY_WORKERS = _FORK["query_workers"]
    return _FORK["fn"](item)


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def voxel_index(P, size):
    _, idx = np.unique(np.floor(P / size).astype(np.int64), axis=0, return_index=True)
    return idx


def read_recording(bag, lidar_offset, window, voxel, rmin, rmax, every=2):
    """INS rear-axle poses + status and the reference scans (LiDAR frame, per-point relative time)."""
    ins_t, ins_p, ins_q, st_t, st_m, scans = [], [], [], [], [], []
    t0 = None; n_ref = 0
    for topic, data, _ in Dataset.open(bag).iter_messages([INS_TOPIC, STATUS_TOPIC, REFERENCE_TOPIC]):
        topic = topic.lstrip("/")
        if topic == INS_TOPIC:
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            ins_t.append(stamp(m)); ins_p.append((p.x, p.y, p.z)); ins_q.append((q.x, q.y, q.z, q.w))
        elif topic == STATUS_TOPIC:
            m = deserialize_message(data, DiagnosticArray)
            values = {i.key: i.value for s in m.status for i in s.values}
            try:
                mode = int(float(values.get("ins_pos_mode", "0") or 0))
            except ValueError:
                mode = 0
            st_t.append(stamp(m)); st_m.append(mode)
        elif topic == REFERENCE_TOPIC:
            m = deserialize_message(data, PointCloud2)
            h = stamp(m) + lidar_offset
            t0 = h if t0 is None else t0
            if h - t0 > window[1] + 1.0:
                break
            if not window[0] <= h - t0 <= window[1]:
                continue
            n_ref += 1
            if (n_ref - 1) % every:
                continue
            a = pc2.read_points(m, field_names=["x", "y", "z", "timestamp"], skip_nans=True)
            P = np.stack([a["x"], a["y"], a["z"]], 1).astype(float)
            ts = np.asarray(a["timestamp"], dtype=float)
            if len(ts) == 0:
                continue
            if ts.max() > 1e6: ts = ts - ts.min()                               # absolute -> relative
            if ts.max() > 1.0: ts = ts * 1e-3 if ts.max() < 1e3 else ts * 1e-9  # ms or ns
            r = np.linalg.norm(P, axis=1)
            keep = (r > rmin) & (r < rmax)
            P, ts = P[keep], ts[keep]
            sub = voxel_index(P, voxel)
            scans.append((P[sub], ts[sub], h, h - t0))
    if not ins_t:
        sys.exit(f"{bag}: no {INS_TOPIC} messages (the stage needs the INS/RTK rear-axle pose)")
    t = np.asarray(ins_t); order = np.argsort(t)
    t, p, q = t[order], np.asarray(ins_p)[order], np.asarray(ins_q)[order]
    uniq = np.r_[True, np.diff(t) > 1e-6]
    ins = {"t": t[uniq], "p": p[uniq], "q": q[uniq], "st_t": np.asarray(st_t), "st_m": np.asarray(st_m), "t0": t0}
    return ins, scans


class InsPoses:
    """Rear-axle pose of the INS (world ENU <- vehicle RFU) with the SLAM's time conventions."""
    def __init__(self, ins, dt_pos, dt_att):
        self.t, self.p, self.dt_pos, self.dt_att = ins["t"], ins["p"], dt_pos, dt_att
        self.slerp = Slerp(self.t, R.from_quat(ins["q"]))
        if len(ins["st_t"]):
            self.mode = ins["st_m"][np.clip(np.searchsorted(ins["st_t"], self.t), 0, len(ins["st_t"]) - 1)]
        else:  # no status topic: accept every sample (reported)
            self.mode = np.full(len(self.t), RTK_FIXED)
        j = np.clip(np.arange(len(self.t)), 10, len(self.t) - 11)
        self.speed = np.linalg.norm(self.p[j + 10] - self.p[j - 10], axis=1) / np.maximum(self.t[j + 10] - self.t[j - 10], 1e-3)
        moving = self.speed > 0.5
        self.t_move = self.t[np.argmax(moving)] if moving.any() else np.inf

    def at(self, tt):
        pos = np.stack([np.interp(tt + self.dt_pos, self.t, self.p[:, k]) for k in range(3)], 1)
        return self.slerp(tt + self.dt_att).as_matrix(), pos

    def rates(self, tt, h=0.05):
        """World-frame velocity of the rear axle and angular velocity (for the time-shift Jacobian)."""
        lo, hi = self.t[0], self.t[-1]
        vel = (np.stack([np.interp(tt + self.dt_pos + h, self.t, self.p[:, k]) for k in range(3)], 1)
               - np.stack([np.interp(tt + self.dt_pos - h, self.t, self.p[:, k]) for k in range(3)], 1)) / (2 * h)
        rot = self.slerp(np.clip(tt + self.dt_att + h, lo, hi)) * self.slerp(np.clip(tt + self.dt_att - h, lo, hi)).inv()
        return vel, rot.as_rotvec() / (2 * h)

    def sample_ok(self, h):
        j = min(np.searchsorted(self.t, h), len(self.t) - 1)
        return self.mode[j] == RTK_FIXED, self.speed[j]


def assemble(scans, poses, T_ref, vmin, motion_age):
    """Per-point LiDAR coordinates, INS world pose, time and pose rates of the usable scans."""
    parts = {k: [] for k in ("P", "R", "t", "tau", "vel", "om", "scan")}
    skipped = {"slow": 0, "startup": 0, "not_rtk_fixed": 0, "outside_ins": 0}
    lo, hi = poses.t[0], poses.t[-1]
    margin = max(abs(poses.dt_pos), abs(poses.dt_att)) + 0.2
    for i, (P, ts, h, rel) in enumerate(scans):
        tt = h + ts
        if tt.min() - margin < lo or tt.max() + margin > hi:
            skipped["outside_ins"] += 1; continue
        fixed, speed = poses.sample_ok(h)
        if speed < vmin:
            skipped["slow"] += 1; continue
        if h < poses.t_move + motion_age:   # single-antenna heading re-aligns after motion starts
            skipped["startup"] += 1; continue
        if not fixed:
            skipped["not_rtk_fixed"] += 1; continue
        Q = P @ T_ref[:3, :3].T + T_ref[:3, 3]
        body = rear_axle_mask(Q, BODY_FILTER)
        P, tt = P[~body], tt[~body]
        Rw, tw = poses.at(tt)
        vel, om = poses.rates(tt)
        for k, v in (("P", P), ("R", Rw), ("t", tw), ("tau", tt), ("vel", vel), ("om", om), ("scan", np.full(len(P), i))):
            parts[k].append(v)
    if not parts["P"]:
        return None, skipped
    data = {k: (np.concatenate(v) if k in ("tau", "scan") else np.vstack(v)) for k, v in parts.items()}
    data["R"] = data["R"].reshape(-1, 3, 3)
    return data, skipped


def solve(data, T0, active, *, solve_dt, gap, nq, K=20, prior=(5.0, 0.5, 0.1), iters=MAX_ITERS, fine=False, seed=0, dt0=0.0):
    """Joint robust GN for the rig correction (rotvec about the LiDAR position, translation) + time shift."""
    rng = np.random.default_rng(seed)
    P, Rw, tw, tau, vel, om = data["P"], data["R"], data["t"], data["tau"], data["vel"], data["om"]
    N = len(P)
    T = T0.copy(); total = np.zeros(7); total[6] = dt0; hist = []
    prior_c = np.zeros(7); prior_c[6] = dt0  # prior pulls toward dt0, not toward 0
    act = np.r_[active, bool(solve_dt)]
    Hp = np.diag([1 / np.radians(prior[0]) ** 2] * 3 + [1 / prior[1] ** 2] * 3 + [1 / prior[2] ** 2])
    H = np.zeros((7, 7)); quiet = 0
    sel = np.sort(rng.choice(N, min(nq, N), replace=False))   # fixed residual sample: plain GN, clean convergence
    for it in range(iters):
        f = 1.0 if fine else min(1.0, it / COARSE_ITERS)
        radius, lam, huber = 1.0 - 0.5 * f, 0.12 - 0.08 * f, 0.20 - 0.15 * f
        c = T[:3, 3]
        q = P @ T[:3, :3].T + c
        Rq = np.einsum("nij,nj->ni", Rw, q)
        pv = vel + np.cross(om, Rq)                  # d(world point)/d(time shift)
        pw = Rq + tw + total[6] * pv
        tree = cKDTree(pw)
        H = np.zeros((7, 7)); g = np.zeros(7); res, nz, gaps = [], [], []
        for ch in np.array_split(sel, max(1, len(sel) // 40000)):
            d, nb = tree.query(pw[ch], k=K, distance_upper_bound=radius, workers=QUERY_WORKERS)
            fin = np.isfinite(d); nbc = np.where(fin, nb, 0)
            valid = fin & (np.abs(tau[nbc] - tau[ch][:, None]) > gap)
            cnt = valid.sum(1); ok = cnt >= 6
            ch, nbc, valid, cnt = ch[ok], nbc[ok], valid[ok], cnt[ok]
            if not len(ch):
                continue
            X = pw[nbc]; wv = valid[..., None].astype(float)
            cen = (X * wv).sum(1) / cnt[:, None]
            D = (X - cen[:, None]) * wv
            ev, evec = np.linalg.eigh(np.einsum("nki,nkj->nij", D, D) / cnt[:, None, None])
            n = evec[:, :, 0]
            planar = (ev[:, 0] < lam ** 2) & (ev[:, 1] > 3 * ev[:, 0])
            ch, nbc, valid, cnt, n, cen = ch[planar], nbc[planar], valid[planar], cnt[planar], n[planar], cen[planar]
            r = np.einsum("ni,ni->n", n, pw[ch] - cen)
            # d r / d(rotvec about c) = (s x R^T n), d r / d t = R^T n, both for the point and (averaged) its plane
            mi = np.einsum("nji,nj->ni", Rw[ch], n)
            Ji = np.hstack([np.cross(q[ch] - c, mi), mi, np.einsum("ni,ni->n", n, pv[ch])[:, None]])
            mj = np.einsum("nkji,nj->nki", Rw[nbc], n)
            Jj = np.concatenate([np.cross(q[nbc] - c, mj), mj, np.einsum("nki,ni->nk", pv[nbc], n)[..., None]], -1)
            J = Ji - (Jj * valid[..., None]).sum(1) / cnt[:, None]
            w = np.minimum(1.0, huber / np.maximum(np.abs(r), 1e-9))
            H += (J * w[:, None]).T @ J; g += (J * w[:, None]).T @ r
            res.append(r); nz.append(np.abs(n[:, 2]))
            gaps.append((np.abs(tau[nbc] - tau[ch][:, None]) * valid).sum(1) / cnt)
        if not res:
            return None
        r = np.concatenate(res); nzz = np.concatenate(nz)
        a = np.flatnonzero(act)
        step = np.zeros(7)
        step[a] = -np.linalg.solve((H + Hp)[np.ix_(a, a)], (g + Hp @ (total - prior_c))[a])
        T[:3, :3] = R.from_rotvec(step[:3]).as_matrix() @ T[:3, :3]
        T[:3, 3] = T[:3, 3] + step[3:6]
        total += step
        gm, vm = nzz > 0.8, nzz < 0.3
        hist.append({"iter": it + 1, "matches": int(len(r)), "median_cm": float(100 * np.median(np.abs(r))),
                     "ground_cm": float(100 * np.median(np.abs(r[gm]))) if gm.any() else float("nan"), "ground_n": int(gm.sum()),
                     "vertical_cm": float(100 * np.median(np.abs(r[vm]))) if vm.any() else float("nan"), "vertical_n": int(vm.sum()),
                     "gap_s": float(np.median(np.concatenate(gaps)))})
        small = (np.degrees(np.linalg.norm(step[:3])) < 0.003 and np.linalg.norm(step[3:6]) < 1e-3 and abs(step[6]) < 2e-4)
        quiet = quiet + 1 if (small and f >= 1.0) else 0
        if quiet >= 2:
            break
    return {"T": T, "dt": float(total[6]), "hist": hist, "H": H, "converged": quiet >= 2}


def correction(T_new, T_old):
    """Rig correction C = T_new * T_old^-1 and its readable form (rotvec deg in rear frame about the LiDAR, dt cm)."""
    C = T_new @ np.linalg.inv(T_old)
    return C, np.degrees(R.from_matrix(C[:3, :3]).as_rotvec()), 100 * (T_new[:3, 3] - T_old[:3, 3])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibration", required=True); ap.add_argument("--bag", required=True)
    ap.add_argument("--output", required=True); ap.add_argument("--report", required=True)
    ap.add_argument("--window", nargs=2, type=float, default=(0.0, 1e9), help="seconds after the first reference scan (default: all)")
    ap.add_argument("--dof", choices=("rpy_xy", "rpy", "xy", "x"), default="rpy_xy",
                    help="parts of the solution that are APPLIED (rotation + horizontal translation are always solved "
                         "together: a fixed, inconsistent rotation would leak into the lever arm). rpy_xy: all (default, "
                         "needed for a raw supplier file); rpy: rotation only; xy: lever arm only (fine-tuning an aligned version)")
    ap.add_argument("--keep-time-offset", action="store_true",
                    help="do not estimate/apply the LiDAR time shift (the extrinsic is solved with the parent's time offsets)")
    ap.add_argument("--gap", type=float, default=1.5, help="minimum time between a point and the returns of its plane (s)")
    ap.add_argument("--voxel", type=float, default=0.25); ap.add_argument("--max-range", type=float, default=80.0)
    ap.add_argument("--min-speed", type=float, default=1.0); ap.add_argument("--motion-age", type=float, default=5.0)
    ap.add_argument("--every", type=int, default=2, help="read every n-th reference scan (default 2 = 5 Hz)")
    ap.add_argument("--max-scans", type=int, default=700, help="scans used at most (evenly thinned)")
    ap.add_argument("--samples", type=int, default=200000, help="residuals sampled per iteration")
    ap.add_argument("--no-split", action="store_true", help="skip the two half-recording repeatability solves")
    ap.add_argument("--no-profile", action="store_true", help="skip the fixed-dt time-offset profile (start the joint solve at dt=0)")
    ap.add_argument("--no-gate", action="store_true", help="write the output even when the quality gate fails")
    ap.add_argument("--jobs", type=int, default=default_jobs(),
                    help="worker processes for the independent solves (time-offset candidates, halves); 1 = sequential "
                         "(default: <= 8, limited by the available memory, ~1.5 GB per worker)")
    a = ap.parse_args()
    t_start = time.time()
    global BODY_FILTER
    calib = json.load(open(a.calibration))
    BODY_FILTER = normalize(calib)
    T_ref = None
    for entry in calib.get("lidar", []):
        if REFERENCE_KEY in entry:
            T_ref = np.asarray(entry[REFERENCE_KEY], float).reshape(4, 4)
    if T_ref is None:
        sys.exit(f"{a.calibration}: reference {REFERENCE_KEY} missing")
    offs = calib.get("time_offsets") or {}
    lidar_off = float(offs.get("lidar", -0.125)); dt_pos = float(offs.get("rtk_position", 0.025)); dt_att = float(offs.get("ins_attitude", 0.0))

    ins, scans = read_recording(a.bag, lidar_off, a.window, a.voxel, 3.0, a.max_range, a.every)
    poses = InsPoses(ins, dt_pos, dt_att)
    rec_span_s = float(scans[-1][3] - scans[0][3]) if len(scans) >= 2 else 0.0
    short_recording = rec_span_s < SHORT_RECORDING_S
    data, skipped = assemble(scans, poses, T_ref, a.min_speed, a.motion_age)
    def _span_s(d):
        if d is None:
            return 0.0
        u = np.unique(d["scan"])
        return float(scans[u[-1]][3] - scans[u[0]][3]) if len(u) >= 2 else 0.0
    if short_recording and (data is None or len(np.unique(data["scan"])) < 50 or _span_s(data) < 20.0):
        for spd_fb, age_fb in ((min(a.min_speed, 0.5), min(a.motion_age, 1.0)), (min(a.min_speed, 0.2), 0.0)):
            data_fb, skipped_fb = assemble(scans, poses, T_ref, spd_fb, age_fb)
            if data_fb is not None and (data is None or len(np.unique(data_fb["scan"])) > len(np.unique(data["scan"]))):
                data, skipped = data_fb, skipped_fb
            if data is not None and len(np.unique(data["scan"])) >= 50 and _span_s(data) >= 15.0:
                break
    if data is not None and len(np.unique(data["scan"])) > a.max_scans:
        keep_scans = np.unique(data["scan"])[:: int(np.ceil(len(np.unique(data["scan"])) / a.max_scans))]
        m = np.isin(data["scan"], keep_scans)
        data = {k: v[m] for k, v in data.items()}
    report = [f"calibration={a.calibration}", f"bag={a.bag}",
              f"reference {REFERENCE_TOPIC} ({REFERENCE_KEY}); poses {INS_TOPIC} (RTK fixed, >= {a.min_speed} m/s, "
              f">= {a.motion_age} s after motion start); time offsets lidar {lidar_off:+.4f} s, rtk_position {dt_pos:+.3f} s, ins_attitude {dt_att:+.3f} s",
              f"scans read {len(scans)}, skipped {skipped}"]
    min_scans = MIN_USABLE_SCANS if short_recording else 50
    if data is None or len(np.unique(data["scan"])) < min_scans:
        if short_recording:
            report.append(f"fewer than {min_scans} usable scans: parent calibration kept (quality: warn)")
            pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
            pathlib.Path(a.report).write_text("\n".join(report) + "\n"); print("\n".join(report), flush=True)
            out = json.loads(json.dumps(calib))
            out["calibration_type"] = calib.get("calibration_type", "") + "+rtk_rear_axle_alignment"
            out.setdefault("refinement", {})["rear_axle_alignment"] = {
                "reference": REFERENCE_KEY, "poses": f"{INS_TOPIC} (RTK fixed)", "scans_used": 0,
                "dof": a.dof, "converged": False, "quality": "warn",
                "reasons": [f"fewer than {min_scans} usable scans; parent calibration kept"]}
            pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True)
            json.dump(out, open(a.output, "w"), indent=2)
            print(f"written {a.output} ({time.time() - t_start:.0f} s)")
            return
        report.append("fewer than 50 usable scans: the recording needs RTK-fixed driving (>= 1 m/s) with a turn")
        pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(a.report).write_text("\n".join(report) + "\n"); print("\n".join(report)); sys.exit(3)
    used = np.unique(data["scan"])
    rel = np.array([scans[i][3] for i in used])
    report.append(f"scans used {len(used)} ({rel.min():.1f}..{rel.max():.1f} s), points {len(data['P'])}; dof {a.dof}"
                  f"{'' if a.keep_time_offset else ' + LiDAR time shift'}; cross-time gap > {a.gap} s")
    active = np.array([True] * 5 + [False])   # z is unobservable (moves the whole map vertically)
    solve_dt = not a.keep_time_offset
    # Time-offset auto-detection: profile the residual over a grid of FIXED time shifts (extrinsics
    # re-solved at each) and start the joint solve at the minimum. A parent with no/wrong offset (raw
    # import, offsets left unset) starts far from the truth, where the joint solve couples rotation and
    # time into a worse local minimum; the profile is cheap, unimodal at plausible offsets and doubles
    # as the documented evidence for the measured value.
    profile, T_start, dt_start, gts_final = [], T_ref, 0.0, 0.0
    if solve_dt and not a.no_profile:
        # Time-offset auto-detection. The first-order time-shift model inside solve() is only valid
        # for |dt| <~ 40 ms (turn curvature), so a far parent offset biases any single solve toward
        # zero. Candidate baselines RE-POSE the points at exactly shifted times and each runs a small
        # local joint solve: near the true offset the solve reaches the deep minimum, far from it the
        # residual stays shallow, so the candidates rank the offset exactly.
        def shifted(gts):
            d2 = dict(data)
            Rw2, tw2 = poses.at(data["tau"] + gts)
            d2["R"], d2["t"] = Rw2, tw2
            return d2
        def solve_candidate(gts):
            s_ = solve(shifted(float(gts)), T_ref, active, solve_dt=True, gap=a.gap,
                       nq=max(a.samples // 2, 100000), iters=14, fine=False, seed=1, dt0=0.0)
            if s_ is None or not s_["hist"]:
                return None
            return {"baseline_s": round(float(gts), 4), "dt_s": round(float(gts) + s_["dt"], 4),
                    "median_cm": round(s_["hist"][-1]["median_cm"], 3),
                    "ground_cm": round(s_["hist"][-1]["ground_cm"], 3), "T": s_["T"]}
        def candidate(*values):
            profile.extend(p_ for p_ in parallel_map(solve_candidate, values, a.jobs) if p_ is not None)
        candidate(*PROFILE)
        # The default grid covers truck29-like offsets (-0.18..0). Other vehicles
        # can sit outside it (truck05: effective offset near/above 0); while the
        # best candidate lies on a grid edge, extend the grid in that direction
        # so the detected offset is an interior minimum, never a grid boundary
        # artifact silently far from the truth.
        step, limit = 0.03, 0.30
        while profile and len(profile) <= 25:
            count = len(profile)
            bases = sorted(p_["baseline_s"] for p_ in profile)
            best = min(profile, key=lambda q_: q_["median_cm"])
            lo, hi = bases[0], bases[-1]
            if best["baseline_s"] <= lo + 1e-9 and lo - step >= -limit:
                candidate(lo - step)
            elif best["baseline_s"] >= hi - 1e-9 and hi + step <= limit:
                candidate(hi + step)
            if len(profile) == count:
                break
        if profile:
            best = min(profile, key=lambda q_: q_["median_cm"])
            gts_final, T_start = best["dt_s"], best["T"]
            data = shifted(gts_final)   # exact re-pose at the detected offset; solve() refines around 0
            report.append("time-offset candidates (exact re-pose baseline + local joint solve): "
                          + ", ".join(f"{p_['baseline_s']:+.3f}/{p_['dt_s']:+.3f}s:{p_['median_cm']:.2f}cm"
                                      for p_ in profile))
            report.append(f"detected time shift {gts_final:+.3f} s (median {best['median_cm']:.2f} cm) -> final joint solve")
    sol = solve(data, T_start, active, solve_dt=solve_dt, gap=a.gap, nq=a.samples, dt0=dt_start)
    dt_total = gts_final + sol["dt"] if solve_dt else 0.0
    if sol is None:
        report.append("no planar cross-time matches"); pathlib.Path(a.report).write_text("\n".join(report) + "\n")
        print("\n".join(report)); sys.exit(3)
    C_full, rv_full, dt_full = correction(sol["T"], T_ref)
    T_apply = T_ref.copy()
    if a.dof in ("rpy_xy", "rpy"):
        T_apply[:3, :3] = sol["T"][:3, :3]
    if a.dof in ("rpy_xy", "xy"):
        T_apply[:3, 3] = sol["T"][:3, 3]
    elif a.dof == "x":
        # Lateral lever arm only (the joint solve still estimates every DoF). Round trips make the lateral offset
        # directly observable (a body-fixed lateral error separates opposite passes by twice its value), while the
        # forward offset of a round trip is indistinguishable from an along-track timing error (RTK association /
        # INS latency) and stays with the window solution and the rtk-time stage.
        T_apply[0, 3] = sol["T"][0, 3]
    C, rv, dtcm = correction(T_apply, T_ref)
    h0, h1 = sol["hist"][0], sol["hist"][-1]
    report.append(f"residual median {h0['median_cm']:.2f} -> {h1['median_cm']:.2f} cm (ground {h0['ground_cm']:.2f} -> {h1['ground_cm']:.2f}, "
                  f"n={h1['ground_n']}; vertical {h0['vertical_cm']:.2f} -> {h1['vertical_cm']:.2f}, n={h1['vertical_n']}); "
                  f"match time gap median {h1['gap_s']:.1f} s; iterations {h1['iter']}{'' if sol['converged'] else ' (NOT converged)'}")
    report.append(f"solution about the LiDAR (rear-axle frame): rotvec x(pitch) {rv_full[0]:+.3f}, y(roll) {rv_full[1]:+.3f}, "
                  f"z(yaw) {rv_full[2]:+.3f} deg; translation x {dt_full[0]:+.1f}, y {dt_full[1]:+.1f} cm")
    report.append(f"applied ({a.dof}): rotvec x(pitch) {rv[0]:+.3f}, y(roll) {rv[1]:+.3f}, z(yaw) {rv[2]:+.3f} deg; "
                  f"translation x {dtcm[0]:+.1f}, y {dtcm[1]:+.1f}, z {dtcm[2]:+.1f} cm"
                  + ("" if a.keep_time_offset else f"; LiDAR time shift {1000 * dt_total:+.1f} ms -> time_offsets.lidar {lidar_off + dt_total:+.4f} s"))
    # observability: square-root information per DoF (unit weights, relative scale only)
    idx = np.flatnonzero(np.r_[active, solve_dt])
    info = {n_: float(np.sqrt(max(sol["H"][i, i], 0))) for n_, i in zip(("rx", "ry", "rz", "tx", "ty", "tz", "dt"), range(7)) if i in idx}
    split = None
    large_translation = np.linalg.norm(dt_full[:2]) / 100 > GATE_MAX_TRANS_M
    translation_confirmed = False
    if not a.no_split:
        mid = np.median(rel)
        parts = (used[rel <= mid], used[rel > mid])
        def solve_half(i):
            m = np.isin(data["scan"], parts[i])
            # Large factory corrections need independent starts, not refinements of the full-data solution.
            return solve({k: v[m] for k, v in data.items()}, T_ref if large_translation else sol["T"], active,
                         solve_dt=solve_dt, gap=a.gap, nq=a.samples // 2,
                         iters=MAX_ITERS if large_translation else 10, fine=not large_translation,
                         seed=1, dt0=float(sol["dt"]))
        halves = parallel_map(solve_half, (0, 1), a.jobs)
        if all(s is not None for s in halves):
            d_rot = np.degrees(R.from_matrix(halves[0]["T"][:3, :3] @ halves[1]["T"][:3, :3].T).as_rotvec())
            d_tr = halves[0]["T"][:3, 3] - halves[1]["T"][:3, 3]
            split = {"rotvec_deg": np.round(d_rot, 3).tolist(), "translation_cm": np.round(100 * d_tr, 1).tolist(),
                     "time_ms": round(1000 * (halves[0]["dt"] - halves[1]["dt"]), 1)}
            translation_confirmed = (
                sol["converged"] and all(s["converged"] for s in halves)
                and np.linalg.norm(d_tr[:2]) <= 0.03
                and np.linalg.norm(d_rot) <= WARN_SPLIT_ROT_DEG
                and abs(halves[0]["dt"] - halves[1]["dt"]) <= WARN_SPLIT_DT_S
                and all(np.linalg.norm((s["T"][:3, 3] - sol["T"][:3, 3])[:2]) <= 0.03
                        and s["hist"][-1]["median_cm"] <= 3.0 for s in halves)
                and h1["median_cm"] <= 3.0)
            report.append(f"repeatability (first vs second half of the used scans): rotvec {split['rotvec_deg']} deg, "
                          f"translation {split['translation_cm']} cm, time {split['time_ms']:+.1f} ms")
    # quality verdict
    reasons, level = [], "ok"
    rot_deg, trans_m = float(np.linalg.norm(rv_full)), float(np.linalg.norm(dt_full) / 100)
    if rot_deg > GATE_MAX_ROT_DEG: reasons.append(f"rotation correction {rot_deg:.2f} deg > {GATE_MAX_ROT_DEG}"); level = "fail"
    if trans_m > GATE_MAX_TRANS_M:
        if translation_confirmed:
            reasons.append(f"translation correction {100 * trans_m:.0f} cm confirmed by independent halves (<= 3 cm)")
            level = "warn" if level == "ok" else level
        else:
            reasons.append(f"translation correction {100 * trans_m:.0f} cm > {100 * GATE_MAX_TRANS_M:.0f}")
            level = "fail"
    abs_dt = abs(lidar_off + dt_total) if solve_dt else abs(lidar_off)
    if solve_dt and abs_dt > GATE_MAX_ABS_DT_S:
        reasons.append(f"absolute LiDAR time offset {1000 * abs_dt:.0f} ms implausible (> {1000 * GATE_MAX_ABS_DT_S:.0f})"); level = "fail"
    if solve_dt and abs(dt_total) > GATE_MAX_DT_S:
        reasons.append(f"time shift {1000 * dt_total:+.0f} ms is a large change vs the parent offset (candidates/longer recording confirms it)")
        level = "warn" if level == "ok" else level
    if h1["median_cm"] > h0["median_cm"] + 0.05: reasons.append("residual did not improve"); level = "fail"
    if not sol["converged"]:
        reasons.append(f"not converged in {MAX_ITERS} iterations"); level = "warn" if level == "ok" else level
    if split:
        if np.linalg.norm(split["rotvec_deg"]) > WARN_SPLIT_ROT_DEG or np.linalg.norm(split["translation_cm"][:2]) > 100 * WARN_SPLIT_TRANS_M \
                or abs(split["time_ms"]) > 1000 * WARN_SPLIT_DT_S:
            reasons.append("halves disagree (few turns/structure: use a longer recording with turns)")
            level = "warn" if level == "ok" else level
    if level == "fail" and not a.no_gate:
        if short_recording:
            reasons.append("short/under-constrained recording: quality gate relaxed to warn")
            level = "warn"
        else:
            print("rear-axle alignment rejected by the quality gate; calibration.json not written (--no-gate overrides)")
            sys.exit(3)
    report.append(f"quality: {level}" + (f"  ({'; '.join(reasons)})" if reasons else ""))
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    pathlib.Path(a.report).write_text("\n".join(report) + "\n")
    print("\n".join(report), flush=True)
    out = json.loads(json.dumps(calib))
    for entry in out["lidar"]:
        for k in list(entry):
            if k.startswith("transform_matrix_lidar_"):
                entry[k] = (C @ np.asarray(entry[k], float).reshape(4, 4)).reshape(-1).tolist()
    cam = (out.get("transforms") or {}).get("midrange_camera_to_rear_axle")
    if cam and cam.get("transform_matrix"):
        cam["transform_matrix"] = (C @ np.asarray(cam["transform_matrix"], float).reshape(4, 4)).reshape(-1).tolist()
    if solve_dt:
        out.setdefault("time_offsets", {})["lidar"] = round(lidar_off + dt_total, 4)
        out["time_offsets"]["lidar_source"] = "rear-axle alignment (cross-time consistency vs INS poses)"
    out["calibration_type"] = calib.get("calibration_type", "") + "+rtk_rear_axle_alignment"
    out.setdefault("refinement", {})["rear_axle_alignment"] = {
        "reference": REFERENCE_KEY, "poses": f"{INS_TOPIC} (RTK fixed)", "applied_to": "all LiDARs and the camera (rig correction)",
        "solution_rotvec_deg": np.round(rv_full, 4).tolist(), "solution_translation_cm": np.round(dt_full, 2).tolist(),
        "correction_rotvec_deg": np.round(rv, 4).tolist(), "correction_translation_cm": np.round(dtcm, 2).tolist(),
        "correction_matrix": C.reshape(-1).tolist(), "time_shift_s": None if a.keep_time_offset else round(dt_total, 4),
        "time_offsets_used": {"lidar": lidar_off, "rtk_position": dt_pos, "ins_attitude": dt_att},
        "residual_cm": {"before": round(h0["median_cm"], 3), "after": round(h1["median_cm"], 3),
                        "ground_after": round(h1["ground_cm"], 3), "vertical_after": round(h1["vertical_cm"], 3)},
        "sqrt_information": {k: round(v, 1) for k, v in info.items()}, "repeatability_halves": split,
        "time_profile": [{k: v for k, v in p_.items() if k != "T"} for p_ in profile],
        "time_profile_min_dt_s": round(gts_final, 4) if profile else None,
        "scans_used": int(len(used)), "window_s": [round(float(rel.min()), 1), round(float(rel.max()), 1)], "dof": a.dof,
        "iterations": h1["iter"], "converged": sol["converged"], "quality": level, "reasons": reasons,
        "runtime_s": round(time.time() - t_start, 1)}
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(a.output, "w"), indent=2)
    print(f"written {a.output} ({time.time() - t_start:.0f} s)")


if __name__ == "__main__":
    main()
