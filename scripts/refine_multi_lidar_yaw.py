#!/usr/bin/env python3
"""Stage-2 multi-LiDAR extrinsic refinement (all 6 DoF) using a driven window and the run's own
LIO poses. Each LiDAR's raw returns are linearly deskewed with the per-frame poses and
accumulated in the world frame; every non-reference LiDAR is then registered (robust point-to-
plane ICP, correction applied in the vehicle frame about the LiDAR position) against the
reference LiDAR's accumulated cloud. Vertical structures (poles, plates, berms, walls) observe
yaw/xy; ground observes roll/pitch/z. Output: a new calibration JSON; inputs are never modified.

Two passes, two pose sources:
  pass 1 (all LiDARs, 6 DoF)   native frames.csv poses. They are locally the smoothest poses
        available, so same-time overlap (side LiDARs vs the front LiDAR) is registered best.
        But they drift (this bag: ~0.7 cm/s in z): a LiDAR whose overlap with the reference is
        only cross-time (the back LiDAR sees the ground the front LiDAR saw 5-15 s earlier)
        absorbs that drift into its z/pitch/roll.
  pass 2 (cross-time LiDARs, tilt about x/y and z only)   drift-free RTK-optimized poses
        (trajectory_optimized.txt), ground matches only, started from the pass-1 solution.
        Applied to a LiDAR when the median time gap between its points and their matched
        reference points exceeds --cross-time-s (default 2 s). Yaw/x/y stay from pass 1 (the
        few-cm RTK/INS jitter of the optimized poses would only add noise there).
  --ground-poses native disables pass 2 (previous behaviour).
  Matches are viewing-side consistent (default, --no-same-side disables): the target normals are
        oriented towards the reference LiDAR that observed them and a source point is matched only
        when its own LiDAR saw the plane from the same side. Without this, a LiDAR that looks the
        other way (the back LiDAR sees the rear faces of poles, sign posts, blocks and wall ends that
        the front LiDAR saw from the front) is pulled onto the opposite face: truck29 v064 solved
        the back LiDAR 4.3 cm further BACK although fit-free tests (facing planes, pole axes) put
        its returns 4-10 cm behind the front LiDARs already.
  --pass1-poses optimized   registers pass 1 (yaw/x/y, or all 6 DoF) with the RTK-optimized poses.
        Use it when the window is long or the LiDAR overlaps the reference only cross-time: the
        native along-track drift (truck29: 26-33 cm median xy error, ~1 cm/s) enters the x/y of a
        LiDAR whose matches are 3-10 s apart as a direct bias (v058: +6..+8 cm y for the side
        LiDARs), while the optimized poses are locally smooth (fused-native relative motion
        <=0.6 cm per second) and drift-free at the cm level.

Pairing (pass 1, LiDARs that overlap the reference at the same time): --pairing same-time (default) matches a
        return only against reference returns observed within --block-s/--pad-s of its own time (time-blocked
        targets, nearest-range observation per voxel). The accumulated target (--pairing accumulated, previous
        behaviour) keeps the FIRST observation per voxel, i.e. the reference LiDAR's far look-ahead, so the side
        LiDARs were matched to returns ~5 s older: the native trajectory error over 5 s (truck29raw 20261013:
        z drift ~0.7 cm/s on the ramp) entered their z/tilt (v018: both side LiDARs 4-7 cm low, 0.1-0.15 deg tilt;
        same-time residual 3.5/5.6 cm vs 1.2/1.4 cm after correction; map blocks 3 s apart tilted up to 0.27 deg).

A per-LiDAR quality verdict (convergence, vertical-structure support, residual regression,
correction magnitude) is written to the report/JSON; a `fail` verdict aborts the stage (exit 3)
without writing calibration.json unless --no-gate is given.

Usage (container):
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/refine_multi_lidar_yaw.py --calibration <parent calibration.json> \
      --bag /home/project/data/haibo/wuhu_livo/ros2bag --run <fusion_run_dir> --window 8 40 \
      --lidar-time-offset -0.125 --output <calibration.json> --report <stage_report.txt>
"""
import argparse, json, pathlib, numpy as np
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

KEYS = {"front_right_lidar": "transform_matrix_lidar_1", "back_lidar": "transform_matrix_lidar_2",
        "front_left_lidar": "transform_matrix_lidar_3", "front_lidar": "transform_matrix_lidar_6"}
REFERENCE = "front_lidar"
MAX_ITERS = 25
# quality gate thresholds (per LiDAR): the prior scale is the plausibility limit of a *refinement*
GATE_MAX_ROT_DEG = 0.5
GATE_MAX_TRANS_M = 0.15
GATE_MIN_VERTICAL = 2000         # vertical planar matches needed to trust yaw/x/y
GATE_VERT_REGRESS_M = 0.005      # vertical residual may not get worse by more than this
SHORT_RECORDING_S = 90.0


def voxel_down(P, size):
    """Index of the first point of every occupied voxel, in lexicographic voxel order. Same result as
    np.unique(key, axis=0, return_index=True), which sorts the rows as a structured dtype (~40 % of the stage on a
    180 s window); one packed int64 key per row sorts in a fraction of that."""
    key = np.floor(P / size).astype(np.int64)
    if len(key) == 0:
        return np.zeros(0, np.int64)
    key -= key.min(0)
    if key.max() >= 1 << 21:
        _, idx = np.unique(key, axis=0, return_index=True)
        return idx
    _, idx = np.unique((key[:, 0] << 42) | (key[:, 1] << 21) | key[:, 2], return_index=True)
    return idx


class Target:
    def __init__(self, A, times, k=12, sensors=None, max_sigma=0.05):
        self.A = A; self.times = times; self.tree = cKDTree(A)
        d, idx = self.tree.query(A, k=k, workers=-1); nbr = A[idx]; c = nbr.mean(1)
        cov = np.einsum("nki,nkj->nij", nbr - c[:, None], nbr - c[:, None]) / k
        w, v = np.linalg.eigh(cov)
        self.normals = v[:, :, 0]
        if sensors is not None:   # orient every normal towards the reference LiDAR that observed the point
            flip = np.einsum("ni,ni->n", sensors - A, self.normals) < 0
            self.normals[flip] *= -1
        self.oriented = sensors is not None
        self.planar = (w[:, 0] < max_sigma ** 2) & (w[:, 1] > 4 * w[:, 0] + 1e-9)

    def match(self, B, max_dist, sensors=None):
        """sensors: world position of the LiDAR that produced each B point; with oriented normals a match is
        kept only when that LiDAR saw the plane from the same side as the reference (no opposite-face pulls)."""
        dist, j = self.tree.query(B, distance_upper_bound=max_dist, workers=-1)
        ok = np.isfinite(dist); ok[ok] &= self.planar[j[ok]]
        if sensors is not None and self.oriented:
            idx = np.flatnonzero(ok)
            same = np.einsum("ni,ni->n", sensors[idx] - self.A[j[idx]], self.normals[j[idx]]) > 0
            ok[idx[~same]] = False
        return ok, self.normals[j[ok]], self.A[j[ok]], self.times[j[ok]]


class Poses:
    """Per-frame IMU poses with linear interpolation (rotation slerp) per point time."""
    def __init__(self, stamps, rot, pos, label):
        self.stamps, self.rot, self.pos, self.label = stamps, rot, pos, label

    def at(self, t):
        # Interpolated once per distinct time: archive scans stamp every return at the scan end, and the per-point
        # slerp of identical times was a third of the stage (same values, expanded by index).
        t, inv = np.unique(t, return_inverse=True)
        s = self.stamps
        j = np.clip(np.searchsorted(s, t), 1, len(s) - 1)
        ta, tb = s[j - 1], s[j]; r = np.clip((t - ta) / (tb - ta), 0, 1)
        ra, rb = self.rot[j - 1], self.rot[j]
        rr = ra * R.from_rotvec((ra.inv() * rb).as_rotvec() * r[:, None])
        return rr[inv], (self.pos[j - 1] * (1 - r)[:, None] + self.pos[j] * r[:, None])[inv]


def load_optimized(run, stamps):
    """trajectory_optimized.txt (mine frame, physical IMU, same stamps as frames.csv) or None."""
    path = run / "trajectory_optimized.txt"
    if not path.exists():
        return None
    arr = np.loadtxt(path, comments="#")
    if len(arr) != len(stamps) or not np.allclose(arr[:, 0], stamps, atol=1e-6):
        print(f"warning: {path} does not match frames.csv stamps ({len(arr)} vs {len(stamps)}); ignored", flush=True)
        return None
    return Poses(stamps, R.from_quat(arr[:, 4:8]), arr[:, 1:4], "RTK-optimized trajectory_optimized.txt")


def world_points(scans, M_t, lr, lt, poses, time_bin=None, max_points=600000):
    """Vehicle-frame points Q with their per-point world pose (rotation matrices Rm, positions Pw) and times.
    time_bin=None: one 0.15 m vehicle-frame voxel set over the whole window (previous behaviour). Every voxel then
    keeps its FIRST observation only, so a window that starts parked (truck29raw 20261013: 14 s) is represented
    almost entirely by the stationary start frames and the 6-DoF fit hardly sees the drive (side-LiDAR roll stayed
    biased by the start terrain: -0.13/+0.13 deg measured over the whole recording). time_bin > 0: voxels per time
    bin, uniformly thinned to max_points, so every part of the window contributes."""
    Qs, Rs, Ps, Ts = [], [], [], []
    for P, ts, h in scans:
        Q = P @ M_t[:3, :3].T + M_t[:3, 3]
        rr, pp = poses.at(h + ts)
        Qs.append(Q); Rs.append(rr.as_matrix()); Ps.append(pp); Ts.append(h + ts)
    Q = np.vstack(Qs); Rm = np.vstack(Rs).reshape(-1, 3, 3); Pw = np.vstack(Ps); T = np.concatenate(Ts)
    if not time_bin:
        sub = voxel_down(Q, 0.15)   # subsample in the vehicle frame keeps far structure
    else:
        # uniform in time at raw density: a density-equalising voxel over-weights the sparse far returns, and roll,
        # the weakest-determined DoF of a side LiDAR, then disagreed with the map/LIO weighting by ~0.05 deg
        sub = np.arange(len(Q)) if len(Q) <= max_points else np.linspace(0, len(Q) - 1, max_points).astype(np.int64)
    return Q[sub], Rm[sub], Pw[sub], T[sub]


def reference_world(scans, M_ref, lr, lt, poses):
    """Reference LiDAR returns in the world frame (per-point deskew) with time and observing-sensor position."""
    Ws, Ts, Ss = [], [], []
    sensor_imu = lr @ M_ref[:3, 3] + lt
    for P, ts, h in scans:
        B = (P @ M_ref[:3, :3].T + M_ref[:3, 3]) @ lr.T + lt
        rr, pp = poses.at(h + ts)
        Ws.append(rr.apply(B) + pp); Ts.append(h + ts); Ss.append(rr.apply(np.tile(sensor_imu, (len(B), 1))) + pp)
    return np.vstack(Ws), np.concatenate(Ts), np.vstack(Ss)


def nearest_first_voxels(W, S, voxel):
    """Voxel subsample keeping the observation nearest to its sensor (finest ring spacing, least angular error)."""
    order = np.argsort(np.einsum("ni,ni->n", W - S, W - S), kind="stable")
    return order[voxel_down(W[order], voxel)]


def build_target(scans, M_ref, lr, lt, poses, voxel, same_side=True, nearest_first=True):
    """Reference LiDAR accumulated in the world frame (per-point deskew), voxelised at `voxel`.
    With same_side the normals are oriented towards the reference LiDAR position at the observation time.
    nearest_first keeps the closest observation per voxel (False: first in time order, previous behaviour)."""
    W, T, S = reference_world(scans, M_ref, lr, lt, poses)
    sub = nearest_first_voxels(W, S, voxel) if nearest_first else voxel_down(W, voxel)
    return Target(W[sub], T[sub], sensors=S[sub] if same_side else None)


class BlockTarget:
    """Reference returns in time blocks: a source return is matched only against the block around its own time
    (same-time pairing), so the trajectory error between the two observations is that of <= block + 2 pad seconds."""
    def __init__(self, scans, M_ref, lr, lt, poses, voxel, block=1.0, pad=0.5, same_side=True, nearest=True, short=False):
        W, T, S = reference_world(scans, M_ref, lr, lt, poses)
        self.t0, self.block = float(T.min()), block
        n_blocks = int(np.ceil((T.max() - self.t0) / block)) + 1
        order = np.argsort(T, kind="stable"); W, T, S = W[order], T[order], S[order]
        for min_pts in ((500,) if not short else (500, 100)):
            self.blocks, self.count = [], 0
            for b in range(n_blocks):
                lo, hi = np.searchsorted(T, [self.t0 + b * block - pad, self.t0 + (b + 1) * block + pad])
                if hi - lo < min_pts:
                    self.blocks.append(None); continue
                sub = lo + (nearest_first_voxels(W[lo:hi], S[lo:hi], voxel) if nearest else voxel_down(W[lo:hi], voxel))
                tgt = Target(W[sub], T[sub], sensors=S[sub] if same_side else None, max_sigma=0.03)
                self.blocks.append(tgt); self.count += len(sub)
            if sum(x is not None for x in self.blocks) >= 5:
                break

    def match(self, B, max_dist, sensors=None, times=None):
        ok = np.zeros(len(B), bool); n = np.zeros((len(B), 3)); q = np.zeros((len(B), 3)); tq = np.zeros(len(B))
        bi = np.clip(((times - self.t0) // self.block).astype(int), 0, len(self.blocks) - 1)
        for b in np.unique(bi):
            tgt = self.blocks[b]
            if tgt is None: continue
            idx = np.flatnonzero(bi == b)
            okb, nb, qb, tb = tgt.match(B[idx], max_dist, sensors[idx] if sensors is not None else None)
            sel = idx[okb]; ok[sel] = True; n[sel] = nb; q[sel] = qb; tq[sel] = tb
        return ok, n[ok], q[ok], tq[ok]


def icp(Q, Rm, Pw, T, center, lr, lt, target, active, x0, prior, normal_filter=None, radii=(0.6,), min_matches=300):
    """Robust point-to-plane ICP of one LiDAR against the reference target.
    Correction x = (rotvec about the LiDAR position, translation) in the vehicle frame.
    radii: match radius per pair of iterations (last value kept); convergence only counts at the final radius.
    A fixed 0.6 m radius keeps matches on neighbouring surfaces that anchor the solution towards zero correction
    (same-time pass on truck29raw: side-LiDAR roll 0.04 deg with 0.6 m vs 0.09 deg with the 0.6/0.4/0.3 schedule)."""
    x = x0.copy(); stats = first = None; dt_med = np.nan
    sensors = np.einsum("nij,j->ni", Rm, lr @ center + lt) + Pw   # this LiDAR's world position per point
    for it in range(MAX_ITERS):
        Rc = R.from_rotvec(x[:3]).as_matrix()
        Qc = (Q - center) @ Rc.T + center + x[3:]
        W = np.einsum("nij,nj->ni", Rm, Qc @ lr.T + lt) + Pw
        radius = radii[min(it // 2, len(radii) - 1)]
        ok, n, q, tq = (target.match(W, radius, sensors, T) if isinstance(target, BlockTarget)
                        else target.match(W, radius, sensors))
        if normal_filter is not None:
            keep = normal_filter(n)
            idx_ok = np.flatnonzero(ok); ok = np.zeros_like(ok); ok[idx_ok[keep]] = True; n, q, tq = n[keep], q[keep], tq[keep]
        if ok.sum() < min_matches:
            return None, None, None, np.nan
        r = np.einsum("ni,ni->n", n, W[ok] - q)
        w = np.minimum(1.0, 0.05 / np.maximum(np.abs(r), 1e-9))
        # d W / d(rotvec about center) = R_i * lr * (-[Qc-center]x) ; d W / d t = R_i * lr
        RL = np.einsum("nij,jk->nik", Rm[ok], lr)
        dq = Qc[ok] - center
        skew = np.stack([np.zeros_like(dq[:, 0]), -dq[:, 2], dq[:, 1], dq[:, 2], np.zeros_like(dq[:, 0]), -dq[:, 0],
                         -dq[:, 1], dq[:, 0], np.zeros_like(dq[:, 0])], 1).reshape(-1, 3, 3)
        J_rot = -np.einsum("nij,njk->nik", RL, skew)
        J = np.hstack([np.einsum("ni,nij->nj", n, J_rot), np.einsum("ni,nij->nj", n, RL)])
        Jw = J * w[:, None]
        H = J.T @ Jw + prior; g = Jw.T @ r + prior @ (x - x0)
        step = np.zeros(6); step[active] = -np.linalg.solve(H[np.ix_(active, active)], g[active])
        x = x + step
        gnd = np.abs(n[:, 2]) > 0.8; vert = np.abs(n[:, 2]) < 0.3
        stats = (it + 1, int(ok.sum()), float(np.median(np.abs(r))), int(gnd.sum()),
                 float(np.median(np.abs(r[gnd]))) if gnd.any() else np.nan,
                 int(vert.sum()), float(np.median(np.abs(r[vert]))) if vert.any() else np.nan)
        if it == 0:
            first = stats
        dt_med = float(np.median(np.abs(T[ok] - tq)))   # time gap between a point and its reference match
        # converged once a step is far below calibration precision (0.0006 deg / 0.1 mm): the re-matching of
        # a few boundary points can otherwise keep a limit cycle of ~1e-6 rad / 0.03 mm going indefinitely
        if np.linalg.norm(step[:3]) < 1e-5 and np.linalg.norm(step[3:]) < 1e-4 and it // 2 >= len(radii) - 1:
            break
    return x, first, stats, dt_med


def quality_verdict(first, stats, x):
    """ok / warn / fail with reasons from the ICP statistics of one LiDAR."""
    reasons, level = [], "ok"
    def warn(msg):
        nonlocal level
        reasons.append(msg); level = "warn" if level == "ok" else level
    def fail(msg):
        nonlocal level
        reasons.append(msg); level = "fail"
    rot_deg = float(np.degrees(np.linalg.norm(x[:3]))); trans_m = float(np.linalg.norm(x[3:]))
    converged = stats[0] < MAX_ITERS
    vert_regress = (stats[6] - first[6]) if np.isfinite(stats[6]) and np.isfinite(first[6]) else 0.0
    if rot_deg > GATE_MAX_ROT_DEG: fail(f"rotation correction {rot_deg:.2f} deg > {GATE_MAX_ROT_DEG}")
    if trans_m > GATE_MAX_TRANS_M: fail(f"translation correction {100*trans_m:.1f} cm > {100*GATE_MAX_TRANS_M:.0f}")
    if not converged and vert_regress > GATE_VERT_REGRESS_M:
        fail(f"not converged in {MAX_ITERS} iterations and vertical residual regressed {100*first[6]:.2f}->{100*stats[6]:.2f} cm")
    elif not converged: warn(f"not converged in {MAX_ITERS} iterations")
    elif vert_regress > GATE_VERT_REGRESS_M: warn(f"vertical residual regressed {100*first[6]:.2f}->{100*stats[6]:.2f} cm")
    if stats[5] < GATE_MIN_VERTICAL: warn(f"only {stats[5]} vertical planar matches (yaw/x/y weakly observed)")
    if stats[2] > first[2]: warn(f"overall residual regressed {100*first[2]:.2f}->{100*stats[2]:.2f} cm")
    return {"level": level, "reasons": reasons, "converged": bool(converged), "iterations": int(stats[0]),
            "matches": int(stats[1]), "vertical_matches": int(stats[5]),
            "median_before_cm": round(100 * first[2], 2), "median_after_cm": round(100 * stats[2], 2),
            "vertical_before_cm": round(100 * first[6], 2) if np.isfinite(first[6]) else None,
            "vertical_after_cm": round(100 * stats[6], 2) if np.isfinite(stats[6]) else None,
            "rotation_deg": round(rot_deg, 3), "translation_cm": round(100 * trans_m, 1)}


def fmt_stats(first, stats):
    return (f"matches {first[1]}->{stats[1]} |res| median {100*first[2]:.2f}->{100*stats[2]:.2f} cm; "
            f"ground {100*first[4]:.2f}->{100*stats[4]:.2f} (n={stats[3]}); vertical {100*first[6]:.2f}->{100*stats[6]:.2f} (n={stats[5]})")


def fmt_x(x):
    return (f"rot deg (x,y,z)=({np.degrees(x[0]):+.3f},{np.degrees(x[1]):+.3f},{np.degrees(x[2]):+.3f}) "
            f"trans cm=({100*x[3]:+.1f},{100*x[4]:+.1f},{100*x[5]:+.1f})")


ARCHIVE_TOPICS = ["front_right_lidar", "back_lidar", "front_left_lidar", "front_lidar"]  # multi_lidar.topics order


def round_extrinsics(run, fallback):
    """Extrinsics the mapper actually used in a round: Log/extrinsic_precheck.json (before / after an applied startup
    correction) or the round's calibration file; `fallback` (parent) when neither is available."""
    rnd = run.parents[2] if len(run.parents) > 2 else run
    pre = rnd / "Log" / "extrinsic_precheck.json"
    out = {}
    if pre.exists():
        doc = json.loads(pre.read_text())
        for e in doc.get("sources", []):
            topic = str(e.get("topic", "")).strip("/")
            mat = e.get("rear_from_lidar_after") if e.get("applied") else e.get("rear_from_lidar_before")
            if topic in fallback and mat and len(mat) == 16:
                out[topic] = np.array(mat, dtype=float).reshape(4, 4)
    if len(out) < len(fallback):
        try:
            cal_path = json.loads((rnd / "round.json").read_text()).get("calibration")
            cal = json.load(open(cal_path)) if cal_path else None
        except (OSError, ValueError):
            cal = None
        for entry in (cal or {}).get("lidar", []):
            for topic, k in KEYS.items():
                if k in entry and topic not in out:
                    out[topic] = np.array(entry[k], dtype=float).reshape(4, 4)
    for topic, m in fallback.items():
        out.setdefault(topic, m)
    return out


def archive_scans(run, rows, window, t0, M_run, max_range, stride=2):
    """Per-LiDAR scans from the round's own archive (scans.bin + sources.bin): exactly the returns the mapper put in
    the map (its deskew, body and range filters). Returned in each LiDAR frame (inverse of the round's extrinsics) and
    stamped at the scan end, so world_points/build_target reproduce the map geometry with the frame poses."""
    scans = {t: [] for t in ARCHIVE_TOPICS}
    with open(run / "scans.bin", "rb") as fs, open(run / "sources.bin", "rb") as fsrc:
        for row in rows:
            stamp, begin, off, count = row[0], row[1], int(row[2]), int(row[3])
            if begin - t0 < window[0] - 0.2 or begin - t0 > window[1] or count <= 0:
                continue
            fs.seek(off); q = np.frombuffer(fs.read(count * 20), "<f4").reshape(-1, 5)[:, :3].astype(float)
            fsrc.seek(8 + (off - 8) // 20); src = np.frombuffer(fsrc.read(count), np.uint8)
            for k, topic in enumerate(ARCHIVE_TOPICS):
                Q = q[src == k]
                if len(Q) < 100: continue
                Mr = M_run[topic]
                P = (Q - Mr[:3, 3]) @ Mr[:3, :3]                       # rear-axle -> LiDAR frame
                r = np.linalg.norm(P, axis=1); P = P[(r > 3.0) & (r < max_range)][::stride]
                if len(P) < 100: continue   # raw density (no voxel): weighted like the mapper/LIO and the map itself
                scans[topic].append((P, np.full(len(P), stamp - begin), begin))
    return scans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", required=True); ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True, help="fusion run dir with frames.csv (native poses)")
    ap.add_argument("--window", type=float, nargs=2, default=(8.0, 40.0), help="seconds after first frame")
    ap.add_argument("--lidar-time-offset", type=float, default=-0.125)
    ap.add_argument("--output", required=True); ap.add_argument("--report", required=True)
    ap.add_argument("--voxel", type=float, default=0.2); ap.add_argument("--max-range", type=float, default=60.0)
    ap.add_argument("--vertical-only", action="store_true", help="use only vertical planar matches (|n_z|<0.3): solves yaw/x/y, keeps roll/pitch/z")
    ap.add_argument("--prior-rot-deg", type=float, default=0.5); ap.add_argument("--prior-trans-m", type=float, default=0.10)
    ap.add_argument("--ground-poses", choices=("optimized", "native"), default="optimized",
                    help="pose source of pass 2 (tilt/z of cross-time LiDARs); native = single pass, previous behaviour")
    ap.add_argument("--cross-time-s", type=float, default=2.0,
                    help="a LiDAR whose median match time gap exceeds this gets pass 2")
    ap.add_argument("--no-same-side", action="store_true",
                    help="allow matches where the source LiDAR saw the plane from the opposite side (previous behaviour)")
    ap.add_argument("--pass1-poses", choices=("native", "optimized"), default="native",
                    help="pose source of pass 1 (yaw/x/y or 6 DoF): native frames.csv (default) or the RTK-optimized trajectory")
    ap.add_argument("--no-gate", action="store_true", help="write the output even when a LiDAR fails the quality gate")
    ap.add_argument("--pairing", choices=("same-time", "accumulated"), default="same-time",
                    help="pass-1 matching of LiDARs that overlap the reference at the same time: time-blocked targets "
                         "(default) or one accumulated target with first-observation voxels (previous behaviour)")
    ap.add_argument("--block-s", type=float, default=1.0, help="same-time pairing: block length (s)")
    ap.add_argument("--block-voxels", choices=("nearest", "first"), default="nearest",
                    help="same-time pairing: observation kept per target voxel inside a block")
    ap.add_argument("--pad-s", type=float, default=0.5, help="same-time pairing: reference returns this much before/after a block also match")
    ap.add_argument("--scans", choices=("auto", "archive", "bag"), default="auto",
                    help="returns to register: the round's own deskewed archive scans.bin/sources.bin (auto when present; "
                         "the geometry the map is built from) or the raw bag with a per-point re-deskew")
    a = ap.parse_args()
    calib = json.load(open(a.calibration))
    M = {}
    for entry in calib["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: M[topic] = np.array(entry[k], dtype=float).reshape(4, 4)
    run = pathlib.Path(a.run)
    lines = (run / "frames.csv").read_text().splitlines()
    lt = np.array([float(v) for v in lines[1].split()[2:]]); lr = np.array([float(v) for v in lines[2].split()[2:]]).reshape(3, 3)
    rows = np.array([[float(v) for v in l.split(",")] for l in lines[4:]])
    stamps = rows[:, 0]; t0 = stamps[0]
    native = Poses(stamps, R.from_quat(rows[:, 7:11]), rows[:, 4:7], "native frames.csv")
    want_optimized = (a.ground_poses == "optimized" and not a.vertical_only) or a.pass1_poses == "optimized"
    optimized = load_optimized(run, stamps) if want_optimized else None
    pose1 = native
    if a.pass1_poses == "optimized":
        if optimized is None:
            print("warning: --pass1-poses optimized requested but the run has no usable trajectory_optimized.txt; pass 1 uses native poses", flush=True)
        else:
            pose1 = optimized
    if a.vertical_only and a.ground_poses == "optimized" and a.pass1_poses != "optimized":
        optimized = None
    if a.ground_poses == "optimized" and optimized is None and not a.vertical_only:
        print("warning: no usable trajectory_optimized.txt in the run (LIO round without RTK fusion?); "
              "pass 2 disabled, cross-time LiDARs keep the native-pose tilt/z", flush=True)
    # never extrapolate with the last pose: clamp the window to the poses that exist
    pose_end = float(stamps[-1] - t0)
    short_recording = pose_end < SHORT_RECORDING_S
    min_icp_matches = 50 if short_recording else 300
    if a.window[1] > pose_end - 0.5:
        print(f"warning: window end {a.window[1]:.1f} s exceeds available poses ({pose_end:.1f} s); clamping", flush=True)
        w0 = a.window[0] if (not short_recording or pose_end - a.window[0] >= 5.0) else max(0.0, pose_end - 10.0)
        a.window = (w0, max(w0 + 1.0, pose_end - 0.5))

    topics = list(KEYS)
    use_archive = a.scans == "archive" or (a.scans == "auto" and (run / "scans.bin").exists() and (run / "sources.bin").exists())
    if use_archive:
        M_run = round_extrinsics(run, M)
        scans = archive_scans(run, rows, a.window, t0, M_run, a.max_range)
        for topic in topics:   # the excluded body box is applied below exactly as for bag scans
            kept = []
            for P, ts, h in scans[topic]:
                Q = P @ M[topic][:3, :3].T + M[topic][:3, 3]
                excl = (Q[:, 0] > -2.8) & (Q[:, 0] < 2.8) & (Q[:, 1] > -4.5) & (Q[:, 1] < 9.5)
                if (~excl).sum() >= 100: kept.append((P[~excl], ts[~excl], h))
            scans[topic] = kept
        n_used = {t: len(scans[t]) for t in topics}
    else:
        scans = {t: [] for t in topics}; n_used = {t: 0 for t in topics}
    dataset = Dataset.open(a.bag) if not use_archive else None  # raw split recording or single bag, merged in time order
    for topic, data, _ in (dataset.iter_messages(topics) if dataset is not None else ()):
        topic = topic.split("/")[-1] if topic.split("/")[-1] in scans else topic
        if topic not in scans: continue
        msg = deserialize_message(data, PointCloud2)
        h = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + a.lidar_time_offset  # LIO time base (scan start)
        if h - t0 < a.window[0] - 0.2: continue
        if h - t0 > a.window[1]:
            if all(n_used[t] > 0 for t in topics) and h - t0 > a.window[1] + 1: break
            continue
        arr = pc2.read_points(msg, field_names=["x", "y", "z", "timestamp"], skip_nans=True)
        P = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(float)
        ts = np.asarray(arr["timestamp"], dtype=float)
        if ts.max() > 1e6: ts = ts - ts.min()          # absolute -> relative
        if ts.max() > 1.0: ts = ts * 1e-3 if ts.max() < 1e3 else ts * 1e-9  # ms or ns
        r = np.linalg.norm(P, axis=1); keep = (r > 3.0) & (r < a.max_range)
        P, ts = P[keep], ts[keep]
        if len(P) < 100: continue
        sub = voxel_down(P, 0.1)
        P, ts = P[sub], ts[sub]
        Q = P @ M[topic][:3, :3].T + M[topic][:3, 3]                  # rear-axle frame
        excl = (Q[:, 0] > -2.8) & (Q[:, 0] < 2.8) & (Q[:, 1] > -4.5) & (Q[:, 1] < 9.5)
        scans[topic].append((P[~excl], ts[~excl], h)); n_used[topic] += 1
    pass2_label = optimized.label if optimized is not None else "disabled"
    report = [f"calibration={a.calibration}", f"run={run}",
              f"window {a.window[0]}..{a.window[1]} s, lidar_time_offset {a.lidar_time_offset}",
              f"pass 1 poses: {pose1.label} ({'yaw/x/y, vertical matches only' if a.vertical_only else '6 DoF'}); pass 2 poses: {pass2_label} (tilt about x/y and z of LiDARs with median match time gap > {a.cross_time_s:.1f} s)",
              "scans used: " + ", ".join(f"{t}:{n_used[t]}" for t in topics),
              "scan source: " + ("round archive scans.bin/sources.bin (mapper deskew; returns at scan end, round extrinsics inverted)"
                                 if use_archive else "raw bag, per-point deskew with the frame poses")]
    if not scans[REFERENCE]:
        pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
        report.append("no reference scans in the window"); pathlib.Path(a.report).write_text("\n".join(report) + "\n")
        print("\n".join(report)); sys.exit(3)
    legacy = a.pairing == "accumulated"
    target1 = build_target(scans[REFERENCE], M[REFERENCE], lr, lt, pose1, a.voxel, same_side=not a.no_same_side,
                           nearest_first=not legacy)
    report.append(f"matching: {'viewing-side consistent (oriented normals)' if not a.no_same_side else 'any side (legacy)'}")
    target2 = None
    report.append(f"reference {REFERENCE}: {len(target1.A)} world points")
    target_same = None
    if not legacy:
        target_same = BlockTarget(scans[REFERENCE], M[REFERENCE], lr, lt, pose1, min(a.voxel, 0.12), a.block_s, a.pad_s,
                                  same_side=not a.no_same_side, nearest=a.block_voxels == "nearest", short=short_recording)
        report.append(f"pairing: same-time for LiDARs overlapping the reference ({a.block_s:.1f} s blocks +-{a.pad_s:.1f} s, "
                      f"{target_same.count} block points); cross-time LiDARs use the accumulated target (nearest-range voxels)")
    else:
        report.append("pairing: accumulated target, first observation per voxel (legacy)")
    prior = np.diag([1 / np.radians(a.prior_rot_deg) ** 2] * 3 + [1 / a.prior_trans_m ** 2] * 3)
    active1 = np.array([not a.vertical_only, not a.vertical_only, True, True, True, not a.vertical_only])
    filt1 = (lambda n: np.abs(n[:, 2]) < 0.3) if a.vertical_only else None
    refined = {REFERENCE: M[REFERENCE].copy()}; corrections = {}; quality = {}
    for t in ("front_right_lidar", "front_left_lidar", "back_lidar"):
        if not scans[t]:
            refined[t] = M[t].copy(); report.append(f"{t}: no scans  [quality: warn]")
            quality[t] = {"level": "warn", "reasons": ["no scans in window"]}; continue
        center = M[t][:3, 3]
        tgt = target_same if (target_same is not None and t != "back_lidar") else target1
        # time-uniform raw-density sampling only for same-time pairing: a cross-time LiDAR (back) matched at raw density
        # puts its dense near returns on reference returns ~5 s older, and the native along-track drift then entered
        # its forward offset (truck29raw: +15.8 cm, rejected by the gate); it keeps the vehicle-frame voxel sampling.
        Q, Rm, Pw, T = world_points(scans[t], M[t], lr, lt, pose1, time_bin=None if (legacy or tgt is target1) else a.block_s)
        x, first, stats, dt_med = icp(Q, Rm, Pw, T, center, lr, lt, tgt, active1, np.zeros(6), prior, filt1,
                                      radii=(0.6,) if (legacy or tgt is target1) else (0.6, 0.4, 0.3), min_matches=min_icp_matches)
        if stats is None:
            refined[t] = M[t].copy(); report.append(f"{t}: insufficient matches; manual matrix kept  [quality: warn]")
            quality[t] = {"level": "warn", "reasons": ["insufficient matches; matrix kept"]}; continue
        q1 = quality_verdict(first, stats, x)
        line = (f"{t}: pass 1 {fmt_stats(first, stats)}; match time gap median {dt_med:.1f} s; correction {fmt_x(x)}; iters {stats[0]}")
        pass2 = None
        if optimized is not None and dt_med > a.cross_time_s and q1["level"] != "fail":
            if target2 is None:
                target2 = build_target(scans[REFERENCE], M[REFERENCE], lr, lt, optimized, a.voxel, same_side=not a.no_same_side,
                                       nearest_first=not legacy)
            Q2, Rm2, Pw2, T2 = world_points(scans[t], M[t], lr, lt, optimized)
            active2 = np.array([True, True, False, False, False, True])
            x2, first2, stats2, dt2 = icp(Q2, Rm2, Pw2, T2, center, lr, lt, target2, active2, x, prior,
                                          lambda n: np.abs(n[:, 2]) > 0.8, min_matches=min_icp_matches)
            if stats2 is not None and stats2[4] <= first2[4] + 0.002:
                d = x2 - x
                line += (f"\n    pass 2 (cross-time LiDAR, drift-free poses, ground only): ground {100*first2[4]:.2f}->{100*stats2[4]:.2f} cm (n={stats2[3]}); "
                         f"tilt about x/y adjusted by ({np.degrees(d[0]):+.3f},{np.degrees(d[1]):+.3f}) deg, z by {100*d[5]:+.1f} cm; iters {stats2[0]}")
                pass2 = {"ground_before_cm": round(100 * first2[4], 2), "ground_after_cm": round(100 * stats2[4], 2), "ground_matches": int(stats2[3]),
                         "delta_rotvec_deg": np.degrees(d[:3]).round(4).tolist(), "delta_trans_cm": (100 * d[3:]).round(2).tolist(), "iterations": int(stats2[0])}
                x = x2
                if stats2[0] >= MAX_ITERS:
                    q1["reasons"].append("pass 2 not converged"); q1["level"] = "warn" if q1["level"] == "ok" else q1["level"]
            else:
                line += "\n    pass 2 skipped (ground residual would not improve)"
        C = np.eye(4); C[:3, :3] = R.from_rotvec(x[:3]).as_matrix(); C[:3, 3] = center - C[:3, :3] @ center + x[3:]
        refined[t] = C @ M[t]
        q1["match_time_gap_s"] = round(dt_med, 2); q1["pass2"] = pass2
        q1["rotation_deg"] = round(float(np.degrees(np.linalg.norm(x[:3]))), 3); q1["translation_cm"] = round(float(100 * np.linalg.norm(x[3:])), 1)
        quality[t] = q1
        corrections[t] = dict(rotvec_deg=np.degrees(x[:3]).tolist(), trans_cm=(100 * x[3:]).tolist(),
                              about="lidar position in rear-axle frame", quality=q1)
        if pass2 is not None:
            line += f"\n    final correction {fmt_x(x)}"
        line += f"  [quality: {q1['level']}{'; ' + '; '.join(q1['reasons']) if q1['reasons'] else ''}]"
        report.append(line)
    levels = [q["level"] for q in quality.values()]
    overall = "fail" if "fail" in levels else ("warn" if "warn" in levels else "ok")
    report.append(f"quality: {overall}" + ("" if overall == "ok" else "  (" + "; ".join(
        f"{t}: {', '.join(q['reasons'])}" for t, q in quality.items() if q["reasons"]) + ")"))
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True); pathlib.Path(a.report).write_text("\n".join(report) + "\n")
    print("\n".join(report))
    if overall == "fail" and not a.no_gate:
        if short_recording:
            print("warning: driven refinement exceeded quality gate on short/under-constrained window; continuing with warn")
            overall = "warn"
        else:
            print("driven refinement rejected by the quality gate; calibration.json not written "
                  "(the window needs vertical structures, a turn and low front-end drift; --no-gate overrides)")
            sys.exit(3)
    out = json.loads(json.dumps(calib))
    for entry in out["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: entry[k] = [float(v) for v in refined[topic].reshape(-1)]
    tag = "+driven_window_vertical_yaw_xy_refinement" if a.vertical_only else "+driven_window_6dof_refinement"
    out["calibration_type"] = calib.get("calibration_type", "") + tag
    out.setdefault("refinement", {})["stage3_vertical" if a.vertical_only else "stage2_6dof"] = dict(
        reference=REFERENCE, corrections=corrections, run=str(run), window=list(a.window), prior=[a.prior_rot_deg, a.prior_trans_m],
        poses={"pass1": pose1.label, "pass2": pass2_label, "cross_time_s": a.cross_time_s},
        pairing={"mode": a.pairing, "block_s": a.block_s, "pad_s": a.pad_s},
        scan_source="archive" if use_archive else "bag", quality=overall)
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True); json.dump(out, open(a.output, "w"), indent=2)
    print("written", a.output)


if __name__ == "__main__":
    main()
