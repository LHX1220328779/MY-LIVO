#!/usr/bin/env python3
"""Estimate the RTK position time-association correction from the map of one fusion run (no extra rounds).

Physics: the backend pulls the rear axle to the RTK fix associated with the scan time + effective offset. An
offset error d moves every optimized pose by v_rear(t) * d along the motion (verified on truck28: two rounds
20 ms apart differ by 19.9 ms * v, residual 1 cm). Structure observed at different velocities (start-up
acceleration, turns, opposite passes) is then smeared, so the correction is the value that makes the map
most self-consistent:

    min over D:  sum_voxels sum_points ( n_j . (w_i + v_i D) - d_j )^2

w_i: world point of the round, v_i: rear-axle velocity of its frame, n_j/d_j: local plane of a voxel.
The cost  sum_j n_j * lambda_min(C_j(D))  (C_j: covariance of voxel j after shifting every point by v_i * D,
i.e. the best local plane refitted at every D) is evaluated on a grid; voxel membership is fixed so the cost
is smooth, and it is re-centred at the minimum until it stops moving (otherwise only voxels that happen to be
sharp at the run's own offset are kept). Only planar voxels seen over >= 1 s carry velocity diversity.
The uncertainty is a delete-one-block jackknife over time blocks.

One round recovers only part of an offset error (the backend follows the native odometry through the
accelerations that make the offset observable): truck28 rounds at -7.5/+12.5/+16.5/+32.5 ms gave corrections
of +11.8/-0.2/-2.7/-12.9 ms, slope 0.6 and a common zero at +12 ms. Callers therefore close the loop with a
second round (secant), see livo_calib.py rtk-time.

Output: JSON with the correction D [s], the best effective offset (= run effective + D), the jackknife
standard error and an observability verdict. Read-only on the run directory.

  estimate_rtk_time_offset.py --run <fusion run dir | round dir> [--window t0 t1] [--json out.json]
                              [--max-se-ms 3.0]
"""
import argparse
import json
import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

VOXEL = 1.0           # m, plane voxel edge
MIN_PTS = 24          # points per voxel
MIN_FRAMES_SPAN = 1.0  # s, a voxel must be seen over at least this long to carry velocity diversity
RANGE = (2.0, 60.0)   # m, LiDAR range of the points used (rotation-error leverage beyond)
SHORT_RECORDING_S = 90.0


def fusion_dir(path):
    p = pathlib.Path(path)
    if (p / "frames.csv").exists():
        return p
    runs = sorted((p / "Log" / "fusion").glob("*/"))
    if not runs:
        sys.exit(f"{p}: no fusion run (Log/fusion/<stamp>/frames.csv) found")
    return runs[-1]


def read_params(run):
    params = {}
    for line in (run / "parameters.txt").read_text().splitlines():
        if "=" in line and not line.startswith("model="):
            k, v = line.split("=", 1)
            params.setdefault(k.strip(), v.strip())  # effective values come first
    return params


def load_frames(run):
    txt = (run / "frames.csv").read_text().splitlines()
    rows = [l for l in txt if l and not l.startswith("#")]
    head = rows[0].split(",")
    body = [r.split(",") for r in rows[1:]]
    body = [r for r in body if len(r) == len(head) and r[-1].strip() != ""]
    data = np.array([[float(v) for v in r] for r in body])
    out = {k: data[:, i] for i, k in enumerate(head)}
    lever = np.zeros(3)
    for l in txt:
        if l.startswith("# lidar_to_imu_translation"):
            lever = np.array([float(v) for v in l.split()[2:5]])
    out["lever"] = lever
    return out


def interp_poses(tr, stamps):
    """Position (linear) and rotation (slerp) of the trajectory at the given stamps."""
    t = tr[:, 0]
    s = np.clip(stamps, t[0], t[-1])
    pos = np.column_stack([np.interp(s, t, tr[:, k]) for k in (1, 2, 3)])
    from scipy.spatial.transform import Slerp
    rot = Slerp(t, R.from_quat(tr[:, 4:8]))(s)
    return pos, rot


def rear_velocity(tr, imu_to_rear, stamps, half=0.2):
    """Rear-axle velocity (world) by a +-half s central difference on the optimized trajectory."""
    t = tr[:, 0]
    rear = tr[:, 1:4] + R.from_quat(tr[:, 4:8]).apply(imu_to_rear)
    a = np.clip(stamps - half, t[0], t[-1])
    b = np.clip(stamps + half, t[0], t[-1])
    pa = np.column_stack([np.interp(a, t, rear[:, k]) for k in range(3)])
    pb = np.column_stack([np.interp(b, t, rear[:, k]) for k in range(3)])
    return (pb - pa) / np.maximum(b - a, 1e-3)[:, None]


def gather(run, window, max_frames, pts_per_frame, seed=7):
    fr = load_frames(run)
    params = read_params(run)
    imu_to_rear = np.array([float(v) for v in params["effective.imu_to_rear_axle"].split()])
    eff = float(params["rtk.time_offset_sec"])
    tr = np.loadtxt(run / "trajectory_optimized.txt", ndmin=2)
    st = fr["stamp"]
    t_rel = st - st[0]
    short_recording = len(st) >= 2 and float(st[-1] - st[0]) < SHORT_RECORDING_S
    min_frames = 2 if short_recording else 20
    sel = np.where((t_rel >= window[0]) & (t_rel <= window[1]) & (st >= tr[0, 0]) & (st <= tr[-1, 0]))[0]
    if len(sel) < min_frames and short_recording:
        sel = np.arange(len(st))
    if len(sel) < min_frames:
        sys.exit(f"only {len(sel)} frames inside the window (need {min_frames})")
    stride = max(1, int(np.ceil(len(sel) / max_frames)))
    sel = sel[::stride]
    pos, rot = interp_poses(tr, st[sel])
    vel = rear_velocity(tr, imu_to_rear, st[sel])
    rng = np.random.default_rng(seed)
    W, V, F = [], [], []
    if not (run / "scans.bin").exists():
        sys.exit(f"{run}: scans.bin missing (the round must save its map frames)")
    with open(run / "scans.bin", "rb") as f:
        for k, i in enumerate(sel):
            n = int(fr["count"][i])
            if n == 0:
                continue
            f.seek(int(fr["offset_bytes"][i]))
            p = np.frombuffer(f.read(n * 20), "<f4").reshape(-1, 5)[:, :3].astype(np.float64)
            r = np.linalg.norm(p, axis=1)
            p = p[(r > RANGE[0]) & (r < RANGE[1])]
            if len(p) > pts_per_frame:
                p = p[rng.choice(len(p), pts_per_frame, replace=False)]
            p = p + fr["lever"]
            W.append(rot[k].apply(p) + pos[k])
            V.append(np.repeat(vel[k][None], len(p), 0))
            F.append(np.full(len(p), t_rel[i]))
    return dict(W=np.concatenate(W), V=np.concatenate(V), T=np.concatenate(F),
                eff=eff, frames=len(sel), stride=stride, speed=np.linalg.norm(vel, axis=1),
                params=params)


def voxel_stats(W, V, T):
    """Fixed voxel membership (at the run's own offset) and per-voxel sufficient statistics of w and v."""
    key = np.floor(W / VOXEL).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel(); m = len(cnt)
    Sw = np.stack([np.bincount(inv, W[:, k], m) for k in range(3)], 1)
    Sv = np.stack([np.bincount(inv, V[:, k], m) for k in range(3)], 1)
    Sww = np.zeros((m, 3, 3)); Swv = np.zeros((m, 3, 3)); Svv = np.zeros((m, 3, 3))
    for a in range(3):
        for b in range(3):
            Sww[:, a, b] = np.bincount(inv, W[:, a] * W[:, b], m)
            Swv[:, a, b] = np.bincount(inv, W[:, a] * V[:, b], m)
            Svv[:, a, b] = np.bincount(inv, V[:, a] * V[:, b], m)
    tmin = np.full(m, np.inf); tmax = np.full(m, -np.inf); tsum = np.bincount(inv, T, m)
    np.minimum.at(tmin, inv, T); np.maximum.at(tmax, inv, T)
    return dict(n=cnt.astype(float), Sw=Sw, Sv=Sv, Sww=Sww, Swv=Swv, Svv=Svv, span=tmax - tmin, tmean=tsum / cnt)


def eig_at(st, D, idx=None):
    sl = slice(None) if idx is None else idx
    n = st["n"][sl]
    mu = (st["Sw"][sl] + D * st["Sv"][sl]) / n[:, None]
    C = (st["Sww"][sl] + D * (st["Swv"][sl] + np.transpose(st["Swv"][sl], (0, 2, 1))) + D * D * st["Svv"][sl]) / n[:, None, None]
    C -= mu[:, :, None] * mu[:, None, :]
    return np.linalg.eigvalsh(C)


def solve(data, grid_ms=(-60.0, 60.0, 1.0), blocks=12, rounds=4, short=False):
    """Repeat the grid solve with the voxel membership/selection re-centred at the previous minimum, so the
    voxels kept are not just the ones that happen to be sharp at the run's own (possibly wrong) offset."""
    centre = 0.0; hist = []
    for _ in range(rounds):
        res = _solve_once(dict(data, W=data["W"] + centre * data["V"]), grid_ms, blocks, short=short)
        res["D"] += centre; res["jackknife_ms"] = [j + 1000 * centre for j in res["jackknife_ms"]]
        hist.append(round(1000 * res["D"], 2))
        if abs(res["D"] - centre) < 0.5e-3:
            break
        centre = res["D"]
    res["hist"] = hist
    return res


def _solve_once(data, grid_ms, blocks, short=False):
    """Map-consistency cost  sum_j n_j * lambda_min(C_j(D))  on a grid of corrections D (planes implicitly refitted
    at every D, voxel membership fixed so the cost is smooth). Minimum by a local parabola; jackknife over time
    blocks of the voxels for the standard error."""
    st = voxel_stats(data["W"], data["V"], data["T"])
    ev0 = eig_at(st, 0.0)
    thick0 = np.sqrt(np.maximum(ev0[:, 0], 0))
    # planar, multi-epoch voxels only (single-epoch voxels carry no velocity diversity)
    use = (st["n"] >= MIN_PTS) & (ev0[:, 0] < 0.2 * ev0[:, 1]) & (thick0 < 0.2) & (st["span"] >= MIN_FRAMES_SPAN)
    if short and use.sum() < 50:
        use = (st["n"] >= 12) & (ev0[:, 0] < 0.25 * ev0[:, 1]) & (thick0 < 0.25) & (st["span"] >= 0.5)
    if short and use.sum() < max(4, blocks):
        use = (st["n"] >= 5) & (ev0[:, 0] < 0.35 * ev0[:, 1]) & (thick0 < 0.35) & (st["span"] >= 0.2)
    idx = np.where(use)[0]
    grid = np.arange(grid_ms[0], grid_ms[1] + 1e-9, grid_ms[2]) / 1000.0
    if len(idx) == 0:
        return dict(D=0.0, interior=False, se=999.0, voxels=0, informative_voxels=0, cost_rise_10ms=0.0,
                    cost_curve=[(round(1000 * g, 1), 0.0) for g in grid[::5]], jackknife_ms=[0.0] * blocks, hist=[])
    if short:
        blocks = max(2, min(blocks, len(idx)))
    cost = np.stack([st["n"][idx] * np.maximum(eig_at(st, D, idx)[:, 0], 0) for D in grid], 1)  # voxels x grid
    # robust: a voxel cannot contribute more than its share at a 4 cm residual level (dynamic objects, edges)
    cap = st["n"][idx] * 0.12 ** 2
    cost = np.minimum(cost, cap[:, None])
    total = cost.sum(0)

    def argmin_parabola(c):
        k = int(np.argmin(c))
        if k == 0 or k == len(c) - 1:
            return float(grid[k]), False
        lo, hi = max(0, k - 5), min(len(c), k + 6)
        A = np.polyfit(grid[lo:hi], c[lo:hi], 2)
        if A[0] <= 0:
            return float(grid[k]), False
        return float(np.clip(-A[1] / (2 * A[0]), grid[lo], grid[hi - 1])), True

    D, interior = argmin_parabola(total)
    tv = st["tmean"][idx]
    edges = np.quantile(tv, np.linspace(0, 1, blocks + 1))
    blk = np.clip(np.searchsorted(edges, tv, side="right") - 1, 0, blocks - 1)
    est = np.array([argmin_parabola(total - cost[blk == k].sum(0))[0] for k in range(blocks)])
    se = float(np.sqrt((blocks - 1) / blocks * np.sum((est - est.mean()) ** 2)))
    # curvature-based information (how sharply the cost rises around the minimum), per metre^2 of cost
    k = int(np.argmin(np.abs(grid - D)))
    rise = float((total[max(0, k - 10)] + total[min(len(grid) - 1, k + 10)]) / 2 - total[k])
    gain = np.diff(cost[:, [max(0, k - 10), k]], axis=1).ravel()
    return dict(D=D, interior=interior, se=se, voxels=int(len(idx)), informative_voxels=int((np.abs(gain) > 1e-4).sum()),
                cost_rise_10ms=rise, cost_curve=[(round(1000 * g, 1), float(c)) for g, c in zip(grid[::5], total[::5])],
                jackknife_ms=(1000 * est).round(2).tolist(), hist=[])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--window", nargs=2, type=float, default=(0.0, 1e9), help="seconds after the first frame")
    ap.add_argument("--max-frames", type=int, default=1500)
    ap.add_argument("--points-per-frame", type=int, default=2500)
    ap.add_argument("--max-se-ms", type=float, default=3.0, help="observability gate on the jackknife standard error")
    ap.add_argument("--json")
    a = ap.parse_args()
    run = fusion_dir(a.run)
    data = gather(run, a.window, a.max_frames, a.points_per_frame)
    short_recording = len(data["T"]) >= 2 and float(np.ptp(data["T"])) < SHORT_RECORDING_S
    res = solve(data, short=short_recording)
    sp = data["speed"]
    se_ms = 1000 * res["se"]
    observable = res["interior"] and se_ms <= a.max_se_ms and res["informative_voxels"] >= 50
    out = {
        "run": str(run), "window_s": list(a.window),
        "span_s": float(np.ptp(data["T"])) if len(data["T"]) else 0.0,
        "frames": data["frames"], "frame_stride": data["stride"],
        "points": int(len(data["W"])), "speed_mps": {"median": float(np.median(sp)), "p95": float(np.percentile(sp, 95)),
                                                    "spread_p95_p5": float(np.percentile(sp, 95) - np.percentile(sp, 5))},
        "run_effective_offset_s": data["eff"],
        "correction_s": float(res["D"]), "correction_se_s": float(res["se"]),
        "best_effective_offset_s": float(data["eff"] + res["D"]),
        "informative_voxels": res["informative_voxels"], "observable": bool(observable),
        "jackknife_ms": res["jackknife_ms"], "recentre_ms": res["hist"], "cost_rise_10ms": res["cost_rise_10ms"], "cost_curve": res["cost_curve"],
    }
    print(f"RTK time correction {1000*res['D']:+.2f} ms (se {se_ms:.2f} ms) -> best effective offset "
          f"{1000*out['best_effective_offset_s']:+.2f} ms (run {1000*data['eff']:+.2f}); voxels {res['voxels']}, informative {res['informative_voxels']}; "
          f"{'observable' if observable else 'NOT observable'}")
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(out, indent=2) + "\n")
    else:
        print(json.dumps({k: v for k, v in out.items() if k not in ("cost_curve", "jackknife_ms")}, indent=1))


if __name__ == "__main__":
    main()
