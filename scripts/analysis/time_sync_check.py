#!/usr/bin/env python3
"""Time-synchronisation self-check of one mapping round (data-driven, no re-run needed).

Answers, from the round's own outputs and the recording's INS/IMU reference:
  [1] Is the INS message chain internally consistent?  attitude-vs-gyro lag, position-vs-velocity lag.
  [2] Native LIO body yaw-rate vs raw gyro lag (INFORMATIONAL). Measured on truck05 with
      lidar_frontend = -0.0119 and -0.125 the lag stayed at +10/+9 ms: the LIO attitude follows the
      gyro between updates, so this number does NOT measure time_offsets.lidar_frontend and no
      recommendation is derived from it. Values far from 0 +- 15 ms indicate stamping problems only.
  [3] INS position association (rtk.time_offset_sec effective): native rear-axle position vs INS
      position, along-track error regressed on INS speed inside short windows (linear drift removed).
      Resolution ~15 ms on LIO-grade native trajectories; a geometric check (map thickness / pole
      ghosts across two candidate values) must confirm before a calibration is changed.
Sign convention: lag > 0 means the SECOND signal's stamps are late.

usage: time_sync_check.py <fusion_dir> <reference_csv_dir> [--lever x y z] [--json out.json]
"""
import argparse, csv, json, math, pathlib, re, sys
import numpy as np


def load_csv(path, cols):
    data = {c: [] for c in cols}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                vals = [float(row[c]) for c in cols]
            except (KeyError, ValueError):
                continue
            for c, v in zip(cols, vals):
                data[c].append(v)
    d = {c: np.asarray(v) for c, v in data.items()}
    o = np.argsort(d["stamp"], kind="stable")
    d = {c: v[o] for c, v in d.items()}
    keep = np.r_[True, np.diff(d["stamp"]) > 1e-6]
    return {c: v[keep] for c, v in d.items()}


def load_traj(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            v = line.split()
            if len(v) < 8:
                continue
            rows.append([float(x) for x in v[:8]])
    a = np.asarray(rows)
    o = np.argsort(a[:, 0], kind="stable")
    a = a[o]
    keep = np.r_[True, np.diff(a[:, 0]) > 1e-6]
    return a[keep]


def quat_to_R(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def body_rates(t, q):
    R = quat_to_R(q)
    dR = np.einsum("nji,njk->nik", R[:-1], R[1:])
    tr = np.clip((np.trace(dR, axis1=1, axis2=2) - 1) / 2, -1, 1)
    ang = np.arccos(tr)
    axis = np.stack([dR[:, 2, 1] - dR[:, 1, 2], dR[:, 0, 2] - dR[:, 2, 0], dR[:, 1, 0] - dR[:, 0, 1]], 1)
    s = 2 * np.sin(ang)
    with np.errstate(invalid="ignore", divide="ignore"):
        rv = axis * np.where(s > 1e-12, ang / s, 0.5)[:, None]
    dt = np.diff(t); ok = dt > 1e-6
    return 0.5 * (t[:-1] + t[1:])[ok], rv[ok] / dt[ok, None]


def smooth(x, n):
    """Moving average with edge padding (no zero-padding artefacts at the ends)."""
    if n <= 1:
        return x
    k = np.ones(n) / n
    h = n // 2

    def one(v):
        vp = np.concatenate([np.full(h, v[0]), v, np.full(n - 1 - h, v[-1])])
        return np.convolve(vp, k, mode="valid")
    if x.ndim == 1:
        return one(x)
    return np.stack([one(x[:, i]) for i in range(x.shape[1])], 1)


def xcorr_lag(t1, s1, t2, s2, max_lag=0.15, step=0.001, tmin=None, tmax=None):
    lo = max(t1[0], t2[0]) + max_lag; hi = min(t1[-1], t2[-1]) - max_lag
    if tmin is not None: lo = max(lo, tmin)
    if tmax is not None: hi = min(hi, tmax)
    if hi - lo < 5:
        return None
    grid = np.arange(lo, hi, step)
    a = np.interp(grid, t1, s1); a = a - a.mean()
    if np.sqrt(np.mean(a * a)) < 1e-4:
        return None
    lags = np.arange(-max_lag, max_lag + step / 2, step)
    scores = np.empty(len(lags))
    for i, L in enumerate(lags):
        b = np.interp(grid + L, t2, s2); b = b - b.mean()
        scores[i] = float(a @ b) / (math.sqrt(float(a @ a) * float(b @ b)) + 1e-12)
    k = int(np.argmax(scores)); L = lags[k]
    if 0 < k < len(lags) - 1:
        y0, y1, y2 = scores[k - 1], scores[k], scores[k + 1]
        den = (y0 - 2 * y1 + y2)
        if den != 0:
            L = lags[k] + (y0 - y2) / (2 * den) * step
    b = np.interp(grid + L, t2, s2); b = b - b.mean()
    return {"lag_ms": 1000 * float(L), "corr": float(scores[k]), "gain": float(a @ b) / (float(b @ b) + 1e-12),
            "rms": float(np.sqrt(np.mean(a * a))), "span_s": float(hi - lo)}


def seg_lags(t1, s1, t2, s2, seg, min_rms, min_corr=0.8, **kw):
    lo = max(t1[0], t2[0]); hi = min(t1[-1], t2[-1]); out = []; s = lo
    while s + seg <= hi + 1e-6:
        r = xcorr_lag(t1, s1, t2, s2, tmin=s, tmax=s + seg, **kw)
        if r and r["rms"] > min_rms and r["corr"] > min_corr:
            out.append({"start_s": round(s - lo, 1), "lag_ms": round(r["lag_ms"], 1), "corr": round(r["corr"], 3)})
        s += seg
    return out


def read_parameters(fusion_dir):
    p = {}
    f = pathlib.Path(fusion_dir) / "parameters.txt"
    if f.exists():
        for line in f.read_text().splitlines():
            m = re.match(r"([^=]+)=(.*)$", line)
            if m and m.group(1) not in p:
                p[m.group(1)] = m.group(2)
    return p


def fnum(p, key, default=float("nan")):
    try:
        return float(p.get(key))
    except (TypeError, ValueError):
        return default


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fusion_dir"); ap.add_argument("reference_csv")
    ap.add_argument("--lever", type=float, nargs=3, help="imu_to_rear_axle lever [m] (default from parameters.txt)")
    ap.add_argument("--window", type=float, default=20.0, help="regression window length [s]")
    ap.add_argument("--json", type=pathlib.Path)
    a = ap.parse_args()
    fusion = pathlib.Path(a.fusion_dir); ref = pathlib.Path(a.reference_csv)
    params = read_parameters(fusion)
    t_pos = fnum(params, "rtk.time_offset_sec", 0.0)
    t_att = fnum(params, "rtk.attitude_time_offset_sec", t_pos)
    lidar_frontend = fnum(params, "effective.lidar_time_offset_sec")
    if not np.isfinite(lidar_frontend):
        lidar_frontend = fnum(params, "time_offset.lidar_time_offset")
    lever_text = params.get("effective.imu_to_rear_axle") or params.get("reference_frame_conversion.imu_to_rear_axle", "0 0 0")
    lever = np.asarray(a.lever if a.lever else [float(v) for v in re.split(r"[,\s]+", lever_text.strip("[]")) if v], float)
    if not a.lever and not np.any(lever):
        # older rounds: read the lever from the mapper log
        launch = fusion.parent.parent.parent / "launch.log"
        if launch.exists():
            m = re.search(r"rear-axle lever \[([-\d.]+) ([-\d.]+) ([-\d.]+)\]", launch.read_text(errors="ignore"))
            if m:
                lever = np.asarray([float(m.group(i)) for i in (1, 2, 3)])
    out = {"fusion_dir": str(fusion), "effective": {"rtk_position_assoc_s": t_pos, "rtk_attitude_assoc_s": t_att,
                                                    "lidar_frontend_s": lidar_frontend}}
    imu = load_csv(ref / "imu_raw.csv", ["stamp", "gx", "gy", "gz"])
    ins = load_csv(ref / "ins_odometry.csv", ["stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz"])
    nat = load_traj(fusion / "trajectory_lio.txt")
    t0, t1 = nat[0, 0] - 1.0, nat[-1, 0] + 1.0
    sel = (ins["stamp"] >= t0 - 2) & (ins["stamp"] <= t1 + 2)
    ins = {k: v[sel] for k, v in ins.items()}
    sel = (imu["stamp"] >= t0 - 2) & (imu["stamp"] <= t1 + 2)
    imu = {k: v[sel] for k, v in imu.items()}
    print(f"round window {nat[-1,0]-nat[0,0]:.1f} s, native poses {len(nat)}, INS samples {len(ins['stamp'])}")
    print(f"effective offsets: scan<->IMU {lidar_frontend:+.4f} s | INS association position {t_pos:+.4f} s, attitude {t_att:+.4f} s | lever {lever}")

    # [1] INS chain
    tq = ins["stamp"]; q = np.stack([ins["qx"], ins["qy"], ins["qz"], ins["qw"]], 1)
    tm, w = body_rates(tq, q)
    r_att = xcorr_lag(tm, w[:, 2], imu["stamp"], imu["gz"])
    p = np.stack([ins["x"], ins["y"], ins["z"]], 1)
    v_pos = np.gradient(smooth(p, 21), tq, axis=0)
    sp_pos = np.linalg.norm(v_pos[:, :2], axis=1)
    sp_tw = np.linalg.norm(np.stack([ins["vx"], ins["vy"]], 1), axis=1)
    r_pv = xcorr_lag(tq, sp_pos, tq, sp_tw) if sp_tw.max() > 0.5 else None
    out["ins_chain"] = {"attitude_vs_gyro": r_att, "position_vs_velocity": r_pv}
    print("[1] INS chain: attitude(yaw-rate) vs gyro:", fmt(r_att), "| position-derived speed vs INS velocity:", fmt(r_pv))

    # [2] native yaw-rate vs gyro
    tn, wn = body_rates(nat[:, 0], nat[:, 4:8])
    r_nat = xcorr_lag(tn, wn[:, 2], imu["stamp"], imu["gz"])
    segs = seg_lags(tn, wn[:, 2], imu["stamp"], imu["gz"], 30.0, 0.02)
    out["native_vs_gyro"] = {"whole": r_nat, "segments_30s": segs}
    if r_nat:
        print(f"[2] native LIO yaw-rate vs gyro: {fmt(r_nat)}  (informational: insensitive to lidar_frontend, "
              f"current {lidar_frontend:+.4f} s; |lag| > 15 ms would indicate a stamping problem)")
        if segs:
            print("     30 s segments:", " ".join(f"{s['start_s']:.0f}s:{s['lag_ms']:+.1f}({s['corr']:.2f})" for s in segs))
    else:
        print("[2] native LIO yaw-rate vs gyro: insufficient rotation")

    # [3] along-track position association residual (native rear axle vs INS)
    Rn = quat_to_R(nat[:, 4:8])
    rear = nat[:, 1:4] + np.einsum("nij,j->ni", Rn, lever)
    tn2 = nat[:, 0]
    ins_at = np.stack([np.interp(tn2 + t_pos, tq, p[:, k]) for k in range(3)], 1)
    vel_at = np.stack([np.interp(tn2 + t_pos, tq, v_pos[:, k]) for k in range(3)], 1)
    speed = np.linalg.norm(vel_at[:, :2], axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        heading = vel_at[:, :2] / np.maximum(speed, 1e-6)[:, None]
    err = rear - ins_at
    e_along = np.einsum("ni,ni->n", err[:, :2], heading)
    e_cross = err[:, 0] * (-heading[:, 1]) + err[:, 1] * heading[:, 0]
    deltas = []
    s = tn2[0]
    while s + a.window <= tn2[-1] + 1e-6:
        m = (tn2 >= s) & (tn2 < s + a.window) & (speed > 0.3)
        if m.sum() > 30 and np.std(speed[m]) > 0.8:
            tt = tn2[m] - s
            A = np.column_stack([np.ones(m.sum()), tt, speed[m]])
            coef, *_ = np.linalg.lstsq(A, e_along[m], rcond=None)
            res = e_along[m] - A @ coef
            # robust re-fit
            keep = np.abs(res) < 3 * (np.median(np.abs(res)) / 0.6745 + 1e-3)
            if keep.sum() > 20:
                coef, *_ = np.linalg.lstsq(A[keep], e_along[m][keep], rcond=None)
            deltas.append({"start_s": round(s - tn2[0], 1), "delta_ms": round(1000 * coef[2], 1),
                           "speed_std": round(float(np.std(speed[m])), 2), "n": int(m.sum())})
        s += a.window / 2
    out["position_association"] = {"windows": deltas}
    if deltas:
        d = np.array([w_["delta_ms"] for w_ in deltas])
        med = float(np.median(d)); mad = float(np.median(np.abs(d - med)) / 0.6745)
        out["position_association"].update({"delta_ms_median": med, "delta_ms_mad": mad, "n_windows": len(d),
                                            "recommended_rtk_position_assoc_s": t_pos + med / 1000.0})
        print(f"[3] native rear-axle along-track error vs INS speed ({len(d)} windows of {a.window:.0f} s with speed variation): "
              f"association delta {med:+.1f} ms (MAD {mad:.1f}) -> recommended effective position association {t_pos + med/1000:+.4f} s "
              f"(current {t_pos:+.4f})")
        print("     windows:", " ".join(f"{w_['start_s']:.0f}s:{w_['delta_ms']:+.0f}" for w_ in deltas))
    else:
        print("[3] position association: no window with enough speed variation (need std > 0.8 m/s)")
    # informational: native vs INS error growth
    e3 = np.linalg.norm(err, axis=1)
    print(f"     native rear-axle vs INS: median {np.median(e3):.2f} m, end {e3[-1]:.2f} m (drift, informational)")
    if a.json:
        a.json.write_text(json.dumps(out, indent=2, default=lambda o: None if o is None else float(o)))


def fmt(r):
    return "n/a" if r is None else f"lag {r['lag_ms']:+.1f} ms (corr {r['corr']:.3f}, gain {r['gain']:.2f})"


if __name__ == "__main__":
    main()
