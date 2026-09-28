#!/usr/bin/env python3
"""Fused rear-axle error vs the INS/RTK reference split into horizontal along-track / cross-track and vertical,
computed exactly like scripts/analysis/round_summary.py (same lever, same time offset, same interpolation).
Usage: xyz_err.py <run> [<run> ...]   (run = results/<round>)"""
import sys, pathlib, numpy as np
from scipy.spatial.transform import Rotation as R

def fusion_dir(run):
    return run if (run / "frames.csv").exists() else sorted((run / "Log" / "fusion").glob("*"))[-1]

def interp_pos(t_ref, p_ref, t):
    return np.column_stack([np.interp(t, t_ref, p_ref[:, k]) for k in range(3)])

def main():
    for arg in sys.argv[1:]:
        run = pathlib.Path(arg); fd = fusion_dir(run)
        params = {}
        for line in (fd / "parameters.txt").read_text().splitlines():
            if "=" in line and not line.startswith("model="):
                k, v = line.split("=", 1); params.setdefault(k.strip(), v.strip())
        lever = np.array([float(v) for v in params["effective.imu_to_rear_axle"].split()])
        toff = float(params.get("rtk.time_offset_sec", 0.0))
        ref = run / "evaluation" / "reference_csv" / "ins_odometry.csv"
        ins = np.genfromtxt(ref, delimiter=",", names=True, usecols=range(14))
        tins = ins["stamp"] - toff
        pins = np.stack([ins["x"], ins["y"], ins["z"]], 1)
        vins = np.stack([ins["vx"], ins["vy"], ins["vz"]], 1)
        opt = np.loadtxt(fd / "trajectory_optimized.txt")
        t = opt[:, 0]; rq = R.from_quat(opt[:, 4:8]); rear = opt[:, 1:4] + rq.apply(lever)
        refp = interp_pos(tins, pins, t)
        err = rear - refp
        # travel direction from the reference position (1 s central difference)
        fwd = interp_pos(tins, pins, t + 0.5) - interp_pos(tins, pins, t - 0.5)
        speed = np.linalg.norm(fwd[:, :2], axis=1)
        d = fwd[:, :2] / np.maximum(speed[:, None], 1e-6)
        along = err[:, 0] * d[:, 0] + err[:, 1] * d[:, 1]
        cross = -err[:, 0] * d[:, 1] + err[:, 1] * d[:, 0]
        xy = np.hypot(err[:, 0], err[:, 1]); z = err[:, 2]
        slope = fwd[:, 2] / np.maximum(speed, 1e-6)  # reference grade
        yaw = np.degrees(np.unwrap(np.arctan2(d[:, 1], d[:, 0]))); yawrate = np.gradient(yaw, t)
        t0 = t[0]; tr = t - t0
        print(f"== {run.name}: {len(t)} frames {tr[-1]:.0f} s; xy median/P95/max {np.median(xy)*100:.2f}/{np.percentile(xy,95)*100:.2f}/{xy.max()*100:.2f} cm (max at {tr[xy.argmax()]:.0f} s, speed {speed[xy.argmax()]:.1f} m/s); z median/P95/max |z| {np.median(np.abs(z))*100:.1f}/{np.percentile(np.abs(z),95)*100:.1f}/{np.abs(z).max()*100:.1f} cm (max at {tr[np.abs(z).argmax()]:.0f} s)")
        mv = speed > 0.5
        print(f"   moving frames: along-track signed median {np.median(along[mv])*100:+.2f} cm (IQR {np.percentile(along[mv],25)*100:+.2f}..{np.percentile(along[mv],75)*100:+.2f}), cross-track {np.median(cross[mv])*100:+.2f} cm (IQR {np.percentile(cross[mv],25)*100:+.2f}..{np.percentile(cross[mv],75)*100:+.2f})")
        for lo, hi in ((0.5, 3), (3, 5), (5, 9)):
            m = (speed >= lo) & (speed < hi)
            if m.sum() > 20:
                print(f"   speed {lo}-{hi} m/s n={m.sum():5d}: along median {np.median(along[m])*100:+.2f} P95|.| {np.percentile(np.abs(along[m]),95)*100:.2f} | cross median {np.median(cross[m])*100:+.2f} P95|.| {np.percentile(np.abs(cross[m]),95)*100:.2f} | xy max {xy[m].max()*100:.2f} | z median {np.median(z[m])*100:+.1f}")
        # along-track vs speed regression -> equivalent time offset
        A = np.column_stack([speed[mv], np.ones(mv.sum())]); coef = np.linalg.lstsq(A, along[mv], rcond=None)[0]
        print(f"   along-track vs speed fit: {coef[0]*1000:+.1f} ms equivalent, intercept {coef[1]*100:+.2f} cm")
        print("   worst 10 s windows by xy max:   window   xymax  xymed  along  cross   z_med  speed  yawrate  grade")
        rows = []
        for b0 in np.arange(0, tr[-1], 10):
            m = (tr >= b0) & (tr < b0 + 10)
            if m.sum() < 5: continue
            rows.append((xy[m].max(), b0, np.median(xy[m]), np.median(along[m]), np.median(cross[m]), np.median(z[m]), speed[m].mean(), np.abs(yawrate[m]).mean(), np.median(slope[m])))
        rows.sort(reverse=True)
        for mx, b0, med, al, cr, zm, sp, yr, gr in rows[:10]:
            print(f"      {b0:4.0f}-{b0+10:4.0f}  {mx*100:5.2f}  {med*100:5.2f}  {al*100:+5.2f}  {cr*100:+5.2f}  {zm*100:+6.1f}  {sp:5.2f}  {yr:6.1f}  {gr*100:+5.1f}%")
        rows.sort(key=lambda r: -abs(r[5]))
        print("   worst 10 s windows by |z| median:")
        for mx, b0, med, al, cr, zm, sp, yr, gr in rows[:6]:
            print(f"      {b0:4.0f}-{b0+10:4.0f}  {mx*100:5.2f}  {med*100:5.2f}  {al*100:+5.2f}  {cr*100:+5.2f}  {zm*100:+6.1f}  {sp:5.2f}  {yr:6.1f}  {gr*100:+5.1f}%")
        # reference self-consistency. ins_odometry velocity is in the child frame (rear axle RFU): compare speeds,
        # and rotate it to the world with the INS attitude to integrate height (pitch-vs-height consistency).
        qins = np.stack([ins["qx"], ins["qy"], ins["qz"], ins["qw"]], 1)
        step = max(1, int(round(0.1 / np.median(np.diff(tins)))))  # 10 Hz
        ti = tins[::step]; pi = pins[::step]; vb = vins[::step]; qi = qins[::step]
        vw = R.from_quat(qi).apply(vb)
        vpos = np.gradient(pi, ti, axis=0)
        spd_i = np.linalg.norm(vw[:, :2], axis=1); spd_p = np.linalg.norm(vpos[:, :2], axis=1); mvr = spd_i > 0.5
        print(f"   reference self-check (10 Hz): |speed_pos - speed_ins| median {np.median(np.abs(spd_p - spd_i)[mvr])*100:.1f} P95 {np.percentile(np.abs(spd_p - spd_i)[mvr],95)*100:.1f} cm/s; vertical |vz_pos - vz_ins| median {np.median(np.abs(vpos[:,2]-vw[:,2])[mvr])*100:.1f} P95 {np.percentile(np.abs(vpos[:,2]-vw[:,2])[mvr],95)*100:.1f} cm/s ({mvr.sum()/10:.0f} moving s)")
        zi = np.concatenate([[0], np.cumsum(0.5 * (vw[1:, 2] + vw[:-1, 2]) * np.diff(ti))])
        resid = (pi[:, 2] - pi[0, 2]) - zi
        print(f"   reference height minus INS-attitude-integrated height: end {resid[-1]*100:+.0f} cm, max |.| {np.abs(resid).max()*100:.0f} cm (over {ti[-1]-ti[0]:.0f} s; a consistent INS/RTK pair stays within a few cm per minute)")
        # reference roughness: position minus 1 s centred moving average, per 10 s window, vs our xy error there
        k = 10; ker = np.ones(k) / k
        sm = np.column_stack([np.convolve(pi[:, j], ker, mode="same") for j in range(3)])
        rough = np.hypot(pi[:, 0] - sm[:, 0], pi[:, 1] - sm[:, 1]); roughz = np.abs(pi[:, 2] - sm[:, 2])
        tri = ti - t0
        print("   reference roughness (xy / z high-pass RMS, cm) in the worst xy windows vs the run median:")
        med_r = np.sqrt(np.mean(rough[mvr] ** 2)) * 100; med_rz = np.sqrt(np.mean(roughz[mvr] ** 2)) * 100
        print(f"      run (moving): xy {med_r:.2f} z {med_rz:.2f}")
        rows.sort(reverse=True)
        for mx, b0, *_ in rows[:6]:
            m = (tri >= b0) & (tri < b0 + 10) & mvr
            if m.sum() > 10:
                print(f"      {b0:4.0f}-{b0+10:4.0f}: xy err max {mx*100:5.2f} | ref roughness xy {np.sqrt(np.mean(rough[m]**2))*100:.2f} z {np.sqrt(np.mean(roughz[m]**2))*100:.2f} cm | speed_pos-speed_ins P95 {np.percentile(np.abs(spd_p - spd_i)[m],95)*100:.1f} cm/s")

if __name__ == "__main__":
    main()
