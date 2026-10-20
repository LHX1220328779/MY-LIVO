#!/usr/bin/env python3
"""Lamp-pole ghosting on revisited roads, measured on a finished round (no SLAM re-run).

1. Poles: tall thin columns of map/global_map.pcd (0.1 m cells whose 0.25 m slices 0.5-10 m above the local
   ground are >= 7.5 m occupied, compact footprint, empty 0.6-1.5 m annulus), or --poles poles.json.
2. Every scan that sees a pole (sensor within --range m) is re-projected exactly like the delivered map
   (native pose * extrinsic, point-time correction C(t), tools/livo/map_export.py); the shaft returns
   (< 0.6 m from the axis, 1-7 m above ground) are split into passes (time gaps > --gap s).
3. One circle per 0.5 m slice shared by all groups + one 2-D offset per group (robust least squares): a
   one-sided arc does not bias the offset. Reported per pole: pass-to-pass offset (along / cross the first
   pass's travel direction) with every return and with returns < --near m only; pooled over poles: the
   offset of each sensor-range bin against the < 12 m bin, split into radial (along the line of sight,
   + = away from the sensor) and tangential (rotation-like) parts.
4. --map-shell: shell width of the DELIVERED map_N.pcd rows around each pole (P10-P90 of the radial
   residual to one circle per 0.5 m slice), i.e. how thick / doubled the pole looks in the saved map.

    /usr/bin/python3 scripts/analysis/pole_revisit.py results/<round> [--map-shell] [--poles poles.json]
"""
import argparse
import json
import pathlib
import sys

import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools" / "livo"))
import map_export as me  # noqa: E402


def read_pcd_xyz(path):
    with open(path, "rb") as f:
        fields, n = [], 0
        while True:
            line = f.readline()
            if line.startswith(b"FIELDS"):
                fields = line.split()[1:]
            elif line.startswith(b"POINTS"):
                n = int(line.split()[1])
            elif line.startswith(b"DATA"):
                break
        return np.fromfile(f, dtype="<f4", count=n * len(fields)).reshape(n, len(fields))[:, :3]


def detect_poles(xyz):
    x, y, z = (xyz[:, k].astype(np.float64) for k in range(3))
    x0, y0 = x.min() - 2, y.min() - 2
    gx = ((x - x0) // 1.0).astype(np.int64); gy = ((y - y0) // 1.0).astype(np.int64)
    nx, ny = int(gx.max()) + 3, int(gy.max()) + 3
    zmin = np.full(nx * ny, np.inf)
    np.minimum.at(zmin, gx * ny + gy, z)
    zmin = np.where(np.isfinite(zmin), zmin, np.nan).reshape(nx, ny)

    def low(v):
        v = v[np.isfinite(v)]
        return np.percentile(v, 20) if len(v) else np.nan
    ground = ndimage.generic_filter(zmin, low, size=5, mode="constant", cval=np.nan)
    h = z - ground[gx, gy]
    m = (h > 0.5) & (h < 10.0)
    cell = 0.1
    bx = ((x[m] - x0) // cell).astype(np.int64); by = ((y[m] - y0) // cell).astype(np.int64)
    bz = ((h[m] - 0.5) // 0.25).astype(np.int64)
    mx, my = int(nx / cell) + 2, int(ny / cell) + 2
    key = np.unique((bx * my + by) * 64 + bz)
    cov = np.bincount(key // 64, minlength=mx * my).reshape(mx, my)
    lab, _ = ndimage.label(ndimage.maximum_filter(cov, size=3) >= 22, structure=np.ones((3, 3)))
    poles = []
    for sl in ndimage.find_objects(lab):
        if sl is None or sl[0].stop - sl[0].start > 7 or sl[1].stop - sl[1].start > 7:
            continue
        ci, cj = (sl[0].start + sl[0].stop) // 2, (sl[1].start + sl[1].stop) // 2
        win = cov[max(ci - 15, 0):ci + 16, max(cj - 15, 0):cj + 16].astype(float)
        ii, jj = np.mgrid[:win.shape[0], :win.shape[1]]
        rr = np.hypot(ii + max(ci - 15, 0) - ci, jj + max(cj - 15, 0) - cj) * cell
        if win[rr <= 0.3].max() < 30 or win[(rr > 0.6) & (rr < 1.5)].mean() > 5:
            continue
        cx, cy = x0 + ci * cell, y0 + cj * cell
        g = float(ground[int((cx - x0) // 1.0), int((cy - y0) // 1.0)])
        for _ in range(2):
            mm = (np.abs(x - cx) < 0.4) & (np.abs(y - cy) < 0.4) & (h > 1.0) & (h < 7.0)
            cx, cy = float(np.median(x[mm])), float(np.median(y[mm]))
        poles.append({"cx": cx, "cy": cy, "ground": g})
    return poles


def kasa(xy):
    a = np.column_stack([2 * xy, np.ones(len(xy))])
    s = np.linalg.lstsq(a, (xy ** 2).sum(1), rcond=None)[0]
    return s[:2], float(np.sqrt(max(s[2] + s[:2] @ s[:2], 1e-9)))


def joint_fit(xy, sl, grp, ngrp, ref):
    """Shared circle per slice + one 2-D offset per group (group `ref` fixed at 0); returns offsets, residuals."""
    slices = np.unique(sl); sidx = np.searchsorted(slices, sl); ns = len(slices)
    c0 = np.zeros((ns, 2)); r0 = np.zeros(ns)
    for i in range(ns):
        c0[i], r0[i] = kasa(xy[sidx == i])
    free = [g for g in range(ngrp) if g != ref]

    def unpack(p):
        d = np.zeros((ngrp, 2))
        if free:
            d[free] = p[3 * ns:].reshape(-1, 2)
        return p[:2 * ns].reshape(ns, 2), p[2 * ns:3 * ns], d

    def fun(p):
        c, r, d = unpack(p)
        q = xy - c[sidx] - d[grp]
        return np.hypot(q[:, 0], q[:, 1]) - r[sidx]
    res = least_squares(fun, np.concatenate([c0.ravel(), r0, np.zeros(2 * len(free))]), loss="soft_l1",
                        f_scale=0.02, max_nfev=200)
    c, r, d = unpack(res.x)
    return d, fun(res.x), float(np.median(r))


def group_fit(D, ground, labels, min_n=60):
    if len(D) < 2 * min_n:
        return None
    names, inv = np.unique(labels, return_inverse=True)
    ok = np.isin(inv, np.flatnonzero(np.bincount(inv) >= min_n))
    sl = np.floor((D[:, 3] - ground - 1.0) / 0.5).astype(int)
    ok &= np.isin(sl, np.flatnonzero(np.bincount(sl - sl.min()) >= 30) + sl.min())
    if len(np.unique(inv[ok])) < 2:
        return None
    used, g = np.unique(inv[ok], return_inverse=True)
    d, resid, radius = joint_fit(D[ok, 1:3], sl[ok], g, len(used), int(np.argmax(np.bincount(g))))
    return {names[u]: (d[i], int((g == i).sum()), ok, g == i) for i, u in enumerate(used)}, resid, radius


def map_shell(run, poles):
    centres = np.array([[p["cx"], p["cy"]] for p in poles]); rows = [[] for _ in poles]
    for path in sorted(run.glob("map_*.pcd"), key=lambda p: int(p.stem.split("_")[-1])):
        a = read_pcd_xyz(path)
        for j, (cx, cy) in enumerate(centres):
            m = (np.abs(a[:, 0] - cx) < 0.6) & (np.abs(a[:, 1] - cy) < 0.6)
            if m.any():
                rows[j].append(a[m])
    out = []
    for j, p in enumerate(poles):
        if not rows[j]:
            out.append(None)
            continue
        P = np.vstack(rows[j]).astype(np.float64); h = P[:, 2] - p["ground"]; res = []
        for z0 in np.arange(1.0, 7.0, 0.5):
            m = (h >= z0) & (h < z0 + 0.5)
            if m.sum() < 30:
                continue
            xy = P[m, :2]; c, r = kasa(xy); keep = np.ones(len(xy), bool)
            for _ in range(8):
                d = np.hypot(*(xy - c).T) - r
                keep = np.abs(d) < max(3 * 1.4826 * np.median(np.abs(d[keep])), 0.02)
                c, r = kasa(xy[keep])
            if r < 0.3:
                res.append(np.hypot(*(xy - c).T) - r)
        if not res:
            out.append(None)
            continue
        res = np.concatenate(res); res = res[np.abs(res) < 0.4]
        q10, q90 = np.percentile(res, [10, 90])
        out.append({"rows": int(len(res)), "width_cm": round(100 * (q90 - q10), 1),
                    "gt5cm_pct": round(100 * float(np.mean(np.abs(res) > 0.05)), 1)})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--poles", help="poles.json (cx, cy, ground) instead of detection")
    ap.add_argument("--range", type=float, default=40.0, help="scans whose sensor is within this of a pole (m)")
    ap.add_argument("--near", type=float, default=20.0, help="second pass-offset estimate with returns < this (m)")
    ap.add_argument("--gap", type=float, default=8.0, help="time gap separating passes (s)")
    ap.add_argument("--map-shell", action="store_true", help="also measure the poles in the saved map_N.pcd")
    ap.add_argument("--json", help="write the report here")
    a = ap.parse_args()
    run = me.resolve_run(a.run)
    poles = json.load(open(a.poles)) if a.poles else detect_poles(read_pcd_xyz(run / "map" / "global_map.pcd"))
    print(f"{run}: {len(poles)} poles")
    if not poles:
        return
    fr, lt, lr = me.read_frames(run); corr = me.read_corrections(run)
    raw_R = me.quat_to_rot(np.stack([fr["qx"], fr["qy"], fr["qz"], fr["qw"]], 1))
    raw_t = np.stack([fr["tx"], fr["ty"], fr["tz"]], 1)
    st = fr["stamp"]
    tr = np.loadtxt(run / "trajectory_optimized.txt", ndmin=2)
    pos = tr[np.clip(np.searchsorted(tr[:, 0], st - 1e-6), 0, len(tr) - 1), 1:3]
    vel = np.gradient(pos, st, axis=0)
    P = np.array([[p["cx"], p["cy"]] for p in poles]); G = np.array([p["ground"] for p in poles])
    dist = np.hypot(pos[:, None, 0] - P[None, :, 0], pos[:, None, 1] - P[None, :, 1])
    near = (dist < a.range) & (dist > 3.0)
    shaft = [[] for _ in poles]
    with open(run / "scans.bin", "rb") as scans:
        for i in np.flatnonzero(near.any(1)):
            n = int(fr["count"][i]); scans.seek(int(fr["offset_bytes"][i]))
            raw = np.frombuffer(scans.read(n * 20), "<f4").reshape(n, 5)
            p = raw[:, :3].astype(np.float64); rng = np.linalg.norm(p, axis=1)
            t = np.clip(fr["begin"][i] + raw[:, 4].astype(np.float64) / 1000.0, fr["begin"][i], st[i])
            w = me.apply_correction(p @ (raw_R[i] @ lr).T + (raw_R[i] @ lt + raw_t[i]), t, corr)
            for j in np.flatnonzero(near[i]):
                hz = w[:, 2] - G[j]
                m = (np.hypot(w[:, 0] - P[j, 0], w[:, 1] - P[j, 1]) < 0.6) & (hz > 1.0) & (hz < 7.0)
                if m.any():
                    shaft[j].append(np.column_stack([np.full(m.sum(), st[i]), w[m], rng[m], np.full(m.sum(), i)]))
    shells = map_shell(run, poles) if a.map_shell else [None] * len(poles)
    report, pairs_all, pairs_near, rbins = [], [], [], {}
    for j, pole in enumerate(poles):
        rec = {"pole": j, "cx": round(pole["cx"], 2), "cy": round(pole["cy"], 2), "map_shell": shells[j]}
        report.append(rec)
        if not shaft[j]:
            continue
        D = np.concatenate(shaft[j])
        fidx = np.flatnonzero(near[:, j])
        passes = []
        for b in np.split(fidx, np.flatnonzero(np.diff(st[fidx]) > a.gap) + 1):
            if len(b) >= 5:
                k = b[np.argmin(dist[b, j])]
                passes.append((st[b[0]], st[b[-1]], vel[k] / max(np.linalg.norm(vel[k]), 1e-6), dist[k, j]))
        pid = np.full(len(D), -1)
        for q, (t0, t1, _u, _c) in enumerate(passes):
            pid[(D[:, 0] >= t0 - 1e-6) & (D[:, 0] <= t1 + 1e-6)] = q
        rec["passes"] = [{"t0": round(t0 - st[0], 1), "closest_m": round(c, 1)} for t0, _t1, _u, c in passes]
        for tag, keep, sink in (("all", pid >= 0, pairs_all), ("near", (pid >= 0) & (D[:, 4] < a.near), pairs_near)):
            fit = group_fit(D[keep], pole["ground"], pid[keep].astype(str))
            if fit is None:
                continue
            groups, resid, radius = fit
            qs = sorted(groups, key=int)
            for q1, q2 in zip(qs[:-1], qs[1:]):
                u = passes[int(q1)][2]; dv = groups[q2][0] - groups[q1][0]
                e = {"a": int(q1), "b": int(q2), "d_cm": round(100 * float(np.hypot(*dv)), 1),
                     "along_cm": round(100 * float(dv @ u), 1), "cross_cm": round(100 * float(dv @ [-u[1], u[0]]), 1)}
                rec.setdefault(f"pairs_{tag}", []).append(e); sink.append(e["d_cm"])
            rec[f"radius_{tag}_cm"] = round(100 * radius, 1)
        # sensor-range bins against the < 12 m bin: radial / tangential to the mean line of sight
        sens = pos[D[:, 5].astype(int)]
        los = P[j] - sens; los /= np.linalg.norm(los, axis=1)[:, None]
        rb = np.digitize(D[:, 4], [12, 16, 20, 25, 30])
        fit = group_fit(D, pole["ground"], rb.astype(str))
        if fit is not None and "0" in fit[0]:
            groups = fit[0]; base = groups["0"][0]
            for b, (d, n, ok, sel) in groups.items():
                if b == "0":
                    continue
                L = los[ok][sel].mean(0); L /= np.linalg.norm(L); dv = d - base
                rbins.setdefault(int(b), []).append((100 * float(dv @ L), 100 * float(dv @ [-L[1], L[0]]),
                                                     float(np.median(D[ok][sel, 4]))))
    for rec in report:
        pa = " ".join(f"{e['a']}->{e['b']} {e['d_cm']:.1f} ({e['along_cm']:+.1f}/{e['cross_cm']:+.1f})" for e in rec.get("pairs_all", []))
        pn = " ".join(f"{e['a']}->{e['b']} {e['d_cm']:.1f} ({e['along_cm']:+.1f}/{e['cross_cm']:+.1f})" for e in rec.get("pairs_near", []))
        sh = rec["map_shell"]
        print(f"  pole {rec['pole']:2d} ({rec['cx']:8.2f},{rec['cy']:8.2f}) closest "
              f"{'/'.join(str(p['closest_m']) for p in rec.get('passes', []))} m | all {pa or '-'} | <{a.near:.0f} m {pn or '-'}"
              + (f" | map shell {sh['width_cm']:.1f} cm >5cm {sh['gt5cm_pct']:.1f}%" if sh else ""))
    summary = {"run": str(run), "poles": len(poles)}
    for tag, v in (("pass_offset_all_cm", pairs_all), ("pass_offset_near_cm", pairs_near)):
        if v:
            summary[tag] = {"pairs": len(v), "median": round(float(np.median(v)), 1),
                            "p90": round(float(np.percentile(v, 90)), 1), "max": round(float(np.max(v)), 1)}
    labels = ["12-16", "16-20", "20-25", "25-30", ">=30"]
    summary["range_bins_vs_lt12m"] = {
        labels[b - 1]: {"groups": len(v), "range_m": round(float(np.median([e[2] for e in v])), 1),
                        "radial_cm": round(float(np.median([e[0] for e in v])), 1),
                        "abs_tangential_cm": round(float(np.median([abs(e[1]) for e in v])), 1)}
        for b, v in sorted(rbins.items())}
    widths = [s["width_cm"] for s in shells if s]
    if widths:
        summary["map_shell_width_cm"] = {"median": round(float(np.median(widths)), 1), "max": round(float(max(widths)), 1)}
    print(json.dumps(summary, ensure_ascii=False))
    if a.json:
        json.dump({"summary": summary, "poles": report}, open(a.json, "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
