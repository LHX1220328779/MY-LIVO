#!/usr/bin/env python3
"""Signed pass-to-pass ground height offset of the delivered map on revisited road sections.

Revisit pairs: frame i (every --spacing m of path) and the closest later frame j within --radius m after
>= --min-path m of driving. Each pass contributes the returns (< --max-range m from the LiDAR, exact map
projection) of its scans within +-half m of path; ground cells (--cell m) in --crop m around the pair midpoint
get one robust plane with a per-pass height offset -> dz = later - earlier. Prints per-pair medians and route
statistics (median dz, median |dz|, P90 |dz|) in cm.

    python3 scripts/analysis/revisit_dz.py results/<round> [--json out.json]
(defaults = docs/mcp-pole-ghost-range-priority-rtk-z.md: radius 12 m, crop 10 m, returns < 15 m)
"""
import argparse, json, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools" / "livo"))
sys.path.insert(0, "tools/livo")
import map_export as me


def cloud(scans, fr, lt, lr, corr, raw_R, raw_t, ids, max_range, centre, crop):
    out = []
    for i in ids:
        cnt = int(fr["count"][i]); scans.seek(int(fr["offset_bytes"][i]))
        a = np.frombuffer(scans.read(cnt * 20), dtype="<f4").reshape(cnt, 5)
        p = a[:, :3].astype(np.float64); rng = np.linalg.norm(p, axis=1)
        m = np.isfinite(rng) & (rng > 1.0) & (rng < max_range)
        p, off = p[m], a[m, 4].astype(np.float64)
        t = np.clip(fr["begin"][i] + off / 1000.0, fr["begin"][i], fr["stamp"][i])
        w = me.apply_correction(p @ (raw_R[i] @ lr).T + (raw_R[i] @ lt + raw_t[i]), t, corr)
        k = np.hypot(w[:, 0] - centre[0], w[:, 1] - centre[1]) < crop
        out.append(w[k])
    return np.vstack(out) if out else np.zeros((0, 3))


def pair_dz(A, B, cell):
    P = np.vstack([A, B]); g = np.r_[np.zeros(len(A)), np.ones(len(B))]
    ck = np.floor(P[:, :2] / cell).astype(np.int64)
    kk = (ck[:, 0] + 2**20) * 2**21 + (ck[:, 1] + 2**20)
    order = np.argsort(kk, kind="stable"); kk, P, g = kk[order], P[order], g[order]
    st = np.flatnonzero(np.r_[True, kk[1:] != kk[:-1]]); en = np.r_[st[1:], len(kk)]
    dz = []
    for s, e in zip(st, en):
        X, G = P[s:e], g[s:e]
        if (G == 0).sum() < 15 or (G == 1).sum() < 15:
            continue
        zl = np.percentile(X[:, 2], 30)
        mm = np.abs(X[:, 2] - zl) < 0.25
        X, G = X[mm], G[mm]
        if (G == 0).sum() < 10 or (G == 1).sum() < 10:
            continue
        Am = np.column_stack([X[:, 0] - X[:, 0].mean(), X[:, 1] - X[:, 1].mean(), np.ones(len(X)), G])
        sol = np.linalg.lstsq(Am, X[:, 2], rcond=None)[0]
        for _ in range(3):
            r = X[:, 2] - Am @ sol
            k = np.abs(r) < max(3 * 1.4826 * np.median(np.abs(r)), 0.01)
            if k.sum() < 10:
                break
            sol = np.linalg.lstsq(Am[k], X[k, 2], rcond=None)[0]
        if np.hypot(sol[0], sol[1]) > 0.12:
            continue
        res = X[:, 2] - Am @ sol
        if 1.4826 * np.median(np.abs(res)) > 0.03:   # not a smooth ground cell
            continue
        dz.append(sol[3])
    return np.array(dz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--radius", type=float, default=12.0)
    ap.add_argument("--min-path", type=float, default=100.0); ap.add_argument("--spacing", type=float, default=8.0)
    ap.add_argument("--half", type=float, default=8.0); ap.add_argument("--max-range", type=float, default=15.0)
    ap.add_argument("--crop", type=float, default=10.0); ap.add_argument("--cell", type=float, default=1.0)
    ap.add_argument("--max-pairs", type=int, default=150); ap.add_argument("--json")
    a = ap.parse_args()
    run = me.resolve_run(a.run)
    fr, lt, lr = me.read_frames(run); corr = me.read_corrections(run)
    raw_R = me.quat_to_rot(np.stack([fr["qx"], fr["qy"], fr["qz"], fr["qw"]], 1))
    raw_t = np.stack([fr["tx"], fr["ty"], fr["tz"]], 1)
    tr = np.loadtxt(run / "trajectory_optimized.txt")
    idx = np.clip(np.searchsorted(tr[:, 0], fr["stamp"] - 1e-6), 0, len(tr) - 1)
    pos = tr[idx, 1:3]
    path = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
    pairs, last = [], -1e9
    for i in range(len(pos)):
        if path[i] - last < a.spacing:
            continue
        later = np.flatnonzero(path > path[i] + a.min_path)
        if not len(later):
            break
        d = np.linalg.norm(pos[later] - pos[i], axis=1)
        if d.min() < a.radius:
            pairs.append((i, int(later[np.argmin(d)]))); last = path[i]
    if len(pairs) > a.max_pairs:
        pairs = [pairs[k] for k in np.linspace(0, len(pairs) - 1, a.max_pairs).astype(int)]
    res = []
    with open(run / "scans.bin", "rb") as scans:
        for i, j in pairs:
            centre = (pos[i] + pos[j]) / 2
            ia = np.flatnonzero(np.abs(path - path[i]) < a.half); ib = np.flatnonzero(np.abs(path - path[j]) < a.half)
            A = cloud(scans, fr, lt, lr, corr, raw_R, raw_t, ia, a.max_range, centre, a.crop)
            B = cloud(scans, fr, lt, lr, corr, raw_R, raw_t, ib, a.max_range, centre, a.crop)
            dz = pair_dz(A, B, a.cell)
            if len(dz) >= 5:
                res.append({"t_a": float(fr["stamp"][i] - fr["stamp"][0]), "t_b": float(fr["stamp"][j] - fr["stamp"][0]),
                            "cells": int(len(dz)), "dz_cm": 100 * float(np.median(dz)), "x": float(centre[0]), "y": float(centre[1])})
    v = np.array([r["dz_cm"] for r in res])
    summ = {"run": str(run), "pairs": len(res)}
    if len(v):
        summ.update(median_dz_cm=round(float(np.median(v)), 2), median_abs_dz_cm=round(float(np.median(np.abs(v))), 2),
                    p90_abs_dz_cm=round(float(np.percentile(np.abs(v), 90)), 2), max_abs_dz_cm=round(float(np.abs(v).max()), 2))
    for r in res:
        print(f"  t {r['t_a']:6.0f} -> {r['t_b']:6.0f}  ({r['x']:8.1f},{r['y']:8.1f})  cells {r['cells']:4d}  dz {r['dz_cm']:+5.2f} cm")
    print(json.dumps(summ))
    if a.json:
        json.dump({"summary": summ, "pairs": res}, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
