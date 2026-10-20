#!/usr/bin/env python3
"""Standard map products of a MY-LIVO fusion run (re-runnable on any finished round, no SLAM re-run).

The mapper archives every deskewed scan (scans.bin, native LiDAR frame), its native pose (frames.csv) and the
RTK/INS correction knots of the final optimization (corrections.txt). This tool rebuilds the delivered map from
them with exactly the mapper's own projection (appendCloud): point time t = scan begin + offset, native world
point = native scan-end pose * LiDAR extrinsic * p, delivered point = C(t) * native, where C(t) is the SE(3)
(Lie) interpolation of the correction knots (held before the first / after the last knot).

Products (default <fusion_run>/map/):
  global_map.pcd          voxel-filtered geometry map (centroid of every voxel; x y z intensity), binary PCD
  global_map_rgb.pcd      LIVO only: per voxel the colour of the nearest / slowest observation (rgb_map logic)
  tiles/tile_<i>_<j>.pcd  optional square tiles (--tile M) of the same products for large sites
  keyframes/              optional (--keyframe-clouds): per-keyframe body-frame clouds + poses.txt (TUM)
  map_info.json           frame, voxel size, source, ranges, counts, bounds, time span, inputs, calibration

Source: --source all (every scan, densest) | keyframes (only the scans at backend keyframes: ~5x lighter,
same surfaces). --voxel 0 keeps every return (no filtering).
Range priority (same rule as the mapper's map_N.pcd, default from the round's rtk.save_map_priority_*): per
0.5 m cell the nearest scan with >= 2 returns there sets r_min; returns farther than max(ratio * r_min,
r_min + margin) are dropped before the voxel filter (--priority-ratio 0 = off). With the round's settings
(no --max-range / --min-range / --priority-*) and its map_rows.bin, the rows are taken from map_rows.bin, i.e.
exactly the mapper's decision including its seen-free cells (rtk.save_map_priority_free_range_m).

    /usr/bin/python3 tools/livo/map_export.py results/<round> [--voxel 0.05] [--source all] [--tile 100]
"""
import argparse
import datetime as dt
import json
import os
import pathlib
import re
import shutil
import sys
import tempfile
import time

import numpy as np

HEADER_SCANS = b"LIVORTK3"
HEADER_COLORS = b"LIVORGB1"


# ----------------------------------------------------------------------------------------------- SE(3) helpers
def quat_to_rot(q):
    """q: (..., 4) x y z w -> (..., 3, 3)"""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    r = np.empty(q.shape[:-1] + (3, 3))
    r[..., 0, 0] = 1 - 2 * (y * y + z * z); r[..., 0, 1] = 2 * (x * y - z * w); r[..., 0, 2] = 2 * (x * z + y * w)
    r[..., 1, 0] = 2 * (x * y + z * w); r[..., 1, 1] = 1 - 2 * (x * x + z * z); r[..., 1, 2] = 2 * (y * z - x * w)
    r[..., 2, 0] = 2 * (x * z - y * w); r[..., 2, 1] = 2 * (y * z + x * w); r[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def rot_log(r):
    c = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if th < 1e-9:
        return 0.5 * w
    return th / (2 * np.sin(th)) * w


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def se3_log(R, t):
    """GTSAM Pose3::Logmap convention: [omega, v] with t = V(omega) v."""
    w = rot_log(R)
    th = np.linalg.norm(w)
    W = skew(w)
    if th < 1e-9:
        vinv = np.eye(3) - 0.5 * W
    else:
        vinv = np.eye(3) - 0.5 * W + (1 - th * np.sin(th) / (2 * (1 - np.cos(th)))) / (th * th) * (W @ W)
    return w, vinv @ t


# ----------------------------------------------------------------------------------------------- inputs
def resolve_run(path):
    p = pathlib.Path(path).resolve()
    if (p / "frames.csv").exists():
        return p
    runs = sorted((p / "Log" / "fusion").glob("*/frames.csv"))
    if not runs:
        raise SystemExit(f"{path}: no fusion run with frames.csv (was the round mapped with save_map?)")
    return runs[-1].parent


def read_frames(run):
    """frames.csv -> dict of arrays + LiDAR extrinsic (virtual LiDAR -> IMU)."""
    lt, lr = np.zeros(3), np.eye(3)
    names = None
    rows = []
    with open(run / "frames.csv") as f:
        for line in f:
            if line.startswith("# lidar_to_imu_translation"):
                lt = np.array([float(v) for v in line.split()[2:5]])
            elif line.startswith("# lidar_to_imu_rotation_row_major"):
                lr = np.array([float(v) for v in line.split()[2:11]]).reshape(3, 3)
            elif line.startswith("#"):
                continue
            elif names is None:
                names = line.strip().split(",")
            elif line.strip():
                rows.append([float(v) for v in line.strip().split(",")])
    a = np.asarray(rows, float).reshape(-1, len(names))
    cols = {n: a[:, i] for i, n in enumerate(names)}
    return cols, lt, lr


def read_corrections(run):
    rows = []
    for line in open(run / "corrections.txt"):
        if line.startswith("#") or not line.strip():
            continue
        rows.append([float(v) for v in line.split()[:8]])
    a = np.asarray(rows, float)
    t = a[:, 0]
    R = quat_to_rot(a[:, 4:8])
    T = a[:, 1:4]
    # per interval Lie increment xi_k = Log(C_k^-1 C_{k+1})
    K = len(t)
    om = np.zeros((max(K - 1, 0), 3)); vv = np.zeros((max(K - 1, 0), 3))
    for k in range(K - 1):
        Rr = R[k].T @ R[k + 1]
        tr = R[k].T @ (T[k + 1] - T[k])
        om[k], vv[k] = se3_log(Rr, tr)
    return t, R, T, om, vv


def se3_exp(w, v):
    th = np.linalg.norm(w)
    W = skew(w)
    if th < 1e-9:
        return np.eye(3) + W, (np.eye(3) + 0.5 * W) @ v
    R = np.eye(3) + np.sin(th) / th * W + (1 - np.cos(th)) / th ** 2 * (W @ W)
    V = np.eye(3) + (1 - np.cos(th)) / th ** 2 * W + (th - np.sin(th)) / th ** 3 * (W @ W)
    return R, V @ v


def correction_at(corr, stamp):
    """Exact C(stamp) (rotation, translation): a * Exp(r * Log(a^-1 b)) between knots, held outside."""
    kt, R, T, om, vv = corr
    k = int(np.searchsorted(kt, stamp, side="right") - 1)
    if k < 0:
        return R[0], T[0]
    if k >= len(kt) - 1:
        return R[-1], T[-1]
    r = (stamp - kt[k]) / (kt[k + 1] - kt[k])
    dR, dt_ = se3_exp(r * om[k], r * vv[k])
    return R[k] @ dR, R[k] @ dt_ + T[k]


def apply_correction(points, times, corr):
    """points (N,3) native world, times (N,) -> delivered world (N,3); first-order Lie interpolation inside a knot
    interval (error < 1e-5 m for the observed knot increments), hold outside the knot span."""
    kt, R, T, om, vv = corr
    out = np.empty_like(points)
    k = np.searchsorted(kt, times, side="right") - 1
    before, after = k < 0, k >= len(kt) - 1
    if before.any():
        out[before] = points[before] @ R[0].T + T[0]
    if after.any():
        out[after] = points[after] @ R[-1].T + T[-1]
    inside = ~(before | after)
    if inside.any():
        ks = k[inside]
        for kk in np.unique(ks):
            m = np.flatnonzero(inside)[ks == kk]
            r = ((times[m] - kt[kk]) / (kt[kk + 1] - kt[kk]))[:, None]
            x = points[m]
            y = x + r * (np.cross(om[kk], x) + vv[kk])
            out[m] = y @ R[kk].T + T[kk]
    return out


def parameters(run):
    out = {}
    try:
        for line in open(run / "parameters.txt"):
            k, _, v = line.strip().partition("=")
            if k and v and k not in out:
                out[k] = v
    except OSError:
        pass
    return out


# ----------------------------------------------------------------------------------------------- PCD output
def pcd_header(fields, n):
    size = {"x": 4, "y": 4, "z": 4, "intensity": 4, "rgb": 4}
    typ = {"x": "F", "y": "F", "z": "F", "intensity": "F", "rgb": "U"}
    return (f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {' '.join(fields)}\n"
            f"SIZE {' '.join(str(size[f]) for f in fields)}\nTYPE {' '.join(typ[f] for f in fields)}\n"
            f"COUNT {' '.join('1' for _ in fields)}\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\nDATA binary\n").encode()


class PcdWriter:
    """Streams records to a temp file and writes header + data on close (point count known only at the end)."""

    def __init__(self, path, fields, dtype):
        self.path, self.fields, self.dtype = pathlib.Path(path), fields, dtype
        self.tmp = self.path.with_suffix(self.path.suffix + ".part")
        self.f = open(self.tmp, "wb")
        self.n = 0

    def write(self, rec):
        if len(rec):
            self.f.write(np.ascontiguousarray(rec, dtype=self.dtype).tobytes())
            self.n += len(rec)

    def close(self):
        self.f.close()
        with open(self.path, "wb") as out, open(self.tmp, "rb") as src:
            out.write(pcd_header(self.fields, self.n))
            shutil.copyfileobj(src, out, 64 << 20)
        self.tmp.unlink()
        return self.n


XYZI = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
XYZRGB = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
# Projection spill record of the geometry map: + sensor range and scan index for the range-priority rule.
GEOREC = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("r", "<f4"), ("f", "<i4")])


# ----------------------------------------------------------------------------------------------- tiling buffers
class TileBuffer:
    """Per-tile point buffers spilled to <dir>/<prefix>t<tile>.bin beyond `limit` points (bounded memory);
    several projection workers write disjoint prefixes into the same directory."""

    def __init__(self, tmpdir, dtype, limit=6_000_000, prefix=""):
        self.tmpdir, self.dtype, self.limit, self.prefix = pathlib.Path(tmpdir), dtype, limit, prefix
        self.mem, self.count = {}, 0

    def add(self, tile_ids, rec):
        order = np.argsort(tile_ids, kind="stable")
        tid, rec = tile_ids[order], rec[order]
        cuts = np.flatnonzero(np.diff(tid)) + 1
        for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(tid)]):
            self.mem.setdefault(int(tid[a]), []).append(rec[a:b])
        self.count += len(rec)
        if self.count > self.limit:
            self.spill()

    def spill(self):
        for t, parts in self.mem.items():
            with open(self.tmpdir / f"{self.prefix}t{t}.bin", "ab") as f:
                for p in parts:
                    f.write(p.tobytes())
        self.mem, self.count = {}, 0

    @staticmethod
    def tiles_on_disk(tmpdir):
        return sorted({int(re.search(r"t(-?\d+)\.bin$", p.name).group(1)) for p in pathlib.Path(tmpdir).glob("*t*.bin")})

    @staticmethod
    def load_all(tmpdir, tile_id, dtype):
        parts = [np.fromfile(p, dtype=dtype) for p in sorted(pathlib.Path(tmpdir).glob(f"*t{tile_id}.bin"))]
        for p in pathlib.Path(tmpdir).glob(f"*t{tile_id}.bin"):
            p.unlink()
        return np.concatenate(parts) if parts else np.zeros(0, dtype)


TILE_BIAS = 1 << 20


def tile_key(xy, size, voxel=0.0):
    """Tile id of every point. With a voxel size the tile is derived from the voxel index (a whole number of
    voxels per tile), so a voxel never straddles two tiles and is reduced exactly once."""
    if voxel > 0:
        per = max(1, int(round(size / voxel)))
        ij = np.floor_divide(np.floor(xy / voxel).astype(np.int64), per)
    else:
        ij = np.floor(xy / size).astype(np.int64)
    return (ij[:, 0] + TILE_BIAS) * (1 << 21) + (ij[:, 1] + TILE_BIAS)


def tile_ij(key):
    return int(key // (1 << 21) - TILE_BIAS), int(key % (1 << 21) - TILE_BIAS)


def voxel_reduce_xyzi(rec, voxel, min_points=1):
    """Voxel centroid + mean intensity. min_points > 1 drops voxels supported by fewer returns: isolated noise
    (dust, rain, multipath), and objects that crossed the scene only briefly (passing vehicles, people) - a static
    surface near the route collects tens to hundreds of returns per 5 cm voxel over the scans of the map."""
    if voxel <= 0 or not len(rec):
        return rec
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], 1).astype(np.float64)
    v = np.floor(xyz / voxel).astype(np.int64)
    v -= v.min(0)
    key = (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]
    uniq, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    out = np.empty(len(uniq), XYZI)
    for name, src in (("x", xyz[:, 0]), ("y", xyz[:, 1]), ("z", xyz[:, 2]), ("intensity", rec["intensity"])):
        out[name] = np.bincount(inv, weights=src, minlength=len(uniq)) / cnt
    return out[cnt >= min_points] if min_points > 1 else out


def voxel_select_rgb(rec, score, voxel):
    """Per voxel keep the record with the lowest score (nearest / slowest observation)."""
    if not len(rec):
        return rec
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], 1).astype(np.float64)
    v = np.floor(xyz / voxel).astype(np.int64)
    v -= v.min(0)
    key = (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]
    order = np.lexsort((score, key))
    k = key[order]
    first = np.r_[True, k[1:] != k[:-1]]
    return rec[order[first]]


def range_priority(rec, ratio, margin, cell):
    """Range priority of the geometry map (the mapper's exportFrame rule): per `cell` the nearest range of a scan
    with >= 2 returns there is r_min; keep returns with r <= max(ratio * r_min, r_min + margin). Cells without
    such a scan keep everything. Tiles are whole numbers of 0.05 m voxels and multiples of the cell, so a cell
    never straddles two tiles."""
    if ratio <= 0 or not len(rec):
        return rec
    xyz = np.stack([rec["x"], rec["y"], rec["z"]], 1).astype(np.float64)
    v = np.floor(xyz / cell).astype(np.int64)
    v -= v.min(0)
    key = (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]
    order = np.lexsort((rec["f"], key))
    k, f, r = key[order], rec["f"][order], rec["r"][order]
    starts = np.flatnonzero(np.r_[True, (k[1:] != k[:-1]) | (f[1:] != f[:-1])])   # (cell, scan) groups
    gmin = np.minimum.reduceat(r, starts)
    supported = np.diff(np.r_[starts, len(k)]) >= 2
    sk, sm = k[starts][supported], gmin[supported]
    if not len(sk):
        return rec
    first = np.flatnonzero(np.r_[True, sk[1:] != sk[:-1]])
    cells, rmin = sk[first], np.minimum.reduceat(sm, first)
    pos = np.minimum(np.searchsorted(cells, k), len(cells) - 1)
    found = cells[pos] == k
    limit = np.where(found, np.maximum(ratio * rmin[pos], rmin[pos] + margin), np.inf)
    keep = np.empty(len(rec), bool)
    keep[order] = r <= limit
    return rec[keep]


# ----------------------------------------------------------------------------------------------- main export
RGBREC = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"), ("s", "<f4")])
_CTX = {}


def _project_frames(job):
    """Worker: project a contiguous range of frames, bucket the delivered points into per-tile spill files."""
    wid, frame_ids = job
    c = _CTX
    a, run, fr, lt, lr, corr = c["a"], c["run"], c["fr"], c["lt"], c["lr"], c["corr"]
    raw_R, raw_t, tmp = c["raw_R"], c["raw_t"], c["tmp"]
    geo = TileBuffer(tmp / "geo", GEOREC, limit=6_000_000, prefix=f"w{wid}_")
    rgbbuf = TileBuffer(tmp / "rgb", RGBREC, limit=6_000_000, prefix=f"w{wid}_") if c["rgb_wanted"] else None
    st = {"returns_in": 0, "returns_kept": 0, "returns_in_cap": 0, "t0": None, "t1": None, "kf": []}
    n_frames = len(fr["stamp"])
    rows = np.memmap(c["rows"], dtype=np.uint8, mode="r") if c["rows"] else None
    with open(run / "scans.bin", "rb") as scans:
        colors = open(run / "colors.bin", "rb") if rgbbuf is not None else None
        for i in frame_ids:
            cnt = int(fr["count"][i])
            if cnt <= 0:
                continue
            scans.seek(int(fr["offset_bytes"][i]))
            pts = np.frombuffer(scans.read(cnt * 20), dtype="<f4").reshape(cnt, 5)
            st["returns_in"] += cnt
            p = pts[:, :3].astype(np.float64)
            keep = np.isfinite(p).all(1)
            rng = np.linalg.norm(p, axis=1)
            in_cap = keep & (rng <= c["max_range"]) if c["max_range"] > 0 else keep
            st["returns_in_cap"] += int(np.count_nonzero(in_cap))
            if rows is not None:   # the mapper's decision (its float32 cap, range priority, seen-free cells)
                s0 = int(c["rows_start"][i])
                keep &= np.unpackbits(rows[s0:s0 + (cnt + 7) // 8], bitorder="little")[:cnt].astype(bool)
            else:
                keep = in_cap
            if a.min_range > 0:
                keep &= rng >= a.min_range
            begin, stamp = fr["begin"][i], fr["stamp"][i]
            if keep.any():
                p, inten, off = p[keep], pts[keep, 3], pts[keep, 4].astype(np.float64)
                times = np.clip(begin + off / 1000.0, begin, stamp)
                native = p @ (raw_R[i] @ lr).T + (raw_R[i] @ lt + raw_t[i])
                world = apply_correction(native, times, corr)
                rec = np.empty(len(world), GEOREC)
                rec["x"], rec["y"], rec["z"], rec["intensity"] = world[:, 0], world[:, 1], world[:, 2], inten
                rec["r"], rec["f"] = rng[keep], i
                geo.add(tile_key(world[:, :2], c["tile"], a.voxel), rec)
                st["returns_kept"] += len(rec)
                st["t0"] = stamp if st["t0"] is None else min(st["t0"], stamp)
                st["t1"] = stamp if st["t1"] is None else max(st["t1"], stamp)
                ordinal = c["kf_ordinal"].get(round(float(stamp), 6)) if a.keyframe_clouds else None
                if ordinal is not None:
                    body = p @ lr.T + lt  # IMU body frame (scan-end, deskewed)
                    kr = np.empty(len(body), XYZI)
                    kr["x"], kr["y"], kr["z"], kr["intensity"] = body[:, 0], body[:, 1], body[:, 2], inten
                    name = f"kf_{ordinal:06d}.pcd"
                    with open(c["kf_dir"] / name, "wb") as f:
                        f.write(pcd_header(["x", "y", "z", "intensity"], len(kr)))
                        f.write(kr.tobytes())
                    Rc, tc = correction_at(corr, stamp)
                    Rw, tw = Rc @ raw_R[i], Rc @ raw_t[i] + tc
                    st["kf"].append((ordinal, float(stamp), *map(float, tw), *rot_to_quat(Rw), name))
            if rgbbuf is not None and int(fr["color_count"][i]) > 0:
                cc = int(fr["color_count"][i])
                colors.seek(int(fr["color_offset_bytes"][i]))
                col = np.frombuffer(colors.read(cc * 24), dtype="<f4").reshape(cc, 6).astype(np.float64)
                origin = raw_R[i] @ lt + raw_t[i]
                rng_c = np.linalg.norm(col[:, :3] - origin, axis=1)
                m = rng_c <= c["rgb_range"] if c["rgb_range"] > 0 else np.ones(cc, bool)
                if m.any():
                    omega = 0.0
                    if i + 1 < n_frames:
                        dtf = max(fr["stamp"][i + 1] - stamp, 1e-3)
                        omega = np.linalg.norm(rot_log(raw_R[i].T @ raw_R[i + 1])) / dtf
                    cw = apply_correction(col[m, :3], np.full(int(m.sum()), stamp), corr)
                    r8, g8, b8 = (np.clip(col[m, k], 0, 255).astype(np.uint32) for k in (3, 4, 5))
                    rr = np.empty(len(cw), RGBREC)
                    rr["x"], rr["y"], rr["z"] = cw[:, 0], cw[:, 1], cw[:, 2]
                    rr["rgb"] = (r8 << 16) | (g8 << 8) | b8
                    rr["s"] = rng_c[m] + 20.0 * omega  # nearest observation wins; fast rotation penalised
                    rgbbuf.add(tile_key(cw[:, :2], c["tile"], a.rgb_voxel), rr)
        if colors:
            colors.close()
    geo.spill()
    if rgbbuf is not None:
        rgbbuf.spill()
    return st


def _reduce_tile(job):
    """Worker: merge one tile from every projection worker, voxel-reduce it, write <tmp>/red/<kind>_<tile>.bin."""
    kind, tile_id = job
    c = _CTX
    tmp, a = c["tmp"], c["a"]
    n_in = n_prio = 0
    if kind == "geo":
        rec = TileBuffer.load_all(tmp / "geo", tile_id, GEOREC)
        n_in = len(rec)
        rec = range_priority(rec, *c["priority"])
        n_prio = len(rec)
        out = voxel_reduce_xyzi(rec, a.voxel, a.min_points)
    else:
        rr = TileBuffer.load_all(tmp / "rgb", tile_id, RGBREC)
        sel = voxel_select_rgb(rr, rr["s"], a.rgb_voxel)
        out = np.empty(len(sel), XYZRGB)
        for k in ("x", "y", "z", "rgb"):
            out[k] = sel[k]
    path = tmp / "red" / f"{kind}_{tile_id}.bin"
    out.tofile(path)
    if len(out):
        xyz = np.stack([out["x"], out["y"], out["z"]], 1)
        return kind, tile_id, len(out), xyz.min(0).tolist(), xyz.max(0).tolist(), n_in, n_prio
    return kind, tile_id, 0, None, None, n_in, n_prio


def export(a):
    import multiprocessing as mp
    t_start = time.time()
    run = resolve_run(a.run)
    out = pathlib.Path(a.out) if a.out else run / "map"
    out.mkdir(parents=True, exist_ok=True)
    prm = parameters(run)
    fr, lt, lr = read_frames(run)
    corr = read_corrections(run)
    n_frames = len(fr["stamp"])
    sel = np.ones(n_frames, bool)
    kf_stamps = None
    if a.source == "keyframes" or a.keyframe_clouds:
        kf_stamps = np.loadtxt(run / "keyframes.csv", delimiter=",", skiprows=1, usecols=0, ndmin=1)
    if a.source == "keyframes":
        idx = np.minimum(np.searchsorted(fr["stamp"], kf_stamps - 1e-6), n_frames - 1)
        ok = np.abs(fr["stamp"][idx] - kf_stamps) < 1e-5
        sel[:] = False
        sel[idx[ok]] = True
    max_range = a.max_range
    if max_range is None:
        try:
            max_range = float(prm.get("rtk.save_map_max_range", "0"))
        except ValueError:
            max_range = 0.0
    rgb_wanted = bool(a.rgb and (run / "colors.bin").exists() and "color_count" in fr and fr["color_count"].sum() > 0)
    rgb_range = a.rgb_max_range
    if rgb_wanted and rgb_range is None:
        try:
            rgb_range = float(prm.get("rtk.save_rgb_max_range", "-1"))
        except ValueError:
            rgb_range = -1.0
        if rgb_range < 0:
            rgb_range = max_range
    priority = []
    for name, key, default in (("priority_ratio", "rtk.save_map_priority_ratio", 1.5),
                               ("priority_margin", "rtk.save_map_priority_margin_m", 2.0),
                               ("priority_cell", "rtk.save_map_priority_voxel_m", 0.5)):
        value = getattr(a, name)
        if value is None:
            try:
                value = float(prm.get(key, default))
            except ValueError:
                value = default
        priority.append(float(value))
    if priority[0] != 0 and (priority[0] < 1 or priority[1] < 0 or priority[2] <= 0):
        raise SystemExit("range priority: ratio 0 (off) or >= 1, margin >= 0, cell > 0")
    # The round's settings + map_rows.bin: take the mapper's own row decision (incl. its seen-free cells).
    rows_path, rows_start = run / "map_rows.bin", None
    if (rows_path.exists() and a.max_range is None and a.min_range <= 0 and a.priority_ratio is None
            and a.priority_margin is None and a.priority_cell is None):
        counts = fr["count"].astype(np.int64)
        rows_start = 8 + np.concatenate([[0], np.cumsum((counts + 7) // 8)[:-1]])
        head = np.fromfile(rows_path, dtype=np.uint8, count=8)
        if bytes(head) != b"LIVOMSK1" or rows_path.stat().st_size != int(rows_start[-1] + (counts[-1] + 7) // 8):
            raise SystemExit(f"{rows_path} does not match frames.csv")
    tile = a.tile if a.tile > 0 else 50.0
    if priority[0] > 0 and a.voxel > 0 and abs(tile / priority[2] - round(tile / priority[2])) > 1e-9:
        raise SystemExit("range priority: the tile size must be a multiple of the priority cell")
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="map_export_", dir=out))
    for d in ("geo", "rgb", "red"):
        (tmp / d).mkdir()
    kf_dir = out / "keyframes"
    if a.keyframe_clouds:
        kf_dir.mkdir(exist_ok=True)
    _CTX.update(a=a, run=run, fr=fr, lt=lt, lr=lr, corr=corr, tmp=tmp, tile=tile, max_range=max_range or 0.0,
                priority=(0.0, priority[1], priority[2]) if rows_start is not None else tuple(priority),
                rows=str(rows_path) if rows_start is not None else None, rows_start=rows_start,
                rgb_wanted=rgb_wanted, rgb_range=(rgb_range or 0.0) if rgb_wanted else 0.0, kf_dir=kf_dir,
                raw_R=quat_to_rot(np.stack([fr["qx"], fr["qy"], fr["qz"], fr["qw"]], 1)),
                raw_t=np.stack([fr["tx"], fr["ty"], fr["tz"]], 1),
                kf_ordinal={round(float(s), 6): k for k, s in enumerate(kf_stamps)} if kf_stamps is not None else {})
    frames_sel = np.flatnonzero(sel)
    workers = max(1, min(a.jobs, len(frames_sel) // 50 or 1))
    chunks = [(k, part) for k, part in enumerate(np.array_split(frames_sel, workers)) if len(part)]
    ctx = mp.get_context("fork")
    with ctx.Pool(workers) as pool:
        parts = pool.map(_project_frames, chunks)
    stats = {"frames_total": int(n_frames), "frames_used": int(len(frames_sel)),
             "returns_in": int(sum(p["returns_in"] for p in parts)), "returns_kept": int(sum(p["returns_kept"] for p in parts)),
             "returns_in_cap": int(sum(p["returns_in_cap"] for p in parts))}
    t0s = [p["t0"] for p in parts if p["t0"] is not None]
    t1s = [p["t1"] for p in parts if p["t1"] is not None]
    t_span = [min(t0s) if t0s else None, max(t1s) if t1s else None]
    kf_poses = sorted(row for p in parts for row in p["kf"])
    t_project = time.time() - t_start
    # ---------------------------------------------------------------- reduce per tile (parallel), assemble products
    jobs = [("geo", t) for t in TileBuffer.tiles_on_disk(tmp / "geo")]
    if rgb_wanted:
        jobs += [("rgb", t) for t in TileBuffer.tiles_on_disk(tmp / "rgb")]
    with ctx.Pool(max(1, min(a.jobs, 6))) as pool:
        reduced = pool.map(_reduce_tile, jobs, chunksize=1)
    stats["returns_range_priority"] = int(sum(r[6] for r in reduced if r[0] == "geo"))
    files, tile_counts = {}, {}
    bounds = [np.full(3, np.inf), np.full(3, -np.inf)]
    tile_dir = out / "tiles"
    if a.tile > 0:
        tile_dir.mkdir(exist_ok=True)
    for kind, fname, fields, dtype in (("geo", "global_map.pcd", ["x", "y", "z", "intensity"], XYZI),
                                      ("rgb", "global_map_rgb.pcd", ["x", "y", "z", "rgb"], XYZRGB)):
        items = sorted(r for r in reduced if r[0] == kind)
        if not items:
            continue
        w = PcdWriter(out / fname, fields, dtype)
        for _k, tid, n, lo, hi, _n_in, _n_prio in items:
            if not n:
                continue
            data = np.fromfile(tmp / "red" / f"{kind}_{tid}.bin", dtype=dtype)
            w.write(data)
            if kind == "geo":
                bounds[0] = np.minimum(bounds[0], lo); bounds[1] = np.maximum(bounds[1], hi)
            if a.tile > 0:
                i, j = tile_ij(tid)
                name = f"tile_{i}_{j}{'_rgb' if kind == 'rgb' else ''}.pcd"
                with open(tile_dir / name, "wb") as f:
                    f.write(pcd_header(fields, n))
                    f.write(data.tobytes())
                tile_counts[name] = int(n)
        files[fname] = w.close()
    shutil.rmtree(tmp, ignore_errors=True)
    if a.keyframe_clouds:
        with open(kf_dir / "poses.txt", "w") as f:
            f.write("# stamp x y z qx qy qz qw file  (delivered IMU pose in the map frame; cloud in the IMU body frame)\n")
            for row in kf_poses:
                f.write(" ".join(f"{v:.9f}" if isinstance(v, float) else str(v) for v in row[1:]) + "\n")
    round_dir = run.parents[2] if run.parent.name == "fusion" else None
    manifest = {}
    if round_dir and (round_dir / "round.json").exists():
        try:
            manifest = json.loads((round_dir / "round.json").read_text())
        except json.JSONDecodeError:
            manifest = {}
    info = {
        "format": "MY-LIVO map products v1",
        "created": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "fusion_run": str(run), "round": manifest.get("round"), "mode": manifest.get("mode"),
        "calibration": manifest.get("calibration") or prm.get("multi_lidar.calibration_file"),
        "frame": {"name": "mine", "description": "local metric frame of the INS odometry (x east, y north, z up); "
                  "trajectory_optimized.txt / corrections.txt use the same frame", "units": "m"},
        "projection": "point time t = scan begin + offset; delivered = C(t) * (native scan-end pose * LiDAR extrinsic * p); "
                      "C(t) = SE(3) interpolation of the final correction knots (identical to the mapper's map_N.pcd, <1e-5 m)",
        "source": a.source, "voxel_m": a.voxel, "min_points_per_voxel": a.min_points,
        "rgb_voxel_m": a.rgb_voxel if rgb_wanted else None,
        "range_m": {"min": a.min_range, "max": max_range or 0.0, "rgb_max": rgb_range if rgb_wanted else None},
        "range_priority": {"ratio": priority[0], "margin_m": priority[1], "cell_m": priority[2],
                           "rows": "map_rows.bin (mapper decision incl. seen-free cells)" if rows_start is not None
                           else "recomputed here (occupied-cell rule)",
                           "returns_in": stats["returns_in_cap"] if rows_start is not None else stats["returns_kept"],
                           "returns_kept": stats["returns_range_priority"]},
        "tile_m": a.tile if a.tile > 0 else None, "tiles": tile_counts or None,
        "points": files, "frames": stats,
        "time_span": {"first": t_span[0], "last": t_span[1]},
        "bounds_m": {"min": [round(float(v), 3) for v in bounds[0]], "max": [round(float(v), 3) for v in bounds[1]]}
        if np.isfinite(bounds[0]).all() else None,
        "keyframe_clouds": len(kf_poses) if a.keyframe_clouds else None,
        "fields": {"global_map.pcd": "x y z intensity (float32; voxel centroid and mean intensity)",
                   "global_map_rgb.pcd": "x y z rgb (packed uint32 0x00RRGGBB; nearest/slowest observation per voxel)"},
        "elapsed_s": round(time.time() - t_start, 1), "project_s": round(t_project, 1), "jobs": a.jobs,
    }
    (out / "map_info.json").write_text(json.dumps(info, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"out": str(out), "points": files, "frames_used": stats["frames_used"],
                      "returns_kept": stats["returns_kept"], "elapsed_s": info["elapsed_s"]}, ensure_ascii=False))
    return info


def rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return float(x), float(y), float(z), float(w)


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="round directory (results/<round>) or fusion run directory (Log/fusion/<stamp>)")
    ap.add_argument("--out", help="output directory (default <fusion_run>/map)")
    ap.add_argument("--voxel", type=float, default=0.05, help="geometry voxel size in m (0 = keep every return)")
    ap.add_argument("--source", choices=("all", "keyframes"), default="all",
                    help="all scans (densest) or only the scans of backend keyframes (~5x fewer, same surfaces)")
    ap.add_argument("--min-range", type=float, default=0.0, help="drop returns closer than this to the LiDAR (m)")
    ap.add_argument("--max-range", type=float, default=None,
                    help="drop returns farther than this (m, 0 = none; default: the round's rtk.save_map_max_range)")
    ap.add_argument("--priority-ratio", type=float, default=None,
                    help="range priority: keep r <= max(ratio*r_min, r_min+margin) per cell (0 = off; default: the round's)")
    ap.add_argument("--priority-margin", type=float, default=None, help="range priority margin (m; default: the round's)")
    ap.add_argument("--priority-cell", type=float, default=None, help="range priority cell (m; default: the round's)")
    ap.add_argument("--tile", type=float, default=0.0, help="also write square tiles of this size (m, 0 = global file only)")
    ap.add_argument("--min-points", type=int, default=1,
                    help="drop geometry voxels with fewer returns (noise / briefly seen moving objects; 1 = keep all)")
    ap.add_argument("--rgb", dest="rgb", action="store_true", default=True)
    ap.add_argument("--no-rgb", dest="rgb", action="store_false", help="skip the colour map (LIVO rounds)")
    ap.add_argument("--rgb-voxel", type=float, default=0.03, help="colour map voxel (m)")
    ap.add_argument("--rgb-max-range", type=float, default=None, help="colour range cap (m; default: the round's)")
    ap.add_argument("--jobs", type=int, default=min(8, max(1, (os.cpu_count() or 2) // 3)),
                    help="parallel worker processes (projection and per-tile reduction)")
    ap.add_argument("--keyframe-clouds", action="store_true",
                    help="also write per-keyframe body-frame clouds + poses.txt (relocalization / map editing)")
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    if a.voxel < 0 or a.rgb_voxel <= 0 or a.tile < 0 or a.min_range < 0 or a.min_points < 1:
        raise SystemExit("--voxel >= 0, --rgb-voxel > 0, --tile >= 0, --min-range >= 0")
    export(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
