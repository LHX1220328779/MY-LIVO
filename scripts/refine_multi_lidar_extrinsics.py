#!/usr/bin/env python3
"""Data-driven refinement of the manual multi-LiDAR extrinsics (lidar -> rear axle).

Observable degrees of freedom only. Ground overlap between LiDARs observes the relative
roll/pitch/height of each LiDAR; yaw and horizontal offsets are solved only if enough
vertical planar overlap exists (otherwise they are kept from the manual calibration).
Stationary start frames (no ego-motion, aggregated) are used for the side LiDARs; the back
LiDAR only overlaps the sides while driving, so it uses per-frame matches from a moving
segment (same scan, same vehicle pose; ground z is insensitive to the <0.5 m intra-scan
travel). The correction is a small rigid transform about the LiDAR's own position in the
rear-axle frame, applied on top of the manual matrix; the original JSON is never modified.

Convergence basin. The point-to-plane solve matches within 0.6 m and carries a 1 deg / 10 cm
prior, i.e. it is a *refinement* of a calibration that is already roughly right. A LiDAR whose
mount moved since the calibration file was written (truck29 2026-10-13: front_left_lidar
6.0 deg fore-aft / 3 deg lateral / 40 cm off the file of 2026-08-05) is outside that basin:
the solve walked away (ground residual 15.5 -> 20.3 cm), the stage still wrote the result,
and the driven stage then rightly rejected the >2 deg follow-up correction, so the pipeline
died. Each LiDAR is therefore first compared with the reference through its *own* RANSAC
ground plane (no matching involved): when the two planes differ by more than the refinement
scale, the closed-form tilt/height alignment of the planes becomes the start of the solve and
the prior is centred on that start. The fine solve itself is unchanged, so a calibration that
is already within the basin is processed bit-identically. A fine result that regresses the
ground residual is never written: the better of the plane start / manual matrix is kept, and
every LiDAR carries a quality verdict (ok / warn) in the report and in calibration.json.

Usage (container):
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 scripts/refine_multi_lidar_extrinsics.py \
      --calibration /home/project/data/haibo/wuhu_livo/sensor-imu_calibration/perception_truck29.json \
      --bag results/prepared/wuhu_livo --output config/perception_truck29_refined.json \
      --report results/calibration_refinement/report.txt
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
SHORT_RECORDING_S = 90.0
# a correction beyond the refinement scale is still applied (the ground observes tilt/z directly) but flagged:
# the sensor moved relative to the calibration file and yaw/x/y have to be re-checked by the driven stage
LARGE_ROT_DEG = 1.0
LARGE_TRANS_M = 0.10


def read_frames(bag, first, count):
    """Frames [first, first+count) of every LiDAR topic, read directly from the raw recording."""
    dataset = Dataset.open(bag)
    topics = list(KEYS)
    clouds = {t: [] for t in topics}; counts = {t: 0 for t in topics}
    for topic, data, _ in dataset.iter_messages(topics):
        topic = topic.split("/")[-1] if topic.split("/")[-1] in counts else topic
        if topic not in counts: continue
        counts[topic] += 1
        if counts[topic] <= first: continue
        if counts[topic] > first + count:
            if all(c > first + count for c in counts.values()): break
            continue
        msg = deserialize_message(data, PointCloud2)
        arr = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        clouds[topic].append(np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(float))
    return clouds  # list of frames per topic


def voxel_down(P, size):
    key = np.floor(P / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[np.sort(idx)]


def clean(Q, rmin=3.0, rmax=60.0):
    r = np.hypot(Q[:, 0], Q[:, 1])
    body = (Q[:, 0] > -2.8) & (Q[:, 0] < 2.8) & (Q[:, 1] > -4.5) & (Q[:, 1] < 9.5)
    return Q[(r > rmin) & (r < rmax) & ~body & np.isfinite(Q).all(1)]


def stationary_check(frames, rmin=3.0, rmax=60.0, voxel=0.2):
    """Median / P90 nearest-neighbour displacement between the first and the last frame of a LiDAR.
    Identical scenes from a standing vehicle differ only by the scan pattern (~0.1 m median here);
    any ego-motion adds its travel distance directly."""
    def prep(P):
        r = np.linalg.norm(P, axis=1); P = P[(r > rmin) & (r < rmax)]
        return voxel_down(P, voxel)
    A, B = prep(frames[0]), prep(frames[-1])
    d, _ = cKDTree(A).query(B)
    return {"median_m": float(np.median(d)), "p90_m": float(np.percentile(d, 90))}


def fit_ground_plane(P, thr=0.08, iters=300, min_nz=0.9, max_z=1.0, max_range=40.0, seed=0):
    """Dominant near-horizontal plane of P (RANSAC on 3-point samples, then three least-squares refits on the
    inliers). Points above max_z (canopies, berms seen from a badly tilted LiDAR) and beyond max_range (lever arm
    of a wrong tilt) are left out of the fit. Returns (n, d, inliers) with n_z > 0 and n . p + d = 0, or None."""
    P = P[(P[:, 2] < max_z) & (np.hypot(P[:, 0], P[:, 1]) < max_range)]
    n_pts = len(P)
    if n_pts < 200:
        return None
    rng = np.random.default_rng(seed); best = None
    for _ in range(iters):
        a, b, c = P[rng.choice(n_pts, 3, replace=False)]
        nv = np.cross(b - a, c - a); nn = np.linalg.norm(nv)
        if nn < 1e-9: continue
        nv /= nn
        if abs(nv[2]) < min_nz: continue
        cnt = int((np.abs(P @ nv + (-nv @ a)) < thr).sum())
        if best is None or cnt > best[0]:
            best = (cnt, nv, -nv @ a)
    if best is None:
        return None
    inl = np.abs(P @ best[1] + best[2]) < thr
    for _ in range(3):
        Q = P[inl]
        if len(Q) < 50: return None
        c = Q.mean(0); _, v = np.linalg.eigh(np.cov((Q - c).T)); nv = v[:, 0]
        if nv[2] < 0: nv = -nv
        d = -nv @ c; inl = np.abs(P @ nv + d) < thr
    return nv, float(d), int(inl.sum())


def plane_init(src, ref, center, min_inliers=300):
    """Closed-form roll/pitch/z start of one LiDAR from its own ground plane against the reference's.

    src / ref: lists of point arrays in the rear-axle frame taken at the same vehicle poses (one aggregated array
    for the stationary case, one array per scan for the moving back LiDAR). Per pair: the minimal rotation about
    `center` (the LiDAR position) that takes the source normal onto the reference normal, and the z shift that
    puts the LiDAR at the reference height above the ground (the rotation about the sensor keeps its own height
    above its plane). Median over the pairs. Returns dict(rot, trans, tilt_deg, fore_aft_deg, lateral_deg, dz_m,
    pairs) or None when no plane pair is usable."""
    rots, dzs, tilts, fa, lat = [], [], [], [], []
    for S, Rf in zip(src, ref):
        ps, pr = fit_ground_plane(S), fit_ground_plane(Rf, seed=1)
        if ps is None or pr is None or ps[2] < min_inliers or pr[2] < min_inliers:
            continue
        ns, ds, _ = ps; nr, dr, _ = pr
        axis = np.cross(ns, nr); s = np.linalg.norm(axis); c = float(np.clip(ns @ nr, -1.0, 1.0))
        ang = float(np.arctan2(s, c))
        rots.append(axis / s * ang if s > 1e-12 else np.zeros(3))
        h_s = float(ns @ center + ds); h_r = float(nr @ center + dr)   # LiDAR height above each plane
        dzs.append((h_s - h_r) / nr[2])
        tilts.append(np.degrees(ang))
        # rear-axle frame: x right, y forward, z up -> fore-aft tilt is about x, lateral about y
        fa.append(np.degrees(np.arctan2(ns[1], ns[2]) - np.arctan2(nr[1], nr[2])))
        lat.append(np.degrees(np.arctan2(ns[0], ns[2]) - np.arctan2(nr[0], nr[2])))
    if not rots:
        return None
    rot = np.median(np.array(rots), axis=0); dz = float(np.median(dzs))
    return dict(rot=rot, trans=np.array([0.0, 0.0, dz]), tilt_deg=float(np.median(tilts)),
                fore_aft_deg=float(np.median(fa)), lateral_deg=float(np.median(lat)), dz_m=dz, pairs=len(rots))


class Target:
    def __init__(self, A, k=12):
        self.A = A; self.tree = cKDTree(A)
        d, idx = self.tree.query(A, k=k); nbr = A[idx]; c = nbr.mean(1)
        cov = np.einsum("nki,nkj->nij", nbr - c[:, None], nbr - c[:, None]) / k
        w, v = np.linalg.eigh(cov)
        self.normals = v[:, :, 0]
        self.planar = (w[:, 0] < 0.0025) & (w[:, 1] > 4 * w[:, 0] + 1e-9)

    def match(self, B, max_dist):
        dist, j = self.tree.query(B, distance_upper_bound=max_dist)
        ok = np.isfinite(dist); ok[ok] &= self.planar[j[ok]]
        return ok, self.normals[j[ok]], self.A[j[ok]]


def solve(pairs, center, solve_yaw_xy, iterations=MAX_ITERS, max_dist=0.6, huber=0.05,
          prior_rot_deg=1.0, prior_trans_m=0.10, x0=None):
    """pairs: list of (Target, B points). Correction: p' = Rc (p - center) + center + t.
    x0: optional start (rotvec, translation); the prior is centred on it, so x0=None (zero) is the historic
    behaviour and a plane start only moves the basin, not the optimum."""
    x0 = np.zeros(6) if x0 is None else np.asarray(x0, dtype=float)
    rot = x0[:3].copy(); trans = x0[3:].copy()
    active = np.array([True, True, solve_yaw_xy, solve_yaw_xy, solve_yaw_xy, True])  # rx, ry, rz, tx, ty, tz
    stats = None
    for it in range(iterations):
        Rm = R.from_rotvec(rot).as_matrix()
        H = np.zeros((6, 6)); g = np.zeros(6); rows = []
        for target, B in pairs:
            Bt = (B - center) @ Rm.T + center + trans
            ok, n, q = target.match(Bt, max_dist)
            if not ok.any(): continue
            p = Bt[ok]; r = np.einsum("ni,ni->n", n, p - q)
            if not solve_yaw_xy:
                keep = np.abs(n[:, 2]) > 0.8   # ground only observes rx, ry, tz
                p, n, q, r = p[keep], n[keep], q[keep], r[keep]
            w = np.minimum(1.0, huber / np.maximum(np.abs(r), 1e-9))
            J = np.hstack([np.cross(p - center, n), n])
            Jw = J * w[:, None]
            H += J.T @ Jw; g += Jw.T @ r
            rows.append(r)
        if not rows: return None
        r_all = np.concatenate(rows)
        if len(r_all) < 200: return None
        # weak prior on the correction (keeps unobservable directions at zero / at the plane start)
        prior = np.diag([1 / np.radians(prior_rot_deg) ** 2] * 3 + [1 / prior_trans_m ** 2] * 3)
        x = np.concatenate([rot, trans])
        H += prior; g += prior @ (x - x0)
        Ha = H[np.ix_(active, active)]; ga = g[active]
        step = np.zeros(6); step[active] = -np.linalg.solve(Ha, ga)
        rot = rot + step[:3]; trans = trans + step[3:]
        stats = (it + 1, len(r_all), float(np.median(np.abs(r_all))), float(np.linalg.norm(step[:3])), float(np.linalg.norm(step[3:])))
        if np.linalg.norm(step[:3]) < 1e-6 and np.linalg.norm(step[3:]) < 1e-5: break
    return rot, trans, stats


def residuals(pairs, center, rot, trans, max_dist=0.6):
    Rm = R.from_rotvec(rot).as_matrix(); out = []
    for target, B in pairs:
        Bt = (B - center) @ Rm.T + center + trans
        ok, n, q = target.match(Bt, max_dist)
        r = np.einsum("ni,ni->n", n, Bt[ok] - q); out.append(np.column_stack([r, np.abs(n[:, 2])]))
    a = np.vstack(out); g = a[:, 1] > 0.8; v = a[:, 1] < 0.3
    return dict(n=len(a), med=float(np.median(np.abs(a[:, 0]))), n_ground=int(g.sum()),
                med_ground=float(np.median(np.abs(a[g, 0]))) if g.any() else np.nan,
                n_vertical=int(v.sum()), med_vertical=float(np.median(np.abs(a[v, 0]))) if v.any() else np.nan)


def _score(res):
    """Quantity the stage minimises: the ground residual (falls back to the overall residual)."""
    return res["med_ground"] if np.isfinite(res["med_ground"]) else res["med"]


def refine_one(t, pairs, center, solve_yaw_allowed, args, lines, src_frames, ref_frames):
    """Plane start (when the LiDAR is outside the refinement basin) -> fine solve -> regression-safe choice.
    Returns (rot, trans, record, line) or None when there is no usable overlap at all."""
    init = None if args.no_coarse else plane_init(src_frames, ref_frames, center)
    outside = init is not None and (init["tilt_deg"] > args.coarse_threshold_deg or abs(init["dz_m"]) > args.coarse_threshold_z)
    before = residuals(pairs, center, np.zeros(3), np.zeros(3))
    use_init = False; x0 = np.zeros(6); start = before; init_note = None
    if outside:
        x0 = np.concatenate([init["rot"], init["trans"]])
        start = residuals(pairs, center, x0[:3], x0[3:])
        # the plane start only moves the basin: it is used when it actually brings the LiDAR closer to the reference
        # ground (a LiDAR looking at different terrain than the reference - back LiDAR on an undulating road - can
        # fit a plane that disagrees with the overlap; then the parent matrix stays the start)
        use_init = _score(start) < _score(before)
        init_note = "used" if use_init else "not used (does not improve the overlap)"
        lines.append(f"{t}: plane start: own ground vs reference tilt {init['tilt_deg']:.2f} deg (fore-aft {init['fore_aft_deg']:+.2f}, "
                     f"lateral {init['lateral_deg']:+.2f}), height {100*init['dz_m']:+.1f} cm over {init['pairs']} plane pair(s) -> "
                     f"start rot deg (x,y,z)=({np.degrees(x0[0]):+.3f},{np.degrees(x0[1]):+.3f},{np.degrees(x0[2]):+.3f}) z {100*x0[5]:+.1f} cm; "
                     f"ground residual {100*before['med_ground']:.2f}->{100*start['med_ground']:.2f} cm (n={before['n_ground']}->{start['n_ground']}): {init_note}")
        if not use_init:
            x0 = np.zeros(6); start = before
    solve_yaw = bool(solve_yaw_allowed and start["n_vertical"] >= args.min_vertical)
    out = solve(pairs, center, solve_yaw, x0=x0 if use_init else None)
    reasons, level = [], "ok"
    def warn(msg):
        nonlocal level
        reasons.append(msg); level = "warn"
    if out is None:
        if not use_init:
            return None
        rot, trans, st = x0[:3].copy(), x0[3:].copy(), (0, start["n"], start["med"], 0.0, 0.0)
        after = start; chosen = "plane start"
        warn("fine solve had no matches; plane start kept")
    else:
        rot, trans, st = out
        after = residuals(pairs, center, rot, trans)
        chosen = "fine"
        # never write a result that is worse than what the stage started from
        alternatives = [("manual", np.zeros(3), np.zeros(3), before)]
        if use_init:
            alternatives.append(("plane start", x0[:3], x0[3:], start))
        best = min(alternatives, key=lambda c: _score(c[3]))
        if _score(after) > _score(best[3]) + 1e-6:
            warn(f"fine solve regressed the ground residual ({100*_score(start):.2f}->{100*_score(after):.2f} cm); kept the {best[0]} matrix ({100*_score(best[3]):.2f} cm)")
            chosen, rot, trans, after = best[0], best[1], best[2], best[3]
        elif st[0] >= MAX_ITERS and (st[3] > 1e-4 or st[4] > 1e-3):
            # the strict 1e-6 rad stop is often never reached (re-matching of boundary points keeps a limit cycle of
            # ~1e-6 rad going); only a last step above calibration precision (0.006 deg / 1 mm) is a real non-convergence
            warn(f"not converged in {MAX_ITERS} iterations (last step {np.degrees(st[3]):.4f} deg / {1000*st[4]:.2f} mm)")
    if use_init:
        warn(f"plane start applied ({init['tilt_deg']:.2f} deg / {100*init['dz_m']:+.1f} cm): outside the refinement basin of the parent calibration (mount moved or coarse parent file); yaw/x/y are re-checked by the driven stage")
    rot_deg = float(np.degrees(np.linalg.norm(rot))); trans_m = float(np.linalg.norm(trans))
    if not use_init and (rot_deg > LARGE_ROT_DEG or trans_m > LARGE_TRANS_M):
        warn(f"large correction ({rot_deg:.2f} deg / {100*trans_m:.1f} cm) beyond the refinement scale")
    record = dict(rotvec_deg=np.degrees(rot).tolist(), trans_cm=(100 * trans).tolist(), yaw_xy_solved=bool(solve_yaw),
                  about="lidar position in rear-axle frame", chosen=chosen,
                  quality=dict(level=level, reasons=reasons, iterations=int(st[0]), matches=int(after["n"]),
                               ground_before_cm=round(100 * before["med_ground"], 2) if np.isfinite(before["med_ground"]) else None,
                               ground_after_cm=round(100 * after["med_ground"], 2) if np.isfinite(after["med_ground"]) else None,
                               rotation_deg=round(rot_deg, 3), translation_cm=round(100 * trans_m, 1)))
    if init is not None:
        record["plane_check"] = dict(tilt_deg=round(init["tilt_deg"], 3), fore_aft_deg=round(init["fore_aft_deg"], 3),
                                     lateral_deg=round(init["lateral_deg"], 3), dz_cm=round(100 * init["dz_m"], 1),
                                     pairs=init["pairs"], outside_basin=bool(outside), applied=bool(use_init))
    line = (f"yaw/xy solved={solve_yaw}; matches {before['n']}->{after['n']}; |res| median {before['med']*100:.2f}->{after['med']*100:.2f} cm; "
            f"ground {before['med_ground']*100:.2f}->{after['med_ground']*100:.2f} cm (n={after['n_ground']}); vertical {before['med_vertical']*100:.2f}->{after['med_vertical']*100:.2f} cm (n={after['n_vertical']}); "
            f"correction rot deg (x,y,z)=({np.degrees(rot[0]):+.3f},{np.degrees(rot[1]):+.3f},{np.degrees(rot[2]):+.3f}) trans cm=({100*trans[0]:+.1f},{100*trans[1]:+.1f},{100*trans[2]:+.1f}); iters {st[0]}"
            f"  [quality: {level}{'; ' + '; '.join(reasons) if reasons else ''}]")
    return rot, trans, record, line


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", required=True); ap.add_argument("--bag", required=True)
    ap.add_argument("--output", required=True); ap.add_argument("--report", required=True)
    ap.add_argument("--stationary", type=int, nargs=2, default=(2, 20), help="first frame, count (aggregated)")
    ap.add_argument("--moving", type=int, nargs=2, default=(300, 40), help="first frame, count (per-frame) for the back LiDAR")
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--min-vertical", type=int, default=400, help="vertical planar matches needed to solve yaw/xy")
    ap.add_argument("--max-still-shift", type=float, default=0.30,
                    help="max median scan displacement (m) between first and last stationary frame; a standing truck gives ~0.1 m from the scan pattern alone, 0.5 m/s of motion gives >0.5 m")
    ap.add_argument("--coarse-threshold-deg", type=float, default=0.5,
                    help="own-ground-plane tilt vs the reference above which the solve starts from the closed-form plane alignment (refinement scale of the solve)")
    ap.add_argument("--coarse-threshold-z", type=float, default=0.05,
                    help="own-ground-plane height difference (m) vs the reference above which the plane start is used")
    ap.add_argument("--no-coarse", action="store_true", help="historic behaviour: always start the solve from the parent matrix")
    a = ap.parse_args()
    short_recording = Dataset.open(a.bag).duration_s < SHORT_RECORDING_S
    calib = json.load(open(a.calibration))
    M = {}
    for entry in calib["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: M[topic] = np.array(entry[k], dtype=float).reshape(4, 4)
    to_rear = lambda t, P: P @ M[t][:3, :3].T + M[t][:3, 3]
    lines = [f"calibration={a.calibration}", f"bag={a.bag}", f"reference={REFERENCE}",
             f"stationary frames {a.stationary[0]}..{a.stationary[0]+a.stationary[1]} (aggregated) for side LiDARs; moving frames {a.moving[0]}..{a.moving[0]+a.moving[1]} (per frame) for back LiDAR"]
    stat = read_frames(a.bag, *a.stationary)
    # The aggregation below assumes the vehicle does not move during the stationary frames; a recording
    # that starts while driving silently biases roll/pitch/z (motion-blurred ground), so verify it.
    still = stationary_check(stat[REFERENCE])
    lines.append(f"stationary check ({REFERENCE}, first vs last frame of {a.stationary[1]}): scan displacement median "
                 f"{100*still['median_m']:.1f} cm / P90 {100*still['p90_m']:.1f} cm (limit {100*a.max_still_shift:.0f} cm) -> "
                 + ("stationary" if still["median_m"] <= a.max_still_shift else "MOVING"))
    if still["median_m"] > a.max_still_shift:
        if short_recording:
            msg = (f"warning: frames {a.stationary[0]}..{a.stationary[0]+a.stationary[1]} exceed stationary displacement limit "
                   f"({100*still['median_m']:.0f} cm > {100*a.max_still_shift:.0f} cm); continuing with short-recording fallback")
            lines.append(msg)
        else:
            msg = (f"ground stage aborted: frames {a.stationary[0]}..{a.stationary[0]+a.stationary[1]} are not stationary "
                   f"(reference scan displacement {100*still['median_m']:.0f} cm). The stage needs ~{0.1*a.stationary[1]:.0f} s of standstill; "
                   "use a recording that starts before the vehicle moves off, or point --stationary at a standstill.")
            lines.append(msg); pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
            pathlib.Path(a.report).write_text("\n".join(lines) + "\n"); print("\n".join(lines)); sys.exit(3)
    agg = {t: voxel_down(clean(to_rear(t, np.vstack(v))), a.voxel) for t, v in stat.items()}
    for t in KEYS: lines.append(f"{t}: stationary aggregated points {len(agg[t])}")
    refined = {REFERENCE: M[REFERENCE].copy()}; corrections = {}; quality = {}
    ref_target = Target(agg[REFERENCE])
    for t in ("front_right_lidar", "front_left_lidar"):
        center = M[t][:3, 3]
        pairs = [(ref_target, agg[t])]
        res = refine_one(t, pairs, center, True, a, lines, [agg[t]], [agg[REFERENCE]])
        if res is None: lines.append(f"{t}: insufficient overlap; manual matrix kept"); refined[t] = M[t].copy(); continue
        rot, trans, record, line = res
        C = np.eye(4); C[:3, :3] = R.from_rotvec(rot).as_matrix(); C[:3, 3] = center - C[:3, :3] @ center + trans
        refined[t] = C @ M[t]
        corrections[t] = record; quality[t] = record["quality"]["level"]
        lines.append(f"{t}: {line}")
    # back LiDAR: per-frame matches against the (refined) side LiDARs in the moving segment
    t = "back_lidar"; center = M[t][:3, 3]
    mov = read_frames(a.bag, *a.moving)
    pairs = []; src_frames = []; ref_frames = []
    for f in range(min(len(v) for v in mov.values())):
        B = voxel_down(clean(to_rear(t, mov[t][f])), a.voxel)
        sides = []
        for s in ("front_right_lidar", "front_left_lidar"):
            if s not in refined: continue
            Cs = refined[s] @ np.linalg.inv(M[s])   # correction of side lidar in rear frame
            S = clean(to_rear(s, mov[s][f])); S = S @ Cs[:3, :3].T + Cs[:3, 3]
            S = voxel_down(S, a.voxel)
            if len(S) > 500 and len(B) > 100: pairs.append((Target(S), B)); sides.append(S)
        if sides:
            src_frames.append(B); ref_frames.append(np.vstack(sides))
    if pairs:
        # Intra-scan ego-motion (~0.5 m per scan while driving) corrupts yaw/xy evidence from
        # vertical surfaces; only the ground-observed roll/pitch/height is trusted here.
        res = refine_one(t, pairs, center, False, a, lines, src_frames, ref_frames)
        if res is None:
            lines.append(f"{t}: insufficient overlap; manual matrix kept"); refined[t] = M[t].copy()
        else:
            rot, trans, record, line = res
            C = np.eye(4); C[:3, :3] = R.from_rotvec(rot).as_matrix(); C[:3, 3] = center - C[:3, :3] @ center + trans
            refined[t] = C @ M[t]
            corrections[t] = record; quality[t] = record["quality"]["level"]
            lines.append(f"{t}: (moving, per-frame vs refined sides) {line}")
    else:
        lines.append(f"{t}: no moving-frame overlap; manual matrix kept"); refined[t] = M[t].copy()
    overall = "warn" if "warn" in quality.values() else "ok"
    lines.append(f"quality: {overall}" + ("" if overall == "ok" else "  (" + "; ".join(
        f"{k}: {', '.join(corrections[k]['quality']['reasons'])}" for k, lv in quality.items() if lv != "ok") + ")"))
    out = json.loads(json.dumps(calib))
    for entry in out["lidar"]:
        for topic, k in KEYS.items():
            if k in entry: entry[k] = [float(v) for v in refined[topic].reshape(-1)]
    out["calibration_type"] = calib.get("calibration_type", "") + "+data_refined_roll_pitch_z"
    out["refinement"] = dict(reference=REFERENCE, corrections=corrections, quality=overall, source=str(a.bag),
                             stationary_frames=list(a.stationary), moving_frames=list(a.moving),
                             coarse_threshold=[a.coarse_threshold_deg, a.coarse_threshold_z])
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(a.output, "w"), indent=2)
    pathlib.Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    pathlib.Path(a.report).write_text("\n".join(lines) + "\n")
    print("\n".join(lines)); print("written", a.output)


if __name__ == "__main__":
    main()
