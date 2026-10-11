#!/usr/bin/env python3
"""Closed-loop camera pitch: fit native-odometry scale vs camera pitch increment and return the zero crossing.

Each LIVO round run with a camera extrinsic rotated by a known pitch increment dp (deg, camera-frame x, as
written by `livo_calib.py set --camera-rot-delta dp 0 0`) yields a native path-length ratio r = native / RTK
(evaluation/metrics.json: headline.native.path_length_ratio). r depends linearly on the pitch error with a
slope of ~0.11 per degree (7-round measurement, docs/mcp-livo-native-drift-analysis.md), so the pitch that
makes r = 1 is dp* = (1 - a) / b for the fitted line r = a + b * dp.

When LiDAR fully constrains the scale (ratio flat at ~1.000 for every increment, e.g. truck28: 0.9996 at
-0.05/0/+0.05 deg) the scale slope is ~0 and pitch is unobservable through it. The fit then falls back to the
native drift itself (headline.native rmse_m, median_m when absent): a convex parabola through the scan points
whose vertex lies inside the scanned range is adopted (clamped); a vertex within KEEP_DEG of 0 means "keep the
current pitch" (ok + keep, no new extrinsic needed). Neither observable -> ok false with keep_recommended.

Usage: camera_pitch_fit.py --point <round>=<dp_deg> [--point ...] [--target 1.0] [--max-abs 0.3] [--exit-zero]
Prints one JSON line: {"ok", "pitch_delta_deg", "slope_per_deg", "intercept", "points", "rms", "reason"}.
"""
import argparse
import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
RESULTS = REPO / "results"
EXPECTED_SLOPE = (0.03, 0.40)  # per degree; measured 0.11
FLAT_SCALE = 0.002             # |intercept - 1| below this with a ~0 slope: scale already locked by LiDAR
KEEP_DEG = 0.005               # drift-minimum vertex closer than this to 0 -> keep the current pitch
MIN_DEPTH_REL, MIN_DEPTH_M = 0.05, 0.02  # parabola must rise this much at the scan ends to count as a minimum


def ratio_of(round_name):
    p = RESULTS / round_name / "evaluation" / "metrics.json"
    if not p.exists():
        return None, f"{round_name}: evaluation/metrics.json missing"
    doc = json.loads(p.read_text())
    r = (doc.get("headline", {}).get("native", {}) or {}).get("path_length_ratio")
    if r is None:
        r = (doc.get("run") or {}).get("path_length_ratio")
    if r is None:
        return None, f"{round_name}: path_length_ratio missing"
    nat = doc.get("headline", {}).get("native", {}) or {}
    drift = nat.get("rmse_m", nat.get("median_m"))
    return (float(r), (doc.get("run") or {}).get("duration_s"), None if drift is None else float(drift)), None


def drift_minimum(out, pts, max_abs):
    """Scale locked by LiDAR: pick the pitch that minimises the native drift (parabola vertex)."""
    scale_reason = out.get("reason")
    q = [(p["pitch_delta_deg"], p["native_drift_m"]) for p in pts if p.get("native_drift_m") is not None]
    xs = sorted({x for x, _ in q})
    out["scale_reason"] = scale_reason
    if len(xs) < 3:
        out["reason"] = f"scale insensitive to pitch ({scale_reason}); drift minimum needs 3 increments"
        out["keep_recommended"] = True
        return
    n = len(q)
    # least-squares parabola y = c0 + c1 x + c2 x^2 (normal equations, 3x3)
    S = [[sum(x ** (i + j) for x, _ in q) for j in range(3)] for i in range(3)]
    T = [sum(y * x ** i for x, y in q) for i in range(3)]
    def det(M):
        return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
    D = det(S)
    c = [det([[T[r] if k == i else S[r][k] for k in range(3)] for r in range(3)]) / D for i in range(3)]
    c0, c1, c2 = c
    lo, hi = xs[0], xs[-1]
    fit = {"drift_metric": "native rmse_m", "drift_poly": [round(v, 6) for v in c],
           "drift_points": [[x, round(y, 4)] for x, y in q]}
    out.update(fit)
    if c2 <= 0:
        out.update(reason=f"scale insensitive ({scale_reason}) and native drift has no minimum (concave)",
                   keep_recommended=True)
        return
    v = -c1 / (2 * c2)
    ymin = c0 + c1 * v + c2 * v * v
    yedge = min(c0 + c1 * lo + c2 * lo * lo, c0 + c1 * hi + c2 * hi * hi)
    depth = yedge - ymin
    out["drift_vertex_deg"], out["drift_min_m"], out["drift_depth_m"] = round(v, 4), round(ymin, 4), round(depth, 4)
    if not (lo <= v <= hi):
        out.update(reason=f"native drift minimum at {v:+.3f} deg is outside the scanned range [{lo:+.2f}, {hi:+.2f}]",
                   keep_recommended=True)
        return
    if depth < max(MIN_DEPTH_M, MIN_DEPTH_REL * max(ymin, 1e-6)):
        out.update(reason=f"native drift too flat vs pitch (depth {100 * depth:.1f} cm): pitch unobservable",
                   keep_recommended=True)
        return
    dp = max(-max_abs, min(max_abs, v))
    keep = abs(dp) < KEEP_DEG
    out.update({"ok": True, "keep": keep, "method": "native drift minimum", "pitch_delta_deg": 0.0 if keep else round(dp, 4),
                "clamped": dp != v, "predicted_ratio": round(out["intercept"], 5),
                "reason": (f"scale locked by LiDAR (slope {out['slope_per_deg']:+.4f}/deg); native drift minimum at "
                           f"{v:+.4f} deg (rmse {ymin:.3f} m, +{100 * depth:.0f} cm at the scan ends)"
                           + ("; current pitch kept" if keep else ""))})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--point", action="append", required=True, metavar="ROUND=DP_DEG")
    ap.add_argument("--target", type=float, default=1.0, help="path-length ratio to aim for (1.0 = RTK)")
    ap.add_argument("--slope", type=float, help="use this slope (per deg, from a previous standard run) instead of "
                                                "fitting one; only the intercept is measured, so one round is enough")
    ap.add_argument("--max-abs", type=float, default=0.3, help="clamp |pitch delta| to this many degrees")
    ap.add_argument("--exit-zero", action="store_true", help="exit 0 even when the fit is rejected")
    a = ap.parse_args()
    pts, errors = [], []
    for item in a.point:
        if "=" not in item:
            errors.append(f"bad --point {item}")
            continue
        name, dp = item.rsplit("=", 1)
        r, err = ratio_of(name.strip())
        if err:
            errors.append(err)
            continue
        pts.append({"round": name.strip(), "pitch_delta_deg": float(dp), "path_length_ratio": r[0], "duration_s": r[1],
                    "native_drift_m": r[2]})
    out = {"ok": False, "points": pts, "target": a.target, "errors": errors}
    xs = [p["pitch_delta_deg"] for p in pts]
    ys = [p["path_length_ratio"] for p in pts]
    durs = [p["duration_s"] for p in pts if p.get("duration_s")]
    if durs and min(durs) < 0.9 * max(durs):
        out["reason"] = (f"round durations differ ({min(durs):.0f}-{max(durs):.0f} s): an incomplete round (mapper aborted on an "
                         "IMU gap?) cannot be compared; re-run it")
    elif a.slope:
        # one measured point is enough: the slope is a property of the camera/LiDAR/VIO configuration
        # (0.104-0.106 /deg measured on this vehicle), so the intercept alone fixes the zero crossing.
        b, n = a.slope, len(pts)
        a0 = sum(y - b * x for x, y in zip(xs, ys)) / n
        rms = (sum((a0 + b * x - y) ** 2 for x, y in zip(xs, ys)) / n) ** 0.5
        out.update({"slope_per_deg": round(b, 5), "slope_source": "model/previous run", "intercept": round(a0, 6),
                    "rms": round(rms, 6)})
        if not (EXPECTED_SLOPE[0] <= b <= EXPECTED_SLOPE[1]):
            out["reason"] = f"slope {b:.4f}/deg outside the expected {EXPECTED_SLOPE}"
        elif abs(a0 - 1.0) > 0.02:
            out["reason"] = (f"current native scale {a0:.4f} is off by more than 2%: the linear model around this working "
                             "point does not hold; the pipeline continues with the standard scan (or run it manually)")
        else:
            dp_star = (a.target - a0) / b
            clamped = abs(dp_star) > a.max_abs
            out["ok"] = True
            out.update({"pitch_delta_deg": round(max(-a.max_abs, min(a.max_abs, dp_star)) if clamped else dp_star, 4),
                        "clamped": clamped, "predicted_ratio": round(a0 + b * dp_star, 5),
                        "reason": "zero crossing with the known slope" + (" (clamped)" if clamped else "")})
    elif len(pts) < 2 or max(xs) - min(xs) < 1e-6:
        out["reason"] = "need at least two rounds with different pitch increments (or --slope)"
    else:
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        a0 = my - b * mx
        rms = (sum((a0 + b * x - y) ** 2 for x, y in zip(xs, ys)) / n) ** 0.5
        out.update({"slope_per_deg": round(b, 5), "intercept": round(a0, 6), "rms": round(rms, 6)})
        if not (EXPECTED_SLOPE[0] <= b <= EXPECTED_SLOPE[1]):
            out["reason"] = (f"slope {b:.4f}/deg outside the expected {EXPECTED_SLOPE} (rounds not comparable or "
                             "camera not contributing); pitch left unchanged")
        elif rms > 0.004:
            out["reason"] = (f"fit residual {rms:.4f} too large (truck29 repeatability is ~0.0003): the native LIVO scale is "
                             "not repeatable in this window (dynamic objects in view, image-degenerate turns, wrong camera "
                             "intrinsics?), so it cannot resolve the camera pitch - check the camera model, pick another "
                             "window (start_offset) or keep the pitch")
        else:
            dp_star = (a.target - a0) / b
            clamped = abs(dp_star) > a.max_abs
            if clamped:
                dp_star = max(-a.max_abs, min(a.max_abs, dp_star))
            out.update({"ok": True, "pitch_delta_deg": round(dp_star, 4), "clamped": clamped,
                        "predicted_ratio": round(a0 + b * dp_star, 5),
                        "reason": "zero crossing of native scale vs camera pitch" + (" (clamped)" if clamped else "")})
    if not out["ok"] and "slope_per_deg" in out and abs(out["slope_per_deg"]) < EXPECTED_SLOPE[0] \
            and abs(out["intercept"] - 1.0) <= FLAT_SCALE:
        drift_minimum(out, pts, a.max_abs)
    print(json.dumps(out, ensure_ascii=False))
    sys.exit(0 if out["ok"] or a.exit_zero else 1)


if __name__ == "__main__":
    main()
