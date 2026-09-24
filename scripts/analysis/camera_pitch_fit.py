#!/usr/bin/env python3
"""Closed-loop camera pitch: fit native-odometry scale vs camera pitch increment and return the zero crossing.

Each LIVO round run with a camera extrinsic rotated by a known pitch increment dp (deg, camera-frame x, as
written by `livo_calib.py set --camera-rot-delta dp 0 0`) yields a native path-length ratio r = native / RTK
(evaluation/metrics.json: headline.native.path_length_ratio). r depends linearly on the pitch error with a
slope of ~0.11 per degree (7-round measurement, docs/mcp-livo-native-drift-analysis.md), so the pitch that
makes r = 1 is dp* = (1 - a) / b for the fitted line r = a + b * dp.

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
    return (float(r), (doc.get("run") or {}).get("duration_s")), None


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
        pts.append({"round": name.strip(), "pitch_delta_deg": float(dp), "path_length_ratio": r[0], "duration_s": r[1]})
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
    print(json.dumps(out, ensure_ascii=False))
    sys.exit(0 if out["ok"] or a.exit_zero else 1)


if __name__ == "__main__":
    main()
