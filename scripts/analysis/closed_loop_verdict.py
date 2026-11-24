#!/usr/bin/env python3
"""Closed-loop A/B of two mapping rounds on the native (front-end) trajectory.

Usage (container):
  /usr/bin/python3 scripts/analysis/closed_loop_verdict.py <baseline_round> <candidate_round> [--version-dir DIR]

Both rounds must have been evaluated (evaluation/metrics.json; generated on demand from the
evaluation artefacts when missing). The candidate calibration is accepted only if, over the time span
covered by BOTH rounds, it does not make the front end worse:
  * native odometry scale |path_length_ratio - 1| not worse than baseline + 0.2 %
    (LIVO: ~11 % scale error per degree of camera pitch extrinsic error, the most sensitive indicator)
  * native rear-axle error in the last common 10 s window <= 1.10 x baseline + 0.10 m
  * native yaw P95 vs INS <= baseline + 0.15 deg
Writes the verdict JSON to stdout and, with --version-dir, to <DIR>/closed_loop.json so the console can
show it next to the calibration version (advisory versions are never activated automatically).
Exit code 0 = pass, 3 = fail (0 with --exit-zero, for pipelines), 2 = usage / missing data.
"""
import json, math, pathlib, subprocess, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
from livo.camera import select_camera, selected_camera, camera_transform_key

REPO = pathlib.Path("/home/project/MY-LIVO")
PY = "/usr/bin/python3"


def metrics(round_name):
    rd = REPO / "results" / round_name
    mj = rd / "evaluation" / "metrics.json"
    if not mj.is_file():
        if not (rd / "evaluation" / "trajectory.log").is_file():
            subprocess.run(["bash", str(REPO / "scripts/evaluate_wuhu_round.sh"), round_name, "--quick"], check=False)
        subprocess.run([PY, str(REPO / "scripts/analysis/round_metrics.py"), str(rd)], check=False)
    if not mj.is_file():
        sys.exit(f"no metrics for round {round_name}")
    return json.loads(mj.read_text())


def pick(doc):
    h = doc.get("headline") or {}
    n = h.get("native") or {}
    s = doc.get("summary") or {}
    win = s.get("windows") or []
    return {"round": doc.get("round"), "mode": doc.get("mode"), "calibration_id": doc.get("calibration_id"),
            "duration_s": (doc.get("run") or {}).get("duration_s"), "scale": n.get("path_length_ratio"),
            "median_m": n.get("median_m"), "final_m": n.get("final_m"), "final_xy_m": n.get("final_xy_m"),
            "yaw_p95_deg": n.get("yaw_p95_deg"), "windows": win}


def main():
    args, vdir, margin, it = [], None, 0.002, iter(sys.argv[1:])
    for a in it:
        if a == "--version-dir":
            vdir = pathlib.Path(next(it, ""))
        elif a == "--scale-margin":
            margin = float(next(it, 0.002))
        elif a.startswith("--"):
            continue
        else:
            args.append(a)
    if len(args) != 2:
        sys.exit(__doc__)
    b, c = pick(metrics(args[0])), pick(metrics(args[1]))
    n = min(len(b["windows"]), len(c["windows"]))
    reasons, ok = [], True
    manifests = [json.loads((REPO / "results" / name / "round.json").read_text()) for name in args]
    cameras = [m.get("camera", "midrange") for m in manifests]
    if any(m.get("status") != "done" for m in manifests):
        ok = False
        reasons.append("both mapping rounds must be complete")
    if cameras[1] == "fisheye":
        if any(m.get("mode") != "livo" for m in manifests):
            ok = False
            reasons.append("fisheye calibration requires actual LIVO rounds")
        if any(not isinstance(d.get(k), (float, int)) or not math.isfinite(d[k])
               for d in (b, c) for k in ("scale", "yaw_p95_deg", "duration_s", "final_m")):
            ok = False
            reasons.append("missing or non-finite native trajectory metrics")
    if vdir:
        target_path = vdir / "calibration.json"
        candidate_path = pathlib.Path(manifests[1].get("calibration") or "")
        if not target_path.is_file() or not candidate_path.is_file():
            ok = False
            reasons.append("missing target calibration or candidate calibration snapshot")
        else:
            target = select_camera(json.loads(target_path.read_text()))
            candidate = select_camera(json.loads(candidate_path.read_text()))
            if (selected_camera(target) != cameras[1]
                    or target["camera"] != candidate["camera"]
                    or target["transforms"][camera_transform_key(target)]["transform_matrix"]
                       != candidate["transforms"][camera_transform_key(candidate)]["transform_matrix"]
                    or target["time_offsets"]["camera"] != candidate["time_offsets"]["camera"]):
                ok = False
                reasons.append("target camera calibration differs from candidate round snapshot")
    if cameras[0] != cameras[1]:
        ok = False
        reasons.append(f"different cameras: {cameras[0]} vs {cameras[1]}")
    if manifests[0].get("dataset") != manifests[1].get("dataset"):
        ok = False
        reasons.append("different source recordings")
    starts = [float((m.get("options") or {}).get("start_offset") or 0) for m in manifests]
    if abs(starts[0] - starts[1]) > 0.01:
        ok = False
        reasons.append("different recording windows")
    common = None
    if n:
        wb, wc = b["windows"][n - 1], c["windows"][n - 1]
        common = {"t0": wb["t0"], "t1": wb["t1"], "baseline_rear_m": wb["native"]["rear_m"], "candidate_rear_m": wc["native"]["rear_m"]}
        if wc["native"]["rear_m"] > 1.10 * wb["native"]["rear_m"] + 0.10:
            ok = False; reasons.append(f"native rear-axle error at {wb['t0']}-{wb['t1']} s {wb['native']['rear_m']:.2f} -> {wc['native']['rear_m']:.2f} m")
    if b["scale"] and c["scale"]:
        if abs(c["scale"] - 1) > abs(b["scale"] - 1) + margin:
            ok = False; reasons.append(f"native odometry scale {b['scale']:.4f} -> {c['scale']:.4f}")
    if b["yaw_p95_deg"] is not None and c["yaw_p95_deg"] is not None and c["yaw_p95_deg"] > b["yaw_p95_deg"] + 0.15:
        ok = False; reasons.append(f"native yaw P95 vs INS {b['yaw_p95_deg']:.2f} -> {c['yaw_p95_deg']:.2f} deg")
    if not n and not (b["scale"] and c["scale"]):
        ok = False; reasons.append("insufficient evaluation data (no window table / path lengths)")
    if b.get("duration_s") and c.get("duration_s") and min(b["duration_s"], c["duration_s"]) < 0.9 * max(b["duration_s"], c["duration_s"]):
        ok = False; reasons.append(f"round durations differ ({b['duration_s']:.0f} vs {c['duration_s']:.0f} s): one run is incomplete")
    for d in (b, c):
        d.pop("windows", None)
    out = {"camera": cameras[1], "verdict": "pass" if ok else "fail", "reasons": reasons, "common_window": common,
           "baseline": b, "candidate": c,
           "scale_margin": margin,
           "criteria": f"scale within baseline+{margin*100:.1f}%; rear error at last common window <= 1.10x+0.10 m; "
                       "yaw P95 <= baseline+0.15 deg; round durations within 10%"}
    text = json.dumps(out, ensure_ascii=False, indent=1)
    print(text)
    if vdir:
        vdir.mkdir(parents=True, exist_ok=True)
        (vdir / "closed_loop.json").write_text(text)
    sys.exit(0 if ok or "--exit-zero" in sys.argv else 3)


if __name__ == "__main__":
    main()
