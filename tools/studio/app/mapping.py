"""Mapping rounds (scripts/run_wuhu_round.sh): start / stop, round listing, live monitoring."""
import datetime as dt
import json
import math
import re
import subprocess

from . import calibration, datasets, jobs
from .common import (PY, REPO, RESULTS, ROUND_SCRIPT, ROS_PROCESS_PATTERN, StudioError, count_lines, csv_rows,
                     parse_kv_log, pgrep, read_json, read_tail, safe_path)

METRICS_SCRIPT = REPO / "scripts" / "analysis" / "round_metrics.py"

PLAYER_RE = re.compile(r"\[player\] t=([\d.]+)/([\d.]+) s (\d+)% msgs=(\d+) lag=(-?\d+) ms")
EVENT_KEYS = ("Extrinsic precheck", "[WARN]", "[ERROR]", "[FATAL]", "RTK finalized", "Final keyframe",
              "INS attitude factors", "Loaded", "Time offsets", "Camera model", "IMU Initials",
              "Merged Multi-LiDAR", "process has died", "Exported", "map_")
_EVENT_CACHE = {}


def round_cmd(round_name, mode, calibration_file, body, *, force_save_map=False, evaluate=None):
    dataset = str(safe_path(body.get("dataset") or ""))
    cmd = ["bash", str(ROUND_SCRIPT), round_name, "--mode", mode, "--dataset", dataset]
    if not body.get("use_rtk", True):
        cmd.append("--no-rtk")
    if not (force_save_map or body.get("save_map", True)):
        cmd.append("--no-save-map")
    if body.get("rviz") and not force_save_map:
        cmd.append("--rviz")
    if body.get("rate"):
        cmd += ["--rate", str(float(body["rate"]))]
    if body.get("start_offset") not in (None, "", 0, "0"):
        cmd += ["--start-offset", str(float(body["start_offset"]))]
    if body.get("duration") not in (None, "", 0, "0"):
        cmd += ["--duration", str(float(body["duration"]))]
    if calibration_file:
        cmd += ["--calibration", str(calibration_file)]
    for key, flag in (("map_range", "--map-range"), ("rgb_range", "--rgb-range")):
        if body.get(key) not in (None, ""):
            try:
                distance = float(body[key])
            except (TypeError, ValueError):
                raise StudioError("点云截取距离必须是非负数（0 表示不限距离）") from None
            if not math.isfinite(distance) or distance < 0:
                raise StudioError("点云截取距离必须是有限非负数（0 表示不限距离）")
            cmd += [flag, str(distance)]
    cmd += ["--set", f"multi_lidar.precheck.enabled={'true' if body.get('precheck', True) else 'false'}"]
    if int(body.get("frame_divider") or 1) > 1:
        cmd += ["--set", f"camera.frame_divider={int(body['frame_divider'])}"]
    for item in str(body.get("extra_sets") or "").split():
        if "=" in item:
            cmd += ["--set", item]
    if evaluate is None:
        evaluate = body.get("evaluate", True)
    if not evaluate:
        cmd.append("--no-eval")
    return cmd


def pipeline_round_cmd(round_name, mode, calibration_file, body, **overrides):
    """Rounds inside a calibration pipeline: archive scans (the camera-lidar stage and the overlay viewer read
    scans.bin) and evaluate (evaluation/metrics.json feeds the closed-loop verdict and the results table).

    `overrides` may carry per-step options: save_map=False (temporary scan rounds: saves ~1.5 GB and the map
    export), quick=True (--quick-eval: trajectory + summary only, ~40 s faster; the scale used by the pitch
    fit and the closed-loop verdict is in both cases the same), start_offset (s, motion-profile window of the
    pipeline; None = from the start of the recording)."""
    body = {**body, **{k: v for k, v in overrides.items() if k in ("save_map", "evaluate", "duration", "start_offset")}}
    cmd = round_cmd(round_name, mode, calibration_file, body,
                    force_save_map=bool(overrides.get("save_map", True)), evaluate=overrides.get("evaluate", True))
    if overrides.get("quick"):
        cmd.append("--quick-eval")
    return cmd


def busy():
    active = jobs.active("mapping") or jobs.active("pipeline")
    if active:
        return active
    if pgrep("fastlivo_mapping"):
        return "external"
    return None


def start(body):
    if busy():
        raise StudioError("已有建图轮次在运行；请先等待其结束或停止它")
    mode = body.get("mode", "livo")
    if mode not in ("lio", "livo"):
        raise StudioError("模式必须是 lio 或 livo")
    dataset = str(safe_path(body.get("dataset") or ""))
    datasets.require(dataset, camera=(mode == "livo"))
    # 轮次命名规范：<矿区>_<车号>_<模式>_<序号>，序号从 1 起；名称不含日期
    # （开始时间已记录在轮次元数据中）。开发测试轮由脚本手工命名并加 _test 后缀。
    mine_part = re.sub(r"[^A-Za-z0-9]", "", str(body.get("mine") or "")).lower() or "site"
    truck_raw = str(body.get("truck") or "")
    truck_part = re.sub(r"[^0-9]", "", truck_raw) or re.sub(r"[^A-Za-z0-9]", "", truck_raw) or "truck"
    base = f"{mine_part}_{truck_part}_{mode}"
    number = 1
    for existing in RESULTS.glob(f"{base}_*"):
        match = re.match(re.escape(base) + r"_(\d+)", existing.name)
        if match:
            number = max(number, int(match.group(1)) + 1)
    round_name = f"{base}_{number}"
    cid = body.get("calibration_id") or ""
    if cid == "active" or not cid:
        cid = calibration.active_id(body.get("mine") or "", body.get("truck") or "") or ""
    # A stale UI draft can carry another vehicle's calibration id (the dropdown then shows
    # "active version" because the id is missing from the current vehicle's list, but the
    # submit still sends the stale id). Mapping with another truck's extrinsics/time offsets
    # yields metre-level native drift from the first frame, so refuse instead of running it
    # (observed: wuhu bag + zongyang/truck07 calibration -> native median 1.83 m, pitch 2.9 deg).
    mine, truck = body.get("mine") or "", body.get("truck") or ""
    if cid and mine and truck and not cid.startswith(f"{mine}/{truck}/"):
        raise StudioError(
            f"标定 {cid} 不属于当前车辆 {mine}/{truck}：请在建图页重新选择标定版本"
            f"（该选项可能是切换车辆前残留的表单草稿）")
    # The recording path names its mine (…/data/haibo/<mine…>/…). Mapping a recording with
    # another mine's vehicle calibration is always a mistake: the extrinsics / lever arms and
    # the per-truck time offsets belong to that other machine (observed 2026-10-09: six wuhu
    # rounds ran with zongyang/truck07 -> the red native path sat metres away from RTK from
    # the first metres, startup included). The cid<->vehicle check above cannot catch this
    # case because the whole vehicle selector was on the other truck; refuse on the dataset.
    if cid:
        from .common import CAL_ROOT
        known_mines = sorted({d.name for d in CAL_ROOT.iterdir()
                              if d.is_dir() and not d.name.startswith(".")})
        ds_l = dataset.lower()
        mines_in_path = [m for m in known_mines if m.lower() in ds_l]
        if mines_in_path and not any(cid.startswith(m + "/") for m in mines_in_path):
            raise StudioError(
                f"数据 {dataset} 按路径属于矿区 {'、'.join(mines_in_path)}，而标定 {cid} "
                f"来自其他矿区的车辆：请先切换到该矿区的车辆（或为当前数据选择正确的标定版本）。"
                f"跨车标定会让原生轨迹从起步就偏离 RTK 米级")
    calib_file = str(calibration.calibration_file(cid)) if cid else None
    cmd = round_cmd(round_name, mode, calib_file, body)
    meta = {"round": round_name, "mode": mode, "dataset": dataset, "calibration_id": cid,
            "options": {k: body.get(k) for k in ("use_rtk", "save_map", "rviz", "rate", "precheck", "frame_divider",
                                                  "evaluate", "extra_sets", "start_offset", "duration", "map_range", "rgb_range")}}
    title = f"{'LIVO' if mode == 'livo' else 'LIO'} 建图 · {round_name}"
    return jobs.start("mapping", title, [{"title": title, "cmd": cmd}], meta=meta)


def stop(mode="finish"):
    job = jobs.active("mapping") or jobs.active("pipeline")
    if job is None:
        if mode == "abort":
            kill_all()
            return {"ok": True, "note": "没有 Studio 任务在运行；已清理残留 ROS 进程"}
        raise StudioError("当前没有正在运行的建图任务")
    job.stop(mode)
    return job.to_json()


def kill_all():
    subprocess.run(["pkill", "-TERM", "-f", ROS_PROCESS_PATTERN], check=False)
    return {"ok": True}


# ------------------------------------------------------------------ results
def parse_summary_metrics(lines):
    out = {}
    for l in lines:
        m = re.match(r"\[(native|optimized)\] rear-axle 3D vs RTK: median ([\d.]+) P95 ([\d.]+) max ([\d.]+) \| route <=10cm ([\d.]+)% >20cm ([\d.]+)%\s+-> budget (\w+)", l)
        if m:
            out[m.group(1)] = {"median_m": float(m.group(2)), "p95_m": float(m.group(3)), "max_m": float(m.group(4)),
                               "route_le10cm_pct": float(m.group(5)), "route_gt20cm_pct": float(m.group(6)),
                               "budget": m.group(7)}
    return out


def _summary_head(path, n=14):
    return read_tail(path, 40_000).splitlines()[:n] if path.exists() else []


def metrics_doc(p, budget=None):
    """evaluation/metrics.json (scripts/analysis/round_metrics.py). Rounds evaluated before the collector
    existed get it generated lazily (at most `budget` rounds per listing so the API stays responsive)."""
    ev = p / "evaluation"
    mj, tl, st = ev / "metrics.json", ev / "trajectory.log", ev / "summary.txt"
    if not tl.exists():
        return None
    newest = max(tl.stat().st_mtime, st.stat().st_mtime if st.exists() else 0.0)
    if mj.exists() and mj.stat().st_mtime >= newest:
        return read_json(mj, None)
    if budget is not None and budget[0] <= 0:
        return read_json(mj, None) if mj.exists() else None
    if budget is not None:
        budget[0] -= 1
    try:
        subprocess.run([PY, str(METRICS_SCRIPT), str(p)], capture_output=True, timeout=60, check=False)
    except (OSError, subprocess.SubprocessError):
        pass
    return read_json(mj, None)


def evaluation_state(p, budget=None):
    """(evaluated, complete, headline metrics, full doc). Falls back to parsing summary.txt."""
    doc = metrics_doc(p, budget)
    if doc:
        return bool(doc.get("evaluated")), bool(doc.get("complete")), doc.get("headline") or {}, doc
    summ = _summary_head(p / "evaluation" / "summary.txt")
    metrics = parse_summary_metrics(summ)
    evaluated = bool(summ) or (p / "evaluation" / "trajectory.log").exists()
    return evaluated, bool(metrics), metrics, None


def round_record(p, budget=None):
    manifest = read_json(p / "round.json", {}) or {}
    kv = parse_kv_log(p / "run.log")
    fusion = sorted((p / "Log" / "fusion").glob("*/")) if (p / "Log" / "fusion").exists() else []
    evaluated, complete, metrics, _doc = evaluation_state(p, budget)
    status = manifest.get("status") or ("done" if kv.get("end") else ("running" if kv.get("start") else "unknown"))
    if status == "running" and not pgrep("run_wuhu_round.sh " + re.escape(p.name)):
        status = "lost"
    return {"round": p.name, "path": str(p), "mode": manifest.get("mode") or kv.get("mode", ""),
            "start": manifest.get("start") or kv.get("start", ""), "end": manifest.get("end") or kv.get("end", ""),
            "status": status, "phase": manifest.get("phase"), "error": manifest.get("error"),
            "overrides": manifest.get("overrides") or kv.get("overrides", ""),
            "dataset": manifest.get("dataset") or kv.get("dataset") or kv.get("bag", ""),
            "calibration": manifest.get("calibration") or kv.get("calibration", ""),
            "options": manifest.get("options"), "commit": manifest.get("commit") or kv.get("commit", ""),
            "fusion_dir": str(fusion[-1]).rstrip("/") if fusion else "",
            "map_chunks": manifest.get("map_chunks"), "evaluated": evaluated, "metrics": metrics,
            "metrics_complete": complete, "evaluation_error": bool(evaluated and not complete),
            "precheck": (p / "Log" / "extrinsic_precheck.json").exists(),
            "mtime": dt.datetime.fromtimestamp(p.stat().st_mtime).isoformat(timespec="seconds")}


def list_runs(limit=100):
    if not RESULTS.exists():
        return []
    dirs = [p for p in RESULTS.iterdir() if p.is_dir() and not p.name.startswith(".") and
            ((p / "run.log").exists() or (p / "round.json").exists())]
    dirs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    budget = [3]   # lazily back-fill metrics.json for at most 3 legacy rounds per listing
    return [round_record(p, budget) for p in dirs[:limit]]


def launch_events(path, limit=60):
    if not path.exists():
        return []
    size = path.stat().st_size
    cache = _EVENT_CACHE.get(path)
    if cache and cache["size"] == size:
        return cache["events"]
    events, pos = (list(cache["events"]), cache["size"]) if cache and cache["size"] <= size else ([], 0)
    with path.open("rb") as f:
        f.seek(pos)
        for raw in f:
            line = raw.decode("utf-8", "replace").rstrip()
            if any(k in line for k in EVENT_KEYS) and "delta time" not in line:
                events.append(re.sub(r"^(\[[^\]]*\]:?\s*)+", "", line)[:260])
    events = events[-limit:]
    _EVENT_CACHE[path] = {"size": size, "events": events}
    return events


def player_progress(path):
    text = read_tail(path, 8_000)
    out = {}
    for line in text.splitlines():
        m = PLAYER_RE.search(line)
        if m:
            out = {"t": float(m.group(1)), "total": float(m.group(2)), "percent": int(m.group(3)),
                   "msgs": int(m.group(4)), "lag_ms": int(m.group(5))}
        elif "[player] completed" in line:
            out["state"] = "completed"
        elif "[player] interrupted" in line:
            out["state"] = "interrupted"
        elif "[player] summary" in line:
            try:
                out["summary"] = json.loads(line.split("[player] summary", 1)[1].strip())
            except json.JSONDecodeError:
                pass
    return out


def live(round_name):
    p = RESULTS / round_name
    out = {"round": round_name, "exists": p.exists()}
    if not p.exists():
        return out
    out["record"] = round_record(p)
    out["player"] = player_progress(p / "playback.log")
    out["mapper_alive"] = pgrep("fastlivo_mapping")
    out["player_alive"] = pgrep("livo/player.py|livo.player")
    out["launch_events"] = launch_events(p / "launch.log")
    fusion = sorted((p / "Log" / "fusion").glob("*/")) if (p / "Log" / "fusion").exists() else []
    if fusion:
        f = fusion[-1]
        out["fusion_dir"] = str(f).rstrip("/")
        out["frames"] = max(0, count_lines(f / "frames.csv") - 4)
        hdr, rows = csv_rows(f / "diagnostics.csv")
        if rows:
            col = {n: i for i, n in enumerate(hdr)}
            series = []
            for r in rows:
                try:
                    series.append({"t": float(r[col["stamp"]]), "frontend_rtk_residual_m": float(r[col["frontend_rtk_residual_m"]]),
                                   "speed": float(r[col["frontend_speed_mps"]]), "keyframes": int(float(r[col["keyframes"]])),
                                   "lag": float(r[col["publish_lag_sec"]]) if "publish_lag_sec" in col else 0.0,
                                   "rtk_residual_m": float(r[col["rtk_residual_m"]]),
                                   "published_diff_m": float(r[col["published_imu_reference_difference_m"]]) if "published_imu_reference_difference_m" in col else None})
                except (ValueError, KeyError, IndexError):
                    continue
            out["diagnostics"] = series
            last = rows[-1]
            out["latest"] = {n: last[i] for n, i in col.items() if n in (
                "stamp", "keyframes", "planes", "factors", "rtk_residual_m", "frontend_rtk_residual_m",
                "frontend_speed_mps", "publish_lag_sec", "latest_key_rtk_residual_m", "processing_ms")}
            finite = [s["frontend_rtk_residual_m"] for s in series if s["frontend_rtk_residual_m"] == s["frontend_rtk_residual_m"]]
            out["latest"]["frontend_rtk_residual_m"] = finite[-1] if finite else None
        hdr, rows = csv_rows(f / "online_path.csv")
        if rows:
            col = {n: i for i, n in enumerate(hdr)}
            last = rows[-1]
            out["online"] = {n: last[i] for n, i in col.items()}
            try:
                out["online_series"] = [{"t": float(r[col["stamp"]]), "latest": float(r[col["latest_imu_difference_m"]]),
                                         "max": float(r[col["max_imu_difference_m"]])} for r in rows]
            except (KeyError, ValueError):
                out["online_series"] = []
        maps = sorted(f.glob("map_*.pcd"))
        out["map_chunks"] = len(maps)
    # per-frame rear-axle error of the optimized (delivered) and native trajectories vs RTK, written by
    # scripts/analysis/round_summary.py at evaluation time (already downsampled to <= 600 rows)
    hdr, rows = csv_rows(p / "evaluation" / "error_series.csv")
    if rows:
        col = {n: i for i, n in enumerate(hdr)}
        series = []
        for r in rows:
            try:
                series.append({"t": float(r[col["stamp"]]), "opt_rear": float(r[col["opt_rear_m"]]),
                               "native_rear": float(r[col["native_rear_m"]])})
            except (ValueError, KeyError, IndexError):
                continue
        out["error_series"] = series
        out["rgb_map_chunks"] = len(list(f.glob("rgb_map_*.pcd")))
        out["map_size_mb"] = round(sum(m.stat().st_size for m in maps) / 1e6, 1)
    pre = p / "Log" / "extrinsic_precheck.json"
    if pre.exists():
        out["precheck"] = read_json(pre, {"status": "unreadable"})
    summ = p / "evaluation" / "summary.txt"
    if summ.exists():
        lines = read_tail(summ, 60_000).splitlines()
        out["summary"] = lines[:160]
        out["metrics"] = parse_summary_metrics(lines)
        traj = p / "evaluation" / "trajectory.log"
        if traj.exists():
            out["thickness"] = [l for l in read_tail(traj, 60_000).splitlines() if "ground-like: cells" in l][:3]
    evaluated, complete, metrics, doc = evaluation_state(p, [1])
    if doc:
        out["metrics"] = metrics
        out["metrics_full"] = doc
    out["evaluated"] = evaluated
    out["metrics_complete"] = complete
    return out


def run_file(round_name, name):
    return safe_path(RESULTS / round_name / name)
