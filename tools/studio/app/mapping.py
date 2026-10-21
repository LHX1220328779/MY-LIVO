"""Mapping rounds (scripts/run_wuhu_round.sh): start / stop, round listing, live monitoring."""
import datetime as dt
import json
import math
import re
import subprocess

from . import calibration, datasets, jobs
from .common import (PY, REPO, RESULTS, ROUND_SCRIPT, ROS_PROCESS_PATTERN, StudioError, binned_series, count_lines,
                     csv_last, csv_rows, parse_kv_log, pgrep, read_json, read_tail, safe_path)

METRICS_SCRIPT = REPO / "scripts" / "analysis" / "round_metrics.py"
MAP_EXPORT_TOOL = REPO / "tools" / "livo" / "map_export.py"


def _map_export_args(body, prefix=""):
    """Validated map-export settings from a form body (keys: map_voxel/map_source/map_tile/map_keyframes or the
    same without the map_ prefix for the results-page re-export)."""
    g = lambda k, d=None: body.get(prefix + k, d)  # noqa: E731
    try:
        voxel = float(g("voxel", 0.05) if g("voxel", 0.05) not in (None, "") else 0.05)
        tile = float(g("tile", 0) or 0)
    except (TypeError, ValueError):
        raise StudioError("地图导出参数必须是数字") from None
    source = g("source", "all") or "all"
    try:
        min_points = int(g("min_points", 1) or 1)
    except (TypeError, ValueError):
        raise StudioError("地图导出：每体素最少点数必须是整数") from None
    if not (0 <= voxel <= 2.0) or not (0 <= tile <= 2000) or source not in ("all", "keyframes") or not 1 <= min_points <= 1000:
        raise StudioError("地图导出：体素 0–2 m（0 = 不滤波）、分块 0–2000 m、来源 all/keyframes、每体素最少点数 1–1000")
    return voxel, source, tile, bool(g("keyframes", False)), min_points


def export_map(round_name, body):
    """Re-export the standard map products of a finished round from its scan archive (no SLAM re-run)."""
    round_name = re.sub(r"[^A-Za-z0-9_\-.]", "", round_name or "")
    p = RESULTS / round_name
    runs = sorted((p / "Log" / "fusion").glob("*/")) if (p / "Log" / "fusion").is_dir() else []
    if not runs or not (runs[-1] / "scans.bin").exists() or not (runs[-1] / "corrections.txt").exists():
        raise StudioError(f"轮次 {round_name} 没有扫描存档（scans.bin / corrections.txt）：需要开启“保存地图”的建图轮次")
    if jobs.active("export"):
        raise StudioError("已有地图导出任务在运行")
    voxel, source, tile, kf, min_points = _map_export_args(body)
    fusion = runs[-1]
    name = "map" if (voxel == 0.05 and source == "all" and not tile and not kf and min_points == 1) else \
        f"map_{voxel * 100:g}cm_{source}{f'_tile{tile:g}' if tile else ''}{'_kf' if kf else ''}" \
        f"{f'_min{min_points}' if min_points > 1 else ''}"
    cmd = [PY, str(MAP_EXPORT_TOOL), str(fusion), "--voxel", f"{voxel:g}", "--source", source, "--tile", f"{tile:g}",
           "--min-points", str(min_points), "--out", str(fusion / name)]
    if kf:
        cmd.append("--keyframe-clouds")
    return jobs.start("export", f"地图导出 · {round_name} · {name}", [{"title": f"导出 {name}", "cmd": cmd}],
                      meta={"round": round_name, "out": str(fusion / name)})


def map_products(fusion):
    """Map product directories (map*/map_info.json) of a fusion run, newest first."""
    out = []
    for info in sorted(fusion.glob("map*/map_info.json"), key=lambda f: f.stat().st_mtime, reverse=True):
        doc = read_json(info, {}) or {}
        files = []
        for fname in ("global_map.pcd", "global_map_rgb.pcd"):
            f = info.parent / fname
            if f.exists():
                files.append({"name": f"{info.parent.name}/{fname}", "mb": round(f.stat().st_size / 1e6, 1),
                              "points": (doc.get("points") or {}).get(fname)})
        out.append({"dir": info.parent.name, "voxel_m": doc.get("voxel_m"), "source": doc.get("source"),
                    "min_points": doc.get("min_points_per_voxel", 1),
                    "tile_m": doc.get("tile_m"), "keyframe_clouds": doc.get("keyframe_clouds"),
                    "created": doc.get("created"), "elapsed_s": doc.get("elapsed_s"), "files": files,
                    "frames_used": (doc.get("frames") or {}).get("frames_used")})
    return out

PLAYER_RE = re.compile(r"\[player\] t=([\d.]+)/([\d.]+) s (\d+)% msgs=(\d+) lag=(-?\d+) ms")
EVENT_KEYS = ("Extrinsic precheck", "[WARN]", "[ERROR]", "[FATAL]", "RTK finalized", "Final keyframe",
              "INS attitude factors", "Loaded", "Time offsets", "Camera model", "IMU Initials",
              "Merged Multi-LiDAR", "process has died", "Exported", "map_", "Loop closure", "Released")
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
    if body.get("rgb_voxel") not in (None, ""):
        try:
            voxel = float(body["rgb_voxel"])
        except (TypeError, ValueError):
            raise StudioError("彩色点云体素必须是 0 到 0.5 m 之间的数（0 表示保留每一行）") from None
        if not math.isfinite(voxel) or not 0 <= voxel <= 0.5:
            raise StudioError("彩色点云体素必须是 0 到 0.5 m 之间的数（0 表示保留每一行）")
        cmd += ["--rgb-voxel", str(voxel)]
    if body.get("skip_stationary"):
        try:
            smin = float(body.get("stationary_min") or 30)
            slidar = float(body.get("stationary_lidar_interval") or 1.0)
        except (TypeError, ValueError):
            raise StudioError("静止跳过参数必须是数字") from None
        if not (smin >= 10 and 0.2 <= slidar <= 5):
            raise StudioError("静止跳过：最短静止时长 >= 10 s，快进期激光间隔 0.2–5 s")
        cmd += ["--stationary-min", f"{smin:g}", "--stationary-lidar-interval", f"{slidar:g}"]
    if body.get("loop_closure", True) is False:
        # finalize-time submap loop closure over revisited sections (on by default)
        cmd += ["--set", "rtk.loop_closure_enabled=false"]
    # saved map: drop far duplicate returns of surfaces another pass saw nearer (on by default)
    cmd.append("--no-map-priority" if body.get("map_priority", True) is False else "--map-priority")
    if body.get("save_map", True) or force_save_map:
        voxel, source, tile, kf, min_points = _map_export_args(body, "map_")
        if body.get("map_export", True) is False:
            cmd.append("--no-map-export")
        else:
            cmd += ["--map-voxel", f"{voxel:g}", "--map-source", source, "--map-tile", f"{tile:g}",
                    "--map-min-points", str(min_points)]
            if kf:
                cmd.append("--map-keyframes")
    if body.get("auto_cleanup", True):
        try:
            free_gb = float(body.get("auto_cleanup_free_gb") or 50)
        except (TypeError, ValueError):
            raise StudioError("磁盘清理阈值必须是数字（GB）") from None
        if free_gb > 0:
            cmd += ["--min-free-gb", f"{free_gb:g}"]
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
    # Calibration rounds feed stages (frames.csv / scans.bin / trajectories) and the closed-loop verdict (trajectory
    # metrics): the standard map products are never used and are deleted by the pipeline cleanup, and the quick
    # evaluation (trajectory + summary + metrics.json) carries every number the pipeline reads (~2 min less per round).
    body["map_export"] = False
    # Playback speed of calibration rounds. The player holds whenever the mapper is more than 1 s behind (lossless
    # flow control), so a higher rate only removes idle time: LIO rounds run at the mapper's own speed (~2.5x real
    # time, 25 fps of 40 ms LIO updates; identical result to 1x, see docs/mcp-my-livo-8-indicators.md); LIVO rounds
    # stay at 1x (photometric update ~40 ms per image already uses the real-time budget).
    try:
        body["rate"] = float(body.get("calib_lio_rate") or 2.5) if mode == "lio" else float(body.get("calib_livo_rate") or 1.0)
    except (TypeError, ValueError):
        raise StudioError("标定轮次回放倍速必须是数字") from None
    body["skip_stationary"] = False  # calibration windows are chosen on the motion profile / by the operator
    cmd = round_cmd(round_name, mode, calibration_file, body,
                    force_save_map=bool(overrides.get("save_map", True)), evaluate=overrides.get("evaluate", True))
    if overrides.get("quick", True) and "--no-eval" not in cmd:
        cmd.append("--quick-eval")
    if not body.get("keep_calib_maps"):
        # the stages read the archives only: map_N / rgb_map_N.pcd (1.7 GB and ~15 s of export per 180 s LIO
        # round) were written and then deleted unread by the pipeline cleanup
        cmd.append("--no-map-chunks")
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
    if not cid:
        # Without a version the mapper silently fell back to config/wuhu_truck29.yaml's calibration_file, a hand-tuned
        # file of the 2026-08 truck29 data: truck29raw 20261013 rounds wuhu_29_livo_4..6 ran with front_left_lidar
        # 6.4/3.7 deg off (startup precheck: out of gates, calibration kept) and the maps showed tilted, crossing blocks.
        raise StudioError(f"车辆 {body.get('mine') or '?'}/{body.get('truck') or '?'} 没有可用的标定版本（未选择且无激活版本）："
                          f"请在建图页选择标定版本。建图不再静默回退到配置文件里的默认外参（该文件属于 2026-08 truck29 手调标定）")
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
                                                  "evaluate", "extra_sets", "start_offset", "duration", "map_range", "rgb_range",
                                                  "rgb_voxel",
                                                  "skip_stationary", "stationary_min", "stationary_lidar_interval",
                                                  "loop_closure", "map_priority",
                                                  "map_export", "map_voxel", "map_source", "map_tile", "map_keyframes")}}
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


def loop_summary(fusion_dir):
    """Finalize-time loop closure of a round from loops.csv (+ loops_rtk_bound.csv); None when it did not run."""
    hdr, rows = csv_rows(fusion_dir / "loops.csv", max_points=100_000)
    col = {n: i for i, n in enumerate(hdr)}
    if not {"accepted", "reason", "correction_m"} <= col.keys():
        return None
    rows = [r for r in rows if len(r) == len(hdr)]
    accepted = [r for r in rows if r[col["accepted"]] == "1"]
    _, dropped = csv_rows(fusion_dir / "loops_rtk_bound.csv", max_points=100_000)
    rejected = {}
    for r in rows:
        if r[col["accepted"]] != "1":
            rejected[r[col["reason"]]] = rejected.get(r[col["reason"]], 0) + 1
    moves = sorted(float(r[col["correction_m"]]) for r in accepted)
    return {"pairs": len(rows), "accepted": len(accepted), "rtk_dropped": len(dropped),
            "kept": len(accepted) - len(dropped), "correction_median_m": moves[len(moves) // 2] if moves else None,
            "rejected": dict(sorted(rejected.items(), key=lambda kv: -kv[1]))}


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
    head = read_tail(path, 4_000) if path.exists() and path.stat().st_size <= 4_000 else ""
    if not head and path.exists():
        with path.open("rb") as f:
            head = f.read(4_000).decode("utf-8", "replace")
    m = re.search(r"\[player\] standstill fast-forward: (\d+) window\(s\), ([\d.]+) s", head)
    if m:
        out["skip_windows"], out["skip_s"] = int(m.group(1)), float(m.group(2))
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
        # Live curves: stable time bins (max per bin for errors), see common.binned_series. Every value is the
        # error of the pose PUBLISHED at that time (causal online estimate); it is written once and never revised.
        # The delivered (smoothed, final-LM) trajectory error is only known after export and is shown from
        # evaluation/error_series.csv once the round has been evaluated.
        series = binned_series(
            f / "diagnostics.csv", "stamp",
            ["frontend_rtk_residual_m", "rtk_residual_m", "frontend_speed_mps", "keyframes", "publish_lag_sec",
             "published_imu_reference_difference_m", "processing_ms"],
            agg={"frontend_speed_mps": "mean", "keyframes": "last", "processing_ms": "mean"},
            rename={"frontend_speed_mps": "speed", "publish_lag_sec": "lag",
                    "published_imu_reference_difference_m": "published_diff_m", "processing_ms": "backend_ms"})
        if series:
            out["diagnostics"] = series
            last = csv_last(f / "diagnostics.csv")
            out["latest"] = {n: v for n, v in last.items() if n in (
                "stamp", "keyframes", "planes", "factors", "rtk_residual_m", "frontend_rtk_residual_m",
                "frontend_speed_mps", "publish_lag_sec", "latest_key_rtk_residual_m", "processing_ms")}
            finite = [s["frontend_rtk_residual_m"] for s in series if s.get("frontend_rtk_residual_m") is not None]
            out["latest"]["frontend_rtk_residual_m"] = finite[-1] if finite else None
        online = binned_series(f / "online_path.csv", "stamp", ["latest_imu_difference_m", "max_imu_difference_m"],
                               agg={"max_imu_difference_m": "last"},
                               rename={"latest_imu_difference_m": "latest", "max_imu_difference_m": "max"})
        if online:
            out["online"] = csv_last(f / "online_path.csv")
            out["online_series"] = online
        maps = sorted(f.glob("map_*.pcd"))
        out["map_chunks"] = len(maps)
        out["map_size_mb"] = round(sum(m.stat().st_size for m in maps) / 1e6, 1)
        out["rgb_map_chunks"] = len(list(f.glob("rgb_map_*.pcd")))
        out["map_products"] = map_products(f)
        out["has_archive"] = (f / "scans.bin").exists()
        out["loops"] = loop_summary(f)
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
    # Throughput / resources (tools/livo/resmon.py): processed frames per second, real-time factor, backend time,
    # RSS/CPU of the mapper, camera decoder and player, host memory and swap.
    res = p / "resources.csv"
    if res.exists():
        out["resources"] = csv_last(res)
        out["resource_series"] = binned_series(
            res, "elapsed_s", ["fps", "rt_factor", "mapper_rss_mb", "decoder_rss_mb", "sys_used_mb", "swap_used_mb",
                               "mapper_cpu_pct"],
            max_points=300, agg={"fps": "mean", "rt_factor": "mean", "mapper_cpu_pct": "mean"})
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
