#!/usr/bin/env python3
"""Disk housekeeping for results/: list round sizes, slim or delete old rounds, keep the free space above a target.

Each round directory results/<round>/ holds small evidence (round.json, run.log, trajectories, diagnostics,
evaluation/metrics.json, map/map_info.json) and large regenerable data:

  heavy   Log/fusion/<run>/scans.bin, sources.bin, colors.bin   (scan archive: 2-8 GB per full round)
          Log/fusion/<run>/map_N.pcd, rgb_map_N.pcd             (time-ordered full-resolution chunks: 1-4 GB)
          Log/pcd/, roslog/, Log/fusion/<run>/map*/tiles|keyframes
  product Log/fusion/<run>/map*/global_map*.pcd                  (standard voxel map: 0.3-1 GB)

Actions:
  slim    delete the heavy data only: trajectories, metrics, evaluation, map_info and (unless --no-keep-maps) the
          standard global map stay, so the round still shows in Studio with all its numbers (~95 % freed)
  delete  remove the round directory

Never touched: results/.studio, results/.mcp, roslog, rounds that are running (round.json status running /
starting / playing / ... or a live mapper writing to them), the newest --keep-recent rounds, pinned rounds
(file results/<round>/KEEP or listed in results/.studio/keep.json).

    /usr/bin/python3 tools/livo/results_gc.py list [--json]
    /usr/bin/python3 tools/livo/results_gc.py slim   <round> [<round> ...] [--apply]
    /usr/bin/python3 tools/livo/results_gc.py delete <round> [<round> ...] [--apply]
    /usr/bin/python3 tools/livo/results_gc.py auto --target-free-gb 80 [--allow-delete] [--keep-recent 3] [--apply]
    /usr/bin/python3 tools/livo/results_gc.py pin|unpin <round>
Without --apply only the plan is printed (dry run).
"""
import argparse
import datetime as dt
import json
import os
import pathlib
import shutil
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
RESULTS = REPO / "results"
KEEP_FILE = RESULTS / ".studio" / "keep.json"
SKIP_DIRS = {".studio", ".mcp", "roslog", "prepared"}
LIVE_PHASES = {"running", "starting", "playing", "draining", "finalizing", "exported", "evaluating"}
HEAVY_NAMES = {"scans.bin", "sources.bin", "colors.bin"}


def _size(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.lstat(os.path.join(root, f)).st_size
            except OSError:
                pass
    return total


def _heavy_files(round_dir, keep_maps=True):
    """Regenerable data of a round: (paths, bytes)."""
    out = []
    fusion = round_dir / "Log" / "fusion"
    for run in (fusion.glob("*/") if fusion.is_dir() else []):
        for f in run.iterdir():
            if f.is_file() and (f.name in HEAVY_NAMES or (f.suffix == ".pcd" and f.name.startswith(("map_", "rgb_map_")))):
                out.append(f)
            elif f.is_dir() and f.name.startswith("map"):
                for sub in ("tiles", "keyframes"):
                    if (f / sub).is_dir():
                        out.append(f / sub)
                if not keep_maps:
                    out += [g for g in f.glob("global_map*.pcd")]
    for d in (round_dir / "Log" / "pcd", round_dir / "roslog"):
        if d.is_dir() and any(d.iterdir()):
            out.append(d)
    return out, sum(_size(p) if p.is_dir() else p.stat().st_size for p in out)


def _pinned():
    try:
        return set(json.loads(KEEP_FILE.read_text()).get("rounds", []))
    except (OSError, ValueError):
        return set()


def _live_round_dirs():
    """Round directories a running mapper/player/evaluation writes to (cmdline mentions results/<round>)."""
    live = set()
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if "results/" in cmd and any(k in cmd for k in ("fastlivo_mapping", "run_wuhu_round", "player.py", "evaluate", "map_export")):
            for part in cmd.split():
                if "results/" in part:
                    name = part.split("results/", 1)[1].split("/", 1)[0]
                    if name:
                        live.add(name)
    return live


def scan(keep_recent=3):
    pinned, live = _pinned(), _live_round_dirs()
    rounds = []
    for d in RESULTS.iterdir() if RESULTS.is_dir() else []:
        if not d.is_dir() or d.name in SKIP_DIRS or d.name.startswith("."):
            continue
        info = {}
        try:
            info = json.loads((d / "round.json").read_text())
        except (OSError, ValueError):
            pass
        mtime = max((p.stat().st_mtime for p in (d, d / "round.json", d / "run.log") if p.exists()), default=d.stat().st_mtime)
        heavy, heavy_bytes = _heavy_files(d)
        rounds.append({"round": d.name, "bytes": _size(d), "heavy_bytes": heavy_bytes, "mtime": mtime,
                       "date": dt.datetime.fromtimestamp(mtime).isoformat(timespec="minutes"),
                       "status": info.get("status"), "phase": info.get("phase"), "mode": info.get("mode"),
                       "kind": "calibration" if d.name.startswith("calib_") else "mapping",
                       "pinned": d.name in pinned or (d / "KEEP").exists(),
                       "running": d.name in live or (info.get("phase") in LIVE_PHASES and info.get("status") == "running"),
                       "slim": heavy_bytes < 50e6 and (d / "Log").exists()})
    rounds.sort(key=lambda r: r["mtime"])
    for r in rounds[-keep_recent:] if keep_recent > 0 else []:
        r["recent"] = True
    for r in rounds:
        r["protected"] = bool(r["pinned"] or r["running"] or r.get("recent"))
    return rounds


def disk():
    du = shutil.disk_usage(RESULTS if RESULTS.exists() else REPO)
    return {"free_gb": round(du.free / 1e9, 1), "total_gb": round(du.total / 1e9, 1), "used_pct": round(100 * du.used / du.total, 1)}


def slim_round(name, apply, keep_maps=True):
    d = RESULTS / name
    files, total = _heavy_files(d, keep_maps=keep_maps)
    if apply:
        for f in files:
            shutil.rmtree(f, ignore_errors=True) if f.is_dir() else f.unlink(missing_ok=True)
        (d / "SLIMMED").write_text(json.dumps({"time": dt.datetime.now().isoformat(timespec="seconds"), "freed_bytes": total,
                                               "kept_global_map": keep_maps}) + "\n")
    return total


def delete_round(name, apply):
    d = RESULTS / name
    total = _size(d)
    if apply:
        shutil.rmtree(d, ignore_errors=True)
    return total


def usage(keep_recent=3):
    rounds = scan(keep_recent)
    return {"disk": disk(), "rounds": rounds, "total_bytes": sum(r["bytes"] for r in rounds),
            "reclaimable_bytes": sum(r["heavy_bytes"] for r in rounds if not r["protected"])}


def set_pinned(names, pinned=True):
    keep = _pinned()
    keep = keep | set(names) if pinned else keep - set(names)
    KEEP_FILE.parent.mkdir(parents=True, exist_ok=True)
    KEEP_FILE.write_text(json.dumps({"rounds": sorted(keep)}, ensure_ascii=False, indent=1) + "\n")
    return sorted(keep)


def run(action, names=(), apply=False, target_free_gb=80.0, allow_delete=False, keep_recent=3, keep_maps=True,
        force=False):
    """slim / delete the given rounds, or auto: oldest unprotected rounds first (slim, then optionally delete)
    until target_free_gb is free. Raises ValueError for unknown or protected rounds."""
    rounds = scan(keep_recent)
    by_name = {r["round"]: r for r in rounds}
    plan = []
    if action in ("slim", "delete"):
        for name in names:
            r = by_name.get(name)
            if r is None:
                raise ValueError(f"unknown round {name}")
            if r["running"] or (r["protected"] and not force):
                raise ValueError(f"{name} is protected ({'running' if r['running'] else 'pinned/recent'})")
            plan.append((action, name))
    elif action == "auto":
        need = (target_free_gb - disk()["free_gb"]) * 1e9
        cands = [r for r in rounds if not r["protected"]]
        for r in cands:
            if need <= 0:
                break
            if r["heavy_bytes"] > 50e6:
                plan.append(("slim", r["round"]))
                need -= r["heavy_bytes"]
        if need > 0 and allow_delete:
            for r in cands:
                if need <= 0:
                    break
                plan.append(("delete", r["round"]))
                need -= r["bytes"] - (r["heavy_bytes"] if ("slim", r["round"]) in plan else 0)
    else:
        raise ValueError(f"unknown action {action}")
    done, freed = [], 0
    for act, name in plan:
        b = slim_round(name, apply, keep_maps=keep_maps) if act == "slim" else delete_round(name, apply)
        freed += b
        done.append({"action": act, "round": name, "bytes": b})
    return {"applied": apply, "plan": done, "freed_bytes": freed, "disk": disk()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("list", "slim", "delete", "auto", "pin", "unpin"))
    ap.add_argument("rounds", nargs="*")
    ap.add_argument("--apply", action="store_true", help="execute (default: dry run)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--target-free-gb", type=float, default=80.0)
    ap.add_argument("--allow-delete", action="store_true", help="auto: delete whole rounds when slimming is not enough")
    ap.add_argument("--keep-recent", type=int, default=3)
    ap.add_argument("--no-keep-maps", action="store_true", help="slim also removes the standard global maps")
    ap.add_argument("--force", action="store_true", help="allow slim/delete of pinned/recent rounds (never running ones)")
    a = ap.parse_args(argv)
    if a.action == "list":
        doc = usage(a.keep_recent)
        if a.json:
            print(json.dumps(doc, ensure_ascii=False))
            return 0
        print(f"disk free {doc['disk']['free_gb']} GB of {doc['disk']['total_gb']} GB; results {doc['total_bytes'] / 1e9:.1f} GB, "
              f"reclaimable by slimming {doc['reclaimable_bytes'] / 1e9:.1f} GB")
        for r in doc["rounds"]:
            flags = ",".join(k for k in ("pinned", "running", "recent", "slim") if r.get(k))
            print(f"  {r['date']}  {r['bytes'] / 1e9:7.2f} GB  heavy {r['heavy_bytes'] / 1e9:6.2f} GB  {r['status'] or '-':10s} {r['round']}  {flags}")
        return 0
    if a.action in ("pin", "unpin"):
        print(json.dumps({"pinned": set_pinned(a.rounds, a.action == "pin")}, ensure_ascii=False))
        return 0
    try:
        doc = run(a.action, a.rounds, a.apply, a.target_free_gb, a.allow_delete, a.keep_recent, not a.no_keep_maps, a.force)
    except ValueError as exc:
        sys.exit(str(exc))
    print(json.dumps(doc, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
