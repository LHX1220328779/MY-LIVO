#!/usr/bin/env python3
"""LIVO Studio: browser front-end for data preparation, calibration management, one-click
mapping and live accuracy monitoring of MY-LIVO. Standard-library HTTP server; runs inside the
calibration container:

  /usr/bin/python3 tools/studio/server.py --port 8765

Everything the UI does is a subprocess of the existing repository tools (scripts/*.sh,
tools/calib/livo_calib.py), so the browser only orchestrates and displays.
"""
import argparse
import datetime as dt
import json
import mimetypes
import os
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

REPO = pathlib.Path(__file__).resolve().parents[2]
WEB = pathlib.Path(__file__).resolve().parent / "web"
CAL_ROOT = REPO / "config" / "carlibration"
RESULTS = REPO / "results"
STATE_FILE = CAL_ROOT / "studio_state.json"
CALIB_TOOL = REPO / "tools" / "calib" / "livo_calib.py"
ALLOWED_ROOTS = [pathlib.Path("/home/project"), REPO]
PY = "/usr/bin/python3"

DEFAULT_STATE = {
    "data": {"bag_root": "/home/project/data/haibo/wuhu_livo/ros2bag",
             "prepared_output": str(RESULTS / "prepared" / "wuhu_livo_camera"), "lio_only": False},
    "calibration": {"mine": "wuhu", "truck": "truck29", "source_file": str(CAL_ROOT / "perception_truck29.json"),
                    "bag": str(RESULTS / "prepared" / "wuhu_livo_camera"), "run": "", "lidar_offset": -0.125,
                    "camera_offset": -0.115},
    "mapping": {"round_prefix": "studio", "mode": "livo", "use_rtk": True, "save_map": True, "rviz": False,
                "rate": 1.0, "bag": str(RESULTS / "prepared" / "wuhu_livo_camera"), "calibration_id": "",
                "precheck": True, "frame_divider": 1, "evaluate": True, "extra_sets": ""},
}

_lock = threading.Lock()


# ----------------------------------------------------------------------------- helpers
def clean(obj):
    """Browsers reject NaN/Infinity in JSON; map them to null."""
    if isinstance(obj, float):
        return obj if obj == obj and obj not in (float("inf"), float("-inf")) else None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


def now():
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def load_state():
    state = json.loads(json.dumps(DEFAULT_STATE))
    if STATE_FILE.exists():
        try:
            saved = json.loads(STATE_FILE.read_text())
            for k, v in saved.items():
                if isinstance(v, dict) and isinstance(state.get(k), dict):
                    state[k].update(v)
                else:
                    state[k] = v
        except Exception:  # noqa: BLE001
            pass
    return state


def save_state(state):
    CAL_ROOT.mkdir(parents=True, exist_ok=True)
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(STATE_FILE)


def safe_path(raw):
    p = pathlib.Path(raw).resolve()
    if not any(p == r or r in p.parents for r in ALLOWED_ROOTS):
        raise PermissionError(f"path outside allowed roots: {p}")
    return p


def read_tail(path, max_bytes=200_000):
    try:
        size = path.stat().st_size
        with path.open("rb") as f:
            if size > max_bytes:
                f.seek(size - max_bytes)
            return f.read().decode("utf-8", "replace")
    except FileNotFoundError:
        return ""


def parse_kv_log(path):
    out = {}
    for line in read_tail(path, 20_000).splitlines():
        if "=" in line and not line.startswith("[") and not line.startswith("dirty"):
            k, _, v = line.partition("=")
            k, v = k.strip(), v.strip()
            if k == "mode":  # "mode=livo use_rtk=true save_map=true rviz=false rate=1.0 start_offset=0"
                parts = v.split()
                v = parts[0]
                for extra in parts[1:]:
                    k2, _, v2 = extra.partition("=")
                    out[k2] = v2
            out[k] = v
    return out


def csv_rows(path, max_points=400, max_bytes=20_000_000):
    """Header + evenly subsampled data rows (always including the last row)."""
    text = read_tail(path, max_bytes)
    lines = [l for l in text.splitlines() if l and not l.startswith("#")]
    if not lines:
        return [], []
    with path.open() as f:
        header = None
        for line in f:
            if not line.startswith("#"):
                header = line.strip().split(",")
                break
    if header is None:
        return [], []
    data = [l for l in lines if not l.startswith(header[0] + ",")]
    if len(data) > max_points:
        step = len(data) / max_points
        picked = [data[int(i * step)] for i in range(max_points)]
        if picked[-1] != data[-1]:
            picked.append(data[-1])
        data = picked
    rows = []
    for l in data:
        parts = l.split(",")
        if len(parts) == len(header):
            rows.append(parts)
    return header, rows


def count_lines(path):
    try:
        with path.open("rb") as f:
            return sum(1 for _ in f)
    except FileNotFoundError:
        return 0


# ----------------------------------------------------------------------------- jobs
class Job:
    def __init__(self, kind, title, cmd, cwd=REPO, env=None, meta=None):
        self.id = f"{int(time.time() * 1000):x}"
        self.kind, self.title, self.cmd, self.meta = kind, title, cmd, meta or {}
        self.started, self.finished, self.returncode = now(), None, None
        self.log = []
        self.log_bytes = 0
        self.proc = None
        self.env = dict(os.environ)
        self.env.update({"PATH": "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"})
        for v in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME", "PYTHONPATH"):
            self.env.pop(v, None)
        if env:
            self.env.update(env)
        self.cwd = cwd
        self.started_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.started_event.wait(5)

    def _run(self):
        try:
            self.proc = subprocess.Popen(self.cmd, cwd=self.cwd, env=self.env, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True, bufsize=1,
                                         start_new_session=True)
        except Exception as e:  # noqa: BLE001
            self.log.append(f"failed to start: {e}")
            self.returncode, self.finished = 127, now()
            self.started_event.set()
            return
        self.started_event.set()
        for line in self.proc.stdout:
            with _lock:
                self.log.append(line.rstrip("\n"))
                if len(self.log) > 5000:
                    del self.log[:1000]
        self.proc.wait()
        self.returncode, self.finished = self.proc.returncode, now()

    @property
    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self, graceful=True):
        if not self.alive:
            return
        if graceful and self.kind == "mapping":
            # Finish early: stop playback; the round script drains, exports and evaluates.
            subprocess.run(["pkill", "-INT", "-f", "ros2 bag play"], check=False)
        else:
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if self.kind == "mapping":
                subprocess.run(["pkill", "-TERM", "-f", "fastlivo_mapping|ros2 bag play|wuhu_truck29.launch|image_transport/republish"],
                               check=False)

    def to_json(self, with_log=False, offset=0):
        d = {"id": self.id, "kind": self.kind, "title": self.title, "cmd": self.cmd, "started": self.started,
             "finished": self.finished, "returncode": self.returncode, "alive": self.alive, "meta": self.meta,
             "lines": len(self.log)}
        if with_log:
            d["log"] = self.log[offset:]
            d["offset"] = offset
        return d


JOBS = {}


def start_job(kind, title, cmd, meta=None):
    job = Job(kind, title, cmd, meta=meta)
    JOBS[job.id] = job
    return job


# ----------------------------------------------------------------------------- domain
def list_calibrations():
    reg = json.loads((CAL_ROOT / "registry.json").read_text()) if (CAL_ROOT / "registry.json").exists() else {"active": {}}
    versions = []
    for meta in sorted(CAL_ROOT.glob("*/*/*/meta.json")):
        try:
            m = json.loads(meta.read_text())
        except Exception as e:  # noqa: BLE001
            m = {"id": str(meta.parent.relative_to(CAL_ROOT)), "error": str(e)}
        m["dir"] = str(meta.parent)
        m["overlays"] = sorted(p.name for p in (meta.parent / "overlays").glob("*.jpg")) if (meta.parent / "overlays").exists() else []
        versions.append(m)
    loose = [str(p) for p in sorted(CAL_ROOT.glob("*.json")) if p.name not in ("registry.json", "studio_state.json")]
    loose += [str(p) for p in sorted((REPO / "config").glob("perception_*.json"))]
    return {"active": reg.get("active", {}), "versions": versions, "loose_files": loose}


def bag_info(path):
    meta = path / "metadata.yaml"
    info = {"path": str(path), "name": path.name, "exists": meta.exists()}
    if meta.exists():
        try:
            import yaml
            d = yaml.safe_load(meta.read_text())["rosbag2_bagfile_information"]
            info["duration_s"] = round(d["duration"]["nanoseconds"] / 1e9, 1)
            info["messages"] = d["message_count"]
            info["topics"] = {t["topic_metadata"]["name"]: t["message_count"] for t in d["topics_with_message_count"]}
            info["camera"] = any(n.endswith("midrange_camera/ffmpeg") for n in info["topics"])
            info["size_mb"] = round(sum(p.stat().st_size for p in path.glob("*.mcap")) / 1e6, 1)
        except Exception as e:  # noqa: BLE001
            info["error"] = str(e)
    return info


def list_bags():
    root = RESULTS / "prepared"
    if not root.exists():
        return []
    return [bag_info(p) for p in sorted(root.iterdir()) if p.is_dir() and (p / "metadata.yaml").exists()]


def summary_head(path, n=14):
    return read_tail(path, 40_000).splitlines()[:n] if path.exists() else []


def parse_summary_metrics(lines):
    out = {}
    for l in lines:
        m = re.match(r"\[(native|optimized)\] rear-axle 3D vs RTK: median ([\d.]+) P95 ([\d.]+) max ([\d.]+) \| route <=10cm ([\d.]+)% >20cm ([\d.]+)%\s+-> budget (\w+)", l)
        if m:
            out[m.group(1)] = {"median_m": float(m.group(2)), "p95_m": float(m.group(3)), "max_m": float(m.group(4)),
                               "route_le10cm_pct": float(m.group(5)), "route_gt20cm_pct": float(m.group(6)),
                               "budget": m.group(7)}
    return out


def list_runs(limit=80):
    runs = []
    if not RESULTS.exists():
        return runs
    dirs = [p for p in RESULTS.iterdir() if p.is_dir() and (p / "run.log").exists()]
    dirs.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    for p in dirs[:limit]:
        kv = parse_kv_log(p / "run.log")
        fusion = sorted((p / "Log" / "fusion").glob("*/")) if (p / "Log" / "fusion").exists() else []
        summ = summary_head(p / "evaluation" / "summary.txt")
        runs.append({"round": p.name, "path": str(p), "mode": kv.get("mode", ""), "start": kv.get("start", ""),
                     "end": kv.get("end", ""), "overrides": kv.get("overrides", ""), "bag": kv.get("bag", ""),
                     "calibration": kv.get("calibration", ""), "fusion_dir": str(fusion[-1]) if fusion else "",
                     "evaluated": bool(summ), "metrics": parse_summary_metrics(summ),
                     "precheck": (p / "Log" / "extrinsic_precheck.json").exists(),
                     "mtime": dt.datetime.fromtimestamp(p.stat().st_mtime).isoformat(timespec="seconds")})
    return runs


_EVENT_CACHE = {}
EVENT_KEYS = ("Extrinsic precheck", "[WARN]", "[ERROR]", "[FATAL]", "RTK finalized", "Final keyframe",
              "INS attitude factors", "Loaded", "Time offsets", "Camera model", "IMU Initials",
              "Merged Multi-LiDAR", "process has died", "Exported", "map_")


def launch_events(path, limit=60):
    """Key algorithm events from launch.log (whole file, incrementally scanned and cached)."""
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


def run_live(round_name):
    p = RESULTS / round_name
    out = {"round": round_name, "exists": p.exists()}
    if not p.exists():
        return out
    out["run_log"] = parse_kv_log(p / "run.log")
    fusion = sorted((p / "Log" / "fusion").glob("*/")) if (p / "Log" / "fusion").exists() else []
    out["mapper_alive"] = subprocess.run(["pgrep", "-f", "fastlivo_mapping"], capture_output=True).returncode == 0
    out["player_alive"] = subprocess.run(["pgrep", "-f", "ros2 bag play"], capture_output=True).returncode == 0
    out["launch_events"] = launch_events(p / "launch.log")
    if fusion:
        f = fusion[-1]
        out["fusion_dir"] = str(f)
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
            finite = [p["frontend_rtk_residual_m"] for p in series if p["frontend_rtk_residual_m"] == p["frontend_rtk_residual_m"]]
            out["latest"]["frontend_rtk_residual_m"] = finite[-1] if finite else None
        hdr, rows = csv_rows(f / "online_path.csv")
        if rows:
            col = {n: i for i, n in enumerate(hdr)}
            last = rows[-1]
            out["online"] = {n: last[i] for n, i in col.items()}
            out["online_series"] = [{"t": float(r[col["stamp"]]), "latest": float(r[col["latest_imu_difference_m"]]),
                                     "max": float(r[col["max_imu_difference_m"]])} for r in rows if len(r) == len(hdr)]
        maps = sorted(f.glob("map_*.pcd"))
        out["map_chunks"] = len(maps)
        out["rgb_map_chunks"] = len(list(f.glob("rgb_map_*.pcd")))
        out["map_size_mb"] = round(sum(m.stat().st_size for m in maps) / 1e6, 1)
    pre = p / "Log" / "extrinsic_precheck.json"
    if pre.exists():
        try:
            out["precheck"] = json.loads(pre.read_text())
        except Exception:  # noqa: BLE001
            out["precheck"] = {"status": "unreadable"}
    summ = p / "evaluation" / "summary.txt"
    if summ.exists():
        lines = read_tail(summ, 60_000).splitlines()
        out["summary"] = lines[:60]
        out["metrics"] = parse_summary_metrics(lines)
        traj = p / "evaluation" / "trajectory.log"
        if traj.exists():
            out["thickness"] = [l for l in read_tail(traj, 60_000).splitlines() if "ground-like: cells" in l][:3]
    return out


# ----------------------------------------------------------------------------- actions
def act_prepare(body):
    bag_root = str(safe_path(body["bag_root"]))
    output = str(safe_path(body["output"]))
    cmd = [PY, str(REPO / "scripts" / "prepare_wuhu_bag.py"), "--bag-root", bag_root, "--output", output]
    if body.get("lio_only"):
        cmd.append("--lio-only")
    return start_job("prepare", f"数据准备 → {pathlib.Path(output).name}", cmd, meta=body)


def act_calib(body):
    stage = body["stage"]
    cmd = [PY, str(CALIB_TOOL), stage]
    if stage == "import":
        cmd += ["--source", str(safe_path(body["source"])), "--mine", body["mine"], "--truck", body["truck"]]
        if body.get("lidar_offset") not in (None, ""):
            cmd += ["--lidar-offset", str(body["lidar_offset"])]
        if body.get("camera_offset") not in (None, ""):
            cmd += ["--camera-offset", str(body["camera_offset"])]
    else:
        cmd += ["--id", body["id"]]
    if stage == "ground":
        cmd += ["--bag", str(safe_path(body["bag"]))]
    elif stage == "driven":
        cmd += ["--bag", str(safe_path(body["bag"])), "--run", str(safe_path(body["run"]))]
    elif stage == "camera-time":
        cmd += ["--bag", str(safe_path(body["bag"]))]
    elif stage == "camera-lidar":
        cmd += ["--bag", str(safe_path(body["bag"])), "--run", str(safe_path(body["run"])),
                "--frames", str(int(body.get("frames", 12)))]
    elif stage == "from-precheck":
        cmd += ["--run-dir", str(safe_path(body["run_dir"]))]
    elif stage == "set":
        for k, flag in (("camera_offset", "--camera-offset"), ("lidar_offset", "--lidar-offset")):
            if body.get(k) not in (None, ""):
                cmd += [flag, str(body[k])]
        if body.get("intrinsics"):
            cmd += ["--intrinsics", *[str(v) for v in body["intrinsics"]]]
        if body.get("distortion"):
            cmd += ["--distortion", *[str(v) for v in body["distortion"]]]
        if body.get("camera_rot_delta"):
            cmd += ["--camera-rot-delta", *[str(v) for v in body["camera_rot_delta"]]]
    if body.get("note"):
        cmd += ["--note", body["note"]]
    if body.get("activate"):
        cmd.append("--activate")
    return start_job("calibration", f"标定 · {stage}", cmd, meta=body)


def act_activate(body):
    r = subprocess.run([PY, str(CALIB_TOOL), "activate", "--id", body["id"]], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stdout + r.stderr)
    return json.loads(r.stdout.strip().splitlines()[-1])


def calibration_file_for(cid):
    d = CAL_ROOT.joinpath(*cid.strip("/").split("/"))
    f = d / "calibration.json"
    if not f.exists():
        raise FileNotFoundError(f"calibration version not found: {cid}")
    return f


def act_mapping(body):
    if any(j.kind == "mapping" and j.alive for j in JOBS.values()):
        raise RuntimeError("a mapping round is already running")
    prefix = re.sub(r"[^A-Za-z0-9_\-]", "_", body.get("round_prefix") or "studio")
    stamp = dt.datetime.now().strftime("%m%d_%H%M%S")
    mode = body.get("mode", "livo")
    round_name = f"{prefix}_{mode}_{stamp}"
    cmd = ["bash", str(REPO / "scripts" / "run_wuhu_round.sh"), round_name, "--mode", mode]
    if not body.get("use_rtk", True):
        cmd.append("--no-rtk")
    if not body.get("save_map", True):
        cmd.append("--no-save-map")
    if body.get("rviz"):
        cmd.append("--rviz")
    if body.get("rate"):
        cmd += ["--rate", str(body["rate"])]
    if body.get("bag"):
        cmd += ["--bag", str(safe_path(body["bag"]))]
    if body.get("calibration_id"):
        cmd += ["--calibration", str(calibration_file_for(body["calibration_id"]))]
    cmd += ["--set", f"multi_lidar.precheck.enabled={'true' if body.get('precheck', True) else 'false'}"]
    if int(body.get("frame_divider", 1) or 1) > 1:
        cmd += ["--set", f"camera.frame_divider={int(body['frame_divider'])}"]
    for item in (body.get("extra_sets") or "").split():
        if "=" in item:
            cmd += ["--set", item]
    if not body.get("evaluate", True):
        cmd.append("--no-eval")
    job = start_job("mapping", f"建图 · {round_name}", cmd, meta={"round": round_name, **body})
    return job


# ----------------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    server_version = "LIVOStudio/1.0"

    def log_message(self, fmt, *args):  # quieter console
        if "/api/jobs" in (args[0] if args else "") or "/live" in (args[0] if args else ""):
            return
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def _send(self, code, payload, ctype="application/json; charset=utf-8"):
        body = payload if isinstance(payload, (bytes, bytearray)) else json.dumps(clean(payload), ensure_ascii=False, default=str).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n) or b"{}") if n else {}

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,PUT,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        try:
            self._get()
        except Exception as e:  # noqa: BLE001
            self._send(500, {"error": str(e)})

    def do_POST(self):
        try:
            self._post()
        except Exception as e:  # noqa: BLE001
            self._send(400, {"error": str(e)})

    def do_PUT(self):
        self.do_POST()

    def _get(self):
        url = urllib.parse.urlparse(self.path)
        q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
        path = url.path
        if path in ("/", "/index.html"):
            return self._file(WEB / "index.html")
        if path.startswith("/static/"):
            return self._file(WEB / path[len("/static/"):])
        if path == "/api/state":
            return self._send(200, load_state())
        if path == "/api/system":
            du = shutil.disk_usage(RESULTS if RESULTS.exists() else REPO)
            load = os.getloadavg()
            gpu = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total,utilization.gpu",
                                  "--format=csv,noheader"], capture_output=True, text=True).stdout.strip()
            git = subprocess.run(["git", "-C", str(REPO), "log", "-1", "--format=%h %s"], capture_output=True, text=True).stdout.strip()
            branch = subprocess.run(["git", "-C", str(REPO), "branch", "--show-current"], capture_output=True, text=True).stdout.strip()
            binary = REPO / "install" / "fast_livo" / "lib" / "fast_livo" / "fastlivo_mapping"
            return self._send(200, {"time": now(), "disk_free_gb": round(du.free / 1e9, 1), "disk_total_gb": round(du.total / 1e9, 1),
                                    "load": load, "cpus": os.cpu_count(), "gpu": gpu, "git": git, "branch": branch,
                                    "binary_mtime": dt.datetime.fromtimestamp(binary.stat().st_mtime).isoformat(timespec="seconds") if binary.exists() else None,
                                    "mapper_alive": subprocess.run(["pgrep", "-f", "fastlivo_mapping"], capture_output=True).returncode == 0,
                                    "repo": str(REPO)})
        if path == "/api/fs":
            p = safe_path(q.get("path") or "/home/project")
            if not p.is_dir():
                p = p.parent
            entries = []
            for child in sorted(p.iterdir(), key=lambda c: (not c.is_dir(), c.name.lower())):
                if child.name.startswith("."):
                    continue
                try:
                    is_dir = child.is_dir()
                    entries.append({"name": child.name, "path": str(child), "dir": is_dir,
                                    "bag": is_dir and (child / "metadata.yaml").exists(),
                                    "size": 0 if is_dir else child.stat().st_size})
                except OSError:
                    continue
            return self._send(200, {"path": str(p), "parent": str(p.parent), "entries": entries[:500]})
        if path == "/api/calibrations":
            return self._send(200, list_calibrations())
        if path == "/api/calibrations/report":
            d = CAL_ROOT.joinpath(*q["id"].strip("/").split("/"))
            files = {n: (d / n).read_text() for n in ("report.md", "stage_report.txt", "camera_time.json") if (d / n).exists()}
            calib = json.loads((d / "calibration.json").read_text())
            return self._send(200, {"id": q["id"], "files": files, "calibration": calib,
                                    "overlays": sorted(p.name for p in (d / "overlays").glob("*.jpg")) if (d / "overlays").exists() else []})
        if path == "/api/calibrations/file":
            d = CAL_ROOT.joinpath(*q["id"].strip("/").split("/"))
            return self._file(safe_path(d / q["name"]))
        if path == "/api/bags":
            return self._send(200, list_bags())
        if path == "/api/bag":
            return self._send(200, bag_info(safe_path(q["path"])))
        if path == "/api/jobs":
            return self._send(200, [j.to_json() for j in sorted(JOBS.values(), key=lambda j: j.started, reverse=True)])
        m = re.match(r"^/api/jobs/([0-9a-f]+)$", path)
        if m:
            job = JOBS.get(m.group(1))
            if not job:
                return self._send(404, {"error": "no such job"})
            return self._send(200, job.to_json(with_log=True, offset=int(q.get("offset", 0))))
        if path == "/api/runs":
            return self._send(200, list_runs())
        m = re.match(r"^/api/runs/([A-Za-z0-9_\-.]+)/live$", path)
        if m:
            return self._send(200, run_live(m.group(1)))
        m = re.match(r"^/api/runs/([A-Za-z0-9_\-.]+)/file$", path)
        if m:
            return self._file(safe_path(RESULTS / m.group(1) / q["name"]), text=True)
        return self._send(404, {"error": f"unknown path {path}"})

    def _post(self):
        url = urllib.parse.urlparse(self.path)
        path = url.path
        body = self._body()
        if path == "/api/state":
            state = load_state()
            for k, v in body.items():
                if isinstance(v, dict) and isinstance(state.get(k), dict):
                    state[k].update(v)
                else:
                    state[k] = v
            save_state(state)
            return self._send(200, state)
        if path == "/api/prepare":
            return self._send(200, act_prepare(body).to_json())
        if path == "/api/calibrations/stage":
            return self._send(200, act_calib(body).to_json())
        if path == "/api/calibrations/activate":
            return self._send(200, act_activate(body))
        if path == "/api/mapping/start":
            return self._send(200, act_mapping(body).to_json())
        m = re.match(r"^/api/jobs/([0-9a-f]+)/stop$", path)
        if m:
            job = JOBS.get(m.group(1))
            if not job:
                return self._send(404, {"error": "no such job"})
            job.stop(graceful=body.get("graceful", True))
            return self._send(200, job.to_json())
        if path == "/api/mapping/kill":
            subprocess.run(["pkill", "-TERM", "-f", "fastlivo_mapping|ros2 bag play|wuhu_truck29.launch|image_transport/republish"], check=False)
            return self._send(200, {"ok": True})
        return self._send(404, {"error": f"unknown path {path}"})

    def _file(self, p, text=False):
        p = pathlib.Path(p)
        if not p.exists() or not p.is_file():
            return self._send(404, {"error": f"not found: {p}"})
        ctype = mimetypes.guess_type(str(p))[0] or ("text/plain; charset=utf-8" if text else "application/octet-stream")
        if ctype.startswith("text/") and "charset" not in ctype:
            ctype += "; charset=utf-8"
        self._send(200, p.read_bytes(), ctype)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    CAL_ROOT.mkdir(parents=True, exist_ok=True)
    if not STATE_FILE.exists():
        save_state(load_state())
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f"LIVO Studio listening on http://{a.host}:{a.port}  (repo {REPO})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
