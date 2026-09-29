#!/usr/bin/env python3
"""LIVO Studio v2: browser console for MY-LIVO (recordings -> calibration -> mapping -> results).

Standard-library HTTP server; runs inside the calibration container:

  /usr/bin/python3 tools/studio/server.py --port 8765

The server bootstraps the ROS 2 environment itself (tools/livo/rosenv.py), so it can be started
from any shell. Everything the UI does is a subprocess of the repository tools
(scripts/run_wuhu_round.sh, tools/calib/livo_calib.py, tools/livo/dataset.py); the browser only
orchestrates and displays. Job history lives in results/.studio/jobs/, operator settings in
config/carlibration/studio_state.json.
"""
import argparse
import json
import mimetypes
import os
import pathlib
import re
import shutil
import subprocess
import sys
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from livo import rosenv  # noqa: E402

rosenv.ensure()

from studio.app import calibration, common, datasets, jobs, mapping  # noqa: E402
from studio.app.common import (CAL_ROOT, MAPPER_BINARY, REPO, RESULTS, STUDIO_DIR, WEB, StudioError,  # noqa: E402
                               load_state, now, remember_dataset, update_state)

VERSION = "2.0"

_APP_MTIMES = {}


def reload_app_modules() -> None:
    """Hot-reload studio.app.* modules when their sources change (never under a running job).

    A long-lived Studio server used to keep executing stale pipeline code while serving fresh UI
    files (2026-09-23: the console listed the new 后轴对齐 step but the server ran the old pipeline).
    jobs.py keeps live job state and is never reloaded here.
    """
    import importlib
    try:
        busy = jobs.active()
    except Exception:  # noqa: BLE001
        busy = None
    for name in ("studio.app.common", "studio.app.datasets", "studio.app.calibration", "studio.app.mapping"):
        mod = sys.modules.get(name)
        path = getattr(mod, "__file__", None) if mod is not None else None
        if not path:
            continue
        try:
            mt = os.stat(path).st_mtime_ns
        except OSError:
            continue
        prev = _APP_MTIMES.get(name)
        if prev is None:
            _APP_MTIMES[name] = mt
            continue
        if mt == prev or busy:
            continue
        try:
            importlib.reload(mod)
            _APP_MTIMES[name] = mt
            print(f"[studio] hot-reloaded {name} (source changed)", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"[studio] hot-reload {name} failed: {exc}", flush=True)



def system_info():
    du = shutil.disk_usage(RESULTS if RESULTS.exists() else REPO)
    gpu = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total,utilization.gpu",
                          "--format=csv,noheader"], capture_output=True, text=True).stdout.strip()
    git = subprocess.run(["git", "-C", str(REPO), "log", "-1", "--format=%h %s"], capture_output=True, text=True).stdout.strip()
    branch = subprocess.run(["git", "-C", str(REPO), "branch", "--show-current"], capture_output=True, text=True).stdout.strip()
    active = jobs.active()
    return {"time": now(), "version": VERSION, "disk_free_gb": round(du.free / 1e9, 1), "disk_total_gb": round(du.total / 1e9, 1),
            "load": os.getloadavg(), "cpus": os.cpu_count(), "gpu": gpu, "git": git, "branch": branch,
            "binary_mtime": (__import__("datetime").datetime.fromtimestamp(MAPPER_BINARY.stat().st_mtime)
                             .isoformat(timespec="seconds") if MAPPER_BINARY.exists() else None),
            "binary_present": MAPPER_BINARY.exists(),
            "mapper_alive": common.pgrep("fastlivo_mapping"), "player_alive": common.pgrep("livo/player.py|livo.player"),
            "active_job": active.to_json() if active else None, "ros": rosenv.status(),
            "repo": str(REPO), "results": str(RESULTS), "jobs_dir": str(STUDIO_DIR / "jobs"), "pid": os.getpid()}


class Handler(BaseHTTPRequestHandler):
    server_version = f"LIVOStudio/{VERSION}"

    def log_message(self, fmt, *args):  # quiet: only errors
        if args and str(args[1]).startswith(("4", "5")) and "/api/" in str(args[0]):
            sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    # ------------------------------------------------------------------ plumbing
    def _send(self, code, payload, ctype="application/json; charset=utf-8"):
        body = payload if isinstance(payload, (bytes, bytearray)) else json.dumps(
            common.clean(payload), ensure_ascii=False, default=str, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _file(self, p, text=False):
        p = pathlib.Path(p)
        if not p.exists() or not p.is_file():
            return self._send(404, {"error": f"not found: {p}"})
        ctype = mimetypes.guess_type(str(p))[0] or ("text/plain; charset=utf-8" if text else "application/octet-stream")
        if ctype.startswith("text/") and "charset" not in ctype:
            ctype += "; charset=utf-8"
        self._send(200, p.read_bytes(), ctype)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b"{}"
        try:
            return json.loads(raw or b"{}")
        except json.JSONDecodeError:
            raise StudioError("请求体不是合法 JSON")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        self._dispatch(self._get)

    def do_POST(self):
        self._dispatch(self._post)

    def _dispatch(self, fn):
        reload_app_modules()
        try:
            fn()
        except StudioError as exc:
            self._send(400, {"error": str(exc)})
        except PermissionError as exc:
            self._send(403, {"error": str(exc)})
        except (FileNotFoundError, KeyError) as exc:
            self._send(404, {"error": f"{type(exc).__name__}: {exc}"})
        except Exception as exc:  # noqa: BLE001
            import traceback
            traceback.print_exc()
            self._send(500, {"error": f"{type(exc).__name__}: {exc}"})

    # ------------------------------------------------------------------ GET
    def _get(self):
        url = urllib.parse.urlparse(self.path)
        q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
        path = url.path
        if path in ("/", "/index.html"):
            return self._file(WEB / "index.html")
        if path.startswith("/static/"):
            return self._file(WEB / path[len("/static/"):].replace("..", ""))
        if path == "/api/state":
            return self._send(200, load_state())
        if path == "/api/system":
            return self._send(200, system_info())
        if path == "/api/overview":
            state = load_state()
            cat = calibration.catalogue()
            return self._send(200, {"state": state, "system": system_info(), "vehicles": calibration.vehicles(cat),
                                    "stages": cat["stages"], "runs": mapping.list_runs(limit=8),
                                    "jobs": jobs.list_jobs(limit=8),
                                    "dataset": datasets.describe(state["data"]["dataset"]) if state["data"].get("dataset") else None,
                                    "assessment": datasets.assessment_summary(state["data"]["dataset"]) if state["data"].get("dataset") else None})
        if path == "/api/fs":
            return self._send(200, datasets.browse(q.get("path") or "/home/project"))
        if path == "/api/datasets":
            info = datasets.describe(q["path"], refresh=q.get("refresh") == "1")
            return self._send(200, info)
        if path == "/api/datasets/assessment":
            return self._send(200, datasets.assessment(q["path"]))
        if path == "/api/vehicles":
            cat = calibration.catalogue(refresh=q.get("refresh") == "1")
            return self._send(200, {"vehicles": calibration.vehicles(cat), "stages": cat["stages"], "sources": cat["sources"]})
        if path == "/api/calibrations":
            return self._send(200, calibration.catalogue(refresh=q.get("refresh") == "1"))
        if path == "/api/calibrations/version":
            return self._send(200, calibration.version_detail(q["id"]))
        if path == "/api/calibrations/file":
            return self._file(calibration.version_file(q["id"], q["name"]))
        if path == "/api/overlays/file":
            return self._file(calibration.overlay_file(q.get("key"), q.get("name", "")))
        if path == "/api/jobs":
            return self._send(200, jobs.list_jobs(kind=q.get("kind"), limit=int(q.get("limit", 60))))
        m = re.match(r"^/api/jobs/([0-9a-f]+)$", path)
        if m:
            job = jobs.get(m.group(1))
            if not job:
                return self._send(404, {"error": "no such job"})
            doc = job.to_json()
            doc["log"] = jobs.log_lines(job, offset=int(q.get("offset", 0)))
            return self._send(200, doc)
        if path == "/api/runs":
            return self._send(200, mapping.list_runs(limit=int(q.get("limit", 100))))
        m = re.match(r"^/api/runs/([A-Za-z0-9_\-.]+)/live$", path)
        if m:
            return self._send(200, mapping.live(m.group(1)))
        m = re.match(r"^/api/runs/([A-Za-z0-9_\-.]+)/file$", path)
        if m:
            return self._file(mapping.run_file(m.group(1), q["name"]), text=True)
        return self._send(404, {"error": f"unknown path {path}"})

    # ------------------------------------------------------------------ POST
    def _post(self):
        path = urllib.parse.urlparse(self.path).path
        body = self._body()
        if path == "/api/state":
            return self._send(200, update_state(body))
        if path == "/api/datasets/select":
            p = str(common.safe_path(body["path"]))
            info = datasets.describe(p)
            if info.get("error"):
                raise StudioError(f"数据源不可用：{info['error']}")
            return self._send(200, {"state": remember_dataset(p), "dataset": info})
        if path == "/api/datasets/assess":
            p = str(common.safe_path(body["path"]))
            if jobs.active("assess"):
                raise StudioError("已有一个数据评估在运行，请等待其结束")
            calib_file = None
            mine, truck = body.get("mine"), body.get("truck")
            if mine and truck:
                calib_file = calibration.vehicle_status(calibration.catalogue(), mine, truck).get("calibration_file")
            cmd, report, info = datasets.assess_cmd(p, calib_file)
            job = jobs.start("assess", f"数据评估 · {info.get('name')}",
                             [{"title": "数据评估（一次流式读取 + 抽样点云/图像）", "cmd": cmd,
                               "after": lambda ctx, lines, _r=report: ctx.update(datasets.assess_result_ctx(_r))}],
                             meta={"path": p, "calibration": calib_file})
            return self._send(200, job.to_json())
        if path == "/api/datasets/forget":
            state = load_state()
            state["data"]["recent"] = [x for x in state["data"].get("recent", []) if x != body.get("path")]
            common.save_state(state)
            return self._send(200, state)
        if path == "/api/calibrations/import":
            cmd = calibration.import_cmd(body)
            job = jobs.start("calibration", f"导入出厂标定 · {body.get('mine')}/{body.get('truck')}",
                             [{"title": "导入出厂标定 (import)", "cmd": cmd,
                               "after": lambda ctx, lines: ctx.__setitem__("result_id", calibration.parse_version_id(lines))}],
                             meta=body)
            return self._send(200, job.to_json())
        if path == "/api/calibrations/stage":
            stage = body.get("stage") or ""
            label = calibration.catalogue()["stages"].get(stage, {}).get("label", stage)
            cmd = calibration.stage_cmd(stage, body)
            job = jobs.start("calibration", f"标定阶段 · {label}",
                             [{"title": f"{label} ({stage})", "cmd": cmd,
                               "after": lambda ctx, lines: ctx.__setitem__("result_id", calibration.parse_version_id(lines))}],
                             meta=body)
            return self._send(200, job.to_json())
        if path == "/api/calibrations/pipeline":
            if mapping.busy():
                raise StudioError("已有建图/流水线任务在运行，请先等待其结束")
            kind = body.get("kind") or "lidar"
            steps, info = calibration.pipeline_steps(kind, body, mapping.pipeline_round_cmd)
            title = {"lidar": "LiDAR 标定流水线", "camera": "相机标定流水线", "camloop": "相机外参闭环微调"}.get(kind, kind) + f" · {info['truck']}"
            job = jobs.start("pipeline", title, steps, meta={"kind": kind, **info, "options": body})
            return self._send(200, job.to_json())
        if path == "/api/calibrations/overlay":
            return self._send(200, calibration.overlay(body))
        if path == "/api/calibrations/activate":
            return self._send(200, calibration.activate(body["id"]))
        if path == "/api/calibrations/validate":
            return self._send(200, calibration.validate(body["id"], strict=bool(body.get("strict"))))
        if path == "/api/calibrations/delete":
            return self._send(200, calibration.delete(body["id"]))
        if path == "/api/mapping/start":
            return self._send(200, mapping.start(body).to_json())
        if path == "/api/mapping/stop":
            return self._send(200, mapping.stop(body.get("mode", "finish")))
        if path == "/api/mapping/kill":
            return self._send(200, mapping.kill_all())
        if path == "/api/evaluate":
            round_name = re.sub(r"[^A-Za-z0-9_\-.]", "", body.get("round") or "")
            if not (RESULTS / round_name).is_dir():
                raise StudioError(f"轮次不存在：{round_name}")
            cmd = ["bash", str(common.EVAL_SCRIPT), round_name]
            if body.get("dataset"):
                cmd += ["--dataset", str(common.safe_path(body["dataset"]))]
            job = jobs.start("evaluate", f"精度评估 · {round_name}", [{"title": "评估", "cmd": cmd}], meta={"round": round_name})
            return self._send(200, job.to_json())
        m = re.match(r"^/api/jobs/([0-9a-f]+)/stop$", path)
        if m:
            job = jobs.get(m.group(1))
            if not job:
                return self._send(404, {"error": "no such job"})
            job.stop(body.get("mode", "finish"))
            return self._send(200, job.to_json())
        return self._send(404, {"error": f"unknown path {path}"})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    CAL_ROOT.mkdir(parents=True, exist_ok=True)
    STUDIO_DIR.mkdir(parents=True, exist_ok=True)
    common.save_state(load_state())  # normalise/migrate persisted settings
    jobs.load_history()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    ros = rosenv.status()
    print(f"LIVO Studio {VERSION} listening on http://{a.host}:{a.port}  repo={REPO}  ros={'ok' if ros.get('ok') else 'MISSING'}",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
