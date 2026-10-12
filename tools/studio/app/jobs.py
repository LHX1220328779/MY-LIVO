"""Background jobs: one or more subprocess steps executed in order, logged to a file, persisted
under results/.studio/jobs/ so the history survives a Studio restart.

A step is a dict {"title": str, "cmd": [..] | "build": callable(ctx) -> [..], "after": callable(ctx, lines)}.
`ctx` is a plain dict shared by the steps (e.g. the version id produced by a calibration stage or
the fusion directory of a mapping round) and is stored in the job record as "results".
"""
import json
import os
import pathlib
import signal
import subprocess
import threading
import time

from . import common
from .common import JOBS_DIR, REPO, RESULTS, ROS_PROCESS_PATTERN, now

_lock = threading.RLock()
JOBS = {}
MAX_HISTORY = 300


class Job:
    def __init__(self, kind, title, steps, meta=None):
        self.id = f"{int(time.time() * 1000):x}"
        self.kind, self.title, self.meta = kind, title, meta or {}
        self.steps = [{"title": s.get("title", f"step {i + 1}"), "status": "pending", "started": None,
                       "finished": None, "returncode": None} for i, s in enumerate(steps)]
        self._specs = steps
        self.ctx = {}
        self.started, self.finished, self.returncode = now(), None, None
        self.status = "running"
        self.current = -1
        self.error = None
        self.stop_mode = None
        self.proc = None
        self.pid = None
        self.log_path = JOBS_DIR / f"{self.id}.log"
        self.lines = 0
        JOBS_DIR.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text("")
        self._save()
        self.thread = threading.Thread(target=self._run, daemon=True, name=f"job-{self.id}")
        self.thread.start()

    # ------------------------------------------------------------------ execution
    def _log(self, text):
        with _lock:
            with self.log_path.open("a") as f:
                f.write(text if text.endswith("\n") else text + "\n")
            self.lines += text.count("\n") or 1

    def _env(self):
        env = dict(os.environ)
        for v in ("CONDA_PREFIX", "CONDA_DEFAULT_ENV", "PYTHONHOME"):
            env.pop(v, None)
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _run(self):
        try:
            for i, spec in enumerate(self._specs):
                if self.stop_mode == "abort":
                    break
                self.current = i
                step = self.steps[i]
                step["status"], step["started"] = "running", now()
                self._save()
                try:
                    cmd = spec["cmd"] if "cmd" in spec else spec["build"](self.ctx)
                except common.StudioError as exc:
                    step["status"], step["finished"], step["returncode"] = "failed", now(), 2
                    self.error = str(exc)
                    self._log(f"=== [{i + 1}/{len(self.steps)}] {step['title']}: {exc}")
                    break
                cmd = [str(c) for c in cmd]
                retries = int(spec.get("retry") or 0)
                rc, lines, start_failed = None, [], False
                for attempt in range(retries + 1):
                    if attempt:
                        # keep the failed attempt for diagnosis and re-run under the same round name
                        self._log(f"!!! {step['title']}: 退出码 {rc}，重试 {attempt}/{retries}")
                        if spec.get("round"):
                            src = RESULTS / spec["round"]
                            if src.is_dir():
                                # a resumed job retries under the same round name: never collide with the
                                # .failedN directory an earlier job left behind
                                n = attempt
                                while (RESULTS / f"{spec['round']}.failed{n}").exists():
                                    n += 1
                                src.rename(RESULTS / f"{spec['round']}.failed{n}")
                    self._log(f"=== [{i + 1}/{len(self.steps)}] {step['title']}\n$ {' '.join(cmd)}")
                    lines = []
                    try:
                        self.proc = subprocess.Popen(cmd, cwd=REPO, env=self._env(), stdout=subprocess.PIPE,
                                                     stderr=subprocess.STDOUT, text=True, bufsize=1,
                                                     start_new_session=True)
                    except Exception as exc:  # noqa: BLE001
                        step["status"], step["finished"], step["returncode"] = "failed", now(), 127
                        self.error = f"failed to start: {exc}"
                        self._log(self.error)
                        start_failed = True
                        break
                    self.pid = self.proc.pid
                    self._save()
                    for line in self.proc.stdout:
                        line = line.rstrip("\n")
                        lines.append(line)
                        self._log(line)
                    self.proc.wait()
                    rc = self.proc.returncode
                    if rc == 0 or self.stop_mode == "abort":
                        break
                if start_failed:
                    break
                step["returncode"], step["finished"] = rc, now()
                if rc != 0:
                    step["status"] = "stopped" if self.stop_mode == "abort" else "failed"
                    if self.stop_mode != "abort":
                        self.error = f"{step['title']} 退出码 {rc}"
                        tail = [l for l in lines[-6:] if l.strip()]
                        if tail:
                            self.error += "：" + tail[-1][:300]
                    break
                step["status"] = "done"
                if spec.get("after"):
                    try:
                        spec["after"](self.ctx, lines)
                    except common.StudioError as exc:
                        step["status"] = "failed"
                        self.error = str(exc)
                        self._log(f"!!! {exc}")
                        break
                self._save()
            for step in self.steps:
                if step["status"] == "pending":
                    step["status"] = "skipped"
            if self.stop_mode == "abort":
                self.status, self.returncode = "stopped", 130
            elif self.error:
                self.status, self.returncode = "failed", 1
            else:
                self.status, self.returncode = "done", 0
        except Exception as exc:  # noqa: BLE001
            self.status, self.returncode, self.error = "failed", 1, f"internal error: {exc}"
            self._log(self.error)
        finally:
            self.finished = now()
            self.proc = None
            self._save()
            for fn in list(_listeners):
                try:
                    fn(self)
                except Exception:  # noqa: BLE001
                    pass

    # ------------------------------------------------------------------ control
    @property
    def alive(self):
        return self.status == "running"

    def stop(self, mode="finish"):
        """finish: let a mapping round end early but complete export/evaluation; abort: kill everything."""
        if not self.alive:
            return
        if mode == "finish":
            self.stop_mode = "finish"
            self._log("=== stop requested: finishing early (playback interrupted, export continues)")
            subprocess.run(["pkill", "-INT", "-f", "livo/player.py|livo.player"], check=False)
            return
        self.stop_mode = "abort"
        self._log("=== abort requested")
        proc = self.proc
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if self.kind in ("mapping", "pipeline"):
            subprocess.run(["pkill", "-TERM", "-f", ROS_PROCESS_PATTERN], check=False)

            def hard_kill():
                time.sleep(8)
                if proc is not None and proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                subprocess.run(["pkill", "-KILL", "-f", ROS_PROCESS_PATTERN], check=False)
            threading.Thread(target=hard_kill, daemon=True).start()

    # ------------------------------------------------------------------ serialisation
    def to_json(self):
        return {"id": self.id, "kind": self.kind, "title": self.title, "meta": self.meta, "steps": self.steps,
                "current": self.current, "started": self.started, "finished": self.finished,
                "returncode": self.returncode, "status": self.status, "error": self.error,
                "stop_mode": self.stop_mode, "results": self.ctx, "pid": self.pid, "log_path": str(self.log_path),
                "lines": self.lines}

    def _save(self):
        with _lock:
            (JOBS_DIR / f"{self.id}.json").write_text(json.dumps(self.to_json(), ensure_ascii=False, indent=1))


class StoredJob:
    """Read-only record of a job from an earlier Studio process."""

    def __init__(self, doc):
        self.doc = doc
        if doc.get("status") == "running":
            pid = doc.get("pid")
            alive = bool(pid) and pathlib.Path(f"/proc/{pid}").exists()
            doc["status"] = "running" if alive else "lost"
            if not alive:
                doc["error"] = doc.get("error") or "Studio 重启时该任务已不在运行"
                doc["finished"] = doc.get("finished") or now()
                for step in doc.get("steps", []):
                    if step.get("status") == "running":
                        step["status"] = "lost"
        self.id = doc["id"]
        self.kind = doc.get("kind")
        self.status = doc["status"]
        self.started = doc.get("started")
        self.log_path = pathlib.Path(doc.get("log_path") or JOBS_DIR / f"{self.id}.log")

    alive = property(lambda self: False)

    def stop(self, mode="finish"):
        return None

    def to_json(self):
        return dict(self.doc)


_listeners = []


def on_finished(fn):
    _listeners.append(fn)


def load_history():
    JOBS_DIR.mkdir(parents=True, exist_ok=True)
    docs = []
    for p in JOBS_DIR.glob("*.json"):
        doc = common.read_json(p)
        if doc and doc.get("id"):
            docs.append(doc)
    docs.sort(key=lambda d: d.get("started") or "", reverse=True)
    for doc in docs[MAX_HISTORY:]:
        for suffix in (".json", ".log"):
            try:
                (JOBS_DIR / f"{doc['id']}{suffix}").unlink()
            except FileNotFoundError:
                pass
    for doc in docs[:MAX_HISTORY]:
        job = StoredJob(doc)
        JOBS[job.id] = job
        if doc["status"] == "lost":
            (JOBS_DIR / f"{job.id}.json").write_text(json.dumps(doc, ensure_ascii=False, indent=1))


def start(kind, title, steps, meta=None):
    job = Job(kind, title, steps, meta=meta)
    JOBS[job.id] = job
    return job


def get(job_id):
    return JOBS.get(job_id)


def list_jobs(kind=None, limit=60):
    rows = [j.to_json() for j in JOBS.values() if kind in (None, "", j.kind)]
    rows.sort(key=lambda d: d.get("started") or "", reverse=True)
    return rows[:limit]


def active(kind=None):
    for j in JOBS.values():
        if j.alive and kind in (None, j.kind):
            return j
    return None


def log_lines(job, offset=0, limit=3000):
    text = common.read_tail(job.log_path, 4_000_000)
    lines = text.splitlines()
    total = len(lines)
    offset = max(0, min(offset, total))
    chunk = lines[offset:offset + limit]
    return {"offset": offset, "next": offset + len(chunk), "total": total, "lines": chunk}
