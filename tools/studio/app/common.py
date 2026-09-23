"""Paths, persisted operator settings and small file helpers shared by the Studio modules."""
import datetime as dt
import json
import pathlib
import subprocess
import threading

REPO = pathlib.Path(__file__).resolve().parents[3]
WEB = REPO / "tools" / "studio" / "web"
TOOLS = REPO / "tools"
CAL_ROOT = REPO / "config" / "carlibration"
RESULTS = REPO / "results"
STUDIO_DIR = RESULTS / ".studio"
JOBS_DIR = STUDIO_DIR / "jobs"
STATE_FILE = CAL_ROOT / "studio_state.json"
CALIB_TOOL = TOOLS / "calib" / "livo_calib.py"
DATASET_TOOL = TOOLS / "livo" / "dataset.py"
ROUND_SCRIPT = REPO / "scripts" / "run_wuhu_round.sh"
EVAL_SCRIPT = REPO / "scripts" / "evaluate_wuhu_round.sh"
MAPPER_BINARY = REPO / "install" / "fast_livo" / "lib" / "fast_livo" / "fastlivo_mapping"
PY = "/usr/bin/python3"
ALLOWED_ROOTS = [pathlib.Path("/home/project"), REPO]
DEFAULT_DATASET = "/home/project/data/haibo/wuhu_livo/ros2bag"
# processes a mapping round may leave behind; used by the emergency stop
ROS_PROCESS_PATTERN = "fastlivo_mapping|livo/player.py|livo.player|ros2 bag play|wuhu_truck29.launch|image_transport/republish"

DEFAULT_STATE = {
    "vehicle": {"mine": "wuhu", "truck": "truck29"},
    "data": {"dataset": DEFAULT_DATASET, "recent": [DEFAULT_DATASET]},
    "calibration": {"source_file": str(CAL_ROOT / "perception_truck29.json"), "parent_id": "", "run": "",
                    "lidar_offset": -0.125, "camera_offset": -0.115, "auto_activate": True, "fast": True},
    "mapping": {"round_prefix": "studio", "mode": "livo", "use_rtk": True, "save_map": True, "rviz": False,
                "rate": 1.0, "calibration_id": "", "precheck": True, "frame_divider": 1, "evaluate": True,
                "extra_sets": "", "start_offset": 0, "duration": ""},
    "ui": {"page": "home"},
}

_state_lock = threading.Lock()


def now():
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def load_state():
    state = json.loads(json.dumps(DEFAULT_STATE))
    if STATE_FILE.exists():
        try:
            saved = json.loads(STATE_FILE.read_text())
        except Exception:  # noqa: BLE001
            saved = {}
        for k, v in saved.items():
            if isinstance(v, dict) and isinstance(state.get(k), dict):
                state[k].update(v)
            else:
                state[k] = v
        # migrate the pre-v2 layout (prepared bags, per-section bag paths)
        data = state["data"]
        if not data.get("dataset") or "results/prepared" in str(data.get("dataset")):
            data["dataset"] = saved.get("data", {}).get("bag_root") or DEFAULT_DATASET
        for legacy in ("bag_root", "prepared_output", "lio_only"):
            data.pop(legacy, None)
        state["mapping"].pop("bag", None)
        for legacy in ("bag", "mine", "truck"):
            if legacy in state["calibration"]:
                if legacy in ("mine", "truck") and state["calibration"][legacy]:
                    state["vehicle"][legacy] = state["calibration"][legacy]
                state["calibration"].pop(legacy)
        recent = [p for p in data.get("recent", []) if p and "results/prepared" not in p]
        if data["dataset"] not in recent:
            recent.insert(0, data["dataset"])
        data["recent"] = recent[:12]
    return state


def save_state(state):
    CAL_ROOT.mkdir(parents=True, exist_ok=True)
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(STATE_FILE)


def update_state(patch):
    with _state_lock:
        state = load_state()
        for k, v in (patch or {}).items():
            if isinstance(v, dict) and isinstance(state.get(k), dict):
                state[k].update(v)
            else:
                state[k] = v
        save_state(state)
        return state


def remember_dataset(path):
    with _state_lock:
        state = load_state()
        recent = [p for p in state["data"].get("recent", []) if p != path]
        recent.insert(0, path)
        state["data"]["recent"] = recent[:12]
        state["data"]["dataset"] = path
        save_state(state)
        return state


def safe_path(raw):
    p = pathlib.Path(str(raw)).expanduser().resolve()
    if not any(p == r or r in p.parents for r in ALLOWED_ROOTS):
        raise PermissionError(f"path outside allowed roots: {p}")
    return p


def read_tail(path, max_bytes=200_000):
    try:
        path = pathlib.Path(path)
        size = path.stat().st_size
        with path.open("rb") as f:
            if size > max_bytes:
                f.seek(size - max_bytes)
            return f.read().decode("utf-8", "replace")
    except (FileNotFoundError, NotADirectoryError):
        return ""


def read_json(path, default=None):
    try:
        return json.loads(pathlib.Path(path).read_text())
    except Exception:  # noqa: BLE001
        return default


def parse_kv_log(path):
    out = {}
    for line in read_tail(path, 20_000).splitlines():
        if "=" in line and not line.startswith("[") and not line.startswith("dirty"):
            k, _, v = line.partition("=")
            k, v = k.strip(), v.strip()
            if k == "mode":  # "mode=livo use_rtk=true save_map=true rviz=false rate=1.0 ..."
                parts = v.split()
                v = parts[0]
                for extra in parts[1:]:
                    k2, _, v2 = extra.partition("=")
                    out[k2] = v2
            out[k] = v
    return out


def csv_rows(path, max_points=400, max_bytes=20_000_000):
    """Header + evenly subsampled data rows (always including the last row)."""
    path = pathlib.Path(path)
    text = read_tail(path, max_bytes)
    lines = [l for l in text.splitlines() if l and not l.startswith("#")]
    if not lines:
        return [], []
    header = None
    try:
        with path.open() as f:
            for line in f:
                if not line.startswith("#"):
                    header = line.strip().split(",")
                    break
    except OSError:
        return [], []
    if header is None:
        return [], []
    data = [l for l in lines if not l.startswith(header[0] + ",")]
    if len(data) > max_points:
        step = len(data) / max_points
        picked = [data[int(i * step)] for i in range(max_points)]
        if picked[-1] != data[-1]:
            picked.append(data[-1])
        data = picked
    rows = [l.split(",") for l in data]
    return header, [r for r in rows if len(r) == len(header)]


def count_lines(path):
    try:
        with pathlib.Path(path).open("rb") as f:
            return sum(1 for _ in f)
    except (FileNotFoundError, NotADirectoryError):
        return 0


def pgrep(pattern):
    return subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0


def run_tool(cmd, timeout=120):
    """Run a repository tool synchronously and return (returncode, stdout+stderr, last JSON line or None)."""
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=timeout)
    text = (proc.stdout or "") + (proc.stderr or "")
    payload = None
    for line in reversed((proc.stdout or "").strip().splitlines()):
        line = line.strip()
        if line.startswith("{") and line.endswith("}"):
            try:
                payload = json.loads(line)
                break
            except json.JSONDecodeError:
                continue
    return proc.returncode, text, payload


def clean(obj):
    """Replace NaN/Inf (invalid in JSON, breaks the browser's JSON.parse) with null, recursively."""
    if isinstance(obj, float):
        return obj if obj == obj and obj not in (float("inf"), float("-inf")) else None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


class StudioError(Exception):
    """User-facing error (message shown in the UI as-is)."""
