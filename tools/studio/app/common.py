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
                    "lidar_offset": -0.125, "camera_offset": -0.115, "auto_activate": True, "fast": True,
                    "from_factory": False},
    "mapping": {"round_prefix": "studio", "mode": "livo", "use_rtk": True, "save_map": True, "rviz": False,
                "rate": 1.0, "calibration_id": "", "precheck": True, "frame_divider": 1, "evaluate": True,
                "extra_sets": "", "start_offset": 0, "duration": "", "map_range": 45.0, "rgb_range": 60.0,
                "rgb_voxel": 0.035, "map_filter": 0.01,
                "skip_stationary": True, "stationary_min": 30, "stationary_lidar_interval": 1.0, "loop_closure": True, "map_priority": True,
                "map_export": True, "map_voxel": 0.05, "map_source": "all", "map_tile": 0, "map_keyframes": False,
                "auto_cleanup": True, "auto_cleanup_free_gb": 50, "map_min_points": 1},
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


_SERIES_CACHE = {}
SERIES_LADDER = (0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 30, 60, 120, 300, 600, 1200)


def csv_all_rows(path):
    """Header + ALL data rows of an append-only CSV, parsed incrementally (cached by path and size)."""
    path = pathlib.Path(path)
    try:
        size = path.stat().st_size
    except OSError:
        return [], []
    key = str(path)
    c = _SERIES_CACHE.get(key)
    if c is None or size < c["pos"]:
        c = {"pos": 0, "header": None, "rows": [], "partial": b""}
    if size > c["pos"]:
        try:
            with path.open("rb") as f:
                f.seek(c["pos"])
                data = f.read(size - c["pos"])
        except OSError:
            return c["header"] or [], c["rows"]
        c["pos"] += len(data)
        lines = (c["partial"] + data).split(b"\n")
        c["partial"] = lines.pop()
        for raw in lines:
            line = raw.decode("utf-8", "replace").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if c["header"] is None:
                c["header"] = parts
            elif parts[0] != c["header"][0] and len(parts) == len(c["header"]):
                c["rows"].append(parts)
    _SERIES_CACHE[key] = c
    return c["header"] or [], c["rows"]


def _num(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if v == v and v not in (float("inf"), float("-inf")) else None


def binned_series(path, time_col, columns, max_points=600, agg=None, rename=None):
    """Stable time-binned series for live charts: [{"t": bin centre, <column>: aggregate, ...}].

    Bins are aligned to the FIRST sample and their width comes from a fixed ladder (0.1 s ... 20 min), so a
    completed bin never changes between polls; only the newest bin is still filling. The previous index
    subsampling (csv_rows: every len/400-th row) changed its stride with every new row, so the plotted history
    re-sampled a different subset of the noisy per-frame values at each refresh and appeared to keep changing.
    agg: {column: "max" | "mean" | "last"} (default "max": an error curve must keep its peaks);
    rename: {column: output key}."""
    header, rows = csv_all_rows(path)
    if not header or not rows or time_col not in header:
        return []
    col = {n: i for i, n in enumerate(header)}
    ti = col[time_col]
    idx = [(c, col[c], (agg or {}).get(c, "max"), (rename or {}).get(c, c)) for c in columns if c in col]
    t0 = next((v for v in (_num(r[ti]) for r in rows) if v is not None), None)
    t1 = next((v for v in (_num(r[ti]) for r in reversed(rows)) if v is not None), None)
    if t0 is None or t1 is None:
        return []
    width = next((s for s in SERIES_LADDER if (t1 - t0) / s <= max_points), SERIES_LADDER[-1])
    bins, order = {}, []
    for r in rows:
        t = _num(r[ti])
        if t is None:
            continue
        k = int((t - t0) // width)
        b = bins.get(k)
        if b is None:
            b = bins[k] = {}
            order.append(k)
        for name, i, mode, _out in idx:
            v = _num(r[i])
            if v is None:
                continue
            if mode == "mean":
                s = b.setdefault(name, [0.0, 0])
                s[0] += v
                s[1] += 1
            elif mode == "last" or name not in b:
                b[name] = v
            else:
                b[name] = max(b[name], v)
    out = []
    for k in order:
        b = bins[k]
        p = {"t": t0 + (k + 0.5) * width}
        for name, _i, mode, outkey in idx:
            v = b.get(name)
            p[outkey] = (v[0] / v[1] if v and v[1] else None) if mode == "mean" else v
        out.append(p)
    return out


def csv_last(path):
    """{column: value} of the newest row of an append-only CSV (shares the incremental cache)."""
    header, rows = csv_all_rows(path)
    return {n: rows[-1][i] for i, n in enumerate(header)} if header and rows else {}


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
