#!/usr/bin/env bash
# One complete, reproducible mapping round inside the calibration container (LIO or LIVO):
#   launch mapper -> time-ordered real-time playback straight from the raw MCAP recording ->
#   drain tail -> SIGINT the mapping node (it optimizes, exports trajectories + every map_*.pcd)
#   -> stop launcher -> evaluate.
# Everything (logs, PCDs, evaluation, effective config, round.json manifest) is written under
# results/<round>/ only. No prepared/merged bag is required: --dataset accepts the raw recording
# root (Perception_*/Camera_* folders) or any single ROS 2 bag folder.
#
# Usage (inside container):
#   bash scripts/run_wuhu_round.sh <round_name> [--mode lio|livo] [--dataset DIR] [--calibration FILE]
#        [--no-rtk] [--no-save-map] [--rviz] [--rate R] [--start-offset S] [--duration D]
#        [--map-range M] [--rgb-range M]   (saved-map product caps in metres, 0 = keep every return;
#                                           defaults come from the yaml: rtk.save_map_max_range / save_rgb_max_range)
#        [--set laser_mapping.key.subkey=value ...] [--no-eval]
#   (--bag DIR is an alias of --dataset)
# Examples:
#   bash scripts/run_wuhu_round.sh wuhu_livo_120 --mode livo --dataset /home/project/data/haibo/wuhu_livo/ros2bag
#   bash scripts/run_wuhu_round.sh quick_lio --mode lio --duration 40 --no-save-map --no-eval
set -uo pipefail
repo=/home/project/MY-LIVO
round=${1:?round name required, e.g. wuhu_rtk_91_ins_attitude}; shift
mode=lio; use_rtk=true; save_map=true; use_rviz=false; rate=1.0; start_offset=0; duration=""; do_eval=1; quick_eval=0
sets=(); dataset="/home/project/data/haibo/wuhu_livo/ros2bag"; calibration=""
while (($#)); do
  case "$1" in
    --mode) mode="$2"; shift 2 ;;
    --no-rtk) use_rtk=false; shift ;;
    --no-save-map) save_map=false; shift ;;
    --rviz) use_rviz=true; shift ;;
    --rate) rate="$2"; shift 2 ;;
    --start-offset) start_offset="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    --set) sets+=("$2"); shift 2 ;;
    --map-range) sets+=("rtk.save_map_max_range=$2"); shift 2 ;;   # geometry product cap [m], 0 = all
    --rgb-range) sets+=("rtk.save_rgb_max_range=$2"); shift 2 ;;   # colour product cap [m], 0 = all, -1 = same as geometry
    --no-eval) do_eval=0; shift ;;
    --quick-eval) quick_eval=1; shift ;;   # trajectory + summary only (scan rounds of the closed loop)
    --bag|--dataset) dataset="$2"; shift 2 ;;
    --calibration) calibration="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$mode" in lio|livo) ;; *) echo "--mode must be lio or livo" >&2; exit 2 ;; esac
if [[ -n "$calibration" ]]; then
  [[ -f "$calibration" ]] || { echo "calibration file not found: $calibration" >&2; exit 2; }
  sets+=("multi_lidar.calibration_file=$calibration")
fi
[[ -d "$dataset" ]] || { echo "dataset directory not found: $dataset" >&2; exit 2; }
out="$repo/results/$round"
mkdir -p "$out"
cd "$repo"
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u; source /opt/ros/humble/setup.bash; source install/fast_livo/share/fast_livo/local_setup.bash; set -u
export ROS_LOG_DIR="$out/roslog"; mkdir -p "$ROS_LOG_DIR"
export FASTDDS_BUILTIN_TRANSPORTS=SHM
# 128 MB SHM segment + 100 ms heartbeat for the player/mapper traffic (see config/fastdds_livo_shm.xml).
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$repo/config/fastdds_livo_shm.xml}"
export OPENBLAS_NUM_THREADS=1

# Describe and validate the dataset (raw split recording or single bag) without converting it.
if ! /usr/bin/python3 tools/livo/dataset.py "$dataset" --json > "$out/dataset.json" 2> "$out/dataset.err"; then
  echo "dataset not readable: $dataset" >&2; cat "$out/dataset.err" >&2; exit 2
fi
ready_key=lio_ready; [[ "$mode" == "livo" ]] && ready_key=livo_ready
if ! /usr/bin/python3 -c "import json,sys; d=json.load(open(sys.argv[1])); sys.exit(0 if d.get(sys.argv[2]) else 1)" "$out/dataset.json" "$ready_key"; then
  echo "dataset is not $ready_key (mode $mode): $dataset" >&2
  /usr/bin/python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print('topics:', sorted(d['topics']))" "$out/dataset.json" >&2
  exit 2
fi

# Round manifest for the Studio (phase is updated as the round progresses).
manifest="$out/round.json"
phase() {  # phase <name> [key=value ...]
  /usr/bin/python3 - "$manifest" "$@" <<'PY'
import json, sys, datetime, os
path, phase_name, *kv = sys.argv[1:]
doc = json.load(open(path)) if os.path.exists(path) else {}
doc["phase"] = phase_name
doc["updated"] = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
for item in kv:
    k, _, v = item.partition("=")
    try:
        v = json.loads(v)
    except Exception:
        pass
    doc[k] = v
json.dump(doc, open(path, "w"), indent=2, ensure_ascii=False)
PY
}
/usr/bin/python3 - "$manifest" "$round" "$mode" "$dataset" "$calibration" "$use_rtk" "$save_map" "$use_rviz" "$rate" "$start_offset" "$duration" "$do_eval" "${sets[@]}" <<'PY'
import json, sys, datetime, subprocess, os
path, round_name, mode, dataset, calibration, use_rtk, save_map, use_rviz, rate, start_offset, duration, do_eval, *sets = sys.argv[1:]
def git(*args):
    try:
        return subprocess.run(["git", *args], capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return ""
binary = "/home/project/MY-LIVO/install/fast_livo/lib/fast_livo/fastlivo_mapping"
doc = {"round": round_name, "mode": mode, "dataset": dataset, "calibration": calibration or None,
       "options": {"use_rtk": use_rtk == "true", "save_map": save_map == "true", "rviz": use_rviz == "true",
                   "rate": float(rate), "start_offset": float(start_offset),
                   "duration": float(duration) if duration else None, "evaluate": do_eval == "1"},
       "overrides": sets, "start": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
       "commit": git("rev-parse", "--short", "HEAD"), "branch": git("branch", "--show-current"),
       "binary_mtime": datetime.datetime.fromtimestamp(os.path.getmtime(binary)).isoformat(timespec="seconds") if os.path.exists(binary) else None,
       "status": "running", "phase": "starting"}
json.dump(doc, open(path, "w"), indent=2, ensure_ascii=False)
PY

# --no-rtk mapping: the exporter (frames.csv/scans.bin/trajectories/map_N.pcd) lives in the RTK
# backend, so the backend must run; rtk.observations_enabled=false starves it of INS/RTK
# observations instead -> pure-odometry graph, products in the INS-anchored mine frame,
# and the standard evaluation keeps working.
rtk_backend=false   # launch arg must never be empty (name:=<value>); false = follow use_rtk
if [[ $use_rtk == false ]]; then
  rtk_backend=true
  sets+=("rtk.observations_enabled=false")
fi

# Effective configuration: copy of the production yaml with the requested overrides applied.
config="$out/config.yaml"
/usr/bin/python3 - "$repo/config/wuhu_truck29.yaml" "$config" "${sets[@]}" <<'PY'
import sys, yaml
src, dst, *sets = sys.argv[1:]
with open(src, encoding="utf-8") as f:
    doc = yaml.safe_load(f)
params = doc["laser_mapping"]["ros__parameters"]
for item in sets:
    key, _, raw = item.partition("=")
    node = params
    parts = key.split(".")
    for p in parts[:-1]:
        node = node.setdefault(p, {})
    old = node.get(parts[-1])
    try:
        val = yaml.safe_load(raw)
    except Exception:
        val = raw
    if isinstance(old, bool) and not isinstance(val, bool):
        val = str(raw).lower() in ("1", "true", "yes")
    elif isinstance(old, float) and isinstance(val, int):
        val = float(val)
    node[parts[-1]] = val
    print(f"override {key}: {old} -> {val}")
with open(dst, "w", encoding="utf-8") as f:
    yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
PY

# Refuse to start on top of a stale mapper/player from an earlier round.
stale='fastlivo_mapping|livo/player.py|livo.player|ros2 bag play|wuhu_truck29.launch'
if pgrep -f "$stale" >/dev/null; then
  echo "stale ROS processes present; killing them" | tee -a "$out/run.log"
  pkill -INT -f 'livo/player.py|livo.player|ros2 bag play' || true; sleep 1
  pkill -TERM -f 'fastlivo_mapping' || true; sleep 2
  pkill -TERM -f 'wuhu_truck29.launch' || true; sleep 2
  pkill -KILL -f "$stale" || true
fi

{
echo "round=$round"; echo "mode=$mode use_rtk=$use_rtk save_map=$save_map rviz=$use_rviz rate=$rate start_offset=$start_offset duration=${duration:-all}"
echo "overrides=${sets[*]:-none}"
echo "bag=$dataset"; echo "dataset=$dataset"; echo "calibration=${calibration:-yaml}"
echo "start=$(date -u +%FT%TZ)"; echo "commit=$(git rev-parse --short HEAD 2>/dev/null)"; echo "branch=$(git branch --show-current 2>/dev/null)"
git status --short -- src include config launch scripts tools CMakeLists.txt 2>/dev/null | sed 's/^/dirty: /'
} > "$out/run.log"

use_camera=false; [[ "$mode" == "livo" ]] && use_camera=true
ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=$use_camera use_rtk:=$use_rtk rtk_backend:=$rtk_backend use_rviz:=$use_rviz \
  rear_axle_to_imu:=true save_map:=$save_map output_directory:="$out" config_file:="$config" > "$out/launch.log" 2>&1 &
launch_pid=$!
for i in $(seq 1 60); do
  grep -q -E "RTK backend enabled|Loaded [0-9]+ LiDAR source" "$out/launch.log" 2>/dev/null && break
  sleep 0.5
done
sleep 3
# Use the PID reported by this launcher in its own PID namespace. /proc-based
# pgrep can return host PIDs inside a sandbox and fail to signal/export the map.
mapper_pid=$(sed -n 's/.*fastlivo_mapping.*process started with pid \[\([0-9][0-9]*\)\].*/\1/p' "$out/launch.log" | head -1)
echo "launch_pid=$launch_pid mapper_pid=${mapper_pid:-none}" >> "$out/run.log"
if [[ -z "${mapper_pid:-}" ]]; then
  echo "mapper did not start" | tee -a "$out/run.log"; phase failed status=failed error='"mapper did not start"'
  kill -INT "$launch_pid" 2>/dev/null; exit 2
fi

phase playing
play_args=("$dataset" --preset "$mode" --rate "$rate" --start-offset "$start_offset")
[[ -n "$duration" ]] && play_args+=(--duration "$duration")
/usr/bin/python3 tools/livo/player.py "${play_args[@]}" > "$out/playback.log" 2>&1 &
player_pid=$!
# A mapper that stopped (FATAL input gap, OOM kill) consumes nothing more: stop the playback instead of
# replaying the remaining minutes into nothing. Background jobs ignore SIGINT here, hence SIGTERM.
while kill -0 "$player_pid" 2>/dev/null; do
  if ! kill -0 "$mapper_pid" 2>/dev/null; then
    echo "mapper exited during playback at $(date -u +%FT%TZ); stopping the player" >> "$out/run.log"
    kill -TERM "$player_pid" 2>/dev/null; break
  fi
  sleep 2
done
wait "$player_pid"
echo "playback_exit=$? at $(date -u +%FT%TZ)" >> "$out/run.log"

# Drain: wait until the mapper has caught up with the played data (tail processed). With save_map the fusion
# frame index frames.csv grows per frame; --no-save-map rounds (closed-loop scan rounds) write nothing before
# the export, so the mapper log (per-frame timing tables) is watched instead. Watching only frames.csv
# stopped those rounds 5 s after playback while the mapper still lagged 6-12 s (truck29 LIVO: 48 of 60 s
# mapped), so scan rounds and their zero round were compared over different spans.
phase draining
fusion_dir=$(ls -dt "$out"/Log/fusion/*/ 2>/dev/null | head -1)
progress="$fusion_dir/frames.csv"
[[ -f "$progress" ]] || progress="$out/launch.log"
last=-1; stable=0
for i in $(seq 1 120); do
  n=$(stat -c %s "$progress" 2>/dev/null || echo 0)
  if [[ "$n" == "$last" ]]; then stable=$((stable+1)); else stable=0; fi
  last=$n
  [[ $stable -ge 5 ]] && break
  sleep 1
done
frames_lines=0; [[ -f "$fusion_dir/frames.csv" ]] && frames_lines=$(wc -l < "$fusion_dir/frames.csv")
echo "frames_csv_lines=$frames_lines drained at $(date -u +%FT%TZ) (watched $(basename "$progress"))" >> "$out/run.log"

# Graceful stop: the node handles SIGINT itself (finish accepted scans, final LM, export).
phase finalizing
kill -INT "$mapper_pid"
for i in $(seq 1 900); do
  kill -0 "$mapper_pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$mapper_pid" 2>/dev/null; then echo "mapper still alive after 900 s" >> "$out/run.log"; fi
echo "mapper_exit_wait_done at $(date -u +%FT%TZ)" >> "$out/run.log"
# Background jobs of a non-interactive shell inherit SIGINT=ignored, so SIGINT would be
# dropped by ros2 launch; SIGTERM is handled (launch shuts the remaining nodes down).
kill -TERM "$launch_pid" 2>/dev/null
for i in $(seq 1 30); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
if kill -0 "$launch_pid" 2>/dev/null; then pkill -KILL -f "image_transport/republish" 2>/dev/null; kill -KILL "$launch_pid" 2>/dev/null; fi
wait "$launch_pid" 2>/dev/null
echo "launch_exit=$?" >> "$out/run.log"
pkill -KILL -f "image_transport/republish" 2>/dev/null || true
grep -E "RTK finalized|Final keyframe|INS attitude factors|position budget|Mapping failed|RTK backend failed" "$out/launch.log" >> "$out/run.log"
# The node aborts on an input IMU gap (> maximum_time_gap, e.g. one dropped 100 Hz message under load) and
# exports only the prefix: keep the output for diagnosis but mark the round incomplete (exit 4).
# A FATAL stop (e.g. a backwards input stamp) or a killed mapper (OOM: exit -9) also leaves only a prefix.
mapper_error=$(grep -m1 -oE "Mapping failed: [^[:cntrl:]]*|RTK backend failed[^[:cntrl:]]*|\[FATAL\] \[[0-9.]+\] \[laser_mapping\]: [^[:cntrl:]]*|\[fastlivo_mapping[^]]*\]: process has died \[pid [0-9]+, exit code -?[0-9]+" "$out/launch.log" | head -1 | sed 's/"/\\"/g')
map_chunks=$(ls "$fusion_dir"/map_*.pcd 2>/dev/null | wc -l)
echo "map_chunks=$map_chunks" >> "$out/run.log"
echo "fusion_dir=$fusion_dir" >> "$out/run.log"
phase exported fusion_dir="\"${fusion_dir%/}\"" map_chunks="$map_chunks"

if [[ $do_eval -eq 1 && -n "${fusion_dir:-}" && -f "$fusion_dir/trajectory_optimized.txt" ]]; then
  phase evaluating
  eval_args=(--dataset "$dataset"); (( quick_eval )) && eval_args+=(--quick)
  bash scripts/evaluate_wuhu_round.sh "$round" "${eval_args[@]}" >> "$out/run.log" 2>&1
fi
rc=0
if [[ -n "${fusion_dir:-}" && -f "$fusion_dir/trajectory_optimized.txt" && -z "$mapper_error" ]]; then
  phase done status=done end="\"$(date -u +%FT%TZ)\""
elif [[ -n "${fusion_dir:-}" && -f "$fusion_dir/trajectory_optimized.txt" ]]; then
  phase done status=incomplete end="\"$(date -u +%FT%TZ)\"" error="\"$mapper_error\""
  echo "INCOMPLETE: $mapper_error" >> "$out/run.log"; rc=4
else
  phase failed status=failed end="\"$(date -u +%FT%TZ)\"" error='"no optimized trajectory exported"'; rc=3
fi
chown -R 1000:1000 "$out" 2>/dev/null || true
echo "end=$(date -u +%FT%TZ)" >> "$out/run.log"
tail -25 "$out/run.log"
exit $rc
