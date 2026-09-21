#!/usr/bin/env bash
# One complete, reproducible Wuhu round inside the calibration container (LIO or LIVO):
#   launch mapper -> ordered 1x playback -> drain tail -> SIGINT the mapping node
#   (it optimizes, exports trajectories + every map_*.pcd) -> stop launcher -> evaluate.
# Everything (logs, PCDs, evaluation, effective config) is written under results/<round>/ only.
#
# Usage (inside container):
#   bash scripts/run_wuhu_round.sh <round_name> [--mode lio|livo] [--no-rtk] [--no-save-map] [--rviz]
#        [--rate R] [--start-offset S] [--set laser_mapping.key.subkey=value ...] [--no-eval]
#        [--bag <prepared bag dir>] [--calibration <calibration.json>]
# Examples:
#   bash scripts/run_wuhu_round.sh wuhu_livo_100_baseline --mode livo
#   bash scripts/run_wuhu_round.sh wuhu_livo_101_img_offset --mode livo --set time_offset.img_time_offset=-0.115
set -uo pipefail
repo=/home/project/MY-LIVO
round=${1:?round name required, e.g. wuhu_rtk_91_ins_attitude}; shift
mode=lio; use_rtk=true; save_map=true; use_rviz=false; rate=1.0; start_offset=0; do_eval=1; sets=(); bag=""; calibration=""
while (($#)); do
  case "$1" in
    --mode) mode="$2"; shift 2 ;;
    --no-rtk) use_rtk=false; shift ;;
    --no-save-map) save_map=false; shift ;;
    --rviz) use_rviz=true; shift ;;
    --rate) rate="$2"; shift 2 ;;
    --start-offset) start_offset="$2"; shift 2 ;;
    --set) sets+=("$2"); shift 2 ;;
    --no-eval) do_eval=0; shift ;;
    --bag) bag="$2"; shift 2 ;;
    --calibration) calibration="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ -n "$calibration" ]]; then
  [[ -f "$calibration" ]] || { echo "calibration file not found: $calibration" >&2; exit 2; }
  sets+=("multi_lidar.calibration_file=$calibration")
fi
if [[ -n "$bag" && ! -f "$bag/metadata.yaml" ]]; then echo "prepared bag not found: $bag" >&2; exit 2; fi
case "$mode" in lio|livo) ;; *) echo "--mode must be lio or livo" >&2; exit 2 ;; esac
out="$repo/results/$round"
mkdir -p "$out"
cd "$repo"
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u; source /opt/ros/humble/setup.bash; source install/fast_livo/share/fast_livo/local_setup.bash; set -u
export ROS_LOG_DIR="$out/roslog"; mkdir -p "$ROS_LOG_DIR"
export FASTDDS_BUILTIN_TRANSPORTS=SHM
export OPENBLAS_NUM_THREADS=1

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
if pgrep -f 'fastlivo_mapping|ros2 bag play|wuhu_truck29.launch' >/dev/null; then
  echo "stale ROS processes present; killing them" | tee -a "$out/run.log"
  pkill -INT -f 'ros2 bag play' || true; sleep 1
  pkill -TERM -f 'fastlivo_mapping' || true; sleep 2
  pkill -TERM -f 'wuhu_truck29.launch' || true; sleep 2
  pkill -KILL -f 'fastlivo_mapping|ros2 bag play|wuhu_truck29.launch' || true
fi

{
echo "round=$round"; echo "mode=$mode use_rtk=$use_rtk save_map=$save_map rviz=$use_rviz rate=$rate start_offset=$start_offset"
echo "overrides=${sets[*]:-none}"
echo "bag=${bag:-default}"; echo "calibration=${calibration:-yaml}"
echo "start=$(date -u +%FT%TZ)"; echo "commit=$(git rev-parse --short HEAD 2>/dev/null)"; echo "branch=$(git branch --show-current 2>/dev/null)"
git status --short -- src include config launch scripts CMakeLists.txt 2>/dev/null | sed 's/^/dirty: /'
} > "$out/run.log"

use_camera=false; [[ "$mode" == "livo" ]] && use_camera=true
ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=$use_camera use_rtk:=$use_rtk use_rviz:=$use_rviz \
  rear_axle_to_imu:=true save_map:=$save_map output_directory:="$out" config_file:="$config" > "$out/launch.log" 2>&1 &
launch_pid=$!
for i in $(seq 1 60); do
  grep -q -E "RTK backend enabled|Loaded [0-9]+ LiDAR source" "$out/launch.log" 2>/dev/null && break
  sleep 0.5
done
sleep 3
mapper_pid=$(pgrep -f "fastlivo_mapping" | head -1)
echo "launch_pid=$launch_pid mapper_pid=${mapper_pid:-none}" >> "$out/run.log"
if [[ -z "${mapper_pid:-}" ]]; then echo "mapper did not start" | tee -a "$out/run.log"; kill -INT "$launch_pid" 2>/dev/null; exit 2; fi

if [[ "$mode" == "livo" ]]; then play_args=(--with-camera); else play_args=(--lio-only); fi
[[ -n "$bag" ]] && play_args+=(--prepared-bag "$bag")
bash scripts/play_all_bags.sh "${play_args[@]}" --rate "$rate" --start-offset "$start_offset" > "$out/playback.log" 2>&1
echo "playback_exit=$? at $(date -u +%FT%TZ)" >> "$out/run.log"

# Drain: wait until the fusion frame index stops growing (tail caught up).
fusion_dir=$(ls -dt "$out"/Log/fusion/*/ 2>/dev/null | head -1)
last=-1; stable=0
for i in $(seq 1 120); do
  n=$(wc -l < "$fusion_dir/frames.csv" 2>/dev/null || echo 0)
  if [[ "$n" == "$last" ]]; then stable=$((stable+1)); else stable=0; fi
  last=$n
  [[ $stable -ge 5 ]] && break
  sleep 1
done
echo "frames_csv_lines=$last drained at $(date -u +%FT%TZ)" >> "$out/run.log"

# Graceful stop: the node handles SIGINT itself (finish accepted scans, final LM, export).
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
ls "$fusion_dir"/map_*.pcd 2>/dev/null | wc -l | sed 's/^/map_chunks=/' >> "$out/run.log"
echo "fusion_dir=$fusion_dir" >> "$out/run.log"

if [[ $do_eval -eq 1 && -n "${fusion_dir:-}" && -f "$fusion_dir/trajectory_optimized.txt" ]]; then
  bash scripts/evaluate_wuhu_round.sh "$round" >> "$out/run.log" 2>&1
fi
chown -R 1000:1000 "$out" 2>/dev/null || true
echo "end=$(date -u +%FT%TZ)" >> "$out/run.log"
tail -25 "$out/run.log"
