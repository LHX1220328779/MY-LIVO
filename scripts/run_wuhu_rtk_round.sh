#!/usr/bin/env bash
# One complete, reproducible Wuhu LIO+RTK round inside the calibration container:
#   launch mapper -> ordered 1x playback -> drain tail -> SIGINT the mapping node
#   (it optimizes, exports trajectories + every map_*.pcd) -> stop launcher -> evaluate.
# Everything (logs, PCDs, evaluation) is written under results/<round>/ only.
#
# Usage (inside container):  bash scripts/run_wuhu_rtk_round.sh <round_name> [--no-eval]
set -uo pipefail
repo=/home/project/MY-LIVO
round=${1:?round name required, e.g. wuhu_rtk_91_ins_attitude}
do_eval=1; [[ "${2:-}" == "--no-eval" ]] && do_eval=0
out="$repo/results/$round"
mkdir -p "$out"
cd "$repo"
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u; source /opt/ros/humble/setup.bash; source install/fast_livo/share/fast_livo/local_setup.bash; set -u
export ROS_LOG_DIR="$out/roslog"; mkdir -p "$ROS_LOG_DIR"
export FASTDDS_BUILTIN_TRANSPORTS=SHM
export OPENBLAS_NUM_THREADS=1

# Refuse to start on top of a stale mapper/player from an earlier round.
if pgrep -f 'fastlivo_mapping|ros2 bag play|wuhu_truck29.launch' >/dev/null; then
  echo "stale ROS processes present; killing them" | tee -a "$out/run.log"
  pkill -INT -f 'ros2 bag play' || true; sleep 1
  pkill -TERM -f 'fastlivo_mapping' || true; sleep 2
  pkill -TERM -f 'wuhu_truck29.launch' || true; sleep 2
  pkill -KILL -f 'fastlivo_mapping|ros2 bag play|wuhu_truck29.launch' || true
fi

{
echo "round=$round"; echo "start=$(date -u +%FT%TZ)"; echo "commit=$(git rev-parse --short HEAD 2>/dev/null)"; echo "branch=$(git branch --show-current 2>/dev/null)"
git status --short -- src include config launch scripts CMakeLists.txt 2>/dev/null | sed 's/^/dirty: /'
} > "$out/run.log"

ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=false use_rtk:=true use_rviz:=false \
  rear_axle_to_imu:=true save_map:=true output_directory:="$out" > "$out/launch.log" 2>&1 &
launch_pid=$!
for i in $(seq 1 60); do
  grep -q "RTK backend enabled" "$out/launch.log" 2>/dev/null && break
  sleep 0.5
done
sleep 3
mapper_pid=$(pgrep -f "fastlivo_mapping" | head -1)
echo "launch_pid=$launch_pid mapper_pid=${mapper_pid:-none}" >> "$out/run.log"
if [[ -z "${mapper_pid:-}" ]]; then echo "mapper did not start" | tee -a "$out/run.log"; kill -INT "$launch_pid" 2>/dev/null; exit 2; fi

bash scripts/play_all_bags.sh --lio-only --rate 1.0 > "$out/playback.log" 2>&1
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
kill -INT "$launch_pid" 2>/dev/null; wait "$launch_pid" 2>/dev/null
echo "launch_exit=$?" >> "$out/run.log"
grep -E "RTK finalized|Final keyframe|INS attitude factors|position budget|Mapping failed|RTK backend failed" "$out/launch.log" >> "$out/run.log"
ls "$fusion_dir"/map_*.pcd 2>/dev/null | wc -l | sed 's/^/map_chunks=/' >> "$out/run.log"
echo "fusion_dir=$fusion_dir" >> "$out/run.log"

if [[ $do_eval -eq 1 && -f "$fusion_dir/trajectory_optimized.txt" ]]; then
  bash scripts/evaluate_wuhu_round.sh "$round" >> "$out/run.log" 2>&1
fi
chown -R 1000:1000 "$out" 2>/dev/null || true
echo "end=$(date -u +%FT%TZ)" >> "$out/run.log"
tail -25 "$out/run.log"

