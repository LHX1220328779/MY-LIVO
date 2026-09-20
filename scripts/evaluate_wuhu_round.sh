#!/usr/bin/env bash
# Standard evaluation bundle for one round: rear-axle/IMU trajectory budget + startup
# analysis + same-point map thickness, pole chunk pairs, native 20 m windows.
# Usage (inside container): bash scripts/evaluate_wuhu_round.sh <round_name>
set -uo pipefail
repo=/home/project/MY-LIVO
round=${1:?round name required}
out="$repo/results/$round"
cd "$repo"
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_LOG_DIR="$out/roslog"; mkdir -p "$ROS_LOG_DIR"
fusion_dir=$(ls -dt "$out"/Log/fusion/*/ 2>/dev/null | head -1); fusion_dir=${fusion_dir%/}
eval_dir="$out/evaluation"; mkdir -p "$eval_dir"
py=/usr/bin/python3
# INS time association must match the run (rtk.time_offset_sec in parameters.txt).
toff=$(grep -m1 '^rtk.time_offset_sec=' "$fusion_dir/parameters.txt" | cut -d= -f2)
toff=${toff:--0.1}
echo "evaluation start $(date -u +%FT%TZ) run=$fusion_dir time_offset=$toff"

$py scripts/evaluate_wuhu_lio.py --bag results/prepared/wuhu_livo --rear-axle-to-imu --time-offset-sec "$toff" \
  --trajectory "$fusion_dir/trajectory_lio.txt" "$fusion_dir/trajectory_optimized.txt" \
  --map-run "$fusion_dir" --analysis-output "$eval_dir/trajectory" > "$eval_dir/trajectory.log" 2>&1
echo "trajectory eval exit=$?"

for pair in "0 10" "5 10" "0 5"; do
  set -- $pair
  $py scripts/evaluate_wuhu_lio.py --pole-pair-run "$fusion_dir" --pole-chunks "$1" "$2" \
    --analysis-output "$eval_dir/pole_${1}_${2}" > "$eval_dir/pole_${1}_${2}.log" 2>&1
  echo "pole $1 vs $2 exit=$?"
done

$py scripts/evaluate_wuhu_lio.py --native-reference results/lio_reference_01 --map-run "$fusion_dir" \
  --analysis-output "$eval_dir/native_windows" > "$eval_dir/native_windows.log" 2>&1
echo "native windows exit=$?"

# Attitude/INS consistency and pole multi-epoch summary (agent tools, outputs stay in evaluation/).
$py scripts/analysis/round_summary.py "$fusion_dir" "$eval_dir" > "$eval_dir/summary.txt" 2>&1
echo "summary exit=$?"
echo "evaluation end $(date -u +%FT%TZ)"

