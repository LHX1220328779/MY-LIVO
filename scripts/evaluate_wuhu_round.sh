#!/usr/bin/env bash
# Standard evaluation bundle for one round: rear-axle/IMU trajectory budget + startup
# analysis + same-point map thickness, pole chunk pairs, native 20 m windows, INS attitude /
# per-window drift summary, and a machine-readable evaluation/metrics.json for the console.
# Usage (inside container): bash scripts/evaluate_wuhu_round.sh <round_name> [--dataset DIR] [--quick]
#   --quick   trajectory + summary + metrics only (no pole pairs / native windows / map_*.pcd scan);
#             used by calibration pipelines for closed-loop A/B of the native trajectory (~30 s).
# The INS/RTK reference is read directly from the recording the round was played from
# (round.json "dataset", or --dataset) and cached per round in evaluation/reference_csv/;
# raw split recordings need no conversion and results/prepared is not used.
set -uo pipefail
repo=/home/project/MY-LIVO
round=${1:?round name required}; shift
dataset=""; quick=0
while (($#)); do
  case "$1" in
    --dataset|--bag) dataset="$2"; shift 2 ;;
    --quick) quick=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
out="$repo/results/$round"
cd "$repo"
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_LOG_DIR="$out/roslog"; mkdir -p "$ROS_LOG_DIR"
py=/usr/bin/python3
if [[ -z "$dataset" && -f "$out/round.json" ]]; then
  dataset=$($py -c "import json,sys; print(json.load(open(sys.argv[1])).get('dataset') or '')" "$out/round.json")
fi
if [[ -z "$dataset" && -f "$out/run.log" ]]; then
  dataset=$(grep -m1 '^dataset=' "$out/run.log" | cut -d= -f2-)
fi
if [[ -z "$dataset" || ! -d "$dataset" ]]; then
  for cand in /home/project/data/haibo/wuhu_livo/ros2bag results/prepared/wuhu_livo; do
    [[ -d "$cand" ]] && { dataset="$cand"; break; }
  done
fi
fusion_dir=$(ls -dt "$out"/Log/fusion/*/ 2>/dev/null | head -1); fusion_dir=${fusion_dir%/}
if [[ -z "$fusion_dir" || ! -f "$fusion_dir/trajectory_lio.txt" ]]; then
  echo "evaluation skipped: no fusion output under $out/Log/fusion" >&2; exit 3
fi
eval_dir="$out/evaluation"; mkdir -p "$eval_dir"
# Native runs normally retain scan/pose archives without exporting PCD. Only
# assess saved-map geometry when its actual chunks exist; never export them here.
map_pcd=0
[[ -f "$fusion_dir/map_0.pcd" ]] && map_pcd=1
geometry_reason=""
if (( quick )); then geometry_reason="quick evaluation requested"
elif (( ! map_pcd )); then geometry_reason="native map PCD chunks are absent; trajectory assessed, map thickness and lamp-pole geometry not assessed"
fi
echo "${geometry_reason:-actual native map PCD chunks available}" > "$eval_dir/geometry_status.txt"
$py -c 'import json,sys; from pathlib import Path; json.dump({"map_pcd_present":sys.argv[2]=="1", "geometry_requested":not bool(sys.argv[3]), "reason":sys.argv[3] or "actual native map PCD chunks available"},open(sys.argv[1],"w"),indent=2)' \
  "$eval_dir/geometry_status.json" "$map_pcd" "$geometry_reason"
[[ -n "$geometry_reason" ]] && echo "geometry evaluation skipped: $geometry_reason"
# INS time association must match the run (rtk.time_offset_sec in parameters.txt).
toff=$(grep -m1 '^rtk.time_offset_sec=' "$fusion_dir/parameters.txt" | cut -d= -f2)
toff=${toff:--0.1}
echo "evaluation start $(date -u +%FT%TZ) run=$fusion_dir dataset=$dataset time_offset=$toff quick=$quick"

# INS/RTK reference straight from the recording (cached once per round).
ref_dir="$eval_dir/reference_csv"
ref_pid=""
if [[ ! -s "$ref_dir/ins_odometry.csv" ]]; then
  # only the round's time span of the recording (calibration rounds are 1-4 min of a 15 min recording)
  span=$(awk 'NR==1{a=$1} {b=$1} END{if (NR) printf "%.3f %.3f", a, b}' "$fusion_dir/trajectory_lio.txt")
  # Runs next to the trajectory evaluation (both only read the recording, ~1 min each); the summary waits for it.
  ( $py scripts/dump_bag_reference.py --bag "$dataset" --output "$ref_dir" ${span:+--span $span} > "$eval_dir/reference.log" 2>&1
    echo "reference dump exit=$?" ) &
  ref_pid=$!
fi

map_args=()
(( map_pcd && ! quick )) && map_args=(--map-run "$fusion_dir")
$py scripts/evaluate_wuhu_lio.py --bag "$dataset" --rear-axle-to-imu --time-offset-sec "$toff" \
  --trajectory "$fusion_dir/trajectory_lio.txt" "$fusion_dir/trajectory_optimized.txt" \
  "${map_args[@]}" --analysis-output "$eval_dir/trajectory" > "$eval_dir/trajectory.log" 2>&1
echo "trajectory eval exit=$?"

if (( ! quick && map_pcd )); then
  for pair in "0 10" "5 10" "0 5"; do
    set -- $pair
    if [[ ! -f "$fusion_dir/map_$1.pcd" || ! -f "$fusion_dir/map_$2.pcd" ]]; then
      echo "pole $1 vs $2 skipped: requested PCD chunks are absent" | tee "$eval_dir/pole_${1}_${2}.log"
      continue
    fi
    $py scripts/evaluate_wuhu_lio.py --pole-pair-run "$fusion_dir" --pole-chunks "$1" "$2" \
      --analysis-output "$eval_dir/pole_${1}_${2}" > "$eval_dir/pole_${1}_${2}.log" 2>&1
    echo "pole $1 vs $2 exit=$?"
  done
  if [[ -d results/lio_reference_01 ]]; then
    $py scripts/evaluate_wuhu_lio.py --native-reference results/lio_reference_01 --map-run "$fusion_dir" \
      --analysis-output "$eval_dir/native_windows" > "$eval_dir/native_windows.log" 2>&1
    echo "native windows exit=$?"
  fi
else
  for pair in "0 10" "5 10" "0 5"; do
    set -- $pair
    echo "not assessed: $geometry_reason" > "$eval_dir/pole_${1}_${2}.log"
  done
  echo "not assessed: $geometry_reason" > "$eval_dir/native_windows.log"
fi

[[ -n "$ref_pid" ]] && wait "$ref_pid"
# Attitude/INS consistency, per-window drift, startup, correction smoothness, pole multi-epoch summary.
summary_opts=(--reference "$ref_dir/ins_odometry.csv")
(( quick || ! map_pcd )) && summary_opts+=(--no-pole)
$py scripts/analysis/round_summary.py "$fusion_dir" "$eval_dir" "${summary_opts[@]}" > "$eval_dir/summary.txt" 2>&1
echo "summary exit=$?"

# Machine-readable metrics for the console / comparisons (evaluation/metrics.json).
$py scripts/analysis/round_metrics.py "$out" > "$eval_dir/metrics.log" 2>&1
echo "metrics exit=$?"
echo "evaluation end $(date -u +%FT%TZ)"
