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
# INS time association must match the run (rtk.time_offset_sec in parameters.txt).
toff=$(grep -m1 '^rtk.time_offset_sec=' "$fusion_dir/parameters.txt" | cut -d= -f2)
toff=${toff:--0.1}
echo "evaluation start $(date -u +%FT%TZ) run=$fusion_dir dataset=$dataset time_offset=$toff quick=$quick"

# INS/RTK reference straight from the recording (cached once per round).
ref_dir="$eval_dir/reference_csv"
if [[ ! -s "$ref_dir/ins_odometry.csv" ]]; then
  $py scripts/dump_bag_reference.py --bag "$dataset" --output "$ref_dir" > "$eval_dir/reference.log" 2>&1
  echo "reference dump exit=$?"
fi

$py scripts/evaluate_wuhu_lio.py --bag "$dataset" --rear-axle-to-imu --time-offset-sec "$toff" \
  --trajectory "$fusion_dir/trajectory_lio.txt" "$fusion_dir/trajectory_optimized.txt" \
  --map-run "$fusion_dir" --analysis-output "$eval_dir/trajectory" > "$eval_dir/trajectory.log" 2>&1
echo "trajectory eval exit=$?"

if (( ! quick )); then
  for pair in "0 10" "5 10" "0 5"; do
    set -- $pair
    $py scripts/evaluate_wuhu_lio.py --pole-pair-run "$fusion_dir" --pole-chunks "$1" "$2" \
      --analysis-output "$eval_dir/pole_${1}_${2}" > "$eval_dir/pole_${1}_${2}.log" 2>&1
    echo "pole $1 vs $2 exit=$?"
  done
  if [[ -d results/lio_reference_01 ]]; then
    $py scripts/evaluate_wuhu_lio.py --native-reference results/lio_reference_01 --map-run "$fusion_dir" \
      --analysis-output "$eval_dir/native_windows" > "$eval_dir/native_windows.log" 2>&1
    echo "native windows exit=$?"
  fi
fi

# Attitude/INS consistency, per-window drift, startup, correction smoothness, pole multi-epoch summary.
summary_opts=(--reference "$ref_dir/ins_odometry.csv")
(( quick )) && summary_opts+=(--no-pole)
$py scripts/analysis/round_summary.py "$fusion_dir" "$eval_dir" "${summary_opts[@]}" > "$eval_dir/summary.txt" 2>&1
echo "summary exit=$?"

# Machine-readable metrics for the console / comparisons (evaluation/metrics.json).
$py scripts/analysis/round_metrics.py "$out" > "$eval_dir/metrics.log" 2>&1
echo "metrics exit=$?"
echo "evaluation end $(date -u +%FT%TZ)"
