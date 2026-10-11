#!/usr/bin/env bash
# truck29 non-regression guard (run inside the container, nothing else running):
#   bash scripts/check_truck29_guard.sh [--skip-rounds <lio_round> <livo_round>] [--calibration FILE]
#
# Runs one LIO and one LIVO round on the truck29 dataset with the ACTIVE truck29 calibration
# (config/carlibration/registry.json) and checks the delivered map and trajectory against the
# accepted truck29 baseline (docs/mcp-colour-only-and-time-offset-audit.md section 12). The
# pipeline is not bit-reproducible run to run (real-time playback, threaded back end: LIO xy max
# 6.03 / 6.71 / 6.38 cm in three identical runs), so the thresholds carry that band plus margin.
# Any FAIL means the change must not be merged / the calibration must not be activated.
set -euo pipefail
repo=/home/project/MY-LIVO; cd "$repo"
dataset=/home/project/data/haibo/wuhu_livo/ros2bag
py=/usr/bin/python3
lio=""; livo=""; calibration=""
while (($#)); do
  case "$1" in
    --skip-rounds) lio="$2"; livo="$3"; shift 3 ;;
    --calibration) calibration="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ -z "$calibration" ]]; then
  calibration=$($py tools/calib/livo_calib.py export --id "$($py - <<'PY'
import json; print(json.load(open("config/carlibration/registry.json"))["active"]["wuhu/truck29"])
PY
)")
fi
echo "guard calibration: $calibration"
stamp=$(date +%Y%m%d_%H%M%S)
if [[ -z "$lio" ]]; then
  lio="guard_t29_lio_$stamp"; livo="guard_t29_livo_$stamp"
  bash scripts/run_wuhu_round.sh "$lio" --mode lio --dataset "$dataset" --calibration "$calibration" > /dev/null
  bash scripts/run_wuhu_round.sh "$livo" --mode livo --dataset "$dataset" --calibration "$calibration" > /dev/null
fi
fail=0
check() {  # name value op limit
  local ok
  if [[ -z "$2" ]]; then printf '  FAIL  %-58s %8s  (could not parse the metric)\n' "$1" "n/a"; fail=1; return; fi
  ok=$($py -c "import sys; print(int(float('$2') $3 float('$4')))")
  if [[ "$ok" == "1" ]]; then printf '  PASS  %-58s %8s  (limit %s %s)\n' "$1" "$2" "$3" "$4"
  else printf '  FAIL  %-58s %8s  (limit %s %s)\n' "$1" "$2" "$3" "$4"; fail=1; fi
}
for pair in "$lio:7.5" "$livo:7.0"; do
  round=${pair%%:*}; maxlim=${pair##*:}
  read -r med p95 mx < <($py - "$round" <<'PY'
import json, sys
m = json.load(open(f"results/{sys.argv[1]}/evaluation/metrics.json"))["trajectory"]["optimized"]["xy"]
print(round(m["median_m"] * 100, 2), round(m["p95_m"] * 100, 2), round(m["max_m"] * 100, 2))
PY
)
  echo "== $round"
  check "fused xy median [cm]" "$med" "<=" 1.0
  check "fused xy P95 [cm]" "$p95" "<=" 3.5
  check "fused xy max [cm]" "$mx" "<=" "$maxlim"
done
fusion=results/$lio/Log/fusion/$(ls "results/$lio/Log/fusion" | tail -1)
echo "== map (LIO round $lio)"
# lamp-pole shell residual |r-R| P90 of the delivered PCDs (poles 1/3/4/5, 2.5-3.5 m band). With the 45 m product
# cap the baseline is 5.5/5.3/4.9/5.1 cm (30 m cap: 4.6/4.6/3.9/4.1); limit 6.0 leaves the run-to-run band.
shell_lines=$($py scripts/analysis/pcd_shell.py "results/$lio" /tmp 2.5 3.5 2>/dev/null | grep -E "^pole [1345]:" || true)
for pole in 1 3 4 5; do
  p90=$(echo "$shell_lines" | grep "^pole $pole:" | sed 's/.*P50\/P90\/P99 *[0-9.]*\/ *\([0-9.]*\)\/.*/\1/' || true)
  check "pole $pole shell |r-R| P90 [cm]" "$p90" "<=" 6.0
done
# startup lamp-pole ghost (view-independent shell metric, fused trajectory, 0-30 s): spread of the 2 s epoch
# offsets. Native 8.4 cm; v069 fused 11.1 cm (the >=23 cm ghost seen in the viewer); v071 fused 6.0 cm.
spread=$($py scripts/analysis/startup_pole_shell.py "$fusion" --traj trajectory_optimized.txt 2>/dev/null | grep "^SHELL" | sed 's/.*(max-min) *\([0-9.]*\) cm.*/\1/' || true)
check "startup pole epoch-offset spread, fused [cm]" "$spread" "<=" 9.0
# back-LiDAR along-track offset (cruise bin) and wall inter-source offset front_right vs back
along=$($py scripts/analysis/along_track.py "results/$lio" 0 118 25 0.4 2>/dev/null | grep -A3 "^back_lidar" | grep "speed 5.5-8.0" | sed 's/.*median *\([-+0-9.]*\) cm.*/\1/' || true)
check "back LiDAR along-track offset, cruise, |median| [cm]" "${along#-}" "<=" 3.0
wall=$($py scripts/analysis/wall_ghost.py "$fusion" 20 118 --fused --max-range 25 --planarity 2 2>/dev/null | grep "front_right_lidar vs back_lidar" | sed 's/.*|off| median *\([0-9.]*\) .*/\1/' || true)
check "wall offset front_right vs back, median [cm]" "$wall" "<=" 3.0
if (( fail )); then echo "GUARD FAIL: truck29 regressed - do not merge / do not activate"; exit 1; fi
echo "GUARD PASS ($lio, $livo)"
