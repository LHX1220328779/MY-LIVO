#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/project/MY-LIVO2.0"
BAG_ROOT="/home/project/data/haibo/wuhu_livo/ros2bag"
PREPARED_BAG="/tmp/wuhu_truck29_lio_runtime"
PLAY_RATE="1.0"

usage() {
  echo "Usage: $0 [--prepared-bag DIR] [--rate RATE]"
}

while (($#)); do
  case "$1" in
    --prepared-bag) PREPARED_BAG="$2"; shift 2 ;;
    --rate) PLAY_RATE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f "$PREPARED_BAG/metadata.yaml" ]]; then
  echo "Prepared bag does not exist: $PREPARED_BAG" >&2
  exit 1
fi
if pgrep -f '[f]ast_livo/lib/fast_livo/fastlivo_mapping' >/dev/null ||
   pgrep -f '[r]os2 bag play' >/dev/null ||
   pgrep -f '[r]os2 launch fast_livo wuhu_truck29.launch.py' >/dev/null; then
  echo "Refusing to start while a mapping or bag process is active." >&2
  exit 1
fi

unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
set +u
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u
export ROS_LOG_DIR="/tmp/my_livo_ros_log"
# The managed development sandbox denies UDP sockets. Fast DDS shared memory
# keeps mapping and bag playback local and deterministic.
export FASTDDS_BUILTIN_TRANSPORTS="SHM"
mkdir -p "$ROS_LOG_DIR" "$PROJECT_ROOT/Log/regression"

# Log contains generated runtime products only.  A clean run must never mix
# keyframes, high-rate RTK rows, or a PCD from different executions.
find "$PROJECT_ROOT/Log" -type f -delete
mkdir -p "$PROJECT_ROOT/Log/regression"

launch_log="$PROJECT_ROOT/Log/regression/launch.log"
bag_log="$PROJECT_ROOT/Log/regression/bag.log"
launch_pid=""

stop_launch() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 60); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
      kill -TERM -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap stop_launch EXIT INT TERM

cd "$PROJECT_ROOT"
setsid ros2 launch fast_livo wuhu_truck29.launch.py \
  use_camera:=false use_rviz:=false rear_axle_to_imu:=true \
  middleware_transport:=SHM \
  >"$launch_log" 2>&1 &
launch_pid="$!"

started=0
for _ in $(seq 1 30); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "Mapping launch exited during startup; see $launch_log" >&2
    exit 1
  fi
  if grep -q "fastlivo_mapping.*process started" "$launch_log"; then
    started=1
    break
  fi
  sleep 1
done
if ((started == 0)); then
  echo "Mapping launch did not become ready; see $launch_log" >&2
  exit 1
fi
sleep 2

"$PROJECT_ROOT/scripts/play_all_bags.sh" \
  --bag-root "$BAG_ROOT" --prepared-bag "$PREPARED_BAG" \
  --rate "$PLAY_RATE" --lio-only >"$bag_log" 2>&1

keyframe_log="$PROJECT_ROOT/Log/backend/keyframes.csv"
stable_seconds=0
previous_signature=""
for _ in $(seq 1 45); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "Mapping launch exited before finalization; see $launch_log" >&2
    exit 1
  fi
  if [[ -f "$keyframe_log" ]]; then
    signature="$(stat -c '%s:%Y' "$keyframe_log")"
    if [[ "$signature" == "$previous_signature" ]]; then
      stable_seconds=$((stable_seconds + 1))
    else
      stable_seconds=0
      previous_signature="$signature"
    fi
    if ((stable_seconds >= 5)); then break; fi
  fi
  sleep 1
done
if ((stable_seconds < 5)); then
  echo "Keyframe output did not settle after playback." >&2
  exit 1
fi

stop_launch
launch_pid=""
trap - EXIT INT TERM

if grep -Eq "process has died|terminate called|what\(\):" "$launch_log"; then
  echo "Mapping process crashed; see $launch_log" >&2
  exit 1
fi
echo "Wuhu RTK regression run completed."
