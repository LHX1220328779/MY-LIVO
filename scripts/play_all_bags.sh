#!/usr/bin/env bash
set -euo pipefail

BAG_ROOT="/home/project/data/haibo/huaining/03/ros2bag"
START_OFFSET="10"
PLAY_RATE="1.0"
PLAY_ALL_TOPICS=0
LIO_ONLY=1
DRY_RUN=0
PREPARED_BAG=""

usage() {
  echo "Usage: $0 [--bag-root DIR] [--prepared-bag DIR] [--start-offset SEC] [--rate RATE] [--lio-only | --with-camera | --all-topics] [--dry-run]"
}

while (($#)); do
  case "$1" in
    --bag-root) BAG_ROOT="$2"; shift 2 ;;
    --prepared-bag) PREPARED_BAG="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --rate) PLAY_RATE="$2"; shift 2 ;;
    --lio-only) LIO_ONLY=1; shift ;;
    --with-camera) LIO_ONLY=0; shift ;;
    --all-topics) PLAY_ALL_TOPICS=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ((LIO_ONLY && PLAY_ALL_TOPICS)); then
  echo "--lio-only and --all-topics cannot be used together" >&2
  exit 2
fi

if [[ ! -d "$BAG_ROOT" ]]; then
  echo "Bag root does not exist: $BAG_ROOT" >&2
  exit 1
fi
if ! awk -v value="$START_OFFSET" 'BEGIN { exit !(value >= 0) }'; then
  echo "--start-offset must be non-negative" >&2
  exit 2
fi
if ! awk -v value="$PLAY_RATE" 'BEGIN { exit !(value > 0) }'; then
  echo "--rate must be positive" >&2
  exit 2
fi

# Keep execution on the Ubuntu 22.04 / ROS 2 system toolchain even if Conda is
# installed in the container image.
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$(dirname "$0")/../install/setup.bash" ]]; then
  source "$(dirname "$0")/../install/setup.bash"
fi
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/my_livo_ros_log}"
mkdir -p "$ROS_LOG_DIR"

# Offline playback is local to this host. Avoid Fast DDS shared-memory port
# lock conflicts left by another ROS 2 process or an unclean previous exit.
export FASTDDS_BUILTIN_TRANSPORTS="UDPv4"

if [[ -z "$PREPARED_BAG" ]]; then PREPARED_BAG="${BAG_ROOT%/}_my_livo"; fi
if [[ -f "$PREPARED_BAG/metadata.yaml" ]]; then
  if ((LIO_ONLY)); then
    prepared_topic_args=(--topics front_left_lidar back_lidar front_right_lidar front_lidar imu_data imu_data/odometry)
  else
    prepared_topic_args=(--topics front_left_lidar back_lidar front_right_lidar front_lidar imu_data imu_data/odometry midrange_camera/ffmpeg)
  fi
  if ((PLAY_ALL_TOPICS)); then prepared_topic_args=(); fi
  echo "playing indexed runtime bag: $PREPARED_BAG start_offset=${START_OFFSET}s rate=${PLAY_RATE}"
  if ((DRY_RUN)); then exit 0; fi
  exec ros2 bag play "$PREPARED_BAG" --storage mcap --rate "$PLAY_RATE" \
    --start-offset "$START_OFFSET" --disable-keyboard-controls "${prepared_topic_args[@]}"
fi
echo "warning: indexed runtime bag not found at $PREPARED_BAG; using slower sequential fallback" >&2

mapfile -t METADATA_FILES < <(find "$BAG_ROOT" -mindepth 2 -maxdepth 2 -name metadata.yaml -type f | sort)
if ((${#METADATA_FILES[@]} == 0)); then
  echo "No ROS 2 bags found under: $BAG_ROOT" >&2
  exit 1
fi

GLOBAL_START=""
GLOBAL_END="0"
CAMERA_METADATA=()
PERCEPTION_METADATA=()
for metadata in "${METADATA_FILES[@]}"; do
  start_ns="$(awk '/nanoseconds_since_epoch:/ {print $2; exit}' "$metadata")"
  duration_ns="$(awk '/^  duration:/ {getline; print $2; exit}' "$metadata")"
  end_ns=$((start_ns + duration_ns))
  if [[ -z "$GLOBAL_START" ]] || ((start_ns < GLOBAL_START)); then
    GLOBAL_START="$start_ns"
  fi
  if ((end_ns > GLOBAL_END)); then GLOBAL_END="$end_ns"; fi
  case "$(basename "$(dirname "$metadata")")" in
    Camera_*) CAMERA_METADATA+=("$metadata") ;;
    Perception_*) PERCEPTION_METADATA+=("$metadata") ;;
  esac
done

session_duration="$(awk -v end="$GLOBAL_END" -v start="$GLOBAL_START" 'BEGIN {printf "%.9f", (end-start)/1e9}')"
if awk -v offset="$START_OFFSET" -v duration="$session_duration" 'BEGIN {exit !(offset >= duration)}'; then
  echo "Start offset is beyond the end of all bags." >&2
  exit 1
fi

run_stream() {
  local stream="$1"
  local metadata_array_name="$2"
  local -n stream_metadata="$metadata_array_name"
  local first_selected=1
  local child_pid=""
  local -a topic_args

  if ((PLAY_ALL_TOPICS)); then
    topic_args=()
  elif [[ "$stream" == "camera" ]]; then
    topic_args=(--topics midrange_camera/ffmpeg)
  else
    topic_args=(--topics front_left_lidar back_lidar front_right_lidar front_lidar imu_data imu_data/odometry)
  fi

  trap 'if [[ -n "${child_pid:-}" ]]; then kill -TERM "$child_pid" 2>/dev/null || true; fi; exit 0' INT TERM
  for metadata in "${stream_metadata[@]}"; do
    local bag_dir bag_name start_ns duration_ns relative_start relative_end
    local local_offset launch_delay
    bag_dir="$(dirname "$metadata")"
    bag_name="$(basename "$bag_dir")"
    start_ns="$(awk '/nanoseconds_since_epoch:/ {print $2; exit}' "$metadata")"
    duration_ns="$(awk '/^  duration:/ {getline; print $2; exit}' "$metadata")"
    relative_start="$(awk -v start="$start_ns" -v origin="$GLOBAL_START" 'BEGIN {printf "%.9f", (start-origin)/1e9}')"
    relative_end="$(awk -v start="$relative_start" -v duration="$duration_ns" 'BEGIN {printf "%.9f", start+duration/1e9}')"
    if awk -v offset="$START_OFFSET" -v end="$relative_end" 'BEGIN {exit !(offset >= end)}'; then
      continue
    fi

    if ((first_selected)); then
      local_offset="$(awk -v offset="$START_OFFSET" -v start="$relative_start" 'BEGIN {printf "%.9f", (offset > start ? offset-start : 0.0)}')"
      launch_delay="$(awk -v offset="$START_OFFSET" -v start="$relative_start" -v rate="$PLAY_RATE" 'BEGIN {printf "%.9f", (start > offset ? (start-offset)/rate : 0.0)}')"
    else
      # Bags in one sensor stream are intentionally started only after the
      # previous player exits. This prevents process startup jitter from
      # interleaving adjacent bags and sending timestamps backwards.
      local_offset="0.000000000"
      launch_delay="after_previous"
    fi

    if [[ "$launch_delay" == "after_previous" ]]; then
      printf 'stream=%-10s %-90s delay=sequential local_offset=%ss\n' \
        "$stream" "$bag_name" "$local_offset"
    else
      printf 'stream=%-10s %-90s delay=%ss local_offset=%ss\n' \
        "$stream" "$bag_name" "$launch_delay" "$local_offset"
    fi
    if ((DRY_RUN)); then
      first_selected=0
      continue
    fi
    if ((first_selected)) && awk -v delay="$launch_delay" 'BEGIN {exit !(delay > 0)}'; then
      sleep "$launch_delay"
    fi
    ros2 bag play "$bag_dir" --storage mcap --rate "$PLAY_RATE" \
      --start-offset "$local_offset" --disable-keyboard-controls "${topic_args[@]}" &
    child_pid="$!"
    wait "$child_pid"
    child_pid=""
    first_selected=0
  done
}

if ((DRY_RUN)); then
  if ((${#PERCEPTION_METADATA[@]})); then run_stream perception PERCEPTION_METADATA; fi
  if ((!LIO_ONLY && ${#CAMERA_METADATA[@]})); then run_stream camera CAMERA_METADATA; fi
  exit 0
fi

PIDS=()
cleanup() {
  trap - INT TERM EXIT
  if ((${#PIDS[@]})); then
    kill -TERM "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

if ((${#PERCEPTION_METADATA[@]})); then
  (run_stream perception PERCEPTION_METADATA) &
  PIDS+=("$!")
fi
if ((!LIO_ONLY && ${#CAMERA_METADATA[@]})); then
  (run_stream camera CAMERA_METADATA) &
  PIDS+=("$!")
fi
wait "${PIDS[@]}"
trap - INT TERM EXIT
