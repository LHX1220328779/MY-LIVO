#!/usr/bin/env bash
# Play a MY-LIVO recording straight from the raw MCAP folders (no prepared/merged bag needed).
# Thin compatibility wrapper around tools/livo/player.py, which merges all Perception_*/Camera_*
# segments in exact time order, publishes /clock, paces at --rate and stops cleanly on Ctrl+C.
#
# Usage:
#   scripts/play_all_bags.sh [--dataset DIR] [--lio-only | --with-camera | --all-topics]
#                            [--start-offset SEC] [--duration SEC] [--rate RATE] [--dry-run]
#   (--bag-root DIR and --prepared-bag DIR are accepted as aliases of --dataset)
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

DATASET="/home/project/data/haibo/wuhu_livo/ros2bag"
START_OFFSET="0"
DURATION=""
PLAY_RATE="1.0"
PRESET="livo"
DRY_RUN=0

usage() {
  echo "Usage: $0 [--dataset DIR] [--lio-only|--with-camera|--all-topics] [--start-offset SEC] [--duration SEC] [--rate RATE] [--dry-run]"
}

while (($#)); do
  case "$1" in
    --dataset|--bag-root|--prepared-bag) DATASET="$2"; shift 2 ;;
    --start-offset) START_OFFSET="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --rate) PLAY_RATE="$2"; shift 2 ;;
    --lio-only) PRESET="lio"; shift ;;
    --with-camera) PRESET="livo"; shift ;;
    --all-topics) PRESET="all"; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -d "$DATASET" ]]; then
  echo "Dataset directory does not exist: $DATASET" >&2
  exit 1
fi

# Keep execution on the Ubuntu 22.04 / ROS 2 system toolchain even if Conda is present.
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$repo_dir/install/fast_livo/share/fast_livo/local_setup.bash" ]]; then
  source "$repo_dir/install/fast_livo/share/fast_livo/local_setup.bash"
fi
set -u
results_root=$(realpath -m -- "$repo_dir/results")
export ROS_LOG_DIR=$(realpath -m -- "${ROS_LOG_DIR:-$results_root/roslog}")
mkdir -p "$ROS_LOG_DIR"
# Decoded 1080p camera frames are ~180 MB/s; keep local traffic on shared memory so that the
# camera stream cannot starve the IMU stream.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-SHM}"

args=("$DATASET" --preset "$PRESET" --rate "$PLAY_RATE" --start-offset "$START_OFFSET")
[[ -n "$DURATION" ]] && args+=(--duration "$DURATION")
((DRY_RUN)) && args+=(--dry-run)
exec /usr/bin/python3 "$repo_dir/tools/livo/player.py" "${args[@]}"
