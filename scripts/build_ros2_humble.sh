#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_jobs=${BUILD_JOBS:-2}
build_dir="$repo_dir/build/fast_livo"
install_dir="$repo_dir/install/fast_livo"

# A previous build was created by a service account (nobody:nogroup).  That
# leaves the canonical incremental build tree readable but not writable by
# the developer who invokes this script.  Repair ownership before CMake sees
# the cache; do not remove the cache or force a clean rebuild.
prepare_build_dir() {
  local owner
  mkdir -p "$build_dir"

  if [[ "$(id -u)" -eq 0 ]]; then
    owner="$(id -u):$(id -g)"
    # Some workspace mounts reject chown even for root.  In that case the
    # build cache must remain shared-writable because its files are owned by
    # the service account that created them.
    if ! chown -R "$owner" "$build_dir" 2>/dev/null; then
      chmod -R a+rwX "$build_dir"
    fi
    # Keep this cache usable when the next build is launched by a different
    # container/service account.
    chmod -R a+rwX "$build_dir"
  elif [[ ! -w "$build_dir" ]] || find "$build_dir" -not -writable -print -quit | grep -q .; then
    echo "Build directory or an existing build file is not writable: $build_dir" >&2
    echo "Repair its ownership/permissions, then rerun this script; the CMake cache is preserved." >&2
    exit 4
  fi
}

# Keep CMake, Python and dependencies on the Ubuntu 22.04 / ROS 2 Humble
# toolchain even when the interactive shell previously activated Conda.
unset CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL CONDA_PREFIX CONDA_DEFAULT_ENV
unset _CE_CONDA _CE_M PYTHONHOME PYTHONPATH
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PKG_CONFIG_PATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
set -u

echo "Repository:    $repo_dir"
echo "System Python: $(command -v python3)"
echo "System CMake:  $(command -v cmake)"
echo "System C++:    $(command -v c++)"
echo "Conda prefix:  ${CONDA_PREFIX-<unset>}"

# This repository contains one ament package. Reuse its CMake build directly;
# keep one canonical build/install pair; never wipe its cache on each build.
prepare_build_dir
cmake -S "$repo_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX="$install_dir"
cmake --build "$build_dir" --parallel "$build_jobs"
cmake --install "$build_dir"

echo
echo "Build complete. In each new terminal run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $install_dir/share/fast_livo/local_setup.bash"
echo "  ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=false use_rviz:=true"
