#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_jobs=${BUILD_JOBS:-8}

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

colcon --log-base "$repo_dir/log" build \
  --base-paths "$repo_dir" \
  --build-base "$repo_dir/build" \
  --install-base "$repo_dir/install" \
  --packages-select fast_livo \
  --parallel-workers "$build_jobs" \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "Build complete. In each new terminal run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $repo_dir/install/setup.bash"
echo "  ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=false use_rviz:=true"
