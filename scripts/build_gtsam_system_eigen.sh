#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
gtsam_source="$repo_dir/3rdparty/gtsam"
gtsam_build="$gtsam_source/build-system-eigen"
gtsam_install="$gtsam_source/install-system-eigen"
build_jobs=${BUILD_JOBS:-4}

if [[ ! -f "$gtsam_source/CMakeLists.txt" ]]; then
  echo "GTSAM source was not found at $gtsam_source" >&2
  exit 3
fi

# Use the same Ubuntu toolchain and Eigen installation as ROS/PCL. Mixing a
# bundled Eigen GTSAM build with the system-Eigen frontend risks ABI/alignment
# problems at the shared-library boundary.
unset CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL CONDA_PREFIX CONDA_DEFAULT_ENV
unset _CE_CONDA _CE_M PYTHONHOME PYTHONPATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

cmake -S "$gtsam_source" -B "$gtsam_build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$gtsam_install" \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN" \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_WITH_TBB=OFF

cmake --build "$gtsam_build" --target install --parallel "$build_jobs"

config="$gtsam_install/lib/cmake/GTSAM/GTSAMConfig.cmake"
if [[ ! -f "$config" ]]; then
  echo "GTSAM installation did not produce $config" >&2
  exit 4
fi

echo "System-Eigen GTSAM installed at $gtsam_install"
