#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
gs_python=${GAUSSIAN_PYTHON:-/opt/venvs/my-livo-gaussian/bin/python}
cuda_prefix=${CUDA_HOME:-/opt/cuda-12.4}
export PATH="$cuda_prefix/bin:/usr/bin:/bin"
export CUDA_HOME="$cuda_prefix"
torch_prefix=$("$gs_python" -c 'import torch; assert torch.cuda.is_available(), "CUDA PyTorch required"; print(torch.utils.cmake_prefix_path)')
cmake -S "$repo_dir/gaussian" -B "$repo_dir/build/gaussian" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$torch_prefix" \
  -DPython_EXECUTABLE="$gs_python" \
  -DCMAKE_CUDA_COMPILER="$cuda_prefix/bin/nvcc" -DCUDAToolkit_ROOT="$cuda_prefix" -DCUDA_TOOLKIT_ROOT_DIR="$cuda_prefix" \
  -DCMAKE_CUDA_ARCHITECTURES="${GAUSSIAN_CUDA_ARCH:-89}"
cmake --build "$repo_dir/build/gaussian" --parallel "${BUILD_JOBS:-1}"
