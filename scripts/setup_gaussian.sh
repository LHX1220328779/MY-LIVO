#!/usr/bin/env bash
# Optional, isolated CUDA/LibTorch environment. Never changes the ROS Python/OpenCV installation.
set -euo pipefail
gs_env=${GAUSSIAN_ENV:-/opt/venvs/my-livo-gaussian}
cuda_prefix=${CUDA_HOME:-/opt/cuda-12.4}
conda_exe=${GAUSSIAN_CONDA:-/opt/conda/bin/conda}
if [[ ! -x "$gs_env/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$gs_env"
fi
if ! "$gs_env/bin/python" -c 'import torch; assert torch.__version__.startswith("2.5.1"); assert torch.cuda.is_available()' 2>/dev/null; then
  "$gs_env/bin/pip" install --no-cache-dir torch==2.5.1 --index-url https://download.pytorch.org/whl/cu124
fi
if [[ ! -x "$cuda_prefix/bin/nvcc" ]]; then
  "$conda_exe" create -y -p "$cuda_prefix" --override-channels -c nvidia/label/cuda-12.4.1 -c defaults \
    cuda-nvcc cuda-cudart-dev cuda-cudart-static cuda-cccl cuda-driver-dev
fi
# Reuse the CUDA libraries bundled with PyTorch rather than downloading a second copy.
"$gs_env/bin/python" - "$cuda_prefix" <<'PY'
from pathlib import Path
import sys
import torch
nvidia = Path(torch.__file__).resolve().parent.parent / "nvidia"
prefix = Path(sys.argv[1])
for source in nvidia.glob("*/include/*"):
    target = prefix / "include" / source.name
    if not target.exists(): target.symlink_to(source)
for source in nvidia.glob("*/lib/*.so*"):
    for name in {source.name, source.name.split(".so")[0] + ".so"}:
        target = prefix / "lib" / name
        if not target.exists(): target.symlink_to(source)
print("PyTorch", torch.__version__, "GPU", torch.cuda.get_device_name())
PY
echo "Dependencies ready. Build with GAUSSIAN_PYTHON=$gs_env/bin/python CUDA_HOME=$cuda_prefix bash scripts/build_gaussian.sh"
