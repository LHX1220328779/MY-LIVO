#!/usr/bin/env bash
# Launch the verified SuperSplat WebGPU configuration without changing normal browser profiles.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
viewer_runtime="$repo_dir/build/gaussian/viewer"
mkdir -p "$viewer_runtime"
command -v node >/dev/null && command -v npm >/dev/null || {
  echo "Node.js >=20.19 and npm are required for the viewer launcher." >&2; exit 2;
}
if [[ ! -f "$viewer_runtime/node_modules/playwright/package.json" ]]; then
  npm install --prefix "$viewer_runtime" --no-audit --no-fund playwright@1.65.0
fi
viewer_browser=$(GAUSSIAN_PLAYWRIGHT="$viewer_runtime/node_modules/playwright" node -e \
  'process.stdout.write(require(process.env.GAUSSIAN_PLAYWRIGHT).chromium.executablePath())')
if [[ ! -x "$viewer_browser" ]]; then
  "$viewer_runtime/node_modules/.bin/playwright" install chromium
fi
export GAUSSIAN_PLAYWRIGHT="$viewer_runtime/node_modules/playwright"
exec node "$repo_dir/tools/livo/gaussian_viewer.cjs" "$@"
