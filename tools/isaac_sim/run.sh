#!/usr/bin/env bash
set -euo pipefail

IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ISAAC_SIM_PACKAGE="/workspace/ros2_hybrid_localization/src/hybrid_localization_isaac_sim"

docker run --rm -it \
  --name isaac-sim \
  --gpus all \
  --network host \
  --ipc host \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e HYBRID_LOCALIZATION_ISAAC_SIM_ROOT="${ISAAC_SIM_PACKAGE}" \
  -v "$HOME/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw" \
  -v "$HOME/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$HOME/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$HOME/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$HOME/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$HOME/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "${REPO_ROOT}:/workspace/ros2_hybrid_localization:ro" \
  "$IMAGE" \
  ./runheadless.sh
