#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="${RVIZ_IMAGE:-ros-jazzy-hybrid-localization-rviz}"
docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
