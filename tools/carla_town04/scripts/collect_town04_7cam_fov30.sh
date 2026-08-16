#!/usr/bin/env bash
# Collect the balanced Town04 matrix with the original six cameras plus a
# co-located 30-degree front camera. The six-camera output is never touched.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OUTPUT="${OUTPUT:-data/carla_town04_7cam_fov30}"

exec bash "${SCRIPT_DIR}/collect_town04_matrix.sh" \
    --camera-profile nuscenes-reference-7cam-front-narrow-fov30-v1 \
    "$@"
