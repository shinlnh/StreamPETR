#!/bin/bash
# Collect 9 weather presets x 3 maneuvers x 300 samples for StreamPETR.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="${OUTPUT:-data/carla_town04_9weather_3maneuver_8100}"
JPEG_QUALITY="${JPEG_QUALITY:-82}"
VEHICLES="${VEHICLES:-60}"
BATCH_SCENES="${BATCH_SCENES:-45}"
EXTRA_ARGS=("$@")

source "${SCRIPT_DIR}/lib.sh"

manifest_frames() {
    python3 - "${OUTPUT}/dataset_manifest.json" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
print(json.loads(path.read_text()).get("total_frames", 0) if path.exists() else 0)
PY
}

previous_frames=-1
stalled_runs=0
while true; do
    set +e
    bash "${SCRIPT_DIR}/collect_town04.sh" \
        --scenario-matrix \
        --town Town04_Opt \
        --frames-per-case 300 \
        --clips-per-case 15 \
        --vehicles "${VEHICLES}" \
        --jpeg-quality "${JPEG_QUALITY}" \
        --max-scenes-per-run "${BATCH_SCENES}" \
        --output "${OUTPUT}" \
        --resume \
        "${EXTRA_ARGS[@]}"
    status=$?
    set -e

    frames=$(manifest_frames)
    echo "Matrix progress checkpoint: ${frames}/8100 frames"
    if [[ "${frames}" -ge 8100 ]]; then
        exit 0
    fi
    if [[ "${status}" -eq 130 ]]; then
        exit 130
    fi
    if [[ "${frames}" -le "${previous_frames}" ]]; then
        stalled_runs=$((stalled_runs + 1))
    else
        stalled_runs=0
    fi
    if [[ "${stalled_runs}" -ge 3 ]]; then
        echo "ERROR: collection made no progress for three consecutive runs." >&2
        exit 1
    fi
    previous_frames="${frames}"

    # CARLA 0.9.13 can accumulate GPU resources after many camera lifecycles.
    # A fresh server between bounded batches avoids a late exit-139 while the
    # atomic PKLs and manifest allow an exact resume at the next scene.
    fix_podman_env
    carla_cname=$(svc_cname carla)
    if [[ -n "${carla_cname}" ]] && svc_running carla; then
        echo "Refreshing CARLA after this batch..."
        podman stop "${carla_cname}" >/dev/null || true
    fi
done
