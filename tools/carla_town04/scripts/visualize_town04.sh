#!/bin/bash
# Run sequential Town04 inference and render GT/prediction comparison panels.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STREAM_PETR_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WORKSPACE_ROOT="$(cd "${STREAM_PETR_ROOT}/.." && pwd)"
ANCHOR_ROOT="${ANCHOR3DLANE_DIR:-${WORKSPACE_ROOT}/Anchor3DLane}"
INFERENCE_IMAGE="${STREAMPETR_TRAIN_IMAGE:-adas-service:latest}"

CONFIG="${1:-projects/configs/StreamPETR/stream_petr_r50_flash_704_bs1_seq_town04.py}"
CHECKPOINT="${2:-work_dirs/stream_petr_town04_hybrid_24e/iter_7680.pth}"
OUTPUT_ROOT="${3:-work_dirs/stream_petr_town04_hybrid_24e/visualizations}"
INFO_PKL="${INFO_PKL:-data/carla_town04_9weather_3maneuver_8100/carla_town04_temporal_infos_val.pkl}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.40}"
MAX_PREDICTIONS="${MAX_PREDICTIONS:-20}"
MIN_VISIBLE_RATIO="${MIN_VISIBLE_RATIO:-0.08}"
INDICES="${INDICES:-0 10 20 39 40 50 60 79}"
WORKERS="${WORKERS:-2}"

PREDICTIONS="${OUTPUT_ROOT}/predictions_val.pkl"
FRAME_DIR="${OUTPUT_ROOT}/frames"
read -r -a RENDER_INDICES <<< "${INDICES}"

cd "${STREAM_PETR_ROOT}"
mkdir -p "${OUTPUT_ROOT}" "${FRAME_DIR}"

if [[ "${SKIP_INFERENCE:-0}" != "1" ]]; then
    PYTHONPATH_VALUE="/workspace/StreamPETR/.deps-py38:/anchor/Model/.deps-py38:/workspace/StreamPETR/mmdetection3d:/workspace/StreamPETR"
    podman run --rm -i --device nvidia.com/gpu=all --shm-size 8g \
        -v "${STREAM_PETR_ROOT}:/workspace/StreamPETR:rw" \
        -v "${ANCHOR_ROOT}:/anchor:ro" \
        -w /workspace/StreamPETR \
        -e "PYTHONPATH=${PYTHONPATH_VALUE}" \
        -e CUDA_VISIBLE_DEVICES=0 \
        -e PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128 \
        --entrypoint python3 \
        "${INFERENCE_IMAGE}" \
        tools/carla_town04/infer.py \
        "${CONFIG}" "${CHECKPOINT}" "${PREDICTIONS}" \
        --workers "${WORKERS}"
fi

python3 tools/carla_town04/visualize.py \
    "${INFO_PKL}" "${PREDICTIONS}" "${FRAME_DIR}" \
    --indices "${RENDER_INDICES[@]}" \
    --score-threshold "${SCORE_THRESHOLD}" \
    --max-predictions "${MAX_PREDICTIONS}" \
    --min-visible-ratio "${MIN_VISIBLE_RATIO}"

echo "Visualizations ready under ${FRAME_DIR}"
