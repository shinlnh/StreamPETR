#!/bin/bash
# Train the low-memory Town04 StreamPETR profile inside the existing
# adas-service CUDA 11.6 image. PyTorch/torchvision are reused read-only from
# Anchor3DLane; StreamPETR-specific packages live in .deps-py38.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STREAM_PETR_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WORKSPACE_ROOT="$(cd "${STREAM_PETR_ROOT}/.." && pwd)"
ANCHOR_ROOT="${ANCHOR3DLANE_DIR:-${WORKSPACE_ROOT}/Anchor3DLane}"
TRAIN_IMAGE="${STREAMPETR_TRAIN_IMAGE:-adas-service:latest}"
CONFIG="${1:-projects/configs/StreamPETR/stream_petr_r50_flash_704_bs1_seq_town04.py}"
WORK_DIR="${2:-work_dirs/stream_petr_town04_matrix}"
MAX_ITERS="${MAX_ITERS:-6480}"
WORKERS="${WORKERS:-2}"
LOG_INTERVAL="${LOG_INTERVAL:-10}"

if [[ ! -d "${STREAM_PETR_ROOT}/.deps-py38" ]]; then
    echo "ERROR: ${STREAM_PETR_ROOT}/.deps-py38 is missing." >&2
    exit 1
fi
if [[ ! -d "${STREAM_PETR_ROOT}/mmdetection3d" ]]; then
    echo "ERROR: ${STREAM_PETR_ROOT}/mmdetection3d is missing." >&2
    exit 1
fi
if [[ ! -d "${ANCHOR_ROOT}/Model/.deps-py38" ]]; then
    echo "ERROR: shared PyTorch dependencies not found under ${ANCHOR_ROOT}." >&2
    exit 1
fi
if [[ ! -f "${STREAM_PETR_ROOT}/${CONFIG}" ]]; then
    echo "ERROR: config not found: ${CONFIG}" >&2
    exit 1
fi

mkdir -p "${STREAM_PETR_ROOT}/${WORK_DIR}" "${STREAM_PETR_ROOT}/.torch-cache"

PYTHONPATH_VALUE="/workspace/StreamPETR/.deps-py38:/anchor/Model/.deps-py38:/workspace/StreamPETR/mmdetection3d:/workspace/StreamPETR"
EVAL_INTERVAL=$((MAX_ITERS + 1))

echo "════════════════════════════════════════════════"
echo "  StreamPETR Town04 — Train"
echo "  Config    : ${CONFIG}"
echo "  Work dir  : ${WORK_DIR}"
echo "  Max iters : ${MAX_ITERS}"
echo "  Log every : ${LOG_INTERVAL} iterations"
echo "════════════════════════════════════════════════"

exec podman run --rm -i --device nvidia.com/gpu=all --shm-size 8g \
    -v "${STREAM_PETR_ROOT}:/workspace/StreamPETR:rw" \
    -v "${ANCHOR_ROOT}:/anchor:ro" \
    -w /workspace/StreamPETR \
    -e "PYTHONPATH=${PYTHONPATH_VALUE}" \
    -e TORCH_HOME=/workspace/StreamPETR/.torch-cache \
    -e CUDA_VISIBLE_DEVICES=0 \
    -e PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128 \
    -e OMP_NUM_THREADS=2 \
    --entrypoint python3 \
    "${TRAIN_IMAGE}" \
    tools/train.py "${CONFIG}" \
    --work-dir "${WORK_DIR}" \
    --seed 0 \
    --no-validate \
    --cfg-options \
        runner.max_iters="${MAX_ITERS}" \
        checkpoint_config.interval="${MAX_ITERS}" \
        evaluation.interval="${EVAL_INTERVAL}" \
        log_config.interval="${LOG_INTERVAL}" \
        data.workers_per_gpu="${WORKERS}"
