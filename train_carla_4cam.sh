#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$ROOT/.venv/bin/python"
CONFIG="projects/configs/StreamPETR/stream_petr_r50_carla_4cam_finetune.py"
SUMMARY="$ROOT/data/carla/summary.json"
WORK_DIR="$ROOT/work_dirs/stream_petr_r50_carla_4cam_finetune"
RESUME_ARGS=()

[[ -x "$PYTHON" ]] || { echo "Run ./setup_blackwell_env.sh first"; exit 1; }
[[ -f "$ROOT/data/carla/carla_streampetr_infos_train.pkl" ]] || {
  echo "Dataset infos are missing; finish or smoke-test collection first"
  exit 1
}
jq -e \
  '.complete_cases == 54 and .samples == 21600 and .train_samples == 17200 and .val_samples == 4400' \
  "$SUMMARY" >/dev/null || {
  echo "Dataset is incomplete. Expected 54 cases / 21,600 samples."
  echo "Inspect: $SUMMARY"
  exit 1
}
if [[ -e "$WORK_DIR/latest.pth" ]]; then
  echo "Resuming from $WORK_DIR/latest.pth"
  RESUME_ARGS=(--resume-from "$WORK_DIR/latest.pth")
fi

cd "$ROOT"
export PYTHONPATH="$ROOT/mmcv:$ROOT:${PYTHONPATH:-}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
exec "$PYTHON" tools/train.py "$CONFIG" \
  --work-dir "$WORK_DIR" \
  --gpu-ids 0 "${RESUME_ARGS[@]}" "$@"
