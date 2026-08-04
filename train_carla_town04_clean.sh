#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$ROOT/.venv/bin/python"
CONFIG="projects/configs/StreamPETR/stream_petr_r50_carla_town04_clean.py"
DATASET="$ROOT/../../scripts/collect_dataset/StreamPETR_CARLA_TOWN04_CLEAN"
SUMMARY="$DATASET/summary.json"
VALIDATOR="$ROOT/../../scripts/collect_dataset/validate_streampetr_dataset.py"
WORK_DIR="$ROOT/work_dirs/stream_petr_r50_carla_town04_clean"
RESUME_ARGS=()

[[ -x "$PYTHON" ]] || { echo "Run ./setup_blackwell_env.sh first"; exit 1; }
[[ -f "$DATASET/carla_streampetr_infos_train.pkl" ]] || {
  echo "Clean Town04 dataset is missing; finish collection first"
  exit 1
}
jq -e \
  '.dataset_name == "StreamPETR_CARLA_TOWN04_CLEAN"
    and .plan_version == "town04-clean-instance-v1"
    and .complete_cases == 9
    and .samples == 3600
    and .train_samples == 2800
    and .val_samples == 800
    and .instance_visibility_filter == true
    and (.class_instances | to_entries | all(.value > 0))' \
  "$SUMMARY" >/dev/null || {
  echo "Dataset is incomplete or failed the clean-data contract."
  echo "Inspect: $SUMMARY"
  exit 1
}

/usr/bin/python3 "$VALIDATOR" "$DATASET" \
  --plan-version town04-clean-instance-v1 \
  --expected-samples 3600 \
  --require-clean-visibility

if [[ -e "$WORK_DIR/latest.pth" ]]; then
  echo "Resuming from $WORK_DIR/latest.pth"
  RESUME_ARGS=(--resume-from "$WORK_DIR/latest.pth")
else
  echo "Starting original CPFPN architecture from six-class nuScenes initialization"
fi

cd "$ROOT"
export PYTHONPATH="$ROOT/mmcv:$ROOT:${PYTHONPATH:-}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
exec "$PYTHON" tools/train.py "$CONFIG" \
  --work-dir "$WORK_DIR" \
  --gpu-ids 0 "${RESUME_ARGS[@]}" "$@"
