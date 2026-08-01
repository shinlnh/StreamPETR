#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$ROOT/.venv/bin/python"
CONFIG="projects/configs/StreamPETR/stream_petr_r50_carla_4cam_ped_balanced.py"
BALANCED_INFO="$ROOT/data/carla/carla_streampetr_infos_train_ped_balanced.pkl"
WORK_DIR="$ROOT/work_dirs/stream_petr_r50_carla_4cam_ped_balanced"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /absolute/path/to/checkpoint.pth [extra train.py arguments]" >&2
  exit 2
fi

CHECKPOINT="$(realpath "$1")"
shift
[[ -x "$PYTHON" ]] || { echo "Missing $PYTHON" >&2; exit 1; }
[[ -f "$CHECKPOINT" ]] || { echo "Checkpoint not found: $CHECKPOINT" >&2; exit 1; }
[[ -f "$BALANCED_INFO" ]] || { echo "Balanced dataset not found: $BALANCED_INFO" >&2; exit 1; }

# Two StreamPETR jobs do not fit in 16 GB VRAM. Exclude this shell's parent
# command and refuse to disturb an existing run.
if pgrep -af '[t]ools/train.py .*stream_petr_r50_carla_4cam' >/dev/null; then
  echo "Another StreamPETR training process is active; wait for it to finish." >&2
  exit 1
fi

cd "$ROOT"
export PYTHONPATH="$ROOT/mmcv:$ROOT:${PYTHONPATH:-}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
exec "$PYTHON" tools/train.py "$CONFIG" \
  --work-dir "$WORK_DIR" \
  --gpu-ids 0 \
  --cfg-options load_from="$CHECKPOINT" "$@"
