#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="$ROOT/.venv/bin/python"
CONFIG="projects/configs/StreamPETR/stream_petr_r50_carla_4cam_finetune.py"
CHECKPOINT="work_dirs/stream_petr_r50_carla_4cam_finetune/iter_16125.pth"
OUTPUT_DIR="$ROOT/work_dirs/stream_petr_r50_carla_4cam_finetune/deployment"
TRTEXEC="${TRTEXEC:-$ROOT/.tools/tensorrt-10.9/usr/src/tensorrt/bin/trtexec}"
TIMING_CACHE="$OUTPUT_DIR/rtx5070ti_timing.cache"
TEST_INPUT_DIR="$OUTPUT_DIR/test_inputs"

[[ -x "$PYTHON" ]] || {
  echo "Python environment is missing: $PYTHON"
  exit 1
}
[[ -f "$ROOT/$CHECKPOINT" ]] || {
  echo "Checkpoint is missing: $ROOT/$CHECKPOINT"
  exit 1
}
[[ -x "$TRTEXEC" ]] || {
  echo "TensorRT trtexec is missing: $TRTEXEC"
  exit 1
}

cd "$ROOT"
export PYTHONPATH="$ROOT/mmcv:$ROOT:${PYTHONPATH:-}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"

"$PYTHON" tools/deployment/export_carla_stream_petr.py \
  --config "$CONFIG" \
  --checkpoint "$CHECKPOINT" \
  --output-dir "$OUTPUT_DIR"

"$TRTEXEC" \
  --onnx="$OUTPUT_DIR/stream_petr_carla_4cam_encoder.onnx" \
  --saveEngine="$OUTPUT_DIR/stream_petr_carla_4cam_encoder_fp16.engine" \
  --timingCacheFile="$TIMING_CACHE" \
  --memPoolSize=workspace:4G \
  --builderOptimizationLevel=5 \
  --fp16 \
  --skipInference

"$TRTEXEC" \
  --onnx="$OUTPUT_DIR/stream_petr_carla_4cam_temporal_head.onnx" \
  --saveEngine="$OUTPUT_DIR/stream_petr_carla_4cam_temporal_head_fp16.engine" \
  --timingCacheFile="$TIMING_CACHE" \
  --memPoolSize=workspace:6G \
  --builderOptimizationLevel=5 \
  --fp16 \
  --skipInference

"$TRTEXEC" \
  --loadEngine="$OUTPUT_DIR/stream_petr_carla_4cam_encoder_fp16.engine" \
  --loadInputs="images:$TEST_INPUT_DIR/images.bin" \
  --warmUp=100 \
  --duration=3 \
  --useCudaGraph \
  --noDataTransfers

HEAD_INPUTS="image_features:$TEST_INPUT_DIR/image_features.bin"
HEAD_INPUTS+=",timestamp:$TEST_INPUT_DIR/timestamp.bin"
HEAD_INPUTS+=",ego_pose:$TEST_INPUT_DIR/ego_pose.bin"
HEAD_INPUTS+=",ego_pose_inv:$TEST_INPUT_DIR/ego_pose_inv.bin"
HEAD_INPUTS+=",prev_exists:$TEST_INPUT_DIR/prev_exists.bin"
HEAD_INPUTS+=",pre_memory_embedding:$TEST_INPUT_DIR/pre_memory_embedding.bin"
HEAD_INPUTS+=",pre_memory_reference_point:$TEST_INPUT_DIR/pre_memory_reference_point.bin"
HEAD_INPUTS+=",pre_memory_timestamp:$TEST_INPUT_DIR/pre_memory_timestamp.bin"
HEAD_INPUTS+=",pre_memory_egopose:$TEST_INPUT_DIR/pre_memory_egopose.bin"
HEAD_INPUTS+=",pre_memory_velocity:$TEST_INPUT_DIR/pre_memory_velocity.bin"

"$TRTEXEC" \
  --loadEngine="$OUTPUT_DIR/stream_petr_carla_4cam_temporal_head_fp16.engine" \
  --loadInputs="$HEAD_INPUTS" \
  --warmUp=100 \
  --duration=3 \
  --useCudaGraph \
  --noDataTransfers

echo "ONNX and TensorRT artifacts: $OUTPUT_DIR"
