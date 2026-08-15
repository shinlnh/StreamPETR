#!/usr/bin/env bash

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="$repo_dir/projects/configs/StreamPETR/stream_petr_r50_nucarla_full_90e.py"
work_dir="$repo_dir/work_dirs/stream_petr_r50_nucarla_full_90e"
train_info="$repo_dir/data/nucarla_full/nucarla_full_temporal_infos_train.pkl"
val_info="$repo_dir/data/nucarla_full/nucarla_full_temporal_infos_val.pkl"
pretrained_checkpoint="$repo_dir/ckpts/stream_petr_r50_flash_704_bs2_seq_90e.pth"
pretrained_sha256="e6323ae5c31adf1eedd46d6dd4fd3c73d95aa26f18cc8aa23c196494b7de3451"

for required_file in "$config" "$train_info" "$val_info" "$pretrained_checkpoint"; do
    if [[ ! -f "$required_file" ]]; then
        echo "Missing required file: $required_file" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "$pretrained_checkpoint" | awk '{print $1}')"
if [[ "$actual_sha256" != "$pretrained_sha256" ]]; then
    echo "Refusing to train with an unverified nuScenes checkpoint." >&2
    echo "Expected SHA-256: $pretrained_sha256" >&2
    echo "Actual SHA-256:   $actual_sha256" >&2
    exit 1
fi

mkdir -p "$work_dir"
exec 9>"$work_dir/train.lock"
if ! flock -n 9; then
    echo "A nuCarla training process already holds $work_dir/train.lock" >&2
    exit 1
fi

resume_args=()
if [[ -f "$work_dir/latest.pth" ]]; then
    resume_checkpoint="$work_dir/latest.pth"
    latest_target="$(readlink -f "$work_dir/latest.pth")"
    latest_name="$(basename "$latest_target")"
    latest_iter="${latest_name#iter_}"
    latest_iter="${latest_iter%.pth}"
    matching_best="$work_dir/best_NuCarla_NDS_iter_${latest_iter}.pth"
    # When a run is interrupted immediately after validation, latest.pth was
    # written just before the EvalHook updated best-score metadata. Resume the
    # matching best file so save_best remains monotonic after restart.
    if [[ -f "$matching_best" ]]; then
        resume_checkpoint="$matching_best"
    fi
    echo "Resuming from $resume_checkpoint"
    resume_args=(--resume-from "$resume_checkpoint")
else
    echo "Starting from the verified official StreamPETR nuScenes R50 90e checkpoint"
fi

cd "$repo_dir"
export PYTHONPATH="$repo_dir/mmcv:$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
export PYTORCH_CUDA_ALLOC_CONF="expandable_segments:True"
exec "$repo_dir/.venv/bin/python" -u tools/train.py \
    "$config" \
    --work-dir "$work_dir" \
    --gpu-ids 0 \
    --seed 0 \
    "${resume_args[@]}" \
    "$@"
