#!/usr/bin/env bash

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="$repo_dir/projects/configs/StreamPETR/stream_petr_r50_nucarla_full_7cam_fov30_finetune.py"
work_dir="$repo_dir/work_dirs/stream_petr_r50_nucarla_full_7cam_fov30_finetune"
data_dir="$repo_dir/data/nucarla_full_7cam_fov30"
train_info="$data_dir/nucarla_full_7cam_fov30_temporal_infos_train.pkl"
val_info="$data_dir/nucarla_full_7cam_fov30_temporal_infos_val.pkl"
source_checkpoint="$repo_dir/work_dirs/stream_petr_r50_nucarla_full_90e/best_NuCarla_NDS_iter_116650.pth"
source_sha256="d46ad8305381fc53cc99581e6548e75e9ce771612e49065d49f4f05bd441544f"

cd "$repo_dir"

for required_file in "$config" "$train_info" "$val_info" "$source_checkpoint"; do
    if [[ ! -f "$required_file" ]]; then
        echo "Missing required file: $required_file" >&2
        exit 1
    fi
done

actual_sha256="$(sha256sum "$source_checkpoint" | awk '{print $1}')"
if [[ "$actual_sha256" != "$source_sha256" ]]; then
    echo "Refusing to fine-tune from an unverified six-camera checkpoint." >&2
    echo "Expected SHA-256: $source_sha256" >&2
    echo "Actual SHA-256:   $actual_sha256" >&2
    exit 1
fi

"$repo_dir/.venv/bin/python" "$repo_dir/tools/validate_nucarla_7cam_fov30.py" \
    --source-root "$repo_dir/data/nucarla_full" \
    --root "$data_dir"

mkdir -p "$work_dir"
exec 9>"$work_dir/train.lock"
if ! flock -n 9; then
    echo "A seven-camera training process already holds $work_dir/train.lock" >&2
    exit 1
fi
printf '%s\n' "$$" > "$work_dir/train.pid"

resume_args=()
if [[ -f "$work_dir/latest.pth" ]]; then
    resume_checkpoint="$(readlink -f "$work_dir/latest.pth")"
    echo "Resuming seven-camera fine-tuning from $resume_checkpoint"
    resume_args=(--resume-from "$resume_checkpoint")
else
    echo "Starting seven-camera fine-tuning from the preserved six-camera checkpoint"
fi

export PYTHONPATH="$repo_dir/mmcv:$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
export PYTORCH_CUDA_ALLOC_CONF="expandable_segments:True"
exec "$repo_dir/.venv/bin/python" -u tools/train.py \
    "$config" \
    --work-dir "$work_dir" \
    --gpu-ids 0 \
    --seed 0 \
    "${resume_args[@]}" \
    "$@"
