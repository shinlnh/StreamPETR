#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
INPUT_BUCKET="${INPUT_BUCKET:-huyluongngoc/streampetr-town04-scratch-input}"
OUTPUT_BUCKET="${OUTPUT_BUCKET:-huyluongngoc/streampetr-town04-scratch-output}"
DATASET_REPO="${DATASET_REPO:-BanVienCorp/StreamPETR_3D_Dataset}"
DATASET_REVISION="${DATASET_REVISION:-2729feccf65a9334378593e2d595ee8482bacb83}"
BUNDLE_DIR="${BUNDLE_DIR:-${REPO_ROOT}/work_dirs/hf_jobs/streampetr_town04_scratch_input}"
RUN_NAME="${RUN_NAME:-stream_petr_town04_6w_train_3w_val_scratch_24e}"
FLAVOR="${FLAVOR:-t4-small}"
TIMEOUT="${TIMEOUT:-5h}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-600}"
CHECKPOINT_KEEP_COUNT="${CHECKPOINT_KEEP_COUNT:-6}"

"${REPO_ROOT}/tools/carla_town04/scripts/prepare_hf_scratch_bundle.sh"

hf buckets create "${INPUT_BUCKET}" --private --exist-ok
hf buckets create "${OUTPUT_BUCKET}" --private --exist-ok
hf buckets sync "${BUNDLE_DIR}" "hf://buckets/${INPUT_BUCKET}" --delete

CURRENT_DATASET_REVISION="$(
    hf datasets info "${DATASET_REPO}" --expand sha --format json |
        python3 -c 'import json,sys; print(json.load(sys.stdin)["sha"])'
)"
if [[ "${CURRENT_DATASET_REVISION}" != "${DATASET_REVISION}" ]]; then
    echo "ERROR: dataset revision changed: expected=${DATASET_REVISION} current=${CURRENT_DATASET_REVISION}" >&2
    exit 3
fi

JOB_OUTPUT="$(
    hf jobs run \
        --detach \
        --flavor "${FLAVOR}" \
        --timeout "${TIMEOUT}" \
        --env "RUN_NAME=${RUN_NAME}" \
        --env "DATASET_REVISION=${DATASET_REVISION}" \
        --env "ITERS_PER_EPOCH=5400" \
        --env "EPOCHS=24" \
        --env "CHECKPOINT_INTERVAL=${CHECKPOINT_INTERVAL}" \
        --env "CHECKPOINT_KEEP_COUNT=${CHECKPOINT_KEEP_COUNT}" \
        --label "project=streampetr" \
        --label "purpose=town04-scratch-weather-holdout" \
        --label "initialization=random_scratch" \
        --label "split=6weather_train_3weather_val" \
        --label "budget_cap_usd=2_00" \
        --volume "hf://buckets/${INPUT_BUCKET}:/inputs:ro" \
        --volume "hf://datasets/${DATASET_REPO}:/hf_dataset:ro" \
        --volume "hf://buckets/${OUTPUT_BUCKET}:/outputs:rw" \
        pytorch/pytorch:1.13.1-cuda11.6-cudnn8-runtime \
        bash /inputs/hf_job_bootstrap.sh
)"
printf '%s\n' "${JOB_OUTPUT}"
JOB_ID="$(
    printf '%s\n' "${JOB_OUTPUT}" |
        grep -Eo '[[:xdigit:]]{24}' |
        tail -n 1
)"
if [[ -z "${JOB_ID}" ]]; then
    echo "ERROR: could not parse Hugging Face Job ID" >&2
    exit 4
fi

SHORT_ID="${JOB_ID:0:8}"
LOCAL_DIR="${REPO_ROOT}/work_dirs/hf_jobs/downloads/${RUN_NAME}_${SHORT_ID}"
SYNC_SESSION="streampetr_hf_sync_${SHORT_ID}"
REMOTE_PREFIX="hf://buckets/${OUTPUT_BUCKET}/${RUN_NAME}"
mkdir -p "${LOCAL_DIR}"
printf '%s\n' \
    "HF_JOB_ID=${JOB_ID}" \
    "HF_JOB_URL=https://huggingface.co/jobs/huyluongngoc/${JOB_ID}" \
    "HF_INPUT_BUCKET=hf://buckets/${INPUT_BUCKET}" \
    "HF_OUTPUT_BUCKET=hf://buckets/${OUTPUT_BUCKET}" \
    "HF_DATASET=${DATASET_REPO}" \
    "HF_DATASET_REVISION=${DATASET_REVISION}" \
    "RUN_NAME=${RUN_NAME}" \
    "LOCAL_SYNC_TMUX_SESSION=${SYNC_SESSION}" \
    > "${LOCAL_DIR}/job.env"

tmux new-session \
    -d \
    -s "${SYNC_SESSION}" \
    "cd '${REPO_ROOT}' && POLL_SECONDS=120 bash tools/carla_town04/scripts/sync_hf_training_artifacts.sh '${JOB_ID}' '${REMOTE_PREFIX}' '${LOCAL_DIR}' >> '${LOCAL_DIR}/sync.log' 2>&1"

echo "[launch] job=${JOB_ID}"
echo "[launch] url=https://huggingface.co/jobs/huyluongngoc/${JOB_ID}"
echo "[launch] local=${LOCAL_DIR}"
echo "[launch] sync_session=${SYNC_SESSION}"
