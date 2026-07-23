#!/usr/bin/env bash
set -Eeuo pipefail

INPUT_ROOT="${INPUT_ROOT:-/inputs}"
DATASET_MOUNT="${DATASET_MOUNT:-/hf_dataset}"
JOB_REPO_ROOT="${JOB_REPO_ROOT:-/tmp/StreamPETR}"
JOB_MODE="${JOB_MODE:-train}"
CODE_ARCHIVE="${INPUT_ROOT}/streampetr_code.tar.gz"
DATASET_NAME="carla_town04_9weather_3maneuver_8100"

cd "${INPUT_ROOT}"
sha256sum -c streampetr_code.tar.gz.sha256

mkdir -p "${JOB_REPO_ROOT}" "${JOB_REPO_ROOT}/data"
tar --no-same-owner --no-same-permissions -xzf "${CODE_ARCHIVE}" -C "${JOB_REPO_ROOT}"

if [[ ! -d "${DATASET_MOUNT}/${DATASET_NAME}" ]]; then
    echo "ERROR: mounted Hugging Face dataset is missing ${DATASET_NAME}" >&2
    exit 2
fi
ln -s "${DATASET_MOUNT}/${DATASET_NAME}" "${JOB_REPO_ROOT}/data/${DATASET_NAME}"

echo "[bootstrap] code=${CODE_ARCHIVE}"
echo "[bootstrap] dataset=${DATASET_MOUNT}/${DATASET_NAME}"
echo "[bootstrap] output=${OUTPUT_ROOT:-/outputs}"
echo "[bootstrap] mode=${JOB_MODE}"

export REPO_ROOT="${JOB_REPO_ROOT}"
case "${JOB_MODE}" in
    train)
        exec bash "${JOB_REPO_ROOT}/tools/carla_town04/scripts/hf_train_full.sh"
        ;;
    smoke)
        exec bash "${JOB_REPO_ROOT}/tools/carla_town04/scripts/hf_smoke_train.sh"
        ;;
    *)
        echo "ERROR: unsupported JOB_MODE=${JOB_MODE}" >&2
        exit 3
        ;;
esac
