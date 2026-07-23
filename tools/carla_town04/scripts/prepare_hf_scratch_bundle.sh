#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUNDLE_DIR="${BUNDLE_DIR:-${REPO_ROOT}/work_dirs/hf_jobs/streampetr_town04_scratch_input}"
ARCHIVE="${BUNDLE_DIR}/streampetr_code.tar.gz"

mkdir -p "${BUNDLE_DIR}"
tar \
    --create \
    --gzip \
    --file "${ARCHIVE}" \
    --exclude="*/__pycache__" \
    --exclude="*.pyc" \
    --directory "${REPO_ROOT}" \
    projects \
    tools \
    mmdetection3d \
    LICENSE \
    README.md

cp "${REPO_ROOT}/tools/carla_town04/scripts/hf_job_bootstrap.sh" \
    "${BUNDLE_DIR}/hf_job_bootstrap.sh"
cp "${REPO_ROOT}/tools/carla_town04/scripts/hf_mount_probe.sh" \
    "${BUNDLE_DIR}/hf_mount_probe.sh"
cp "${REPO_ROOT}/tools/carla_town04/scripts/hf_extract_probe.sh" \
    "${BUNDLE_DIR}/hf_extract_probe.sh"

(
    cd "${BUNDLE_DIR}"
    sha256sum streampetr_code.tar.gz > streampetr_code.tar.gz.sha256
    sha256sum -c streampetr_code.tar.gz.sha256
)

if tar -tzf "${ARCHIVE}" | grep -Ei '^(data|work_dirs|ckpts)/|\.pth$' > /dev/null; then
    echo "ERROR: scratch bundle unexpectedly contains data or a checkpoint" >&2
    exit 2
fi

echo "[bundle] scratch-only input ready: ${BUNDLE_DIR}"
du -sh "${BUNDLE_DIR}"
