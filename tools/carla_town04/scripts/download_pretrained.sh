#!/bin/bash
# Download and verify the official StreamPETR R50 704x256 90-epoch checkpoint.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STREAM_PETR_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
CHECKPOINT_DIR="${STREAM_PETR_ROOT}/ckpts"
CHECKPOINT_PATH="${CHECKPOINT_DIR}/stream_petr_r50_flash_704_bs2_seq_90e.pth"
CHECKPOINT_URL="https://github.com/exiawsh/storage/releases/download/v1.0/stream_petr_r50_flash_704_bs2_seq_90e.pth"
EXPECTED_SHA256="e6323ae5c31adf1eedd46d6dd4fd3c73d95aa26f18cc8aa23c196494b7de3451"

mkdir -p "${CHECKPOINT_DIR}"
if [[ ! -f "${CHECKPOINT_PATH}" ]]; then
    curl -L --fail --retry 3 --continue-at - \
        --output "${CHECKPOINT_PATH}" "${CHECKPOINT_URL}"
fi

echo "${EXPECTED_SHA256}  ${CHECKPOINT_PATH}" | sha256sum --check
echo "Official checkpoint ready: ${CHECKPOINT_PATH}"
