#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "$#" -ne 3 ]]; then
    echo "Usage: $0 JOB_ID REMOTE_BUCKET_PREFIX LOCAL_DIR" >&2
    exit 2
fi

JOB_ID="$1"
REMOTE_PREFIX="$2"
LOCAL_DIR="$3"
POLL_SECONDS="${POLL_SECONDS:-300}"

mkdir -p "${LOCAL_DIR}"
echo "[sync] job=${JOB_ID}"
echo "[sync] remote=${REMOTE_PREFIX}"
echo "[sync] local=${LOCAL_DIR}"

while true; do
    date -Is
    if ! hf buckets sync "${REMOTE_PREFIX}" "${LOCAL_DIR}" --no-delete; then
        echo "[sync] WARNING: sync failed; retrying later" >&2
    fi

    STATUS_JSON="${LOCAL_DIR}/hf_job_status.json"
    if hf jobs inspect "${JOB_ID}" --json > "${STATUS_JSON}.tmp"; then
        mv "${STATUS_JSON}.tmp" "${STATUS_JSON}"
    fi
    STAGE="$(
        python3 - "${STATUS_JSON}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.is_file():
    print("UNKNOWN")
else:
    payload = json.loads(path.read_text())
    if isinstance(payload, list):
        payload = payload[0] if payload else {}
    status = payload.get("status", {})
    print(status.get("stage", "UNKNOWN") if isinstance(status, dict) else status)
PY
    )"
    echo "[sync] stage=${STAGE}"

    case "${STAGE}" in
        COMPLETED|ERROR|CANCELED|CANCELLED|TIMEOUT)
            hf buckets sync "${REMOTE_PREFIX}" "${LOCAL_DIR}" --no-delete || true
            echo "[sync] final sync complete"
            exit 0
            ;;
    esac
    sleep "${POLL_SECONDS}"
done
