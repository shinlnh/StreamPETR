#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$ROOT/../.." && pwd)"
SUMMARY="$ROOT/data/carla/summary.json"
VALIDATOR="$REPO_ROOT/scripts/collect_dataset/validate_streampetr_dataset.py"
REPORT="$ROOT/work_dirs/carla_dataset_validation.json"

echo "[WAIT] waiting for a complete 54-case CARLA dataset"
while ! jq -e \
  '.complete_cases == 54 and .samples == 21600 and .train_samples == 17200 and .val_samples == 4400' \
  "$SUMMARY" >/dev/null 2>&1; do
  sleep 30
done

while systemctl --user is-active --quiet adas-streampetr-dataset-54; do
  sleep 2
done

mkdir -p "$(dirname "$REPORT")"
if ! jq -e '.valid == true and .samples_checked == 21600 and .images_checked == 86400' \
  "$REPORT" >/dev/null 2>&1; then
  echo "[VALIDATE] checking all 21,600 samples and 86,400 image files"
  /usr/bin/python3 "$VALIDATOR" >"$REPORT"
fi
echo "[TRAIN] dataset valid; starting StreamPETR fine-tuning"
exec "$ROOT/train_carla_4cam.sh"
