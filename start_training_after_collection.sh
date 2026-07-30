#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if systemctl --user is-active --quiet adas-streampetr-finetune-54; then
  echo "StreamPETR training queue is already active."
  exit 0
fi

systemctl --user reset-failed adas-streampetr-finetune-54 2>/dev/null || true
systemd-run --user --unit=adas-streampetr-finetune-54 \
  --property="WorkingDirectory=$ROOT" \
  --property="Restart=on-failure" \
  --property="RestartSec=10s" \
  --property="StartLimitIntervalSec=0" \
  "$ROOT/train_after_collection.sh"

echo "Training is queued after collection and full dataset validation."
echo "Follow: journalctl --user -u adas-streampetr-finetune-54 -f"
