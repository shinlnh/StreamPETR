#!/usr/bin/env bash
set -Eeuo pipefail

cd /inputs
sha256sum -c streampetr_code.tar.gz.sha256
mkdir -p /tmp/StreamPETR-extract-probe
tar --no-same-owner --no-same-permissions \
    -xzf /inputs/streampetr_code.tar.gz \
    -C /tmp/StreamPETR-extract-probe
test -f /tmp/StreamPETR-extract-probe/tools/train.py
test -f \
    /tmp/StreamPETR-extract-probe/projects/configs/StreamPETR/stream_petr_r50_flash_704_bs1_seq_town04.py
test -f /tmp/StreamPETR-extract-probe/tools/carla_town04/scripts/hf_train_full.sh
echo "EXTRACT_PROBE_PASS"
