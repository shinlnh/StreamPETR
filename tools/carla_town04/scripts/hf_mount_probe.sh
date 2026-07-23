#!/usr/bin/env bash
set -Eeuo pipefail

cd /inputs
sha256sum -c streampetr_code.tar.gz.sha256
test -f \
    /hf_dataset/carla_town04_9weather_3maneuver_8100/carla_town04_temporal_infos_train_6weather.pkl
test -f \
    /hf_dataset/carla_town04_9weather_3maneuver_8100/carla_town04_temporal_infos_val_3weather.pkl

mkdir -p /outputs/probes
cp /hf_dataset/carla_town04_9weather_3maneuver_8100/validation_report.json \
    /outputs/probes/validation_report.json
date -u -Is | tee /outputs/probes/mount_probe_utc.txt
echo "MOUNT_PROBE_PASS"
