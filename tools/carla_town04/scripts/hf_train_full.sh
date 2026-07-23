#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="${REPO_ROOT:-/tmp/StreamPETR}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/outputs}"
RUN_NAME="${RUN_NAME:-stream_petr_town04_6w_train_3w_val_scratch_24e}"
RUN_DIR="${OUTPUT_ROOT}/${RUN_NAME}"
CONFIG="${CONFIG:-projects/configs/StreamPETR/stream_petr_r50_flash_704_bs1_seq_town04.py}"
DATASET_REVISION="${DATASET_REVISION:-2729feccf65a9334378593e2d595ee8482bacb83}"
ITERS_PER_EPOCH="${ITERS_PER_EPOCH:-5400}"
EPOCHS="${EPOCHS:-24}"
CHECKPOINT_INTERVAL="${CHECKPOINT_INTERVAL:-600}"
CHECKPOINT_KEEP_COUNT="${CHECKPOINT_KEEP_COUNT:-6}"
MAX_ITERS=$((ITERS_PER_EPOCH * EPOCHS))
ATTEMPT_ID="$(date -u +%Y%m%dT%H%M%SZ)"
ATTEMPT_DIR="${RUN_DIR}/attempts/${ATTEMPT_ID}"

cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}/mmdetection3d:${REPO_ROOT}:${PYTHONPATH:-}"
export TORCH_HOME="${REPO_ROOT}/.torch-cache"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-max_split_size_mb:128}"
export PYTHONUNBUFFERED=1

if (( ITERS_PER_EPOCH % CHECKPOINT_INTERVAL != 0 )); then
    echo "ERROR: CHECKPOINT_INTERVAL must divide ITERS_PER_EPOCH" >&2
    exit 2
fi

mkdir -p "${RUN_DIR}" "${ATTEMPT_DIR}"
exec > >(tee -a "${RUN_DIR}/launcher.log" "${ATTEMPT_DIR}/launcher.log") 2>&1

echo "[train] StreamPETR Town04 resumable run"
echo "[train] attempt=${ATTEMPT_ID}"
echo "[train] epochs=${EPOCHS} iters_per_epoch=${ITERS_PER_EPOCH} max_iters=${MAX_ITERS}"
echo "[train] checkpoint_interval=${CHECKPOINT_INTERVAL} keep_rolling=${CHECKPOINT_KEEP_COUNT}"
echo "[train] output=${RUN_DIR}"
echo "[train] dataset_revision=${DATASET_REVISION}"

if ! ldconfig -p | grep -q 'libGL\.so\.1'; then
    echo "[train] Installing OpenCV runtime libraries"
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0
    rm -rf /var/lib/apt/lists/*
fi

echo "[train] Installing pinned Python dependencies"
python -m pip install --no-cache-dir --disable-pip-version-check \
    -r tools/carla_town04/train_requirements_py38.txt
python -m pip install --no-cache-dir --disable-pip-version-check \
    mmcv-full==1.7.0 \
    -f https://download.openmmlab.com/mmcv/dist/cu116/torch1.13.0/index.html

cp "${CONFIG}" "${ATTEMPT_DIR}/source_config.py"
cp tools/carla_town04/train_requirements_py38.txt "${ATTEMPT_DIR}/requirements.txt"
cp tools/carla_town04/scripts/hf_train_full.sh "${ATTEMPT_DIR}/launcher_source.sh"
cp tools/carla_town04/watch_epoch_artifacts.py "${ATTEMPT_DIR}/watcher_source.py"
python -m pip freeze > "${ATTEMPT_DIR}/pip_freeze.txt"
nvidia-smi -q > "${ATTEMPT_DIR}/nvidia_smi.txt"

echo "[train] Verifying full dataset and training environment"
export ATTEMPT_DIR DATASET_REVISION EPOCHS ITERS_PER_EPOCH MAX_ITERS
export CHECKPOINT_INTERVAL CHECKPOINT_KEEP_COUNT
python - <<'PY'
from collections import Counter
import json
import os
import pickle
from datetime import datetime, timezone
from pathlib import Path

import mmcv
import mmdet
import mmseg
import torch
import torchvision

dataset = Path("data/carla_town04_9weather_3maneuver_8100")
cfg = mmcv.Config.fromfile(
    "projects/configs/StreamPETR/"
    "stream_petr_r50_flash_704_bs1_seq_town04.py"
)
assert cfg.model.img_backbone.pretrained is None
assert cfg.load_from is None and cfg.resume_from is None
assert cfg.fp16 is None
assert cfg.optimizer_config.type == "GradientCumulativeOptimizerHook"
assert not cfg.model.pts_bbox_head.transformer.decoder.transformerlayers.attn_cfgs[1].fp16

counts = {}
scenes = {}
weather = {}
classes = set()
split_files = {
    "train": "carla_town04_temporal_infos_train_6weather.pkl",
    "val": "carla_town04_temporal_infos_val_3weather.pkl",
}
expected_counts = {"train": 5400, "val": 2700}
expected_weather = {
    "train": {
        "ClearNoon", "CloudyNoon", "WetNoon", "WetCloudyNoon",
        "SoftRainNoon", "MidRainyNoon",
    },
    "val": {"HardRainNoon", "ClearSunset", "ClearNight"},
}
expected_classes = {"car", "truck", "bus", "motorcycle", "bicycle"}
for split in ("train", "val"):
    with (dataset / split_files[split]).open("rb") as handle:
        payload = pickle.load(handle)
    counts[split] = len(payload["infos"])
    scenes[split] = len({info["scene_token"] for info in payload["infos"]})
    weather[split] = dict(Counter(info["weather"] for info in payload["infos"]))
    classes.update(name for info in payload["infos"] for name in info["gt_names"])
    assert counts[split] == expected_counts[split], (
        split, counts[split], expected_counts[split]
    )
    assert set(weather[split]) == expected_weather[split], (
        split, weather[split], expected_weather[split]
    )
    for info in payload["infos"]:
        assert len(info["cams"]) == 6
        for labels2d in info["labels2d"]:
            if len(labels2d):
                assert int(labels2d.min()) >= 0
                assert int(labels2d.max()) < len(expected_classes), labels2d
assert classes == expected_classes, (classes, expected_classes)

assert torch.cuda.is_available()
manifest = {
    "attempt_id": Path(os.environ["ATTEMPT_DIR"]).name,
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "dataset": "BanVienCorp/StreamPETR_3D_Dataset",
    "dataset_revision": os.environ["DATASET_REVISION"],
    "dataset_path": str(dataset),
    "frames": counts,
    "scenes": scenes,
    "weather": weather,
    "split_strategy": "weather_holdout_6_train_3_val",
    "classes": sorted(classes),
    "maneuvers": ["left", "right", "straight"],
    "epochs": int(os.environ["EPOCHS"]),
    "iters_per_epoch": int(os.environ["ITERS_PER_EPOCH"]),
    "max_iters": int(os.environ["MAX_ITERS"]),
    "batch_size": 1,
    "gradient_accumulation": 4,
    "effective_batch_size": 4,
    "workers_per_gpu": 2,
    "training_precision": "fp32",
    "optimizer": "AdamW",
    "learning_rate": 1e-4,
    "warmup_iters": 1000,
    "image_final_dim": [256, 704],
    "checkpoint_every_iters": int(os.environ["CHECKPOINT_INTERVAL"]),
    "rolling_checkpoint_keep_count": int(os.environ["CHECKPOINT_KEEP_COUNT"]),
    "completed_epoch_checkpoints_archived": True,
    "initialization": "scratch_random_weights_no_external_checkpoint",
    "validation": {
        "every_iters": int(os.environ["ITERS_PER_EPOCH"]),
        "metric": "carla_center_distance",
        "distance_thresholds_m": [0.5, 1.0, 2.0, 4.0],
    },
    "versions": {
        "torch": torch.__version__,
        "torchvision": torchvision.__version__,
        "cuda": torch.version.cuda,
        "mmcv": mmcv.__version__,
        "mmdet": mmdet.__version__,
        "mmseg": mmseg.__version__,
    },
}
output = Path(os.environ["ATTEMPT_DIR"]) / "run_manifest.json"
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
(Path(os.environ["ATTEMPT_DIR"]).parent.parent / "latest_run_manifest.json").write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(manifest, sort_keys=True))
PY

echo "[train] Looking for the newest valid resumable checkpoint"
RESUME_CHECKPOINT="$(
    python - "${RUN_DIR}" <<'PY'
import re
import sys
from pathlib import Path

import torch

root = Path(sys.argv[1])
pattern = re.compile(r"(?:^iter_|_iter_)(\d+)\.pth$")
candidates = []
for directory in (root, root / "epochs"):
    for path in directory.glob("*.pth"):
        match = pattern.search(path.name)
        if match:
            candidates.append((int(match.group(1)), path))

for iteration, path in sorted(candidates, reverse=True):
    try:
        payload = torch.load(str(path), map_location="cpu")
        if "state_dict" not in payload or "optimizer" not in payload:
            raise ValueError("checkpoint lacks model or optimizer state")
    except Exception as exc:
        print("[resume] ignoring invalid {}: {}".format(path, exc), file=sys.stderr)
        continue
    print(path)
    break
PY
)"

RESUME_ARGS=()
if [[ -n "${RESUME_CHECKPOINT}" ]]; then
    echo "[train] Resuming model, optimizer and runner from ${RESUME_CHECKPOINT}"
    RESUME_ARGS=(--resume-from "${RESUME_CHECKPOINT}")
else
    echo "[train] No checkpoint in this run; initializing every model weight from scratch"
fi

python tools/carla_town04/watch_epoch_artifacts.py \
    --work-dir "${RUN_DIR}" \
    --iters-per-epoch "${ITERS_PER_EPOCH}" \
    --epochs "${EPOCHS}" \
    --checkpoint-interval "${CHECKPOINT_INTERVAL}" \
    --poll-seconds 30 &
ARTIFACT_WATCHER_PID=$!
TRAIN_PID=""

forward_signal() {
    echo "[train] Received termination signal; preserving completed artifacts"
    if [[ -n "${TRAIN_PID}" ]]; then
        kill -TERM "${TRAIN_PID}" 2>/dev/null || true
    fi
}
trap forward_signal TERM INT

TRAIN_COMMAND=(
    python tools/train.py "${CONFIG}"
    --work-dir "${RUN_DIR}"
    --seed 42
    --deterministic
    "${RESUME_ARGS[@]}"
    --cfg-options
    runner.max_iters="${MAX_ITERS}"
    checkpoint_config.interval="${CHECKPOINT_INTERVAL}"
    checkpoint_config.max_keep_ckpts="${CHECKPOINT_KEEP_COUNT}"
    checkpoint_config.save_optimizer=True
    checkpoint_config.create_symlink=False
    evaluation.interval="${ITERS_PER_EPOCH}"
    log_config.interval=50
    data.workers_per_gpu=2
    load_from=None
    resume_from=None
)
printf '%q ' "${TRAIN_COMMAND[@]}" > "${ATTEMPT_DIR}/training_command.txt"
printf '\n' >> "${ATTEMPT_DIR}/training_command.txt"

set +e
"${TRAIN_COMMAND[@]}" &
TRAIN_PID=$!
wait "${TRAIN_PID}"
TRAIN_EXIT=$?
set -e
TRAIN_PID=""

python tools/carla_town04/watch_epoch_artifacts.py \
    --work-dir "${RUN_DIR}" \
    --iters-per-epoch "${ITERS_PER_EPOCH}" \
    --epochs "${EPOCHS}" \
    --checkpoint-interval "${CHECKPOINT_INTERVAL}" \
    --once || true
kill "${ARTIFACT_WATCHER_PID}" 2>/dev/null || true
wait "${ARTIFACT_WATCHER_PID}" 2>/dev/null || true

if [[ "${TRAIN_EXIT}" -eq 0 ]]; then
    printf 'COMPLETED attempt=%s utc=%s\n' "${ATTEMPT_ID}" "$(date -u +%FT%TZ)" \
        > "${RUN_DIR}/training_status.txt"
    echo "[train] COMPLETED: ${EPOCHS} epochs"
else
    printf 'INTERRUPTED_OR_FAILED exit_code=%s attempt=%s utc=%s\n' \
        "${TRAIN_EXIT}" "${ATTEMPT_ID}" "$(date -u +%FT%TZ)" \
        > "${RUN_DIR}/training_status.txt"
    echo "[train] INTERRUPTED_OR_FAILED: exit_code=${TRAIN_EXIT}" >&2
fi
exit "${TRAIN_EXIT}"
