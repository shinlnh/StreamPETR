#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="${REPO_ROOT:-/workspace/StreamPETR}"
SMOKE_WORK_DIR="${SMOKE_WORK_DIR:-/outputs/stream_petr_town04_scratch_smoke}"
CONFIG="projects/configs/StreamPETR/stream_petr_r50_flash_704_bs1_seq_town04.py"

cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}/mmdetection3d:${REPO_ROOT}:${PYTHONPATH:-}"
export TORCH_HOME="${REPO_ROOT}/.torch-cache"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"

if ! ldconfig -p | grep -q 'libGL\.so\.1'; then
    echo "[smoke] Installing OpenCV runtime libraries missing from the base image"
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0
    rm -rf /var/lib/apt/lists/*
fi

echo "[smoke] Installing the pinned StreamPETR runtime dependencies"
python -m pip install --no-cache-dir --disable-pip-version-check \
    -r tools/carla_town04/train_requirements_py38.txt
python -m pip install --no-cache-dir --disable-pip-version-check \
    mmcv-full==1.7.0 \
    -f https://download.openmmlab.com/mmcv/dist/cu116/torch1.13.0/index.html

echo "[smoke] Verifying GPU, versions, and representative files from all 27 cases"
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
python - <<'PY'
import pickle
from pathlib import Path

import mmcv
import mmdet
import mmseg
import torch
import torchvision

assert torch.cuda.is_available(), "CUDA is unavailable inside the Hugging Face Job"
cfg = mmcv.Config.fromfile(
    "projects/configs/StreamPETR/"
    "stream_petr_r50_flash_704_bs1_seq_town04.py"
)
assert cfg.model.img_backbone.pretrained is None
assert cfg.load_from is None and cfg.resume_from is None
assert cfg.fp16 is None
assert cfg.optimizer_config.type == "GradientCumulativeOptimizerHook"
assert not cfg.model.pts_bbox_head.transformer.decoder.transformerlayers.attn_cfgs[1].fp16

dataset = Path("data/carla_town04_9weather_3maneuver_8100")
split_files = {
    "train": "carla_town04_temporal_infos_train_6weather.pkl",
    "val": "carla_town04_temporal_infos_val_3weather.pkl",
}
expected_counts = {"train": 5400, "val": 2700}
expected_cases = {"train": 18, "val": 9}
seen_classes = set()
for split, filename in split_files.items():
    with (dataset / filename).open("rb") as handle:
        payload = pickle.load(handle)
    assert len(payload["infos"]) == expected_counts[split], len(payload["infos"])
    representative = {}
    for info in payload["infos"]:
        case = info.get("case_token", info["scene_token"])
        representative.setdefault(case, info)
        seen_classes.update(info["gt_names"])
        for labels2d in info["labels2d"]:
            if len(labels2d):
                assert int(labels2d.min()) >= 0
                assert int(labels2d.max()) < 5, labels2d
    assert len(representative) == expected_cases[split], len(representative)
    for info in representative.values():
        assert len(info["cams"]) == 6
        for camera in info["cams"].values():
            assert Path(camera["data_path"]).is_file(), camera["data_path"]

assert seen_classes == {"car", "truck", "bus", "motorcycle", "bicycle"}, seen_classes

print("torch={}; torchvision={}; cuda={}".format(
    torch.__version__, torchvision.__version__, torch.version.cuda
))
print("mmcv={}; mmdet={}; mmseg={}".format(
    mmcv.__version__, mmdet.__version__, mmseg.__version__
))
print("dataset_frames=8100; cases=27; representative_images_checked=162")
PY

echo "[smoke] Running four micro-batches and one accumulated optimizer update"
rm -rf "${SMOKE_WORK_DIR}"
python tools/train.py "${CONFIG}" \
    --work-dir "${SMOKE_WORK_DIR}" \
    --no-validate \
    --seed 42 \
    --deterministic \
    --cfg-options \
        runner.max_iters=4 \
        checkpoint_config.interval=4 \
        checkpoint_config.max_keep_ckpts=1 \
        evaluation.interval=5 \
        log_config.interval=1 \
        data.workers_per_gpu=0 \
        load_from=None \
        resume_from=None

CHECKPOINT="${SMOKE_WORK_DIR}/iter_4.pth"
test -s "${CHECKPOINT}"
TRAIN_LOG="$(find "${SMOKE_WORK_DIR}" -maxdepth 1 -type f -name '*.log' -print -quit)"
test -n "${TRAIN_LOG}"
ITERATION_LINE="$(grep -E 'Iter \[4/4\]' "${TRAIN_LOG}")"
if grep -Eq 'grad_norm: (inf|nan)' <<<"${ITERATION_LINE}"; then
    echo "[smoke] FAIL: the single training iteration produced a non-finite gradient norm" >&2
    exit 1
fi
echo "${ITERATION_LINE}"
echo "[smoke] PASS: 4x accumulation -> optimizer -> checkpoint"
ls -lh "${CHECKPOINT}"
sha256sum "${CHECKPOINT}"
