#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_DIR="$ROOT/.venv"
MICROMAMBA="$ROOT/.tools/micromamba"
MMCV_DIR="$ROOT/mmcv"
MMDET3D_DIR="$ROOT/mmdetection3d"
PATCH_DIR="$ROOT/patches"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"

apply_vendor_patch() {
  local repo="$1"
  local patch_file="$2"

  if git -C "$repo" apply --unidiff-zero --check "$patch_file"; then
    git -C "$repo" apply --unidiff-zero "$patch_file"
  elif git -C "$repo" apply --unidiff-zero --reverse --check "$patch_file"; then
    echo "Patch already applied: $(basename "$patch_file")"
  else
    echo "Patch does not match vendor checkout: $patch_file" >&2
    exit 1
  fi
}

mkdir -p "$ROOT/.tools"
if [[ ! -x "$MICROMAMBA" ]]; then
  archive="$(mktemp)"
  curl -L --fail https://micro.mamba.pm/api/micromamba/linux-64/latest -o "$archive"
  tar -xjf "$archive" -C "$ROOT/.tools" --strip-components=1 bin/micromamba
  rm -f "$archive"
fi

if [[ ! -x "$ENV_DIR/bin/python" ]]; then
  "$MICROMAMBA" create -y -p "$ENV_DIR" python=3.10 pip=24.3 numpy=1.23.5
fi

PYTHON="$ENV_DIR/bin/python"
"$PYTHON" -m pip install --upgrade pip wheel ninja packaging

# CUDA 12.8 wheels contain Blackwell (sm_120) support. StreamPETR's original
# torch 1.9/cu111 environment cannot execute on an RTX 5070 Ti.
"$PYTHON" -m pip install \
  torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cu128
"$PYTHON" -m pip install "setuptools==69.5.1"
"$PYTHON" -m pip install \
  "numpy==1.23.5" "opencv-python-headless<4.10" \
  "addict==2.4.0" "pyyaml<7" "yapf<0.41"
if [[ ! -d "$MMCV_DIR/.git" ]]; then
  git clone --depth 1 --branch v1.7.2 \
    https://github.com/open-mmlab/mmcv.git "$MMCV_DIR"
fi
if [[ ! -d "$MMDET3D_DIR/.git" ]]; then
  git clone --depth 1 --branch v1.0.0rc6 \
    https://github.com/open-mmlab/mmdetection3d.git "$MMDET3D_DIR"
fi
apply_vendor_patch "$MMCV_DIR" "$PATCH_DIR/mmcv-1.7.2-blackwell.patch"
apply_vendor_patch \
  "$MMDET3D_DIR" \
  "$PATCH_DIR/mmdetection3d-1.0.0rc6-cuda-eval.patch"
if ! "$PYTHON" -c \
  "import mmcv; from mmcv.ops import box_iou_rotated; assert mmcv.__version__ == '1.7.2'"; then
  MMCV_WITH_OPS=1 TORCH_CUDA_ARCH_LIST="12.0+PTX" MAX_JOBS="${MAX_JOBS:-4}" \
    "$PYTHON" -m pip install --no-build-isolation --no-deps -e "$MMCV_DIR"
fi
"$PYTHON" -m pip install \
  "mmdet==2.28.2" \
  "mmsegmentation==0.30.0" \
  "einops==0.8.2" \
  "fvcore==0.1.5.post20221221" \
  "iopath==0.1.10" \
  "numpy==1.23.5" \
  "numba==0.57.1" \
  "nuscenes-devkit==1.1.11" \
  "opencv-python==4.9.0.80" \
  "pandas==1.5.3" \
  "plyfile==0.9" \
  "pyquaternion==0.9.9" \
  "scikit-image==0.21.0" \
  "setuptools==69.5.1"
"$PYTHON" -m pip install \
  "tensorboard>=2.20,<3" \
  "trimesh==2.35.39"
"$PYTHON" -m pip install --no-deps "lyft-dataset-sdk==0.0.8"
"$PYTHON" -m pip install --no-build-isolation --no-deps -e "$MMDET3D_DIR"

cd "$ROOT"
export PYTHONPATH="$MMCV_DIR:$ROOT:${PYTHONPATH:-}"
"$PYTHON" tools/prepare_carla_checkpoint.py
"$PYTHON" - <<'PY'
import torch
import mmcv
from mmcv.ops import box_iou_rotated
import projects.mmdet3d_plugin
print("torch", torch.__version__, "cuda", torch.version.cuda)
print("mmcv", mmcv.__version__, "StreamPETR plugin OK")
print("gpu", torch.cuda.get_device_name(0))
capability = torch.cuda.get_device_capability(0)
print("capability", capability)
assert capability >= (12, 0), capability
boxes = torch.tensor([[0., 0., 2., 2., 0.]], device="cuda")
assert box_iou_rotated(boxes, boxes).item() == 1.0
PY

echo "Environment ready: $ENV_DIR"
