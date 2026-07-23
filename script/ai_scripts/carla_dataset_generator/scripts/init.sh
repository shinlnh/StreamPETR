#!/bin/bash
set -e
# Source the environment setup script
source ../../carla_script/setup_env_carla.sh || { echo "Error: CARLA initiation failed. Make sure you are in the ai_scripts dir when executing this"; exit 1; }
echo "CARLA setup done."

# Set XLA flags
export XLA_FLAGS="--xla_gpu_cuda_data_dir=/usr/local/cuda-11.1"
echo "XLA setup done."

# Set TensorRT paths (add to existing PATH and LD_LIBRARY_PATH)
TENSORRT_BASE="/prj/es-automotive/adas/common/TensorRT-8.4.1.5"
export PATH="${TENSORRT_BASE}/bin:${PATH}"
export LD_LIBRARY_PATH="${TENSORRT_BASE}/lib:${LD_LIBRARY_PATH}"
echo "TensorRT lib setup done."