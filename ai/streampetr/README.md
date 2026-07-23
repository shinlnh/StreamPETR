# StreamPETR

## Introduction
StreamPETR is a 3D object detection model in the PETR model family. It leverage both spacial and temporal information to achieve higher scene understanding. The architecture also use object-centric query technique to achieve much higher inference speed. This is the original [StreamPETR repo](https://github.com/exiawsh/StreamPETR).

This folder contains the Python and C++ implementation of StreamPETR and utilizing TensorRT for hardware acceleration.

## Python script use guide
The Python script main purpose is for demo and testing. In the `scripts/` folder, you will find 3 python scripts:
- `realtime_infer.py`: this script is for inferencing StreamPETR using nuScene dataset
- `pth2onnx.py`: this is the script for exporting the pytorch trained model to onnx format
- `tensorrt_test.py`: this script is for benchmarking the speed of the model when ran with TensorRT

### Preparation
#### 1. Install StreamPETR
```shell
# Create conda environment and activate
conda create --name streampetr_env python=3.10 -y
conda activate streampetr_env

# Install torch
pip install torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 --index-url https://download.pytorch.org/whl/cu121

# Install flash attention
pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.3/flash_attn-2.7.3+cu12torch2.1cxx11abiFALSE-cp310-cp310-linux_x86_64.whl

# Install MMCV core
pip install mmcv-full==1.7.2 -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.1.0/index.html
pip install mmdet==2.28.2
pip install mmsegmentation==0.30.0

# Fix some version and package dependencies
pip install "numpy<2" "opencv-python<4.11"
pip install setuptools==69.5.1 wheel
pip install ipython fvcore

# Clone StreamPETR
git clone https://github.com/exiawsh/StreamPETR
cd ./StreamPETR

# Install MMDetection3D
git clone https://github.com/open-mmlab/mmdetection3d.git
cd mmdetection3d
git checkout v1.0.0rc6
sed -i 's/numba==0.53.0/numba>=0.56.0,<0.58.0/g' requirements/runtime.txt
sed -i 's/networkx>=2.2,<2.3/networkx>=2.8,<3.0/g' requirements/runtime.txt
sed -i "s/mmcv_maximum_version = '1.7.0'/mmcv_maximum_version = '1.7.2'/g" mmdet3d/__init__.py
pip install -e . --no-build-isolation
cd ..

# Patch mmcv for pytorch 2+
SCATTER_FILE=$(python -c "import mmcv; import os; print(os.path.join(os.path.dirname(mmcv.__file__), 'parallel/_functions.py'))")
sed -i "s/_get_stream(device) for device in target_gpus/_get_stream(torch.device(f'cuda:{device}')) for device in target_gpus/g" $SCATTER_FILE

# Patch attention file
ATTN_FILE="projects/mmdet3d_plugin/models/utils/attention.py"
sed -i 's/from flash_attn.flash_attn_interface import flash_attn_unpadded_kvpacked_func/from flash_attn.flash_attn_interface import flash_attn_varlen_kvpacked_func as flash_attn_unpadded_kvpacked_func/g' $ATTN_FILE
```

#### 2. Install ONNX and TensorRT and dependencies 
```shell
# ONNX
pip install onnx onnxruntime onnxsim

# TensorRT
pip install tensorrt_cu12_libs tensorrt_cu12_bindings tensorrt-cu12
pip install pycuda

# open3D for visualization
pip install open3d
```

### Export ONNX
To export trained pytorch model, follow this instruction:

#### 1. Download pretrained model (or use your trained model)
```shell
mkdir models
cd models
wget https://github.com/exiawsh/storage/releases/download/v1.0/stream_petr_r50_flash_704_bs2_seq_428q_nui_60e.pth
cd ..
```

#### 2. Export to ONNX
Copy the `scripts/pth2onnx.py` to `StreamPETR/tools/` folder first.

Then export with:
```shell
# Export python path (run each time restart terminal)
cd /path/to/your/StreamPETR/
export PYTHONPATH=${pwd}:$PYTHONPATH

# Export 2 stages
CUBLAS_WORKSPACE_CONFIG=:4096:8 python tools/pth2onnx.py projects/configs/test_speed/stream_petr_r50_704_bs2_seq_428q_nui_speed_test.py models/stream_petr_r50_flash_704_bs2_seq_428q_nui_60e.pth --section extract_img_feat
CUBLAS_WORKSPACE_CONFIG=:4096:8 python tools/pth2onnx.py projects/configs/test_speed/stream_petr_r50_704_bs2_seq_428q_nui_speed_test.py models/stream_petr_r50_flash_704_bs2_seq_428q_nui_60e.pth --section pts_head_memory

# Export single stages (only for testing speed)
CUBLAS_WORKSPACE_CONFIG=:4096:8 python tools/pth2onnx.py projects/configs/test_speed/stream_petr_r50_704_bs2_seq_428q_nui_speed_test.py models/stream_petr_r50_flash_704_bs2_seq_428q_nui_60e.pth
```

After exporting finished, you will get these files:
- `position_encoder_*.bin`: Include B1, B2, W1, W2 files. These are raw float weight files for the position encoder. It's for the app to load.
- `pseudo_reference_points.bin`: Like the above, this is raw float weight for reference points.
- `extract_img_feat.onnx` and `pts_head_memory.onnx`: These are the ONNX model directly exported by pytorch, you can ignore/delete them.
- `simplify_extract_img_feat.onnx` and `simplify_pts_head_memory.onnx`: These are the simplified version of the two above. You should use these to build TensorRT engine (or for other API).

### Building TensorRT engine for C++ App
After exporting to ONNX, you can build the TRT engine for backbone and detector with these commands:
```shell
trtexec --onnx=/path/to/model.onnx --saveEngine=/path/to/save/model.engine
```
For backbone, you should use `--fp16` flag to enable FP16 PTQ.\
For detector, I advise you to only use `--fp16` with ONNX opset 17 or higher. Otherwise, you should force FP32 precision for LayerNorm operations. To do so, you will have to use TensorRT API to write your own engine builder. The main steps can be seen in `scripts/realtime_infer.py:1184` **_layernorm_force_fp32** function.

**NOTE**: You don't have to build TensorRT engine to use with python demo script. The python script is capable of building its own engine.

### Running Python demo
1. Prepare nuScenes dataset

    Go to [nuScenes download page](https://www.nuscenes.org/download), create an account and download the mini v1.0 dataset (or bigger one if you want).

    Extract the dataset to `StreamPETR/data/nuscenes/`.
    
    Then run this command.

    ```shell
    python tools/create_data_nusc.py --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes2d --version v1.0-mini
    ```

    Change the version to suit the one you downloaded.

2. Open the `scripts/realtime_infer.py` script and update:

    - `WEIGHT_PATH` to the folder that store weights
    - `NUSCENES_PATH` to the dataset path
    - `NUSCENES_VER` to the version you downloaded
    - `MODE` to correct exported stage setting
    - `ENABLE_VISUALIZE` to True if want to see live detection

3. Run the demo:

    ```shell
    python realtime_infer.py
    ```


## C++ implementation

The C++ implementation is stored in `app/` folder. It is written in format similar to Autoware node for easier adaptation to micro-service architecture.

The node is incomplete as there is no build script for it, but you can still use the code inside and adapt it to use somewhere else. If you wish to run it as a node then write the build script. The node dependency is basic ROS2 and Autoware, with Eigen 3.4+ and TensorRT.

Structure of the source code:
- `stream_petr_node.*`: This is the code for launching ROS node and managing communication.
- `streampetr.*`: This contains the main logic for StreamPETR. During initialization, it takes a config struct that encapsulates all needed infomation. Then you can simply call infer which takes an input struct and then returns an output struct.
- `decoder.*`, `memory.*`, `trt_engine.hpp`: Contain definitions and functions for managine TensorRT engine, the model's memory and post processing which are accelerated by CUDA.
- `utils.*`: Some function just to keep the code cleaner.
