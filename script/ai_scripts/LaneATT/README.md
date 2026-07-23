# LaneATT

### 1. Installation
#### Prerequisites
- Python (tested on 3.8.10)
- CUDA (tested on 11.1)

Check `../carla_dataset_generator/INSTALLATION.md` for CUDA install instruction.

#### Python packages
Feel free to use any virtual environment (venv, conda) as you wish.
```
pip install torch==1.10.1+cu111 torchvision==0.11.2+cu111 torchaudio==0.10.1 -f https://download.pytorch.org/whl/cu111/torch_stable.html
pip install -r requirements.txt
export CUDA_HOME=/usr/local/cuda-11.1
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export PATH=$CUDA_HOME/bin:$PATH
cd lib/nms; python setup.py install; cd -
```


### 2. Preparation
#### Dataset
Currently, LaneATT supports 4 datasets: TuSimple, CULane, LLAMAS, Carla.
For development, we use Carla dataset generated from `script/ai_scripts/carla_dataset_generator`.
Follow the instruction there to generate one.

#### Config file
To train a model, a config file is needed. This config file specify which dataset to use, backbone's size and other parameters.
You can find example configs in cfgs/

#### Generate Anchor frequency
Anchor frequency helps filter out anchors that are rarely used to speed-up inference.
Once dataset and config are available, run this command.
```
python utils/gen_anchor_mask.py --cfg cfgs/laneatt_carla_resnet34.yaml --output data/carla_anchors_freq.pt
```
Afterward, update the **anchors_freq_path** value in config file accordingly.


### 3. Procedure
#### Training:
To train LaneATT with the ResNet-34 backbone on Carla generated dataset, run:
```
python main.py train --exp_name laneatt_r34_carla --cfg cfgs/laneatt_carla_resnet34.yaml
```
Directory `experiments/laneatt_r34_carla` will be created and contains related data (model checkpoints, logs, evaluation results, etc)

To resume training, use `--resume`:
```
python main.py train --exp_name laneatt_r34_carla --cfg cfgs/laneatt_carla_resnet34.yaml --resume
```

#### Evaluate:
```
python main.py test --exp_name laneatt_r34_carla
```
Use `--epoch` flag to specify the checkpoint to use (eg `--epoch 2`).
To **visualize the predictions**, use flag `--view all`.

#### Export:
```
python main.py export --exp_name laneatt_r34_carla
```
You can specify output path using `--output_path` flag.
Export also receive `--epoch` flag to specify checkpoint.

#### Build engine
First, specify the path of `trtexec`

- On PC:
```
TENSORRT_BASE="/prj/es-automotive/adas/common/TensorRT-8.4.1.5"
export PATH="${TENSORRT_BASE}/bin:${PATH}"
export LD_LIBRARY_PATH="${TENSORRT_BASE}/lib:${LD_LIBRARY_PATH}"
```

- On Board:
```
export PATH="/usr/src/tensorrt/bin/:$PATH"
```

Then
```
trtexec --onnx=laneatt_r34_carla.onnx --saveEngine=laneatt.engine --fp16 --explicitBatch
```
This will build engine and save to laneatt.engine.

**For the rest of infomation about LaneATT, check [LaneATT - Github](https://github.com/lucastabelini/LaneATT)**