# StreamPETR fine-tuning on CARLA

This directory contains the upstream official StreamPETR source at commit
`95f6470`, MMDetection3D `v1.0.0rc6`, and the released R50 90-epoch nuScenes
checkpoint.

The CARLA configuration changes the input from six nuScenes views to four
synchronized CARLA views and changes both classification heads from 10
nuScenes categories to the six dynamic categories available consistently in
CARLA:

`car, truck, bus, motorcycle, bicycle, pedestrian`.

The 960×540 source images follow `config/carla/vehicle_1.json`; augmentation
still produces StreamPETR's 704×256 network input.

The checkpoint adapter removes only class-dependent output tensors. All
nuScenes-trained image, geometry, query, temporal-memory and box-regression
weights remain available for domain fine-tuning.

## Environment

The upstream environment (`torch 1.9 + CUDA 11.1 + flash-attn 0.2`) predates
the RTX 5070 Ti. Run:

```bash
./setup_blackwell_env.sh
```

The local setup uses a Python 3.10 micromamba environment and CUDA 12.8 PyTorch
wheels. The CARLA config replaces legacy FlashAttention with StreamPETR's
regular fp16 attention.

## Train

```bash
./train_carla_4cam.sh
```

The effective batch size is 16 on one GPU. The base fine-tuning rate is
`2e-5` per sample and is scaled to `3.2e-4` for that batch size. Training runs
for 15 epochs and writes a checkpoint after every epoch.

Validation does not call the nuScenes database API. `CarlaStreamPetrDataset`
evaluates directly against the simulator-native boxes embedded in the info
files and reports class-wise 3D IoU AP/recall at 0.25 and 0.50.

To queue validation and training behind the long CARLA collection job:

```bash
./start_training_after_collection.sh
journalctl --user -u adas-streampetr-finetune-54 -f
```

The queue refuses to train unless `summary.json` contains exactly 54 complete
scenes, 21,600 samples, a 17,200-sample train split, and a 4,400-sample
validation split. It then validates every one of the 86,400 image paths before
launching the 15-epoch fine-tune.

Single-GPU inference and CARLA 3D IoU evaluation:

```bash
PYTHONPATH="$PWD/mmcv:$PWD" .venv/bin/python tools/test.py \
  projects/configs/StreamPETR/stream_petr_r50_carla_4cam_finetune.py \
  work_dirs/stream_petr_r50_carla_4cam_finetune/latest.pth \
  --eval bbox
```
