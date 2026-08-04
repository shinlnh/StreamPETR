# StreamPETR — nuCarla six-camera 3D detection

Camera-only 3D object detection trained on the [nuCarla](https://arxiv.org/abs/2511.13744)
dataset (Town04 subset). Takes six surround-view images and returns 3D boxes in
the ego/lidar frame, carrying a temporal memory across frames.

| | |
|---|---|
| Architecture | StreamPETR, ResNet-50 backbone + CPFPN neck |
| Classes | `car, truck, bus, motorcycle, bicycle, pedestrian` |
| Input | 6 cameras, 1600×900 → 704×256 after preprocessing |
| mAP@0.25 (3D IoU) | 0.5120 on nuCarla Town04 val |
| mAP (nuScenes center-distance) | see *Accuracy* below |
| Latency | 4.5 ms/frame end-to-end on an RTX 5070 Ti |

---

## 1. Package contents

```
stream_petr_6cam_encoder.onnx              95 MiB   images -> image features
stream_petr_6cam_temporal_head.onnx        47 MiB   features + memory -> boxes + memory
stream_petr_6cam_encoder_fp16.engine       51 MiB   TensorRT 10.9, FP16
stream_petr_6cam_temporal_head_fp16.engine 29 MiB   TensorRT 10.9, FP16
deployment_manifest.json                            exact tensor specs + checksums
test_inputs/                                        one reference frame, raw float32
reference_outputs/                                  what PyTorch produced for it
```

The two `.engine` files are **built for a specific GPU and TensorRT version**.
On any other GPU, rebuild them from the `.onnx` files — see §8. The ONNX graphs
themselves are portable.

## 2. Requirements

```bash
pip install tensorrt==10.9.0.34 numpy torch
```

`torch` is used only for its CUDA allocator and stream handling; no training
dependency (mmcv, mmdet3d) is needed for inference.

The bundled engines were serialized by TensorRT **10.9.0.34** on an
**RTX 5070 Ti** (CUDA 12.8, driver 595). TensorRT refuses to load engines
serialized by a different version, so match it or rebuild.

## 3. Quick start

```python
import numpy as np
from streampetr_trt_runner import StreamPETRRunner
from carla_stream_petr_postprocess import decode_stream_petr

runner = StreamPETRRunner(
    "stream_petr_6cam_encoder_fp16.engine",
    "stream_petr_6cam_temporal_head_fp16.engine",
)

runner.reset()                      # start of a sequence
for frame in sequence:
    out = runner(
        images=frame.images,        # [1, 6, 3, 256, 704] float32, normalized
        timestamp=frame.seconds,    # float, seconds
        ego_pose=frame.lidar2global,        # [1, 4, 4] float32
        ego_pose_inv=frame.global2lidar,    # [1, 4, 4] float32
    )
    det = decode_stream_petr(
        out["class_logits"], out["bbox_predictions"], score_threshold=0.35
    )
    # det["boxes_3d"], det["scores"], det["labels"], det["class_names"]
```

Call `runner.reset()` whenever the sequence breaks (new clip, sensor dropout).
It zeroes the memory and makes the next frame set `prev_exists = 0`.

## 4. Camera rig — this must match

The 3D position embedding is **baked into the exported graph** for the rig
below. Feeding images from a different camera layout produces meaningless
geometry, silently. If your rig differs, re-export from the checkpoint.

Offsets are in the ego frame (x forward, y left, z up, metres), yaw is
counter-clockwise in degrees:

| Camera | x | y | z | yaw |
|---|---|---|---|---|
| `CAM_FRONT` | 1.901 | 0.016 | 1.511 | 0.3° |
| `CAM_FRONT_LEFT` | 1.524 | 0.495 | 1.509 | 55.2° |
| `CAM_FRONT_RIGHT` | 1.551 | −0.493 | 1.496 | −56.4° |
| `CAM_BACK` | 0.028 | 0.003 | 1.579 | 179.9° |
| `CAM_BACK_LEFT` | 1.036 | 0.485 | 1.591 | 108.6° |
| `CAM_BACK_RIGHT` | 1.015 | −0.481 | 1.562 | −110.8° |

All six share the same intrinsics — 1600×900, horizontal FOV 65°:

```
fx = fy = 1255.7484    cx = 800.0    cy = 450.0
```

which is what CARLA produces from `image_size_x=1600, image_size_y=900,
fov=65`. The reference lidar sits at `[0.943713, 0.0, 1.84023]` with rotation
`[0.707796, -0.006492, 0.010646, -0.706307]` (w, x, y, z); all detections are
returned in that lidar frame.

**Camera order matters.** Stack them along axis 1 in exactly this order:

```
0 CAM_FRONT   1 CAM_FRONT_LEFT   2 CAM_FRONT_RIGHT
3 CAM_BACK    4 CAM_BACK_LEFT    5 CAM_BACK_RIGHT
```

## 5. Preprocessing

Per camera, starting from the raw 1600×900 RGB frame:

1. **Resize** to 704×396 (scale factor 0.44, aspect preserved).
2. **Crop** `(left=0, top=140, right=704, bottom=396)` — the bottom 256 rows,
   full width.
3. **Normalize** as RGB float32: `(pixel − mean) / std` with
   `mean = [123.675, 116.28, 103.53]`, `std = [58.395, 57.12, 57.375]`.
4. Stack to `[1, 6, 3, 256, 704]`, channels-first.

The intrinsics that correspond to the preprocessed image are

```
fx = fy = 552.529    cx = 352.0    cy = 58.0
```

(1255.7484 × 0.44 = 552.529; cy = 450 × 0.44 − 140 = 58). You do not need to
feed these anywhere — they are already baked in — but they are what the model
assumes, so a different resize or crop will break it.

## 6. Tensor specification

**Encoder**

| | name | dtype | shape |
|---|---|---|---|
| in | `images` | float32 | `[1, 6, 3, 256, 704]` |
| out | `image_features` | float32 | `[1, 6, 256, 16, 44]` |

**Temporal head**

| | name | dtype | shape |
|---|---|---|---|
| in | `image_features` | float32 | `[1, 6, 256, 16, 44]` |
| in | `timestamp` | float32 | `[1]` |
| in | `ego_pose` | float32 | `[1, 4, 4]` |
| in | `ego_pose_inv` | float32 | `[1, 4, 4]` |
| in | `prev_exists` | float32 | `[1]` |
| in | `pre_memory_embedding` | float32 | `[1, 1024, 256]` |
| in | `pre_memory_reference_point` | float32 | `[1, 1024, 3]` |
| in | `pre_memory_timestamp` | float32 | `[1, 1024, 1]` |
| in | `pre_memory_egopose` | float32 | `[1, 1024, 4, 4]` |
| in | `pre_memory_velocity` | float32 | `[1, 1024, 2]` |
| out | `class_logits` | float32 | `[1, 900, 6]` |
| out | `bbox_predictions` | float32 | `[1, 900, 10]` |
| out | `post_memory_*` | float32 | same shapes as `pre_memory_*` |

`ego_pose` is the **lidar-to-global** 4×4 matrix, i.e. `ego2global @ lidar2ego`,
and `ego_pose_inv` is its inverse. `timestamp` is in seconds. Both are used to
compensate ego motion when reading the memory, so they must be consistent
across frames of a sequence — an arbitrary but fixed global origin is fine.

## 7. Temporal protocol

The head is stateful. Per frame:

1. Set `prev_exists = 0.0` on the first frame of a sequence, `1.0` afterwards.
2. Feed each `post_memory_X` from frame *t* into `pre_memory_X` at frame *t+1*.
3. On the first frame, pass zeros for every `pre_memory_*`.

`StreamPETRRunner` does all of this; the raw engines do not.

Memory holds 1024 slots: the newest 256 come from this frame's top-scoring
queries, the remaining 768 are the previous frames shifted down.

## 8. Rebuilding the engines

Required when changing GPU, TensorRT version or precision:

```bash
python build_streampetr_engines.py --deployment-dir . --cameras 6
python build_streampetr_engines.py --deployment-dir . --cameras 6 --fp32   # FP32
```

Takes about 15 seconds. The script uses the TensorRT Python API, so the engine
is serialized by the same library that will load it — engines built by a
`trtexec` binary from a different TensorRT package will fail to deserialize
even when the version string matches.

To re-export the ONNX itself (different checkpoint, rig or resolution) you need
the training environment:

```bash
python tools/deployment/export_carla_stream_petr.py \
  --config projects/configs/StreamPETR/stream_petr_r50_nucarla_town04.py \
  --checkpoint <checkpoint>.pth \
  --output-dir <output>
```

## 9. Output format

`decode_stream_petr` applies the model's NMS-free coder and returns:

| key | shape | meaning |
|---|---|---|
| `boxes_3d` | `[N, 9]` | `cx, cy, cz, w, l, h, yaw, vx, vy` |
| `scores` | `[N]` | sigmoid confidence |
| `labels` | `[N]` | class index, 0–5 |
| `class_names` | `[N]` | class name strings |

Coordinates are metres in the lidar frame, `yaw` is radians, velocities are
m/s. Boxes are filtered to `±61.2 m` in x/y and `±10 m` in z. There is no NMS —
the model is trained to emit one query per object — so a score threshold is the
only filtering usually needed. 0.35 is a reasonable starting point.

## 10. Accuracy

On the nuCarla Town04 validation split (29 scenes, 1160 frames):

| Metric | Value |
|---|---|
| mAP, 3D IoU @0.25 | 0.5120 |
| mAP, nuScenes center-distance (0.5/1/2/4 m) | 0.4638 † |
| mATE @2 m | 0.872 m † |

† measured on the earlier checkpoint; the numbers for this one are expected to
be slightly better but have not been re-measured.

Per class, 3D IoU @0.25 — smaller objects score far lower under an IoU metric
because a 0.42 m wide pedestrian box tolerates only ~0.25 m of lateral error,
against ~1.2 m for a car. The center-distance metric the nuCarla paper reports
is far more forgiving for them (pedestrian AP rises from 0.018 to 0.309).

**Comparison to the paper.** nuCarla reports mAP 0.745 for PETR, but that is
center-distance on the full dataset with a VoVNet backbone at 1600×640 and 150
GPU-hours. This model saw one map out of seven, a ResNet-50 at 704×256 (5.7×
fewer pixels) and ~5 GPU-hours. The numbers are not directly comparable.

## 11. Verification

The exported graph was checked against the PyTorch model at three levels.

**Graph export** — the ONNX wrapper reproduces the native PyTorch head exactly:

```
class_logits     max |Δ| = 0.0
bbox_predictions max |Δ| = 0.0
```

**FP16 engine vs PyTorch FP32**, on the bundled reference frame:

| tensor | correlation | relative error |
|---|---|---|
| `image_features` | 0.999996 | 2.9e-3 |
| `bbox_predictions` | 0.999992 | 2.8e-2 |
| `class_logits` | 0.999546 | 7.6e-2 |

**Decoded detections** — the number and placement of detections agree:

```
detections @score>0.2:  TensorRT 17    PyTorch 17
position agreement:     0.02 - 0.10 m for every detection
```

Two pairs swap rank order, both between detections whose scores are tied to
within 0.003. Their box positions match the partner row exactly, so the
detection set is the same; only the ordering of near-ties differs.

**Known FP16 effect.** Of the 1024 memory slots, the 768 carried over from
previous frames are bit-exact. Within the 256 slots selected fresh each frame,
FP16 scoring picks a slightly different subset near the top-256 cut-off, so
those rows differ from PyTorch. The effect on the detections themselves is what
the table above measures — small. Its accumulation over a long sequence has not
been measured; build the FP32 engine if you need to rule it out.

## 12. Performance

RTX 5070 Ti, FP16, batch 1:

| stage | latency |
|---|---|
| encoder | 1.93 ms |
| temporal head | 0.91 ms |
| **GPU total** | **2.84 ms** (~350 fps) |
| end-to-end incl. host copies | 4.51 ms (~220 fps) |

Keeping the images on the GPU and reading the outputs there avoids most of the
1.7 ms gap.

## 13. Files you also need

Copy these two alongside the engines — they are the runtime, not part of the
training package:

- `streampetr_trt_runner.py` — engine wrapper and memory handling
- `carla_stream_petr_postprocess.py` — decoder from raw head outputs to boxes
- `build_streampetr_engines.py` — engine builder, for other GPUs
