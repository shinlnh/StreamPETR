# CARLA four-camera StreamPETR deployment

Run the complete conversion and local TensorRT build from the repository root:

```bash
./export_carla_tensorrt.sh
```

Artifacts are written to:

```text
work_dirs/stream_petr_r50_carla_4cam_finetune/deployment/
```

The deployment has two fixed-shape FP16 engines:

1. `stream_petr_carla_4cam_encoder_fp16.engine`
   - input `images`: FP32 `[1, 4, 3, 256, 704]`
   - output `image_features`: FP32 `[1, 4, 256, 16, 44]`
2. `stream_petr_carla_4cam_temporal_head_fp16.engine`
   - consumes `image_features`, timestamp, ego poses and five memory tensors
   - returns class logits, encoded 3D boxes and the five memory tensors for the
     next frame

Set `prev_exists=0` on the first frame or when changing scene. Set it to `1`
for each subsequent frame, and feed every `post_memory_*` output back to its
matching `pre_memory_*` input. The exact tensor shapes and preprocessing values
are recorded in `deployment_manifest.json`.

The engine's `class_logits` and `bbox_predictions` match the original
`NMSFreeCoder` inputs. Use `decode_stream_petr` from
`tools/deployment/carla_stream_petr_postprocess.py` to obtain scored 3D boxes.

The TensorRT engines were built for TensorRT 10.9, CUDA 12.8 and the local
NVIDIA GeForce RTX 5070 Ti. Rebuild them with the script when changing the GPU,
TensorRT version, camera rig, input resolution or checkpoint.
