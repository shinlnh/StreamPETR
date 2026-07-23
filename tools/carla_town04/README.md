# CARLA Town04 dataset for StreamPETR

This tool collects a six-camera temporal 3D detection dataset on CARLA
`Town04_Opt` and writes the PKL schema consumed by StreamPETR's
`CustomNuScenesDataset`.

The CARLA/podman launcher was copied from
`ad_sim_dev_launch/scripts/carla/collect_dataset` and adapted to run from this
repository. The original collector itself targets LaneATT (one forward camera
and lane-marking labels), so its image/label output is **not** suitable for
StreamPETR. This version adds the pieces StreamPETR needs:

- six synchronized RGB cameras in nuScenes camera order;
- a six-camera rig using the nuScenes reference positions, orientations and
  fields of view;
- camera intrinsics and camera-to-ego extrinsics;
- ego pose and timestamps for temporal training;
- CARLA ground-truth 3D boxes, class names, yaw and velocity;
- per-camera 2D boxes, centers and depth for StreamPETR's auxiliary 2D head;
- episode-level train/validation splitting.

Generated data, ZIP files, logs and the old local pip cache (about 26 GB in the
source directory) are deliberately not copied.

## Prerequisites

The adjacent `ad_sim_dev_launch` checkout must already be configured, with
these podman images visible:

- `carla-ros-bridge:foxy`
- `carlasim/carla:0.9.13`

By default the launcher resolves it as `../ad_sim_dev_launch`. Override that
location when needed:

```bash
export AD_SIM_DEV_LAUNCH_DIR=/path/to/ad_sim_dev_launch
```

## Preflight

From the StreamPETR repository root:

```bash
bash tools/carla_town04/scripts/collect_town04.sh --check
```

This starts or reuses the CARLA compose service and verifies that the tool
container can reach port 2000. The ROS bridge, ADAS service and HMI are stopped
while collection owns CARLA synchronous mode.

## Smoke collection

Start small before an overnight run:

```bash
bash tools/carla_town04/scripts/collect_town04.sh \
  --episodes 2 \
  --frames-per-episode 20 \
  --vehicles 20 \
  --output tools/carla_town04/data/smoke

python3 tools/carla_town04/validate_dataset.py tools/carla_town04/data/smoke
```

The smoke output is ignored by Git. The output directory must be empty for a
new run.

## nuScenes-reference camera rig

The default rig uses 1600x900 images and the nuScenes camera channel order.
Its horizontal fields of view are 70 degrees for the front and four corner
cameras, and 110 degrees for the rear camera. The mount translations and
rotations in `config.py` are converted from a representative nuScenes
`calibrated_sensor` record. nuScenes calibrates each physical capture vehicle
separately, so this profile is a canonical reference rig rather than one set of
numbers shared by every nuScenes scene.

The older `data/carla_town04` collection used the hybrid 30-degree ADAS front
camera and remains untouched. New collections default to the separate
`data/carla_town04_nuscenes_rig` directory so the two calibrations cannot be
mixed accidentally.

## Default Town04 collection

```bash
bash tools/carla_town04/scripts/collect_town04.sh
```

Defaults are defined in `config.py`: ten 20-second episodes, 40 saved frames per
episode, 60 traffic vehicles, 10 FPS simulation, and one saved sample every
five ticks (2 Hz). The episode split produces 320 training frames and 80
validation frames under `data/carla_town04_nuscenes_rig`. Override settings on
the command line, for example:

```bash
bash tools/carla_town04/scripts/collect_town04.sh \
  --town Town04 \
  --episodes 20 \
  --frames-per-episode 500 \
  --vehicles 80 \
  --output data/carla_town04_nuscenes_rig
```

`Town04_Opt` is the default because it matches the ADAS scenarios. Use
`--town Town04` if the non-layered map is required.

The output layout is:

```text
data/carla_town04_nuscenes_rig/
├── samples/
│   ├── CAM_FRONT/
│   ├── CAM_FRONT_RIGHT/
│   ├── CAM_BACK_RIGHT/
│   ├── CAM_BACK/
│   ├── CAM_BACK_LEFT/
│   └── CAM_FRONT_LEFT/
├── lidar_placeholder.bin
├── carla_town04_temporal_infos_train.pkl
└── carla_town04_temporal_infos_val.pkl
```

The placeholder LiDAR file is intentional: StreamPETR is configured as
camera-only, while the inherited nuScenes info schema still requires a LiDAR
reference path.

## Balanced 8,100-frame Town04 collection

The matrix launcher collects nine CARLA weather presets crossed with explicit
`left`, `right`, and `straight` paths through Town04 junctions. The controlled
ego follows a deterministic 5 m/s curve while surrounding traffic remains
dynamic. The ego is a collision-free virtual reference rig, so following the
fixed trajectory cannot strike or launch traffic actors. Each of the 27 cases contains exactly 300 samples, for 8,100
synchronized six-camera frames (48,600 JPEG files):

```bash
bash tools/carla_town04/scripts/collect_town04_matrix.sh
```

Each case is divided into fifteen independent 20-frame temporal clips so traffic
is respawned before nearby-object density falls off. The collector's source
annotations use a 12/3 clip split inside every case. For model training,
`resplit_weather_holdout.py` recombines those annotations and creates a strict
weather holdout: six complete weather presets (5,400 frames) for training and
three unseen presets (2,700 frames) for validation. Images remain 1600x900 and
use JPEG quality 82 to keep the complete dataset near 8 GiB. Every frame records
`weather`, `maneuver`, `case_token`, `case_frame_idx`, and `clip_index` in its
info entry. `dataset_manifest.json` records the expected and collected counts.

The launcher always uses safe resume. Re-running the same command skips every
completed clip and continues the first missing one. It also refreshes the CARLA
server after each 45 newly completed clips to avoid long-run GPU resource
accumulation. Override output location, image quality, traffic count, or refresh
interval with environment variables:

```bash
OUTPUT=/larger/disk/town04_matrix JPEG_QUALITY=85 VEHICLES=60 BATCH_SCENES=45 \
  bash tools/carla_town04/scripts/collect_town04_matrix.sh
```

After collection, require the matrix counts, temporal links, image paths, and
manifest to pass validation. Validation also checks that every left/right clip
uses a 60–120 degree topology branch and completes at least 60 degrees in the
correct direction, every straight clip stays within 15 degrees, and ego
traverses at least 20 metres:

```bash
python3 tools/carla_town04/validate_dataset.py \
  data/carla_town04_9weather_3maneuver_8100
```

## Train

Create and validate the weather-holdout annotations first:

```bash
python3 tools/carla_town04/resplit_weather_holdout.py \
  data/carla_town04_9weather_3maneuver_8100

python3 tools/carla_town04/validate_dataset.py \
  data/carla_town04_9weather_3maneuver_8100 \
  --train-info carla_town04_temporal_infos_train_6weather.pkl \
  --val-info carla_town04_temporal_infos_val_3weather.pkl

bash tools/dist_train.sh \
  projects/configs/StreamPETR/stream_petr_r50_flash_704_bs1_seq_town04.py \
  1
```

On the local RTX 3050 6 GB workstation, use the prepared low-memory container
launcher instead. It runs one 6,480-iteration epoch by default:

```bash
bash tools/carla_town04/scripts/train_town04.sh
```

The supplied config trains from random initialization: `pretrained`,
`load_from`, and `resume_from` are all disabled for a first run. It uses only
the five classes present in the CARLA data, the 5,400/2,700 weather split,
nuScenes-style 256x704 image augmentation, effective batch size four through
gradient accumulation, full FP32 training to avoid scratch-initialization
gradient overflows, and validation after each epoch.

To package the exact code without any checkpoint and launch it on Hugging Face
Jobs, use:

```bash
bash tools/carla_town04/scripts/launch_hf_scratch_training.sh
```

The launcher writes checkpoints every 600 iterations, keeps the optimizer and
runner state, copies logs/config/environment manifests to the output bucket,
and starts a local background sync. Restarting an interrupted attempt may
resume only a checkpoint produced by that same scratch run.

## Visualize labels and predictions

Run sequential inference on the 1,620 validation frames, then render selected
frames with the six camera views, a bird's-eye view, and a perspective 3D view:

```bash
bash tools/carla_town04/scripts/visualize_town04.sh
```

Ground truth boxes are green, model predictions are magenta/red, and the ego
vehicle/camera directions are cyan. By default the renderer shows at most 20
predictions with score at least 0.40. It also removes nuScenes-only classes,
physically invalid/duplicate boxes, and camera boxes whose centre or almost all
of their projected area is hidden behind a nearer box. BEV and 3D panels retain
all GT boxes because occlusion only applies to a particular camera view.
Override the selected validation indices, score threshold, or minimum visible
area with environment variables:

```bash
INDICES="0 20 39 40 60 79" SCORE_THRESHOLD=0.45 MIN_VISIBLE_RATIO=0.08 \
  bash tools/carla_town04/scripts/visualize_town04.sh
```

To render again from the existing prediction PKL without rerunning the model:

```bash
SKIP_INFERENCE=1 SCORE_THRESHOLD=0.40 \
  bash tools/carla_town04/scripts/visualize_town04.sh
```

The generated files are written below
`work_dirs/stream_petr_town04_hybrid_24e/visualizations/`.

### Official nuScenes-pretrained checkpoint

The official 90-epoch R50 checkpoint needs its original 644-query, six-decoder
architecture and 256x704 preprocessing. Use the supplied inference-only config;
it replaces FlashAttention with parameter-compatible PyTorch attention but
keeps the released weight shapes unchanged:

```bash
bash tools/carla_town04/scripts/download_pretrained.sh

SCORE_THRESHOLD=0.40 \
  bash tools/carla_town04/scripts/visualize_town04.sh \
  projects/configs/StreamPETR/stream_petr_r50_704_seq_90e_town04_inference.py \
  ckpts/stream_petr_r50_flash_704_bs2_seq_90e.pth \
  work_dirs/stream_petr_pretrained_town04/visualizations
```

## Scope

The current collector creates random Traffic Manager episodes; it does not
launch the `.xosc` ACC/AEB/TJA scenarios. Those files select `Town04_Opt` and
can later be integrated as a second, scenario-driven collection mode without
changing the dataset format.
