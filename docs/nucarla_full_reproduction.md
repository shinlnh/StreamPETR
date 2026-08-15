# StreamPETR R50 reproduction on full nuCarla

## Reference protocol

- StreamPETR upstream: <https://github.com/exiawsh/StreamPETR>
- Upstream 90-epoch R50 config: `stream_petr_r50_flash_704_bs2_seq_90e.py`
- nuCarla dataset/paper code: <https://github.com/michigan-traffic-lab/nuCarla>
- nuCarla paper: <https://arxiv.org/abs/2511.13744>

nuCarla does not publish a StreamPETR experiment. This run warm-starts from
the official nuScenes R50 900-query 90-epoch release, keeps its streaming
recipe, and replaces only the dataset, dataset ordering, evaluator and
unavailable flash-attn kernel.

The exact initialization file is
`ckpts/stream_petr_r50_flash_704_bs2_seq_90e.pth`. Its SHA-256 is
`e6323ae5c31adf1eedd46d6dd4fd3c73d95aa26f18cc8aa23c196494b7de3451`.
The launcher verifies this hash before every new or resumed run.

The closest published nuCarla reference is PETR VovNet at 0.745 six-class
mAP and 0.710 six-class NDS. It is a comparison target, not a claimed
StreamPETR result; the new run must be judged from its saved validation logs.

## Architecture audit

The local upstream R50 90-epoch config is byte-identical to upstream commit
`95f64702306ccdb7a78889578b2a55b5deb35b2a`. The reproduction retains:

- ResNet-50 ImageNet initialization and CPFPN with C4/C5 features;
- six camera views and raw 1600x900 images augmented to 256x704;
- 644 learned queries plus 256 propagated queries (900 total);
- temporal memory length 1024 and six decoder layers;
- ten nuScenes output classes, denoising settings and x/y code weights 2.0;
- streaming sequence mode, sequence split 2, batch 12, AdamW 3e-4,
  cosine schedule, 500-iteration warmup and 90 epochs.

The resulting model has 37,259,345 parameters, exactly the same count as the
upstream flash-attn model. The released upstream R50 checkpoint loads without
missing or unexpected parameters.

Two runtime adaptations do not change trainable parameters:

1. Independent CARLA clips are sorted by `(scene_token, frame_idx)` because
   simulator timestamps are not guaranteed to be globally unique.
2. The unavailable legacy flash-attn extension is replaced by the existing
   `PETRMultiheadAttention`. Attention weights are disabled because PETR never
   reads them, allowing PyTorch 2.7 to use fused SDPA. A representative
   cross-attention micro-benchmark measured a 6.44x speedup and reduced peak
   allocation from 258 MiB to 33 MiB, with mean absolute output difference
   `4.1e-6` in FP16.

Optional local CARLA additions such as class weighting, small-object FPN and
CUDA-only inference are not enabled by this config.

## Data and metrics

`data/nucarla_full` uses the official complete trainval metadata and the
downloaded Town01-Town07 samples:

- train: 700 scenes x 40 frames = 28,000 frames;
- validation: 150 scenes x 40 frames = 6,000 frames;
- six present classes: car, truck, bus, motorcycle, bicycle and pedestrian.

The network keeps the official ten-class nuScenes heads. Validation reports
both the raw ten-class nuScenes metrics and the six-class nuCarla mAP/NDS used
by the nuCarla paper. Best-checkpoint selection uses
the slash-free `NuCarla_NDS` alias; detailed logs also contain
`pts_bbox_NuCarla/NDS`.

Regenerate the StreamPETR temporal files, including auxiliary 2D labels, with:

```bash
PYTHONPATH="$PWD/mmcv:$PWD" .venv/bin/python \
  tools/create_nucarla_full_infos.py
```

## Hardware smoke result

On the local RTX 5070 Ti 16 GB, a physical batch of 16 reached 13.65 GiB in
the training process and eventually failed when a dense sample needed another
1.08 GiB. The durable run therefore uses physical batch 12 and LR 3e-4, the
linear midpoint of the upstream batch-8/2e-4 and batch-16/4e-4 guidance. CUDA
expandable segments are enabled to reduce allocator fragmentation.

## Run and resume

Start or resume the run with:

```bash
./train_nucarla_full_90e.sh
```

The run saves a checkpoint every 2,333 iterations (one epoch), evaluates every
five epochs, retains ten rolling checkpoints and maintains `latest.pth`. The
same command detects and resumes `latest.pth` automatically. The work directory is
`work_dirs/stream_petr_r50_nucarla_full_90e`.
