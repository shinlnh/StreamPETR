_base_ = ["./stream_petr_r50_flash_704_bs2_seq_90e.py"]

# Full official nuCarla train/val split: 700/150 scenes, 40 frames per scene.
# The upstream StreamPETR R50 recipe is intentionally kept at 90 epochs and
# uses the same raw 1600x900 -> augmented 256x704 image pipeline.
data_root = "./data/nucarla_full/"
pretrained_checkpoint = (
    "./ckpts/stream_petr_r50_flash_704_bs2_seq_90e.pth"
)
num_train_samples = 28000
num_val_samples = 6000
num_epochs = 90

# A physical batch of 16 passed a short smoke test but a dense random batch
# later peaked above the 16 GiB card. Batch 12 leaves headroom for dense frames
# while still driving the GPU; its LR follows the upstream linear batch rule.
num_gpus = 1
batch_size = 12
global_batch_size = num_gpus * batch_size
num_iters_per_epoch = num_train_samples // (num_gpus * batch_size)

# The old flash-attn API used by the upstream repository is unavailable for
# this PyTorch 2.7/CUDA 12.8 Blackwell environment. This is the repository's
# mathematically equivalent attention implementation; model dimensions,
# queries, memory, decoder depth and loss weights remain unchanged.
regular_attention = dict(
    type="PETRMultiheadAttention",
    embed_dims=256,
    num_heads=8,
    dropout=0.1,
    fp16=True,
    # PETR never consumes attention maps. On PyTorch 2.7 this enables fused
    # scaled-dot-product attention without changing model parameters.
    need_weights=False,
)
model = dict(
    pts_bbox_head=dict(
        transformer=dict(
            decoder=dict(
                transformerlayers=dict(
                    attn_cfgs=[
                        dict(
                            type="MultiheadAttention",
                            embed_dims=256,
                            num_heads=8,
                            dropout=0.1,
                        ),
                        regular_attention,
                    ]
                )
            )
        )
    )
)

dataset_type = "NuCarlaDataset"
data = dict(
    samples_per_gpu=batch_size,
    workers_per_gpu=4,
    pin_memory=True,
    persistent_workers=True,
    prefetch_factor=2,
    train=dict(
        type=dataset_type,
        data_root=data_root,
        ann_file=data_root + "nucarla_full_temporal_infos_train.pkl",
        # CARLA timestamps are not globally unique across independent clips.
        # Sort by scene/frame so StreamPETR memory never crosses clips.
        sequence_order_by_scene=True,
    ),
    val=dict(
        type=dataset_type,
        samples_per_gpu=1,
        data_root=data_root,
        ann_file=data_root + "nucarla_full_temporal_infos_val.pkl",
        sequence_order_by_scene=True,
    ),
    test=dict(
        type=dataset_type,
        samples_per_gpu=1,
        data_root=data_root,
        ann_file=data_root + "nucarla_full_temporal_infos_val.pkl",
        sequence_order_by_scene=True,
    ),
)

optimizer = dict(lr=4e-4 * global_batch_size / 16)
optimizer_config = dict(
    _delete_=True,
    type="Fp16OptimizerHook",
    loss_scale="dynamic",
    grad_clip=dict(max_norm=35, norm_type=2),
)
lr_config = dict(
    policy="CosineAnnealing",
    warmup="linear",
    warmup_iters=500,
    warmup_ratio=1.0 / 3,
    min_lr_ratio=1e-3,
)
runner = dict(max_iters=num_epochs * num_iters_per_epoch)

# Preserve every epoch boundary and retain ten rolling checkpoints. Validate
# every five epochs so a simulator-domain model that peaks before epoch 90 is
# pinned as the best checkpoint; the final iteration is evaluated as well.
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=10)
evaluation = dict(
    interval=5 * num_iters_per_epoch,
    save_best="NuCarla_NDS",
    rule="greater",
)

cudnn_benchmark = True
# Warm-start every matching StreamPETR parameter from the official R50 900q
# 90-epoch nuScenes release. The launcher verifies its exact SHA-256 before
# starting so a Town04/local checkpoint can never be selected accidentally.
load_from = pretrained_checkpoint
resume_from = None
work_dir = "./work_dirs/stream_petr_r50_nucarla_full_90e"
