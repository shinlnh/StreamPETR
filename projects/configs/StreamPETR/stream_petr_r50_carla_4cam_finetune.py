_base_ = ["./stream_petr_r50_flash_704_bs2_seq_90e.py"]

# CARLA 0.9.16 provides these dynamic nuScenes classes consistently in every
# selected town. The four nuScenes-only static/special categories are omitted.
class_names = [
    "car",
    "truck",
    "bus",
    "motorcycle",
    "bicycle",
    "pedestrian",
]
num_classes = len(class_names)

data_root = "./data/carla/"
checkpoint_path = "./ckpts/stream_petr_r50_carla_6class_init.pth"
point_cloud_range = [-51.2, -51.2, -5.0, 51.2, 51.2, 3.0]
collect_keys = [
    "lidar2img",
    "intrinsics",
    "extrinsics",
    "timestamp",
    "img_timestamp",
    "ego_pose",
    "ego_pose_inv",
]
img_norm_cfg = dict(
    mean=[123.675, 116.28, 103.53],
    std=[58.395, 57.12, 57.375],
    to_rgb=True,
)

# One RTX 5070 Ti (16 GB): four rather than six views and activation
# checkpointing inherited from the upstream R50 config. Batch 16 is tested
# locally before the final run; if it does not fit, a smaller physical batch
# is paired with gradient accumulation to retain effective batch 16. The old
# flash-attn 0.2 extension cannot target Blackwell/CUDA 12.8, so use the
# repository's regular fp16 attention implementation.
num_gpus = 1
batch_size = 16
effective_batch_size = 16
gradient_accumulation_steps = effective_batch_size // batch_size
assert effective_batch_size % batch_size == 0
# The collector assigns every fifth complete scene to validation:
# 43 train scenes * 400 samples = 17,200.
num_train_samples = 17200
num_iters_per_epoch = num_train_samples // (num_gpus * batch_size)
num_epochs = 15

regular_attention = dict(
    type="PETRMultiheadAttention",
    embed_dims=256,
    num_heads=8,
    dropout=0.1,
    fp16=True,
)

model = dict(
    require_cuda_inference=True,
    img_roi_head=dict(num_classes=num_classes),
    pts_bbox_head=dict(
        num_classes=num_classes,
        bbox_coder=dict(num_classes=num_classes),
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
        ),
    ),
)

ida_aug_conf = {
    "resize_lim": (0.75, 0.90),
    "final_dim": (256, 704),
    "bot_pct_lim": (0.0, 0.0),
    "rot_lim": (0.0, 0.0),
    "H": 540,
    "W": 960,
    "rand_flip": True,
}

train_pipeline = [
    dict(type="LoadMultiViewImageFromFiles", to_float32=True),
    dict(
        type="LoadAnnotations3D",
        with_bbox_3d=True,
        with_label_3d=True,
        with_bbox=True,
        with_label=True,
        with_bbox_depth=True,
    ),
    dict(type="ObjectRangeFilter", point_cloud_range=point_cloud_range),
    dict(type="ObjectNameFilter", classes=class_names),
    dict(
        type="ResizeCropFlipRotImage",
        data_aug_conf=ida_aug_conf,
        training=True,
    ),
    dict(
        type="GlobalRotScaleTransImage",
        rot_range=[-0.3925, 0.3925],
        translation_std=[0, 0, 0],
        scale_ratio_range=[0.95, 1.05],
        reverse_angle=True,
        training=True,
    ),
    dict(type="NormalizeMultiviewImage", **img_norm_cfg),
    dict(type="PadMultiViewImage", size_divisor=32),
    dict(
        type="PETRFormatBundle3D",
        class_names=class_names,
        collect_keys=collect_keys + ["prev_exists"],
    ),
    dict(
        type="Collect3D",
        keys=[
            "gt_bboxes_3d",
            "gt_labels_3d",
            "img",
            "gt_bboxes",
            "gt_labels",
            "centers2d",
            "depths",
            "prev_exists",
        ]
        + collect_keys,
        meta_keys=(
            "filename",
            "ori_shape",
            "img_shape",
            "pad_shape",
            "scale_factor",
            "flip",
            "box_mode_3d",
            "box_type_3d",
            "img_norm_cfg",
            "scene_token",
            "gt_bboxes_3d",
            "gt_labels_3d",
        ),
    ),
]

test_pipeline = [
    dict(type="LoadMultiViewImageFromFiles", to_float32=True),
    dict(
        type="ResizeCropFlipRotImage",
        data_aug_conf=ida_aug_conf,
        training=False,
    ),
    dict(type="NormalizeMultiviewImage", **img_norm_cfg),
    dict(type="PadMultiViewImage", size_divisor=32),
    dict(
        type="MultiScaleFlipAug3D",
        img_scale=(1333, 800),
        pts_scale_ratio=1,
        flip=False,
        transforms=[
            dict(
                type="PETRFormatBundle3D",
                collect_keys=collect_keys,
                class_names=class_names,
                with_label=False,
            ),
            dict(
                type="Collect3D",
                keys=["img"] + collect_keys,
                meta_keys=(
                    "filename",
                    "ori_shape",
                    "img_shape",
                    "pad_shape",
                    "scale_factor",
                    "flip",
                    "box_mode_3d",
                    "box_type_3d",
                    "img_norm_cfg",
                    "scene_token",
                ),
            ),
        ],
    ),
]

data = dict(
    samples_per_gpu=batch_size,
    workers_per_gpu=4,
    pin_memory=True,
    persistent_workers=True,
    prefetch_factor=2,
    train=dict(
        type="CarlaStreamPetrDataset",
        point_cloud_range=point_cloud_range,
        data_root=data_root,
        ann_file=data_root + "carla_streampetr_infos_train.pkl",
        classes=class_names,
        pipeline=train_pipeline,
    ),
    val=dict(
        type="CarlaStreamPetrDataset",
        point_cloud_range=point_cloud_range,
        # StreamPETR memory is temporal: frames must be evaluated in order,
        # so use one sequential frame while all neural compute stays on CUDA.
        samples_per_gpu=1,
        data_root=data_root,
        ann_file=data_root + "carla_streampetr_infos_val.pkl",
        classes=class_names,
        pipeline=test_pipeline,
    ),
    test=dict(
        type="CarlaStreamPetrDataset",
        point_cloud_range=point_cloud_range,
        data_root=data_root,
        ann_file=data_root + "carla_streampetr_infos_val.pkl",
        classes=class_names,
        pipeline=test_pipeline,
    ),
)

optimizer = dict(
    type="AdamW",
    # Linear scaling from the verified 2e-5 batch-one fine-tuning rate.
    lr=2e-5 * effective_batch_size,
    paramwise_cfg=dict(custom_keys={"img_backbone": dict(lr_mult=0.1)}),
    weight_decay=0.01,
)
# The legacy MMCV default starts dynamic scaling at 65,536, which overflows
# StreamPETR's denoising losses for several warm-up iterations on torch 2.7.
# Start conservatively and retain automatic growth/backoff.
optimizer_config = dict(
    _delete_=True,
    type=(
        "GradientCumulativeFp16OptimizerHook"
        if gradient_accumulation_steps > 1
        else "Fp16OptimizerHook"
    ),
    **(
        dict(cumulative_iters=gradient_accumulation_steps)
        if gradient_accumulation_steps > 1
        else {}
    ),
    loss_scale=dict(
        init_scale=16.0,
        growth_factor=2.0,
        backoff_factor=0.5,
        growth_interval=2000,
    ),
    grad_clip=dict(max_norm=35, norm_type=2),
)
lr_config = dict(
    policy="CosineAnnealing",
    warmup="linear",
    warmup_iters=1000,
    warmup_ratio=0.1,
    min_lr_ratio=1e-3,
)
runner = dict(type="IterBasedRunner", max_iters=num_epochs * num_iters_per_epoch)
cudnn_benchmark = True
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=3)
evaluation = dict(
    interval=num_iters_per_epoch,
    metric=[0.25, 0.5],
    pipeline=test_pipeline,
)
# Keep detailed metrics in JSON/TensorBoard while presenting one compact live
# line in the terminal. Validation already has its own MMCV progress bar.
log_config = dict(
    _delete_=True,
    interval=50,
    hooks=[
        dict(type="CarlaFileLoggerHook"),
        dict(type="TensorboardLoggerHook"),
    ],
)
custom_hooks = [
    dict(
        type="CarlaProgressBarHook",
        iters_per_epoch=num_iters_per_epoch,
        num_epochs=num_epochs,
        mininterval=0.25,
        priority="ABOVE_NORMAL",
    )
]
load_from = checkpoint_path
resume_from = None
work_dir = "./work_dirs/stream_petr_r50_carla_4cam_finetune"
