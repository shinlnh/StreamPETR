"""Train StreamPETR-R50 from scratch on the Town04 weather holdout split."""

_base_ = ["./stream_petr_r50_flash_704_bs2_seq_24e.py"]

data_root = "./data/carla_town04_9weather_3maneuver_8100/"
num_gpus = 1
batch_size = 1
gradient_accumulation = 4
effective_batch_size = batch_size * gradient_accumulation
num_train_frames = 5400
num_epochs = 24
num_iters_per_epoch = num_train_frames // (num_gpus * batch_size)

# The six-camera collection is calibrated for the nuScenes image geometry.
# Keep nuScenes' 256x704 training crop. Decoder capacity remains sized for a
# single T4, and regular PETR attention avoids an unavailable flash-attn wheel.
ida_aug_conf = dict(
    resize_lim=(0.38, 0.55),
    final_dim=(256, 704),
    bot_pct_lim=(0.0, 0.0),
    rot_lim=(0.0, 0.0),
    H=900,
    W=1600,
    rand_flip=True,
)

model = dict(
    img_backbone=dict(
        pretrained=None,
        norm_cfg=dict(type="BN2d", requires_grad=True),
        norm_eval=False,
    ),
    img_roi_head=dict(num_classes=5),
    pts_bbox_head=dict(
        num_classes=5,
        num_query=300,
        memory_len=512,
        topk_proposals=128,
        num_propagated=128,
        scalar=5,
        bbox_coder=dict(num_classes=5),
        transformer=dict(
            decoder=dict(
                num_layers=3,
                transformerlayers=dict(
                    attn_cfgs=[
                        dict(
                            type="MultiheadAttention",
                            embed_dims=256,
                            num_heads=8,
                            dropout=0.1,
                        ),
                        dict(
                            type="PETRMultiheadAttention",
                            embed_dims=256,
                            num_heads=8,
                            dropout=0.1,
                            fp16=False,
                        ),
                    ],
                    feedforward_channels=512,
                ),
            )
        ),
    )
)

collect_keys = [
    "lidar2img",
    "intrinsics",
    "extrinsics",
    "timestamp",
    "img_timestamp",
    "ego_pose",
    "ego_pose_inv",
]
class_names = [
    "car",
    "truck",
    "bus",
    "motorcycle",
    "bicycle",
]
img_norm_cfg = dict(
    mean=[123.675, 116.28, 103.53],
    std=[58.395, 57.12, 57.375],
    to_rgb=True,
)
point_cloud_range = [-51.2, -51.2, -5.0, 51.2, 51.2, 3.0]

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
    dict(type="ResizeCropFlipRotImage", data_aug_conf=ida_aug_conf, training=True),
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
    dict(type="ResizeCropFlipRotImage", data_aug_conf=ida_aug_conf, training=False),
    dict(type="NormalizeMultiviewImage", **img_norm_cfg),
    dict(type="PadMultiViewImage", size_divisor=32),
    dict(
        type="MultiScaleFlipAug3D",
        img_scale=(704, 256),
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
    train=dict(
        data_root=data_root,
        ann_file=data_root + "carla_town04_temporal_infos_train_6weather.pkl",
        classes=class_names,
        sequence_order_by_scene=True,
        pipeline=train_pipeline,
    ),
    val=dict(
        data_root=data_root,
        ann_file=data_root + "carla_town04_temporal_infos_val_3weather.pkl",
        classes=class_names,
        sequence_order_by_scene=True,
        pipeline=test_pipeline,
    ),
    test=dict(
        data_root=data_root,
        ann_file=data_root + "carla_town04_temporal_infos_val_3weather.pkl",
        classes=class_names,
        sequence_order_by_scene=True,
        pipeline=test_pipeline,
    ),
)

# True scratch initialization: no model checkpoint and no ImageNet backbone.
# Four micro-batches are accumulated before each optimizer update, giving an
# effective batch of four without increasing peak GPU memory.
optimizer = dict(
    _delete_=True,
    type="AdamW",
    lr=1.0e-4,
    weight_decay=0.01,
)
optimizer_config = dict(
    _delete_=True,
    type="GradientCumulativeOptimizerHook",
    cumulative_iters=gradient_accumulation,
    grad_clip=dict(max_norm=35, norm_type=2),
)
# Randomly initialized StreamPETR produced non-finite gradients in mixed
# precision on T4. FP32 is stable and comfortably fits this reduced model.
fp16 = None
lr_config = dict(
    policy="CosineAnnealing",
    warmup="linear",
    warmup_iters=1000,
    warmup_ratio=0.1,
    min_lr_ratio=1.0e-3,
)

checkpoint_config = dict(
    interval=600,
    max_keep_ckpts=6,
    save_optimizer=True,
    create_symlink=False,
)
runner = dict(type="IterBasedRunner", max_iters=num_epochs * num_iters_per_epoch)

evaluation = dict(
    interval=num_iters_per_epoch,
    metric="carla_center_distance",
    distance_thresholds=(0.5, 1.0, 2.0, 4.0),
    point_cloud_range=point_cloud_range,
    save_best="carla/mAP",
    rule="greater",
)
load_from = None
resume_from = None
