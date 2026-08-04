_base_ = ["./stream_petr_r50_carla_4cam_finetune.py"]

# Fine-tune on the published nuCarla dataset (Town04 subset, arXiv:2511.13744)
# instead of the self-collected StreamPETR_CARLA_TOWN04_CLEAN set. Same
# ResNet-50 + CPFPN architecture as stream_petr_r50_carla_town04_clean.
data_root = "data/nucarla/"

# nuCarla Town04: 91 train scenes / 29 val scenes (official nuScenes
# scene-name split intersected with the 120 Town04 scenes), 40 keyframes each.
num_train_samples = 3640
num_val_samples = 1160
# Batch 16 OOMs even with the GPU otherwise idle (needs ~14.9GB+ on a
# 15.46GB card -- confirmed, too tight to run reliably). Falling back to
# batch 12, the same physical batch size proven to work for this exact
# architecture in the original stream_petr_r50_carla_town04_clean run.
batch_size = 12
effective_batch_size = 12
num_iters_per_epoch = (num_train_samples + batch_size - 1) // batch_size
num_epochs = 50

# The base CARLA config describes the self-collected 960x540 capture. nuCarla
# ships 1600x900 frames like real nuScenes, and ResizeCropFlipRotImage derives
# its resize factor from these numbers rather than from the image on disk. Left
# at 960x540 the pixels get scaled by 704/1600 = 0.44 while the intrinsics get
# scaled by max(256/540, 704/960) = 0.733, so the model is handed a focal
# length 1.67x too long and a principal point at 83% of the frame width. Those
# are the nuScenes values from stream_petr_r50_flash_704_bs2_seq_90e.
ida_aug_conf = {
    "resize_lim": (0.38, 0.55),
    "final_dim": (256, 704),
    "bot_pct_lim": (0.0, 0.0),
    "rot_lim": (0.0, 0.0),
    "H": 900,
    "W": 1600,
    "rand_flip": True,
}

# The augmentation dict is inlined into the inherited pipelines, so the
# pipelines have to be restated for the corrected values to take effect.
img_norm_cfg = dict(
    mean=[123.675, 116.28, 103.53], std=[58.395, 57.12, 57.375], to_rgb=True
)
class_names = ["car", "truck", "bus", "motorcycle", "bicycle", "pedestrian"]
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
    train=dict(
        data_root=data_root,
        ann_file=data_root + "nucarla_temporal_infos_train.pkl",
        pipeline=train_pipeline,
    ),
    val=dict(
        samples_per_gpu=1,
        data_root=data_root,
        ann_file=data_root + "nucarla_temporal_infos_val.pkl",
        pipeline=test_pipeline,
    ),
    test=dict(
        data_root=data_root,
        ann_file=data_root + "nucarla_temporal_infos_val.pkl",
        pipeline=test_pipeline,
    ),
)

# Warm-start from the original nuScenes-pretrained checkpoint, not from a
# model already fine-tuned on the self-collected CARLA set. All six target
# classes also exist in nuScenes, so tools/prepare_nucarla_checkpoint.py
# slices the matching classification rows across instead of dropping them --
# nothing in the network is randomly initialized.
load_from = "./ckpts/stream_petr_r50_nuscenes_6class_sliced.pth"
resume_from = None

# Same lr proven for batch 12 in stream_petr_r50_carla_town04_clean.
optimizer = dict(lr=1.2e-4)
optimizer_config = dict(
    _delete_=True,
    type="Fp16OptimizerHook",
    loss_scale=dict(
        init_scale=16.0,
        growth_factor=2.0,
        backoff_factor=0.5,
        growth_interval=2000,
    ),
    grad_clip=dict(max_norm=35, norm_type=2),
)
lr_config = dict(warmup_iters=100)

runner = dict(max_iters=num_epochs * num_iters_per_epoch)
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=10)
# max_keep_ckpts rotated the previous run's best epoch away before it could be
# used, so pin the best-scoring checkpoint as well as the last ten.
evaluation = dict(
    interval=num_iters_per_epoch,
    pipeline=test_pipeline,
    save_best="mAP_0.25",
    rule="greater",
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
work_dir = "./work_dirs/stream_petr_r50_nucarla_town04"
