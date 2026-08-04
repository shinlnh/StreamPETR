_base_ = ["./stream_petr_r50_carla_4cam_finetune.py"]

# Base CARLA train split plus one 12-frame far-pedestrian clip from each of
# the 43 supplemental train scenes.
num_train_samples = 17716
batch_size = 6
effective_batch_size = 12
gradient_accumulation_steps = 2
# Round the exact 2,953 loader steps up to the next accumulation boundary.
# This prevents checkpoint/validation from running with one pending micro-step.
num_iters_per_epoch = 2954
num_epochs = 10

data = dict(
    samples_per_gpu=batch_size,
    train=dict(
        ann_file="./data/carla/carla_streampetr_infos_train_ped_far.pkl",
    ),
)

# Add ResNet C3 and make the first FPN output stride 8 instead of stride 16.
# SmallObjectCPFPN remaps the pretrained C4/C5 lateral weights automatically;
# only the new C3 lateral starts from Xavier initialization.
model = dict(
    stride=8,
    img_backbone=dict(out_indices=(1, 2, 3)),
    img_neck=dict(
        _delete_=True,
        type="SmallObjectCPFPN",
        in_channels=[512, 1024, 2048],
        out_channels=256,
        num_outs=3,
    ),
    img_roi_head=dict(
        stride=8,
        train_ratio=0.5,
        infer_ratio=1.0,
        positive_class_weights=[1.0, 1.0, 1.0, 1.0, 1.0, 1.5],
    ),
    pts_bbox_head=dict(
        stride=8,
        positive_class_weights=[1.0, 1.0, 1.0, 1.0, 1.0, 1.5],
    ),
)

optimizer = dict(
    lr=5e-6,
    paramwise_cfg=dict(
        custom_keys={
            "img_backbone": dict(lr_mult=0.1),
            # Learn the newly added stride-8 lateral faster than pretrained
            # detector parameters.
            "img_neck.lateral_convs.0": dict(lr_mult=10.0),
        }
    ),
)
optimizer_config = dict(
    _delete_=True,
    type="GradientCumulativeFp16OptimizerHook",
    cumulative_iters=gradient_accumulation_steps,
    loss_scale=dict(
        init_scale=16.0,
        growth_factor=2.0,
        backoff_factor=0.5,
        growth_interval=2000,
    ),
    grad_clip=dict(max_norm=35, norm_type=2),
)
lr_config = dict(warmup_iters=300)
runner = dict(max_iters=num_epochs * num_iters_per_epoch)
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=10)
evaluation = dict(interval=num_iters_per_epoch)
custom_hooks = [
    dict(
        type="CarlaProgressBarHook",
        iters_per_epoch=num_iters_per_epoch,
        num_epochs=num_epochs,
        mininterval=0.25,
        priority="ABOVE_NORMAL",
    )
]

load_from = None
resume_from = None
work_dir = "./work_dirs/stream_petr_r50_carla_4cam_ped_far_stride8"
