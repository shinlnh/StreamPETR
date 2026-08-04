_base_ = ["./stream_petr_r50_carla_4cam_finetune.py"]

# Diagnostic/focused stage: all non-pedestrian annotations are removed while
# every temporal frame is retained. Images are reused from the original CARLA
# dataset, so the dedicated metadata adds less than 100 MB on disk.
num_train_samples = 17200
batch_size = 12
num_iters_per_epoch = 1434  # ceil(17,200 / 12)
num_epochs = 3

pedestrian_data_root = (
    "/HELIOS/ADAS/ADAS_Service/scripts/collect_dataset/"
    "StreamPETR_CARLA_PED_ONLY/"
)

data = dict(
    samples_per_gpu=batch_size,
    train=dict(
        ann_file=pedestrian_data_root
        + "carla_streampetr_infos_train_ped_only.pkl",
    ),
    val=dict(
        ann_file=pedestrian_data_root
        + "carla_streampetr_infos_val_far_ped_only.pkl",
    ),
    test=dict(
        ann_file=pedestrian_data_root
        + "carla_streampetr_infos_val_far_ped_only.pkl",
    ),
)

# Only the 2D proposal head and temporal 3D head adapt. Freezing the visual
# encoder makes this a controlled test of whether existing stride-16 features
# contain enough information to learn distant pedestrians.
optimizer = dict(lr=1e-5)
lr_config = dict(warmup_iters=100)
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
runner = dict(max_iters=num_epochs * num_iters_per_epoch)
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=3)
evaluation = dict(interval=num_iters_per_epoch)
custom_hooks = [
    dict(
        type="FreezeModelModulesHook",
        module_names=["img_backbone", "img_neck"],
        set_eval=True,
        priority="VERY_HIGH",
    ),
    dict(
        type="CarlaProgressBarHook",
        iters_per_epoch=num_iters_per_epoch,
        num_epochs=num_epochs,
        mininterval=0.25,
        priority="ABOVE_NORMAL",
    ),
]

load_from = None
resume_from = None
work_dir = "./work_dirs/stream_petr_r50_carla_4cam_ped_only_frozen"
