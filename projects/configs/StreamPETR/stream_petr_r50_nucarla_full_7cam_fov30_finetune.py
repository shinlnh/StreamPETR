_base_ = ["./stream_petr_r50_nucarla_full_90e.py"]

# Fine-tune the mature six-view nuCarla model with one extra synchronized,
# co-located 30-degree front view. The model has no camera-count-specific
# parameters: view count is inferred from the image and calibration tensors.
source_data_root = "./data/nucarla_full/"
seven_camera_info_root = "./data/nucarla_full_7cam_fov30/"
six_camera_checkpoint = (
    "./work_dirs/stream_petr_r50_nucarla_full_90e/"
    "best_NuCarla_NDS_iter_116650.pth"
)

num_train_samples = 28000
num_val_samples = 6000
num_epochs = 15
num_gpus = 1

# Seven views at batch 8 use fewer total image activations than the verified
# six-view batch 12 run (56 versus 72 views), leaving room for dense frames.
batch_size = 8
global_batch_size = num_gpus * batch_size
num_iters_per_epoch = num_train_samples // global_batch_size

data = dict(
    samples_per_gpu=batch_size,
    workers_per_gpu=4,
    train=dict(
        data_root=source_data_root,
        ann_file=(
            seven_camera_info_root
            + "nucarla_full_7cam_fov30_temporal_infos_train.pkl"
        ),
    ),
    val=dict(
        samples_per_gpu=1,
        data_root=source_data_root,
        ann_file=(
            seven_camera_info_root
            + "nucarla_full_7cam_fov30_temporal_infos_val.pkl"
        ),
    ),
    test=dict(
        samples_per_gpu=1,
        data_root=source_data_root,
        ann_file=(
            seven_camera_info_root
            + "nucarla_full_7cam_fov30_temporal_infos_val.pkl"
        ),
    ),
)

# Conservative fine-tuning rate for the already-converged six-camera model.
optimizer = dict(lr=1.0e-4)
# ``load_from`` intentionally does not restore optimizer state. Start at the
# loss scale stored in the mature six-camera checkpoint (512) instead of
# MMCV's dynamic-scaler default (65,536), which would skip several warm-up
# steps while backing off from predictable fp16 overflows.
optimizer_config = dict(
    _delete_=True,
    type="Fp16OptimizerHook",
    loss_scale=dict(
        init_scale=512.0,
        growth_factor=2.0,
        backoff_factor=0.5,
        growth_interval=2000,
    ),
    grad_clip=dict(max_norm=35, norm_type=2),
)
lr_config = dict(warmup_iters=500)
runner = dict(max_iters=num_epochs * num_iters_per_epoch)
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=10)
evaluation = dict(
    interval=num_iters_per_epoch,
    save_best="NuCarla_NDS",
    rule="greater",
)

load_from = six_camera_checkpoint
resume_from = None
work_dir = "./work_dirs/stream_petr_r50_nucarla_full_7cam_fov30_finetune"
