_base_ = ["./stream_petr_r50_carla_4cam_finetune.py"]

# Original 17,200 frames plus one centered 12-frame real pedestrian clip from
# each of 43 supplemental train scenes. The independent validation set remains
# unchanged, preventing weather/town leakage into metrics.
num_train_samples = 17716
# Dense close-range pedestrian targets make batch 16 exceed 16 GB during
# backward. Batch 12 stays within the measured headroom without accumulation.
batch_size = 12
effective_batch_size = 12
gradient_accumulation_steps = 1
num_iters_per_epoch = 1477  # ceil(17,716 / 12)
num_epochs = 10

data = dict(
    samples_per_gpu=batch_size,
    train=dict(
        ann_file="./data/carla/carla_streampetr_infos_train_ped_real.pkl",
    ),
)

# Real close-range pedestrian frames carry a strong corrective signal. Keep a
# conservative LR and validate every epoch so the best checkpoint can be
# selected instead of assuming the final epoch is optimal.
optimizer = dict(lr=5e-6)
# Each runner iteration is one optimizer update at batch 12.
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
work_dir = "./work_dirs/stream_petr_r50_carla_4cam_ped_real"
