_base_ = ["./stream_petr_r50_carla_4cam_finetune.py"]

# Clean Town04-only retraining deliberately inherits the original StreamPETR
# ResNet-50 + CPFPN architecture. It does not use the later multiscale fusion
# neck or any pedestrian-only adaptation stage.
data_root = "../../scripts/collect_dataset/StreamPETR_CARLA_TOWN04_CLEAN/"

# Nine complete weather scenes: scene indices 0 and 5 are held out by the
# collector, leaving seven train scenes and two validation scenes.
num_train_samples = 2800
num_val_samples = 800
batch_size = 12
effective_batch_size = 12
num_iters_per_epoch = (num_train_samples + batch_size - 1) // batch_size
num_epochs = 10

data = dict(
    samples_per_gpu=batch_size,
    workers_per_gpu=4,
    train=dict(
        data_root=data_root,
        ann_file=data_root + "carla_streampetr_infos_train.pkl",
    ),
    val=dict(
        samples_per_gpu=1,
        data_root=data_root,
        ann_file=data_root + "carla_streampetr_infos_val.pkl",
    ),
    test=dict(
        data_root=data_root,
        ann_file=data_root + "carla_streampetr_infos_val.pkl",
    ),
)

# Restart from the nuScenes-derived six-class initialization, not from a model
# trained on the discarded noisy CARLA labels.
load_from = "./ckpts/stream_petr_r50_carla_6class_init.pth"
resume_from = None

# Batch 12 is the locally verified physical batch for the 16 GB GPU. This is a
# conservative clean-data fine-tune rate, with one genuine optimizer step per
# iteration and no gradient accumulation.
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
work_dir = "./work_dirs/stream_petr_r50_carla_town04_clean"
