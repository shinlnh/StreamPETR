_base_ = ["./stream_petr_r50_carla_4cam_finetune.py"]

# Class counts alone hide the pedestrian failure mode in the original CARLA
# set: most walkers are beyond 40 m and project to very small 2D boxes.  This
# annotation file retains every original frame and appends temporal clips rich
# in pedestrians within 30 m.  The file is produced by:
# scripts/collect_dataset/build_streampetr_balanced_infos.py
balanced_train_samples = 20680
batch_size = 16
num_iters_per_epoch = 1293  # ceil(20,680 / 16)
num_epochs = 3

data = dict(
    samples_per_gpu=batch_size,
    train=dict(
        ann_file="./data/carla/carla_streampetr_infos_train_ped_balanced.pkl",
    ),
)

# This is a short corrective stage loaded from an already-trained CARLA
# checkpoint.  A lower LR limits regression on the strong car/bus classes.
optimizer = dict(lr=1e-5)
lr_config = dict(warmup_iters=200)
runner = dict(max_iters=num_epochs * num_iters_per_epoch)
checkpoint_config = dict(interval=num_iters_per_epoch, max_keep_ckpts=3)
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

# The launcher requires an explicit checkpoint so this corrective stage can
# never silently restart from the nuScenes initialization checkpoint.
load_from = None
resume_from = None
work_dir = "./work_dirs/stream_petr_r50_carla_4cam_ped_balanced"
