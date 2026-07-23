"""Run the official 90-epoch nuScenes checkpoint on CARLA Town04 images.

The model topology and 256x704 preprocessing match the released R50 checkpoint.
Only FlashAttention is replaced by the parameter-compatible PyTorch attention
implementation because the local CUDA/PyTorch stack has no flash-attn wheel.
"""

_base_ = ["./stream_petr_r50_flash_704_bs2_seq_90e.py"]

data_root = "./data/carla_town04_9weather_3maneuver_8100/"

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
                        dict(
                            type="PETRMultiheadAttention",
                            embed_dims=256,
                            num_heads=8,
                            dropout=0.1,
                            fp16=True,
                        ),
                    ],
                    with_cp=False,
                )
            )
        )
    )
)

data = dict(
    samples_per_gpu=1,
    workers_per_gpu=2,
    val=dict(
        data_root=data_root,
        ann_file=data_root + "carla_town04_temporal_infos_val.pkl",
    ),
    test=dict(
        data_root=data_root,
        ann_file=data_root + "carla_town04_temporal_infos_val.pkl",
    ),
)
