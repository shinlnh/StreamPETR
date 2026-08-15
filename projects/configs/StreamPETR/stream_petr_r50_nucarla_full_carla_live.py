"""Inference config for a live CARLA clip using the full nuCarla model.

Keep the ten-class head and every preprocessing/model setting identical to the
checkpoint trained by ``stream_petr_r50_nucarla_full_90e.py``.  The renderer
filters its display to the six classes that actually exist in nuCarla.
"""

_base_ = ["./stream_petr_r50_nucarla_full_90e.py"]

data = dict(
    samples_per_gpu=1,
    workers_per_gpu=2,
    test=dict(
        samples_per_gpu=1,
        data_root="data/carla_live/",
        ann_file="data/carla_live/carla_live_infos.pkl",
        sequence_order_by_scene=True,
    ),
)

# Inference loads the explicit epoch-50 checkpoint passed on the command line.
load_from = None
resume_from = None
