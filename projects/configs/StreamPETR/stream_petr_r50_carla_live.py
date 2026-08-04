_base_ = ["./stream_petr_r50_nucarla_town04.py"]

# Inference-only config for a clip recorded by tools/carla_live_capture.py.
# The rig matches nuCarla's calibration, so everything except the annotation
# file carries over from the training config unchanged.
data_root = "data/carla_live/"

data = dict(
    test=dict(
        data_root=data_root,
        ann_file=data_root + "carla_live_infos.pkl",
    ),
)
