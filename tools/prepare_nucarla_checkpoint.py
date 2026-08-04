#!/usr/bin/env python3
"""Adapt the 10-class nuScenes checkpoint to the six-class head by slicing.

Unlike ``prepare_carla_checkpoint.py``, which drops the class-dependent output
layers entirely and leaves them randomly initialized, every one of the six
target classes also exists in nuScenes. So the matching rows can be carried
over verbatim, preserving the pretrained classification weights instead of
relearning them from scratch.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

# Row order of the pretrained 10-class nuScenes heads.
NUSCENES_CLASSES = [
    "car",
    "truck",
    "trailer",
    "bus",
    "construction_vehicle",
    "bicycle",
    "motorcycle",
    "pedestrian",
    "traffic_cone",
    "barrier",
]

# Row order expected by the six-class CARLA/nuCarla head (see class_names in
# projects/configs/StreamPETR/stream_petr_r50_carla_4cam_finetune.py).
TARGET_CLASSES = ["car", "truck", "bus", "motorcycle", "bicycle", "pedestrian"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("ckpts/stream_petr_r50_flash_704_bs2_seq_90e.pth"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("ckpts/stream_petr_r50_nuscenes_6class_sliced.pth"),
    )
    args = parser.parse_args()

    row_index = torch.tensor([NUSCENES_CLASSES.index(c) for c in TARGET_CLASSES])
    print("class row mapping (target <- nuScenes):")
    for target_idx, (name, src_idx) in enumerate(zip(TARGET_CLASSES, row_index)):
        print(f"  {target_idx} {name:<11} <- {int(src_idx)} {NUSCENES_CLASSES[src_idx]}")

    checkpoint = torch.load(args.input, map_location="cpu", weights_only=False)
    state_dict = checkpoint.get("state_dict", checkpoint)

    sliced = []
    for key in list(state_dict):
        # The classification layers are the only ones whose leading dimension
        # is the class count; every other tensor transfers unchanged.
        if key in {"img_roi_head.cls.weight", "img_roi_head.cls.bias"} or (
            "pts_bbox_head.cls_branches." in key
            and (key.endswith(".6.weight") or key.endswith(".6.bias"))
        ):
            tensor = state_dict[key]
            assert tensor.shape[0] == len(NUSCENES_CLASSES), (
                f"{key} has leading dim {tensor.shape[0]}, "
                f"expected {len(NUSCENES_CLASSES)}"
            )
            state_dict[key] = tensor[row_index].clone()
            sliced.append((key, tuple(tensor.shape), tuple(state_dict[key].shape)))

    assert sliced, "no classification layers found; checkpoint layout changed?"

    checkpoint["state_dict"] = state_dict
    meta = checkpoint.setdefault("meta", {})
    meta["carla_classes"] = list(TARGET_CLASSES)
    meta["source_checkpoint"] = str(args.input)
    meta["class_head_init"] = "sliced_from_nuscenes"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)
    print(f"\nWrote {args.output}; sliced {len(sliced)} class-head tensors")
    for key, old_shape, new_shape in sliced:
        print(f"  {key}: {old_shape} -> {new_shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
