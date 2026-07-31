#!/usr/bin/env python3
"""Adapt the 10-class nuScenes checkpoint to the six-class CARLA head."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch


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
        default=Path("ckpts/stream_petr_r50_carla_6class_init.pth"),
    )
    args = parser.parse_args()
    checkpoint = torch.load(args.input, map_location="cpu", weights_only=False)
    state_dict = checkpoint.get("state_dict", checkpoint)
    removed = []
    for key in list(state_dict):
        # Keep the full nuScenes-trained backbone, neck, temporal transformer,
        # queries, and regression heads. Reinitialize only class-dependent
        # output layers whose shapes change from 10 to 6.
        if key in {"img_roi_head.cls.weight", "img_roi_head.cls.bias"} or (
            "pts_bbox_head.cls_branches." in key
            and (key.endswith(".6.weight") or key.endswith(".6.bias"))
        ):
            removed.append((key, tuple(state_dict[key].shape)))
            del state_dict[key]
    checkpoint["state_dict"] = state_dict
    meta = checkpoint.setdefault("meta", {})
    meta["carla_classes"] = [
        "car",
        "truck",
        "bus",
        "motorcycle",
        "bicycle",
        "pedestrian",
    ]
    meta["source_checkpoint"] = str(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)
    print(f"Wrote {args.output}; reinitialized {len(removed)} class-head tensors")
    for key, shape in removed:
        print(f"  {key}: {shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
