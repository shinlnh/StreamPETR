#!/usr/bin/env python3
"""Run sequential StreamPETR inference on the CARLA Town04 validation split.

The stock StreamPETR test entry point is distributed-only and its nuScenes
formatter expects nuScenes database tables.  This script keeps inference local,
preserves temporal memory by iterating samples in order, and stores only the
portable arrays needed by the Town04 visualizer.
"""

import argparse
import importlib
import os
import pickle
from pathlib import Path

import numpy as np
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model
from mmdet3d.datasets import build_dataset
from mmdet3d.models import build_model

from projects.mmdet3d_plugin.datasets.builder import build_dataloader


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Town04 StreamPETR config")
    parser.add_argument("checkpoint", help="trained checkpoint")
    parser.add_argument("output", help="output prediction PKL")
    parser.add_argument(
        "--ann-file",
        help="override cfg.data.test.ann_file (useful for collection smoke tests)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="stop after this many sequential frames (0 means the full split)",
    )
    parser.add_argument("--workers", type=int, default=2)
    return parser.parse_args()


def import_plugin(cfg, config_path):
    if not getattr(cfg, "plugin", False):
        return
    plugin_dir = getattr(cfg, "plugin_dir", None)
    module_dir = os.path.dirname(plugin_dir or config_path)
    module_path = ".".join(part for part in module_dir.split("/") if part)
    importlib.import_module(module_path)


def as_numpy(value, dtype=None):
    if torch.is_tensor(value):
        value = value.detach().cpu().numpy()
    value = np.asarray(value)
    return value.astype(dtype, copy=False) if dtype is not None else value


def main():
    args = parse_args()
    cfg = Config.fromfile(args.config)
    import_plugin(cfg, args.config)

    if args.ann_file:
        cfg.data.test.ann_file = args.ann_file

    cfg.model.pretrained = None
    cfg.model.train_cfg = None
    cfg.data.test.test_mode = True
    cfg.data.workers_per_gpu = args.workers

    dataset = build_dataset(cfg.data.test)
    data_loader = build_dataloader(
        dataset,
        samples_per_gpu=1,
        workers_per_gpu=args.workers,
        dist=False,
        shuffle=False,
        nonshuffler_sampler=cfg.data.nonshuffler_sampler,
    )

    model = build_model(cfg.model, test_cfg=cfg.get("test_cfg"))
    if cfg.get("fp16") is not None:
        wrap_fp16_model(model)
    checkpoint = load_checkpoint(model, args.checkpoint, map_location="cpu")
    model.CLASSES = checkpoint.get("meta", {}).get("CLASSES", dataset.CLASSES)
    class_names = tuple(model.CLASSES)
    model = MMDataParallel(model.cuda().eval(), device_ids=[0])

    limit = len(dataset) if args.max_frames <= 0 else min(args.max_frames, len(dataset))
    records = []
    with torch.no_grad():
        for index, data in enumerate(data_loader):
            if index >= limit:
                break
            result = model(return_loss=False, rescale=True, **data)[0]["pts_bbox"]
            info = dataset.data_infos[index]
            records.append(
                {
                    "index": index,
                    "token": info["token"],
                    "scene_token": info["scene_token"],
                    "frame_idx": info["frame_idx"],
                    "boxes_3d": as_numpy(result["boxes_3d"].tensor, np.float32),
                    "scores_3d": as_numpy(result["scores_3d"], np.float32),
                    "labels_3d": as_numpy(result["labels_3d"], np.int64),
                }
            )
            print(
                f"[{index + 1:03d}/{limit:03d}] {info['token']}: "
                f"{len(records[-1]['scores_3d'])} raw predictions",
                flush=True,
            )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as handle:
        pickle.dump(
            {
                "predictions": records,
                "class_names": class_names,
                "config": args.config,
                "checkpoint": args.checkpoint,
            },
            handle,
            protocol=pickle.HIGHEST_PROTOCOL,
        )
    print(f"Wrote {len(records)} frames to {output}")


if __name__ == "__main__":
    main()
