#!/usr/bin/env python3
"""Evaluate a checkpoint with the nuScenes-style center-distance metric.

``CarlaStreamPetrDataset.evaluate`` reports rotated 3D IoU AP, which is far
stricter than the metric the nuCarla paper reports. That evaluator exists
because the self-collected CARLA set ships no nuScenes database, a constraint
nuCarla does not share. This script keeps that dataset class -- its
scene-contiguous ordering is what StreamPETR's temporal memory needs -- and
just scores the predictions with the center-distance evaluator instead, using
the same (0.5, 1, 2, 4) m thresholds nuScenes averages over.
"""

from __future__ import annotations

import argparse
import importlib

import mmcv
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model

from mmdet3d.apis import single_gpu_test
from mmdet3d.datasets import build_dataset
from mmdet3d.models import build_model


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("checkpoint")
    parser.add_argument("--out", default=None, help="optional .pkl of raw predictions")
    args = parser.parse_args()

    cfg = Config.fromfile(args.config)
    if cfg.get("plugin", False):
        importlib.import_module(cfg.plugin_dir.replace("/", ".").rstrip("."))

    cfg.model.pretrained = None
    cfg.data.test.test_mode = True

    dataset = build_dataset(cfg.data.test)
    from projects.mmdet3d_plugin.datasets.builder import build_dataloader

    data_loader = build_dataloader(
        dataset,
        samples_per_gpu=1,
        workers_per_gpu=cfg.data.workers_per_gpu,
        dist=False,
        shuffle=False,
        nonshuffler_sampler=cfg.data.nonshuffler_sampler,
    )

    model = build_model(cfg.model, test_cfg=cfg.get("test_cfg"))
    if cfg.get("fp16", None) is not None or "Fp16" in str(
        cfg.get("optimizer_config", {}).get("type", "")
    ):
        wrap_fp16_model(model)
    load_checkpoint(model, args.checkpoint, map_location="cpu")
    model = MMDataParallel(model.cuda(), device_ids=[0])
    model.eval()

    outputs = single_gpu_test(model, data_loader)
    if args.out:
        mmcv.dump(outputs, args.out)

    from projects.mmdet3d_plugin.core.evaluation.carla_eval import (
        evaluate_carla_center_distance,
    )

    metrics = evaluate_carla_center_distance(
        results=outputs,
        data_infos=dataset.data_infos,
        class_names=dataset.CLASSES,
        point_cloud_range=cfg.data.test.get("point_cloud_range"),
    )

    classes = list(dataset.CLASSES)
    thresholds = ["0.5", "1", "2", "4"]
    header = "class".ljust(12) + "".join(f"AP@{t}m".rjust(10) for t in thresholds)
    header += "mAP".rjust(10)
    print("\n" + header)
    print("-" * len(header))
    for name in classes:
        row = name.ljust(12)
        for t in thresholds:
            value = metrics.get(f"carla/{name}_AP_dist_{t}m")
            row += ("-" if value is None else f"{value:.4f}").rjust(10)
        row += f"{metrics.get(f'carla/{name}_mAP', 0.0):.4f}".rjust(10)
        print(row)
    print("-" * len(header))
    print("OVERALL".ljust(12) + "".rjust(40) + f"{metrics['carla/mAP']:.4f}".rjust(10))
    print(f"\nmATE @2m: {metrics.get('carla/mATE_2m', float('nan')):.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
