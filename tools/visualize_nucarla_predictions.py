#!/usr/bin/env python3
"""Render predictions against ground truth on nuCarla frames.

Draws the six surround-view cameras plus a bird's-eye view for a handful of
validation samples, the layout the nuCarla paper uses for qualitative figures.
Ground truth is green, predictions are colored per class.
"""

from __future__ import annotations

import argparse
import importlib
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model

from mmdet3d.core.bbox import LiDARInstance3DBoxes
from mmdet3d.datasets import build_dataset
from mmdet3d.models import build_model

CAM_LAYOUT = [
    "CAM_FRONT_LEFT",
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
]

CLASS_COLORS = {
    "car": "#e6194b",
    "truck": "#f58231",
    "bus": "#ffe119",
    "motorcycle": "#4363d8",
    "bicycle": "#911eb4",
    "pedestrian": "#00d5ff",
}

# The 12 edges of a box given mmdet3d's corner ordering.
EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def project_to_image(corners_lidar, cam_info):
    """Project (N, 8, 3) lidar-frame corners into pixel coordinates."""
    rotation = np.asarray(cam_info["sensor2lidar_rotation"])
    translation = np.asarray(cam_info["sensor2lidar_translation"])
    intrinsic = np.asarray(cam_info["cam_intrinsic"])

    flat = corners_lidar.reshape(-1, 3)
    in_cam = (flat - translation) @ rotation  # inverse of R @ p + t
    depth = in_cam[:, 2]
    projected = in_cam @ intrinsic.T
    with np.errstate(divide="ignore", invalid="ignore"):
        pixels = projected[:, :2] / projected[:, 2:3]
    return (
        pixels.reshape(-1, 8, 2),
        depth.reshape(-1, 8),
    )


def draw_boxes_on_image(axis, corners_lidar, colors, cam_info, width, height):
    if len(corners_lidar) == 0:
        return
    pixels, depth = project_to_image(corners_lidar, cam_info)
    for box_pixels, box_depth, color in zip(pixels, depth, colors):
        # Skip boxes that are behind the camera or entirely off-frame.
        if (box_depth <= 0.5).any():
            continue
        if (
            box_pixels[:, 0].max() < 0
            or box_pixels[:, 0].min() > width
            or box_pixels[:, 1].max() < 0
            or box_pixels[:, 1].min() > height
        ):
            continue
        for start, end in EDGES:
            axis.plot(
                [box_pixels[start, 0], box_pixels[end, 0]],
                [box_pixels[start, 1], box_pixels[end, 1]],
                color=color,
                linewidth=1.8,
                alpha=0.9,
            )


def draw_bev(axis, corners_lidar, colors, limit=55.0):
    for box_corners, color in zip(corners_lidar, colors):
        footprint = box_corners[:4, :2]  # bottom face
        closed = np.vstack([footprint, footprint[:1]])
        axis.plot(closed[:, 0], closed[:, 1], color=color, linewidth=1.2)
    axis.set_xlim(-limit, limit)
    axis.set_ylim(-limit, limit)
    axis.set_aspect("equal")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("checkpoint")
    parser.add_argument("--out-dir", default="work_dirs/nucarla_vis")
    parser.add_argument("--indices", type=int, nargs="+", default=[0, 200, 400, 600])
    parser.add_argument("--score-thr", type=float, default=0.3)
    args = parser.parse_args()

    cfg = Config.fromfile(args.config)
    if cfg.get("plugin", False):
        importlib.import_module(cfg.plugin_dir.replace("/", ".").rstrip("."))

    cfg.model.pretrained = None
    cfg.data.test.test_mode = True
    dataset = build_dataset(cfg.data.test)
    classes = list(dataset.CLASSES)

    model = build_model(cfg.model, test_cfg=cfg.get("test_cfg"))
    if "Fp16" in str(cfg.get("optimizer_config", {}).get("type", "")):
        wrap_fp16_model(model)
    load_checkpoint(model, args.checkpoint, map_location="cpu")
    model = MMDataParallel(model.cuda(), device_ids=[0])
    model.eval()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    from projects.mmdet3d_plugin.datasets.builder import build_dataloader

    data_loader = build_dataloader(
        dataset,
        samples_per_gpu=1,
        workers_per_gpu=cfg.data.workers_per_gpu,
        dist=False,
        shuffle=False,
        nonshuffler_sampler=cfg.data.nonshuffler_sampler,
    )

    # StreamPETR carries a temporal memory queue, so the frames must be fed in
    # order. Walk the loader once and keep only the requested samples.
    wanted = sorted(set(args.indices))
    collected = {}
    with torch.no_grad():
        for step, batch in enumerate(data_loader):
            if step > wanted[-1]:
                break
            result = model(return_loss=False, rescale=True, **batch)
            if step in wanted:
                collected[step] = result[0]["pts_bbox"]

    for index in wanted:
        prediction = collected[index]
        scores = prediction["scores_3d"].numpy()
        keep = scores >= args.score_thr
        pred_boxes = prediction["boxes_3d"][keep]
        pred_labels = prediction["labels_3d"].numpy()[keep]
        pred_corners = pred_boxes.corners.numpy() if len(pred_boxes) else np.zeros((0, 8, 3))

        info = dataset.data_infos[index]
        gt_names = info["gt_names"]
        gt_mask = np.array([n in classes for n in gt_names], dtype=bool)
        gt_raw = info["gt_boxes"][gt_mask]
        # Info files store the box centre; mmdet3d boxes are bottom-anchored.
        gt_corners = (
            LiDARInstance3DBoxes(
                torch.tensor(gt_raw[:, :7], dtype=torch.float32),
                box_dim=7,
                origin=(0.5, 0.5, 0.5),
            ).corners.numpy()
            if len(gt_raw)
            else np.zeros((0, 8, 3))
        )

        figure = plt.figure(figsize=(30, 12))
        grid = figure.add_gridspec(2, 5, width_ratios=[1, 1, 1, 0.08, 1.5])

        for position, cam_name in enumerate(CAM_LAYOUT):
            axis = figure.add_subplot(grid[position // 3, position % 3])
            cam_info = info["cams"][cam_name]
            image = plt.imread(cam_info["data_path"])
            axis.imshow(image)
            height, width = image.shape[:2]
            draw_boxes_on_image(
                axis, gt_corners, ["#3cb44b"] * len(gt_corners), cam_info, width, height
            )
            draw_boxes_on_image(
                axis,
                pred_corners,
                [CLASS_COLORS[classes[label]] for label in pred_labels],
                cam_info,
                width,
                height,
            )
            axis.set_title(cam_name, fontsize=9)
            axis.axis("off")

        bev = figure.add_subplot(grid[:, 4])
        draw_bev(bev, gt_corners, ["#3cb44b"] * len(gt_corners))
        draw_bev(bev, pred_corners, [CLASS_COLORS[classes[l]] for l in pred_labels])
        bev.plot(0, 0, marker="^", color="black", markersize=9)
        bev.set_title("BEV — green = ground truth, colored = prediction", fontsize=10)
        bev.grid(alpha=0.25)

        handles = [
            plt.Line2D([], [], color="#3cb44b", label="ground truth", linewidth=2)
        ] + [
            plt.Line2D([], [], color=CLASS_COLORS[c], label=c, linewidth=2)
            for c in classes
        ]
        figure.legend(handles=handles, loc="lower center", ncol=7, fontsize=10)

        scene = info["scene_token"][:8]
        figure.suptitle(
            f"nuCarla Town04 val — sample {index} (scene {scene}) — "
            f"{len(gt_corners)} GT vs {len(pred_corners)} pred @ score>{args.score_thr}",
            fontsize=13,
        )
        figure.tight_layout(rect=[0, 0.04, 1, 0.97])
        output = out_dir / f"sample_{index:04d}.jpg"
        figure.savefig(output, dpi=120, bbox_inches="tight")
        plt.close(figure)
        print(f"wrote {output}  ({len(gt_corners)} GT, {len(pred_corners)} pred)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
