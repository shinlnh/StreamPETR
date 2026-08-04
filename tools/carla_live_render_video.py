#!/usr/bin/env python3
"""Run a checkpoint over a recorded CARLA clip and write an annotated video.

Each output frame carries the six surround cameras with predicted 3D boxes
drawn on them plus a bird's-eye panel. Frames go through the dataloader in
order because StreamPETR's memory queue is temporal.
"""

from __future__ import annotations

import argparse
import importlib

import cv2
import numpy as np
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model

from mmdet3d.datasets import build_dataset
from mmdet3d.models import build_model

CAM_GRID = [
    ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"],
    ["CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"],
]

# BGR, matching the class order the model emits.
CLASS_COLORS = [
    (60, 20, 220),   # car
    (49, 130, 245),  # truck
    (25, 225, 255),  # bus
    (216, 99, 67),   # motorcycle
    (180, 30, 145),  # bicycle
    (255, 213, 0),   # pedestrian
]

EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def project(corners, cam):
    rotation = np.asarray(cam["sensor2lidar_rotation"])
    translation = np.asarray(cam["sensor2lidar_translation"])
    intrinsic = np.asarray(cam["cam_intrinsic"])
    flat = corners.reshape(-1, 3)
    in_cam = (flat - translation) @ rotation
    projected = in_cam @ intrinsic.T
    with np.errstate(divide="ignore", invalid="ignore"):
        pixels = projected[:, :2] / projected[:, 2:3]
    return pixels.reshape(-1, 8, 2), in_cam[:, 2].reshape(-1, 8)


def draw_on_camera(image, corners, labels, cam, scale):
    pixels, depth = project(corners, cam)
    height, width = image.shape[:2]
    for box_pixels, box_depth, label in zip(pixels, depth, labels):
        if (box_depth <= 0.5).any():
            continue
        points = (box_pixels * scale).astype(np.int32)
        if (
            points[:, 0].max() < 0
            or points[:, 0].min() > width
            or points[:, 1].max() < 0
            or points[:, 1].min() > height
        ):
            continue
        color = CLASS_COLORS[int(label) % len(CLASS_COLORS)]
        for start, end in EDGES:
            cv2.line(
                image, tuple(points[start]), tuple(points[end]), color, 2, cv2.LINE_AA
            )
    return image


def render_bev(corners, labels, size, limit=55.0):
    canvas = np.full((size, size, 3), 24, dtype=np.uint8)
    scale = size / (2 * limit)

    def to_pixel(x, y):
        # Vehicle x forward, y left -> image up is forward, right is -y.
        return int(size / 2 - y * scale), int(size / 2 - x * scale)

    for radius in (10, 20, 30, 40, 50):
        cv2.circle(
            canvas, to_pixel(0, 0), int(radius * scale), (55, 55, 55), 1, cv2.LINE_AA
        )
    cv2.drawMarker(
        canvas, to_pixel(0, 0), (255, 255, 255), cv2.MARKER_TRIANGLE_UP, 14, 2
    )
    for box_corners, label in zip(corners, labels):
        footprint = box_corners[:4, :2]
        points = np.array([to_pixel(x, y) for x, y in footprint], dtype=np.int32)
        color = CLASS_COLORS[int(label) % len(CLASS_COLORS)]
        cv2.polylines(canvas, [points], True, color, 2, cv2.LINE_AA)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("checkpoint")
    parser.add_argument("--out", default="work_dirs/carla_live.mp4")
    parser.add_argument("--score-thr", type=float, default=0.35)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--fps", type=int, default=4)
    args = parser.parse_args()

    cfg = Config.fromfile(args.config)
    if cfg.get("plugin", False):
        importlib.import_module(cfg.plugin_dir.replace("/", ".").rstrip("."))
    cfg.model.pretrained = None
    cfg.data.test.test_mode = True

    dataset = build_dataset(cfg.data.test)
    classes = list(dataset.CLASSES)
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
    if "Fp16" in str(cfg.get("optimizer_config", {}).get("type", "")):
        wrap_fp16_model(model)
    load_checkpoint(model, args.checkpoint, map_location="cpu")
    model = MMDataParallel(model.cuda(), device_ids=[0])
    model.eval()

    cam_width = args.cam_width
    cam_height = int(cam_width * 900 / 1600)
    bev_size = cam_height * 2
    frame_width = cam_width * 3 + bev_size
    frame_height = cam_height * 2
    writer = cv2.VideoWriter(
        args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (frame_width, frame_height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open {args.out} for writing")

    scale = cam_width / 1600.0
    counts = []
    with torch.no_grad():
        for index, batch in enumerate(data_loader):
            result = model(return_loss=False, rescale=True, **batch)[0]["pts_bbox"]
            scores = result["scores_3d"].numpy()
            keep = scores >= args.score_thr
            boxes = result["boxes_3d"][keep]
            labels = result["labels_3d"].numpy()[keep]
            corners = boxes.corners.numpy() if len(boxes) else np.zeros((0, 8, 3))
            counts.append(len(corners))

            info = dataset.data_infos[index]
            rows = []
            for row in CAM_GRID:
                tiles = []
                for channel in row:
                    image = cv2.imread(info["cams"][channel]["data_path"])
                    image = cv2.resize(image, (cam_width, cam_height))
                    draw_on_camera(
                        image, corners, labels, info["cams"][channel], scale
                    )
                    cv2.putText(
                        image, channel, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA,
                    )
                    tiles.append(image)
                rows.append(np.hstack(tiles))
            camera_block = np.vstack(rows)
            bev = render_bev(corners, labels, bev_size)
            frame = np.hstack([camera_block, bev])

            banner = f"frame {index + 1}/{len(dataset)}   {len(corners)} detections"
            cv2.putText(
                frame, banner, (10, frame_height - 34), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA,
            )
            for position, name in enumerate(classes):
                cv2.putText(
                    frame, name,
                    (10 + position * 110, frame_height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    CLASS_COLORS[position], 2, cv2.LINE_AA,
                )
            writer.write(frame)

    writer.release()
    print(
        f"wrote {args.out}  ({len(counts)} frames, "
        f"{np.mean(counts):.1f} detections/frame on average)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
