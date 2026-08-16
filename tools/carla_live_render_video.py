#!/usr/bin/env python3
"""Run a checkpoint over a recorded CARLA clip and write an annotated video.

Each output frame carries the six or seven surround cameras with predicted 3D
boxes drawn on them plus a bird's-eye panel. Frames go through the dataloader
in order because StreamPETR's memory queue is temporal.
"""

from __future__ import annotations

import argparse
import importlib
import json
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmcv.runner import load_checkpoint, wrap_fp16_model

from mmdet3d.datasets import build_dataset
from mmdet3d.models import build_model

SIX_CAM_GRID = [
    ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"],
    ["CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"],
]
SEVEN_CAM_GRID = [
    ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_NARROW", "CAM_FRONT_RIGHT"],
    ["CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT", None],
]

# The full nuCarla checkpoint deliberately keeps the official nuScenes
# ten-class head. Only these six classes have labels in nuCarla, and their
# indices are not contiguous in that head, so always map colors by class name.
DISPLAY_CLASSES = (
    "car",
    "truck",
    "bus",
    "motorcycle",
    "bicycle",
    "pedestrian",
)
CLASS_COLORS = {
    "car": (60, 20, 220),
    "truck": (49, 130, 245),
    "bus": (25, 225, 255),
    "motorcycle": (216, 99, 67),
    "bicycle": (180, 30, 145),
    "pedestrian": (255, 213, 0),
}

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


def draw_on_camera(image, corners, names, scores, cam, scale):
    pixels, depth = project(corners, cam)
    height, width = image.shape[:2]
    for box_pixels, box_depth, name, score in zip(pixels, depth, names, scores):
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
        color = CLASS_COLORS[name]
        for start, end in EDGES:
            cv2.line(
                image, tuple(points[start]), tuple(points[end]), color, 2, cv2.LINE_AA
            )
        center = points.mean(axis=0).astype(np.int32)
        if 0 <= center[0] < width and 0 <= center[1] < height:
            cv2.putText(
                image,
                f"{name} {score:.2f}",
                tuple(center),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
                cv2.LINE_AA,
            )
    return image


def render_bev(corners, names, size, limit=55.0):
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
    for box_corners, name in zip(corners, names):
        footprint = box_corners[:4, :2]
        points = np.array([to_pixel(x, y) for x, y in footprint], dtype=np.int32)
        color = CLASS_COLORS[name]
        cv2.polylines(canvas, [points], True, color, 2, cv2.LINE_AA)
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("checkpoint")
    parser.add_argument("--out", default="work_dirs/carla_live.mp4")
    parser.add_argument(
        "--ann-file",
        help="override cfg.data.test.ann_file with a freshly captured clip",
    )
    parser.add_argument("--score-thr", type=float, default=0.35)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--fps", type=int, default=2)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()

    cfg = Config.fromfile(args.config)
    if cfg.get("plugin", False):
        importlib.import_module(cfg.plugin_dir.replace("/", ".").rstrip("."))
    cfg.model.pretrained = None
    cfg.data.test.test_mode = True
    cfg.data.workers_per_gpu = args.workers
    if args.ann_file:
        cfg.data.test.ann_file = args.ann_file

    # ``samples_per_gpu`` configures the loader, not the dataset constructor.
    # The stock test entry point removes it before calling ``build_dataset``.
    test_samples_per_gpu = cfg.data.test.pop("samples_per_gpu", 1)
    dataset = build_dataset(cfg.data.test)
    if test_samples_per_gpu != 1:
        raise ValueError("temporal CARLA demo requires samples_per_gpu=1")
    classes = list(dataset.CLASSES)
    camera_names = tuple(dataset.data_infos[0]["cams"])
    cam_grid = SEVEN_CAM_GRID if "CAM_FRONT_NARROW" in camera_names else SIX_CAM_GRID
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

    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    cam_width = args.cam_width
    cam_height = int(cam_width * 900 / 1600)
    bev_size = cam_height * 2
    frame_width = cam_width * len(cam_grid[0]) + bev_size
    frame_height = cam_height * 2
    writer = cv2.VideoWriter(
        str(output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (frame_width, frame_height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"could not open {args.out} for writing")

    scale = cam_width / 1600.0
    counts = []
    class_counts = Counter()
    inference_seconds = []
    poster_index = len(dataset) // 2
    with torch.no_grad():
        for index, batch in enumerate(data_loader):
            torch.cuda.synchronize()
            started = time.perf_counter()
            result = model(return_loss=False, rescale=True, **batch)[0]["pts_bbox"]
            torch.cuda.synchronize()
            inference_seconds.append(time.perf_counter() - started)

            scores = result["scores_3d"].detach().cpu().numpy()
            labels = result["labels_3d"].detach().cpu().numpy()
            class_names = np.asarray([classes[int(label)] for label in labels])
            centers = result["boxes_3d"].tensor[:, :2].detach().cpu().numpy()
            keep = (
                (scores >= args.score_thr)
                & np.isin(class_names, DISPLAY_CLASSES)
                & (np.linalg.norm(centers, axis=1) <= 55.0)
            )
            boxes = result["boxes_3d"][keep]
            names = class_names[keep]
            kept_scores = scores[keep]
            class_counts.update(names.tolist())
            corners = boxes.corners.numpy() if len(boxes) else np.zeros((0, 8, 3))
            counts.append(len(corners))

            info = dataset.data_infos[index]
            rows = []
            for row in cam_grid:
                tiles = []
                for channel in row:
                    if channel is None:
                        tiles.append(
                            np.full((cam_height, cam_width, 3), 24, dtype=np.uint8)
                        )
                        continue
                    image = cv2.imread(info["cams"][channel]["data_path"])
                    image = cv2.resize(image, (cam_width, cam_height))
                    draw_on_camera(
                        image,
                        corners,
                        names,
                        kept_scores,
                        info["cams"][channel],
                        scale,
                    )
                    cv2.putText(
                        image, channel, (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA,
                    )
                    tiles.append(image)
                rows.append(np.hstack(tiles))
            camera_block = np.vstack(rows)
            bev = render_bev(corners, names, bev_size)
            frame = np.hstack([camera_block, bev])

            banner = (
                f"LIVE CARLA | frame {index + 1}/{len(dataset)} | "
                f"{len(corners)} detections | inference "
                f"{inference_seconds[-1] * 1000:.0f} ms"
            )
            cv2.putText(
                frame, banner, (10, frame_height - 34), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA,
            )
            for position, name in enumerate(DISPLAY_CLASSES):
                cv2.putText(
                    frame, name,
                    (10 + position * 110, frame_height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    CLASS_COLORS[name], 2, cv2.LINE_AA,
                )
            writer.write(frame)
            if index == poster_index:
                cv2.imwrite(str(output.with_suffix(".jpg")), frame)
            print(
                f"[{index + 1:03d}/{len(dataset):03d}] "
                f"{len(corners):02d} detections, "
                f"{inference_seconds[-1] * 1000:.0f} ms",
                flush=True,
            )

    writer.release()
    mean_seconds = float(np.mean(inference_seconds))
    steady_seconds = inference_seconds[1:] if len(inference_seconds) > 1 else inference_seconds
    steady_mean_seconds = float(np.mean(steady_seconds))
    summary = {
        "video": str(output),
        "poster": str(output.with_suffix(".jpg")),
        "config": args.config,
        "checkpoint": args.checkpoint,
        "annotation_file": cfg.data.test.ann_file,
        "camera_order": list(camera_names),
        "score_threshold": args.score_thr,
        "frames": len(counts),
        "mean_detections_per_frame": float(np.mean(counts)),
        "detection_counts_by_class": dict(sorted(class_counts.items())),
        "cold_start_inference_ms": inference_seconds[0] * 1000.0,
        "mean_inference_ms": mean_seconds * 1000.0,
        "p95_inference_ms": float(np.percentile(inference_seconds, 95) * 1000.0),
        "mean_inference_fps": 1.0 / mean_seconds,
        "steady_state_mean_inference_ms": steady_mean_seconds * 1000.0,
        "steady_state_p95_inference_ms": float(
            np.percentile(steady_seconds, 95) * 1000.0
        ),
        "steady_state_inference_fps": 1.0 / steady_mean_seconds,
    }
    with output.with_suffix(".json").open("w") as stream:
        json.dump(summary, stream, indent=2)
    print(
        f"wrote {output}  ({len(counts)} frames, "
        f"{np.mean(counts):.1f} detections/frame, "
        f"{mean_seconds * 1000:.0f} ms/frame on average)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
