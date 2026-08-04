#!/usr/bin/env python3
"""NumPy post-processing for the CARLA StreamPETR TensorRT head outputs."""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np


CLASS_NAMES = np.asarray(
    ["car", "truck", "bus", "motorcycle", "bicycle", "pedestrian"]
)
POST_CENTER_RANGE = np.asarray(
    [-61.2, -61.2, -10.0, 61.2, 61.2, 10.0], dtype=np.float32
)


def decode_stream_petr(
    class_logits: np.ndarray,
    bbox_predictions: np.ndarray,
    max_detections: int = 300,
    score_threshold: Optional[float] = None,
) -> Dict[str, np.ndarray]:
    """Apply the model's NMS-free coder to a batch-one TensorRT output.

    Returned boxes use ``[cx, cy, cz, width, length, height, yaw, vx, vy]``
    in the ego/lidar coordinate frame.
    """
    logits = np.asarray(class_logits, dtype=np.float32).reshape(
        -1, len(CLASS_NAMES)
    )
    encoded_boxes = np.asarray(bbox_predictions, dtype=np.float32).reshape(
        -1, 10
    )
    scores_all = 1.0 / (1.0 + np.exp(-np.clip(logits, -80.0, 80.0)))
    flat_scores = scores_all.reshape(-1)
    count = min(int(max_detections), flat_scores.size)
    flat_indexes = np.argpartition(flat_scores, -count)[-count:]
    flat_indexes = flat_indexes[
        np.argsort(flat_scores[flat_indexes])[::-1]
    ]

    scores = flat_scores[flat_indexes]
    labels = (flat_indexes % len(CLASS_NAMES)).astype(np.int64)
    query_indexes = flat_indexes // len(CLASS_NAMES)
    encoded = encoded_boxes[query_indexes]

    boxes = np.concatenate(
        [
            encoded[:, 0:3],
            np.exp(encoded[:, 3:6]),
            np.arctan2(encoded[:, 6:7], encoded[:, 7:8]),
            encoded[:, 8:10],
        ],
        axis=-1,
    )
    valid = np.logical_and(
        np.all(boxes[:, :3] >= POST_CENTER_RANGE[:3], axis=1),
        np.all(boxes[:, :3] <= POST_CENTER_RANGE[3:], axis=1),
    )
    if score_threshold is not None:
        valid &= scores >= float(score_threshold)

    return {
        "boxes_3d": boxes[valid],
        "scores": scores[valid],
        "labels": labels[valid],
        "class_names": CLASS_NAMES[labels[valid]],
        "query_indexes": query_indexes[valid].astype(np.int64),
    }
