"""CARLA 3D detection metrics that do not require nuScenes database tables."""

import json
import os
from pathlib import Path

import numpy as np


DEFAULT_DISTANCE_THRESHOLDS = (0.5, 1.0, 2.0, 4.0)


def _as_numpy(value, dtype=None):
    if hasattr(value, "tensor"):
        value = value.tensor
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    value = np.asarray(value)
    if dtype is not None:
        value = value.astype(dtype, copy=False)
    return value


def _average_precision(recall, precision):
    if recall.size == 0:
        return 0.0
    recall = np.concatenate(([0.0], recall, [1.0]))
    precision = np.concatenate(([0.0], precision, [0.0]))
    for index in range(precision.size - 1, 0, -1):
        precision[index - 1] = max(precision[index - 1], precision[index])
    changes = np.where(recall[1:] != recall[:-1])[0]
    return float(
        np.sum((recall[changes + 1] - recall[changes]) * precision[changes + 1])
    )


def _prediction_arrays(result):
    if "pts_bbox" in result:
        result = result["pts_bbox"]
    boxes = _as_numpy(result["boxes_3d"], np.float32)
    scores = _as_numpy(result["scores_3d"], np.float32).reshape(-1)
    labels = _as_numpy(result["labels_3d"], np.int64).reshape(-1)
    if boxes.ndim != 2 or boxes.shape[0] != scores.size or scores.size != labels.size:
        raise ValueError("prediction arrays have inconsistent shapes")
    return boxes, scores, labels


def _within_range(centers, point_cloud_range):
    if centers.size == 0 or point_cloud_range is None:
        return np.ones(centers.shape[0], dtype=bool)
    x_min, y_min, _, x_max, y_max, _ = point_cloud_range
    return (
        (centers[:, 0] >= x_min)
        & (centers[:, 0] <= x_max)
        & (centers[:, 1] >= y_min)
        & (centers[:, 1] <= y_max)
    )


def _class_ap(predictions, ground_truth, threshold):
    positives = sum(len(centers) for centers in ground_truth.values())
    if positives == 0:
        return None, None, None

    ordered = sorted(predictions, key=lambda item: item[0], reverse=True)
    matched = {
        sample_index: np.zeros(len(centers), dtype=bool)
        for sample_index, centers in ground_truth.items()
    }
    true_positive = np.zeros(len(ordered), dtype=np.float64)
    false_positive = np.zeros(len(ordered), dtype=np.float64)
    matched_distances = []

    for prediction_index, (_, sample_index, center) in enumerate(ordered):
        targets = ground_truth.get(sample_index)
        if targets is None or len(targets) == 0:
            false_positive[prediction_index] = 1.0
            continue

        distances = np.linalg.norm(targets[:, :2] - center[:2], axis=1)
        distances[matched[sample_index]] = np.inf
        target_index = int(np.argmin(distances))
        distance = float(distances[target_index])
        if distance <= threshold:
            true_positive[prediction_index] = 1.0
            matched[sample_index][target_index] = True
            matched_distances.append(distance)
        else:
            false_positive[prediction_index] = 1.0

    true_positive = np.cumsum(true_positive)
    false_positive = np.cumsum(false_positive)
    recall = true_positive / float(positives)
    precision = true_positive / np.maximum(
        true_positive + false_positive, np.finfo(np.float64).eps
    )
    ap = _average_precision(recall, precision)
    final_recall = float(recall[-1]) if recall.size else 0.0
    mean_distance = (
        float(np.mean(matched_distances)) if matched_distances else float("nan")
    )
    return ap, final_recall, mean_distance


def evaluate_carla_center_distance(
    results,
    data_infos,
    class_names,
    distance_thresholds=DEFAULT_DISTANCE_THRESHOLDS,
    point_cloud_range=None,
    jsonfile_prefix=None,
):
    """Evaluate class-wise AP using nuScenes-style BEV center-distance matching."""
    if len(results) != len(data_infos):
        raise ValueError(
            "result count {} does not match dataset size {}".format(
                len(results), len(data_infos)
            )
        )

    class_names = tuple(class_names)
    ground_truth = {name: {} for name in class_names}
    predictions = {name: [] for name in class_names}

    for sample_index, (result, info) in enumerate(zip(results, data_infos)):
        gt_boxes = _as_numpy(info["gt_boxes"], np.float32)
        gt_names = _as_numpy(info["gt_names"]).astype(str)
        gt_mask = _within_range(gt_boxes[:, :2], point_cloud_range)
        valid_flag = np.asarray(
            info.get("valid_flag", np.ones(len(gt_boxes), dtype=bool)), dtype=bool
        )
        gt_mask &= valid_flag
        for class_name in class_names:
            class_mask = gt_mask & (gt_names == class_name)
            ground_truth[class_name][sample_index] = gt_boxes[class_mask, :3]

        boxes, scores, labels = _prediction_arrays(result)
        pred_mask = _within_range(boxes[:, :2], point_cloud_range)
        for box, score, label in zip(
            boxes[pred_mask], scores[pred_mask], labels[pred_mask]
        ):
            label = int(label)
            if 0 <= label < len(class_names):
                predictions[class_names[label]].append(
                    (float(score), sample_index, box[:3])
                )

    metrics = {}
    all_ap = []
    translation_errors = []
    for class_name in class_names:
        class_ap = []
        for threshold in distance_thresholds:
            ap, recall, mean_distance = _class_ap(
                predictions[class_name], ground_truth[class_name], float(threshold)
            )
            threshold_name = "{:g}".format(float(threshold))
            if ap is None:
                continue
            metrics[
                "carla/{}_AP_dist_{}m".format(class_name, threshold_name)
            ] = ap
            metrics[
                "carla/{}_recall_dist_{}m".format(class_name, threshold_name)
            ] = recall
            class_ap.append(ap)
            all_ap.append(ap)
            if abs(float(threshold) - 2.0) < 1e-6 and np.isfinite(mean_distance):
                translation_errors.append(mean_distance)
        metrics["carla/{}_mAP".format(class_name)] = (
            float(np.mean(class_ap)) if class_ap else 0.0
        )

    metrics["carla/mAP"] = float(np.mean(all_ap)) if all_ap else 0.0
    metrics["carla/mATE_2m"] = (
        float(np.mean(translation_errors)) if translation_errors else float("nan")
    )

    if jsonfile_prefix:
        output = Path(str(jsonfile_prefix) + ".carla_metrics.json")
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(metrics, stream, indent=2, sort_keys=True, allow_nan=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(output))

    return metrics
