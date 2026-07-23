#!/usr/bin/env python3
"""Validate CARLA temporal info PKLs before starting StreamPETR training."""

import argparse
import json
import math
import pickle
from collections import Counter
from pathlib import Path

import numpy as np

from config import CAMERA_NAMES


REQUIRED_KEYS = {
    "lidar_path",
    "token",
    "sweeps",
    "cams",
    "lidar2ego_translation",
    "lidar2ego_rotation",
    "ego2global_translation",
    "ego2global_rotation",
    "timestamp",
    "prev",
    "next",
    "scene_token",
    "frame_idx",
    "gt_boxes",
    "gt_names",
    "gt_velocity",
    "num_lidar_pts",
    "num_radar_pts",
    "valid_flag",
    "bboxes2d",
    "labels2d",
    "centers2d",
    "depths",
    "bboxes_ignore",
}
MATRIX_KEYS = {
    "case_token",
    "case_frame_idx",
    "weather",
    "maneuver",
    "clip_index",
    "junction_id",
    "route_entry",
    "route_exit",
    "route_turn_angle",
    "route_progress_m",
}


def quaternion_yaw_degrees(quaternion):
    w, x, y, z = quaternion
    return math.degrees(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    )


def signed_angle_delta(first, second):
    return (second - first + 180.0) % 360.0 - 180.0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "root",
        nargs="?",
        default="data/carla_town04_9weather_3maneuver_8100",
    )
    parser.add_argument(
        "--train-info",
        default="carla_town04_temporal_infos_train.pkl",
        help="training annotation filename relative to root",
    )
    parser.add_argument(
        "--val-info",
        default="carla_town04_temporal_infos_val.pkl",
        help="validation annotation filename relative to root",
    )
    parser.add_argument(
        "--expect-train-weathers",
        nargs="*",
        help="require this exact set of weather presets in train",
    )
    parser.add_argument(
        "--expect-val-weathers",
        nargs="*",
        help="require this exact set of weather presets in val",
    )
    parser.add_argument(
        "--expect-classes",
        nargs="*",
        help="require this exact set of ground-truth class names",
    )
    return parser.parse_args()


def validate_file(root, info_path):
    with info_path.open("rb") as stream:
        payload = pickle.load(stream)
    infos = payload.get("infos")
    if not isinstance(infos, list):
        raise ValueError("{}: payload['infos'] must be a list".format(info_path))
    scene_counts = {}
    scene_infos = {}
    case_counts = Counter()
    image_paths = set()
    calibration_reference = {}
    object_count = 0
    metadata = payload.get("metadata", {})
    matrix_mode = metadata.get("collection_mode") == "scenario_matrix"
    labels2d_classes = metadata.get("labels2d_classes")
    expected_2d_classes = len(labels2d_classes) if labels2d_classes else None
    for index, info in enumerate(infos):
        missing = REQUIRED_KEYS - set(info)
        if missing:
            raise ValueError("{} info {} missing {}".format(info_path, index, missing))
        if matrix_mode:
            missing_matrix = MATRIX_KEYS - set(info)
            if missing_matrix:
                raise ValueError(
                    "{} info {} missing matrix keys {}".format(
                        info_path, index, missing_matrix
                    )
                )
        if tuple(info["cams"]) != CAMERA_NAMES:
            raise ValueError("{} info {} has wrong camera order".format(info_path, index))
        annotation_groups = (
            "bboxes2d",
            "labels2d",
            "centers2d",
            "depths",
            "bboxes_ignore",
        )
        for key in annotation_groups:
            if len(info[key]) != len(CAMERA_NAMES):
                raise ValueError(
                    "{} info {} has wrong {} camera count".format(
                        info_path, index, key
                    )
                )
        for camera_index in range(len(CAMERA_NAMES)):
            boxes2d = np.asarray(info["bboxes2d"][camera_index])
            labels2d = np.asarray(info["labels2d"][camera_index])
            centers2d = np.asarray(info["centers2d"][camera_index])
            depths = np.asarray(info["depths"][camera_index])
            count2d = len(labels2d)
            if boxes2d.shape != (count2d, 4):
                raise ValueError(
                    "{} info {} camera {} has inconsistent 2D boxes".format(
                        info_path, index, camera_index
                    )
                )
            if centers2d.shape != (count2d, 2) or depths.shape != (count2d,):
                raise ValueError(
                    "{} info {} camera {} has inconsistent 2D targets".format(
                        info_path, index, camera_index
                    )
                )
            if expected_2d_classes is not None and count2d:
                minimum = int(labels2d.min())
                maximum = int(labels2d.max())
                if minimum < 0 or maximum >= expected_2d_classes:
                    raise ValueError(
                        "{} info {} camera {} has 2D label range [{}, {}], "
                        "expected [0, {}]".format(
                            info_path,
                            index,
                            camera_index,
                            minimum,
                            maximum,
                            expected_2d_classes - 1,
                        )
                    )
        boxes = np.asarray(info["gt_boxes"])
        names = np.asarray(info["gt_names"])
        velocity = np.asarray(info["gt_velocity"])
        if boxes.shape != (len(names), 7) or velocity.shape != (len(names), 2):
            raise ValueError("{} info {} has inconsistent GT shapes".format(info_path, index))
        object_count += len(names)
        scene_counts[info["scene_token"]] = scene_counts.get(info["scene_token"], 0) + 1
        scene_infos.setdefault(info["scene_token"], []).append(info)
        case_counts[info.get("case_token", info["scene_token"])] += 1
        for camera_name, camera in info["cams"].items():
            image_path = Path(camera["data_path"])
            if not image_path.is_absolute():
                image_path = Path.cwd() / image_path
            if not image_path.is_file():
                raise FileNotFoundError(image_path)
            normalized_path = str(image_path.resolve())
            if normalized_path in image_paths:
                raise ValueError("duplicate camera image path: {}".format(image_path))
            image_paths.add(normalized_path)
            if np.asarray(camera["sensor2lidar_rotation"]).shape != (3, 3):
                raise ValueError("invalid sensor2lidar_rotation")
            if np.asarray(camera["cam_intrinsic"]).shape != (3, 3):
                raise ValueError("invalid cam_intrinsic")
            if matrix_mode:
                calibration = (
                    np.asarray(camera["sensor2lidar_rotation"]),
                    np.asarray(camera["sensor2lidar_translation"]),
                )
                if camera_name not in calibration_reference:
                    calibration_reference[camera_name] = calibration
                else:
                    reference_rotation, reference_translation = calibration_reference[
                        camera_name
                    ]
                    if not np.allclose(calibration[0], reference_rotation, atol=1e-3):
                        raise ValueError(
                            "{} calibration rotation drifts across frames".format(
                                camera_name
                            )
                        )
                    if not np.allclose(calibration[1], reference_translation, atol=1e-3):
                        raise ValueError(
                            "{} calibration translation drifts across frames".format(
                                camera_name
                            )
                        )
    for scene_token, frames in scene_infos.items():
        frames.sort(key=lambda item: item["frame_idx"])
        for frame_index, info in enumerate(frames):
            if info["frame_idx"] != frame_index:
                raise ValueError("{} has non-contiguous frame_idx".format(scene_token))
            expected_prev = "" if frame_index == 0 else frames[frame_index - 1]["token"]
            expected_next = "" if frame_index == len(frames) - 1 else frames[frame_index + 1]["token"]
            if info["prev"] != expected_prev or info["next"] != expected_next:
                raise ValueError("{} has broken temporal links".format(info["token"]))
        if matrix_mode and frames:
            maneuver = frames[0]["maneuver"]
            start_yaw = quaternion_yaw_degrees(frames[0]["ego2global_rotation"])
            end_yaw = quaternion_yaw_degrees(frames[-1]["ego2global_rotation"])
            actual_turn = signed_angle_delta(start_yaw, end_yaw)
            topology_turn = float(frames[0]["route_turn_angle"])
            if maneuver == "left" and not (
                actual_turn >= 60.0 and -120.0 <= topology_turn <= -60.0
            ):
                raise ValueError(
                    "{} requested left but route/actual yaw is {:.1f}/{:.1f}".format(
                        scene_token, topology_turn, actual_turn
                    )
                )
            if maneuver == "right" and not (
                actual_turn <= -60.0 and 60.0 <= topology_turn <= 120.0
            ):
                raise ValueError(
                    "{} requested right but route/actual yaw is {:.1f}/{:.1f}".format(
                        scene_token, topology_turn, actual_turn
                    )
                )
            if maneuver == "straight" and abs(actual_turn) > 15.0:
                raise ValueError(
                    "{} requested straight but actual yaw delta is {:.1f}".format(
                        scene_token, actual_turn
                    )
                )
            start_position = np.asarray(frames[0]["ego2global_translation"])
            end_position = np.asarray(frames[-1]["ego2global_translation"])
            if np.linalg.norm(end_position[:2] - start_position[:2]) < 20.0:
                raise ValueError("{} ego did not traverse its route".format(scene_token))
    return {
        "frames": len(infos),
        "scenes": len(scene_counts),
        "objects": object_count,
        "case_counts": case_counts,
        "image_paths": image_paths,
        "metadata": metadata,
    }


def main():
    args = parse_args()
    root = Path(args.root)
    total_frames = total_scenes = total_objects = 0
    total_case_counts = Counter()
    all_image_paths = set()
    split_results = {}
    split_files = {"train": args.train_info, "val": args.val_info}
    for split in ("train", "val"):
        info_path = root / split_files[split]
        if not info_path.is_file():
            raise FileNotFoundError(info_path)
        result = validate_file(root, info_path)
        split_results[split] = result
        total_frames += result["frames"]
        total_scenes += result["scenes"]
        total_objects += result["objects"]
        total_case_counts.update(result["case_counts"])
        overlap = all_image_paths & result["image_paths"]
        if overlap:
            raise ValueError("train/val share camera images: {}".format(next(iter(overlap))))
        all_image_paths.update(result["image_paths"])
        print(
            "{}: {} frames, {} scenes, {} objects, {} cases".format(
                split,
                result["frames"],
                result["scenes"],
                result["objects"],
                len(result["case_counts"]),
            )
        )
    expected_weather = {
        "train": args.expect_train_weathers,
        "val": args.expect_val_weathers,
    }
    for split, expected in expected_weather.items():
        if expected is None:
            continue
        observed = {
            weather
            for case in split_results[split]["case_counts"]
            for weather in [case.split("-")[1]]
        }
        metadata = split_results[split]["metadata"]
        canonical = {
            str(weather).lower(): str(weather)
            for weather in metadata.get("weather_presets", [])
        }
        observed_names = {canonical.get(weather, weather) for weather in observed}
        if observed_names != set(expected):
            raise ValueError(
                "{} weather mismatch: observed={} expected={}".format(
                    split, sorted(observed_names), sorted(expected)
                )
            )
    if total_frames == 0:
        raise ValueError("dataset contains no frames")
    train_metadata = split_results["train"]["metadata"]
    val_metadata = split_results["val"]["metadata"]
    if train_metadata != val_metadata:
        raise ValueError("train and val metadata differ")
    if train_metadata.get("collection_mode") == "scenario_matrix":
        expected_frames = train_metadata["expected_frames"]
        expected_cases = train_metadata["expected_cases"]
        frames_per_case = train_metadata["frames_per_case"]
        if total_frames != expected_frames:
            raise ValueError(
                "matrix has {} frames, expected {}".format(total_frames, expected_frames)
            )
        if len(total_case_counts) != expected_cases:
            raise ValueError(
                "matrix has {} cases, expected {}".format(
                    len(total_case_counts), expected_cases
                )
            )
        invalid_cases = {
            case: count
            for case, count in total_case_counts.items()
            if count != frames_per_case
        }
        if invalid_cases:
            raise ValueError("matrix case counts are invalid: {}".format(invalid_cases))
        manifest_path = root / "dataset_manifest.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(manifest_path)
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("total_frames") != total_frames:
            raise ValueError("manifest total_frames does not match PKLs")
        expected_images = total_frames * len(CAMERA_NAMES)
        if len(all_image_paths) != expected_images:
            raise ValueError(
                "matrix references {} images, expected {}".format(
                    len(all_image_paths), expected_images
                )
            )
    if args.expect_classes is not None:
        observed_classes = set()
        for result in split_results.values():
            observed_classes.update(result["metadata"].get("classes", []))
        if not observed_classes:
            for info_path in (root / args.train_info, root / args.val_info):
                with info_path.open("rb") as stream:
                    payload = pickle.load(stream)
                observed_classes.update(
                    name for info in payload["infos"] for name in info["gt_names"]
                )
        if observed_classes != set(args.expect_classes):
            raise ValueError(
                "class mismatch: observed={} expected={}".format(
                    sorted(observed_classes), sorted(args.expect_classes)
                )
            )
    print(
        "OK: {} frames, {} scenes, {} objects, {} camera images".format(
            total_frames, total_scenes, total_objects, len(all_image_paths)
        )
    )


if __name__ == "__main__":
    main()
