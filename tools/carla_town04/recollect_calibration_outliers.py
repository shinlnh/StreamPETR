#!/usr/bin/env python3
"""Checkpoint calibration-outlier scenes so the collector can resume them."""

import argparse
import json
import pickle
import shutil
from collections import Counter
from pathlib import Path

import numpy as np


CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("backup", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-3)
    parser.add_argument(
        "--scene-token",
        action="append",
        default=[],
        help="checkpoint this explicit scene instead of scanning calibration",
    )
    return parser.parse_args()


def load_payload(path):
    with path.open("rb") as handle:
        return pickle.load(handle)


def atomic_pickle(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as handle:
        pickle.dump(payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
    temporary.replace(path)


def atomic_json(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def reference_calibrations(infos):
    references = {}
    for camera in CAMERA_NAMES:
        translations = np.asarray(
            [info["cams"][camera]["sensor2lidar_translation"] for info in infos]
        )
        rotations = np.asarray(
            [info["cams"][camera]["sensor2lidar_rotation"] for info in infos]
        )
        references[camera] = {
            "translation": np.median(translations, axis=0),
            "rotation": np.median(rotations, axis=0),
        }
    return references


def outlier_scenes(infos, references, tolerance):
    affected = set()
    bad_frames = []
    for info in infos:
        bad_cameras = []
        for camera in CAMERA_NAMES:
            calibration = info["cams"][camera]
            reference = references[camera]
            translation_error = np.max(
                np.abs(
                    calibration["sensor2lidar_translation"]
                    - reference["translation"]
                )
            )
            rotation_error = np.max(
                np.abs(
                    calibration["sensor2lidar_rotation"]
                    - reference["rotation"]
                )
            )
            if max(float(translation_error), float(rotation_error)) > tolerance:
                bad_cameras.append(camera)
        if bad_cameras:
            affected.add(info["scene_token"])
            bad_frames.append(
                {"token": info["token"], "cameras": bad_cameras}
            )
    return affected, bad_frames


def manifest(train_infos, val_infos, metadata):
    all_infos = train_infos + val_infos
    return {
        "metadata": metadata,
        "train_frames": len(train_infos),
        "val_frames": len(val_infos),
        "total_frames": len(all_infos),
        "total_camera_images": len(all_infos) * len(CAMERA_NAMES),
        "case_counts": dict(sorted(Counter(
            info["case_token"] for info in all_infos
        ).items())),
        "split_case_counts": {
            "train": dict(sorted(Counter(
                info["case_token"] for info in train_infos
            ).items())),
            "val": dict(sorted(Counter(
                info["case_token"] for info in val_infos
            ).items())),
        },
    }


def main():
    args = parse_args()
    root = args.dataset.resolve()
    backup = args.backup.resolve()
    if backup.exists() and any(backup.iterdir()):
        raise SystemExit("Backup directory is not empty: {}".format(backup))
    backup.mkdir(parents=True, exist_ok=True)

    paths = {
        split: root / "carla_town04_temporal_infos_{}.pkl".format(split)
        for split in ("train", "val")
    }
    payloads = {split: load_payload(path) for split, path in paths.items()}
    all_infos = payloads["train"]["infos"] + payloads["val"]["infos"]
    if args.scene_token:
        requested = set(args.scene_token)
        existing = {info["scene_token"] for info in all_infos}
        missing_scenes = requested - existing
        if missing_scenes:
            raise SystemExit(
                "Unknown scene tokens: {}".format(sorted(missing_scenes))
            )
        affected = requested
        bad_frames = []
    else:
        references = reference_calibrations(all_infos)
        affected, bad_frames = outlier_scenes(
            all_infos, references, args.tolerance
        )
    if not affected:
        print("No calibration-outlier scenes found.")
        return

    affected_infos = [
        info for info in all_infos if info["scene_token"] in affected
    ]
    image_paths = [
        root / "samples" / camera / "{}.jpg".format(info["token"])
        for info in affected_infos
        for camera in CAMERA_NAMES
    ]
    missing = [str(path) for path in image_paths if not path.is_file()]
    if missing:
        raise SystemExit(
            "Refusing to checkpoint: {} affected images are missing".format(
                len(missing)
            )
        )

    for path in paths.values():
        shutil.copy2(path, backup / path.name)
    manifest_path = root / "dataset_manifest.json"
    if manifest_path.exists():
        shutil.copy2(manifest_path, backup / manifest_path.name)

    for source in image_paths:
        relative = source.relative_to(root)
        destination = backup / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)

    report = {
        "tolerance": args.tolerance,
        "affected_scenes": sorted(affected),
        "affected_scene_count": len(affected),
        "affected_frame_count": len(affected_infos),
        "bad_calibration_frame_count": len(bad_frames),
        "bad_frames": bad_frames,
        "explicit_scene_selection": bool(args.scene_token),
    }
    atomic_json(backup / "calibration_outliers.json", report)

    filtered = {}
    for split in ("train", "val"):
        payload = payloads[split]
        payload["infos"] = [
            info
            for info in payload["infos"]
            if info["scene_token"] not in affected
        ]
        filtered[split] = payload["infos"]
        atomic_pickle(paths[split], payload)
    metadata = payloads["train"].get("metadata", {})
    atomic_json(
        manifest_path,
        manifest(filtered["train"], filtered["val"], metadata),
    )
    print("Checkpointed {} scenes / {} frames / {} images".format(
        len(affected), len(affected_infos), len(image_paths)
    ))
    print("Clean frames retained for resume: {}".format(
        len(filtered["train"]) + len(filtered["val"])
    ))
    print("Backup: {}".format(backup))


if __name__ == "__main__":
    main()
