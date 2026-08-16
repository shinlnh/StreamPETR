#!/usr/bin/env python3
"""Validate lineage, calibration, images, and annotation alignment for 7 views."""

import argparse
import hashlib
import json
import math
import pickle
from pathlib import Path

import numpy as np
from PIL import Image


CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_FRONT_NARROW",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)
SOURCE_CAMERA_ORDER = tuple(
    camera for camera in CAMERA_ORDER if camera != "CAM_FRONT_NARROW"
)
EXPECTED_SAMPLES = {"train": 28000, "val": 6000}
ANNOTATION_GROUPS = (
    "bboxes2d",
    "labels2d",
    "centers2d",
    "depths",
    "bboxes_ignore",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default="data/nucarla_full")
    parser.add_argument(
        "--root", default="data/nucarla_full_7cam_fov30"
    )
    parser.add_argument("--source-prefix", default="nucarla_full")
    parser.add_argument(
        "--prefix", default="nucarla_full_7cam_fov30"
    )
    parser.add_argument("--target-fov", type=float, default=30.0)
    parser.add_argument(
        "--skip-source-hash",
        action="store_true",
        help="skip only the expensive lineage hash; structural checks still run",
    )
    return parser.parse_args()


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_payload(path):
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open("rb") as stream:
        payload = pickle.load(stream)
    if not isinstance(payload.get("infos"), list):
        raise ValueError("{} has no infos list".format(path))
    return payload


def arrays_equal(first, second):
    first = np.asarray(first)
    second = np.asarray(second)
    return first.shape == second.shape and np.array_equal(first, second)


def resolve_data_path(path):
    path = Path(path)
    return path if path.is_absolute() else (Path.cwd() / path).resolve()


def validate_split(args, split, manifest_split):
    source_path = Path(args.source_root) / (
        "{}_temporal_infos_{}.pkl".format(args.source_prefix, split)
    )
    target_path = Path(args.root) / (
        "{}_temporal_infos_{}.pkl".format(args.prefix, split)
    )
    if not args.skip_source_hash:
        actual_hash = sha256_file(source_path)
        expected_hash = manifest_split["source_info_sha256"]
        if actual_hash != expected_hash:
            raise ValueError(
                "{} changed: expected {}, got {}".format(
                    source_path, expected_hash, actual_hash
                )
            )

    source = load_payload(source_path)
    target = load_payload(target_path)
    source_infos = source["infos"]
    target_infos = target["infos"]
    expected = EXPECTED_SAMPLES[split]
    if len(source_infos) != expected or len(target_infos) != expected:
        raise ValueError(
            "{} count mismatch: source={}, target={}, expected={}".format(
                split, len(source_infos), len(target_infos), expected
            )
        )
    metadata = target.get("metadata", {})
    if tuple(metadata.get("camera_order", ())) != CAMERA_ORDER:
        raise ValueError("{} metadata has wrong camera order".format(split))
    if metadata.get("derived_method") != "calibrated_center_crop_and_resize":
        raise ValueError("{} has unknown derived-camera method".format(split))

    image_paths = set()
    first_image = last_image = None
    for index, (source_info, target_info) in enumerate(
        zip(source_infos, target_infos)
    ):
        if source_info["token"] != target_info["token"]:
            raise ValueError("{} sample {} token changed".format(split, index))
        if tuple(source_info["cams"]) != SOURCE_CAMERA_ORDER:
            raise ValueError("{} source sample {} camera order changed".format(split, index))
        if tuple(target_info["cams"]) != CAMERA_ORDER:
            raise ValueError("{} target sample {} has wrong camera order".format(split, index))
        for camera_name in SOURCE_CAMERA_ORDER:
            source_camera = source_info["cams"][camera_name]
            target_camera = target_info["cams"][camera_name]
            for key in (
                "data_path",
                "timestamp",
                "sensor2lidar_rotation",
                "sensor2lidar_translation",
                "cam_intrinsic",
            ):
                first, second = source_camera[key], target_camera[key]
                if isinstance(first, str):
                    unchanged = first == second
                else:
                    unchanged = arrays_equal(first, second)
                if not unchanged:
                    raise ValueError(
                        "{} sample {} modified original {} {}".format(
                            split, index, camera_name, key
                        )
                    )
        front = target_info["cams"]["CAM_FRONT"]
        narrow = target_info["cams"]["CAM_FRONT_NARROW"]
        if front["timestamp"] != narrow["timestamp"]:
            raise ValueError("{} sample {} is not synchronized".format(split, index))
        for key in ("sensor2lidar_rotation", "sensor2lidar_translation"):
            if not arrays_equal(front[key], narrow[key]):
                raise ValueError("{} sample {} extrinsic mismatch".format(split, index))
        intrinsic = np.asarray(narrow["cam_intrinsic"], dtype=np.float64)
        fov = math.degrees(
            2.0 * math.atan(1600.0 / (2.0 * float(intrinsic[0, 0])))
        )
        if abs(fov - args.target_fov) > 1e-4:
            raise ValueError("{} sample {} FOV is {}".format(split, index, fov))
        image_path = resolve_data_path(narrow["data_path"])
        if not image_path.is_file() or image_path.stat().st_size == 0:
            raise FileNotFoundError(image_path)
        if image_path in image_paths:
            raise ValueError("duplicate derived image {}".format(image_path))
        image_paths.add(image_path)
        first_image = first_image or image_path
        last_image = image_path

        for key in ANNOTATION_GROUPS:
            if len(target_info[key]) != len(CAMERA_ORDER):
                raise ValueError(
                    "{} sample {} {} does not have seven views".format(
                        split, index, key
                    )
                )
        for camera_index in range(len(CAMERA_ORDER)):
            count = len(target_info["labels2d"][camera_index])
            shapes = (
                np.asarray(target_info["bboxes2d"][camera_index]).reshape(-1, 4).shape[0],
                np.asarray(target_info["centers2d"][camera_index]).reshape(-1, 2).shape[0],
                len(target_info["depths"][camera_index]),
            )
            if any(value != count for value in shapes):
                raise ValueError(
                    "{} sample {} camera {} annotations are misaligned".format(
                        split, index, camera_index
                    )
                )
        for source_index, camera_name in enumerate(SOURCE_CAMERA_ORDER):
            target_index = CAMERA_ORDER.index(camera_name)
            for key in ANNOTATION_GROUPS:
                if not arrays_equal(
                    source_info[key][source_index],
                    target_info[key][target_index],
                ):
                    raise ValueError(
                        "{} sample {} modified original {} annotations".format(
                            split, index, camera_name
                        )
                    )

    for image_path in (first_image, last_image):
        with Image.open(image_path) as image:
            if image.size != (1600, 900):
                raise ValueError("{} has size {}".format(image_path, image.size))
            image.verify()
    print(
        "{} OK: {} samples, {} unique synchronized FOV{} images".format(
            split, expected, len(image_paths), args.target_fov
        )
    )
    return {
        "samples": expected,
        "derived_images": len(image_paths),
        "source_info_sha256": manifest_split["source_info_sha256"],
    }


def main():
    args = parse_args()
    manifest_path = Path(args.root) / "build_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    manifest_splits = {
        item["split"]: item for item in manifest.get("splits", [])
    }
    results = {
        split: validate_split(args, split, manifest_splits[split])
        for split in ("train", "val")
    }
    report = {
        "status": "ok",
        "camera_order": list(CAMERA_ORDER),
        "target_fov_degrees": args.target_fov,
        "source_root": str(Path(args.source_root).resolve()),
        "root": str(Path(args.root).resolve()),
        "splits": results,
    }
    report_path = Path(args.root) / "validation_report.json"
    temporary = report_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    temporary.replace(report_path)
    print("Validation passed; wrote {}".format(report_path))


if __name__ == "__main__":
    main()
