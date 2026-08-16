#!/usr/bin/env python3
"""Add a synchronized 30-degree front view to existing nuCarla infos.

The added view is a calibrated center crop of CAM_FRONT, resized back to the
source resolution.  It is therefore geometrically equivalent to a co-located
narrow-FOV camera and, unlike an independently rendered CARLA image, preserves
the original scene, timestamp, objects, and ego pose exactly.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import pickle
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image


CAMERA_PROFILE = "nucarla-7cam-front-narrow-fov30-v1"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default="data/nucarla_full")
    parser.add_argument(
        "--output-root", default="data/nucarla_full_7cam_fov30"
    )
    parser.add_argument("--source-prefix", default="nucarla_full")
    parser.add_argument(
        "--output-prefix", default="nucarla_full_7cam_fov30"
    )
    parser.add_argument("--source-camera", default="CAM_FRONT")
    parser.add_argument("--camera-name", default="CAM_FRONT_NARROW")
    parser.add_argument("--target-fov", type=float, default=30.0)
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument(
        "--splits", nargs="+", choices=("train", "val"), default=("train", "val")
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        help="limit each split for a smoke test; omit for the production dataset",
    )
    parser.add_argument("--overwrite-images", action="store_true")
    args = parser.parse_args()
    if not 1.0 < args.target_fov < 179.0:
        parser.error("--target-fov must be in (1, 179) degrees")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be in [1, 100]")
    if args.workers < 1:
        parser.error("--workers must be positive")
    if args.max_samples is not None and args.max_samples < 1:
        parser.error("--max-samples must be positive")
    return args


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_data_path(path):
    path = Path(path)
    return path if path.is_absolute() else (Path.cwd() / path).resolve()


def camera_geometry(source_intrinsic, image_size, target_fov):
    width, height = image_size
    source_intrinsic = np.asarray(source_intrinsic, dtype=np.float64)
    if source_intrinsic.shape != (3, 3):
        raise ValueError("CAM_FRONT intrinsic must be 3x3")
    source_fx = float(source_intrinsic[0, 0])
    source_fy = float(source_intrinsic[1, 1])
    source_cx = float(source_intrinsic[0, 2])
    source_cy = float(source_intrinsic[1, 2])
    target_focal = width / (2.0 * math.tan(math.radians(target_fov) / 2.0))
    target_cx = width / 2.0
    target_cy = height / 2.0
    scale_x = target_focal / source_fx
    scale_y = target_focal / source_fy
    crop_box = (
        source_cx - target_cx / scale_x,
        source_cy - target_cy / scale_y,
        source_cx + (width - target_cx) / scale_x,
        source_cy + (height - target_cy) / scale_y,
    )
    left, top, right, bottom = crop_box
    if left < 0 or top < 0 or right > width or bottom > height:
        raise ValueError(
            "target FOV is not narrower than the source image's calibrated FOV"
        )
    target_intrinsic = np.array(
        [
            [target_focal, 0.0, target_cx],
            [0.0, target_focal, target_cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    return target_intrinsic, crop_box, scale_x, scale_y


def transform_boxes(boxes, source_intrinsic, target_intrinsic, image_size):
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4).copy()
    width, height = image_size
    if not len(boxes):
        return boxes, np.zeros(0, dtype=bool)
    scale_x = float(target_intrinsic[0, 0] / source_intrinsic[0, 0])
    scale_y = float(target_intrinsic[1, 1] / source_intrinsic[1, 1])
    source_cx, source_cy = source_intrinsic[0, 2], source_intrinsic[1, 2]
    target_cx, target_cy = target_intrinsic[0, 2], target_intrinsic[1, 2]
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - source_cx) * scale_x + target_cx
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - source_cy) * scale_y + target_cy
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0.0, float(width))
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0.0, float(height))
    keep = (boxes[:, 2] - boxes[:, 0] >= 1.0) & (
        boxes[:, 3] - boxes[:, 1] >= 1.0
    )
    return boxes[keep], keep


def transform_centers(
    centers, keep, source_intrinsic, target_intrinsic
):
    centers = np.asarray(centers, dtype=np.float32).reshape(-1, 2).copy()
    if len(centers) != len(keep):
        raise ValueError("2D centers do not align with CAM_FRONT boxes")
    scale_x = float(target_intrinsic[0, 0] / source_intrinsic[0, 0])
    scale_y = float(target_intrinsic[1, 1] / source_intrinsic[1, 1])
    centers[:, 0] = (
        (centers[:, 0] - source_intrinsic[0, 2]) * scale_x
        + target_intrinsic[0, 2]
    )
    centers[:, 1] = (
        (centers[:, 1] - source_intrinsic[1, 2]) * scale_y
        + target_intrinsic[1, 2]
    )
    return centers[keep]


def select_aligned(values, keep, dtype=None):
    values = np.asarray(values, dtype=dtype)
    if len(values) != len(keep):
        raise ValueError("CAM_FRONT annotations are not aligned")
    return values[keep]


def insert_after(values, index, new_value):
    values = list(values)
    values.insert(index + 1, new_value)
    return values


def output_image_name(source_path, token, source_camera, camera_name):
    source_name = Path(source_path).name
    marker = "__{}__".format(source_camera)
    if marker in source_name:
        return source_name.replace(marker, "__{}__".format(camera_name), 1)
    return "{}__{}.jpg".format(token, camera_name)


def transform_info(info, args, image_size, stored_samples_root, physical_samples_root):
    if args.camera_name in info["cams"]:
        raise ValueError(
            "{} already contains {}".format(info["token"], args.camera_name)
        )
    camera_names = list(info["cams"])
    if args.source_camera not in camera_names:
        raise KeyError("{} has no {}".format(info["token"], args.source_camera))
    source_index = camera_names.index(args.source_camera)
    source_camera = info["cams"][args.source_camera]
    source_intrinsic = np.asarray(source_camera["cam_intrinsic"], dtype=np.float32)
    target_intrinsic, crop_box, _, _ = camera_geometry(
        source_intrinsic, image_size, args.target_fov
    )

    source_image = resolve_data_path(source_camera["data_path"])
    image_name = output_image_name(
        source_camera["data_path"],
        info["token"],
        args.source_camera,
        args.camera_name,
    )
    stored_image = stored_samples_root / image_name
    physical_image = physical_samples_root / image_name

    new_info = copy.deepcopy(info)
    narrow_camera = copy.deepcopy(source_camera)
    narrow_camera.update(
        {
            "data_path": str(stored_image),
            "cam_intrinsic": target_intrinsic,
            "derived_from_camera": args.source_camera,
            "derived_target_fov_degrees": float(args.target_fov),
        }
    )
    ordered_cameras = {}
    for camera_name, camera in new_info["cams"].items():
        ordered_cameras[camera_name] = camera
        if camera_name == args.source_camera:
            ordered_cameras[args.camera_name] = narrow_camera
    new_info["cams"] = ordered_cameras

    narrow_boxes, keep = transform_boxes(
        info["bboxes2d"][source_index],
        source_intrinsic,
        target_intrinsic,
        image_size,
    )
    narrow_centers = transform_centers(
        info["centers2d"][source_index],
        keep,
        source_intrinsic,
        target_intrinsic,
    )
    narrow_labels = select_aligned(
        info["labels2d"][source_index], keep, dtype=np.int64
    )
    narrow_depths = select_aligned(
        info["depths"][source_index], keep, dtype=np.float32
    )
    narrow_ignored, _ = transform_boxes(
        info["bboxes_ignore"][source_index],
        source_intrinsic,
        target_intrinsic,
        image_size,
    )
    for key, value in (
        ("bboxes2d", narrow_boxes),
        ("labels2d", narrow_labels),
        ("centers2d", narrow_centers),
        ("depths", narrow_depths),
        ("bboxes_ignore", narrow_ignored),
    ):
        new_info[key] = insert_after(new_info[key], source_index, value)

    if "bboxes3d_cams" in new_info:
        narrow_boxes3d = select_aligned(
            info["bboxes3d_cams"][source_index], keep, dtype=np.float32
        )
        new_info["bboxes3d_cams"] = insert_after(
            new_info["bboxes3d_cams"], source_index, narrow_boxes3d
        )
    if "visibilities" in new_info:
        narrow_visibility = select_aligned(
            info["visibilities"][source_index], keep
        ).tolist()
        new_info["visibilities"] = insert_after(
            new_info["visibilities"], source_index, narrow_visibility
        )
    return new_info, (source_image, physical_image, crop_box, image_size)


def render_narrow_image(task, jpeg_quality, overwrite):
    source_path, output_path, crop_box, image_size = task
    if output_path.is_file() and not overwrite:
        with Image.open(output_path) as image:
            if image.size != image_size:
                raise ValueError("{} has wrong image size".format(output_path))
        return False
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(
        output_path.suffix + ".{}.tmp".format(os.getpid())
    )
    with Image.open(source_path) as image:
        if image.size != image_size:
            raise ValueError(
                "{} has size {}, expected {}".format(
                    source_path, image.size, image_size
                )
            )
        narrow = image.convert("RGB").resize(
            image_size,
            resample=Image.Resampling.LANCZOS,
            box=crop_box,
        )
        narrow.save(
            temporary,
            format="JPEG",
            quality=jpeg_quality,
            subsampling=2,
            optimize=False,
        )
    temporary.replace(output_path)
    return True


def write_pickle_atomic(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    temporary.replace(path)


def detect_image_size(first_info, source_camera):
    image_path = resolve_data_path(first_info["cams"][source_camera]["data_path"])
    with Image.open(image_path) as image:
        return image.size


def convert_split(args, split, stored_output_root, physical_output_root):
    source_path = (
        Path(args.source_root)
        / "{}_temporal_infos_{}.pkl".format(args.source_prefix, split)
    )
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    with source_path.open("rb") as stream:
        source_payload = pickle.load(stream)
    source_infos = source_payload.get("infos")
    if not isinstance(source_infos, list) or not source_infos:
        raise ValueError("{} has no infos list".format(source_path))
    if args.max_samples is not None:
        source_infos = source_infos[: args.max_samples]
    image_size = detect_image_size(source_infos[0], args.source_camera)
    stored_samples_root = stored_output_root / "samples" / args.camera_name
    physical_samples_root = physical_output_root / "samples" / args.camera_name

    new_infos = []
    image_tasks = []
    for info in source_infos:
        new_info, image_task = transform_info(
            info,
            args,
            image_size,
            stored_samples_root,
            physical_samples_root,
        )
        new_infos.append(new_info)
        image_tasks.append(image_task)

    print(
        "{}: rendering {} synchronized {} views with {} workers".format(
            split, len(image_tasks), args.camera_name, args.workers
        ),
        flush=True,
    )
    generated = 0
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        results = executor.map(
            lambda task: render_narrow_image(
                task, args.jpeg_quality, args.overwrite_images
            ),
            image_tasks,
        )
        for index, was_generated in enumerate(results, 1):
            generated += int(was_generated)
            if index == 1 or index % 500 == 0 or index == len(image_tasks):
                print(
                    "  {}/{} images ({} new)".format(
                        index, len(image_tasks), generated
                    ),
                    flush=True,
                )

    metadata = copy.deepcopy(source_payload.get("metadata", {}))
    metadata.update(
        {
            "camera_profile": CAMERA_PROFILE,
            "camera_order": list(new_infos[0]["cams"]),
            "derived_camera": args.camera_name,
            "derived_from_camera": args.source_camera,
            "derived_target_fov_degrees": float(args.target_fov),
            "derived_method": "calibrated_center_crop_and_resize",
            "image_size": list(image_size),
            "source_info": str(source_path),
            "source_info_sha256": sha256_file(source_path),
        }
    )
    output_path = physical_output_root / (
        "{}_temporal_infos_{}.pkl".format(args.output_prefix, split)
    )
    write_pickle_atomic(output_path, {"infos": new_infos, "metadata": metadata})
    print("{}: wrote {} samples to {}".format(split, len(new_infos), output_path))
    return {
        "split": split,
        "samples": len(new_infos),
        "generated_images": generated,
        "output_info": str(output_path),
        "source_info": str(source_path),
        "source_info_sha256": metadata["source_info_sha256"],
        "image_size": list(image_size),
        "camera_order": metadata["camera_order"],
    }


def main():
    args = parse_args()
    source_root = Path(args.source_root).resolve()
    physical_output_root = Path(args.output_root).resolve()
    if source_root == physical_output_root:
        raise SystemExit("--output-root must differ from --source-root")
    physical_output_root.mkdir(parents=True, exist_ok=True)
    stored_output_root = Path(args.output_root)
    if stored_output_root.is_absolute():
        stored_output_root = physical_output_root

    results = [
        convert_split(
            args, split, stored_output_root, physical_output_root
        )
        for split in args.splits
    ]
    manifest = {
        "camera_profile": CAMERA_PROFILE,
        "source_root": str(source_root),
        "output_root": str(physical_output_root),
        "source_camera": args.source_camera,
        "camera_name": args.camera_name,
        "target_fov_degrees": args.target_fov,
        "jpeg_quality": args.jpeg_quality,
        "splits": results,
    }
    manifest_path = physical_output_root / "build_manifest.json"
    temporary = manifest_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    temporary.replace(manifest_path)
    print("Wrote {}".format(manifest_path))


if __name__ == "__main__":
    main()
