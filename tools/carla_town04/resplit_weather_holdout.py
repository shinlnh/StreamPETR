#!/usr/bin/env python3
"""Create deterministic six-weather train and three-weather validation PKLs."""

import argparse
import json
import os
import pickle
from collections import Counter
from pathlib import Path

import numpy as np


DEFAULT_TRAIN_WEATHERS = (
    "ClearNoon",
    "CloudyNoon",
    "WetNoon",
    "WetCloudyNoon",
    "SoftRainNoon",
    "MidRainyNoon",
)
DEFAULT_VAL_WEATHERS = ("HardRainNoon", "ClearSunset", "ClearNight")
DEFAULT_CLASSES = ("car", "truck", "bus", "motorcycle", "bicycle")
SOURCE_CLASSES = (
    "car",
    "truck",
    "construction_vehicle",
    "bus",
    "trailer",
    "barrier",
    "motorcycle",
    "bicycle",
    "pedestrian",
    "traffic_cone",
)
SOURCE_FILES = (
    "carla_town04_temporal_infos_train.pkl",
    "carla_town04_temporal_infos_val.pkl",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root",
        nargs="?",
        default="data/carla_town04_9weather_3maneuver_8100",
        type=Path,
    )
    parser.add_argument(
        "--train-output",
        default="carla_town04_temporal_infos_train_6weather.pkl",
    )
    parser.add_argument(
        "--val-output",
        default="carla_town04_temporal_infos_val_3weather.pkl",
    )
    parser.add_argument(
        "--manifest-output",
        default="weather_holdout_split_manifest.json",
    )
    return parser.parse_args()


def atomic_pickle(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def atomic_json(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def load_all_infos(root):
    infos_by_token = {}
    source_metadata = None
    for filename in SOURCE_FILES:
        path = root / filename
        with path.open("rb") as stream:
            payload = pickle.load(stream)
        metadata = payload.get("metadata", {})
        if source_metadata is None:
            source_metadata = metadata
        elif metadata != source_metadata:
            raise ValueError("source train/val metadata differ")
        for info in payload["infos"]:
            token = info["token"]
            if token in infos_by_token:
                raise ValueError("duplicate token: {}".format(token))
            infos_by_token[token] = info
    return list(infos_by_token.values()), dict(source_metadata or {})


def sorted_infos(infos):
    return sorted(
        infos,
        key=lambda info: (
            str(info["weather"]),
            str(info["maneuver"]),
            str(info["scene_token"]),
            int(info["frame_idx"]),
            int(info["timestamp"]),
        ),
    )


def remap_2d_labels(infos):
    """Convert collector class IDs to the compact five-class training IDs."""
    target_by_name = {name: index for index, name in enumerate(DEFAULT_CLASSES)}
    source_to_target = {
        source_index: target_by_name[name]
        for source_index, name in enumerate(SOURCE_CLASSES)
        if name in target_by_name
    }
    for info in infos:
        remapped_cameras = []
        for camera_index, labels in enumerate(info["labels2d"]):
            labels = np.asarray(labels, dtype=np.int64)
            unexpected = sorted(set(labels.tolist()) - set(source_to_target))
            if unexpected:
                raise ValueError(
                    "{} camera {} contains unsupported 2D class IDs {}".format(
                        info["token"], camera_index, unexpected
                    )
                )
            remapped_cameras.append(
                np.asarray(
                    [source_to_target[int(label)] for label in labels],
                    dtype=np.int64,
                )
            )
        info["labels2d"] = remapped_cameras
    return source_to_target


def summarize(infos):
    return {
        "frames": len(infos),
        "scenes": len({info["scene_token"] for info in infos}),
        "weather_counts": dict(sorted(Counter(info["weather"] for info in infos).items())),
        "maneuver_counts": dict(
            sorted(Counter(info["maneuver"] for info in infos).items())
        ),
        "class_counts": dict(
            sorted(Counter(name for info in infos for name in info["gt_names"]).items())
        ),
    }


def main():
    args = parse_args()
    root = args.root
    all_infos, metadata = load_all_infos(root)
    if len(all_infos) != 8100:
        raise ValueError("expected 8100 unique frames, got {}".format(len(all_infos)))

    known_weathers = {info["weather"] for info in all_infos}
    requested_weathers = set(DEFAULT_TRAIN_WEATHERS) | set(DEFAULT_VAL_WEATHERS)
    if known_weathers != requested_weathers:
        raise ValueError(
            "weather mismatch: dataset={} requested={}".format(
                sorted(known_weathers), sorted(requested_weathers)
            )
        )

    observed_classes = {name for info in all_infos for name in info["gt_names"]}
    if observed_classes != set(DEFAULT_CLASSES):
        raise ValueError(
            "class mismatch: dataset={} configured={}".format(
                sorted(observed_classes), sorted(DEFAULT_CLASSES)
            )
        )

    source_to_target = remap_2d_labels(all_infos)
    train_infos = sorted_infos(
        [info for info in all_infos if info["weather"] in DEFAULT_TRAIN_WEATHERS]
    )
    val_infos = sorted_infos(
        [info for info in all_infos if info["weather"] in DEFAULT_VAL_WEATHERS]
    )
    if len(train_infos) != 5400 or len(val_infos) != 2700:
        raise ValueError(
            "expected 5400/2700 frames, got {}/{}".format(
                len(train_infos), len(val_infos)
            )
        )

    train_scenes = {info["scene_token"] for info in train_infos}
    val_scenes = {info["scene_token"] for info in val_infos}
    overlap = train_scenes & val_scenes
    if overlap:
        raise ValueError("train/val share scene {}".format(next(iter(overlap))))

    split_metadata = dict(metadata)
    split_metadata.update(
        {
            "split_strategy": "weather_holdout_6_train_3_val",
            "train_weathers": list(DEFAULT_TRAIN_WEATHERS),
            "val_weathers": list(DEFAULT_VAL_WEATHERS),
            "classes": list(DEFAULT_CLASSES),
            "source_classes": list(SOURCE_CLASSES),
            "labels2d_classes": list(DEFAULT_CLASSES),
            "labels2d_source_to_target": {
                str(source): target
                for source, target in sorted(source_to_target.items())
            },
            "train_frames": len(train_infos),
            "val_frames": len(val_infos),
        }
    )
    train_payload = {"infos": train_infos, "metadata": split_metadata}
    val_payload = {"infos": val_infos, "metadata": split_metadata}

    train_path = root / args.train_output
    val_path = root / args.val_output
    manifest_path = root / args.manifest_output
    atomic_pickle(train_path, train_payload)
    atomic_pickle(val_path, val_payload)

    manifest = {
        "strategy": split_metadata["split_strategy"],
        "source_files": list(SOURCE_FILES),
        "train_file": train_path.name,
        "val_file": val_path.name,
        "classes": list(DEFAULT_CLASSES),
        "labels2d_classes": list(DEFAULT_CLASSES),
        "labels2d_source_to_target": {
            str(source): target
            for source, target in sorted(source_to_target.items())
        },
        "train_weathers": list(DEFAULT_TRAIN_WEATHERS),
        "val_weathers": list(DEFAULT_VAL_WEATHERS),
        "train": summarize(train_infos),
        "val": summarize(val_infos),
    }
    atomic_json(manifest_path, manifest)
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
