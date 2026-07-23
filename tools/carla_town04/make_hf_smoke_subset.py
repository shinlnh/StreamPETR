#!/usr/bin/env python3
"""Create a tiny, path-compatible Town04 subset for a remote smoke test."""

import argparse
import pickle
import shutil
from pathlib import Path


INFO_FILES = {
    "train": "carla_town04_temporal_infos_train_6weather.pkl",
    "val": "carla_town04_temporal_infos_val_3weather.pkl",
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Complete Town04 dataset directory")
    parser.add_argument(
        "destination",
        type=Path,
        help="Subset directory; keep the original dataset basename",
    )
    parser.add_argument("--frames-per-split", type=int, default=20)
    return parser.parse_args()


def copy_referenced_file(path_string, source_repo, destination_repo):
    relative_path = Path(path_string)
    if relative_path.is_absolute() or ".." in relative_path.parts:
        raise ValueError("Dataset path must be repository-relative: {}".format(path_string))

    source_path = source_repo / relative_path
    destination_path = destination_repo / relative_path
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    destination_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(source_path), str(destination_path))


def first_scene(infos, limit):
    if not infos:
        raise ValueError("Annotation file contains no frames")
    scene_token = infos[0]["scene_token"]
    selected = [info for info in infos if info["scene_token"] == scene_token][:limit]
    if len(selected) < 2:
        raise ValueError("Smoke subset needs at least two temporal frames")
    return selected


def main():
    args = parse_args()
    source = args.source.resolve()
    destination = args.destination.resolve()
    if source.name != destination.name:
        raise ValueError(
            "Destination basename must remain {!r} because paths inside the PKL are "
            "repository-relative".format(source.name)
        )
    if args.frames_per_split < 2:
        raise ValueError("--frames-per-split must be at least 2")

    # source is <repo>/data/<dataset>; destination follows the same layout.
    source_repo = source.parents[1]
    destination_repo = destination.parents[1]
    destination.mkdir(parents=True, exist_ok=True)

    copied_paths = set()
    split_counts = {}
    for split in ("train", "val"):
        source_info_path = source / INFO_FILES[split]
        with source_info_path.open("rb") as handle:
            payload = pickle.load(handle)

        selected = first_scene(payload["infos"], args.frames_per_split)
        for info in selected:
            referenced = [info["lidar_path"]]
            referenced.extend(cam["data_path"] for cam in info["cams"].values())
            referenced.extend(sweep["data_path"] for sweep in info.get("sweeps", []))
            for path_string in referenced:
                if path_string in copied_paths:
                    continue
                copy_referenced_file(path_string, source_repo, destination_repo)
                copied_paths.add(path_string)

        metadata = dict(payload.get("metadata", {}))
        metadata.update(
            smoke_subset=True,
            smoke_source_frames=len(payload["infos"]),
            smoke_frames=len(selected),
        )
        subset_payload = {"infos": selected, "metadata": metadata}
        destination_info_path = destination / INFO_FILES[split]
        with destination_info_path.open("wb") as handle:
            pickle.dump(subset_payload, handle, protocol=pickle.HIGHEST_PROTOCOL)
        split_counts[split] = len(selected)

    image_count = sum(1 for path in copied_paths if path.lower().endswith(".jpg"))
    expected_images = 6 * sum(split_counts.values())
    if image_count != expected_images:
        raise RuntimeError(
            "Expected {} camera images, copied {}".format(expected_images, image_count)
        )

    print(
        "HF smoke subset ready: train={} val={} images={} path={}".format(
            split_counts["train"], split_counts["val"], image_count, destination
        )
    )


if __name__ == "__main__":
    main()
