#!/usr/bin/env python3
"""Create StreamPETR temporal info files for the full nuCarla trainval set."""

import argparse
from pathlib import Path

import mmcv

from data_converter.nuscenes_converter import create_nuscenes_infos


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root-path", default="data/nucarla_full")
    parser.add_argument("--info-prefix", default="nucarla_full")
    parser.add_argument("--max-sweeps", type=int, default=10)
    return parser.parse_args()


def main():
    args = parse_args()
    root_path = Path(args.root_path)
    metadata_path = root_path / "v1.0-trainval" / "sample.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(
            f"Missing full nuCarla trainval metadata: {metadata_path}"
        )

    create_nuscenes_infos(
        root_path=str(root_path),
        info_prefix=args.info_prefix,
        version="v1.0-trainval",
        max_sweeps=args.max_sweeps,
    )

    expected_sizes = {"train": 28000, "val": 6000}
    for split, expected_size in expected_sizes.items():
        info_path = (
            root_path
            / f"{args.info_prefix}_temporal_infos_{split}.pkl"
        )
        actual_size = len(mmcv.load(str(info_path))["infos"])
        if actual_size != expected_size:
            raise RuntimeError(
                f"{info_path} contains {actual_size} samples; "
                f"expected {expected_size}"
            )
        print(f"Validated {split}: {actual_size} samples")


if __name__ == "__main__":
    main()
