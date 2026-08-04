#!/usr/bin/env python3
"""Turn a carla_live_capture recording into an info pkl the model can read.

The capture rig mirrors nuCarla's calibration, so the sensor-to-lidar
transforms are lifted straight from a nuCarla info file instead of being
re-derived. Only the fields the dataset touches in test mode are filled in:
annotations are absent because this clip has no ground truth.
"""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import numpy as np
from pyquaternion import Quaternion

CAM_ORDER = [
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", default="data/carla_live/capture.json")
    parser.add_argument(
        "--reference",
        default="data/nucarla/nucarla_temporal_infos_val.pkl",
        help="nuCarla infos supplying the calibration",
    )
    parser.add_argument("--out", default="data/carla_live/carla_live_infos.pkl")
    args = parser.parse_args()

    with open(args.capture) as stream:
        capture = json.load(stream)

    with open(args.reference, "rb") as stream:
        reference = pickle.load(stream)["infos"][0]

    calibration = {
        channel: dict(
            sensor2lidar_rotation=np.asarray(
                reference["cams"][channel]["sensor2lidar_rotation"]
            ),
            sensor2lidar_translation=np.asarray(
                reference["cams"][channel]["sensor2lidar_translation"]
            ),
            cam_intrinsic=np.asarray(reference["cams"][channel]["cam_intrinsic"]),
        )
        for channel in CAM_ORDER
    }

    infos = []
    scene_token = "carla_live_0001"
    records = capture["records"]
    for position, record in enumerate(records):
        yaw = np.radians(record["ego_yaw_deg"])
        cams = {}
        for channel in CAM_ORDER:
            cams[channel] = dict(
                data_path=record["cams"][channel],
                type=channel,
                timestamp=record["timestamp"],
                sensor2lidar_rotation=calibration[channel]["sensor2lidar_rotation"],
                sensor2lidar_translation=calibration[channel][
                    "sensor2lidar_translation"
                ],
                cam_intrinsic=calibration[channel]["cam_intrinsic"],
            )
        infos.append(
            dict(
                token=f"carla_live_{position:06d}",
                scene_token=scene_token,
                frame_idx=position,
                timestamp=record["timestamp"],
                prev=f"carla_live_{position - 1:06d}" if position else "",
                next=(
                    f"carla_live_{position + 1:06d}"
                    if position + 1 < len(records)
                    else ""
                ),
                sweeps=[],
                lidar_path="",
                cams=cams,
                lidar2ego_translation=list(reference["lidar2ego_translation"]),
                lidar2ego_rotation=list(reference["lidar2ego_rotation"]),
                ego2global_translation=list(record["ego_translation"]),
                ego2global_rotation=list(Quaternion(axis=[0, 0, 1], radians=yaw)),
                # Empty annotation arrays: this clip is unlabelled, and the
                # dataset only reads them outside test mode.
                gt_boxes=np.zeros((0, 7), dtype=np.float32),
                gt_names=np.array([], dtype="<U32"),
                gt_velocity=np.zeros((0, 2), dtype=np.float32),
                num_lidar_pts=np.zeros((0,), dtype=np.int64),
                num_radar_pts=np.zeros((0,), dtype=np.int64),
                valid_flag=np.zeros((0,), dtype=bool),
            )
        )

    output = Path(args.out)
    with output.open("wb") as stream:
        pickle.dump({"infos": infos, "metadata": {"version": "carla_live"}}, stream)
    print(f"wrote {len(infos)} frames to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
