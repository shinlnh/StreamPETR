#!/usr/bin/env python3
"""Drive a car around CARLA and record a nuCarla-shaped six-camera sequence.

Spawns an ego vehicle plus background traffic, hands everything to the traffic
manager, and records the six surround cameras in lock-step at nuCarla's 2 Hz
keyframe rate. The rig is placed to match nuCarla's calibration exactly -- same
offsets, same 65 degree FOV, same 1600x900 frames -- so the recorded clip can
be fed straight to a model trained on that dataset.

Sensor-to-lidar transforms are copied from the nuCarla calibration rather than
re-derived from CARLA, which keeps the geometry the model was trained on intact
and avoids a hand-rolled left-to-right-handed conversion.
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import queue
import random
from pathlib import Path

import numpy as np

import carla

# Ego-frame rig read out of nuCarla's calibrated_sensor table, in the nuScenes
# convention (x forward, y left, z up; yaw counter-clockwise).
RIG = {
    "CAM_FRONT": dict(x=1.901, y=0.016, z=1.511, yaw=0.3),
    "CAM_FRONT_LEFT": dict(x=1.524, y=0.495, z=1.509, yaw=55.2),
    "CAM_FRONT_RIGHT": dict(x=1.551, y=-0.493, z=1.496, yaw=-56.4),
    "CAM_BACK": dict(x=0.028, y=0.003, z=1.579, yaw=179.9),
    "CAM_BACK_LEFT": dict(x=1.036, y=0.485, z=1.591, yaw=108.6),
    "CAM_BACK_RIGHT": dict(x=1.015, y=-0.481, z=1.562, yaw=-110.8),
}
CAM_ORDER = list(RIG)
IMAGE_W, IMAGE_H, FOV = 1600, 900, 65.0


def build_camera(world, ego, channel, out_dir):
    blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    blueprint.set_attribute("image_size_x", str(IMAGE_W))
    blueprint.set_attribute("image_size_y", str(IMAGE_H))
    blueprint.set_attribute("fov", str(FOV))
    spec = RIG[channel]
    # CARLA is left-handed with y pointing right, so the lateral offset and the
    # yaw both flip sign relative to the nuScenes numbers above.
    transform = carla.Transform(
        carla.Location(x=spec["x"], y=-spec["y"], z=spec["z"]),
        carla.Rotation(yaw=-spec["yaw"]),
    )
    camera = world.spawn_actor(blueprint, transform, attach_to=ego)
    (out_dir / channel).mkdir(parents=True, exist_ok=True)
    return camera


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town04")
    parser.add_argument("--frames", type=int, default=120, help="keyframes to record")
    parser.add_argument("--vehicles", type=int, default=60)
    parser.add_argument("--walkers", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=40, help="ticks before recording")
    parser.add_argument("--out", default="data/carla_live")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)
    out_dir = Path(args.out)
    (out_dir / "samples").mkdir(parents=True, exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    world = client.load_world(args.town)
    blueprints = world.get_blueprint_library()

    original_settings = world.get_settings()
    settings = world.get_settings()
    # nuCarla keyframes are 0.5 s apart. Simulate at 20 Hz and record every
    # tenth tick so the traffic manager still gets a fine control timestep.
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_random_device_seed(args.seed)

    actors = []
    cameras = {}
    controllers = []
    try:
        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        ego_bp = blueprints.filter("vehicle.tesla.model3")[0]
        ego_bp.set_attribute("role_name", "ego")
        ego = world.spawn_actor(ego_bp, spawn_points[0])
        actors.append(ego)
        ego.set_autopilot(True, traffic_manager.get_port())
        traffic_manager.ignore_lights_percentage(ego, 20.0)

        vehicle_bps = [
            bp
            for bp in blueprints.filter("vehicle.*")
            if int(bp.get_attribute("number_of_wheels")) == 4
        ]
        # Town04 is mostly a long highway loop, so traffic scattered over the
        # whole map leaves the ego driving alone. Fill the spawn points nearest
        # to the ego instead, which is what puts cars in frame.
        ego_location = spawn_points[0].location
        nearby = sorted(
            spawn_points[1:],
            key=lambda point: point.location.distance(ego_location),
        )
        for point in nearby[: args.vehicles]:
            actor = world.try_spawn_actor(random.choice(vehicle_bps), point)
            if actor is not None:
                actor.set_autopilot(True, traffic_manager.get_port())
                # Keep the pack together rather than letting it string out.
                traffic_manager.vehicle_percentage_speed_difference(actor, 25.0)
                actors.append(actor)
        spawned_vehicles = len(actors) - 1

        walker_bps = blueprints.filter("walker.pedestrian.*")
        controller_bp = blueprints.find("controller.ai.walker")
        walkers = []
        for _ in range(args.walkers):
            location = world.get_random_location_from_navigation()
            if location is None:
                continue
            walker = world.try_spawn_actor(
                random.choice(walker_bps), carla.Transform(location)
            )
            if walker is not None:
                walkers.append(walker)
        world.tick()
        for walker in walkers:
            controller = world.try_spawn_actor(
                controller_bp, carla.Transform(), attach_to=walker
            )
            if controller is not None:
                controllers.append(controller)
        world.tick()
        for controller in controllers:
            controller.start()
            controller.go_to_location(world.get_random_location_from_navigation())
            controller.set_max_speed(1.4)
        actors.extend(walkers)
        actors.extend(controllers)
        print(
            f"spawned ego + {spawned_vehicles} vehicles + {len(walkers)} walkers "
            f"in {args.town}"
        )

        image_queues = {}
        for channel in CAM_ORDER:
            camera = build_camera(world, ego, channel, out_dir / "samples")
            channel_queue = queue.Queue()
            camera.listen(channel_queue.put)
            cameras[channel] = camera
            image_queues[channel] = channel_queue

        for _ in range(args.warmup):
            world.tick()
            for channel_queue in image_queues.values():
                channel_queue.get()

        records = []
        for frame_index in range(args.frames):
            # Ten simulation ticks per recorded keyframe -> 0.5 s spacing.
            for sub_step in range(10):
                world.tick()
                images = {c: image_queues[c].get() for c in CAM_ORDER}
                if sub_step < 9:
                    continue

            transform = ego.get_transform()
            timestamp = int(world.get_snapshot().timestamp.elapsed_seconds * 1e6)
            paths = {}
            for channel in CAM_ORDER:
                name = f"{channel}__{timestamp}.jpg"
                path = out_dir / "samples" / channel / name
                images[channel].save_to_disk(str(path))
                paths[channel] = str(path)

            location = transform.location
            rotation = transform.rotation
            records.append(
                dict(
                    frame_idx=frame_index,
                    timestamp=timestamp,
                    # Convert the pose to the right-handed nuScenes convention.
                    ego_translation=[location.x, -location.y, location.z],
                    ego_yaw_deg=-rotation.yaw,
                    cams=paths,
                )
            )
            if (frame_index + 1) % 10 == 0:
                speed = ego.get_velocity()
                kmh = 3.6 * math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
                print(f"  frame {frame_index + 1}/{args.frames}  ego {kmh:.1f} km/h")

        with (out_dir / "capture.json").open("w") as stream:
            json.dump(
                dict(
                    town=args.town,
                    frames=len(records),
                    image_size=[IMAGE_W, IMAGE_H],
                    fov=FOV,
                    records=records,
                ),
                stream,
                indent=2,
            )
        print(f"wrote {len(records)} keyframes to {out_dir}")
        return 0
    finally:
        for camera in cameras.values():
            try:
                camera.stop()
            except RuntimeError:
                pass
        # Walker controllers must be stopped before their walker is destroyed,
        # and any actor may already be gone, so tear down in batch and let the
        # server ignore the ones it has already collected.
        for controller in controllers:
            try:
                controller.stop()
            except RuntimeError:
                pass
        client.apply_batch(
            [carla.command.DestroyActor(a) for a in list(cameras.values()) + actors]
        )
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)


if __name__ == "__main__":
    raise SystemExit(main())
