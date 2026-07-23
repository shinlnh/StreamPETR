#!/usr/bin/env python3
"""Collect a nuScenes-info-compatible, six-camera CARLA dataset for StreamPETR.

The collector is intentionally self-contained so it can run in the same
carla-ros-bridge tool image used by ad_sim_dev_launch.  It writes RGB images
and StreamPETR temporal info PKLs directly; no nuScenes JSON tables are needed
for training.
"""

import argparse
import json
import math
import pickle
import queue
import random
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import config as cfg

try:
    import carla
except ImportError as exc:  # pragma: no cover - only available in CARLA image
    raise SystemExit(
        "CARLA PythonAPI is missing. Run this via scripts/collect_town04.sh."
    ) from exc


CV_TO_UNREAL = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)
UNREAL_TO_CANONICAL = np.diag([1.0, -1.0, 1.0]).astype(np.float64)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect six-camera Town04 data for StreamPETR"
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=cfg.TRAFFIC_MANAGER_PORT)
    parser.add_argument("--town", default=cfg.TOWN)
    parser.add_argument("--output", default=cfg.OUTPUT_ROOT)
    parser.add_argument("--episodes", type=int, default=cfg.EPISODES)
    parser.add_argument(
        "--frames-per-episode", type=int, default=cfg.FRAMES_PER_EPISODE
    )
    parser.add_argument("--capture-every", type=int, default=cfg.CAPTURE_EVERY)
    parser.add_argument("--warmup-frames", type=int, default=cfg.WARMUP_FRAMES)
    parser.add_argument("--vehicles", type=int, default=cfg.NUM_VEHICLES)
    parser.add_argument("--fps", type=int, default=cfg.FPS)
    parser.add_argument("--seed", type=int, default=cfg.SEED)
    parser.add_argument("--val-ratio", type=float, default=cfg.VAL_RATIO)
    parser.add_argument(
        "--scenario-matrix",
        action="store_true",
        help="collect the balanced weather x maneuver matrix instead of episodes",
    )
    parser.add_argument(
        "--weather-presets",
        nargs="+",
        default=list(cfg.WEATHER_PRESETS),
        help="CARLA WeatherParameters names used by --scenario-matrix",
    )
    parser.add_argument(
        "--maneuvers",
        nargs="+",
        choices=cfg.MANEUVERS,
        default=list(cfg.MANEUVERS),
    )
    parser.add_argument(
        "--frames-per-case", type=int, default=cfg.FRAMES_PER_CASE
    )
    parser.add_argument(
        "--clips-per-case", type=int, default=cfg.CLIPS_PER_CASE
    )
    parser.add_argument(
        "--jpeg-quality", type=int, default=cfg.JPEG_QUALITY
    )
    parser.add_argument(
        "--min-free-disk-gb", type=float, default=cfg.MIN_FREE_DISK_GB
    )
    parser.add_argument("--resume", action="store_true")
    parser.add_argument(
        "--max-scenes-per-run",
        type=int,
        default=0,
        help=(
            "stop cleanly after this many newly collected scenes; 0 collects "
            "all remaining scenes. Useful for periodically restarting CARLA"
        ),
    )
    parser.add_argument(
        "--no-reload-world",
        action="store_true",
        help="Use the currently loaded world instead of client.load_world().",
    )
    args = parser.parse_args()
    if args.episodes < 1 or args.frames_per_episode < 1:
        parser.error("episodes and frames-per-episode must be positive")
    if args.capture_every < 1 or args.fps < 1:
        parser.error("capture-every and fps must be positive")
    if not 0.0 <= args.val_ratio < 1.0:
        parser.error("val-ratio must be in [0, 1)")
    if args.frames_per_case < 1 or args.clips_per_case < 1:
        parser.error("frames-per-case and clips-per-case must be positive")
    if args.frames_per_case % args.clips_per_case:
        parser.error("frames-per-case must be divisible by clips-per-case")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("jpeg-quality must be in [1, 100]")
    if args.min_free_disk_gb < 0:
        parser.error("min-free-disk-gb cannot be negative")
    if args.max_scenes_per_run < 0:
        parser.error("max-scenes-per-run cannot be negative")
    return args


def transform_matrix(transform):
    return np.asarray(transform.get_matrix(), dtype=np.float64)


def inverse_transform_matrix(transform):
    return np.asarray(transform.get_inverse_matrix(), dtype=np.float64)


def transform_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    rotation = matrix[:3, :3]
    # CARLA/Unreal uses a left-handed yaw-pitch-roll matrix where R[2, 0]
    # equals +sin(pitch) and R[2, 1] equals -cos(pitch) * sin(roll).
    pitch = math.asin(float(np.clip(rotation[2, 0], -1.0, 1.0)))
    cosine_pitch = math.cos(pitch)
    if abs(cosine_pitch) > 1e-6:
        roll = math.atan2(-rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return carla.Transform(
        carla.Location(
            x=float(matrix[0, 3]),
            y=float(matrix[1, 3]),
            z=float(matrix[2, 3]),
        ),
        carla.Rotation(
            pitch=math.degrees(pitch),
            yaw=math.degrees(yaw),
            roll=math.degrees(roll),
        ),
    )


def compose_transforms(parent, relative):
    return transform_from_matrix(transform_matrix(parent) @ transform_matrix(relative))


class VirtualEgo:
    """Collision-free reference pose for the deterministic camera rig."""

    id = -1
    is_virtual = True
    is_alive = True

    def __init__(self, transform):
        self._transform = transform

    def get_transform(self):
        return self._transform

    def get_location(self):
        return self._transform.location

    def set_transform(self, transform):
        self._transform = transform


def rotation_matrix(rotation):
    return transform_matrix(carla.Transform(carla.Location(), rotation))[:3, :3]


def matrix_to_quaternion(matrix):
    """Return a normalized quaternion in nuScenes [w, x, y, z] order."""
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [0.25 * s, (m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        )
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            quat = np.array(
                [(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                 (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
            )
        elif i == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            quat = np.array(
                [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                 0.25 * s, (m[1, 2] + m[2, 1]) / s]
            )
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            quat = np.array(
                [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                 (m[1, 2] + m[2, 1]) / s, 0.25 * s]
            )
    return (quat / np.linalg.norm(quat)).tolist()


def camera_intrinsic(width, height, fov_degrees):
    focal = width / (2.0 * math.tan(math.radians(fov_degrees) / 2.0))
    return np.array(
        [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )


def carla_category_from_type_id(type_id):
    type_id = type_id.lower()
    if type_id.startswith("walker.pedestrian"):
        return "pedestrian"
    if not type_id.startswith("vehicle."):
        return None
    bicycle_tokens = ("crossbike", "century", "omafiets", "bicycle", "bike")
    motorcycle_tokens = ("harley", "kawasaki", "vespa", "yamaha", "motorcycle")
    truck_tokens = ("truck", "carlacola", "firetruck", "ambulance")
    bus_tokens = ("bus", "fusorosa", "sprinter", "volkswagen.t2")
    if any(token in type_id for token in bicycle_tokens):
        return "bicycle"
    if any(token in type_id for token in motorcycle_tokens):
        return "motorcycle"
    if any(token in type_id for token in truck_tokens):
        return "truck"
    if any(token in type_id for token in bus_tokens):
        return "bus"
    return "car"


def carla_category(actor):
    return carla_category_from_type_id(actor.type_id)


def get_matching_sensor_frame(sensor_queue, frame, timeout=10.0):
    while True:
        image = sensor_queue.get(timeout=timeout)
        if image.frame == frame:
            return image
        if image.frame > frame:
            raise RuntimeError(
                "Sensor advanced past world frame {} (received {})".format(
                    frame, image.frame
                )
            )


def spawn_vehicle(world, blueprint, transform):
    blueprint.set_attribute("role_name", "hero")
    if blueprint.has_attribute("color"):
        colors = blueprint.get_attribute("color").recommended_values
        if colors:
            blueprint.set_attribute("color", random.choice(colors))
    return world.try_spawn_actor(blueprint, transform)


def local_traffic_spawns(world_map, ego_location):
    """Build valid driving-lane spawns within StreamPETR's annotation range."""
    center = world_map.get_waypoint(
        ego_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if center is None:
        return []
    lanes = [center]
    for direction in ("get_left_lane", "get_right_lane"):
        waypoint = center
        for _ in range(2):
            waypoint = getattr(waypoint, direction)()
            if waypoint is None or waypoint.lane_type != carla.LaneType.Driving:
                break
            lanes.append(waypoint)

    candidates = []
    seen = set()
    for lane in lanes:
        for distance in (18.0, 32.0, 46.0):
            waypoints = list(lane.next(distance)) + list(lane.previous(distance))
            for waypoint in waypoints:
                transform = waypoint.transform
                key = (
                    round(transform.location.x, 1),
                    round(transform.location.y, 1),
                    round(transform.rotation.yaw, 1),
                )
                if key in seen:
                    continue
                seen.add(key)
                transform.location.z += 0.3
                candidates.append(transform)
    return candidates


def signed_yaw_delta(from_yaw, to_yaw):
    return (to_yaw - from_yaw + 180.0) % 360.0 - 180.0


def maneuver_from_waypoints(entry, exit_waypoint):
    angle = signed_yaw_delta(
        entry.transform.rotation.yaw,
        exit_waypoint.transform.rotation.yaw,
    )
    if abs(angle) < 35.0:
        return "straight", angle
    if abs(angle) >= 145.0:
        return None, angle
    if abs(angle) < 60.0:
        return None, angle
    # CARLA/Unreal's positive yaw turns clockwise in the map coordinates.
    return ("left" if angle < 0.0 else "right"), angle


def closest_same_heading(waypoints, reference_yaw):
    if not waypoints:
        return None
    return min(
        waypoints,
        key=lambda waypoint: abs(
            signed_yaw_delta(reference_yaw, waypoint.transform.rotation.yaw)
        ),
    )


def interpolate_line(start, end, spacing):
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    distance = float(np.linalg.norm(end[:2] - start[:2]))
    count = max(2, int(math.ceil(distance / spacing)) + 1)
    return [start + (end - start) * ratio for ratio in np.linspace(0.0, 1.0, count)]


def build_route_trajectory(approach, entry, exit_waypoint, continuations):
    """Create a smooth deterministic pose sequence through one junction."""
    spacing = cfg.ROUTE_SAMPLE_METERS
    start = np.array(
        [
            approach.transform.location.x,
            approach.transform.location.y,
            approach.transform.location.z + 0.3,
        ]
    )
    entry_point = np.array(
        [
            entry.transform.location.x,
            entry.transform.location.y,
            entry.transform.location.z + 0.3,
        ]
    )
    exit_point = np.array(
        [
            exit_waypoint.transform.location.x,
            exit_waypoint.transform.location.y,
            exit_waypoint.transform.location.z + 0.3,
        ]
    )
    points = interpolate_line(start, entry_point, spacing)[:-1]

    chord = float(np.linalg.norm(exit_point[:2] - entry_point[:2]))
    entry_yaw = math.radians(entry.transform.rotation.yaw)
    exit_yaw = math.radians(exit_waypoint.transform.rotation.yaw)
    first_control = entry_point.copy()
    second_control = exit_point.copy()
    first_control[:2] += 0.55 * chord * np.array(
        [math.cos(entry_yaw), math.sin(entry_yaw)]
    )
    second_control[:2] -= 0.55 * chord * np.array(
        [math.cos(exit_yaw), math.sin(exit_yaw)]
    )
    curve_count = max(3, int(math.ceil(chord * 1.5 / spacing)) + 1)
    for ratio in np.linspace(0.0, 1.0, curve_count):
        inverse = 1.0 - ratio
        point = (
            inverse ** 3 * entry_point
            + 3.0 * inverse ** 2 * ratio * first_control
            + 3.0 * inverse * ratio ** 2 * second_control
            + ratio ** 3 * exit_point
        )
        points.append(point)

    previous = exit_point
    for continuation in continuations:
        target = np.array(
            [
                continuation.transform.location.x,
                continuation.transform.location.y,
                continuation.transform.location.z + 0.3,
            ]
        )
        points.extend(interpolate_line(previous, target, spacing)[1:])
        previous = target

    transforms = []
    for index, point in enumerate(points):
        before = points[max(0, index - 1)]
        after = points[min(len(points) - 1, index + 1)]
        direction = after - before
        yaw = math.degrees(math.atan2(direction[1], direction[0]))
        transforms.append(
            carla.Transform(
                carla.Location(
                    x=float(point[0]), y=float(point[1]), z=float(point[2])
                ),
                carla.Rotation(yaw=yaw),
            )
        )
    return transforms


def build_maneuver_route_families(world_map, approach_distance=10.0):
    """Find junction approaches that have explicit left/right/straight paths."""
    junctions = {}
    for waypoint in world_map.generate_waypoints(2.0):
        if waypoint.is_junction:
            junction = waypoint.get_junction()
            junctions[junction.id] = junction

    families = []
    for junction_id, junction in sorted(junctions.items()):
        approaches = {}
        for entry, exit_waypoint in junction.get_waypoints(carla.LaneType.Driving):
            maneuver, angle = maneuver_from_waypoints(entry, exit_waypoint)
            if maneuver is None:
                continue
            key = (
                round(entry.transform.location.x, 1),
                round(entry.transform.location.y, 1),
                round(entry.transform.rotation.yaw % 360.0, 1),
            )
            approaches.setdefault(key, {})[maneuver] = (
                entry,
                exit_waypoint,
                angle,
            )
        for approach_key, routes in approaches.items():
            if not all(name in routes for name in cfg.MANEUVERS):
                continue
            family = {}
            valid = True
            for maneuver in cfg.MANEUVERS:
                entry, exit_waypoint, angle = routes[maneuver]
                approach = closest_same_heading(
                    entry.previous(approach_distance),
                    entry.transform.rotation.yaw,
                )
                if approach is None:
                    valid = False
                    break
                path = [entry.transform.location, exit_waypoint.transform.location]
                continuations = []
                for distance in (15.0, 30.0, 45.0):
                    candidate = closest_same_heading(
                        exit_waypoint.next(distance),
                        exit_waypoint.transform.rotation.yaw,
                    )
                    if candidate is not None:
                        path.append(candidate.transform.location)
                        continuations.append(candidate)
                trajectory = build_route_trajectory(
                    approach, entry, exit_waypoint, continuations
                )
                family[maneuver] = {
                    "spawn_transform": trajectory[0],
                    "path": path,
                    "trajectory": trajectory,
                    "junction_id": junction_id,
                    "approach": approach_key,
                    "entry": [
                        entry.transform.location.x,
                        entry.transform.location.y,
                    ],
                    "exit": [
                        exit_waypoint.transform.location.x,
                        exit_waypoint.transform.location.y,
                    ],
                    "turn_angle": angle,
                }
            if valid:
                families.append(family)
    if not families:
        raise RuntimeError("No junction approach has left/right/straight routes")
    return families


def filter_route_families_for_capture(families, args):
    """Keep routes that complete the requested maneuver inside one clip."""
    frames_per_clip = args.frames_per_case // args.clips_per_case
    last_capture_step = (frames_per_clip - 1) * args.capture_every
    capture_distance = last_capture_step * cfg.EGO_ROUTE_SPEED_MPS / args.fps
    trajectory_index = int(round(capture_distance / cfg.ROUTE_SAMPLE_METERS))
    eligible = []
    for family in families:
        valid = True
        for maneuver in args.maneuvers:
            trajectory = family[maneuver]["trajectory"]
            end_index = min(trajectory_index, len(trajectory) - 1)
            actual_turn = signed_yaw_delta(
                trajectory[0].rotation.yaw,
                trajectory[end_index].rotation.yaw,
            )
            if maneuver == "left" and actual_turn > -60.0:
                valid = False
            elif maneuver == "right" and actual_turn < 60.0:
                valid = False
            elif maneuver == "straight" and abs(actual_turn) > 15.0:
                valid = False
        if valid:
            eligible.append(family)
    if not eligible:
        raise RuntimeError("No route family completes its maneuver inside a clip")
    return eligible


def balanced_blueprint_schedule(blueprints, count, rng):
    """Return a diverse blueprint schedule instead of mostly passenger cars."""
    pools = {}
    for blueprint in blueprints:
        category = carla_category_from_type_id(blueprint.id)
        pools.setdefault(category, []).append(blueprint)
    # Expected mix for 60 vehicles: 39 cars, 6 trucks, 6 buses, 5
    # motorcycles, and 4 bicycles. Missing CARLA blueprint categories fall
    # back to cars without aborting the collection.
    ratios = (
        ("car", 0.65),
        ("truck", 0.10),
        ("bus", 0.10),
        ("motorcycle", 0.08),
        ("bicycle", 0.07),
    )
    categories = []
    for category, ratio in ratios:
        categories.extend([category] * int(round(count * ratio)))
    categories = categories[:count]
    categories.extend(["car"] * (count - len(categories)))
    rng.shuffle(categories)
    car_pool = pools.get("car", blueprints)
    return [rng.choice(pools.get(category, car_pool)) for category in categories]


def spawn_episode_actors(
    world, traffic_manager, num_vehicles, rng, maneuver=None, ego_route=None
):
    blueprints = [
        bp
        for bp in world.get_blueprint_library().filter("vehicle.*")
        if not bp.id.endswith("isetta") and not bp.id.endswith("carlacola")
    ]
    spawn_points = list(world.get_map().get_spawn_points())
    rng.shuffle(spawn_points)
    rng.shuffle(blueprints)
    if not spawn_points:
        raise RuntimeError("The CARLA map has no vehicle spawn points")

    ego = None
    ego_spawn_index = None
    if ego_route is not None:
        ego = VirtualEgo(ego_route["spawn_transform"])
    else:
        preferred = world.get_blueprint_library().find("vehicle.tesla.model3")
        for index, transform in enumerate(spawn_points):
            ego = spawn_vehicle(world, preferred, transform)
            if ego is not None:
                ego_spawn_index = index
                break
    if ego is None:
        raise RuntimeError("Could not spawn the ego vehicle")

    # Town04's predefined spawn points are sparse. Add waypoint-based spawns at
    # 18/32/46 m so every episode starts with useful objects inside the model's
    # 51.2 m range, then fill the remaining quota from normal map spawns.
    # Actor state is not populated client-side until the next world tick; use
    # the successful spawn transform here instead of ego.get_location().
    ego_location = (
        ego_route["spawn_transform"].location
        if ego_route is not None
        else spawn_points[ego_spawn_index].location
    )
    local_spawns = local_traffic_spawns(world.get_map(), ego_location)
    if ego_route is not None:
        # Do not place traffic directly on the short controlled route. Other
        # incoming/outgoing lanes remain populated and visible to all cameras.
        route_locations = [ego_location] + list(ego_route["path"])
        local_spawns = [
            transform
            for transform in local_spawns
            if all(
                transform.location.distance(location) > 6.0
                for location in route_locations
            )
        ]
    rng.shuffle(local_spawns)
    remaining_spawns = [
        transform
        for index, transform in enumerate(spawn_points)
        if index != ego_spawn_index
        and transform.location.distance(ego_location) > 5.0
    ]
    if ego_route is not None:
        remaining_spawns = [
            transform
            for transform in remaining_spawns
            if all(
                transform.location.distance(location) > 6.0
                for location in route_locations
            )
        ]
    remaining_spawns.sort(
        key=lambda transform: transform.location.distance(ego_location)
    )
    nearby_pool_size = min(len(remaining_spawns), max(num_vehicles * 2, 1))
    nearby_spawns = remaining_spawns[:nearby_pool_size]
    distant_spawns = remaining_spawns[nearby_pool_size:]
    rng.shuffle(nearby_spawns)
    rng.shuffle(distant_spawns)

    traffic = []
    blueprint_schedule = balanced_blueprint_schedule(blueprints, num_vehicles, rng)
    for transform in local_spawns + nearby_spawns + distant_spawns:
        if len(traffic) >= num_vehicles:
            break
        blueprint = blueprint_schedule[len(traffic)]
        blueprint.set_attribute("role_name", "autopilot")
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            actor.set_autopilot(True, traffic_manager.get_port())
            if hasattr(traffic_manager, "update_vehicle_lights"):
                traffic_manager.update_vehicle_lights(actor, True)
            traffic.append(actor)

    if ego_route is None:
        ego.set_autopilot(True, traffic_manager.get_port())
        if hasattr(traffic_manager, "update_vehicle_lights"):
            traffic_manager.update_vehicle_lights(ego, True)
        traffic_manager.auto_lane_change(ego, False)
        traffic_manager.ignore_lights_percentage(ego, 0.0)
        traffic_manager.distance_to_leading_vehicle(ego, 3.0)
    if ego_route is None and maneuver is not None:
        route_command = maneuver.capitalize()
        traffic_manager.set_route(
            ego, [route_command] * cfg.ROUTE_COMMAND_REPETITIONS
        )
    category_counts = {}
    for actor in traffic:
        name = carla_category(actor)
        category_counts[name] = category_counts.get(name, 0) + 1
    print(
        "Spawned {} traffic vehicles: {}".format(len(traffic), category_counts),
        flush=True,
    )
    return ego, traffic


def spawn_cameras(world, ego):
    sensors = {}
    queues = {}
    library = world.get_blueprint_library()
    for name in cfg.CAMERA_NAMES:
        blueprint = library.find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(cfg.IMAGE_WIDTH))
        blueprint.set_attribute("image_size_y", str(cfg.IMAGE_HEIGHT))
        blueprint.set_attribute("fov", str(cfg.CAMERA_FOVS[name]))
        blueprint.set_attribute("sensor_tick", "0.0")
        x, y, z, pitch, yaw, roll = cfg.CAMERA_TRANSFORMS[name]
        relative_transform = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
        )
        if getattr(ego, "is_virtual", False):
            sensor = world.spawn_actor(
                blueprint,
                compose_transforms(ego.get_transform(), relative_transform),
            )
        else:
            sensor = world.spawn_actor(
                blueprint,
                relative_transform,
                attach_to=ego,
                attachment_type=carla.AttachmentType.Rigid,
            )
        image_queue = queue.Queue()
        sensor.listen(image_queue.put)
        sensors[name] = sensor
        queues[name] = image_queue
    return sensors, queues


def update_virtual_cameras(client, cameras, ego):
    if not getattr(ego, "is_virtual", False):
        return
    commands = []
    for name, sensor in cameras.items():
        x, y, z, pitch, yaw, roll = cfg.CAMERA_TRANSFORMS[name]
        relative_transform = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
        )
        commands.append(
            carla.command.ApplyTransform(
                sensor.id,
                compose_transforms(ego.get_transform(), relative_transform),
            )
        )
    responses = client.apply_batch_sync(commands, False)
    errors = [response.error for response in responses if response.has_error()]
    if errors:
        raise RuntimeError("Could not update virtual camera rig: {}".format(errors))


def camera_calibration(camera_name, camera_world_transform, ego, intrinsic):
    ego_inv = inverse_transform_matrix(ego.get_transform())
    cam_world = transform_matrix(camera_world_transform)
    cam_to_ego_unreal = ego_inv @ cam_world
    rotation = (
        UNREAL_TO_CANONICAL
        @ cam_to_ego_unreal[:3, :3]
        @ CV_TO_UNREAL
    )
    translation = UNREAL_TO_CANONICAL @ cam_to_ego_unreal[:3, 3]
    x, y, z, pitch, yaw, roll = cfg.CAMERA_TRANSFORMS[camera_name]
    configured = transform_matrix(
        carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
        )
    )
    expected_rotation = (
        UNREAL_TO_CANONICAL @ configured[:3, :3] @ CV_TO_UNREAL
    )
    expected_translation = UNREAL_TO_CANONICAL @ configured[:3, 3]
    max_error = max(
        float(np.max(np.abs(rotation - expected_rotation))),
        float(np.max(np.abs(translation - expected_translation))),
    )
    if max_error > 1e-3:
        raise RuntimeError(
            "{} captured out of sync with the virtual rig "
            "(calibration error {:.6f})".format(camera_name, max_error)
        )
    return {
        "sensor2lidar_rotation": rotation.astype(np.float32),
        "sensor2lidar_translation": translation.astype(np.float32),
        "cam_intrinsic": intrinsic.copy(),
    }


def project_actor_box(actor, camera_world_transform, intrinsic, width, height):
    vertices = actor.bounding_box.get_world_vertices(actor.get_transform())
    world_to_camera = inverse_transform_matrix(camera_world_transform)
    pixels = []
    depths = []
    for vertex in vertices:
        point_world = np.array([vertex.x, vertex.y, vertex.z, 1.0])
        point_unreal = (world_to_camera @ point_world)[:3]
        point_cv = np.array([point_unreal[1], -point_unreal[2], point_unreal[0]])
        if point_cv[2] <= 0.1:
            continue
        pixel = intrinsic @ point_cv
        pixels.append(pixel[:2] / pixel[2])
        depths.append(point_cv[2])
    if len(pixels) < 2:
        return None
    pixels = np.asarray(pixels)
    x1 = float(np.clip(pixels[:, 0].min(), 0.0, width - 1.0))
    y1 = float(np.clip(pixels[:, 1].min(), 0.0, height - 1.0))
    x2 = float(np.clip(pixels[:, 0].max(), 0.0, width - 1.0))
    y2 = float(np.clip(pixels[:, 1].max(), 0.0, height - 1.0))
    if x2 - x1 < 1.0 or y2 - y1 < 1.0:
        return None

    actor_matrix = transform_matrix(actor.get_transform())
    bbox_location = actor.bounding_box.location
    center_world = actor_matrix @ np.array(
        [bbox_location.x, bbox_location.y, bbox_location.z, 1.0]
    )
    center_unreal = (world_to_camera @ center_world)[:3]
    center_cv = np.array([center_unreal[1], -center_unreal[2], center_unreal[0]])
    if center_cv[2] <= 0.1:
        return None
    center_pixel = intrinsic @ center_cv
    center_pixel = center_pixel[:2] / center_pixel[2]
    return (
        np.array([x1, y1, x2, y2], dtype=np.float32),
        center_pixel.astype(np.float32),
        np.float32(center_cv[2]),
    )


def collect_ground_truth(world, ego, camera_transforms, intrinsics):
    ego_inv = inverse_transform_matrix(ego.get_transform())
    ego_rotation_inv = ego_inv[:3, :3]
    boxes = []
    names = []
    velocities = []
    actors = []

    candidates = list(world.get_actors().filter("vehicle.*"))
    candidates.extend(world.get_actors().filter("walker.pedestrian.*"))
    for actor in candidates:
        if actor.id == ego.id:
            continue
        name = carla_category(actor)
        if name not in cfg.CLASS_NAMES:
            continue
        actor_matrix = transform_matrix(actor.get_transform())
        bbox = actor.bounding_box
        center_world = actor_matrix @ np.array(
            [bbox.location.x, bbox.location.y, bbox.location.z, 1.0]
        )
        center_ego_unreal = (ego_inv @ center_world)[:3]
        center = UNREAL_TO_CANONICAL @ center_ego_unreal
        if np.linalg.norm(center[:2]) > cfg.MAX_ANNOTATION_DISTANCE:
            continue

        bbox_rotation = rotation_matrix(bbox.rotation)
        box_in_ego_unreal = ego_rotation_inv @ actor_matrix[:3, :3] @ bbox_rotation
        box_rotation = (
            UNREAL_TO_CANONICAL @ box_in_ego_unreal @ UNREAL_TO_CANONICAL
        )
        yaw = math.atan2(box_rotation[1, 0], box_rotation[0, 0])
        dimensions = np.array(
            [2.0 * bbox.extent.x, 2.0 * bbox.extent.y, 2.0 * bbox.extent.z]
        )
        velocity_world = actor.get_velocity()
        velocity_ego_unreal = ego_rotation_inv @ np.array(
            [velocity_world.x, velocity_world.y, velocity_world.z]
        )
        velocity = UNREAL_TO_CANONICAL @ velocity_ego_unreal

        boxes.append(np.concatenate([center, dimensions, [yaw]]))
        names.append(name)
        velocities.append(velocity[:2])
        actors.append(actor)

    class_to_index = {name: index for index, name in enumerate(cfg.CLASS_NAMES)}
    bboxes2d_cams = []
    labels2d_cams = []
    centers2d_cams = []
    depths_cams = []
    bboxes_ignore_cams = []
    for camera_name in cfg.CAMERA_NAMES:
        camera_boxes = []
        camera_labels = []
        camera_centers = []
        camera_depths = []
        camera_world_transform = camera_transforms[camera_name]
        intrinsic = intrinsics[camera_name]
        for actor, name in zip(actors, names):
            projection = project_actor_box(
                actor,
                camera_world_transform,
                intrinsic,
                cfg.IMAGE_WIDTH,
                cfg.IMAGE_HEIGHT,
            )
            if projection is None:
                continue
            bbox2d, center2d, depth = projection
            camera_boxes.append(bbox2d)
            camera_labels.append(class_to_index[name])
            camera_centers.append(center2d)
            camera_depths.append(depth)
        bboxes2d_cams.append(
            np.asarray(camera_boxes, dtype=np.float32).reshape(-1, 4)
        )
        labels2d_cams.append(np.asarray(camera_labels, dtype=np.int64))
        centers2d_cams.append(
            np.asarray(camera_centers, dtype=np.float32).reshape(-1, 2)
        )
        depths_cams.append(np.asarray(camera_depths, dtype=np.float32))
        bboxes_ignore_cams.append(np.empty((0, 4), dtype=np.float32))

    count = len(boxes)
    return {
        "gt_boxes": np.asarray(boxes, dtype=np.float32).reshape(-1, 7),
        "gt_names": np.asarray(names),
        "gt_velocity": np.asarray(velocities, dtype=np.float32).reshape(-1, 2),
        # CARLA supplies perfect boxes without LiDAR.  A positive pseudo-count
        # keeps mmdet3d's nuScenes valid-object filter from discarding them.
        "num_lidar_pts": np.ones(count, dtype=np.int64),
        "num_radar_pts": np.zeros(count, dtype=np.int64),
        "valid_flag": np.ones(count, dtype=bool),
        "bboxes2d": bboxes2d_cams,
        "labels2d": labels2d_cams,
        "centers2d": centers2d_cams,
        "depths": depths_cams,
        "bboxes_ignore": bboxes_ignore_cams,
    }


def ego_pose(ego):
    transform = ego.get_transform()
    matrix_unreal = transform_matrix(transform)
    rotation = (
        UNREAL_TO_CANONICAL
        @ matrix_unreal[:3, :3]
        @ UNREAL_TO_CANONICAL
    )
    translation = UNREAL_TO_CANONICAL @ matrix_unreal[:3, 3]
    return matrix_to_quaternion(rotation), translation.astype(np.float32)


def destroy_actors(client, actors):
    for actor in actors:
        if actor is None:
            continue
        try:
            if hasattr(actor, "stop"):
                actor.stop()
        except RuntimeError:
            pass
    actor_ids = [actor.id for actor in reversed(actors) if actor is not None]
    if not actor_ids:
        return
    try:
        # One batch RPC is both faster and safer than waiting for a separate
        # destroy response for every traffic actor when CARLA is unhealthy.
        commands = [
            carla.command.DestroyActor(actor_id) for actor_id in actor_ids
        ]
        client.apply_batch(commands)
    except RuntimeError:
        pass


def save_rgb_jpeg(carla_image, path, quality):
    """Write CARLA BGRA pixels as a space-efficient RGB JPEG."""
    rgba = Image.frombuffer(
        "RGBA",
        (carla_image.width, carla_image.height),
        bytes(carla_image.raw_data),
        "raw",
        "BGRA",
        0,
        1,
    )
    rgba.convert("RGB").save(
        str(path), format="JPEG", quality=quality, subsampling=2
    )


def ensure_free_disk_space(output, minimum_gb):
    free_gb = shutil.disk_usage(str(output)).free / (1024.0 ** 3)
    if free_gb < minimum_gb:
        raise RuntimeError(
            "Free disk space {:.2f} GiB fell below the {:.2f} GiB safety "
            "limit; collection stopped cleanly and can be resumed.".format(
                free_gb, minimum_gb
            )
        )


def collection_metadata(args):
    metadata = {
        "collection_mode": "scenario_matrix" if args.scenario_matrix else "episodes",
        "jpeg_quality": args.jpeg_quality,
        "capture_every": args.capture_every,
        "fps": args.fps,
        "vehicles": args.vehicles,
        "seed": args.seed,
    }
    if args.scenario_matrix:
        metadata.update(
            {
                "weather_presets": list(args.weather_presets),
                "maneuvers": list(args.maneuvers),
                "frames_per_case": args.frames_per_case,
                "clips_per_case": args.clips_per_case,
                "expected_cases": len(args.weather_presets) * len(args.maneuvers),
                "expected_frames": (
                    len(args.weather_presets)
                    * len(args.maneuvers)
                    * args.frames_per_case
                ),
                "ego_motion": "collision_free_virtual_rig",
                "ego_route_speed_mps": cfg.EGO_ROUTE_SPEED_MPS,
                "route_sample_meters": cfg.ROUTE_SAMPLE_METERS,
            }
        )
    return metadata


def write_infos(path, infos, town, extra_metadata=None):
    metadata = {
        "version": "carla-town04-v1.1",
        "town": town,
        "camera_profile": cfg.CAMERA_PROFILE,
        "camera_order": list(cfg.CAMERA_NAMES),
        "camera_transforms": dict(cfg.CAMERA_TRANSFORMS),
        "camera_fovs": dict(cfg.CAMERA_FOVS),
        "image_size": [cfg.IMAGE_WIDTH, cfg.IMAGE_HEIGHT],
    }
    metadata.update(extra_metadata or {})
    payload = {
        "infos": infos,
        "metadata": metadata,
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    temporary.replace(path)


def write_manifest(output, train_infos, val_infos, args):
    all_infos = train_infos + val_infos
    case_counts = {}
    split_case_counts = {"train": {}, "val": {}}
    for split, infos in (("train", train_infos), ("val", val_infos)):
        for info in infos:
            case_token = info.get("case_token", info["scene_token"])
            case_counts[case_token] = case_counts.get(case_token, 0) + 1
            split_counts = split_case_counts[split]
            split_counts[case_token] = split_counts.get(case_token, 0) + 1
    manifest = {
        "metadata": collection_metadata(args),
        "total_frames": len(all_infos),
        "train_frames": len(train_infos),
        "val_frames": len(val_infos),
        "total_camera_images": len(all_infos) * len(cfg.CAMERA_NAMES),
        "case_counts": case_counts,
        "split_case_counts": split_case_counts,
    }
    path = output / "dataset_manifest.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def read_infos(path, expected_metadata=None):
    if not path.is_file():
        return []
    with path.open("rb") as stream:
        payload = pickle.load(stream)
    profile = payload.get("metadata", {}).get("camera_profile")
    if profile != cfg.CAMERA_PROFILE:
        raise ValueError(
            "{} uses camera profile {!r}, expected {!r}; refusing to mix "
            "calibrations during --resume".format(
                path, profile, cfg.CAMERA_PROFILE
            )
        )
    metadata = payload.get("metadata", {})
    for key, expected in (expected_metadata or {}).items():
        actual = metadata.get(key)
        if actual != expected:
            raise ValueError(
                "{} uses metadata {}={!r}, expected {!r}; refusing to mix "
                "different collection plans during --resume".format(
                    path, key, actual, expected
                )
            )
    infos = payload.get("infos", [])
    if not isinstance(infos, list):
        raise ValueError("{} does not contain an infos list".format(path))
    return infos


def validate_output_location(output, resume):
    output = output.resolve()
    if output.exists() and any(output.iterdir()) and not resume:
        raise SystemExit(
            "Output {} is not empty. Use --resume or choose another path.".format(output)
        )
    output.mkdir(parents=True, exist_ok=True)
    return output


def slug(value):
    return "".join(character.lower() for character in value if character.isalnum())


def build_scene_specs(args):
    if not args.scenario_matrix:
        val_scenes = 0
        if args.episodes > 1 and args.val_ratio > 0.0:
            val_scenes = max(1, int(round(args.episodes * args.val_ratio)))
        train_scenes = args.episodes - val_scenes
        return [
            {
                "scene_token": "{}-scene-{:04d}".format(args.town.lower(), index),
                "case_token": "{}-default".format(args.town.lower()),
                "frames": args.frames_per_episode,
                "case_frame_offset": index * args.frames_per_episode,
                "weather": None,
                "maneuver": None,
                "clip_index": index,
                "route_slot": index,
                "split": "train" if index < train_scenes else "val",
                "seed": args.seed + index,
            }
            for index in range(args.episodes)
        ]

    frames_per_clip = args.frames_per_case // args.clips_per_case
    val_clips = 0
    if args.clips_per_case > 1 and args.val_ratio > 0.0:
        val_clips = max(1, int(round(args.clips_per_case * args.val_ratio)))
    train_clips = args.clips_per_case - val_clips
    specs = []
    case_index = 0
    for weather_index, weather in enumerate(args.weather_presets):
        if not hasattr(carla.WeatherParameters, weather):
            raise ValueError("Unknown CARLA weather preset: {}".format(weather))
        for maneuver in args.maneuvers:
            case_token = "{}-{}-{}".format(
                args.town.lower(), slug(weather), maneuver
            )
            for clip_index in range(args.clips_per_case):
                specs.append(
                    {
                        "scene_token": "{}-clip-{:02d}".format(
                            case_token, clip_index
                        ),
                        "case_token": case_token,
                        "frames": frames_per_clip,
                        "case_frame_offset": clip_index * frames_per_clip,
                        "weather": weather,
                        "maneuver": maneuver,
                        "clip_index": clip_index,
                        "route_slot": (
                            weather_index * args.clips_per_case + clip_index
                        ),
                        "split": "train" if clip_index < train_clips else "val",
                        "seed": args.seed + case_index * 100 + clip_index,
                    }
                )
            case_index += 1
    return specs


def collect_scene(
    client, world, traffic_manager, intrinsics, output, placeholder, args, spec,
    scene_number, scene_total
):
    rng = random.Random(spec["seed"])
    random.seed(spec["seed"])
    np.random.seed(spec["seed"])
    traffic_manager.set_random_device_seed(spec["seed"])
    if spec["weather"] is not None:
        world.set_weather(getattr(carla.WeatherParameters, spec["weather"]))

    ego_route = spec.get("ego_route")
    print(
        "\n[scene {}/{}] {} | weather={} maneuver={} split={} seed={} "
        "junction={}".format(
            scene_number,
            scene_total,
            spec["scene_token"],
            spec["weather"] or "current",
            spec["maneuver"] or "autopilot",
            spec["split"],
            spec["seed"],
            ego_route["junction_id"] if ego_route else "autopilot",
        ),
        flush=True,
    )
    scene_actors = []
    scene_infos = []
    try:
        ego, traffic = spawn_episode_actors(
            world,
            traffic_manager,
            args.vehicles,
            rng,
            maneuver=spec["maneuver"],
            ego_route=ego_route,
        )
        cameras, camera_queues = spawn_cameras(world, ego)
        scene_actors.extend(traffic)
        if not getattr(ego, "is_virtual", False):
            scene_actors.append(ego)
        scene_actors.extend(cameras.values())

        for _ in range(args.warmup_frames):
            frame = world.tick()
            for sensor_queue in camera_queues.values():
                get_matching_sensor_frame(sensor_queue, frame)

        distances = sorted(
            round(actor.get_location().distance(ego.get_location()), 1)
            for actor in traffic
            if actor.is_alive
        )
        print(
            "After warmup, nearest traffic distances: {}".format(distances[:10]),
            flush=True,
        )

        previous_token = ""
        for step in range(spec["frames"] * args.capture_every):
            route_progress = None
            if ego_route is not None:
                route_progress = step * cfg.EGO_ROUTE_SPEED_MPS / args.fps
                trajectory_index = min(
                    int(round(route_progress / cfg.ROUTE_SAMPLE_METERS)),
                    len(ego_route["trajectory"]) - 1,
                )
                ego.set_transform(ego_route["trajectory"][trajectory_index])
                update_virtual_cameras(client, cameras, ego)
            frame = world.tick()
            images = {
                name: get_matching_sensor_frame(camera_queues[name], frame)
                for name in cfg.CAMERA_NAMES
            }
            if step % args.capture_every:
                continue

            frame_index = len(scene_infos)
            if frame_index % 10 == 0:
                ensure_free_disk_space(output, args.min_free_disk_gb)
            token = "{}-{:06d}".format(spec["scene_token"], frame_index)
            timestamp = int(images["CAM_FRONT"].timestamp * 1e6)
            camera_transforms = {
                name: images[name].transform for name in cfg.CAMERA_NAMES
            }
            cams = {}
            for camera_name in cfg.CAMERA_NAMES:
                relative_path = Path("samples") / camera_name / (token + ".jpg")
                absolute_path = output / relative_path
                save_rgb_jpeg(
                    images[camera_name], absolute_path, args.jpeg_quality
                )
                calibration = camera_calibration(
                    camera_name,
                    camera_transforms[camera_name],
                    ego,
                    intrinsics[camera_name],
                )
                calibration.update(
                    {
                        "data_path": str(Path(args.output) / relative_path),
                        "timestamp": timestamp,
                    }
                )
                cams[camera_name] = calibration

            pose_rotation, pose_translation = ego_pose(ego)
            info = {
                "lidar_path": str(Path(args.output) / placeholder.name),
                "token": token,
                "sweeps": [] if frame_index == 0 else [
                    {
                        "data_path": str(Path(args.output) / placeholder.name),
                        "sensor2lidar_rotation": np.eye(3, dtype=np.float32),
                        "sensor2lidar_translation": np.zeros(3, dtype=np.float32),
                        "timestamp": scene_infos[-1]["timestamp"],
                    }
                ],
                "cams": cams,
                "lidar2ego_translation": np.zeros(3, dtype=np.float32),
                "lidar2ego_rotation": [1.0, 0.0, 0.0, 0.0],
                "ego2global_translation": pose_translation,
                "ego2global_rotation": pose_rotation,
                "timestamp": timestamp,
                "prev": previous_token,
                "next": "",
                "scene_token": spec["scene_token"],
                "frame_idx": frame_index,
                "case_token": spec["case_token"],
                "case_frame_idx": spec["case_frame_offset"] + frame_index,
                "weather": spec["weather"],
                "maneuver": spec["maneuver"],
                "clip_index": spec["clip_index"],
                "junction_id": (
                    ego_route["junction_id"] if ego_route else None
                ),
                "route_entry": ego_route["entry"] if ego_route else None,
                "route_exit": ego_route["exit"] if ego_route else None,
                "route_turn_angle": (
                    ego_route["turn_angle"] if ego_route else None
                ),
                "route_progress_m": route_progress,
            }
            info.update(
                collect_ground_truth(world, ego, camera_transforms, intrinsics)
            )
            if scene_infos:
                scene_infos[-1]["next"] = token
            scene_infos.append(info)
            previous_token = token
            if frame_index == 0 or (frame_index + 1) % 10 == 0:
                print(
                    "  frame {}/{}: {} objects".format(
                        frame_index + 1,
                        spec["frames"],
                        len(info["gt_names"]),
                    ),
                    flush=True,
                )
    finally:
        destroy_actors(client, scene_actors)
        for _ in range(2):
            world.tick()
    if len(scene_infos) != spec["frames"]:
        raise RuntimeError(
            "Scene {} produced {} of {} frames".format(
                spec["scene_token"], len(scene_infos), spec["frames"]
            )
        )
    return scene_infos


def collect(args):
    output = validate_output_location(Path(args.output), args.resume)
    ensure_free_disk_space(output, args.min_free_disk_gb)
    samples_root = output / "samples"
    for camera_name in cfg.CAMERA_NAMES:
        (samples_root / camera_name).mkdir(parents=True, exist_ok=True)
    placeholder = output / "lidar_placeholder.bin"
    placeholder.touch(exist_ok=True)
    specs = build_scene_specs(args)
    print(
        "Collection plan: {} scenes, {} cases, {} frames, {} camera images".format(
            len(specs),
            len(set(spec["case_token"] for spec in specs)),
            sum(spec["frames"] for spec in specs),
            sum(spec["frames"] for spec in specs) * len(cfg.CAMERA_NAMES),
        ),
        flush=True,
    )

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    if args.no_reload_world:
        world = client.get_world()
    else:
        world = client.load_world(args.town)
    loaded_town = world.get_map().name.rsplit("/", 1)[-1]
    if loaded_town != args.town:
        print("WARNING: requested {}, CARLA loaded {}".format(args.town, loaded_town))
    if args.scenario_matrix:
        route_families = build_maneuver_route_families(world.get_map())
        discovered_route_count = len(route_families)
        route_families = filter_route_families_for_capture(
            route_families, args
        )
        for spec in specs:
            family = route_families[spec["route_slot"] % len(route_families)]
            spec["ego_route"] = family[spec["maneuver"]]
        print(
            "Explicit route families: {}/{} junction approaches complete all "
            "maneuvers inside one clip".format(
                len(route_families), discovered_route_count
            ),
            flush=True,
        )

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0 / args.fps
    settings.no_rendering_mode = False
    world.apply_settings(settings)
    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_synchronous_mode(True)

    intrinsics = {
        name: camera_intrinsic(
            cfg.IMAGE_WIDTH, cfg.IMAGE_HEIGHT, cfg.CAMERA_FOVS[name]
        )
        for name in cfg.CAMERA_NAMES
    }
    train_info_path = output / "carla_town04_temporal_infos_train.pkl"
    val_info_path = output / "carla_town04_temporal_infos_val.pkl"
    metadata = collection_metadata(args)
    train_infos = (
        read_infos(train_info_path, metadata) if args.resume else []
    )
    val_infos = read_infos(val_info_path, metadata) if args.resume else []
    completed_scenes = {
        info["scene_token"] for info in train_infos + val_infos
    }
    newly_collected_scenes = 0
    try:
        for scene_index, spec in enumerate(specs):
            if spec["scene_token"] in completed_scenes:
                print(
                    "[scene {}/{}] {} already complete; skipping".format(
                        scene_index + 1, len(specs), spec["scene_token"]
                    )
                )
                continue
            scene_infos = collect_scene(
                client,
                world,
                traffic_manager,
                intrinsics,
                output,
                placeholder,
                args,
                spec,
                scene_index + 1,
                len(specs),
            )
            if spec["split"] == "train":
                train_infos.extend(scene_infos)
            else:
                val_infos.extend(scene_infos)
            write_infos(train_info_path, train_infos, args.town, metadata)
            write_infos(val_info_path, val_infos, args.town, metadata)
            write_manifest(output, train_infos, val_infos, args)
            newly_collected_scenes += 1
            if (
                args.max_scenes_per_run
                and newly_collected_scenes >= args.max_scenes_per_run
            ):
                print(
                    "Reached --max-scenes-per-run={}; stopping cleanly for "
                    "a CARLA refresh.".format(args.max_scenes_per_run),
                    flush=True,
                )
                break
    finally:
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)

    print("\nDataset ready at {}".format(output))
    print("  train frames : {}".format(len(train_infos)))
    print("  val frames   : {}".format(len(val_infos)))
    print("  total frames : {}".format(len(train_infos) + len(val_infos)))
    print(
        "  camera images: {}".format(
            (len(train_infos) + len(val_infos)) * len(cfg.CAMERA_NAMES)
        )
    )


def main():
    args = parse_args()
    try:
        collect(args)
    except KeyboardInterrupt:
        print("\nInterrupted; completed episodes were checkpointed.", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
