"""Default sensor and collection settings for the CARLA Town04 dataset."""

DEFAULT_CAMERA_PROFILE = "nuscenes-reference-v1"

NUSCENES_CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)

# CARLA uses x-forward, y-right, z-up and degrees for rotations. These are the
# six-camera nuScenes reference poses converted from a representative nuScenes
# calibrated_sensor record into CARLA's left-handed vehicle frame. nuScenes
# calibrates each capture vehicle separately, so these values are a canonical
# reference rather than a universal calibration shared by every nuScenes log.
NUSCENES_CAMERA_TRANSFORMS = {
    # x, y, z, pitch, yaw, roll
    "CAM_FRONT": (1.700791, -0.015946, 1.510958, -0.323227, -0.325455, -0.046129),
    "CAM_FRONT_RIGHT": (1.550848, 0.493405, 1.495748, -0.781991, 56.397315, 0.518892),
    "CAM_FRONT_LEFT": (1.523878, -0.494631, 1.509328, 0.140225, -55.160667, 0.121436),
    "CAM_BACK": (0.028326, -0.003451, 1.579103, 0.959396, -179.857406, 0.229229),
    "CAM_BACK_LEFT": (1.035691, -0.484795, 1.590970, -0.917357, -108.596801, -0.215210),
    "CAM_BACK_RIGHT": (1.014878, 0.480568, 1.562395, -0.932012, 110.789212, 0.619177),
}

NUSCENES_CAMERA_FOVS = {
    "CAM_FRONT": 70.0,
    "CAM_FRONT_RIGHT": 70.0,
    "CAM_FRONT_LEFT": 70.0,
    "CAM_BACK": 110.0,
    "CAM_BACK_LEFT": 70.0,
    "CAM_BACK_RIGHT": 70.0,
}

# The seventh long-range view is deliberately co-located with CAM_FRONT. This
# makes its calibration compatible with a center-cropped 30-degree view derived
# from an existing CAM_FRONT image, while a fresh CARLA collection still uses a
# genuine independent RGB sensor at the same pose.
FRONT_NARROW_CAMERA = "CAM_FRONT_NARROW"
SEVEN_CAMERA_NAMES = (
    "CAM_FRONT",
    FRONT_NARROW_CAMERA,
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)
SEVEN_CAMERA_TRANSFORMS = dict(NUSCENES_CAMERA_TRANSFORMS)
SEVEN_CAMERA_TRANSFORMS[FRONT_NARROW_CAMERA] = NUSCENES_CAMERA_TRANSFORMS[
    "CAM_FRONT"
]
SEVEN_CAMERA_FOVS = dict(NUSCENES_CAMERA_FOVS)
SEVEN_CAMERA_FOVS[FRONT_NARROW_CAMERA] = 30.0

CAMERA_PROFILES = {
    DEFAULT_CAMERA_PROFILE: {
        "output_root": "data/carla_town04_nuscenes_rig",
        "camera_names": NUSCENES_CAMERA_NAMES,
        "camera_transforms": NUSCENES_CAMERA_TRANSFORMS,
        "camera_fovs": NUSCENES_CAMERA_FOVS,
        "require_instance_visibility": False,
    },
    "nuscenes-reference-7cam-front-narrow-fov30-v1": {
        "output_root": "data/carla_town04_7cam_fov30",
        "camera_names": SEVEN_CAMERA_NAMES,
        "camera_transforms": SEVEN_CAMERA_TRANSFORMS,
        "camera_fovs": SEVEN_CAMERA_FOVS,
        "require_instance_visibility": True,
    },
}


def activate_camera_profile(profile_name):
    """Select a camera rig without changing the six-camera default."""
    if profile_name not in CAMERA_PROFILES:
        raise ValueError("Unknown camera profile: {}".format(profile_name))
    profile = CAMERA_PROFILES[profile_name]
    global CAMERA_PROFILE, OUTPUT_ROOT, CAMERA_NAMES
    global CAMERA_TRANSFORMS, CAMERA_FOVS, REQUIRE_INSTANCE_VISIBILITY
    CAMERA_PROFILE = profile_name
    OUTPUT_ROOT = profile["output_root"]
    CAMERA_NAMES = tuple(profile["camera_names"])
    CAMERA_TRANSFORMS = dict(profile["camera_transforms"])
    CAMERA_FOVS = dict(profile["camera_fovs"])
    REQUIRE_INSTANCE_VISIBILITY = bool(profile["require_instance_visibility"])


activate_camera_profile(DEFAULT_CAMERA_PROFILE)

IMAGE_WIDTH = 1600
IMAGE_HEIGHT = 900
INSTANCE_IMAGE_WIDTH = 480
INSTANCE_IMAGE_HEIGHT = 270

# Conservative z-buffer visibility thresholds for clean 2D/3D supervision.
# Values match the already validated clean CARLA collector in this repository.
VISIBILITY_FRAME_MARGIN_PX = 4.0
VISIBILITY_CENTER_PATCH_RADIUS_PX = 2
VISIBILITY_MIN_RGB_BBOX_WIDTH_PX = {
    "default": 12.0,
    "pedestrian": 7.0,
    "bicycle": 8.0,
    "motorcycle": 8.0,
}
VISIBILITY_MIN_RGB_BBOX_HEIGHT_PX = {
    "default": 12.0,
    "pedestrian": 18.0,
    "bicycle": 14.0,
    "motorcycle": 14.0,
}
VISIBILITY_MIN_ACTOR_PIXELS = {
    "default": 45,
    "pedestrian": 16,
    "bicycle": 16,
    "motorcycle": 18,
}
VISIBILITY_MIN_WIDTH_COVERAGE = {
    "default": 0.55,
    "pedestrian": 0.45,
    "bicycle": 0.40,
    "motorcycle": 0.45,
}
VISIBILITY_MIN_HEIGHT_COVERAGE = {
    "default": 0.60,
    "pedestrian": 0.60,
    "bicycle": 0.50,
    "motorcycle": 0.50,
}
VISIBILITY_MIN_AREA_RATIO = {
    "default": 0.10,
    "pedestrian": 0.06,
    "bicycle": 0.035,
    "motorcycle": 0.05,
}
FPS = 10

# Balanced collection matrix used by scripts/collect_town04_matrix.sh.
# Nine presets cover clear/cloudy/wet/rain plus noon/sunset/night lighting.
WEATHER_PRESETS = (
    "ClearNoon",
    "CloudyNoon",
    "WetNoon",
    "WetCloudyNoon",
    "SoftRainNoon",
    "MidRainyNoon",
    "HardRainNoon",
    "ClearSunset",
    "ClearNight",
)
MANEUVERS = ("left", "right", "straight")
FRAMES_PER_CASE = 300
CLIPS_PER_CASE = 15
JPEG_QUALITY = 82
MIN_FREE_DISK_GB = 5.0
ROUTE_COMMAND_REPETITIONS = 64
EGO_ROUTE_SPEED_MPS = 5.0
ROUTE_SAMPLE_METERS = 0.5

TOWN = "Town04_Opt"
TRAFFIC_MANAGER_PORT = 8000
EPISODES = 10
FRAMES_PER_EPISODE = 40
WARMUP_FRAMES = 20
NUM_VEHICLES = 60
CAPTURE_EVERY = 5
VAL_RATIO = 0.2
MAX_ANNOTATION_DISTANCE = 55.0
SEED = 42

CLASS_NAMES = (
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
