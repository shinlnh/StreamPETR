"""Default sensor and collection settings for the CARLA Town04 dataset."""

CAMERA_PROFILE = "nuscenes-reference-v1"
OUTPUT_ROOT = "data/carla_town04_nuscenes_rig"

CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
)

# CARLA uses x-forward, y-right, z-up and degrees for rotations. This is the
# six-camera nuScenes reference rig converted from a representative nuScenes
# calibrated_sensor record into CARLA's left-handed vehicle frame. nuScenes
# calibrates each capture vehicle separately, so these values are a canonical
# reference rather than a universal calibration shared by every nuScenes log.
CAMERA_TRANSFORMS = {
    # x, y, z, pitch, yaw, roll
    "CAM_FRONT": (1.700791, -0.015946, 1.510958, -0.323227, -0.325455, -0.046129),
    "CAM_FRONT_RIGHT": (1.550848, 0.493405, 1.495748, -0.781991, 56.397315, 0.518892),
    "CAM_FRONT_LEFT": (1.523878, -0.494631, 1.509328, 0.140225, -55.160667, 0.121436),
    "CAM_BACK": (0.028326, -0.003451, 1.579103, 0.959396, -179.857406, 0.229229),
    "CAM_BACK_LEFT": (1.035691, -0.484795, 1.590970, -0.917357, -108.596801, -0.215210),
    "CAM_BACK_RIGHT": (1.014878, 0.480568, 1.562395, -0.932012, 110.789212, 0.619177),
}

IMAGE_WIDTH = 1600
IMAGE_HEIGHT = 900
CAMERA_FOVS = {
    "CAM_FRONT": 70.0,
    "CAM_FRONT_RIGHT": 70.0,
    "CAM_FRONT_LEFT": 70.0,
    "CAM_BACK": 110.0,
    "CAM_BACK_LEFT": 70.0,
    "CAM_BACK_RIGHT": 70.0,
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
