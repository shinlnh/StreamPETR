
# ==============================================================================
# -- CARLA Variables ----------------------------------------------------------
# ==============================================================================

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
FPS = 20
FOV = 90.0
CARLA_TOWN = 'Town04'

# Turn rendering on/off
isRender = True
# Keep own vehicle either in center of road or oscillate between lanemarkings
isCenter = True
# Draw all lanes in carla simulator
draw3DLanes = False
# Calculate and draw 3D Lanes on Juction
junctionMode = True
# Third-person view for the ego vehicle
isThirdPerson = True

# Number of images stored in a .npy file
number_of_images = 100
# Max number of .npy files
total_number_of_imagesets = 1000
# Vertical startposition of the lanepoints in the 2D-image
row_anchor_start = 0
# Number of anchor points
num_row_anchor = 72
# Number of images to collect before respawning agent
images_until_respawn = 0
# Distance between the calculated lanepoints
meters_per_frame = 1.0
# Number of points to sample per lane marking
number_of_lanepoints = 80
# Y value of points to sample the anchor
h_samples = list(range(row_anchor_start, IMAGE_HEIGHT, int((IMAGE_HEIGHT - row_anchor_start) / (num_row_anchor - 1))))[-num_row_anchor:]


# ==============================================================================
# -- Saver Variables ----------------------------------------------------------
# ==============================================================================

# Output for .npy files
output_directory = 'data/rawimages/'+ CARLA_TOWN + '/'
# Loading directory of .npy files
loading_directory = output_directory
# Path to the image and label files
saving_directory = 'data/dataset/' + CARLA_TOWN + '/'
visualize_directory = 'data/visualized/' + CARLA_TOWN + '/'
label_raw = saving_directory + 'label_raw.json'
# Split setting [train, val, test]
label_split = True
split_ratio = {'train': 0.7, 'val': 0.1, 'test': 0.2}
# Max size of images per folder
max_files_per_classification = 30000
