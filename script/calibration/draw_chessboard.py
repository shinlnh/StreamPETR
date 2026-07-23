import carla
import signal
import sys
import time
import argparse

# Define command-line arguments
parser = argparse.ArgumentParser(description="Draw a chessboard for camera calibration, fixed 3 meters south of the dash camera's initial position in CARLA.")
parser.add_argument('--board_width', type=float, default=4.0, help='Width of the chessboard in meters')
parser.add_argument('--board_height', type=float, default=5.5, help='Height of the chessboard in meters')
parser.add_argument('--rows', type=int, default=8, help='Number of rows in the chessboard')
parser.add_argument('--columns', type=int, default=8, help='Number of columns in the chessboard')
parser.add_argument('--checker_width', type=float, default=0.05, help='Width of each checker square in meters')
args = parser.parse_args()

# Predefined colors (R, G, B, A)
black_color = carla.Color(0, 0, 0, 255)
light_gray_color = carla.Color(1, 1, 1, 255)

# Global variables
client = carla.Client('127.0.0.1', 2000)
client.get_world().wait_for_tick()
client.set_timeout(2.0)
world = client.get_world()
debug_helper = world.debug

# Get the vehicle and its initial position (center of gravity)
vehicle = next((actor for actor in world.get_actors() if 'vehicle' in actor.type_id), None)
if vehicle is None:
    print("No vehicle found in the world. Please spawn one in the simulator.")
    sys.exit(1)

# Get the vehicle's initial location
initial_location = vehicle.get_transform().location

# Camera offset relative to the vehicle's center of gravity
camera_offset = carla.Vector3D(x=2.2, y=0.0, z=1.2)

# Calculate the global position of the camera
camera_position = carla.Location(
    x=initial_location.x + camera_offset.x,
    y=initial_location.y + camera_offset.y,
    z=initial_location.z + camera_offset.z
)

# Calculate the fixed chessboard position (3 meters south of the camera's initial position)
fixed_x = camera_position.x
fixed_y = camera_position.y  # 
fixed_z = camera_position.z + 3  # Align the chessboard's base with the camera's z-height

# Function to handle Ctrl+C gracefully
def signal_handler(sig, frame):
    print('Exiting...')
    debug_helper.clear()
    sys.exit(0)

# Function to draw a chessboard at a fixed position facing south
def draw_chessboard():
    board_width = args.board_width
    board_height = args.board_height
    rows = args.rows
    columns = args.columns
    checker_width = args.checker_width

    start_y = -board_width / 2.0
    start_z = -board_height / 2.0

    for row in range(rows):
        for col in range(columns):
            color = light_gray_color if (row + col) % 2 == 0 else black_color
            y1 = start_y + col * checker_width
            z1 = start_z + row * checker_width
            y2 = y1 + checker_width
            z2 = z1 + checker_width

            corners = [
                carla.Vector3D(x=fixed_x, y=y1 + fixed_y, z=z1 + fixed_z),
                carla.Vector3D(x=fixed_x, y=y2 + fixed_y, z=z1 + fixed_z),
                carla.Vector3D(x=fixed_x, y=y2 + fixed_y, z=z2 + fixed_z),
                carla.Vector3D(x=fixed_x, y=y1 + fixed_y, z=z2 + fixed_z)
            ]

            # Draw filled square with additional lines
            num_lines = 2
            for i in range(num_lines + 1):
                t = i / num_lines
                start = carla.Vector3D(
                    x=corners[0].x,
                    y=corners[0].y + t * (corners[1].y - corners[0].y),
                    z=corners[0].z
                )
                end = carla.Vector3D(
                    x=corners[3].x,
                    y=corners[3].y + t * (corners[2].y - corners[3].y),
                    z=corners[3].z
                )
                debug_helper.draw_line(
                    start,
                    end,
                    thickness=0.025,
                    color=color,
                    life_time=0.1
                )

            # Draw outline for clarity
            for i in range(4):
                debug_helper.draw_line(
                    corners[i],
                    corners[(i + 1) % 4],
                    thickness=0.15,
                    color=color,
                    life_time=0.03
                )

# Register signal handler for Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

# Print out the requested parameters
print(f"Board width: {args.board_width} m")
print(f"Board height: {args.board_height} m")
print(f"Rows: {args.rows}")
print(f"Columns: {args.columns}")
print(f"Checker width: {args.checker_width} m")

try:
    while True:
        draw_chessboard()
        # time.sleep(0.01)

except KeyboardInterrupt:
    print('Ctrl+C pressed. Exiting...')
    sys.exit(0)
