import carla
import math
import signal
import sys
import time
import argparse

# Define command-line arguments
parser = argparse.ArgumentParser(description="Draw a rectangle in front of a vehicle in CARLA simulator in METER.")
parser.add_argument('--distance', type=float, default=2.0, help='Distance from the vehicle to the rectangle in meter')
parser.add_argument('--width', type=float, default=5.0, help='Width of the rectangle in meter')
parser.add_argument('--length', type=float, default=60.0, help='Length of the rectangle in meter')
parser.add_argument('--debug', action="store_true", help='For topdown debug, enable to draw 35x35m rectangle')
args = parser.parse_args()

# Pre defined color (R, G, B, A)
draw_color = carla.Color(10, 0, 0, 0)
# Global variables
client = carla.Client('127.0.0.1', 2000)
client.get_world().wait_for_tick()
client.set_timeout(2.0)
world = client.get_world()
debug_helper = world.debug

# Get the hero1 vehicle (assuming it's already initialized)
print(world.get_actors())
vehicle = next((actor for actor in world.get_actors() if 'vehicle' in actor.type_id), None)
if vehicle is None:
    print("No vehicle found in the world. Please spawn one in the simulator.")
    sys.exit(1)
print(vehicle)

# Function to handle Ctrl+C gracefully
def signal_handler(sig, frame):
    print('Exiting...')
    # Clear all drawings when exiting
    debug_helper.clear()
    sys.exit(0)

# Function to draw a rectangle directly in front of the vehicle
def draw_rectangle_in_front(vehicle_location, vehicle_rotation, vehicle_boundingbox):
    # Define rectangle dimensions (width and length) and its distance from the car
    # Default value are:  
    # distance = 5.0 m 
    # width = 3.0 m
    # length = 5.0 m

    # Use command-line arguments for rectangle dimensions
    if args.debug:
        distance = 0
        width = 35
        length = 35
        half_length = 0
        z = 0.5
    else:
        distance = args.distance
        width = args.width
        length = args.length
        half_length = abs(vehicle_boundingbox.extent.x)
        z = 0.0

    # Calculate the corners of the rectangle based on vehicle's orientation (measured in radians)
    yaw = math.radians(vehicle_rotation.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Calculate the corners of the rectangle
    corners = []
    half_width = width / 2.0

    # Calculate corners relative to the vehicle's location and rotation
    front_left = carla.Vector3D(y=-half_width, x=half_length + distance + length, z=z)
    front_right = carla.Vector3D(y=half_width, x=half_length + distance + length, z=z)
    rear_right = carla.Vector3D(y=half_width, x=half_length + distance, z=z)
    rear_left = carla.Vector3D(y=-half_width, x=half_length + distance, z=z)

    # Rotate corners based on vehicle's yaw angle
    corners.append(vehicle_location + carla.Vector3D(x=front_left.x * cos_yaw - front_left.y * sin_yaw,
                                                    y=front_left.x * sin_yaw + front_left.y * cos_yaw,
                                                    z=front_left.z))
    corners.append(vehicle_location + carla.Vector3D(x=front_right.x * cos_yaw - front_right.y * sin_yaw,
                                                    y=front_right.x * sin_yaw + front_right.y * cos_yaw,
                                                    z=front_right.z))
    corners.append(vehicle_location + carla.Vector3D(x=rear_right.x * cos_yaw - rear_right.y * sin_yaw,
                                                    y=rear_right.x * sin_yaw + rear_right.y * cos_yaw,
                                                    z=rear_right.z))
    corners.append(vehicle_location + carla.Vector3D(x=rear_left.x * cos_yaw - rear_left.y * sin_yaw,
                                                    y=rear_left.x * sin_yaw + rear_left.y * cos_yaw,
                                                    z=rear_left.z))

    # Draw lines to form the rectangle
    debug_helper.draw_line(corners[0], corners[1], thickness=0.2, color= draw_color, life_time=0.1)  
    debug_helper.draw_line(corners[1], corners[2], thickness=0.1, color= draw_color, life_time=0.1)  
    debug_helper.draw_line(corners[2], corners[3], thickness=0.1, color= draw_color, life_time=0.1)  
    debug_helper.draw_line(corners[3], corners[0], thickness=0.1, color= draw_color, life_time=0.1)  

# Register signal handler for Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

bounding_box = vehicle.bounding_box
car_width = bounding_box.extent.y * 2  # The extent.y is half the width

# Print out the requested dimensions
print(f"car_width: {car_width}")
print(f"carpet_width: {args.width}")
print(f"car_to_carpet_distance: {args.distance}")
print(f"carpet_length: {args.length}")

try:
    while True:
        # Get vehicle transform (location and rotation)
        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation
        bounding_box = vehicle.bounding_box
        # Draw rectangle directly in front of the vehicle
        draw_rectangle_in_front(vehicle_location, vehicle_rotation, bounding_box)

        # Sleep for a short time to control loop frequency
        time.sleep(0.01)

except KeyboardInterrupt:
    print('Ctrl+C pressed. Exiting...')
    sys.exit(0)
