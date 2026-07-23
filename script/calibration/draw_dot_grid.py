import carla
import time

# Grid and plane parameters
GRID_WIDTH = 11             # Number of dots along the x-axis
GRID_HEIGHT = 9             # Number of dots along the y-axis
DOT_RADIUS = 0.3            # Radius of each dot (meters)
DOT_SPACING = 0.3           # Distance between dots (meters)

PLANE_WIDTH = (GRID_WIDTH - 1) * DOT_SPACING
PLANE_HEIGHT = (GRID_HEIGHT - 1) * DOT_SPACING
DOT_GRID_DISTANCE = 3.8    # Distance in front of the vehicle to draw grid
GRIDDED_DOT_Z_LOCATION = 1.42

def draw_dot_grid(world, vehicle_tf, duration=0.0):
    """
    Draws a grid of dots in front of the vehicle.
    """
    center = vehicle_tf.location + vehicle_tf.get_forward_vector() * DOT_GRID_DISTANCE + vehicle_tf.get_up_vector() * GRIDDED_DOT_Z_LOCATION

    # Top-left corner of the grid
    top_left = center - vehicle_tf.get_right_vector() * (PLANE_WIDTH / 2) + vehicle_tf.get_up_vector() * PLANE_HEIGHT

    for i in range(GRID_HEIGHT):
        for j in range(GRID_WIDTH):
            x_offset = j * DOT_SPACING
            y_offset = -i * DOT_SPACING
            dot_location = top_left \
                + vehicle_tf.get_right_vector() * x_offset \
                + vehicle_tf.get_up_vector() * y_offset

            world.debug.draw_point(
                dot_location,
                color=carla.Color(r=1, g=1, b=1),  # White dots
                size=DOT_RADIUS,
                life_time=duration
            )

def draw_black_background(world, vehicle_tf):
    """
    Draws a black background plane behind the dot grid.
    """
    forward_offset = (DOT_GRID_DISTANCE + 0.5) 
    vertical_offset = 0.5 + PLANE_HEIGHT / 2

    center = vehicle_tf.location \
        + vehicle_tf.get_forward_vector() * forward_offset \
        + vehicle_tf.get_up_vector() * vertical_offset

    bounding_box = carla.BoundingBox(
        center,
        carla.Vector3D(0.1, PLANE_WIDTH , PLANE_HEIGHT)
    )

    world.debug.draw_box(
        bounding_box,
        rotation=vehicle_tf.rotation,
        thickness=PLANE_HEIGHT * 3,
        color=carla.Color(r=0, g=0, b=0, a=255),
        life_time=0.0,
        persistent_lines=True
    )

def find_first_vehicle(world):
    """
    Returns the first available vehicle in the world.
    """
    while True:
        vehicles = world.get_actors().filter('vehicle.*')
        if vehicles:
            print("Found vehicle:", vehicles[0])
            return vehicles[0]
        print("Waiting for vehicle...")
        time.sleep(1)

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    vehicle = find_first_vehicle(world)
    vehicle_tf = vehicle.get_transform()

    draw_black_background(world, vehicle_tf)
    draw_dot_grid(world, vehicle_tf)

if __name__ == '__main__':
    main()
