#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from carla_msgs_custom.msg import VehicleData, VehicleDataArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import carla
import math
import numpy as np
import signal
import sys

def ros_timestamp(sec=0, nsec=0, from_sec=False):
    if from_sec:
        seconds = int(sec)
        nanoseconds = int((sec - seconds) * 1e9)
        return Time(sec=seconds, nanosec=nanoseconds)
    return Time(sec=int(sec), nanosec=int(nsec))

def change_coordinate(my_vehicle):
    # get current direction of my vehicle and set it to the reference direction
    vehicle_transform = my_vehicle.get_transform()

    # Get the car locally x, y and z vectors: 
    forward_vector = vehicle_transform.get_forward_vector() # x locally
    right_vector = vehicle_transform.get_right_vector()     # y locally 
    up_vector = vehicle_transform.get_up_vector()           # z locally

    # Align those into a matrix and inverse to find the base transformation matrix: 
    ref_direction = np.array([[right_vector.x, forward_vector.x, up_vector.x],
                             [right_vector.y, forward_vector.y, up_vector.y],
                             [right_vector.z, forward_vector.z, up_vector.z]])

    # calculate change basis matrix in global coordinate
    # since from global -> local, and the local in global coor, we will mul for invert matrix
    change_basis_matrix = np.linalg.inv(ref_direction)

    # set first position of my vehical to reference point
    ref_point = np.array([[vehicle_transform.location.x],
                         [vehicle_transform.location.y],
                         [vehicle_transform.location.z]])
    return change_basis_matrix, ref_point

def get_bbox_corners_in_global(my_vehicle):
    # Step 1: Get the vehicle's transform (position and rotation)
    vehicle_transform = my_vehicle.get_transform()

    # Step 2: Extract the forward, right, and up vectors of the vehicle
    forward_vector = vehicle_transform.get_forward_vector()  # Local x
    right_vector = vehicle_transform.get_right_vector()      # Local y
    up_vector = vehicle_transform.get_up_vector()            # Local z (not needed here)
    
    # Step 3: Get the bounding box of the vehicle (half extents in each direction)
    bbox = my_vehicle.bounding_box
    extent_x = bbox.extent.x  # Half-length along the x-axis
    extent_y = bbox.extent.y  # Half-width along the y-axis

    # Step 4: Compute the 4 corners of the bounding box in the vehicle's local coordinate system
    local_corners = [
        carla.Vector3D(extent_x, extent_y, 0),   # Front-Right
        carla.Vector3D(extent_x, -extent_y, 0),  # Front-Left
        carla.Vector3D(-extent_x, -extent_y, 0), # Rear-Left
        carla.Vector3D(-extent_x, extent_y, 0)   # Rear-Right
    ]

    # Step 5: Transform the local corners into global coordinates
    global_corners = []
    for corner in local_corners:
        # Apply rotation (vehicle's transform) to each corner
        global_corner = (
            vehicle_transform.location +
            forward_vector * corner.x +
            right_vector * corner.y
        )
        global_corners.append(global_corner)

    return global_corners

def line_intersection(corner1, corner2, center1, center2):
    # Unpack the corner and center coordinates
    x1, y1 = corner1
    x2, y2 = corner2
    x3, y3, z3 = center1
    x4, y4, z4 = center2
    
    # Calculate the orientation of the triplet (p, q, r)
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        return 0 if val == 0 else (1 if val > 0 else 2)

    # Check if point q lies on segment pr
    def on_segment(p, q, r):
        return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])

    # Check for intersection of two line segments (corner1, corner2) and (center1, center2)
    o1 = orientation((x1, y1), (x2, y2), (x3, y3))
    o2 = orientation((x1, y1), (x2, y2), (x4, y4))
    o3 = orientation((x3, y3), (x4, y4), (x1, y1))
    o4 = orientation((x3, y3), (x4, y4), (x2, y2))
    
    # General case
    if o1 != o2 and o3 != o4:
        # Calculate intersection point using line-line intersection formula
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:
            return None  # Parallel lines (no intersection)
        intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
        intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
        return (intersect_x, intersect_y)
    
    # Special cases (collinear points)
    if o1 == 0 and on_segment((x1, y1), (x3, y3), (x2, y2)):
        return (x3, y3)
    if o2 == 0 and on_segment((x1, y1), (x4, y4), (x2, y2)):
        return (x4, y4)
    if o3 == 0 and on_segment((x3, y3), (x1, y1), (x4, y4)):
        return (x1, y1)
    if o4 == 0 and on_segment((x3, y3), (x2, y2), (x4, y4)):
        return (x2, y2)
    
    return None  # No intersection

class CarlaRos2Node(Node):
    def __init__(self):
        super().__init__('carla_api')
        self.get_logger().info("Run CARLA API")
        self.pub = self.create_publisher(VehicleDataArray, '/adas/hero0/carla_api', 10)
        self.timer = self.create_timer(0.05, self.publish_vehicle_data)  # 20 Hz
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.vehicles = []
        self.ego_vehicle = None

    def get_carla_timestamp(self):
        carla_stamp = self.world.get_snapshot().timestamp.elapsed_seconds
        secs = int(carla_stamp)
        nsecs = int((carla_stamp - secs) * 1e9)
        return Time(sec=secs, nanosec=nsecs)

    def publish_vehicle_data(self):
        if not hasattr(self.publish_vehicle_data, "skip_until_time"):
            skip_until_time = self.get_clock().now()

        # Get all vehicles in the simulation
        self.vehicles = self.world.get_actors().filter('vehicle.*')
        if not self.vehicles:
            if self.get_clock().now() < skip_until_time:
                return
            self.get_logger().debug("No vehicles found in the simulation.")
            skip_until_time = self.get_clock().now()
            return

        # Find ego vehicle
        self.ego_vehicle = None
        for vehicle in self.vehicles:
            if 'hero' in vehicle.attributes['role_name']:
                self.ego_vehicle = vehicle
                break
        
        if not self.ego_vehicle:
            if len(self.vehicles) > 0:
                self.ego_vehicle = self.vehicles[0]
            else:
                return

        vehicle_data_array = VehicleDataArray()
        vehicle_data_array.header = Header()
        vehicle_data_array.header.stamp = self.get_carla_timestamp()
        vehicle_data_array.header.frame_id = ""  # this frame_id is used to check whether the data is valid or a duplicate

        vehicle_data_array.vehicle_count = len(self.vehicles) - 1

        change_basis_matrix, ref_point = change_coordinate(self.ego_vehicle)
        
        for vehicle in self.vehicles:
            location = vehicle.get_location()
            velocity = vehicle.get_velocity()
            global_corners = get_bbox_corners_in_global(vehicle)
            local_corners = []

            for corner in global_corners:
                # Subtract the reference point (vehicle's global position) to center around the vehicle
                cur_position = np.array([[corner.x], [corner.y], [corner.z]])

                # Apply the inverse transformation (global -> local)
                relate_position = np.matmul(change_basis_matrix, cur_position - ref_point)

                # Extract the local coordinates (relative to the ego vehicle)
                relative_x = relate_position[0, 0]
                relative_y = relate_position[1, 0]
                relative_z = relate_position[2, 0]

                # Store the local corner
                local_corners.append((relative_x, relative_y, relative_z))
            
            edges = [((local_corners[i][0], local_corners[i][1]), 
                      (local_corners[(i + 1) % 4][0], local_corners[(i + 1) % 4][1])) for i in range(4)]

            speed_kph_magnitude = math.sqrt(self.ego_vehicle.get_velocity().x**2 + 
                                           self.ego_vehicle.get_velocity().y**2 + 
                                           self.ego_vehicle.get_velocity().z**2) * 3.6
                                           
            speed_mps = np.array([[self.ego_vehicle.get_velocity().x], 
                                  [self.ego_vehicle.get_velocity().y],
                                  [self.ego_vehicle.get_velocity().z]])
            speed_kph = speed_mps * 3.6
            
            # Compute the relative position
            cur_position = np.array([[location.x],
                                     [location.y],
                                     [location.z]])
            
            # Multiply for inverse to move the car global coor in to local coor
            relate_position = np.matmul(change_basis_matrix, cur_position - ref_point)
    
            relative_x = relate_position[0, 0]
            relative_y = relate_position[1, 0]
            relative_z = relate_position[2, 0]

            for i, (corner1, corner2) in enumerate(edges):
                intersection = line_intersection(corner1, corner2, 
                                                np.array([0.0, 0.0, 0.0]), 
                                                np.array([relative_x, relative_y, relative_z]))
                if intersection: 
                    relative_x, relative_y = intersection
                    break
            
            centripetal_location = math.sqrt(relative_x**2 + relative_y**2 + relative_z**2)
            
            # Compute related velocity
            vel_np = np.array([[velocity.x - self.ego_vehicle.get_velocity().x],
                              [velocity.y - self.ego_vehicle.get_velocity().y],
                              [velocity.z - self.ego_vehicle.get_velocity().z]])
            
            relate_vel = np.matmul(change_basis_matrix, vel_np)
            speed_kph = np.matmul(change_basis_matrix, speed_kph)

            # Create and fill the VehicleData message
            vehicle_data = VehicleData()
            vehicle_data.id = vehicle.id
            vehicle_data.type = "surround"

            if vehicle.id == self.ego_vehicle.id:
                vehicle_data.type = "ego"

            vehicle_data.location.x = relative_x
            vehicle_data.location.y = relative_y
            vehicle_data.location.z = relative_z
            vehicle_data.velocity.x = relate_vel[0, 0]
            vehicle_data.velocity.y = relate_vel[1, 0]
            vehicle_data.velocity.z = relate_vel[2, 0]

            vehicle_data.speed_kph = speed_kph_magnitude

            vehicle_data.vector_speed_kph.x = speed_kph[0, 0]
            vehicle_data.vector_speed_kph.y = speed_kph[1, 0]
            vehicle_data.vector_speed_kph.z = speed_kph[2, 0]

            vehicle_data.centripetal_location = centripetal_location - 4.75

            vehicle_data_array.vehicles.append(vehicle_data)
        
        self.pub.publish(vehicle_data_array)

def signal_handler(sig, frame):
    logger = get_logger('carla_api_logger')
    logger.info('"SIGINT caught, shutting down CARLA API node...')
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)
    
    node = CarlaRos2Node()
    rclpy.spin(node)

    # Cleanup and shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()