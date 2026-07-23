#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from carla_msgs_custom.msg import WheelSensor
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

import signal
import sys
import math
import carla
import weakref
import numpy as np

STRAIGHT_THRESHOLD_DEG = 0.1

class CarlaWheelAngleNode(Node):
    def __init__(self):
        super().__init__("carla_wheel_sensor_node")
        self.get_logger().info("Starting CARLA Wheel Angle API node...")

        # Declare parameters
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)

        # Read parameters
        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        # Connect to CARLA
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)

        self.world = self.client.get_world()
        self.vehicle = None

        # ROS Publisher with new message type
        self.pub = self.create_publisher(
            WheelSensor,
            "/carla/hero0/wheel_sensor",
            10
        )

        # Register callback for world tick (event-driven approach)
        weak_self = weakref.ref(self)
        self.tick_callback_id = self.world.on_tick(
            lambda snapshot: CarlaWheelAngleNode._on_world_tick(weak_self, snapshot)
        )

        self.get_logger().info("CARLA Wheel Angle node initialized!")

    def find_ego_vehicle(self):
        actors = self.world.get_actors().filter("vehicle.*")
        if not actors:
            return None

        for vehicle in actors:
            if vehicle.attributes.get("role_name", "") == "hero":
                return vehicle

        return actors[0]

    def compute_steering_angle(self, fl, fr):
        """
        Compute average steering angle from front wheel angles (Ackermann geometry).
        """
        if abs(fl) < STRAIGHT_THRESHOLD_DEG or abs(fr) < STRAIGHT_THRESHOLD_DEG:
            return (fl + fr) / 2.0

        fl_rad = math.radians(fl)
        fr_rad = math.radians(fr)
        avg_cot = (1.0 / math.tan(fl_rad) + 1.0 / math.tan(fr_rad)) / 2.0

        if abs(avg_cot) < 1e-9:
            return 0.0
        return math.degrees(math.atan(1.0 / avg_cot))

    def destroy_node(self):
        """
        Cleanup callback when node is shutting down.
        """
        try:
            if hasattr(self, 'tick_callback_id') and self.tick_callback_id is not None:
                self.world.remove_on_tick(self.tick_callback_id)
                self.get_logger().info("Removed world tick callback")
        except Exception as e:
            self.get_logger().warning(f"Error removing callback: {e}")

        super().destroy_node()

    @staticmethod
    def _on_world_tick(weak_self, snapshot):
        """
        Callback triggered on every Carla world tick (event-driven).
        This ensures synchronization with Carla server updates.
        """
        self = weak_self()
        if not self:
            return

        try:
            # Find ego vehicle if not already found
            if self.vehicle is None:
                self.vehicle = self.find_ego_vehicle()
                if self.vehicle is None:
                    return

                self.get_logger().info(
                    f"Tracking vehicle ID={self.vehicle.id}, role_name={self.vehicle.attributes.get('role_name')}"
                )

            # Create message with header and timestamp from Carla
            msg = WheelSensor()
            msg.header = Header()
            msg.header.frame_id = "hero0"

            # Get timestamp from Carla snapshot
            carla_timestamp = snapshot.timestamp.elapsed_seconds
            msg.header.stamp.sec = int(carla_timestamp)
            msg.header.stamp.nanosec = int((carla_timestamp - int(carla_timestamp)) * 1e9)

            # Get wheel angles and compute steering angle
            fl = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            fr = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)

            steering_deg = self.compute_steering_angle(fl, fr)
            msg.steering_angle = steering_deg

            # Get wheel position and convert to local
            msg.wheel_position = []
            vehicle_transform = self.vehicle.get_transform()
            inv_matrix = np.array(vehicle_transform.get_inverse_matrix())
            physics_control = self.vehicle.get_physics_control()
            for wheel in physics_control.wheels:
                wheel_pos_global = np.array([wheel.position.x, wheel.position.y, wheel.position.z, 1.0])
                wheel_pos_global[:3] /= 100  # convert cm to m
                wheel_pos_local = np.dot(inv_matrix, wheel_pos_global)
                wheel_pos_local[:3] *= 100  # convert m to cm
                
                wheel_pos = Vector3()
                wheel_pos.x, wheel_pos.y, wheel_pos.z = wheel_pos_local[:3].astype(float)
                msg.wheel_position.append(wheel_pos)

            # Publish the message
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Steering Angle Error: {e}")
            # Reset vehicle reference - it may have been destroyed
            self.vehicle = None

def signal_handler(sig, frame):
    print("SIGINT caught, shutting down wheel_angle_node...")
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)

    node = CarlaWheelAngleNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
