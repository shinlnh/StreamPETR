import launch
from launch import LaunchService
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from rclpy.logging import get_logger

import os
import subprocess
from pathlib import Path
from parser_launch import inject_launch_arguments, get_ros_parser

ROS_WS = str(Path(__file__).resolve().parents[1])
SDK_WS = str(Path(__file__).resolve().parents[2])
USER_NAME = os.environ.get('USER')
LOCALHOST='127.0.0.1'
ADAS_IP='192.168.240.110'
IVI_IP='192.168.240.1'

def get_git_info():
    """Get git version information"""
    try:
        git_version = subprocess.check_output(
            ['git', 'describe', '--tags', '--always', '--dirty'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_version = "unknown"
    
    try:
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_hash = "unknown"
    
    try:
        git_branch = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_branch = "unknown"
    
    return git_version, git_hash, git_branch

def define_launch_argument():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name = 'board_ip',
            default_value = '127.0.0.1',
            description = 'IP of target board (default: localhost)'
        ),
        launch.actions.DeclareLaunchArgument(
            name = 'vehicle_name',
            default_value = 'hero0',
            description = 'The name of the ego vehicle the user controls in this session'
        ),
        launch.actions.DeclareLaunchArgument(
            name = 'image_port',
            default_value = '3445',
            description = 'Port used for stream image from camera'
        ),
        launch.actions.DeclareLaunchArgument(
            name = 'visualize_radar',
            default_value = 'false',
            description = 'Enable Radar Visualization'
        ),
        launch.actions.DeclareLaunchArgument(
            name = 'visualize_max_y',
            default_value = '260',
            description = 'Maximum Y range (forward) for radar visualization in meters'
        ),
        launch.actions.DeclareLaunchArgument(
            name = 'visualize_max_x',
            default_value = '50',
            description = 'Maximum X range (lateral) for radar visualization in meters'
        )
    ])
    return ld

def define_main_action():
    ld = launch.LaunchDescription([
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_rgb_front_node',
            name = 'adas_bridge_rgb_front_node',
            output = 'log',
            parameters=[{
                'board_ip': LaunchConfiguration('board_ip'),
                'image_port': LaunchConfiguration('image_port'),
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }]
        ),
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_radar_front_node',
            name = 'adas_bridge_radar_front_node',
            output = 'log',
            parameters=[{
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'visualize_radar': LaunchConfiguration('visualize_radar'),
                'visualize_max_y': LaunchConfiguration('visualize_max_y'),
                'visualize_max_x': LaunchConfiguration('visualize_max_x')
            }]
        ),
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_imu_node',
            name = 'adas_bridge_imu_node',
            output = 'log',
            parameters=[{
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }]
        ),
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_odometer_node',
            name = 'adas_bridge_odometer_node',
            output = 'log',
            parameters=[{
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }]
        ),
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_control_msg_node',
            name = 'adas_bridge_control_msg_node',
            output = 'log',
            parameters=[{
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }]
        ),
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_gnss_node',
            name = 'adas_bridge_gnss_node',
            output = 'log',
            parameters=[{
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }]
        ),
        Node(
            package = 'adas_bridge',
            executable = 'adas_bridge_vehicle_status_node',
            name = 'adas_bridge_vehicle_status_node',
            output = 'log',
            parameters=[{
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }]
        )
    ])
    return ld


def generate_launch_description():
    ld = launch.LaunchDescription([
        *define_launch_argument().entities,
        *define_main_action().entities,
        RegisterEventHandler(
            OnProcessExit(
                on_exit=[
                    LogInfo(msg="A node has exited. Shutting down entire launch..."),
                    EmitEvent(event=Shutdown())
                ]
            )   
        )
    ])
    return ld

def main():
    # Get logger
    logger = get_logger('launch_logger')
    
    # Display version information
    git_version, git_hash, git_branch = get_git_info()
    print("==============================================")
    print(f"BV ADAS Bridge {git_version}")
    print(f"Commit: {git_hash}")
    print(f"Branch: {git_branch}")
    print("==============================================")
    
    # Parse python argurements
    parser = get_ros_parser()
    args = parser.parse_args()
    mode = args.mode
    ros_args = args.ros_args
    if ros_args is None:
        ros_args = []

    # Update parameter for each mode
    if mode == "local":
        logger.info(f"Run with LOCAL mode, IP = {LOCALHOST}")
        ros_args += [f"board_ip:={LOCALHOST}"]
    elif mode == "board":
        board_ip_list = [opt for opt in ros_args if 'board_ip:=' in opt]
        if len(board_ip_list) > 0:
            logger.info(f"Run with BOARD mode, IP = {board_ip_list[0][10:]}")
        else:
            logger.info(f"Run with BOARD mode, IP = {ADAS_IP}")
            ros_args += [f"board_ip:={ADAS_IP}"]


    # Create a LaunchDescription with the node
    ld = generate_launch_description()

    # Start the launch service
    ls = LaunchService()
    ls.include_launch_description(ld)
    inject_launch_arguments(ls, ros_args)
    ls.run()

if __name__ == '__main__':
    main()