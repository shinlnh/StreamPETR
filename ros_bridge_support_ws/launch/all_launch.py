from launch import LaunchService, LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from adas_bridge_launch import SDK_WS, LOCALHOST, ADAS_IP
import adas_bridge_launch
import carla_bridge_launch
from parser_launch import inject_launch_arguments, get_ros_parser

from rclpy.logging import get_logger

import os
import time
import socket
import subprocess
import signal
import argparse
from colorama import init, Fore, Style

def show_help(ld: LaunchDescription) -> str:
    lines = ["ROS arguments (pass arguments as '<name>:=<value>'):\n"]

    for action in ld.entities:
        if isinstance(action, DeclareLaunchArgument):
            name = action.name
            default = ", ".join([value.text for value in action.default_value])
            if name == 'objects_definition_file':
                default = '$SDK_PATH/common/config/carla/' + os.path.basename(default)
            desc = action.description

            name_colored = f"{Fore.GREEN}{name}{Style.RESET_ALL}"
            if default is not None:
                lines.append(f"    {name_colored}: default '{default}'")
            else:
                lines.append(f"    {name_colored}: no default")

            if desc:
                lines.append(f"        {desc}")
            else:
                lines.append("")

    return "\n".join(lines)

def kill_carla_server():
    global CARLA_SERVER
    if CARLA_SERVER is None:
        return
    os.killpg(os.getpgid(CARLA_SERVER.pid), signal.SIGINT)
    try:
        CARLA_SERVER.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(CARLA_SERVER.pid), signal.SIGKILL)
        CARLA_SERVER.wait()
    CARLA_SERVER = None

def start_carla_server(context, *args, **kwargs):
    global CARLA_SERVER
    # Get Launch Parameter
    host = LaunchConfiguration('host').perform(context)
    port = int(LaunchConfiguration('port').perform(context))
    timeout = float(LaunchConfiguration('timeout').perform(context))

    # Try to start CARLA server
    start_time = time.time()
    logger = get_logger('launch_logger')
    is_open = False
    while time.time() - start_time < timeout:
        try:
            with socket.create_connection((host, port), timeout=2):
                logger.info(f"Port {port} on {host} is open.")
                is_open = True
                break
        except (ConnectionRefusedError, socket.timeout):
            if CARLA_SERVER is None:
                CARLA_SERVER = subprocess.Popen(
                    [
                        'bash',
                        os.path.join(SDK_WS, 'script', 'carla_script', 'run_carla.sh'),
                        '-r', 'OFF'
                    ],
                    stdout = subprocess.DEVNULL, 
                    stderr = subprocess.DEVNULL, 
                    text = True 
                )
            logger.warn(f"Waiting for {host}:{port}...")
            time.sleep(1)

    # Add action when start CARLA server successfully
    if is_open:
        all_actions = [
            *adas_bridge_launch.define_main_action().entities,
            *carla_bridge_launch.define_main_action().entities,
            RegisterEventHandler(
                OnProcessExit(
                    on_exit=[
                        LogInfo(msg="A node has exited. Shutting down entire launch..."),
                        EmitEvent(event=Shutdown())
                    ]
                )   
            )
        ]
        return all_actions
    
    # Log error when cannot start CARLA server
    logger.error(f"Timeout reached. Port {port} on {host} is still closed.")
    return []

def signal_handler(sig, frame):
    logger = get_logger('launch_logger')
    logger.info("Receive SIGINT, shutting down...")

def get_git_info():
    """Get git version information"""
    try:
        # Get git version tag
        git_version = subprocess.check_output(
            ['git', 'describe', '--tags', '--always', '--dirty'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_version = "unknown"
    
    try:
        # Get git commit hash
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_hash = "unknown"
    
    try:
        # Get git branch
        git_branch = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_branch = "unknown"
    
    return git_version, git_hash, git_branch
        
def main():
    # Get logger
    logger = get_logger('launch_logger')

    # Display version information
    git_version, git_hash, git_branch = get_git_info()
    print("==============================================")
    print(f"BV ROS Bridge {git_version}")
    print(f"Commit: {git_hash}")
    print(f"Branch: {git_branch}")
    print("==============================================")

    # Initial CARLA_SERVER instance
    global CARLA_SERVER
    CARLA_SERVER = None

    # Reset color after print text
    init(autoreset=True)

    # Handle when receiving SIGINT
    signal.signal(signal.SIGINT, signal_handler)

    # Create a LaunchDescription
    ld = LaunchDescription([
        *adas_bridge_launch.define_launch_argument().entities,
        *carla_bridge_launch.define_launch_argument().entities,
        OpaqueFunction(function=start_carla_server)
    ])

    # Parse python argurements
    parser = get_ros_parser()
    parser.epilog = show_help(ld)
    parser.formatter_class = argparse.RawDescriptionHelpFormatter

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

    # Start the launch service
    ls = LaunchService()
    ls.include_launch_description(ld)
    inject_launch_arguments(ls, ros_args)
    try:
        ls.run()
    finally:
        kill_carla_server()

if __name__ == "__main__":
    main()
