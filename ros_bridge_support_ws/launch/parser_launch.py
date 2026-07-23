from launch import LaunchService
import argparse

def inject_launch_arguments(ls: LaunchService, argv):
    if argv is None:
        return
    for arg in argv:
        if ':=' in arg:
            name, value = arg.split(':=', 1)
            ls.context.launch_configurations[name] = value

def get_ros_parser():
    parser = argparse.ArgumentParser(
        description="Launch script for running the system"
    )
    parser.add_argument(
        "-m", "--mode",
        type = str,
        choices = ["local", "board"],
        help = "Mode of operation (local or board)"
    )
    parser.add_argument(
        "--ros-args",
        nargs = argparse.REMAINDER,
        help = "Additional ROS arguments (e.g., param1:=value1 param2:=value2)"
    )
    return parser
