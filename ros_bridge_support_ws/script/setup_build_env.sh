#!/bin/bash
set -e
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ROS_WS_PATH="$(realpath $SCRIPT_PATH/..)"
SDK_PATH="$(realpath $ROS_WS_PATH/..)"

# Setup Carla and ROS bridge
CARLA_ROS_BRIDGE_WS=/prj/es-automotive/adas/common/carla_ros_bridge_2
source $CARLA_ROS_BRIDGE_WS/install/setup.sh

# Setup OpenCV 4.2.0 for ROS 2 Foxy
export OpenCV_DIR="/usr/lib/x86_64-linux-gnu/cmake/opencv4"

# Export cuda for PC
export PATH=/usr/local/cuda-11.1/bin:$PATH 
export LD_LIBRARY_PATH=/usr/local/cuda-11.1/lib64:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
