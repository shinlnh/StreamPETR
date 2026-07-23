#!/bin/bash 

set -e
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ROS_WS_PATH="$(realpath $SCRIPT_PATH/..)"
SDK_PATH="$(realpath $ROS_WS_PATH/..)"

# Setup Carla and ROS bridge
export CARLA_ROOT=/prj/common/carla
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla:$PYTHONPATH
export ROS_DOMAIN_ID=0

source $ROS_WS_PATH/install/setup.bash  # Install internal package
