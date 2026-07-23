#!/bin/bash
set -e
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ROS_WS_PATH="$(realpath $SCRIPT_PATH/..)"

# Setup environment and install ROS 2 node for ADAS bridge
source $SCRIPT_PATH/setup_runtime_env.sh

# Run ADAS bridge
python $ROS_WS_PATH/launch/adas_bridge_launch.py $@
