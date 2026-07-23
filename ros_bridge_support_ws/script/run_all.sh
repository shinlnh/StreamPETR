#!/bin/bash
set -e
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ROS_WS_PATH="$(realpath $SCRIPT_PATH/..)"

source $SCRIPT_PATH/setup_runtime_env.sh
python $ROS_WS_PATH/launch/all_launch.py $@
