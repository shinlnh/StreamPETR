#!/bin/bash
# get path to script
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
ROS_WS_PATH="$(realpath $SCRIPT_PATH/..)"

clean_all()
{
    rm -rf "$ROS_WS_PATH/build" "$ROS_WS_PATH/install" "$ROS_WS_PATH/log" "$ROS_WS_PATH/launch/__pycache__"
}

clean_all