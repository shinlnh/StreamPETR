#!/bin/bash
set -e

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath "$SCRIPT_PATH/..")"

# Clear PC environment variables
unset PC_BUILD
unset LD_LIBRARY_PATH

# Setup cross-compilation environment
SDK_ENV_SCRIPT="/prj/es-automotive/adas/common/sdk/v3.2.1/environment-setup-armv8a-poky-linux"
source "$SDK_ENV_SCRIPT" || { echo "Error: Failed to source SDK environment script"; exit 1; }

# ROS2 Humble environment
export ROS_DISTRO=humble
export ROS_LOCALHOST_ONLY=0
export ROS_PYTHON_VERSION=3
export ROS_VERSION=2

# Python and ROS paths
export PYTHONPATH="$OECORE_NATIVE_SYSROOT/usr/lib/python3.10/site-packages:$OECORE_TARGET_SYSROOT/opt/ros/humble/lib/python3.10/site-packages"
export AMENT_PREFIX_PATH="$OECORE_TARGET_SYSROOT/opt/ros/humble"
export CMAKE_PREFIX_PATH="$OECORE_TARGET_SYSROOT/opt/ros/humble:$OECORE_TARGET_SYSROOT/opt/ros/humble/share:$OECORE_TARGET_SYSROOT/opt/ros/humble/lib/cmake"
