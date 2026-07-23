#!/bin/bash
set -e

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath $SCRIPT_PATH/..)"

export BUILD_MODE="pc"

# Ensure script run correct folder
cd $SDK_PATH

# Create pc_build directory if it doesn't exist
mkdir -p ${BUILD_MODE}_build
cd ${BUILD_MODE}_build

# Default to using 4 cores
JOBS="4"

# Modules selected
BUILD_ADAS_SERVICE=OFF
BUILD_CAN_SERVICE=OFF
BUILD_OLED_SERVICE=OFF
BUILD_TEST=OFF
TEST_EXPLICITLY_SET=false

# CMake Argurements
CMAKE_ARG=""

# Help function
print_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Build Options:"
    echo "  -j N                       Specify the number of parallel jobs (e.g., -j 4)"
    echo "  -jN                        Alternative syntax to specify the number of jobs (e.g., -j4)"
    echo "                             Default: 4 jobs"
    echo ""
    echo "Module Options:"
    echo "  If no service is specified, ADAS Service (with Visualize) is built by default."
    echo "  --adas-service             Build ADAS Service (with Visualize Service)"
    echo "  --can-service              Build CAN Service"
    echo "  --oled-service             Build OLED Service"
    echo "  --test                     Enable coverage instrumentation for testing"
    echo ""
    echo "  -h, --help                 Display this help message and exit"
    echo ""
    echo "Notes:"
    echo "  - ADAS Visualize Service is only built when ADAS Service is enabled"
    echo "  - Multiple services can be built together by combining flags"
    echo "  - --test option only affects ADAS Service (enables coverage instrumentation)"
    echo "  - Use --test before running unit tests to generate coverage reports"
    echo ""
    echo "Examples:"
    echo "  $0                         Build ADAS Service (default)"
    echo "  $0 -j8 --adas-service      Build ADAS Service with 8 parallel jobs"
    echo "  $0 --adas-service --can-service    Build both ADAS and CAN Services"
    echo "  $0 --can-service           Build only CAN Service"
    echo "  $0 --test                  Build ADAS Service with coverage instrumentation"
}

# Parse arguments
parse_args() {
    local number
    while [[ $# -gt 0 ]]; do
        case $1 in
            -j)
                if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                    JOBS="$2"
                    shift 2
                else
                    echo "Error: number must be provided after -j"
                    print_help
                    return 1
                fi
                ;;
            -j*)
                number="${1:2}"
                if [[ "$number" =~ ^[0-9]+$ ]]; then
                    JOBS="$number"
                    shift 1
                else
                    echo "Error: -j must be followed by a number, example -j4"
                    print_help
                    return 1
                fi
                ;;
            --adas-service)
                BUILD_ADAS_SERVICE=ON
                shift 1
                ;;
            --can-service)
                BUILD_CAN_SERVICE=ON
                shift 1
                ;;
            --oled-service)
                BUILD_OLED_SERVICE=ON
                shift 1
                ;;
            --test)
                BUILD_TEST=ON
                TEST_EXPLICITLY_SET=true
                shift 1
                ;;
            -h|--help)
                print_help
                return 1
                ;;
            *)
                echo "Invalid argurements"
                print_help
                return 1
                ;;
        esac
    done
    return 0
}

# Create CMakeList argurements
create_cmake_args() {
    # Visualize service only builds with ADAS service
    if [ "$BUILD_ADAS_SERVICE" = "ON" ]; then
        BUILD_ADAS_VISUALIZE_SERVICE=ON
    else
        BUILD_ADAS_VISUALIZE_SERVICE=OFF
    fi

    CMAKE_ARG+=" -DBUILD_ADAS_SERVICE=$BUILD_ADAS_SERVICE"
    CMAKE_ARG+=" -DBUILD_ADAS_VISUALIZE_SERVICE=$BUILD_ADAS_VISUALIZE_SERVICE"
    CMAKE_ARG+=" -DBUILD_CAN_SERVICE=$BUILD_CAN_SERVICE"
    CMAKE_ARG+=" -DBUILD_OLED_SERVICE=$BUILD_OLED_SERVICE"
    CMAKE_ARG+=" -DBUILD_TEST=$BUILD_TEST"
}

build_ros_msg() {
    colcon build \
        --base-paths $SDK_PATH/ros_bridge_support_ws/src $SDK_PATH/common/ipc_helper/src \
        --packages-select carla_msgs_custom ipc_helper
    source install/setup.bash
}

parse_args "$@"

# Default to ADAS service if no service is specified
if [ "$BUILD_ADAS_SERVICE" = "OFF" ] && [ "$BUILD_CAN_SERVICE" = "OFF" ] && [ "$BUILD_OLED_SERVICE" = "OFF" ]; then
    BUILD_ADAS_SERVICE=ON
fi

# Warn if --test is used without ADAS service
if [ "$TEST_EXPLICITLY_SET" = true ] && [ "$BUILD_ADAS_SERVICE" = "OFF" ]; then
    echo "Warning: --test option is only applicable when building ADAS Service."
    echo "         Coverage instrumentation will not be enabled for CAN/OLED services."
    echo "         The BUILD_TEST flag may be ignored by the build system."
    echo ""
fi

create_cmake_args

# Unset LD_LIBRARY_PATH temporarily
unset LD_LIBRARY_PATH

# Setenv for PC_BUILD
source $SCRIPT_PATH/pc_setenv.sh

# Build ROS messages
build_ros_msg

# Build ADAS
cmake .. $CMAKE_ARG
make -j$JOBS -l$(nproc)
