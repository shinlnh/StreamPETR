#!/bin/bash
set -e

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath $SCRIPT_PATH/..)"

# Ensure script run correct folder
cd $SDK_PATH

# Modules selected
BUILD_ADAS_SERVICE=OFF
BUILD_CAN_SERVICE=OFF
BUILD_OLED_SERVICE=OFF

# CMake Argurements
CMAKE_ARG=""

# Parse arguments
BUILD_MODE="dds"
JOBS="4"    # Default to using 4 cores
MODE_EXPLICITLY_SET=false

# Help function
print_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Build Options:"
    echo "  -m, --mode BUILD_MODE      Specify the execution mode (dds, can)"
    echo "                             Only applicable when building ADAS Service"
    echo "                             (Default: dds)"
    echo "  -j N                       Specify the number of parallel jobs (e.g., -j 4)"
    echo "  -jN                        Alternative syntax to specify the number of jobs (e.g., -j4)"
    echo "                             Default: 4 jobs"
    echo ""
    echo "Module Options:"
    echo "  If no service is specified, ADAS Service (with Visualize) is built by default."
    echo "  --adas-service             Build ADAS Service (with Visualize Service)"
    echo "  --can-service              Build CAN Service"
    echo "  --oled-service             Build OLED Service"
    echo ""
    echo "  -h, --help                 Display this help message and exit"
    echo ""
    echo "Notes:"
    echo "  - ADAS Visualize Service is only built when ADAS Service is enabled"
    echo "  - Mode (-m) is only meaningful for ADAS Service builds"
    echo "  - Multiple services can be built together by combining flags"
    echo ""
    echo "Examples:"
    echo "  $0                         Build ADAS Service (default, DDS mode)"
    echo "  $0 -m can -j 4             Build ADAS Service in CAN mode with 4 parallel jobs"
    echo "  $0 --adas-service --can-service    Build both ADAS and CAN Services"
    echo "  $0 --can-service           Build only CAN Service"
    echo "  $0 --oled-service          Build only OLED Service"
    return 0
}

parse_args() {
    local number
    while [[ $# -gt 0 ]]; do
        case $1 in
            -m|--mode)
                BUILD_MODE="$2"
                MODE_EXPLICITLY_SET=true
                shift 2
                ;;
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
            -h|--help)
                print_help
                return 1
                ;;
            *)
                echo "Error: Invalid argurements"
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

    # link ROS message when using cmakelist
    CMAKE_ARG+=" -Dipc_helper_DIR=$SDK_PATH/$1/install/ipc_helper/share/ipc_helper/cmake"
    CMAKE_ARG+=" -Dcarla_msgs_custom_DIR=$SDK_PATH/$1/install/carla_msgs_custom/share/carla_msgs_custom/cmake"

    if [ "$1" = "can_build" ]; then
        CMAKE_ARG+=" -DENABLE_CAN=ON"
    fi
}

build_ros_msg() {
    local toolchain_file="${1:-$SDK_PATH/toolchain.cmake}"
    colcon build \
        --base-paths $SDK_PATH/ros_bridge_support_ws/src $SDK_PATH/common/ipc_helper/src \
        --cmake-args -DCMAKE_TOOLCHAIN_FILE="$toolchain_file" \
        --packages-select ipc_helper carla_msgs_custom

    # Strip absolute sysroot paths from colcon-generated setup scripts so
    # that the install tree is not tied to the build machine's SDK location.
    # SDKTARGETSYSROOT (e.g. /opt/sdk/.../sysroots/aarch64-fsl-linux) is
    # removed, leaving target-relative paths such as /usr/lib.
    # We cover all setup variants that colcon generates.
    local _strip="${SDKTARGETSYSROOT}"
    for _f in install/setup.bash install/setup.sh \
               install/local_setup.bash install/local_setup.sh; do
        [ -f "$_f" ] && sed -i "s|${_strip}||g" "$_f"
    done
    unset _strip _f
}

build_with_mode() {
    local build_dir
    local build_mode
    if [[ $# != 1 ]]; then
        return 1
    fi

    build_mode="$1"

    local ENV_SCRIPT="$SCRIPT_PATH/setenv_nativesdk.sh"
    local TOOLCHAIN_FILE="$SDK_PATH/toolchain.cmake"

    if [ "$BUILD_CAN_SERVICE" = "ON" ] && [ "$BUILD_ADAS_SERVICE" = "OFF" ] && [ "$BUILD_OLED_SERVICE" = "OFF" ]; then
        # Building CAN service only - use FSL SDK configuration and separate build dir
        build_dir="can_service_build"
        ENV_SCRIPT="$SCRIPT_PATH/setenv_can.sh"
        TOOLCHAIN_FILE="$SDK_PATH/toolchain_can.cmake"
        echo "==> Building CAN service with FSL SDK configuration"
        echo "==> Using build directory: ${build_dir}"
    else
        # Building ADAS service or multiple services - use Poky SDK configuration
        build_dir="${build_mode}_build"
        echo "==> Building with Poky SDK configuration"
        echo "==> Using build directory: ${build_dir}"
    fi

    # Create build directory
    mkdir -p ${build_dir}
    cd ${build_dir}

    # Setup environment
    source "$ENV_SCRIPT"

    if [ "$BUILD_CAN_SERVICE" = "ON" ] && [ "$BUILD_ADAS_SERVICE" = "OFF" ] && [ "$BUILD_OLED_SERVICE" = "OFF" ]; then
        build_ros_msg "$TOOLCHAIN_FILE"
    else
        build_ros_msg
    fi

    # Build main project
    create_cmake_args "${build_dir}"

    if [ "$BUILD_CAN_SERVICE" = "ON" ] && [ "$BUILD_ADAS_SERVICE" = "OFF" ] && [ "$BUILD_OLED_SERVICE" = "OFF" ]; then
        cmake .. -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" $CMAKE_ARG
    else
        cmake .. $CMAKE_ARG
    fi

    make -j$JOBS -l$(nproc)

    echo "Using mode ${BUILD_MODE^^}"
    echo "The base path for build directory is ${SDK_PATH}/${build_dir}"
}

parse_args "$@"

# Default to ADAS service if no service is specified
if [ "$BUILD_ADAS_SERVICE" = "OFF" ] && [ "$BUILD_CAN_SERVICE" = "OFF" ] && [ "$BUILD_OLED_SERVICE" = "OFF" ]; then
    BUILD_ADAS_SERVICE=ON
fi

# Warn if mode is specified without ADAS service
if [ "$MODE_EXPLICITLY_SET" = true ] && [ "$BUILD_ADAS_SERVICE" = "OFF" ]; then
    echo "Warning: Build mode (-m) is only applicable when building ADAS Service."
    echo "         The mode option will be ignored."
    echo ""
fi

# Block mixed build: Only ADAS or CAN at a time
if [ "$BUILD_ADAS_SERVICE" = "ON" ] && [ "$BUILD_CAN_SERVICE" = "ON" ]; then
    echo "Error: Cannot build both ADAS and CAN services together."
    echo ""
    echo "Please build separately:"
    echo "  - For ADAS service: ./script/build.sh --adas-service"
    echo "  - For CAN service:  ./script/build.sh --can-service"
    echo ""
    exit 1
fi

# Build stage
build_with_mode "${BUILD_MODE}"
