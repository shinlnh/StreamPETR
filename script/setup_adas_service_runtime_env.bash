#!/bin/bash

set -e
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath "$SCRIPT_PATH/..")"

BUILD_MODE="dds"
BUILD_DIR="build"

print_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -m, --mode MODE   Specify the execution mode (dds, can)"
    echo "                    (Default: dds)"
    echo "  -h, --help        Show this help message and exit"
    echo ""
    echo "Note: option -m, --mode impact if running in board only"
    echo "Examples:"
    echo "  $0 -m can"
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -m|--mode)
                BUILD_MODE="$2"
                shift 2
                ;;
            -h|--help)
                print_help
                return 2
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                return 1
                ;;
        esac
    done
}

parse_args "$@"

# Get the base path for build directory
if [ "$(uname -m)" == "x86_64" ]; then
    BUILD_MODE="pc"
fi
BUILD_DIR="${BUILD_MODE}_build"

# Export all required environment variable
if [ "$(uname -m)" == "aarch64" ]; then
    # For board
    export LD_LIBRARY_PATH="${SDK_PATH}/${BUILD_DIR}/lib/"
    export QT_QPA_FONTDIR="/usr/share/fonts/ttf/"
    export QT_XCB_GL_INTEGRATION=none

else
    # For PC
    TENSORRT_LIB="/prj/es-automotive/adas/common/TensorRT-8.6.1.6/lib"
    PC_BUILD_LIB="${SDK_PATH}/${BUILD_DIR}/lib/"
    QT_BASE_DIR="/opt/qt515"
    
    export LD_LIBRARY_PATH="${TENSORRT_LIB}:${PC_BUILD_LIB}:${LD_LIBRARY_PATH}"
    export QTDIR="${QT_BASE_DIR}"
    export PATH="${QT_BASE_DIR}/bin:${PATH}"
    export LD_LIBRARY_PATH="${QT_BASE_DIR}/lib/x86_64-linux-gnu:${QT_BASE_DIR}/lib:${LD_LIBRARY_PATH}"
    export PC_BUILD=true
fi

# Source setup environment from third party
export ROS_DOMAIN_ID=0
source "${SDK_PATH}/${BUILD_DIR}/install/setup.bash"
