#!/bin/bash
# Get path
set -e
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
ROS_WS_PATH="$(realpath "$SCRIPT_PATH/..")"

# Build 4 packages in parallel by default.
PACKAGES=4

print_help() {
    echo "Usage: $0 [options] [arguments]"
    echo ""
    echo "Options:"
    echo "  -j <number>    Set number of packages to process parallely"
    echo "  -j<number>     Same as above (e.g., -j4)"
    echo "  -h, --help     Show this help message"
}

parse_arg() {
    local number_packages

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -j)
                if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
                    PACKAGES="$2"
                    shift 2
                else
                    echo "Error: number must be provided after -j"
                    print_help
                    return 1
                fi
                ;;
            -j[0-9]*)
                number_packages="${1:2}"
                if [[ "$number_packages" =~ ^[0-9]+$ ]]; then
                    PACKAGES="$number_packages"
                    shift 1
                else
                    echo "Error: -j must be followed by a number, example -j4"
                    print_help
                    return 1
                fi
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

parse_arg "$@"

# Setup environment
source "$ROS_WS_PATH/script/setup_build_env.sh"

# Build the ROS workspace using colcon
colcon build --parallel-workers "$PACKAGES" \
    --base-paths "$ROS_WS_PATH/src" "$SDK_PATH/common/ipc_helper/src" \
    --build-base "$ROS_WS_PATH/build" \
    --install-base "$ROS_WS_PATH/install" \
    --packages-select carla_msgs_custom adas_bridge ipc_helper
