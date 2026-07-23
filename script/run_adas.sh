#!/usr/bin/env bash
set -e

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath "${SCRIPT_PATH}/..")"

BUILD_MODE="dds"
BUILD_DIR="build"
SERVICE_EXEC="adas_service"                     # ADAS Service Execution
VISUALIZE_EXEC="adas_visualize_service"         # ADAS Visualize Service Execution

# Log Configuration
LOG_BASE_DIR="${SDK_PATH}/log"
SERVICE_LOG_FILE=""
VISUALIZE_LOG_FILE=""
KEEP_LOGS=10
ENABLE_LOG="false"

# Default Options of Service
DEBUG_AEB="false"
LAUNCH="service"
SRC_TYPE="udp"
IP_ADDR="192.168.240.110"

cd ${SDK_PATH}

# Source log utilities
source "${SCRIPT_PATH}/log_utils.sh"

# Help function
print_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -m, --mode MODE         Specify the execution mode (dds, can)."
    echo "                          (Default: dds)"
    echo "  --launch=<value>        Launch Execution Service."
    echo "                          Available fields to launch:"
    echo "                          * service"
    echo "                              ADAS Service (default)"
    echo "                          * visualize"
    echo "                              ADAS Visualize Service"
    echo "                          * all"
    echo "                              Run all"
    echo "  --src-type=<value>      Specify the source type (udp|tcp|camera|video)."
    echo "                          (Default: udp)"
    echo "  --service-ip=<ip_addr>  Specify the IP address of ADAS Service."
    echo "                          (Default: 192.168.240.110 (127.0.0.1 if launch=all))"
    echo "  --enable-logfile        Enable logging to file."
    echo "                          (Default: disabled)"
    echo "  --log-dir=<path>        Custom log directory."
    echo "                          (Default: ${SDK_PATH}/log)"
    echo "  --keep-logs=<number>    Number of log files to keep."
    echo "                          (Default: 20)"
    echo "  -aeb, --debug_aeb       Debug AEB mode."
    echo "  -h, --help              Show this help message and exit."
    echo ""
    echo "Log files location:"
    echo "  ${LOG_BASE_DIR}/<service_name>/<service_name>_YYYYMMDD_HHMMSS.log"
    echo ""
    echo "Examples:"
    echo "  $0 --launch=service"
    echo "  $0 --launch=service -m can"
    echo "  $0 --launch=service --enable-log"
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -m|--mode)
                BUILD_MODE="$2"
                shift 2
                ;;
            --launch=*)
                LAUNCH="${1#*=}"
                shift
                ;;
            --src-type=*)
                SRC_TYPE="${1#*=}"
                shift
                ;;
            --service-ip=*)
                IP_ADDR="${1#*=}"
                shift
                ;;
            -aeb|--debug_aeb)
                DEBUG_AEB="true"
                shift
                ;;
            --enable-logfile)
                ENABLE_LOG="true"
                shift
                ;;
            --log-dir=*)
                LOG_BASE_DIR="${1#*=}"
                shift
                ;;
            --keep-logs=*)
                KEEP_LOGS="${1#*=}"
                shift
                ;;
            -h|--help)
                print_help
                return 1
                ;;
            *)
                echo "Error: Invalid argurements"
                return 1
                ;;
        esac
    done

    # Update IP Address if launch all
    if [ "$LAUNCH" = "all" ]; then
        IP_ADDR="127.0.0.1"
    fi

    # Validate IP Address
    valid_ip=false
    if [[ $IP_ADDR =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
        # Further check: ensure numbers are between 0 and 255
        valid_ip=true
        IFS='.' read -r -a octets <<< "$IP_ADDR"
        for octet in "${octets[@]}"; do
            if ((octet < 0 || octet > 255)); then
                valid_ip=false
                break
            fi
        done
    fi

    if [ "$valid_ip" = false ]; then
        echo "Error: Invalid IP Address"
        return 1
    fi

    return 0
}

parse_args $@


validate_executables() {
    for exec_name in "$@"; do
        if [ ! -f "${SDK_PATH}/${BUILD_DIR}/${exec_name}/${exec_name}" ]; then
            printf "[ERRO] Can't find the ${exec_name} executable. Make sure it exists in ${SDK_PATH}/${BUILD_DIR}/${exec_name}/. Exiting...\n"
            exit 1
        fi
    done
}

cleanup() {
echo ""    
    if [ ! -z "$PID_SERVICE" ] && kill -0 $PID_SERVICE 2>/dev/null; then
        echo "[INFO]: Shutting down ${SERVICE_EXEC}..."
        kill -TERM -$PID_SERVICE 2>/dev/null || true

        # Clean old and keep last logs
        cleanup_old_logs ${SERVICE_EXEC} ${KEEP_LOGS}

        if [ -n "$SERVICE_LOG_FILE" ]; then
            echo "[INFO]: Log file path: ${SERVICE_LOG_FILE}"
        fi
    fi
    
    if [ ! -z "$PID_VISUALIZE" ] && kill -0 $PID_VISUALIZE 2>/dev/null; then
        echo "[INFO]: Shutting down ${VISUALIZE_EXEC}..."
        kill -TERM -$PID_VISUALIZE 2>/dev/null || true

        # Clean old and keep last logs
        cleanup_old_logs ${VISUALIZE_EXEC} ${KEEP_LOGS}

        if [ -n "$VISUALIZE_LOG_FILE" ]; then
            echo "[INFO]: Log file path: ${VISUALIZE_LOG_FILE}"
        fi
    fi

    wait 2>/dev/null || true
}

run_adas_service() {
    validate_executables ${SERVICE_EXEC}

    echo -e "[INFO]: Starting ADAS Service..."
    if [ "$ENABLE_LOG" = "true" ]; then
        SERVICE_LOG_FILE=$(setup_log_dir ${SERVICE_EXEC} ${LOG_BASE_DIR})
        echo -e "[INFO]: Enable log file"
    else
        echo -e "[INFO]: Disable log file"
    fi
    
    (
        if [ "$ENABLE_LOG" = "true" ]; then
            exec stdbuf -oL ${SDK_PATH}/${BUILD_DIR}/${SERVICE_EXEC}/${SERVICE_EXEC} \
            --src_type=${SRC_TYPE} --debug_aeb=${DEBUG_AEB} \
            2>&1 | tee -a "${SERVICE_LOG_FILE}"
        else
            exec ${SDK_PATH}/${BUILD_DIR}/${SERVICE_EXEC}/${SERVICE_EXEC} \
            --src_type=${SRC_TYPE} --debug_aeb=${DEBUG_AEB}
        fi
    ) &
    PID_SERVICE=$!
}

run_adas_visualize_service() {
    validate_executables ${VISUALIZE_EXEC}

    echo -e "Starting ADAS Visualize..."
    if [ "$ENABLE_LOG" = "true" ]; then
        VISUALIZE_LOG_FILE=$(setup_log_dir ${VISUALIZE_EXEC} ${LOG_BASE_DIR})
        echo -e "Enable log file"
    else
        echo -e "Disable log file"
    fi

    (
        if [ "$ENABLE_LOG" = "true" ]; then
            exec stdbuf -oL ${SDK_PATH}/${BUILD_DIR}/${VISUALIZE_EXEC}/${VISUALIZE_EXEC} \
            --service_ip=${IP_ADDR} \
            2>&1 | tee -a "${VISUALIZE_LOG_FILE}"
        else
            exec ${SDK_PATH}/${BUILD_DIR}/${VISUALIZE_EXEC}/${VISUALIZE_EXEC} \
            --service_ip=${IP_ADDR}
        fi
    ) &
    PID_VISUALIZE=$!
}

# Register handler for signals
trap cleanup EXIT SIGINT SIGTERM

# Get the base path for build directory
if [ "$(uname -m)" == "x86_64" ]; then
    BUILD_MODE="pc"
fi
BUILD_DIR="${BUILD_MODE}_build"

source ${SCRIPT_PATH}/setup_adas_service_runtime_env.bash -m ${BUILD_MODE}

TARGET_EXECUTION=""
case "$LAUNCH" in
    service)
        run_adas_service
        wait
        ;;
    visualize)
        run_adas_visualize_service
        wait
        ;;
    all)
        run_adas_service
        run_adas_visualize_service
        wait
        ;;
esac