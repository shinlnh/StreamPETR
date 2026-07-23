#!/usr/bin/env bash

# Setup log directory and return log file path
setup_log_dir() {
    local service_name=$1
    local log_base_dir=$2
    local timestamp=$(date +"%Y%m%d_%H%M%S")
    local log_dir="${log_base_dir}/${service_name}"
    # Configure log file path
    local log_file="${log_dir}/${service_name}_${timestamp}.log"

    # Validate and Create log directory
    if ! mkdir -p "${log_dir}" 2>/dev/null; then
        echo "Failed to create log directory: ${log_dir}" >&2
        return 1
    fi
    
    echo "${log_file}"
}

cleanup_old_logs() {
    local service_name=$1
    local keep_count=$2
    local log_dir="${LOG_BASE_DIR}/${service_name}"
    
    if [ -d "${log_dir}" ]; then
        # Find and delete old log files, keep recent last
        ls -t "${log_dir}"/${service_name}_*.log 2>/dev/null | tail -n +${keep_count} | xargs -r rm -f
    fi
}