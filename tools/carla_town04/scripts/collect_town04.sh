#!/bin/bash
# Start/reuse CARLA from ad_sim_dev_launch and run the StreamPETR collector in
# the existing carla-ros-bridge image.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

CHECK_ONLY=false
COLLECT_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --check) CHECK_ONLY=true; shift ;;
        *) COLLECT_ARGS+=("$1"); shift ;;
    esac
done

# CARLA's server container needs access to an X server even though the Python
# collector itself has no window. Adopt the user's local display when possible.
if [[ -z "${DISPLAY:-}" ]]; then
    for socket in /tmp/.X11-unix/X*; do
        [[ -S "$socket" && -O "$socket" ]] || continue
        export DISPLAY=":${socket##*/X}"
        if [[ -z "${XAUTHORITY:-}" && -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
            export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
        fi
        break
    done
fi

echo "════════════════════════════════════════════════"
echo "  StreamPETR CARLA Dataset — Town04"
echo "════════════════════════════════════════════════"

fix_podman_env
check_podman
check_images
if [[ -n "${DISPLAY:-}" ]]; then
    prepare_x11
fi

CARLA_CNAME=$(ensure_carla)
stop_conflicting_services
wait_for_carla "$CARLA_CNAME"

if $CHECK_ONLY; then
    echo "Preflight OK — the collector container can reach CARLA."
    exit 0
fi

NETFLAG=$(carla_netflag "$CARLA_CNAME")
RUN_COMMAND="${CARLA_PYTHONPATH_EXPORT}; exec python3 tools/carla_town04/collect.py \"\$@\""

# No TTY is required, which makes long collections safe to run under tmux/nohup.
# shellcheck disable=SC2086
podman run --rm -i ${NETFLAG} \
    --label "${TOOL_RUN_LABEL}" \
    -v "${STREAM_PETR_ROOT}:${TOOL_MOUNT}:rw" \
    -w "${TOOL_MOUNT}" \
    --entrypoint bash \
    "${TOOL_IMAGE}" \
    -c "${RUN_COMMAND}" bash "${COLLECT_ARGS[@]}"

echo ""
echo "Collection finished. Validate it with:"
echo "  python3 tools/carla_town04/validate_dataset.py <output-directory>"
