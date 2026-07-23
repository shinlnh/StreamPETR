#!/bin/bash
# Shared helpers for the CARLA StreamPETR collection tool.
#
# Copied from ad_sim_dev_launch/scripts/carla/collect_dataset/scripts/lib.sh
# and adapted so the collector can live in the StreamPETR repository while
# continuing to reuse ad_sim_dev_launch's CARLA compose service.
#
# Containers managed here:
#   carla   compose service (CARLA server), started on demand from the
#           umbrella compose.yml — same project name as scripts/host/run.sh,
#           so the dashboard and this tool see the same containers.
#   tool    ephemeral `podman run --rm` containers from the carla-ros-bridge
#           image. That image already ships python3 + pygame + opencv + the
#           CARLA PythonAPI egg, so nothing needs to be installed on the host.
#
# Shared image store: images are served from the read-only
# additionalimagestores path configured once in
# ~/.config/containers/storage.conf by scripts/host/setup_member.sh.
# Nothing has to be sourced per shell for podman to see them. The one
# known pitfall is snap-packaged terminals (e.g. VSCode from snap), which
# export XDG_DATA_HOME/XDG_CONFIG_HOME into the snap sandbox and make
# podman read a different store/config — fix_podman_env() undoes that.

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOL_DIR="$(cd "${LIB_DIR}/.." && pwd)"
STREAM_PETR_ROOT="$(cd "${TOOL_DIR}/../.." && pwd)"
DEFAULT_AD_SIM_DEV_LAUNCH_DIR="$(cd "${STREAM_PETR_ROOT}/.." && pwd)/ad_sim_dev_launch"
AD_SIM_DEV_LAUNCH_DIR="${AD_SIM_DEV_LAUNCH_DIR:-${DEFAULT_AD_SIM_DEV_LAUNCH_DIR}}"

TOOL_IMAGE="carla-ros-bridge:${ROS_DISTRO:-foxy}"
CARLA_IMAGE="carlasim/carla:${CARLA_VERSION:-0.9.13}"
TOOL_MOUNT="/workspace/StreamPETR"
TOOL_RUN_LABEL="streampetr.carla_town04=tool"

# Same per-user compose project as scripts/host/run.sh.
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-$USER}"

# Resolved inside the tool container at runtime (single quotes intentional)
# so the egg version never has to be hardcoded host-side.
CARLA_PYTHONPATH_EXPORT='export PYTHONPATH="$(ls /opt/carla/PythonAPI/carla/dist/carla-*py3*linux-x86_64.egg 2>/dev/null | head -1):/opt/carla/PythonAPI/carla"'

# ── Environment guards ──────────────────────────────────────────────────────

fix_podman_env() {
    if [[ "${XDG_DATA_HOME:-}" == *"/snap/"* ]]; then
        echo "  NOTE: unsetting XDG_DATA_HOME (snap sandbox) so podman uses your normal store."
        unset XDG_DATA_HOME
    fi
    if [[ "${XDG_CONFIG_HOME:-}" == *"/snap/"* ]]; then
        echo "  NOTE: unsetting XDG_CONFIG_HOME (snap sandbox) so podman reads ~/.config/containers."
        unset XDG_CONFIG_HOME
    fi
}

check_podman() {
    if ! command -v podman &>/dev/null; then
        echo "ERROR: podman not found. Run scripts/host/setup_member.sh first." >&2
        return 1
    fi
    if ! command -v podman-compose &>/dev/null; then
        echo "ERROR: podman-compose not found. Run scripts/host/setup_member.sh first." >&2
        return 1
    fi
    if [[ ! -f "${AD_SIM_DEV_LAUNCH_DIR}/compose.yml" ]]; then
        echo "ERROR: ad_sim_dev_launch compose.yml not found at:" >&2
        echo "  ${AD_SIM_DEV_LAUNCH_DIR}/compose.yml" >&2
        echo "Set AD_SIM_DEV_LAUNCH_DIR to the correct checkout." >&2
        return 1
    fi
}

_image_present() {
    podman image exists "$1" 2>/dev/null || podman image exists "docker.io/$1" 2>/dev/null
}

check_images() {
    local missing=false
    for img in "${TOOL_IMAGE}" "${CARLA_IMAGE}"; do
        if _image_present "$img"; then
            echo "  Image OK : $img"
        else
            echo "  MISSING  : $img"
            missing=true
        fi
    done
    if $missing; then
        echo "" >&2
        echo "ERROR: required images not visible to podman." >&2
        echo "  They should be served from the shared store configured in" >&2
        echo "  ~/.config/containers/storage.conf (additionalimagestores)." >&2
        echo "  Run scripts/host/setup_member.sh, or contact the Lead." >&2
        echo "  If you are inside a snap VSCode terminal, retry from a normal terminal." >&2
        return 1
    fi
}

# ── Compose service helpers (same conventions as scripts/host/run.sh) ───────

svc_cname() {
    podman ps -a \
        --filter "label=io.podman.compose.project=${COMPOSE_PROJECT_NAME}" \
        --filter "label=io.podman.compose.service=$1" \
        --format "{{.Names}}" 2>/dev/null | head -1
}

svc_running() {
    local cname
    cname=$(svc_cname "$1")
    [[ -z "$cname" ]] && return 1
    podman ps --filter "name=^${cname}$" --filter "status=running" --format "{{.Names}}" 2>/dev/null \
        | grep -q "^${cname}$"
}

# ── X11 (mirrors _prepare_x11 in scripts/host/run.sh) ───────────────────────

prepare_x11() {
    xhost +local: >/dev/null 2>&1 || true
    XAUTH_FILE="/tmp/.docker.xauth.${USER}"
    if [ -d "$XAUTH_FILE" ]; then
        rm -rf "$XAUTH_FILE"
    fi
    if [ ! -f "$XAUTH_FILE" ]; then
        touch "$XAUTH_FILE"
        xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge - 2>/dev/null || true
        chmod 644 "$XAUTH_FILE"
    fi
    export XAUTH_FILE
}

# ── CARLA server lifecycle ──────────────────────────────────────────────────

# Start the carla compose service if it is not already running.
# Prints the container name on stdout (logs go to stderr).
ensure_carla() {
    local cname
    cname=$(svc_cname carla)
    if [[ -n "$cname" ]] && svc_running carla; then
        echo "  CARLA container already running: ${cname}" >&2
    else
        echo "  Starting CARLA server container (${CARLA_IMAGE})..." >&2
        podman-compose -p "${COMPOSE_PROJECT_NAME}" -f "${AD_SIM_DEV_LAUNCH_DIR}/compose.yml" up -d carla >&2
        cname=$(svc_cname carla)
        if [[ -z "$cname" ]] || ! svc_running carla; then
            echo "ERROR: CARLA container failed to start. Check:" >&2
            echo "  podman ps -a && podman logs ${cname:-<carla container>}" >&2
            return 1
        fi
    fi
    printf '%s' "$cname"
}

# The collector loads its own world and drives CARLA in synchronous mode,
# which conflicts with the ROS bridge (also a sync master, restart=on-failure)
# and anything consuming it. Stop them for the duration of the collection.
stop_conflicting_services() {
    local stopped=""
    for svc in carla_ros_adas_bridge adas_service hmi; do
        if svc_running "$svc"; then
            echo "  Stopping ${svc} (conflicts with the collector's sync mode)..."
            podman stop "$(svc_cname "$svc")" >/dev/null
            stopped+=" ${svc}"
        fi
    done
    if [[ -n "$stopped" ]]; then
        echo "  Stopped:${stopped}"
        echo "  Restart them later from the dashboard: ./scripts/host/run.sh"
    fi
}

# Network flag for tool containers: join the carla container's network
# namespace so localhost:2000 resolves to the server. Handles all three
# layouts: host networking (board overlay), podman-compose pod (default),
# and a bare container.
carla_netflag() {
    local mode pod
    mode=$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$1" 2>/dev/null)
    if [[ "$mode" == "host" ]]; then
        printf -- '--network host'
        return
    fi
    pod=$(podman inspect -f '{{.Pod}}' "$1" 2>/dev/null)
    if [[ -n "$pod" && "$pod" != "<no value>" ]]; then
        # Joining the netns of a pod member requires joining the pod too
        # (this is also how podman-compose realizes network_mode: service:carla).
        printf -- '--pod %s --network container:%s' "$pod" "$1"
    else
        printf -- '--network container:%s' "$1"
    fi
}

# Block until the CARLA server accepts connections on port 2000 (or time out).
wait_for_carla() {
    local cname="$1" netflag
    netflag=$(carla_netflag "$cname")
    echo "  Waiting for CARLA server (port 2000, up to 180s)..."
    # shellcheck disable=SC2086
    podman run --rm -i ${netflag} --label "${TOOL_RUN_LABEL}" --entrypoint python3 "${TOOL_IMAGE}" - <<'PY'
import socket, sys, time
deadline = time.time() + 180
while time.time() < deadline:
    try:
        socket.create_connection(("localhost", 2000), 3).close()
        print("  CARLA is up.")
        sys.exit(0)
    except OSError:
        time.sleep(2)
print("ERROR: CARLA did not open port 2000 within 180s.", file=sys.stderr)
sys.exit(1)
PY
}

# ── Offline pipeline steps (no CARLA / X11 needed) ──────────────────────────

# Run one of the offline python steps (dataset_generator.py,
# image_to_video.py) inside an ephemeral tool container with the tool dir
# bind-mounted, so outputs land in this directory on the host.
run_offline_tool() {
    podman run --rm -i \
        --label "${TOOL_RUN_LABEL}" \
        -v "${STREAM_PETR_ROOT}:${TOOL_MOUNT}:rw" \
        -w "${TOOL_MOUNT}" \
        --entrypoint bash \
        "${TOOL_IMAGE}" \
        -c "python3 $*"
}
