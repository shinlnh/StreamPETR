#!/bin/bash
set -e

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath "$SCRIPT_PATH/..")"

# Clear PC environment variables
unset PC_BUILD
unset LD_LIBRARY_PATH

# Setup cross-compilation environment (FSL SDK)
# Set GATEWAY_FSL_SDK_PATH environment variable to override default path
: ${GATEWAY_FSL_SDK_PATH:="/prj/es-automotive/gateway/common/sdk/v3.2.1"}
SDK_ENV_SCRIPT="$GATEWAY_FSL_SDK_PATH/environment-setup-aarch64-fsl-linux"

source "$SDK_ENV_SCRIPT" || {
    echo "Error: Failed to source SDK environment script at $SDK_ENV_SCRIPT"
    echo "Please set GATEWAY_FSL_SDK_PATH to your FSL SDK path"
    exit 1
}

# ---------------------------------------------------------------------------
# One-time fix: cmake export files installed into the FSL SDK sysroot may
# contain absolute paths pointing back into the Yocto build's recipe-sysroot,
# e.g.:
#   .../recipe-sysroot/usr/lib/libtinyxml2.so  (fastrtps-targets.cmake)
#   .../recipe-sysroot/usr/lib/libyaml.so      (rcl_yaml_param_parserExport.cmake)
#
# These paths only exist inside the Yocto build tree and cause linker errors
# like "No rule to make target '/path/.../recipe-sysroot/usr/lib/libtinyxml2.so'"
# when building the CAN service with this SDK on a developer machine.
#
# We replace every such path with the correct SDKTARGETSYSROOT path so cmake
# and the linker can find the libraries at their actual location in the SDK.
# A marker file prevents re-running the fix on every build invocation.
# ---------------------------------------------------------------------------
_FIX_MARKER="${SDK_PATH}/can_service_build/.cmake_abspaths_fixed"
if [ ! -f "$_FIX_MARKER" ]; then
    echo "==> [setenv_can] Fixing absolute recipe-sysroot paths in FSL SDK cmake files..."
    find "$SDKTARGETSYSROOT" -name "*.cmake" -type f -print0 2>/dev/null | \
    while IFS= read -r -d '' _cmake_file; do
        if grep -q "recipe-sysroot" "$_cmake_file" 2>/dev/null; then
            # Replace: /any/path/recipe-sysroot  ->  $SDKTARGETSYSROOT
            sed -i "s|/[^;\"' <>&]*recipe-sysroot|${SDKTARGETSYSROOT}|g" "$_cmake_file"
        fi
    done
    mkdir -p "$(dirname "$_FIX_MARKER")" && touch "$_FIX_MARKER"
    echo "==> [setenv_can] cmake fix complete. Marker: ${_FIX_MARKER}"
fi
unset _FIX_MARKER

# Set base_libdir for Python sysconfig (required by Yocto SDK)
export base_libdir="lib"

# ROS2 Humble environment
export ROS_DISTRO=humble
export ROS_LOCALHOST_ONLY=0
export ROS_PYTHON_VERSION=3
export ROS_VERSION=2

# Python and ROS paths (Python 3.8 in FSL SDK)
export PYTHON_EXECUTABLE="$OECORE_NATIVE_SYSROOT/usr/bin/python3"
export PYTHONPATH="$OECORE_NATIVE_SYSROOT/usr/lib/python3.8/site-packages:$OECORE_TARGET_SYSROOT/usr/lib/python3.8/site-packages"
export AMENT_PREFIX_PATH="$OECORE_TARGET_SYSROOT/usr"
export CMAKE_PREFIX_PATH="$OECORE_TARGET_SYSROOT/usr:$OECORE_TARGET_SYSROOT/usr/share:$OECORE_TARGET_SYSROOT/usr/lib/cmake"

# Add numpy include path for ROS2 message Python bindings
NUMPY_INCLUDE="$OECORE_NATIVE_SYSROOT/usr/lib/python3.8/site-packages/numpy/core/include"
if [ -d "$NUMPY_INCLUDE" ]; then
    export CFLAGS="$CFLAGS -I$NUMPY_INCLUDE"
    export CXXFLAGS="$CXXFLAGS -I$NUMPY_INCLUDE"
fi
