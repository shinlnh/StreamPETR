#!/bin/bash
SCRIPT_PLACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_PLACE_DIR}/setup_default_carla_root.sh

CARLA_ROOT_DIRECTORY=$CARLA_ROOT
while getopts ":d:" option; do
  case $option in
    d)
      CARLA_ROOT_FLAG="$OPTARG"
      ;;
    *)
      echo "Usage: $0 [-d <carla_roor_dir>(default, <path>)]"
      exit 1
      ;;
  esac
done

if [[ "$CARLA_ROOT_FLAG" == "default" ]] || [[ "$CARLA_ROOT_FLAG" == "" ]]
then
    echo "Using CARLA at NAS directory"
    CARLA_ROOT=$CARLA_ROOT
else
    CARLA_ROOT=$CARLA_ROOT_FLAG
fi

echo "CARLA_ROOT is set to $CARLA_ROOT"

export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
source  /opt/ros/foxy/setup.bash
source  $CARLA_ROOT/carla-ros-bridge-2/install/setup.bash