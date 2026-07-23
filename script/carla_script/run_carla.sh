#!/bin/bash
set -e
SCRIPT_PLACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_PLACE_DIR}/setup_default_carla_root.sh

CARLA_SH_PLACE=$CARLA_ROOT/CarlaUE4.sh
GRAPHIC_LEVEL_DEFAULT="Epic"
RENDER_OFFSCENE_DEFAULT=""

GRAPHIC_LEVEL=${GRAPHIC_LEVEL_DEFAULT}
RENDER_OFFSCENE=${RENDER_OFFSCENE_DEFAULT}

GRAPHIC_FLAG="E"
RENDER_FLAG="ON"
CARLA_ROOT_FLAG=$CARLA_ROOT

while getopts ":g:r:d:" option; do
  case $option in
    g)
      GRAPHIC_FLAG="$OPTARG"
      ;;
    r)
      RENDER_FLAG="$OPTARG"
      ;;
    d)
      CARLA_ROOT_FLAG="$OPTARG"
      ;;
    *)
      echo "Usage: $0 [-g <graphic_level>(L,E)] [-r <render_mode>(ON, OFF)]  [-d <carla_roor_dir>(default, <path>)]"
      exit 1
      ;;
  esac
done

if [[ "$GRAPHIC_FLAG" == "L" ]] || [[ "$GRAPHIC_FLAG" == "Low" ]]
then
    GRAPHIC_LEVEL="Low"
elif [[ "$GRAPHIC_FLAG" == "E" ]] || [[ "$GRAPHIC_FLAG" == "Epic" ]]
then
    GRAPHIC_LEVEL="Epic"
else
    echo "$GRAPHIC_FLAG is not a graphic level, change to default ${GRAPHIC_LEVEL_DEFAULT} level"
    GRAPHIC_LEVEL=${GRAPHIC_LEVEL_DEFAULT}
fi

if [[ "$RENDER_FLAG" == "OFF" ]]
then
    RENDER_OFFSCENE="-RenderOffScreen"
elif [[ "$RENDER_FLAG" == "Normal" ]] || [[ "$RENDER_FLAG" == "ON" ]]
then
    RENDER_OFFSCENE=""
else
    echo "$RENDER_FLAG is not a screen render option, change to default mode(render screen)"
    RENDER_OFFSCENE=""
fi

if [[ "$CARLA_ROOT_FLAG" == "default" ]]
then
    echo "Using CARLA at NAS directory"
    CARLA_SH_PLACE=$CARLA_ROOT/CarlaUE4.sh
else
    CARLA_SH_PLACE=$CARLA_ROOT_FLAG/CarlaUE4.sh
fi

echo "To reproduce the script, run this command: ${CARLA_SH_PLACE} ${RENDER_OFFSCENE} -quality-level=${GRAPHIC_LEVEL} -prefernvidia"

/bin/bash ${CARLA_SH_PLACE} ${RENDER_OFFSCENE} -quality-level=${GRAPHIC_LEVEL} -prefernvidia