#!/bin/bash
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
SDK_PATH="$(realpath $SCRIPT_PATH/..)"

cd $SDK_PATH

rm -rf \
  pc_build \
  build \
  dds_build \
  can_build \
  can_service_build \
  log