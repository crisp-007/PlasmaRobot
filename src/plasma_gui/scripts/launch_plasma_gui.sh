#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
PACKAGE_PREFIX="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_INSTALL="$(cd -- "${PACKAGE_PREFIX}/.." && pwd)"

source /opt/ros/galactic/setup.bash
source "${WORKSPACE_INSTALL}/setup.bash"

# The L515 ROS package was built against the local librealsense 2.50 runtime.
export LD_LIBRARY_PATH="/home/larusxu/.local/librealsense-v4l2-2.50.0/lib:${LD_LIBRARY_PATH:-}"

cd "$(cd -- "${WORKSPACE_INSTALL}/.." && pwd)"
exec "${SCRIPT_DIR}/plasma_gui" "$@"
