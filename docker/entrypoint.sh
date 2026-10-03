#!/usr/bin/env bash

set -eo pipefail

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

if [ -f /stingray/install/setup.bash ]; then
    echo "[INFO] Using existing workspace build."
    source /stingray/install/setup.bash
else
    echo "[INFO] Workspace build not found."
fi

exec "$@"
