#!/bin/bash
set -e

source /stingray/install/setup.bash

exec "$@"
# exec ros2 launch stingray_launch zbar.launch.py
