#!/usr/bin/env bash
set -eo pipefail

cd /stingray

if ! grep -Fq "source /opt/ros/humble/setup.bash" /root/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
fi

if ! grep -Fq "/stingray/install/setup.bash" /root/.bashrc; then
    echo "[ -f /stingray/install/setup.bash ] && source /stingray/install/setup.bash" >> /root/.bashrc
fi

source /opt/ros/humble/setup.bash

echo "[INFO] Installing ROS dependencies..."

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y

echo "[INFO] Development container is ready."
echo "[INFO] Build with: ./build.sh"
