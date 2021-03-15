# stingray
Autonomous underwater vehicle platform based on ROS

## Building
Install dependencies:

```bash
sudo apt-get install ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-image-view ros-$ROS_DISTRO-actionlib ros-$ROS_DISTRO-smach ros-$ROS_DISTRO-smach-viewer
```

Initialize and update git submodules used in project:

```bash
git submodule init
git submodule update
```

Use following commands to build:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make
```

Build debug:
```bash
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

Do not forget to setup workspace before you start to work:
```bash
source devel/setup.bash
```

## Cuda installation

[NVIDIA CUDA Installation Guide for Linux](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html)

## How to run tests

```bash

```
