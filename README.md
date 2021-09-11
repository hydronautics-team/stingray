# stingray
Autonomous underwater vehicle platform based on ROS2 Galactic

## Building
Install dependencies:
```bash
sudo apt-get install ros-galactic-desktop
```
Initialize and update git submodules used in project:
```bash
git submodule init
git submodule update
```
Use following commands to build:
```bash
source /opt/ros/galactic/setup.bash
colcon build
```
Do not forget to setup workspace before you start to work:
```bash
source install/setup.bash
```