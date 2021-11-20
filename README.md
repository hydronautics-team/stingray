# stingray
Autonomous underwater vehicle platform based on ROS2 Galactic

## Building
Install dependencies:
```bash
sudo apt-get install ros-galactic-desktop ros-galactic-usb-cam ros-galactic-serial-driver
```
Use following commands to build:
```bash
source /opt/ros/galactic/setup.bash
colcon build
```
To build specific packages use:
```bash
colcon build --packages-select 'package name'
```
Do not forget to setup workspace before you start to work:
```bash
source install/setup.bash
```