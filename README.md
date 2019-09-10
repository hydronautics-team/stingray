# stingray
Autonomous underwater vehicle platform based on ROS

## Building
Install dependencies:
```bash
sudo apt-get install ros-melodic-rosbridge-server ros-melodic-web-video-server ros-melodic-image-view ros-melodic-actionlib ros-melodic-serial ros-melodic-smach ros-melodic-smach-viewer
```
Initialize and update git submodules used in project:
```bash
git submodule init
git submodule update
```
Use following commands to build:
```bash
source /opt/ros/melodic/setup.bash
catkin_make
```
Do not forget to setup workspace before you start to work:
```bash
source devel/setup.bash
```