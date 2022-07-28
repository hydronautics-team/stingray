# stingray

Stingray is a framework for remote operated (ROV) and autonomous (AUV) underwater vehicles based on ROS

![Stingray logo](logo.jpg "Stingray logo")


# Installation

Initialize and update git submodules used in project:
```bash
git submodule update --init --recursive
```

## Pre-build steps
### Object detection steps

- Install **requirements** from [yolov5](https://github.com/ultralytics/yolov5)

If you want to run your own trained yolov5:
- Edit [config.yaml](src/vision/stingray_object_detection/weights/config.yaml) to add your labels
- Put best checkpoint of yolov5 as **best.pt** in [weights folder](src/vision/stingray_object_detection/weights)

### Common steps

Install dependencies:

```bash
$ROS_DISTRO=noetic
sudo apt-get install ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-image-view ros-$ROS_DISTRO-actionlib ros-$ROS_DISTRO-smach ros-$ROS_DISTRO-smach-viewer
sudo apt install graphviz-dev
pip3 install pygraphviz transitions
```

## Build
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make
```

# Run

Setup workspace before you start to work:

```bash
source devel/setup.bash
```

### Run object detection
- Run detection on real cameras:
```bash
roslaunch stingray_startup object_detection.launch real_cam:=true 
```
- Run detection on simulation cameras:
```bash
roslaunch stingray_startup object_detection.launch simulation:=true
```
- Run detection on video from files:
```bash
roslaunch stingray_startup object_detection.launch file:=true file1_path:=PATH_TO_VIDEO_1 file2_path:=PATH_TO_VIDEO_2
```