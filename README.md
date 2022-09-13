# stingray

Stingray is a ROS based framework for autonomous (AUV) and remote operated (ROV) underwater vehicles

<div align="center">
    <img src="logo.jpg" alt="Stingray logo" width="600"/>
</div>

## Dependencies
- [ROS Noetic](https://wiki.ros.org/noetic)
- [YOLOv5](https://github.com/ultralytics/yolov5)


# Install

- Initialize and update git submodules used in project:
```bash
git submodule update --init --recursive
```

- Install **requirements** from [yolov5](https://github.com/ultralytics/yolov5)


- Install ros packages:

```bash
$ROS_DISTRO=noetic
```

```bash
sudo apt-get install ros-$ROS_DISTRO-serial ros-$ROS_DISTRO-usb-cam ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-image-view ros-$ROS_DISTRO-actionlib ros-$ROS_DISTRO-zbar-ros
```
- Install other dependencies
```bash
sudo apt install graphviz-dev
pip3 install pygraphviz transitions
```

- Build
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make
```

# Run

### Setup workspace before you start to work:

```bash
source devel/setup.bash
roslaunch stingray_startup main.launch simulation:=true
```

## Basic launch file:

```bash
roslaunch stingray_startup base.launch 
```
*see args [inside launch file](src/utils/stingray_startup/launch/base.launch) or [below](#other-args)

### Run with simulator

* Clone and build our [simulator](https://github.com/hidronautics/simulator) (now it's only for sauvc competition).
* Run simulator.
* in stingray directory run

And use arg:
```bash
simulation:=true 
```

### Run with qr codes

[➦ Full description here](#launch-with-qrtrigger-node)

Use arg:
```bash
qr_launch:=true 
```
Show your qr code to vehicle camera. 

If `stop` qr code has been detected then the running launch file will be stopped.

### Other args:
- `hardware_connection:=false` - disable connection btw jetson and stm32 via serial (uart_driver) 
- `stream:=true` - enable web video stream from all cameras 
- `debug:=false` - disable image_view nodes and publishing output videos after object detection
- `file_cam:=true` - provide input videos from file
- `record_raw:=true` - enable recording video from all cameras 
- `record_output:=true` - enable recording video after object detection 

## Vision launch file with object detection

- Run detection on real cameras:
```bash
roslaunch stingray_startup vision.launch real_cam:=true 
```
- Run detection on simulation cameras:
```bash
roslaunch stingray_startup vision.launch simulation:=true
```
- Run detection on video from files:
```bash
roslaunch stingray_startup vision.launch file:=true file1_path:=PATH_TO_VIDEO_1 file2_path:=PATH_TO_VIDEO_2
```

### If you want to run your own trained yolov5:
- Edit [config.yaml](src/vision/stingray_object_detection/weights/config.yaml) to add your labels
- Put best checkpoint of yolov5 as **best.pt** in [weights folder](src/vision/stingray_object_detection/weights)


# Packages

## stingray_communication
TODO: description

## stingray_gazebo_communication
TODO: description

## stingray_devices
TODO: description

## stingray_tfsm
TODO: description

## stingray_movement
TODO: description

## stingray_resources

Contains config files and util libs

Config files:
- `control.json` - all stuff for control algos and control system
- `hardware.json` - all stuff for different devices, hardware communication and etc.
- `ros.json` - names for ROS topics, services, actions
- `simulation.json`- all stuff for simulation mode

Libs:
- `json.hpp` - cpp lib for reading json files
- `utils.py` - util python methods
    - `load_config` - loads config file with name

## stingray_startup

Contains launch files and qr trigger node.

Launch files:
- `base.launch`
- `vision.launch`

### Launch with qr_trigger node

Qr trigger node has parameters:
- `launch_pkg_name` - package name with *launch/* directori which contains launch files you want to trigger  
- `name_pattern` - specify this prefix to trigger specific launch files

Generate qr code from launch file name without custom prefix and *.launch*. 

> Example: you have `stingray_qr_mission.launch` file. **stingray_qr_** is the prefix which you pass to qr_trigger node as `name_pattern` parameter. Also you don't need **.launch** to generate qr code. Eventually you need to generate `mission` to qr code

Launch `base.launch` with param:
```bash
qr_launch:=true 
```
and you'll able to trigger launch files with qr codes.

If `stop` has been detected then running launch file will be stopped.

## stingray_video_recorder
TODO: description
