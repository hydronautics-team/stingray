#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

sleep 0.5
rostopic pub /stingray_action_linear_movement/goal stingray_movement_msgs/LinearMoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  direction: 1
  velocity: 0.4
  duration: 4000"

sleep 4.5

rostopic pub /stingray_action_linear_movement/goal stingray_movement_msgs/LinerMoveActionGoal "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  goal_id:
    stamp:
      secs: 0
      nsecs: 0
    id: ''
  goal:
    direction: 0
    velocity: 0.0
    duration: 1"

sleep 0.1

rostopic pub /stingray_action_rotate/goal stingray_movement_msgs/RotateActionGoal "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  goal_id:
    stamp:
      secs: 0
      nsecs: 0
    id: ''
  goal:
    yaw: 60"

sleep 1

rostopic pub /stingray_action_linear_movement/goal stingray_movement_msgs/LinerMoveActionGoal "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  goal_id:
    stamp:
      secs: 0
      nsecs: 0
    id: ''
  goal:
    direction: 1
    velocity: 0.4
    duration: 4000"

sleep 4.5

rostopic pub /stingray_action_linear_movement/goal stingray_movement_msgs/LinerMoveActionGoal "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  goal_id:
    stamp:
      secs: 0
      nsecs: 0
    id: ''
  goal:
    direction: 0
    velocity: 0.0
    duration: 1"

