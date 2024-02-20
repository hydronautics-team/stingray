from transitions.extensions.asyncio import AsyncMachine
import asyncio
import json
from pathlib import Path
import yaml
import logging
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class EulerAngles:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Robot:
    def __init__(self, node: Node):
        """Robot class for executing underwater vehicle commands:
        Services:
        - set_velocity
        - set_pose
        - set_stabilization
        - set_device
        Actions:
        - move_to
        - explore
        - search
        - center
        - follow
        """
        self.node = node

        self.set_velocity_service = node.create_client(Twist, "set_velocity")
        self.set_pose_service = node.create_client(SetPose, "set_pose")
        self.set_stabilization_service = node.create_client(
            SetStabilization, "set_stabilization")
        self.set_device_service = node.create_client(SetDevice, "set_device")

    def set_velocity(self, linear: float, angular: float):
        """Set the velocity of the underwater vehicle"""
        req = Twist()
        req.linear.x = linear
        req.angular.z = angular
        self.set_velocity_service.call_async(req)

