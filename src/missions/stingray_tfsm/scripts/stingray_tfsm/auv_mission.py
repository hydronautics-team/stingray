#! /usr/bin/env python3

from abc import abstractmethod
from stingray_tfsm.core.pure_mission import PureMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_object_detection_msgs.srv import SetEnableObjectDetection, ObjectsArray
from stingray_communication_msgs.srv import SetStabilization
from stingray_object_detection.utils import get_objects_topic
from stingray_resources.utils import load_config
import rospy


class AUVMission(PureMission):
    """ Abstract class to implement missions for AUV with useful methods """
    FSM_CLASS = AUVStateMachine

    @abstractmethod
    def __init__(self, name: str):
        """ Abstract class to implement missions for AUV with useful methods

        Args:
            name (str): mission name
        """
        self.ros_config = load_config("ros.json")
        super().__init__(name)

    def enable_object_detection(self, camera_topic: str, enable: bool = True):
        """ method to enable object detection for specific camera

        Args:
            camera_topic (str): camera topic name
        """
        srv_name = self.ros_config["services"]["set_enable_object_detection"]
        rospy.wait_for_service(srv_name)
        set_camera = rospy.ServiceProxy(srv_name, SetEnableObjectDetection)
        response = set_camera(camera_topic, enable)
        received = rospy.wait_for_message(
            get_objects_topic(camera_topic), ObjectsArray)
        rospy.loginfo(
            f"Object detection enabled: {response.success} for camera: {camera_topic} ")

    def enable_stabilization(self, depthStabilization: bool = False, yawStabilization: bool = False, lagStabilization: bool = False):
        """ method to enable object detection for specific camera

        Args:
            camera_topic (str): camera topic name
        """
        srv_name = self.ros_config["services"]["set_stabilization_enabled"]
        rospy.wait_for_service(srv_name)
        set_stabilization = rospy.ServiceProxy(srv_name, SetStabilization)
        response = set_stabilization(depthStabilization, yawStabilization, lagStabilization)
        rospy.loginfo(
            f"Stabilization enabled: {response.success}, message: {response.message} ")
