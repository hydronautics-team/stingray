#! /usr/bin/env python3

from stingray_tfsm.core.pure_mission import PureMission
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_object_detection_msgs.srv import SetEnableObjectDetection
from stingray_object_detection_msgs.msg import ObjectsArray
from stingray_object_detection.utils import get_objects_topic
from stingray_resources.utils import load_config
import rospy


class AUVMission(PureMission):
    """ Abstract class to implement missions for AUV with useful methods """
    FSM_CLASS = AUVStateMachine

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
        rospy.loginfo(f" Cam {camera_topic} enabled: {response}")
        rospy.wait_for_message(get_objects_topic(camera_topic), ObjectsArray)
