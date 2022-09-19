#! /usr/bin/env python3

from abc import abstractmethod
import rospy
from stingray_tfsm.core.pure_mission import PureMission
from stingray_tfsm.core.pure_events import PureEvent
from stingray_tfsm.auv_fsm import AUVStateMachine
from stingray_object_detection_msgs.srv import SetEnableObjectDetection
from stingray_object_detection_msgs.msg import ObjectsArray
from stingray_communication_msgs.srv import SetStabilization
from std_msgs.msg import Bool
from stingray_object_detection.utils import get_objects_topic
from stingray_resources.utils import load_config


class AUVMission(PureMission):
    """ Abstract class to implement missions for AUV with useful methods """
    FSM_CLASS = AUVStateMachine

    @abstractmethod
    def __init__(self, name: str, simulation: bool = True, *args, **kwargs):
        """ Abstract class to implement missions for AUV with useful methods

        Args:
            name (str): mission name
        """
        self.ros_config = load_config("ros.json")
        self.simulation = simulation
        self.machine = AUVStateMachine(name, simulation=simulation)
        super().__init__(name, self.machine)

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

    def enable_stabilization(self, depthStabilization: bool = False, pitchStabilization: bool = False, yawStabilization: bool = False, lagStabilization: bool = False):
        """ method to enable object detection for specific camera

        Args:
        camera_topic (str): camera topic name
        """
        srv_name = self.ros_config["services"]["set_stabilization_enabled"]
        rospy.wait_for_service(srv_name)
        set_stabilization = rospy.ServiceProxy(srv_name, SetStabilization)
        response = set_stabilization(
            False, pitchStabilization, yawStabilization, lagStabilization)
        rospy.loginfo(
            f"Stabilization enabled: {response.success}, message: {response.message} ")


    def enable_reset_imu(self):
        """
        method to enable object detection for specific camera
        Args:
        camera_topic (str): camera topic name
        """
        if not self.simulation:
            srv_name = self.ros_config["services"]["set_imu_enabled"]
            rospy.wait_for_service(srv_name)
            set_imu_enabled = rospy.ServiceProxy(srv_name, Bool)
            response = set_imu_enabled(True)
            rospy.sleep(1)
            rospy.loginfo(
                f"IMU reset: {response.success}, message: {response.message} ")
        else:
            rospy.loginfo("no imu reset needed in simulator")

    def check_machine(self):
        if type(self.machine) is AUVStateMachine:
            return 1
        else:
            raise TypeError("AUVStateMachine was not initialized")

    @staticmethod
    def event_handler(event: PureEvent, wait: float = 0.5, *args, **kwargs):
        """
        The event_handler function is a function that is called when the event is triggered.
        It returns True or False depending on whether the event was triggered or not.

        :param wait:
        :param event: Pass the event that is being handled
        :return: True if the event is triggered and false if it is not

        """
        if wait is None:
            wait = 0
        event.start_listening()
        rospy.sleep(wait)
        value = event.is_triggered()
        event.stop_listening()
        return value
