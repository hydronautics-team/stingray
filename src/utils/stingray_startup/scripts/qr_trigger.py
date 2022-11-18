#! /usr/bin/env python3

from typing import List
import rospkg
import rospy
from std_msgs.msg import String
import os
import json
from roslaunch import rlutil, parent, configure_logging
from glob import glob


class QrTrigger:
    """ Node to trigger launch files with qr codes
    """
    def __init__(self, launch_pkg_name: str, name_pattern: str):
        """
        Args:
            launch_pkg_name (str): package name with *launch/* directori which contains launch files you want to trigger
            name_pattern (str): specify this prefix to trigger specific launch files
        """
        rospy.loginfo('QrTrigger init')

        self.launch_dir_path = os.path.join(
            rospkg.RosPack().get_path(launch_pkg_name), "launch")
        self.name_pattern = name_pattern
        self.launch_names = self.get_launch_names(
            self.launch_dir_path, self.name_pattern)
        rospy.loginfo('launch names {}'.format(self.launch_names))

        # configs
        # TODO MOVE THIS TO UTILS OR RESOURSES
        stingray_resources_path = rospkg.RosPack().get_path("stingray_resources")
        with open(os.path.join(stingray_resources_path, "configs/ros.json")) as f:
            self.ros_config = json.load(f)

        # ROS subscribers
        rospy.Subscriber('/barcode', String, self.barcode_callback)

        self.detected_qr = None
        self.is_launched = False
        self.launched_file = None
        self.launched_name = None
        self.detected_but_not_launched = False

    def get_launch_names(self, path: str, pattern: str) -> List[str]:
        """ Explore directory and find files mathing the pattern

        Args:
            path (str): Directory path
            pattern (str): Name pattern

        Returns:
            List[str]: list of paths
        """
        matched_files = glob(os.path.join(path, pattern) + "*.launch")
        launch_names = []
        for match in matched_files:
            launch_names.append(os.path.split(
                match)[-1].split(pattern)[-1].split(".launch")[0])
        return launch_names

    def barcode_callback(self, msg: String):
        """ Callback when qr code was found

        Args:
            msg (String): decoded qr code
        """
        self.detected_qr = msg.data.lower()
        self.detected_but_not_launched = True
        rospy.loginfo('Detected {}'.format(self.detected_qr))

    def launch_detected(self):
        """ Trigger launch file after qr code has been detected
        
        If 'stop' has been detected then running launch file will be stopped 
        """
        if self.detected_qr is None:
            return
        if not self.detected_but_not_launched:
            return
        if self.is_launched:
            if self.detected_qr == "stop":
                self.launched_file.shutdown()
                self.is_launched = False
            elif self.detected_qr == self.launched_name:
                rospy.logwarn('{} already launched'.format(self.detected_qr))
            elif self.detected_qr in self.launch_names:
                rospy.logwarn('Stop {} first, then launch {}'.format(
                    self.launched_name, self.detected_qr))
        else:
            if self.detected_qr == "stop":
                rospy.logwarn('Nothing launched!')
            elif self.detected_qr in self.launch_names:
                launchfile_name = self.name_pattern + self.detected_qr + ".launch"
                launchfile_path = os.path.join(
                    self.launch_dir_path, launchfile_name)
                if os.path.isfile(launchfile_path):
                    uuid = rlutil.get_or_generate_uuid(None, False)
                    configure_logging(uuid)
                    self.launched_file = parent.ROSLaunchParent(
                        uuid, [launchfile_path])
                    self.launched_file.start()
                    self.is_launched = True
                    self.launched_name = self.detected_qr
                    rospy.loginfo('Launching {}'.format(launchfile_name))
                else:
                    rospy.logwarn("Launch file with name {} doesn't exist in {}!".format(
                        launchfile_name, self.launch_dir_path))
            else:
                rospy.logerr('Unknown qr code {}'.format(self.detected_qr))
        
        self.detected_but_not_launched = False

if __name__ == '__main__':
    rospy.init_node('qr_trigger')
    # parameters
    launch_pkg_name = rospy.get_param('~launch_pkg_name')
    name_pattern = rospy.get_param('~name_pattern')

    qr = QrTrigger(launch_pkg_name, name_pattern)

    try:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            qr.launch_detected()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("Shutting down {} node".format(rospy.get_name()))
