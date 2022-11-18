from actionlib import SimpleActionClient, SimpleGoalState
from stingray_movement_msgs.msg import HorizontalMoveAction, DiveAction, HorizontalMoveGoal, DiveGoal
from stingray_resources.utils import load_config
from stingray_devices_msgs.msg import UpDownAction, UpDownGoal
import rospy


"""@package docstring
Contains class for controlling AUV in missions.
"""
# TODO rewrite it in c++ and make it an independent node
# TODO or reintegrate it with auv_fsm.py

def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" %
                  (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


class AUVControl:
    #  TODO make this a singleton to avoid multiple instances
    """Class for controlling AUV in missions.

    The class is actually a unified wrap over actions and services calls.
    """

    # TODO: Errors handling

    def __init__(self, verbose=False, *args, **kwargs):
        """ Class for controlling AUV in missions.

        The class is actually a unified wrap over actions and services calls.

        Args:
            verbose (bool, optional): More debug messages. Defaults to False.
        """
        self.verbose = verbose
        # configs
        self.ros_config = load_config("ros.json")
        self.control_config = load_config("control.json")

        self.HorizontalMoveClient = SimpleActionClient(
            self.ros_config["actions"]["movement"]["horizontal"], HorizontalMoveAction)
        self.DiveClient = SimpleActionClient(
            self.ros_config["actions"]["movement"]["dive"], DiveAction)
        self.DevicesClient = SimpleActionClient(
            self.ros_config["actions"]["updown"], UpDownAction)

        self.HorizontalMoveClient.wait_for_server()
        self.DiveClient.wait_for_server()
        self.DevicesClient.wait_for_server()

    def execute_lifter_goal(self, scene, *args, **kwargs):
        device_id = 1  # Lifter_id
        if 'lift' in scene:
            velocity = -110
            pause_common = 6
        elif 'lower' in scene:
            velocity = 110
            pause_common = 6
        else:
            velocity = 0
            pause_common = 1

        if 'wait' in scene:
            pause_optional = scene['wait']
        else:
            pause_optional = 0

        goal = UpDownGoal(device=device_id, velocity=velocity,
                          pause_common=pause_common, pause_optional=pause_optional)

        self.DevicesClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                     active_cb=callback_active)
        self.DevicesClient.wait_for_server()
        rospy.sleep(pause_common + pause_optional)

    def execute_dropper_goal(self, *args, velocity=100, **kwargs):
        device_id = 4  # dropper_id

        pause_common = 3
        pause_optional = 0

        goal = UpDownGoal(device=device_id, velocity=velocity,
                          pause_common=pause_common, pause_optional=pause_optional)

        self.DevicesClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                     active_cb=callback_active)
        self.DevicesClient.wait_for_server()
        rospy.sleep(pause_common)

    def execute_move_goal(self, scene):
        """
        The execute_move_goal function sends a goal to the HorizontalMoveClient action server.
        The goal is defined by the scene dictionary, which contains direction, velocity and duration.
        The function waits for the result of the action execution and returns it.

        :param self: Access the class attributes and methods
        :param scene: Pass the scene information to the execute_move_goal function
        :return: :

        """

        check_yaw = False
        if 'check_yaw' in scene:
            check_yaw = scene['check_yaw']


        if self.verbose:
            rospy.loginfo(f"Setting yaw delta: {scene['yaw']}")

        goal = HorizontalMoveGoal(
            scene['march'], scene['lag'], scene['yaw'], check_yaw)

        self.HorizontalMoveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                            active_cb=callback_active)
        if self.verbose:
            rospy.loginfo('Move goal sent')
        result = self.HorizontalMoveClient.wait_for_result(
            timeout=rospy.Duration(secs=5))
        if self.verbose:
            rospy.loginfo(f'Move result got {result}')
        if 'wait' in scene:
            rospy.loginfo(f"Wait for {scene['wait']} seconds ...")
            rospy.sleep(scene['wait'])

    def execute_dive_goal(self, scene):
        """
        The execute_dive_goal function sends a goal to the Dive action server.
        It waits for the action server to finish performing the task, then it returns whether or not
        the task was successful. The function also sets up callbacks that allow us to print feedback
        and determine when the task is complete.

        :param self: Access the variables and methods of the class
        :param scene: Pass the scene information to the dive action server
        :return: The result of the action

        """
        check_depth = False
        if 'check_depth' in scene:
            check_depth = scene['check_depth']

        goal = DiveGoal(scene['depth'], check_depth)
        self.DiveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                  active_cb=callback_active)
        if self.verbose:
            rospy.loginfo('Goal sent')
        self.DiveClient.wait_for_result(timeout=rospy.Duration(secs=5))
        if self.verbose:
            rospy.loginfo('Result got')

    def execute_stop_goal(self):
        self.execute_move_goal({
            'march': 0.0,
            'lag': 0.0,
            'yaw': 0,
        })
        rospy.loginfo('Everything stopped!')
