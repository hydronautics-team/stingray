from actionlib import SimpleActionClient, SimpleGoalState
from stingray_movement_msgs.msg import HorizontalMoveAction, DiveAction, HorizontalMoveGoal, DiveGoal
from std_msgs.msg import Int32
from stingray_resources.utils import load_config
from stingray_devices_msgs.msg import UpDownAction, UpDownGoal
import rospy


"""@package docstring
Contains class for controlling AUV in missions.
"""


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" %
                  (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


class AUVControl:
    """Class for controlling AUV in missions.

    The class is actually a unified wrap over actions and services calls.
    """

    # TODO: Errors handling
    # TODO: Devices support

    def __init__(self, verbose=False, multiplier=1):
        """ Class for controlling AUV in missions.

        The class is actually a unified wrap over actions and services calls.

        Args:
            verbose (bool, optional): More debug messages. Defaults to False.
        """
        self.verbose = verbose
        self.mult = multiplier
        self.yaw_angle = 0
        self.inited = False
        # configs
        self.ros_config = load_config("ros.json")
        self.control_config = load_config("control.json")

        self.yaw_subscriber = rospy.Subscriber(self.ros_config["topics"]["yaw"],
                                               Int32,
                                               callback=self.yaw_topic_callback,
                                               queue_size=10)

        self.HorizontalMoveClient = SimpleActionClient(
            self.ros_config["actions"]["movement"]["horizontal"], HorizontalMoveAction)
        self.DiveClient = SimpleActionClient(self.ros_config["actions"]["movement"]["dive"], DiveAction)
        self.DevicesClient = SimpleActionClient(self.ros_config["actions"]["updown"], UpDownAction)

        self.HorizontalMoveClient.wait_for_server()
        self.DiveClient.wait_for_server()
        self.DevicesClient.wait_for_server()

    @property
    def yaw(self):
        return self.yaw_angle

    def yaw_topic_callback(self, msg):
        """
        The yaw_topic_callback function is a callback function that is called every time the yaw_topic publishes a message.
        The yaw_topic is the topic that publishes the current angle of rotation of the drone.
        This function sets self.yaw to this value, which will be used in other functions.

        :param self: Access the variables and methods inside a class
        :param msg: Store the data from the topic
        :return: None

        """
        if not self.inited:
            self.yaw_angle = msg.data
            rospy.loginfo(f"self.yaw_angle: {self.yaw_angle}")
        if self.verbose:
            rospy.loginfo(
                f"Absolute angle got from machine is {msg.data}; It is set on higher level")

    def execute_lifter_goal(self, scene):
        device_id = 1  # Lifter_id
        if 'lift' in scene:
            velocity = 1
        elif 'lower' in scene:
            velocity = -1
        else:
            velocity = 0

        pause_common = 3
        if 'wait' in scene:
            pause_optional = scene['wait']
        else:
            pause_optional = 0

        goal = UpDownGoal(device=device_id, velocity=velocity, pause_common=pause_common, pause_optional=pause_optional)

        self.DevicesClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                     active_cb=callback_active)
        self.DevicesClient.wait_for_server()
        rospy.sleep(pause_common + pause_optional)

    def execute_dropper_goal(self, *args, **kwargs):
        device_id = 2  # Lifter_id
        velocity = 0

        pause_common = 1
        pause_optional = 0

        goal = UpDownGoal(device=device_id, velocity=velocity, pause_common=pause_common, pause_optional=pause_optional)

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
        if 'wait' in scene:
            if self.mult != 1:
                actual_duration = scene['wait'] * self.mult
            else:
                actual_duration = scene['wait']

        check_yaw = False
        if 'check_yaw' in scene:
            check_yaw = scene['check_yaw']

        rospy.loginfo(f"self.yaw: {self.yaw}")
        rospy.loginfo(f"self.yaw + scene['yaw']: {self.yaw + scene['yaw']}")
        goal = HorizontalMoveGoal(scene['march'], scene['lag'], self.yaw + scene['yaw'], check_yaw)

        self.HorizontalMoveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                            active_cb=callback_active)
        if self.verbose:
            rospy.loginfo('Move goal sent')
        result = self.HorizontalMoveClient.wait_for_result(timeout=rospy.Duration(secs=5))
        if self.verbose:
            rospy.loginfo(f'Move result got {result}')
        if 'wait' in scene:
            rospy.loginfo(f"Wait for {actual_duration} seconds ...")
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
        self.execute_dive_goal({
            'depth': 0,
        })
        rospy.loginfo('Everything stopped!')
