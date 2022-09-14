import rospy
from actionlib import SimpleActionClient, SimpleGoalState
from stingray_movement_msgs.msg import LinearMoveAction, RotateAction, DiveAction, LinearMoveGoal, RotateGoal, DiveGoal
from std_msgs.msg import Int32
from stingray_resources.utils import load_config
from stingray_tfsm.core.pure_events import PureEvent

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
        # configs
        self.ros_config = load_config("ros.json")
        self.control_config = load_config("control.json")

        self.yaw_subscriber = rospy.Subscriber(self.ros_config["topics"]["yaw"],
                                               Int32,
                                               callback=self.yaw_topic_callback,
                                               queue_size=10)

        self.LinearMoveClient = SimpleActionClient(self.ros_config["actions"]["movement"]["linear"],
                                                   LinearMoveAction)
        self.RotateClient = SimpleActionClient(self.ros_config["actions"]["movement"]["rotate"],
                                               RotateAction)
        self.DiveClient = SimpleActionClient(self.ros_config["actions"]["movement"]["dive"],
                                             DiveAction)
        self.LinearMoveClient.wait_for_server()
        self.RotateClient.wait_for_server()
        self.DiveClient.wait_for_server()

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
        self.yaw_angle = msg.data
        if self.verbose:
            rospy.loginfo(
                f"Absolute angle got from machine is {msg.data}; It is set on higher level")

    def execute_move_goal(self, scene):
        """
        The execute_move_goal function sends a goal to the LinearMoveClient action server.
        The goal is defined by the scene dictionary, which contains direction, velocity and duration.
        The function waits for the result of the action execution and returns it.

        :param self: Access the class attributes and methods
        :param scene: Pass the scene information to the execute_move_goal function
        :return: :

        """
        if self.mult != 1:
            actual_duration = scene['duration']*self.mult
        else:
            actual_duration = scene['duration']
        goal = LinearMoveGoal(
            scene['direction'], scene['velocity'], actual_duration)
        self.LinearMoveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)
        if self.verbose:
            rospy.loginfo('Goal sent')
        self.LinearMoveClient.wait_for_result(
            timeout=rospy.Duration(secs=self.mult // 1000 + 1))
        
        if self.verbose:
            rospy.loginfo('Result got')

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
        goal = DiveGoal(scene['depth'])
        self.DiveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                  active_cb=callback_active)
        if self.verbose:
            rospy.loginfo('Goal sent')
        self.DiveClient.wait_for_result(timeout=rospy.Duration(secs=5))
        if self.verbose:
            rospy.loginfo('Result got')

    def execute_rotate_goal(self, scene):
        """
        The execute_rotate_goal function rotates the robot by a specified angle.
        :param self: Access the attributes and methods of the class in python
        :param scene: Pass the scene information to the rotate goal function
        :return: The result of the action server

        """
        angle = scene['angle']
        self.new_yaw = self.yaw + angle

        if self.new_yaw > 360:
            self.new_yaw -= 360
        elif self.new_yaw < -360:
            self.new_yaw += 360
        if self.verbose:
            rospy.loginfo("Absolute angle is {} now".format(
                self.new_yaw))

        if angle != 0:
            goal = RotateGoal(yaw=self.new_yaw)
            self.RotateClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)

            if self.mult == 1:
                rospy.sleep(0.1)
            else:
                rospy.sleep(angle//3/10 + 0.5)
            if self.verbose:
                rospy.loginfo('Goal sent')
            self.RotateClient.wait_for_result(timeout=rospy.Duration(secs=5))
            if self.verbose:
                rospy.loginfo('Result got')
        else:
            rospy.loginfo('Rotating is not required to achieve this angle')

    def rotate(self, yaw_deg: float):
        """Rotates the vehicle. Blocking call.

        :param yaw_deg: Angle to rotate in degrees, positive is for counterclockwise, negative - for clockwise.
        """

        goal = self._create_rotate_goal(yaw_deg=yaw_deg)
        self._exec_action(self.RotateClient, goal)

    def march(self, duration_ms: int, velocity: float, stopping_event: PureEvent = None):
        """Moves the vehicle by march. Blocking call. Action execution will be canceled if stopping_event is triggered.

        :param duration_ms: Duration of movement in milliseconds.
        :param velocity:  Velocity of movement, from 0.0 to 1.0, positive to move forward, negative - to backward movement.
        :param stopping_event: Event that stops movement.
        """

        goal = self._create_march_goal(duration_ms, velocity)
        return self._exec_action(self.LinearMoveClient, goal, stopping_event)

    def lag(self, duration_ms: int, velocity: float, stopping_event: PureEvent = None):
        """Moves the vehicle by lag. Blocking call. Action execution will be canceled if stopping_event is triggered.

        :param duration_ms: Duration of the movement in milliseconds.
        :param velocity: Velocity of movement, from 0.0 to 1.0, positive to move right, negative - to move left (from vehicle point of view).
        :param stopping_event: Event that stops movement.
        """

        goal = self._create_lag_goal(duration_ms, velocity)
        return self._exec_action(self.LinearMoveClient, goal, stopping_event)

    def dive(self, depth_cm: int):
        """Dives the vehicle. Blocking call.

        :param depth_cm: Depth of diving in centimeters, must be positive.
        """

        goal = DiveGoal(depth=depth_cm)
        self.DiveClient.send_goal(goal)
        self.DiveClient.wait_for_result()

    def stop(self):
        """Stops the vehicle. Blocking call.
        """

        goal = self._create_stop_goal()
        self.LinearMoveClient.send_goal(goal)
        self.LinearMoveClient.wait_for_result()

    @staticmethod
    def _create_rotate_goal(yaw_deg: float):
        """Constructs action goal for rotation action server. This is a private method.

        :param yaw_deg: Angle to rotate in degrees, positive is for counterclockwise, negative - for clockwise.
        :return: Action goal.
        """
        return RotateGoal(yaw=yaw_deg)

    @staticmethod
    def _create_march_goal(duration_ms: int, velocity: float):
        """Constructs action goal for linear movement server to move by march. This is a private method.

        :param duration_ms: Duration of movement in milliseconds.
        :param velocity:  Velocity of movement, from 0.0 to 1.0, positive to move forward, negative - to backward movement.
        :return: Action goal.
        """

        direction = LinearMoveGoal.DIRECTION_MARCH_FORWARD if velocity >= 0 \
            else LinearMoveGoal.DIRECTION_MARCH_BACKWARDS
        goal = LinearMoveGoal(direction=direction, duration=duration_ms,
                              velocity=abs(velocity))
        return goal

    @staticmethod
    def _create_lag_goal(duration_ms: int, velocity: float):
        """Constructs action goal for linear movement server to move by lag. This is a private method.

        :param duration_ms: Duration of the movement in milliseconds.
        :param velocity: Velocity of movement, from 0.0 to 1.0, positive to move right, negative - to move left (from vehicle point of view).
        :return: Action goal.
        """

        direction = LinearMoveGoal.DIRECTION_LAG_RIGHT if velocity >= 0 \
            else LinearMoveGoal.DIRECTION_LAG_LEFT
        goal = LinearMoveGoal(direction=direction, duration=duration_ms,
                              velocity=abs(velocity))
        return goal

    @staticmethod
    def _create_stop_goal():
        """Constructs action goal to stop any movement. This is a private method.

        :return: Action goal.
        """

        return LinearMoveGoal(
            direction=LinearMoveGoal.DIRECTION_STOP, duration=0, velocity=0)

    @staticmethod
    def _exec_action(action_client: SimpleActionClient, goal, stopping_event: PureEvent = None):
        """Executes goal with specified action client and optional event that stops action execution when triggered.
        This is a private method.

        :param action_client: Action client to use.
        :param goal: Action goal to execute.
        :param stopping_event: Optional event that stops goal execution when triggered.
        :return: None if stopping_event is None. True if goal execution is stopped because of triggered stopping_event,
        False otherwise.
        """
        if stopping_event is None:
            action_client.send_goal(goal)
            action_client.wait_for_result()
            return

        stopping_event.start_listening()
        action_client.send_goal(goal,
                                active_cb=None,
                                feedback_cb=None,
                                done_cb=None)

        check_rate = rospy.Rate(100)

        while True:
            goal_state = action_client.get_state()
            if goal_state == SimpleGoalState.DONE:
                stopping_event.stop_listening()
                return False
            if stopping_event.is_triggered():
                action_client.cancel_goal()
                return True
            check_rate.sleep()
