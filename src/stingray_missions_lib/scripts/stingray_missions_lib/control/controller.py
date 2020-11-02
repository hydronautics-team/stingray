from actionlib import SimpleActionClient, SimpleGoalState
import stingray_movement_msgs.msg
from .event import EventBase

"""@package docstring
Contains class for controlling AUV in missions.
"""


class AUVController:
    """Class for controlling AUV in missions.

    The class is actually a unified wrap over actions and services calls.
    """

    # TODO: Errors handling
    # TODO: Devices support

    def __init__(self):
        """The constructor.
        """
        self.rotation_client_ = SimpleActionClient('stingray_action_rotate',
                                                   stingray_movement_msgs.msg.RotateAction)
        self.dive_client_ = SimpleActionClient('stingray_action_dive',
                                               stingray_movement_msgs.msg.DiveAction)
        self.linear_client_ = SimpleActionClient('stingray_action_linear_movement',
                                                 stingray_movement_msgs.msg.LinearMoveAction)
        self.rotation_client_.wait_for_server()
        self.dive_client_.wait_for_server()
        self.linear_client_.wait_for_server()

    def rotate(self, yaw_deg: float):
        """Rotates the vehicle. Blocking call.

        :param yaw_deg: Angle to rotate in degrees, positive is for counterclockwise, negative - for clockwise.
        """

        goal = self._create_rotate_goal(yaw_deg=yaw_deg)
        self._exec_action(self.rotation_client_, goal)

    def march(self, duration_ms: int, velocity: float, stopping_event: EventBase = None):
        """Moves the vehicle by march. Blocking call. Action execution will be canceled if stopping_event is triggered.

        :param duration_ms: Duration of movement in milliseconds.
        :param velocity:  Velocity of movement, from 0.0 to 1.0, positive to move forward, negative - to backward movement.
        :param stopping_event: Event that stops movement.
        """

        goal = self._create_march_goal(duration_ms, velocity)
        return self._exec_action(self.linear_client_, goal, stopping_event)

    def lag(self, duration_ms: int, velocity: float, stopping_event: EventBase = None):
        """Moves the vehicle by lag. Blocking call. Action execution will be canceled if stopping_event is triggered.

        :param duration_ms: Duration of the movement in milliseconds.
        :param velocity: Velocity of movement, from 0.0 to 1.0, positive to move right, negative - to move left (from vehicle point of view).
        :param stopping_event: Event that stops movement.
        """

        goal = self._create_lag_goal(duration_ms, velocity)
        return self._exec_action(self.linear_client_, goal, stopping_event)

    def dive(self, depth_cm: int):
        """Dives the vehicle. Blocking call.

        :param depth_cm: Depth of diving in centimeters, must be positive.
        """

        goal = stingray_movement_msgs.msg.DiveGoal(depth=depth_cm)
        self.dive_client_.send_goal(goal)
        self.dive_client_.wait_for_result()

    def stop(self):
        """Stops the vehicle. Blocking call.
        """

        goal = self._create_stop_goal()
        self.linear_client_.send_goal(goal)
        self.linear_client_.wait_for_result()

    @staticmethod
    def _create_rotate_goal(yaw_deg: float):
        """Constructs action goal for rotation action server. This is a private method.

        :param yaw_deg: Angle to rotate in degrees, positive is for counterclockwise, negative - for clockwise.
        :return: Action goal.
        """
        return stingray_movement_msgs.msg.RotateGoal(yaw=yaw_deg)

    @staticmethod
    def _create_march_goal(duration_ms: int, velocity: float):
        """Constructs action goal for linear movement server to move by march. This is a private method.

        :param duration_ms: Duration of movement in milliseconds.
        :param velocity:  Velocity of movement, from 0.0 to 1.0, positive to move forward, negative - to backward movement.
        :return: Action goal.
        """

        direction = stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_MARCH_FORWARD if velocity >= 0 \
            else stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_MARCH_BACKWARDS
        goal = stingray_movement_msgs.msg.LinearMoveGoal(direction=direction, duration=duration_ms,
                                                         velocity=abs(velocity))
        return goal

    @staticmethod
    def _create_lag_goal(duration_ms: int, velocity: float):
        """Constructs action goal for linear movement server to move by lag. This is a private method.

        :param duration_ms: Duration of the movement in milliseconds.
        :param velocity: Velocity of movement, from 0.0 to 1.0, positive to move right, negative - to move left (from vehicle point of view).
        :return: Action goal.
        """

        direction = stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_LAG_RIGHT if velocity >= 0 \
            else stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_LAG_LEFT
        goal = stingray_movement_msgs.msg.LinearMoveGoal(direction=direction, duration=duration_ms,
                                                         velocity=abs(velocity))
        return goal

    @staticmethod
    def _create_stop_goal():
        """Constructs action goal to stop any movement. This is a private method.

        :return: Action goal.
        """

        return stingray_movement_msgs.msg.LinearMoveGoal(
            direction=stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_STOP, duration=0, velocity=0)

    @staticmethod
    def _exec_action(action_client: SimpleActionClient, goal, stopping_event: EventBase = None):
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
        while True:
            goal_state = action_client.get_state()
            if goal_state == SimpleGoalState.DONE:
                stopping_event.stop_listening()
                return False
            if stopping_event.is_triggered():
                action_client.cancel_goal()
                return True
