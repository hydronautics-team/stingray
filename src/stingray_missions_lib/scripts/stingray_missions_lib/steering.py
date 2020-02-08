import rospy
import actionlib
import stingray_movement_msgs.msg
import enum


class BasicSteerer:
    def __init__(self):
        self.rotation_client_ = actionlib.SimpleActionClient('stingray_action_rotate',
                                                             stingray_movement_msgs.msg.RotateAction)
        self.dive_client_ = actionlib.SimpleActionClient('stingray_action_dive',
                                                         stingray_movement_msgs.msg.DiveAction)
        self.linear_client_ = actionlib.SimpleActionClient('stingray_action_linear_movement',
                                                           stingray_movement_msgs.msg.LinearMoveAction)

    def create_rotate_goal(self, yaw):
        return stingray_movement_msgs.msg.RotateGoal(yaw=yaw)

    def create_march_goal(self, duration_ms, velocity):
        direction = stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_MARCH_FORWARD if velocity >= 0 \
            else stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_MARCH_BACKWARDS
        goal = stingray_movement_msgs.msg.LinearMoveGoal(direction=direction, duration=duration_ms, velocity=velocity)
        return goal

    def create_lag_goal(self, duration_ms, velocity):
        direction = stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_LAG_RIGHT if velocity >= 0 \
            else stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_LAG_LEFT
        goal = stingray_movement_msgs.msg.LinearMoveGoal(direction=direction, duration=duration_ms, velocity=velocity)
        return goal

    def create_stop_goal(self):
        return stingray_movement_msgs.msg.LinearMoveGoal(
            direction=stingray_movement_msgs.msg.LinearMoveGoal.DIRECTION_STOP, duration=0, velocity=0)

    def rotate(self, yaw):
        goal = self.create_rotate_goal(yaw=yaw)
        self.rotation_client_.send_goal(goal)
        self.rotation_client_.wait_for_result()

    def march(self, duration_ms, velocity):
        goal = self.create_march_goal(duration_ms, velocity)
        self.linear_client_.send_goal(goal)
        self.linear_client_.wait_for_result()

    def lag(self, duration_ms, velocity):
        goal = self.create_lag_goal(duration_ms, velocity)
        self.linear_client_.send_goal(goal)
        self.linear_client_.wait_for_result()

    def stop(self):
        goal = self.create_stop_goal()
        self.linear_client_.send_goal(goal)
        self.linear_client_.wait_for_result()


