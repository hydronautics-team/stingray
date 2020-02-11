#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import stingray_movement_msgs.msg as msg


class AUV:
    """All actions are unlocked and are supposed to be used asynchronously"""
    absolute_angle = 0

    @staticmethod
    def callback_active():
        rospy.loginfo("Action server is processing the goal")

    @staticmethod
    def callback_done(state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    @staticmethod
    def callback_feedback(feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    def rotate(self, angle=0):
        self.absolute_angle += angle
        rospy.loginfo("Absolute angle is %{} now".format(self.absolute_angle))

        if angle != 0:
            rospy.loginfo("Rotation init")
            client = actionlib.SimpleActionClient('stingray_action_rotate', msg.RotateAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = msg.RotateGoal(yaw=self.absolute_angle)
            rospy.loginfo("Sending goal")
            client.send_goal(goal,
                             active_cb=self.callback_active,
                             feedback_cb=self.callback_feedback,
                             done_cb=self.callback_done)

    def forward(self, duration=0, velocity=0.0):
        """angle in degrees, duration in ms, velocity is relative(from 0 to 1)"""
        if duration != 0 and velocity != 0:
            rospy.loginfo("Marching init")
            client = actionlib.SimpleActionClient('stingray_action_linear_movement', msg.LinearMoveAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = msg.LinearMoveGoal(direction=1, duration=duration, velocity=velocity)
            rospy.loginfo("Sending goal")
            client.send_goal(goal,
                             active_cb=self.callback_active,
                             feedback_cb=self.callback_feedback,
                             done_cb=self.callback_done)

        rospy.sleep(duration / 1000)    # TODO check if it breaks the async

    def lag_right(self, duration=0, velocity=0.0):
        if duration != 0 and velocity != 0:
            rospy.loginfo("Lagging init")
            client = actionlib.SimpleActionClient('stingray_action_linear_movement', msg.LinearMoveAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = msg.LinearMoveGoal(direction=3, duration=duration, velocity=velocity)
            rospy.loginfo("Sending goal")
            client.send_goal(goal,
                             active_cb=self.callback_active,
                             feedback_cb=self.callback_feedback,
                             done_cb=self.callback_done)

    def dive(self, depth=100):
        client = actionlib.SimpleActionClient('stingray_action_dive', msg.DiveAction)
        print("Waiting for server")
        client.wait_for_server()
        goal = msg.DiveGoal(depth=depth)
        print("Sending goal")
        client.send_goal(goal,
                         active_cb=self.callback_active,
                         feedback_cb=self.callback_feedback,
                         done_cb=self.callback_done)


