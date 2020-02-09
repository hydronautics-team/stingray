import rospy
import rospkg
import actionlib
import stingray_movement_msgs.msg as msg


class AUV:
    absolute_angle = 0

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    def rotate_and_forward(self, angle=0, duration=0, velocity=0.0):
        """angle in degrees, duration in ms, velocity is relative(from 0 to 1)"""
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

        rospy.sleep(duration / 1000)

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
