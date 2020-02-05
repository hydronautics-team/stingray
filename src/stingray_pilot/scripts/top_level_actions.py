import rospy
import rospkg
import actionlib
import stingray_movement_msgs.msg


class AUV:
    absolute_angle = 0

    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")

    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))

    def rotate_and_forward_locked(self, angle, duration, velocity):
        """angle in degrees, duration in ms, velocity is relative(from 0 to 1)"""
        global absolute_angle
        absolute_angle += angle
        rospy.loginfo("Absolute angle is %d now".format(absolute_angle))

        if angle != 0:
            rospy.loginfo("Rotation init")
            client = actionlib.SimpleActionClient('stingray_action_rotate', stingray_movement_msgs.msg.RotateAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = stingray_movement_msgs.msg.RotateGoal(yaw=absolute_angle)
            rospy.loginfo("Sending goal")
            client.send_goal(goal)
            rospy.loginfo("Waiting for result")
            client.wait_for_result()
            rospy.loginfo("Result received")

        if duration != 0 and velocity != 0:
            rospy.loginfo("Marching init")
            client = actionlib.SimpleActionClient('stingray_action_linear_movement',
                                                  stingray_movement_msgs.msg.LinearMoveAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = stingray_movement_msgs.msg.LinearMoveGoal(direction=1, duration=duration, velocity=velocity)
            rospy.loginfo("Sending goal")
            client.send_goal(goal)
            rospy.loginfo("Waiting for result")
            client.wait_for_result()
            rospy.loginfo("Result received")

    def rotate_and_forward(self, angle=0, duration=0, velocity=0.0):
        """angle in degrees, duration in ms, velocity is relative(from 0 to 1)"""
        global absolute_angle
        absolute_angle += angle
        rospy.loginfo("Absolute angle is %{} now".format(absolute_angle))

        if angle != 0:
            rospy.loginfo("Rotation init")
            client = actionlib.SimpleActionClient('stingray_action_rotate', stingray_movement_msgs.msg.RotateAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = stingray_movement_msgs.msg.RotateGoal(yaw=absolute_angle)
            rospy.loginfo("Sending goal")
            client.send_goal(goal,
                             active_cb=self.callback_active,
                             feedback_cb=self.callback_feedback,
                             done_cb=self.callback_done)
        if duration != 0 and velocity != 0:
            rospy.loginfo("Marching init")
            client = actionlib.SimpleActionClient('stingray_action_linear_movement',
                                                  stingray_movement_msgs.msg.LinearMoveAction)
            rospy.loginfo("Waiting for server")
            client.wait_for_server()
            goal = stingray_movement_msgs.msg.LinearMoveGoal(direction=1, duration=duration, velocity=velocity)
            rospy.loginfo("Sending goal")
            client.send_goal(goal,
                             active_cb=self.callback_active,
                             feedback_cb=self.callback_feedback,
                             done_cb=self.callback_done)

        rospy.sleep(duration / 1000)
