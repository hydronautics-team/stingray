from pure_transitions import FSM_Simple
from actionlib import SimpleActionClient
import stingray_movement_msgs.msg as msg
import rospy


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


class AUVStateMachine(FSM_Simple):
    def __init__(self, topics_to_sub: dict, states: tuple, transitions: dict, path=None):
        super().__init__(states, transitions, path)
        self.LinearMoveClient = SimpleActionClient('stingray_action_linear_movement', msg.LinearMoveAction)

        self.absolute_angle=0
        self.RotateClient = SimpleActionClient('stingray_action_rotate', msg.RotateAction)
        self.DiveClient = SimpleActionClient('stingray_action_dive', msg.DiveAction)
        self.topics_to_sub = dict()

        for topic in topics_to_sub:
            self.topics_to_sub[topic] = topics_to_sub[topic]

    def execute_move_goal(self, userdata):
        goal = msg.LinearMoveGoal(userdata['direction'], userdata['velocity'], userdata['duration'])
        self.LinearMoveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)
        rospy.loginfo('Goal sent')
        self.LinearMoveClient.wait_for_result(timeout=rospy.Duration(secs=userdata['duration'] // 1000 + 1))
        rospy.loginfo('Result got')

    def execute_dive_goal(self, userdata):
        goal = msg.DiveClientGoal(userdata['depth'])
        self.DiveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                    active_cb=callback_active)
        rospy.loginfo('Goal sent')
        self.DiveClient.wait_for_result(timeout=rospy.Duration(secs=5))
        rospy.loginfo('Result got')

    def execute_rotate_goal(self, userdata):
        angle = userdata['angle']
        self.absolute_angle += angle

        if self.absolute_angle > 360:
            self.absolute_angle -= 360
        elif self.absolute_angle < -360:
            self.absolute_angle += 360

        rospy.loginfo("Absolute angle is {} now".format(self.absolute_angle))

        if angle != 0:
            goal = msg.RotateGoal(yaw=self.absolute_angle)
            self.RotateClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)
            rospy.loginfo('Goal sent')
            self.RotateClient.wait_for_result(timeout=rospy.Duration(secs=5))
            rospy.loginfo('Result got')
        else:
            rospy.loginfo('Rotating is not required to achieve this angle')

    def monitor_state

