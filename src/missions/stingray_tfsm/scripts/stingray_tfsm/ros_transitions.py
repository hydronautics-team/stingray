from stingray_tfsm.pure_transitions import FSM_Simple
from actionlib import SimpleActionClient
import stingray_movement_msgs.msg as msg
import rospy


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


TRANSITIONS = [     # Timings
    {'start', 'init', 'march_1'}
    ]


class AUVStateMachine(FSM_Simple):
    def __init__(self, states: tuple, transitions: list = None, scene: dict = None, path=None):
        super().__init__(states, transitions, path, verbose)
        self.LinearMoveClient = SimpleActionClient('stingray_action_linear_movement', msg.LinearMoveAction)
        self.scene = scene

        self.absolute_angle = 0
        self.RotateClient = SimpleActionClient('stingray_action_rotate', msg.RotateAction)
        self.DiveClient = SimpleActionClient('stingray_action_dive', msg.DiveAction)

    def execute_move_goal(self, scene):
        goal = msg.LinearMoveGoal(scene['direction'], scene['velocity'], scene['duration'])
        self.LinearMoveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                        active_cb=callback_active)
        rospy.loginfo('Goal sent')
        self.LinearMoveClient.wait_for_result(timeout=rospy.Duration(secs=scene['duration'] // 1000 + 1))
        rospy.loginfo('Result got')

    def execute_dive_goal(self, scene):
        goal = msg.DiveClientGoal(scene['depth'])
        self.DiveClient.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                    active_cb=callback_active)
        rospy.loginfo('Goal sent')
        self.DiveClient.wait_for_result(timeout=rospy.Duration(secs=5))
        rospy.loginfo('Result got')

    def execute_rotate_goal(self, scene):
        angle = scene['angle']
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

    # def condition(self, args):
    #     """this is most probably a stub"""
    #     if str(1).isalpha():
    #         return 1
    #     else:
    #         return 0

    def next_step(self):
        state_keyword = self.state.split('_')[0]
        scene = self.scene[self.state]
        if self.verbose:
            rospy.loginfo("Current state of ros machine is ", self.state)

        if state_keyword == 'init':
            rospy.sleep(scene['time'])

        elif state_keyword == 'move':
            self.execute_move_goal(scene)

        elif state_keyword == 'rotate':
            self.execute_rotate_goal(scene)

        elif state_keyword == 'dive':
            self.execute_dive_goal(scene)

        elif state_keyword == 'condition':
            decision = scene['condition'](scene['args'])
            if decision:
                if self._verbose:
                    rospy.loginfo("Current condition results True")
                self.trigger('condition_s')
            else:
                if self._verbose:
                    rospy.loginfo("Current condition results False")
                self.trigger('condition_f')
            return
        elif state_keyword == 'done':
            exit()
        self.trigger(self.fsm.get_triggers(self.state)[0])


