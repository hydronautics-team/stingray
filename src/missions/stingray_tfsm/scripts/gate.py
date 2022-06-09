from stingray_tfsm.fsm_transitions import FSM_Simple
import rospy
from actionlib import SimpleActionClient
import stingray_movement_msgs.msg as msg

LAG_DIRECTION = 3  # 4 in real = 'LEFT'
FIRST_MARCH_TIME = 50  # int(rospy.get_param('~firstMarchTime', '5500'))
FIRST_LAG_TIME = 4500  # int(rospy.get_param('~firstMarchTime', '4500'))
SECOND_MARCH_TIME = 6800  # int(rospy.get_param('~secondMarchTime', '12000'))
SECOND_LAG_TIME = 3000

STATES = ('init', 'march_1', 'lag_1', 'march_2', 'stop' 'aborted', 'done')
TRANSITIONS = [     # Timings
    {'trigger': 'start', 'source': 'init', 'dest': 'march_1', 'prepare': 'callback_wrapper'},
    {'trigger': 'step1', 'source': 'march_1', 'dest': 'lag_1', 'prepare': 'callback_wrapper'},
    {'trigger': 'step2', 'source': 'lag_1', 'dest': 'march_2', 'prepare': 'callback_wrapper'},
    {'trigger': 'step3', 'source': 'march_2', 'dest': 'stop', 'prepare': 'callback_wrapper'},
    {'trigger': 'finish', 'source': 'stop', 'dest': 'done', 'prepare': 'callback_wrapper'},
    {'trigger': 'abort', 'source': ['march_1', 'lag_1', 'march_2', 'stop'], 'dest': 'aborted'},
    {'trigger': 'restart', 'source': ['done', 'aborted'], 'dest': 'init'},
]


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


class GateMission(FSM_Simple):
    def __init__(self, name, path=None):
        super().__init__(name, path, states=STATES, transitions=TRANSITIONS)
        self.move_client = SimpleActionClient('stingray_action_linear_movement', msg.LinearMoveAction)

    def execute_move_goal(self, userdata):
        if 'LAG' not in userdata:
            userdata['LAG'] = 1  # in real 1 in simulation 2
        if 'TIME' not in userdata:
            userdata['TIME'] = 50
        goal = msg.LinearMoveGoal(userdata['LAG'], 0.4, userdata['TIME'])
        self.move_client.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                   active_cb=callback_active)
        rospy.loginfo('goal sent')
        self.move_client.wait_for_result(timeout=rospy.Duration(secs=userdata['TIME'] // 1000 + 1))
        rospy.loginfo('result got')

    def blank_action(self):  # awful costyl made to avoid loss of first action
        goal = msg.LinearMoveGoal(0, 0, 0)
        self.move_client.send_goal(goal, done_cb=callback_done, feedback_cb=callback_feedback,
                                   active_cb=callback_active)
        self.move_client.wait_for_result(timeout=rospy.Duration(nsecs=10000))
        rospy.loginfo('blank action done')

    def next_step(self):
        if self.state == 'init':
            userdata = {
                'TIME': FIRST_MARCH_TIME
            }
            self.blank_action()
            self.blank_action()

        elif self.state == 'march_1':
            userdata = {
                'LAG': LAG_DIRECTION,
                'TIME': FIRST_LAG_TIME
            }
        elif self.state == 'lag_1':
            userdata = {
                "TIME": SECOND_MARCH_TIME
            }
        elif self.state == 'march_2':
            userdata = {
                'LAG': LAG_DIRECTION,
                'TIME': SECOND_LAG_TIME
            }
        elif self.state == 'stop':
            userdata = {
                'STOP': True,
                'LAG': 0
            }
        else:
            userdata = None
        self.trigger(self.fsm.get_triggers(self.state)[0],
                     userdata, external_cb=self.goal_switch)

    def goal_switch(self, userdata: dict):  # TODO this is redundant?
        print("here we're switchin'")
        self.move_client.wait_for_server(rospy.Duration(nsecs=1000))
        if 'LAG' in userdata:
            self.execute_move_goal(userdata)

        elif 'TIME' in userdata:
            self.execute_move_goal(userdata)

        elif 'STOP' in userdata:
            self.execute_move_goal(userdata)
        else:
            raise rospy.ERROR("Invalid data sent to fsm goal callback wrapper")


if __name__ == '__main__':
    rospy.init_node("control_fsm")
    generic_mission = GateMission('timings')
    generic_mission.run()
