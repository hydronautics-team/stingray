from stingray_tfsm.fsm_transitions import FSM_Simple
import rospy
import actionlib

LAG_DIRECTION = 'LEFT'
FIRST_MARCH_TIME = 5500  # int(rospy.get_param('~firstMarchTime', '5500'))
FIRST_LAG_TIME = 4500  # int(rospy.get_param('~firstMarchTime', '4500'))
SECOND_MARCH_TIME = 12000  # int(rospy.get_param('~secondMarchTime', '12000'))

STATES = ('init', 'march_1', 'lag_1', 'march_2', 'aborted', 'done')
TRANSITIONS = [
    {'trigger': 'start', 'source': 'init', 'dest': 'march_1', 'prepare': 'callback_wrapper'},
    {'trigger': 'step1', 'source': 'march_1', 'dest': 'lag_1', 'prepare': 'callback_wrapper'},
    {'trigger': 'step2', 'source': 'lag_1', 'dest': 'march_2', 'prepare': 'callback_wrapper'},
    {'trigger': 'finish', 'source': 'march_2', 'dest': 'done'},
    {'trigger': 'abort', 'source': ['march_1', 'lag_1', 'march_2'], 'dest': 'aborted'},
    {'trigger': 'restart', 'source': ['done', 'aborted'], 'dest': 'init'},
]


class GateMission(FSM_Simple):
    def __init__(self, name, path=None):
        super().__init__(name, path, states=STATES, transitions=TRANSITIONS)

    def next_step(self):
        if self.state == 'init':
            userdata = {
                'FIRST_MARCH_TIME': FIRST_MARCH_TIME
            }
        elif self.state == 'march_1':
            userdata = {
                'LAG_DIRECTION': LAG_DIRECTION,
                'FIRST_LAG_TIME': FIRST_LAG_TIME
            }
        elif self.state == 'lag_1':
            userdata = {
                "SECOND_MARCH_TIME": SECOND_MARCH_TIME
            }
        elif self.state == 'march_2':
            userdata = {
                'STOP': True
            }
        self.trigger(self.fsm.get_triggers(self.state)[0],
                     userdata, external_cb=self.goal_switch)

    def goal_switch(self, userdata: dict):
        if userdata is not None:
            for arg, val in userdata.items():
                print(arg, ':', val)
        else:
            print("No data")


generic_mission = GateMission('timings')

generic_mission.run()
generic_mission.describe()
