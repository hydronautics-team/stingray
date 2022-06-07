from transitions import Machine
from transitions.extensions import GraphMachine
from time import sleep
from copy import copy
from ast import literal_eval
import rospy


def generic_callback(userdata: dict = None):
    """this exists only to test wrapper"""
    if userdata is not None:
        for arg, val in userdata.items():
            if arg == 'state_name':
                print(f"kinda doin' something in state {val}")
                break
        else:
            print("this func has no idea what is it doin'")
    else:
        print("this func has no idea what is it doin'")


class FSM_Simple:
    @staticmethod
    def imitate_work():
        # print('imitating work(dont tell anyone pls)')
        sleep(1)

    @staticmethod
    def callback_wrapper(userdata: dict = None, external_cb=generic_callback):
        if not callable(external_cb):
            raise TypeError("Callable function should be passed to callback_wrapper")
        print('This exists to make execution of any function here possible')
        external_cb(userdata)

    default_states = ('init', 'online', 'aborted', 'done')
    default_transitions = (
        {'trigger': 'start', 'source': 'init', 'dest': 'online'},
        {'trigger': 'work', 'source': 'online', 'dest': 'done', 'prepare': 'callback_wrapper'},
        {'trigger': 'stop', 'source': 'online', 'dest': 'aborted'},
        {'trigger': 'restart', 'source': ['done', 'aborted'], 'dest': 'init'}
    )

    def next_step(self):
        """default variant of next step. Should be overridden to do complex callbacks"""
        self.trigger(self.fsm.get_triggers(self.state)[0],
                     {'state_name': self.state})

    @staticmethod
    def read_rulebook(path):
        """rules for writings rulebooks should be specified"""
        states = None
        tr_list = []
        with open(path) as f:
            for line in f:
                if line[0] == '(':
                    states = literal_eval(line)
                elif line[0] == '{':
                    tr_list.append(literal_eval(line))
        return states, tr_list

    def __init__(self, name, path=None, states=default_states, transitions=default_transitions):
        self.name = name
        self.gsm = copy(self)
        if path is not None:
            states, transitions = self.read_rulebook(path)
        self.g_fsm = GraphMachine(model=self.gsm, states=states, transitions=transitions, initial='init')
        self.fsm = Machine(model=self, states=states, transitions=transitions, initial='init', auto_transitions=False)

    def describe(self):
        self.gsm.get_graph().draw("state_diagram.png")

    def run(self):
        current_state = self.state
        print("here we go, loud and verbosely")

        while current_state != 'done' and current_state != 'aborted':
            """conditional transitions with one name should be used"""
            """in the style: if prev step is done -> proceed to next"""

            self.next_step()
            current_state = self.state
            print('\n==== STEP IS OVER ====\n')

        if current_state == 'done':
            print("here i should report to topic that I'm done")
        elif current_state == 'aborted':
            print("something went wrong and is reported to ros")








