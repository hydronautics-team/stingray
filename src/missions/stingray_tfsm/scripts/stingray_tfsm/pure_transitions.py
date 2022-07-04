from transitions import Machine
from transitions.extensions import GraphMachine
from copy import copy
from ast import literal_eval


class FSM_Simple:
    @staticmethod
    def callback_wrapper(userdata: dict = None, external_cb=None):
        if not callable(external_cb):
            raise TypeError("Callable function should be passed to callback_wrapper")
        external_cb(userdata)

    def next_step(self, *args, **kwargs):
        """default variant of next step. Should be overridden to do complex callbacks"""
        print(self.state)
        print(self.fsm.get_triggers(self.state))

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

    def __init__(self, states: tuple = (), transitions: list = (), path=None):
        self.gsm = copy(self)
        if path is not None:
            states, transitions = self.read_rulebook(path)
        self.g_fsm = GraphMachine(model=self.gsm, states=states, transitions=transitions, initial='init')
        self.fsm = Machine(model=self, states=states, transitions=transitions, initial='init', auto_transitions=False)

    def describe(self):
        self.gsm.get_graph().draw("state_diagram.png")

    def run(self, *args, **kwargs):
        current_state = self.state
        while current_state != 'done' and current_state != 'aborted':
            """conditional transitions are handled in next_step"""

            self.next_step()
            current_state = self.state
            # print('\n==== STEP IS OVER ====\n')

        if current_state == 'done':
            return 0
        elif current_state == 'aborted':
            return 1

    def add_state(self, states, **kwargs):
        self.fsm.add_states(states, **kwargs)

    def add_transitions(self, transitions):
        self.fsm.add_transitions(transitions)








