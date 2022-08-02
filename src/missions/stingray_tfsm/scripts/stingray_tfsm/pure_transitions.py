from transitions import Machine
from transitions.extensions import GraphMachine
from copy import copy
from ast import literal_eval
import json
import rospkg
import os


class FSM_Simple:

    @staticmethod
    def callback_wrapper(userdata: dict = None, external_cb=None):
        """
        The callback_wrapper function is a helper function that wraps the user-defined callback function.
        It is used to handle the following:
            - The return value of the callback_wrapper is always a dictionary with two keys: 'status' and 'data'.
              If there was an error, status will be set to False and data will contain an error message.
              Otherwise, status will be True and data contains your original return value from your callback function.

        :param userdata:dict=None: Pass in data to the callback function
        :param external_cb=None: Pass in a callback function that has been passed to the state machine
        :return: A function that takes in a userdata and an external_cb argument
        :doc-author: Trelent
        """
        if not callable(external_cb):
            raise TypeError(
                "Callable function should be passed to callback_wrapper")
        external_cb(userdata)

    def next_step(self, *args, **kwargs):
        """
        The next_step function is the default variant of next step. It should be overridden to do complex callbacks

        :param self: Access the attributes of the class
        :param *args: Pass a non-keyworded, variable-length argument list
        :param **kwargs: Pass a dictionary of additional keyword arguments to the function
        :return: The trigger that is associated with the current state
        :doc-author: Trelent
        """
        if self.verbose:
            print(f"DEBUG: current state of abstract machine is {self.state}")
            print(
                f"DEBUG: doing the transition {self.fsm.get_triggers(self.state)[0]}")

        self.trigger(self.fsm.get_triggers(self.state)[0],
                     {'state_name': self.state})

    @staticmethod
    def read_rulebook(path):
        """
        The read_rulebook function reads a rulebook and returns the states, transitions,
        and initial state. The function takes one argument: path to the file containing
        the information about the states and transitions.

        :param path: Specify the location of the rulebook
        :return: A list of dictionaries
        :doc-author: Trelent
        """
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

    def __init__(self, states: tuple = (), transitions: list = (), path=None, *args, **kwargs):
        """
        The __init__ function is called when the class is instantiated.
        :param self: Refer to the object itself
        :param states:tuple=(): Define the states of the machine
        :param transitions:list=(): Define the transitions of the graph
        :param path=None: Pass the path to the rulebook
        :param *args: Pass a non-keyworded, variable-length argument list
        :param **kwargs: Pass keyworded, variable-length arguments to the function
        :return: The object of the class
        :doc-author: Trelent
        """
        self.gsm = copy(self)
        if path is not None:
            states, transitions = self.read_rulebook(path)
        self.g_fsm = GraphMachine(
            model=self.gsm, states=states, transitions=transitions, initial='init')
        self.fsm = Machine(model=self, states=states, transitions=transitions,
                           initial='init', auto_transitions=False)
        self.verbose = True
        # configs
        stingray_resources_path = rospkg.RosPack().get_path("stingray_resources")
        with open(os.path.join(stingray_resources_path, "configs/ros.json")) as f:
            self.ros_config = json.load(f)
        with open(os.path.join(stingray_resources_path, "configs/control.json")) as f:
            self.control_config = json.load(f)

    def set_verbose(self, verbose):
        """
        The set_verbose function is used to set the verbose attribute of FSM_simple class.
        It is typically called to allow callers to control whether or not
        detailed output is printed by that module.

        :param self: Refer to the object itself
        :param verbose: Determine whether or not the function will print out a message
        :return: The value of the verbose parameter
        :doc-author: Trelent
        """
        if verbose:
            self.verbose = True
        else:
            self.verbose = False

    def describe(self):
        """
        The describe function draws a state diagram of machine


        :param self: Reference the current instance of the class
        :return: None
        :doc-author: Trelent
        """
        self.gsm.get_graph().draw("state_diagram.png")

    def set_state(self, state):
        """
        The set_state function sets the state of the FSM.



        :param self: Access variables that belongs to the class
        :param state: Set the valid state of the FSM
        :return: None
        :doc-author: Trelent
        """
        self.fsm.set_state(state)

    def run(self, *args, **kwargs):
        """
        The run function is the main function of the state machine. It is called
        when a state machine is started, and it continues to execute until it reaches
        the 'done' or 'aborted' states. The run function executes one step of the
        state machine at a time, where each step performs some operation on the robot
        and then advances to its next state based on which transition conditions are met.



        :param self: Access the class attributes and methods
        :param *args: Pass a non-keyworded, variable-length argument list
        :param **kwargs: Pass a variable number of arguments to a function
        :return: 1 if the state is done
        :doc-author: Trelent
        """
        current_state = self.state
        while current_state != 'done' and current_state != 'aborted':
            """conditional transitions are handled in next_step"""

            self.next_step()
            current_state = self.state
            # print('\n==== STEP IS OVER ====\n')

        if current_state == 'done':
            return 1
        elif current_state == 'aborted':
            return 0

    def add_state(self, states, **kwargs):
        self.fsm.add_states(states, **kwargs)

    def add_transitions(self, transitions):
        self.fsm.add_transitions(transitions)
