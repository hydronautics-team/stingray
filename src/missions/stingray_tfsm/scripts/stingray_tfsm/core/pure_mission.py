from abc import ABC, abstractmethod
from typing import Dict, List, Tuple
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.core.pure_events import PureEvent, TopicEvent

"""
Contains an abstract class for creating missions. It should unify the ROS state machine and required events.
Inspired by TimeEscaper
"""


class PureMission(ABC):
    """ Abstract class with default transitions, states and basic methods to implement mission """

    @abstractmethod
    def __init__(self,
                 name: str,
                 machine: PureStateMachine = None,
                 verbose: bool = False
                 ):
        """ Abstract class with default transitions, states and basic methods to implement mission

        Args: 
            mission_name (str): mission name
        """
        self.name = name
        """ mission name """
        self.machine: PureStateMachine = machine
        """ the PureStateMachine object """
        self.verbose = verbose

        self._reset()

    def mission_state(self, state: str):
        return self.name.upper() + "_" + state.upper()

    def mission_transition(self, transition: str):
        return self.name.lower() + "_" + transition.lower()

    def _reset(self):
        """
        The reset function is called at the beginning of each trial. It is used to
        set the initial state of any variables that are needed for your condition
        script, and can also be used to reset ROS nodes between trials.

        :param self: Access the class's attributes and methods
        :param *args: Pass a non-keyworded, variable-length argument list
        :param **kwargs: Pass a variable number of keyword arguments to a function
        :return: The scene dictionary

        """
        self.setup_events()
        if self.machine is None:
            self.machine = PureStateMachine(self.name, verbose=self.verbose)
        self.machine.add_states(self.setup_states())
        self.machine.add_transitions(self.setup_transitions())
        self.machine.add_default_transitions()
        self.machine.add_scene(self.setup_scene())

    @abstractmethod
    def setup_states(self) -> Tuple:
        """ Method to setup user states

        Raises:
            NotImplementedError: you need to implement it in your mission class

        Returns:
            Tuple: tuple of states
        """
        raise NotImplementedError

    @abstractmethod
    def setup_transitions(self) -> List:
        """ Method to setup user transitions

        Raises:
            NotImplementedError: you need to implement it in your mission class

        Returns:
            List: list of transitions
        """
        raise NotImplementedError

    @abstractmethod
    def setup_scene(self) -> Dict:
        """ Method to setup user scene args

        Raises:
            NotImplementedError: you need to implement it in your mission class

        Returns:
            Dict: dict with args
        """
        raise NotImplementedError

    @abstractmethod
    def setup_events(self, *args, **kwargs):
        """
        The setup_events function sets up the events for a specific topic and object.
        It is called by the setup_events function in the Topic class, which sets up events for all topics and objects.
        """
        raise NotImplementedError

    def run(self):
        """
        The run function is a relay to state machine's run function.

        :return: True if machine finished its tasks successfully

        """
        if self.check_machine():
            value = self.machine.run()
            return value

    def check_machine(self):
        if type(self.machine) is PureStateMachine:
            return 1
        else:
            raise TypeError("PureStateMachine was not initialized")

    def set_init_state(self, ):
        if self.check_machine():
            self.machine.set_state(self.machine.state_init)

    def set_state(self, state):
        if self.check_machine():
            self.machine.set_state(state)
