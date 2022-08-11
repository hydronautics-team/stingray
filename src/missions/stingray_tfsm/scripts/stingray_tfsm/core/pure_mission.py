from abc import ABC, abstractmethod
from typing import Dict, List, Tuple
from stingray_tfsm.core.pure_fsm import PureStateMachine
from stingray_tfsm.core.pure_events import PureEvent, TopicEvent
import rospy

"""
Contains an abstract class for creating missions. It should unify the ROS state machine and required events.
Inspired by TimeEscaper
"""


class PureMission(ABC):
    """ Abstract class with default transitions, states and basic methods to implement mission """

    FSM_CLASS = PureStateMachine

    @abstractmethod
    def __init__(self, name: str):
        """ Abstract class with default transitions, states and basic methods to implement mission
        
        Args: 
            mission_name (str): mission name
        """
        self.name = name
        """ mission name """
        """ default transitions for FSM """
        self.default_scene = {
            self.machine.state_init: {
                'time': 0.1
            }
        }
        """ default arguments for FSM """
        self.machine: PureStateMachine = None
        """ the PureStateMachine object """

        self._reset()

    @abstractmethod
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
        self.machine = self.FSM_CLASS(self.name, self.setup_states(), self.setup_transitions, self.default_scene.update(self.setup_scene()))

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
    def setup_events(self):
        """
        The setup_events function sets up the events for a specific topic and object.
        It is called by the setup_events function in the Topic class, which sets up events for all topics and objects.
        """
        raise NotImplementedError

    @staticmethod
    def event_handler(event: PureEvent):
        """
        The event_handler function is a function that is called when the event is triggered.
        It returns True or False depending on whether the event was triggered or not.

        :param event: Pass the event that is being handled
        :return: True if the event is triggered and false if it is not
        
        """
        event.start_listening()
        rospy.sleep(0.5)
        value = event.is_triggered()
        event.stop_listening()
        return value

    @abstractmethod
    def verbose(self, verbose):
        """
        The verbose function is used to set machine's extended output

        :param self: Reference the class itself
        :param verbose: Determine whether to print the progress of the algorithm
        :return: None
        
        """
        self.machine.set_verbose(verbose)

    @abstractmethod
    def run(self):
        """
        The run function is a relay to state machine's run function.

        :param self: Reference the current instance of the class
        :return: True if machine finished its tasks successfully
        
        """
        if self.machine is not None:
            return self.machine.run()
        else:
            raise TypeError("FSM was not initialized")
