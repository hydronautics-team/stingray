
from abc import ABC, abstractmethod
from typing import List
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.core.pure_fsm import PureStateMachine
import rospy


class AUVController(ABC):
    """ Control fsm class for AUV missions """
    @abstractmethod
    def __init__(self):
        self.machine: PureStateMachine = None
        """ pure state machine """
        self._reset()
        
    def _reset(self):
        """
        The reset function is called at the beginning of each trial. It is used to
        set the initial state of any variables that are needed for your condition
        script, and can also be used to reset ROS nodes between trials.
        
        """
        self.machine = PureStateMachine(
            self.__class__.__name__)
        self.setup_missions()
        self.machine.add_transitions((
            {'trigger': 'skip',
             'source': self.machine.state_init,
             'dest': self.machine.state_done,
             'prepare': self.no_mission_set},
        ))

    def add_mission(self, mission: AUVMission, mission_transitions: List):
        """ Adding AUVMission to control fsm

        Args:
            mission (AUVMission): mission object which name is the state in control fsm
            mission_transitions (List): transitions for this mission
        """
        self.machine.add_states(mission.name, on_enter=mission.run)
        self.machine.add_transitions(mission_transitions)

    def add_mission_transitions(self, mission_transitions: List):
        """ Adding custom transitions

        Args:
            mission_transitions (List): custom transitions
        """
        self.machine.add_transitions(mission_transitions)

    @abstractmethod
    def setup_missions(self):
        """ Add your custom AUVMission here

        Raises:
            NotImplementedError: you need to implement it in your child controller class
        """
        raise NotImplementedError

    def run(self):
        """ Start control fsm """
        self.machine.run()

    def no_mission_set(self):
        rospy.loginfo("No mission set!")
