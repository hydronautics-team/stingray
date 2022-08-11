
from abc import ABC, abstractmethod
from typing import List
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.core.pure_fsm import PureStateMachine
import rospy


class AUVController(ABC):
    """ Control fsm class for AUV missions """
    @abstractmethod
    def __init__(self):
        transitions = (
            {'trigger': 'skip',
             'source': 'init',
             'dest': 'done',
             'prepare': self.no_mission_set},
        )
        self.machine = PureStateMachine(
            self.__class__.__name__, transitions=transitions)
        """ pure state machine """

    def add_mission(self, mission: AUVMission, mission_transitions: List):
        """ Adding AUVMission to control fsm

        Args:
            mission (AUVMission): mission object which name is the state in control fsm
            mission_transitions (List): transitions for this mission
        """
        self.machine.add_state(mission.name, on_enter=mission.run)
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
