
from abc import ABC, abstractmethod
from typing import List
from stingray_tfsm.auv_mission import AUVMission
from stingray_tfsm.core.pure_fsm import PureStateMachine


class AUVController(ABC):
    """ Control fsm class for AUV missions """
    @abstractmethod
    def __init__(self):
        self.state_init = self.__class__.__name__.upper() + "_INIT"
        """ default init FSM state"""
        self.state_aborted = self.__class__.__name__.upper() + "_ABORTED"
        """ default aborted FSM state"""
        self.state_done = self.__class__.__name__.upper() + "_DONE"
        """ default done FSM state"""
        self.transition_start = self.__class__.__name__.lower() + "_start"
        """ default start FSM transition"""
        self.transition_end = self.__class__.__name__.lower() + "_end"
        """ default end FSM transition"""
        self.default_states = (self.state_init,
                               self.state_aborted, self.state_done)
        """ default states for FSM """
        self.default_transitions = [
            [self.transition_end, '*', self.state_done]
        ]
        """ default transitions for FSM """
        self.master_fsm = PureStateMachine(
            self.default_states, self.default_transitions)
        """ pure state machine """

    @abstractmethod
    def add_mission(self, mission: AUVMission, mission_transitions: List):
        """ Adding AUVMission to control fsm

        Args:
            mission (AUVMission): mission object which name is the state in control fsm
            mission_transitions (List): transitions for this mission
        """
        self.master_fsm.add_state(mission.name, on_enter=mission.run)
        self.master_fsm.add_transitions(mission_transitions)
    
    @abstractmethod
    def add_mission_transitions(self, mission_transitions: List):
        """ Adding custom transitions

        Args:
            mission_transitions (List): custom transitions
        """
        self.master_fsm.add_transitions(mission_transitions)

    @abstractmethod
    def setup_missions(self):
        """ Add your custom AUVMission here

        Raises:
            NotImplementedError: you need to implement it in your child controller class
        """
        raise NotImplementedError

    @abstractmethod
    def run(self):
        """ Start control fsm """
        self.master_fsm.run()
