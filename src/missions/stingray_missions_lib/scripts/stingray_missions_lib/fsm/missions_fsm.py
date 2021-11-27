import smach
from abc import ABCMeta, abstractmethod

"""@package docstring
Contains classes for implementing missions in Stingray missions infrastructure.
"""


class Mission(smach.State):
    """Base class for implementing missions in Stingray.

    Each mission is actually a state in a SMACH based Finite State Machine (FSM). Each mission has a name, and
    FSM outcomes names are generated from this name by doing upper case and adding "_OK" and "_FAIL" postfixes
    for successful mission cancellation and mission failure correspondingly.
    """

    __metaclass__ = ABCMeta

    def __init__(self, mission_name: str):
        """The constructor.

        :param mission_name: Name of the mission used to generate outcome names.
        """
        self.outcome_ok = mission_name.upper() + "_OK"
        self.outcome_failed = mission_name.upper() + "_FAILED"
        smach.State.__init__(self, outcomes=[self.outcome_ok, self.outcome_failed])

    @abstractmethod
    def execute(self, userdata):
        """Execute method from smach.State class. This method is used to run mission logic.

        :param userdata: Data from SMACH state transitions, usually not used.
        :return: Resulting state outcome. Return self.outcome_ok if mission is successfully completed, return
        self.outcome_failed otherwise.
        """
        pass


class MissionsFSMFactory(smach.StateMachine):
    """Factory (in terms of standard OOP patterns) that creates SMACH Finite State Machine (FSM) that
    defines missions and transitions between them. This FSM is then used in parental Stingray top level FSM.
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        """The constructor. Generates names for success outcome and fail outcome.
        """
        self.outcome_ok = "MISSIONS_OK"
        self.outcome_failed = "MISSIONS_FAILED"

    @abstractmethod
    def create_fsm(self):
        """Creates SMACH FSM with missions. This FSM must return only self.outcome_ok and self.outcome_failed
        outcomes.

        For more information on how to create SMACH FSM, please refer documentation: http://wiki.ros.org/smach

        :return: SMACH FSM with missions.
        """
        pass

# TODO: MissionsFSMFactory that receives just a sequence of missions and creates FSM automatically

