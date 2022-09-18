
from abc import ABC, abstractmethod
from stingray_tfsm.core.pure_mission import PureMission
from stingray_tfsm.core.pure_fsm import PureStateMachine
import rospy


class PureMissionsController(ABC):
    """ Class for controlling missions """

    @abstractmethod
    def __init__(self, verbose: bool = False):
        """ Class for controlling missions.
        """
        self.verbose = verbose
        self.machine: PureStateMachine = None
        self.last_mission = None
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
        self._arrange_finish()
        if self.verbose:
            rospy.loginfo("Missions setup done")

    def _arrange_finish(self):
        if self.last_mission is None:
            self.machine.add_transitions((
                {'trigger': self.__class__.__name__ + '_skip',
                 'source': self.machine.state_init,
                 'dest': self.machine.state_end,
                 'prepare': self._no_mission_set},
            ))
        else:
            self.add_mission_transitions([
                [self.machine.transition_end,
                    self.last_mission, self.machine.state_end],
            ])

    def add_mission(self, mission: PureMission, mission_transitions: list = []):
        """ Adding PureMission to control fsm

        Args:
            mission (PureMission): mission object which name is the state in control fsm
            mission_transitions (List): transitions for this mission
        """
        self.machine.add_states(mission.name, on_enter=mission.run)
        if self.last_mission is None:
            self.add_mission_transitions([
                [self.machine.transition_start,
                    self.machine.state_init, mission.name],
            ])
            self.last_mission = mission.name
        else:
            self.add_mission_transitions([
                [f"FROM_{self.last_mission}_TO_{mission.name}",
                 self.last_mission, mission.name],
            ])
        self.last_mission = mission.name
        if self.verbose:
            rospy.loginfo(f'Added mission: {self.last_mission}')

        if mission_transitions:
            self.machine.add_transitions(mission_transitions)

    def add_mission_transitions(self, mission_transitions: list):
        """ Adding custom transitions

        Args:
            mission_transitions (List): custom transitions
        """
        self.machine.add_transitions(mission_transitions)
        if self.verbose:
            rospy.loginfo(f'Added transitions: {mission_transitions}')

    @abstractmethod
    def setup_missions(self):
        """ Add your custom PureMission here

        Raises:
            NotImplementedError: you need to implement it in your child controller class
        """
        raise NotImplementedError

    def run(self):
        """ Start control fsm """
        self.machine.run()

    def _no_mission_set(self):
        rospy.loginfo("No mission set!")
