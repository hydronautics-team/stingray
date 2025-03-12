from pathlib import Path
from rclpy.node import Node
from rclpy.logging import get_logger

from stingray_missions.fsm_states import State
from stingray_missions.action import StateActionBase
from stingray_utils.config import load_yaml


class StateDescription():
    name: str
    timeout: float
    action_args: dict
    def __init__(self,
                 node: Node,
                 name: str = "",
                 timeout: float = None,
                 action: dict = None,
                 **kwargs):
        """State class"""
        self.name = name.upper()
        self.timeout = timeout
        if action:
            self.action_args = action
        else:
            self.action_args = None
        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} state unused kwargs: {kwargs}")

    def __repr__(self) -> str:
        return f"""
                        State: {self.name}
                            timeout: {self.timeout}
                            action: 
                                {self.action_args}
        """


class MissionDescription:
    def __init__(self,
                 node: Node,
                 name: str = "",
                 initial: str = "",
                 states: dict[str, dict] = {},
                 transitions: list[dict[str, str]] = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name.upper()
        self.initial_state = self._custom_state_name(initial)
        self.states = [StateDescription(node=node, name=self._custom_state_name(s_name), **s_params)
                       for s_name, s_params in states.items()]
        self.mission_transitions = []
        for transition in transitions:
            trigger = transition['trigger']
            if isinstance(transition['source'], list):
                source = [self._custom_state_name(
                    state) for state in transition['source']]
            else:
                if transition['source'] == State.ALL:
                    source = [state.name for state in self.states]
                else:
                    source = self._custom_state_name(transition['source'])
            if transition['dest'] in State.aslist():
                dest = transition['dest']
            else:
                dest = self._custom_state_name(transition['dest'])
            self.mission_transitions.append(
                {'trigger': trigger, 'source': source, 'dest': dest})

        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} mission unused kwargs: {kwargs}")

    def _custom_state_name(self, name: str):
        return f"{self.name}|{name}".upper()

    def __repr__(self) -> str:
        states_print = "\n".join(
            [f"{state}" for state in self.states])
        return f"""
                Mission: {self.name}
                    {states_print}
        """

    def set_OK_outcome(self, state: str):
        for transition in self.mission_transitions:
            if transition['dest'] == State.OK:
                transition['dest'] = state

    def set_FAILED_outcome(self, state: str):
        for transition in self.mission_transitions:
            if transition['dest'] == State.FAILED:
                transition['dest'] = state

    @staticmethod
    def load(node: Node, config_name: str, package_name="stingray_missions", custom_name: str = None) -> "MissionDescription":
        if custom_name is None:
            custom_name = Path(config_name).stem
        return MissionDescription(node=node, name=custom_name, **load_yaml(config_path=f"configs/missions/{config_name}", package_name=package_name))

    @property
    def transitions(self) -> list[list[str, str, str]]:
        return [[transition['trigger'], transition['source'], transition['dest']] for transition in self.mission_transitions]


class ScenarioDescription:
    def __init__(self,
                 node: Node,
                 name: str = "",
                 initial: str = "",
                 ok_state_package_name: str = "stingray_missions",
                 failed_state_package_name: str = "stingray_missions",
                 missions: dict[str, dict] = {},
                 transitions: list[dict] = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name.upper()
        self.scenario_transitions = transitions
        self.missions = {self._custom_mission_name(m_name): MissionDescription.load(node=node, custom_name=self._custom_mission_name(m_name), **m_params)
                         for m_name, m_params in missions.items()}
        default_missions = {
            self._custom_mission_name("OK"): MissionDescription.load(node=node, custom_name=self._custom_mission_name("OK"), config_name="ok", package_name=ok_state_package_name),
            self._custom_mission_name("FAILED"): MissionDescription.load(node=node, custom_name=self._custom_mission_name("FAILED"), config_name="failed", package_name=failed_state_package_name),
        }
        self.missions.update(default_missions)

        self.initial_mission = self._custom_mission_name(initial)

        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} scenario unused kwargs: {kwargs}")

    def _custom_mission_name(self, name: str):
        return f"{self.name}|{name}".upper()

    def __repr__(self) -> str:
        missions_print = "\n".join(
            [f"{mission}" for mission in self.missions.values()])

        return f"""
        Scenario: {self.name}
            {missions_print}
        """

    @staticmethod
    def load(node: Node, config_name: str, package_name="stingray_missions", custom_name: str = None) -> "ScenarioDescription":
        if custom_name is None:
            custom_name = Path(config_name).stem
        return ScenarioDescription(node=node, name=custom_name, **load_yaml(config_path=f"configs/scenarios/{config_name}", package_name=package_name))

    @property
    def initial_state(self) -> str:
        return self.missions[self.initial_mission].initial_state

    @property
    def states(self) -> list[StateDescription]:
        """Return all states in the scenario"""
        states = []
        for mission in self.missions.values():
            states.extend(mission.states)
        return states

    @property
    def transitions(self) -> list[list[str, str, str]]:
        scenario_transitions = [
            [self.name.lower(), State.IDLE, self.initial_state]
        ]
        for transition in self.scenario_transitions:
            source: str | list[str] = transition['source']
            outcome: str = transition['outcome']
            dest: str = transition['dest']
            if dest == State.IDLE:
                if isinstance(source, list):
                    for mission in source:
                        self.missions[self._custom_mission_name(mission)].set_OK_outcome(
                            State.IDLE)
                else:
                    self.missions[self._custom_mission_name(source)].set_OK_outcome(
                        State.IDLE)
            else:
                match outcome.upper():
                    case State.OK:
                        if isinstance(source, list):
                            for mission in source:
                                self.missions[self._custom_mission_name(mission)].set_OK_outcome(
                                    self.missions[self._custom_mission_name(dest)].initial_state)
                        else:
                            self.missions[self._custom_mission_name(source)].set_OK_outcome(
                                self.missions[self._custom_mission_name(dest)].initial_state)
                    case State.FAILED:
                        if isinstance(source, list):
                            for mission in source:
                                self.missions[self._custom_mission_name(mission)].set_FAILED_outcome(
                                    self.missions[self._custom_mission_name(dest)].initial_state)
                        else:
                            self.missions[self._custom_mission_name(source)].set_FAILED_outcome(
                                self.missions[self._custom_mission_name(dest)].initial_state)
                    case _:
                        raise ValueError(
                            f"Invalid outcome: {outcome} in scenario {self.name}")

        for mission in self.missions.values():
            scenario_transitions.extend(mission.transitions)

        return scenario_transitions
