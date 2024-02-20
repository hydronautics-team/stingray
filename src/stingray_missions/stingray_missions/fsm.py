# from transitions.extensions.asyncio import AsyncMachine
from transitions.extensions.factory import AsyncGraphMachine
import logging
from rclpy.node import Node
from rclpy.logging import get_logger
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from stingray_missions.event import TopicEvent
from stingray_utils.config import load_yaml, StingrayConfig
from stingray_interfaces.srv import TransitionSrv


class StateAction():
    def __init__(self,
                 type: str = "",
                 action: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.action = action
        if kwargs:
            get_logger("fsm").warning(
                f"{self.type} state action unused kwargs: {kwargs}")
        get_logger("fsm").info(f"{self.type} state action desc loaded")

    def __repr__(self) -> str:
        return f"type: {self.type}"


class TransitionEvent():
    def __init__(self,
                 type: str = "",
                 trigger: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.trigger = trigger
        if kwargs:
            get_logger("fsm").warning(
                f"{self.type} transition event unused kwargs: {kwargs}")
        get_logger("fsm").info(f"{self.type} transition event desc loaded")

    def __repr__(self) -> str:
        return f"type: {self.type}, trigger: {self.trigger}"


class StateDescription():
    def __init__(self,
                 name: str = "",
                 transition_event: dict = {},
                 timeout: float = 5,
                 action: dict = {},
                 **kwargs):
        """State class"""
        self.name = name.upper()
        self.transition_event = TransitionEvent(**transition_event)
        self.timeout = timeout
        self.action = StateAction(**action)
        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} state unused kwargs: {kwargs}")
        get_logger("fsm").info(f"{self.name} state desc loaded")

    def __repr__(self) -> str:
        return f"""
                        State: {self.name}
                            timeout: {self.timeout}
                            transition event: 
                                {self.transition_event}
                            action: 
                                {self.action}
        """


class MissionDescription:
    def __init__(self,
                 name: str = "",
                 initial: str = "",
                 states: dict[str, dict] = {},
                 transitions: list[dict[str, str]] = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name.upper()
        self.initial_state = self._custom_state_name(initial)
        self.states = [StateDescription(name=self._custom_state_name(s_name), **s_params)
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
        get_logger("fsm").info(f"{self.name} mission desc loaded")

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

    @property
    def transitions(self) -> list[list[str, str, str]]:
        return [[transition['trigger'], transition['source'], transition['dest']] for transition in self.mission_transitions]


class ScenarioDescription:
    def __init__(self,
                 name: str = "",
                 initial: str = "",
                 missions: dict[str, dict] = {},
                 transitions: list[dict] = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name.upper()
        self.scenario_transitions = transitions
        self.missions = {self._custom_mission_name(m_name): load_mission(custom_name=self._custom_mission_name(m_name), **m_params)
                         for m_name, m_params in missions.items()}
        self.initial_mission = self._custom_mission_name(initial)

        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} scenario unused kwargs: {kwargs}")
        get_logger("fsm").info(f"{self.name} scenario desc loaded")

    def _custom_mission_name(self, name: str):
        return f"{self.name}|{name}".upper()

    def __repr__(self) -> str:
        missions_print = "\n".join(
            [f"{mission}" for mission in self.missions.values()])

        return f"""
        Scenario: {self.name}
            {missions_print}
        """

    @property
    def initial_state(self) -> str:
        return self.missions[self.initial_mission].initial_state

    @property
    def states(self) -> list[str]:
        """Return all states in the scenario"""
        states = []
        for mission in self.missions.values():
            states.extend([state.name for state in mission.states])
        return states

    @property
    def transitions(self) -> list[list[str, str, str]]:
        scenario_transitions = [
            [self.name.lower(), State.IDLE, self.initial_state]
        ]
        for transition in self.scenario_transitions:
            source = transition['source']
            outcome = transition['outcome']
            dest = transition['dest']
            if dest not in State.aslist():
                if outcome == State.OK:
                    if isinstance(source, list):
                        for mission in source:
                            self.missions[self._custom_mission_name(mission)].set_OK_outcome(
                                self.missions[self._custom_mission_name(dest)].initial_state)
                    else:
                        self.missions[self._custom_mission_name(source)].set_OK_outcome(
                            self.missions[self._custom_mission_name(dest)].initial_state)
                elif outcome == State.FAILED:
                    if isinstance(source, list):
                        for mission in source:
                            self.missions[self._custom_mission_name(mission)].set_FAILED_outcome(
                                self.missions[self._custom_mission_name(dest)].initial_state)
                    else:
                        self.missions[self._custom_mission_name(source)].set_FAILED_outcome(
                            self.missions[self._custom_mission_name(dest)].initial_state)
        for mission in self.missions.values():
            scenario_transitions.extend(mission.transitions)

        return scenario_transitions


class State:
    ALL = "*"
    IDLE = "IDLE"
    OK = "OK"
    FAILED = "FAILED"
    ABORTED = "ABORTED"

    @staticmethod
    def aslist():
        return [State.ALL, State.IDLE, State.OK, State.FAILED, State.ABORTED]


class Transition:
    ok = "ok"
    fail = "fail"
    abort = "abort"
    reset = "reset"
    timeout = "timeout"


class FSM(object):
    def __init__(self, node: Node, scenarios_packages: list[str]):
        """FSM class for executing scenarios and missions"""
        self.node = node
        self.pending_transition = None
        self.events: dict[str, TopicEvent] = {}
        self.expiration_timer = None

        self.transition_srv = self.node.create_service(
            TransitionSrv, StingrayConfig.ros.services["stingray_missions"]["transition"], self._transition_callback)

        self._initialize_machine(scenarios_packages)

        get_logger("fsm").info(f"FSM created")

    def _initialize_machine(self, scenarios_packages: list[str]):
        self.machine = AsyncGraphMachine(
            model=self,
            states=[State.IDLE, State.OK, State.FAILED, State.ABORTED],
            initial=State.IDLE,
            auto_transitions=False,
            after_state_change="execute_state",
            before_state_change="leave_state",
        )

        self._register_scenarios_from_packages(
            package_names=scenarios_packages)

        global_transitions = [
            [Transition.abort, State.ALL, State.ABORTED],
            [Transition.reset, [State.ABORTED, State.FAILED, State.OK], State.IDLE],
        ]
        self.machine.add_transitions(global_transitions)

    def _transition_callback(self, request: TransitionSrv.Request, response: TransitionSrv.Response):
        self.pending_transition = request.transition
        response.ok = True

        return response

    async def process_pending_transition(self):
        if self.pending_transition:
            pending = self.pending_transition
            self.pending_transition = None
            if pending in self.machine.get_triggers(self.state):
                await self.trigger(pending)
            else:
                get_logger("fsm").error(
                    f"Transition {pending} not found in {self.state}. Valid transitions: {self.machine.get_triggers(self.state)}")

    def _register_scenarios_from_packages(self, package_names: list[str]):
        """Registering scenarios from packages"""
        for package_name in package_names:
            pakage_path = get_package_share_directory(package_name)
            configs = Path(pakage_path, "configs/scenarios").glob("*.yaml")
            for config in configs:
                scenario = load_scenario(
                    config_name=config.name, package_name=package_name)
                self.machine.add_states(scenario.states)
                self.machine.add_transitions(scenario.transitions)
                # self._register_events(scenario.events)
                get_logger("fsm").info(f"Registered scenario {scenario}")

    def _register_events(self, states: dict):
        for state, args in states.items():
            if args["transition_event"]:
                try:
                    self._parse_event(state, args["transition_event"])
                except KeyError:
                    get_logger("fsm").error(
                        f"Event {args['transition_event']} not found in events list")

    def _parse_event(self, state_name: str, args: dict):
        """Registering events from the config file"""
        if args["type"] == "TopicEvent":
            self.events[state_name] = TopicEvent(
                trigger_fn=self.trigger,
                topic=args["topic_name"],
                data=args["data"],
                trigger=args["trigger"],
                count=args["count"],
            )
            get_logger("fsm").info(f"Added event {state_name}")
        else:
            raise ValueError("Event type not supported")

    async def on_enter_FAILED(self):
        """Executing on entering the FAILED state"""
        for event in self.events.values():
            await event.unsubscribe(self.node)
        get_logger("fsm").info("Mission failed")

    async def on_enter_SUCCESS(self):
        """Executing on entering the SUCCESS state"""
        for event in self.events.values():
            await event.unsubscribe(self.node)
        get_logger("fsm").info("Mission succeeded")

    async def execute_state(self):
        """Executing as soon as the state is entered"""
        get_logger("fsm").info(f"{self.state} executing ... Transitions: {self.machine.get_triggers(self.state)}")
        if self.state in self.events:
            await self.events[self.state].subscribe(self.node)
        # try:
        #     timeout_value = self.desc.states[self.state].timeout
        #     self.expiration_timer = self.node.create_timer(
        #         timeout_value, self.countdown)
        #     get_logger("fsm").info(
        #         f"State {self.state} will expire in {timeout_value} seconds")
        # except KeyError:
        #     get_logger("fsm").info(
        #         f"No expiration time for state {self.state}")

    async def leave_state(self):
        """Executing before leaving the state"""
        try:
            await self.events[self.state].unsubscribe(self.node)
        except KeyError:
            get_logger("fsm").info(f"No event for state {self.state}")
        if self.expiration_timer and not self.expiration_timer.is_ready():
            self.expiration_timer.cancel()
            self.expiration_timer = None
            get_logger("fsm").info(f"Cancel expiration timer for {self.state}")
        get_logger("fsm").info(f"{self.state} ended")

    async def countdown(self):
        """Countdown for the mission"""
        get_logger("fsm").info(
            f"State {self.state} expired. Triggering {self.timeout}")
        await self.trigger(self.timeout)

    def draw(self):
        self.machine.get_combined_graph().draw("fsm_graph.png", prog="dot")
        get_logger("fsm").info(f"FSM graph saved to fsm_graph.png")


def load_mission(config_name: str, package_name="stingray_missions", custom_name: str = None) -> MissionDescription:
    if custom_name is None:
        custom_name = Path(config_name).stem
    return MissionDescription(name=custom_name, **load_yaml(config_path=f"configs/missions/{config_name}", package_name=package_name))


def load_scenario(config_name: str, package_name="stingray_missions", custom_name: str = None) -> ScenarioDescription:
    if custom_name is None:
        custom_name = Path(config_name).stem
    return ScenarioDescription(name=custom_name, **load_yaml(config_path=f"configs/scenarios/{config_name}", package_name=package_name))
