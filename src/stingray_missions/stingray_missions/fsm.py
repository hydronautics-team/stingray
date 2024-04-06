from __future__ import annotations
from transitions.extensions.factory import AsyncGraphMachine
import asyncio
from rclpy.node import Node
from rclpy.logging import get_logger
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from stingray_missions.event import StringEvent, ObjectDetectionEvent, SubscriptionEvent
from stingray_missions.action import StateAction, create_action
from stingray_utils.config import load_yaml
from stingray_interfaces.srv import SetTransition
from stingray_core_interfaces.msg import UVState


class TransitionEvent():
    def __init__(self,
                 type: str = "",
                 topic_name: str = "",
                 data: str = "",
                 count: int = 0,
                 trigger: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.topic_name = topic_name
        self.data = data
        self.count = count
        self.trigger = trigger
        if kwargs:
            get_logger("fsm").warning(
                f"{self.type} transition event unused kwargs: {kwargs}")

    def __repr__(self) -> str:
        return f"type: {self.type}, trigger: {self.trigger}"


class StateDescription():
    def __init__(self,
                 node: Node,
                 name: str = "",
                 transition_event: dict = None,
                 timeout: float = None,
                 action: dict | StateAction = None,
                 **kwargs):
        """State class"""
        self.name = name.upper()
        if transition_event:
            self.transition_event = TransitionEvent(**transition_event)
        else:
            self.transition_event = None
        self.timeout = timeout
        if action:
            if isinstance(action, dict):
                self.action = create_action(node=node, action=action)
            elif isinstance(action, StateAction):
                self.action = action
            else:
                raise NotImplementedError(
                    f"Action type {action} not implemented")
        else:
            self.action = None
        if kwargs:
            get_logger("fsm").warning(
                f"{self.name} state unused kwargs: {kwargs}")

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
        self.missions = {self._custom_mission_name(m_name): load_mission(node=node, custom_name=self._custom_mission_name(m_name), **m_params)
                         for m_name, m_params in missions.items()}
        default_missions = {
            self._custom_mission_name("OK"): load_mission(node=node, custom_name=self._custom_mission_name("OK"), config_name="ok", package_name=ok_state_package_name),
            self._custom_mission_name("FAILED"): load_mission(node=node, custom_name=self._custom_mission_name("FAILED"), config_name="failed", package_name=failed_state_package_name),
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
            source = transition['source']
            outcome = transition['outcome']
            dest = transition['dest']
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

    @staticmethod
    def aslist():
        return [State.ALL, State.IDLE, State.OK, State.FAILED]

    @staticmethod
    def state_description(node: Node, state: str):
        if state in State.aslist():
            return StateDescription(node=node, name=state)
        else:
            raise ValueError(f"Invalid state: {state}")
        return None


class Transition:
    ok = "ok"
    fail = "fail"
    reset = "reset"
    timeout = "timeout"


class FSM(object):
    def __init__(self, node: Node, scenarios_packages: list[str]):
        """FSM class for executing scenarios and missions"""
        self.node = node
        self.pending_transition = None
        self.pending_action: StateAction = None
        self.registered_states: dict[str, StateDescription] = {}
        self.events: dict[str, SubscriptionEvent] = {}
        self.expiration_timer = None
        self.wait_action_success_event = asyncio.Event()

        # TODO kostyl
        self.flare_sequence: list[str] = ["Y", "R", "B"]

        self.lock_coroutine = asyncio.Lock()
        self.node.declare_parameter(
            'uv_state_topic', '/stingray/topics/uv_state')
        self.node.declare_parameter('twist_action', '/stingray/actions/twist')
        self.node.declare_parameter(
            'bbox_search_twist_action', '/stingray/actions/bbox_search_twist')
        self.node.declare_parameter(
            'bbox_centering_twist_action', '/stingray/actions/bbox_centering_twist')
        self.node.declare_parameter(
            'device_action', '/stingray/actions/device')
        self.node.declare_parameter(
            'reset_imu_srv', '/stingray/services/reset_imu')
        self.node.declare_parameter(
            'transition_srv', '/stingray/services/transition')
        self.node.declare_parameter(
            'set_stabilization_srv', '/stingray/services/set_stabilization')
        self.node.declare_parameter(
            'enable_object_detection_topic', '/stingray/topics/enable_object_detection')

        self.transition_srv = self.node.create_service(
            SetTransition, self.node.get_parameter('transition_srv').get_parameter_value().string_value, self._transition_callback)
        self.uv_state_sub = self.node.create_subscription(
            UVState,
            self.node.get_parameter(
                'uv_state_topic').get_parameter_value().string_value,
            self._uv_state_callback,
            1)

        self._initialize_machine(scenarios_packages)

        get_logger("fsm").info(f"FSM created")

    def _initialize_machine(self, scenarios_packages: list[str]):
        self.machine = AsyncGraphMachine(
            model=self,
            states=[State.IDLE, State.OK, State.FAILED],
            initial=State.IDLE,
            auto_transitions=False,
            after_state_change="execute_state",
            before_state_change="leave_state",
        )

        self._register_scenarios_from_packages(
            package_names=scenarios_packages)

        # add global transitions
        global_transitions = [
            [Transition.reset, [State.FAILED, State.OK], State.IDLE],
            [Transition.fail, State.ALL, State.FAILED],
        ]
        self.machine.add_transitions(global_transitions)

        # remember global states
        self.registered_states[State.IDLE] = State.state_description(
            node=self.node,
            state=State.IDLE)
        self.registered_states[State.FAILED] = State.state_description(
            node=self.node,
            state=State.FAILED)
        self.registered_states[State.OK] = State.state_description(
            node=self.node,
            state=State.OK)

    def _uv_state_callback(self, uv_state: UVState):
        get_logger("fsm").info(f"uv_state.flare_seq: {uv_state.flare_seq}")
        unpacked = [chr(i) for i in uv_state.flare_seq]
        if "R" in unpacked and "Y" in unpacked and "B" in unpacked:
            self.flare_sequence = unpacked
        elif "F" in unpacked:
            self.add_pending_transition(Transition.fail)
        else:
            self.flare_sequence = ["Y", "R", "B"]

    def _transition_callback(self, request: SetTransition.Request, response: SetTransition.Response):
        self.add_pending_transition(request.transition)
        response.ok = True

        return response

    def add_pending_transition(self, transition: str):
        if not self.pending_transition:
            self.pending_transition = transition
            get_logger("fsm").info(
                f"Added pending transition {self.pending_transition}")
        else:
            get_logger("fsm").error(
                f"FSM already has a pending transition {self.pending_transition}")

    async def process_pending_transition(self):
        if self.pending_transition:
            if self.pending_transition in self.machine.get_triggers(self.state):
                await self.trigger(self.pending_transition)
            else:
                get_logger("fsm").error(
                    f"Transition {self.pending_transition} not found in {self.state}. Valid transitions: {self.machine.get_triggers(self.state)}")
                self.pending_transition = None

    def add_pending_action(self, action: StateAction):
        if not self.pending_action:
            self.pending_action = action
            # get_logger("fsm").info(
            #     f"Added pending action {self.pending_action}")
        else:
            get_logger("fsm").error(
                f"FSM already has a pending action {self.pending_action}")

    async def process_pending_action(self):
        if self.pending_action:
            self.wait_action_success_event.clear()
            get_logger("fsm").info(
                f"{self.pending_action.type} executing: {self.pending_action}")
            # TODO kostyl
            if self.pending_action.type == "SequencePunchBboxTwist":
                self.pending_action.sequence = self.flare_sequence
            result = await self.pending_action.execute()
            get_logger("fsm").info(
                f"{self.pending_action.type} result: {result}, stopped: {self.pending_action.stopped}")
            if not self.pending_action.stopped:
                if result:
                    self.add_pending_transition(Transition.ok)
                else:
                    self.add_pending_transition(Transition.fail)
            self.pending_action = None
            self.wait_action_success_event.set()

    def _register_scenarios_from_packages(self, package_names: list[str]):
        """Registering scenarios from packages"""
        for package_name in package_names:
            pakage_path = get_package_share_directory(package_name)
            configs = Path(pakage_path, "configs/scenarios").glob("*.yaml")
            for config in configs:
                scenario = load_scenario(
                    node=self.node,
                    config_name=config.name,
                    package_name=package_name)
                self.machine.add_states(
                    [state.name for state in scenario.states])
                self.registered_states.update(
                    {state.name: state for state in scenario.states})
                self.machine.add_transitions(scenario.transitions)
                self._register_events(scenario.states)
                # get_logger("fsm").info(f"Registered scenario {scenario}")

    def _register_events(self, states: list[StateDescription]):
        for state in states:
            if state.transition_event:
                if state.transition_event.type == "StringEvent":
                    self.events[state.name] = StringEvent(
                        transition_fn=self.add_pending_transition,
                        topic=state.transition_event.topic_name,
                        data=state.transition_event.data,
                        trigger=state.transition_event.trigger,
                        count=state.transition_event.count,
                    )
                    # get_logger("fsm").info(
                    #     f"Event {state.transition_event.type} registered for state {state.name}")
                # TODO
                elif state.transition_event.type == "ObjectDetectionEvent":
                    self.events[state.name] = ObjectDetectionEvent(
                        transition_fn=self.add_pending_transition,
                        topic=state.transition_event.topic_name,
                        data=state.transition_event.data,
                        trigger=state.transition_event.trigger,
                        count=state.transition_event.count,
                    )
                    # get_logger("fsm").info(f"Event {state.transition_event.type} registered for state {state.name}")
                else:
                    raise ValueError(
                        f"Event type not supported: {state.transition_event.type}")

    async def on_enter_FAILED(self):
        """Executing on entering the FAILED state"""
        for event in self.events.values():
            event.unsubscribe(self.node)
        get_logger("fsm").info("Mission FAILED")

    async def on_enter_OK(self):
        """Executing on entering the OK state"""
        for event in self.events.values():
            event.unsubscribe(self.node)
        get_logger("fsm").info("Mission OK")

    async def execute_state(self):
        """Executing as soon as the state is entered"""
        get_logger("fsm").info(
            f"{self.state} executing ... Transitions: {self.machine.get_triggers(self.state)}")
        self.pending_transition = None

        # register event
        if self.state in self.events:
            self.events[self.state].subscribe(self.node)

        # register timeout
        if Transition.timeout in self.machine.get_triggers(self.state) and self.registered_states[self.state].timeout:
            get_logger("fsm").info(
                f"Register timeout for {self.registered_states[self.state].timeout} seconds")
            self.expiration_timer = self.node.create_timer(
                self.registered_states[self.state].timeout, self._state_expired)
        if self.registered_states[self.state].action:
            get_logger("fsm").info(
                f"Register action for {self.state}")
            self.add_pending_action(self.registered_states[self.state].action)

        if self.state == State.OK or self.state == State.FAILED:
            self.add_pending_transition(Transition.reset)

    async def leave_state(self):
        """Executing before leaving the state"""
        get_logger("fsm").info(f"Start leaving {self.state}")

        # unsubscribe event
        if self.state in self.events:
            self.events[self.state].unsubscribe(self.node)
        # stop expiration timer
        if self.expiration_timer:
            if not self.expiration_timer.is_ready():
                self.expiration_timer.cancel()
                get_logger("fsm").info(
                    f"Cancel expiration timer for {self.state}")
            self.node.destroy_timer(self.expiration_timer)
            self.expiration_timer = None
        # stop action and wait until executed
        if self.pending_action and not self.pending_action.executed:
            self.pending_action.stop()
            try:
                await asyncio.wait_for(self.wait_action_success_event.wait(), timeout=5)
            except asyncio.TimeoutError:
                get_logger("fsm").error(
                    f"Stopping {self.pending_action.type} timed out")
                self.pending_action = None
                self.add_pending_transition(Transition.timeout)
            self.wait_action_success_event.clear()

        get_logger("fsm").info(f"{self.state} ended")

    async def _state_expired(self):
        """Countdown for the mission"""
        get_logger("fsm").info(
            f"State {self.state} expired!")
        self.add_pending_transition(Transition.timeout)

    def draw(self):
        self.machine.get_combined_graph().draw("fsm_graph.png", prog="dot")
        get_logger("fsm").info(f"FSM graph saved to fsm_graph.png")


def load_mission(node: Node, config_name: str, package_name="stingray_missions", custom_name: str = None) -> MissionDescription:
    if custom_name is None:
        custom_name = Path(config_name).stem
    return MissionDescription(node=node, name=custom_name, **load_yaml(config_path=f"configs/missions/{config_name}", package_name=package_name))


def load_scenario(node: Node, config_name: str, package_name="stingray_missions", custom_name: str = None) -> ScenarioDescription:
    if custom_name is None:
        custom_name = Path(config_name).stem
    return ScenarioDescription(node=node, name=custom_name, **load_yaml(config_path=f"configs/scenarios/{config_name}", package_name=package_name))
