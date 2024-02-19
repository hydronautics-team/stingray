from transitions.extensions.asyncio import AsyncMachine
import logging
from rclpy.node import Node
from pathlib import Path

from stingray_missions.event import TopicEvent
from stingray_utils.config import load_yaml


logger = logging.getLogger(__name__)


class StateAction():
    def __init__(self,
                 type: str = "",
                 action: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.action = action
        if kwargs:
            logger.warning(f"{self.type} state action unused kwargs: {kwargs}")
        logger.info(f"{self.type} state action desc loaded")


class TransitionEvent():
    def __init__(self,
                 type: str = "",
                 trigger: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.trigger = trigger
        if kwargs:
            logger.warning(f"{self.type} transition event unused kwargs: {kwargs}")
        logger.info(f"{self.type} transition event desc loaded")


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
            logger.warning(f"{self.name} state unused kwargs: {kwargs}")
        logger.info(f"{self.name} state desc loaded")


class MissionDescription:
    def __init__(self,
                 name: str = "",
                 initial_state: str = "",
                 states: dict[str, dict] = {},
                 transitions: list[dict] = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name.upper()
        self.initial_state = initial_state
        self.transitions = transitions
        self.states = {self.custom_state_name(s_name): StateDescription(name=self.custom_state_name(s_name), **s_params)
                       for s_name, s_params in states.items()}
        if kwargs:
            logger.warning(f"{self.name} mission unused kwargs: {kwargs}")
        logger.info(f"{self.name} mission desc loaded")

    def custom_state_name(self, name: str):
        return f"{self.name}|{name}".upper()


class ScenarioDescription:
    def __init__(self,
                 name: str = "",
                 initial_mission: str = "",
                 missions: dict[str, dict] = {},
                 transitions: list[dict] = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name.upper()
        self.transitions = transitions
        self.missions = {self.custom_mission_name(m_name): load_mission(custom_name=self.custom_mission_name(m_name), **m_params)
                         for m_name, m_params in missions.items()}
        self.initial_mission = self.custom_mission_name(initial_mission)

        if kwargs:
            logger.warning(f"{self.name} scenario unused kwargs: {kwargs}")
        logger.info(f"{self.name} scenario desc loaded")

    def custom_mission_name(self, name: str):
        return f"{self.name}|{name}".upper()


def load_mission(config_name: str, package_name="stingray_missions", custom_name: str = None) -> MissionDescription:
    if custom_name is None:
        custom_name = Path(config_name).stem
    return MissionDescription(name=custom_name, **load_yaml(config_path=f'configs/missions/{config_name}', package_name=package_name))


def load_scenario(config_name: str, package_name="stingray_missions", custom_name: str = None) -> ScenarioDescription:
    if custom_name is None:
        custom_name = Path(config_name).stem
    return ScenarioDescription(name=custom_name, **load_yaml(config_path=f'configs/scenarios/{config_name}', package_name=package_name))


class States:
    ALL = '*'
    IDLE = 'IDLE'
    SUCCESS = 'SUCCESS'
    FAILED = 'FAILED'


class Transitions:
    ok = 'ok'
    abort = 'abort'
    reset = 'reset'
    timeout = 'timeout'


class FSM(object):
    def __init__(self, node: Node):
        """Mission class for executing a mission from a config file"""
        self.node = node
        self.events: dict[str, TopicEvent] = {}
        self.expiration_timer = None

        logger.info(f"FSM created")

    def initialize(self, scenario: ScenarioDescription):
        """Registering the state machine from the config file"""

        states = [States.IDLE, States.SUCCESS, States.FAILED]
        transitions = [
            [Transitions.abort, States.ALL, States.FAILED],
            [Transitions.reset, States.ALL, States.IDLE],
            [Transitions.ok, States.IDLE, scenario.initial_state]]

        states += scenario["states"].keys()
        transitions += scenario["transitions"]

        self.machine = AsyncMachine(
            model=self,
            states=states,
            transitions=transitions,
            initial=States.IDLE,
            after_state_change="execute_state",
            before_state_change="leave_state",
        )

        self._parse_events(scenario["states"])

    def _parse_events(self, states: dict):
        for state, args in states.items():
            if args['transition_event']:
                try:
                    self._parse_event(state, args['transition_event'])
                except KeyError:
                    logger.error(
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
            logger.info(f"Added event {state_name}")
        else:
            raise ValueError("Event type not supported")

    async def on_enter_FAILED(self):
        """Executing on entering the FAILED state"""
        for event in self.events.values():
            await event.unsubscribe(self.node)
        logger.info("Mission failed")

    async def on_enter_SUCCESS(self):
        """Executing on entering the SUCCESS state"""
        for event in self.events.values():
            await event.unsubscribe(self.node)
        logger.info("Mission succeeded")

    async def execute_state(self):
        """Executing as soon as the state is entered"""
        logger.info(f"Executing {self.state}")
        logger.info(f"Transitions: {self.machine.get_triggers(self.state)}")
        if self.state in self.events:
            await self.events[self.state].subscribe(self.node)
        try:
            timeout_value = self.desc.states[self.state].timeout
            self.expiration_timer = self.node.create_timer(
                timeout_value, self.countdown)
            logger.info(
                f"State {self.state} will expire in {timeout_value} seconds")
        except KeyError:
            logger.info(f"No expiration time for state {self.state}")

        logger.info(f"{self.state} started")

    async def leave_state(self):
        """Executing before leaving the state"""
        try:
            await self.events[self.state].unsubscribe(self.node)
        except KeyError:
            logger.info(f"No event for state {self.state}")
        if self.expiration_timer and not self.expiration_timer.is_ready():
            self.expiration_timer.cancel()
            self.expiration_timer = None
            logger.info(f"Cancel expiration timer for {self.state}")
        logger.info(f"{self.state} ended")

    async def countdown(self):
        """Countdown for the mission"""
        logger.info(
            f"State {self.state} expired. Triggering {self.timeout}")
        await self.trigger(self.timeout)
