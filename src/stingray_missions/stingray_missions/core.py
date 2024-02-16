from transitions.extensions.asyncio import AsyncMachine
import logging
from rclpy.node import Node

from stingray_missions.event import TopicEvent


logger = logging.getLogger(__name__)


class StateAction():
    def __init__(self,
                 type: str = "",
                 action: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.action = action
        self.kwargs = kwargs
        logger.info(f"StateAction Unused kwargs: {kwargs}")


class TransitionEvent():
    def __init__(self,
                 type: str = "",
                 trigger: str = "",
                 **kwargs):
        """State class"""
        self.type = type
        self.trigger = trigger
        self.kwargs = kwargs
        logger.info(f"TransitionEvent Unused kwargs: {kwargs}")


class StateDescription():
    def __init__(self,
                 transition_event: dict = {},
                 expire_time: float = 5,
                 action: dict = {},
                 **kwargs):
        """State class"""
        self.transition_event = TransitionEvent(**transition_event)
        self.expire_time = expire_time
        self.action = StateAction(**action)
        logger.info(f"StateDescription Unused kwargs: {kwargs}")


class MissionDescription:
    def __init__(self,
                 name: str = "",
                 initial_state: str = "",
                 states: dict = {},
                 **kwargs
                 ):
        """Mission class for executing a mission from a config file"""
        self.name = name
        self.initial_state = initial_state
        self.states = {k.upper(): StateDescription(**v)
                       for k, v in states.items()}
        logger.info(f"MissionDescription Unused kwargs: {kwargs}")


class FSM(object):
    def __init__(self, node: Node, mission_description: dict):
        """Mission class for executing a mission from a config file"""
        self.node = node
        self.desc = MissionDescription(**mission_description)
        self.events: dict[str, TopicEvent] = {}
        self.expiration_timer = None

        self._setup_fsm(mission_description)
        logger.info(f"Mission {self.desc.name} created")

    def _setup_fsm(self, mission_description: dict):
        """Registering the state machine from the config file"""

        self.IDLE_STATE = 'IDLE'
        self.SUCCESS_STATE = 'SUCCESS'
        self.FAILURE_STATE = 'FAILURE'
        self.go_transition = 'go'
        self.ok_transition = 'ok'
        self.abort_transition = 'abort'
        self.reset_transition = 'reset'
        self.timeout_transition = 'timeout'

        states = [self.IDLE_STATE, self.SUCCESS_STATE, self.FAILURE_STATE]
        transitions = [
            [self.abort_transition, "*", self.FAILURE_STATE],
            [self.reset_transition, "*", self.IDLE_STATE],
            [self.go_transition, self.IDLE_STATE, mission_description['initial_state']]]

        states += mission_description["states"].keys()
        transitions += mission_description["transitions"]

        self.machine = AsyncMachine(
            model=self,
            states=states,
            transitions=transitions,
            initial=self.IDLE_STATE,
            auto_transitions=False,
            after_state_change="execute_state",
            before_state_change="leave_state",
        )

        self._parse_events(mission_description["states"])

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

    async def on_enter_FAILURE(self):
        """Executing on entering the FAILURE state"""
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
            timeout_value = self.desc.states[self.state].expire_time
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
            f"State {self.state} expired. Triggering {self.timeout_transition}")
        await self.trigger(self.timeout_transition)
