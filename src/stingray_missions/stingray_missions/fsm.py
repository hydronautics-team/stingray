from __future__ import annotations
from transitions.extensions.factory import AsyncGraphMachine
import asyncio
from rclpy.node import Node
from rclpy.logging import get_logger
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from stingray_missions.fsm_states import State, Transition
from stingray_missions.action import StateActionBase
from stingray_missions.descriptions import StateDescription, ScenarioDescription, MissionDescription
from stingray_interfaces.srv import SetTransition


class FSM(object):
    def __init__(self,
                 node: Node,
                 scenarios_packages: list[str],
                 actions: dict[str, StateActionBase]
                 ):
        """FSM class for executing scenarios and missions"""
        self.node = node
        self.pending_transition = None
        self.pending_action: dict = None
        self.registered_states: dict[str, StateDescription] = {}
        self.registered_actions: dict[str, StateActionBase] = actions

        get_logger("fsm").info(f"Registering actions: {list(self.registered_actions.keys())}")

        self.expiration_timer = None
        self.wait_action_success_event = asyncio.Event()
        self.lock_coroutine = asyncio.Lock()

        self.transition_srv = self.node.create_service(
            SetTransition, self.node.get_parameter('transition_srv').get_parameter_value().string_value, self._transition_callback)

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
        ]
        self.machine.add_transitions(global_transitions)

        # remember global states
        self.registered_states[State.IDLE] = StateDescription(State.IDLE)
        self.registered_states[State.FAILED] = StateDescription(State.FAILED)
        self.registered_states[State.OK] = StateDescription(State.OK)
        
        self.draw()

        get_logger("fsm").info(f"FSM created")

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

    def set_ok(self):
        self.add_pending_transition(Transition.ok)

    def set_failed(self):
        self.add_pending_transition(Transition.fail)

    def set_timeout(self):
        self.add_pending_transition(Transition.timeout)

    async def process_pending_transition(self):
        if self.pending_transition:
            if self.pending_transition in self.machine.get_triggers(self.state):
                await self.trigger(self.pending_transition)
            else:
                get_logger("fsm").error(
                    f"Transition {self.pending_transition} not found in {self.state}. Valid transitions: {self.machine.get_triggers(self.state)}")
                self.pending_transition = None

    def add_pending_action(self, action_args: dict):
        if not self.pending_action:
            self.pending_action = action_args
            # get_logger("fsm").info(
            #     f"Added pending action {self.pending_action}")
        else:
            get_logger("fsm").error(
                f"FSM already has a pending action {self.pending_action}")

    async def process_pending_action(self):
        if self.pending_action:
            if self.registered_actions is None:
                get_logger("fsm").error(
                    f"No actions registered")
                self.pending_action = None
                self.set_failed()
                return
            
            get_logger("fsm").info(
                f'{self.pending_action["type"]} executing: {self.pending_action}')
            if not "type" in self.pending_action:
                get_logger("fsm").warning(
                    f'No action type in {self.pending_action}')
                self.set_failed()
                return
            
            self.wait_action_success_event.clear()
            if self.pending_action["type"] in self.registered_actions:

                action = self.registered_actions[self.pending_action["type"]]
                result = await action.execute(**self.pending_action)
                get_logger("fsm").info(
                    f"{action.type} result: {result}, stopped: {action.stopped}")
                if not action.stopped:
                    if result:
                        self.add_pending_transition(Transition.ok)
                    else:
                        self.add_pending_transition(Transition.fail)
                action.stopped = False
                action.executed = False
                self.pending_action = None
                self.wait_action_success_event.set()
            else:
                get_logger("fsm").error(
                    f'Action {self.pending_action["type"]} not found')
                self.set_failed()
                return

    def _register_scenarios_from_packages(self, package_names: list[str]):
        """Registering scenarios from packages"""
        custom_transitions = []
        custom_states = []

        for package_name in package_names:
            pakage_path = get_package_share_directory(package_name)
            configs = Path(pakage_path, "configs/scenarios").glob("*.yaml")

            for config in configs:
                scenario = ScenarioDescription.load(
                    config_name=config.name,
                    package_name=package_name)
                get_logger("fsm").info(
                    f"Registering scenario {scenario.name}")
                self.registered_states.update(
                    {state.name: state for state in scenario.states})
                custom_states += [state.name for state in scenario.states]
                custom_transitions += scenario.transitions
            
            if package_name in ScenarioDescription.loaded_oks:
                custom_states += [state.name for state in ScenarioDescription.loaded_oks[package_name].states]
                custom_transitions += ScenarioDescription.loaded_oks[package_name].transitions
            if package_name in ScenarioDescription.loaded_faileds:
                custom_states += [state.name for state in ScenarioDescription.loaded_faileds[package_name].states]
                custom_transitions += ScenarioDescription.loaded_faileds[package_name].transitions
                
        self.machine.add_states(custom_states)
        self.machine.add_transitions(
            custom_transitions
        )

    async def execute_state(self):
        """Executing as soon as the state is entered"""
        get_logger("fsm").info(
            f"{self.state} executing ... Transitions: {self.machine.get_triggers(self.state)}")
        self.pending_transition = None

        # register timeout
        if Transition.timeout in self.machine.get_triggers(self.state) and self.registered_states[self.state].timeout:
            get_logger("fsm").info(
                f"Register timeout for {self.registered_states[self.state].timeout} seconds")
            self.expiration_timer = self.node.create_timer(
                self.registered_states[self.state].timeout, self._state_expired)
        if self.registered_states[self.state].action_args:
            get_logger("fsm").info(
                f"Register action for {self.state}")
            self.add_pending_action(
                self.registered_states[self.state].action_args)

        if self.state == State.OK or self.state == State.FAILED:
            self.add_pending_transition(Transition.reset)

    async def leave_state(self):
        """Executing before leaving the state"""
        get_logger("fsm").info(f"Start leaving {self.state}")

        # stop expiration timer
        if self.expiration_timer:
            if not self.expiration_timer.is_ready():
                self.expiration_timer.cancel()
                get_logger("fsm").info(
                    f"Cancel expiration timer for {self.state}")
            self.node.destroy_timer(self.expiration_timer)
            self.expiration_timer = None

        # stop action and wait until executed
        if self.pending_action:
            if self.registered_actions is None:
                get_logger("fsm").error(
                    f"No actions registered")
                self.pending_action = None
                return
            
            get_logger("fsm").info(
                    f'{self.pending_action["type"]} stopping: {self.pending_action}')
            if self.pending_action["type"] in self.registered_actions:
                action = self.registered_actions[self.pending_action["type"]]
                if not action.executed:
                    action.stop()
                try:
                    await asyncio.wait_for(self.wait_action_success_event.wait(), timeout=1)
                except asyncio.TimeoutError:
                    get_logger("fsm").error(
                        f"Stopping {action.type} timed out")
                action.stopped = False
                action.executed = False
                self.pending_action = None
                self.wait_action_success_event.clear()
            else:
                get_logger("fsm").error(
                    f'Action {self.pending_action["type"]} not found')
                return

        get_logger("fsm").info(f"{self.state} ended")

    async def _state_expired(self):
        """Countdown for the mission"""
        get_logger("fsm").info(
            f"State {self.state} expired!")
        self.set_timeout()

    def draw(self):
        self.machine.get_combined_graph().draw("fsm_graph.png", prog="dot")
        get_logger("fsm").info(f"FSM graph saved to fsm_graph.png")
