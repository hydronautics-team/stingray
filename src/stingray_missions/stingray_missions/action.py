from rclpy.logging import get_logger
import time
import asyncio
from rclpy.node import Node
from stingray_utils.acyncio import AsyncActionClient
from stingray_interfaces.action import TwistAction


class StateAction():
    def __init__(self,
                 node: Node,
                 type: str = "default",
                 **kwargs):
        """State class"""
        self.node = node
        self.type = type
        if kwargs:
            get_logger("fsm").warning(
                f"{self.type} state action unused kwargs: {kwargs}")

    def __repr__(self) -> str:
        return f"type: {self.type}"

    def stop(self):
        get_logger("fsm").info(
            f"Stopped {self.type} action")
        return True

    async def execute(self) -> bool:
        raise NotImplementedError(
            f"execute method not implemented for {self.type} state action")


class DurationAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "Duration",
                 duration: float = 60,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.duration = duration
        self.expiration_event = asyncio.Event()

    def __repr__(self) -> str:
        return f"type: {self.type}, duration: {self.duration}"

    def stop(self):
        get_logger("fsm").info(
            f"Stop {self.type} action")
        self.expiration_event.set()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("fsm").info(
            f"Executing {self.type} state action for {self.duration} seconds")
        await asyncio.wait_for(self.expiration_event.wait(), timeout=self.duration)
        self.expiration_event.clear()
        return True


class InitIMUAction(DurationAction):
    def __init__(self,
                 node: Node,
                 type: str = "InitIMU",
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)

    def __repr__(self) -> str:
        return f"type: {self.type}, duration: {self.duration}"

    async def execute(self) -> bool:
        get_logger("fsm").info(f"Executing {self.type} state action")
        return await super().execute()


class EnableStabilizationAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "EnableStabilization",
                 surge: bool = False,
                 sway: bool = False,
                 depth: bool = False,
                 roll: bool = False,
                 pitch: bool = False,
                 yaw: bool = False,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)

        self.surge = surge
        self.sway = sway
        self.depth = depth
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __repr__(self) -> str:
        return f"type: {self.type}, surge: {self.surge}, sway: {self.sway}, depth: {self.depth}, roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}"

    async def execute(self) -> bool:
        get_logger("fsm").info(f"Executing {self.type} state action")
        return True


class ThrusterIndicationAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "ThrusterIndication",
                 repeat: int = 1,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.repeat = repeat

        self.goal = TwistAction.Goal()
        self.goal.surge = 10.0
        self.goal.duration = 2.0

        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"{str(super())}, repeat: {self.repeat}"

    async def execute(self) -> bool:
        get_logger("fsm").info(f"Executing {self.type} state action")
        for i in range(self.repeat):
            get_logger("fsm").info(f"Thruster indication {i+1}/{self.repeat}")
            async for (_, result) in self.twist_action_client.send_goal_async(self.goal):
                if result:
                    break
        return True


class MoveAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "Move",
                 surge: float = 0.0,
                 sway: float = 0.0,
                 depth: float = 0.0,
                 roll: float = 0.0,
                 pitch: float = 0.0,
                 yaw: float = 0.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.goal = TwistAction.Goal()
        self.goal.surge = surge
        self.goal.sway = sway
        self.goal.depth = depth
        self.goal.roll = roll
        self.goal.pitch = pitch
        self.goal.yaw = yaw

        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, surge: {self.goal.surge}, sway: {self.goal.sway}, depth: {self.goal.depth}, roll: {self.goal.roll}, pitch: {self.goal.pitch}, yaw: {self.goal.yaw}"

    async def execute(self) -> bool:
        get_logger("fsm").info(f"Executing {self.type} state action")
        async for (_, result) in self.twist_action_client.send_goal_async(self.goal):
            if result:
                return result.done
        return False


def create_action(node: Node, action: dict) -> StateAction:
    if action['type'] == "Duration":
        return DurationAction(node=node, **action)
    elif action['type'] == "InitIMU":
        return InitIMUAction(node=node, **action)
    elif action['type'] == "EnableStabilization":
        return EnableStabilizationAction(node=node, **action)
    elif action['type'] == "ThrusterIndication":
        return ThrusterIndicationAction(node=node, **action)
    elif action['type'] == "Move":
        return MoveAction(node=node, **action)
    else:
        raise NotImplementedError(
            f"Action type {action['type']} not implemented")
    return None
