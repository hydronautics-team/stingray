from rclpy.logging import get_logger
import time
import asyncio
from rclpy.node import Node
from rclpy.publisher import Publisher
from stingray_utils.acyncio import AsyncActionClient
from stingray_interfaces.action import TwistAction, TwistAction_GetResult_Response
from stingray_interfaces.action import BboxCenteringTwistAction, BboxCenteringTwistAction_GetResult_Response
from stingray_interfaces.action import BboxSearchTwistAction, BboxSearchTwistAction_GetResult_Response
from stingray_interfaces.action import DeviceAction, DeviceAction_GetResult_Response
from stingray_interfaces.msg import EnableObjectDetection
from stingray_core_interfaces.srv import SetStabilization
from std_srvs.srv import Trigger


class StateAction():
    def __init__(self,
                 node: Node,
                 type: str = "default",
                 **kwargs):
        """State class"""
        self.node = node
        self.type = type
        self.stopped = False
        self.executed = False

        if kwargs:
            get_logger("action").warning(
                f"{self.type} state action unused kwargs: {kwargs}")

    def __repr__(self) -> str:
        return f"type: {self.type}"

    def stop(self):
        get_logger("action").info(
            f"Stopped {self.type} action")
        self.stopped = True
        return True

    async def execute(self) -> bool:
        raise NotImplementedError(
            f"execute method not implemented for {self.type} state action")


class DurationStateAction(StateAction):
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
        get_logger("action").info(
            f"Enter stop {self.type}")
        self.expiration_event.set()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(
            f"Executing {self.type} state action for {self.duration} seconds")
        try:
            await asyncio.wait_for(self.expiration_event.wait(), timeout=self.duration)
            self.expiration_event.clear()
        except asyncio.TimeoutError:
            pass
        self.executed = True
        get_logger("action").info(
            f"Finished executing {self.type}")
        return True


class ResetIMUStateAction(DurationStateAction):
    def __init__(self,
                 node: Node,
                 type: str = "ResetIMU",
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)

        self.reset_imu_client = self.node.create_client(
            Trigger, self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value)

        while not self.reset_imu_client.wait_for_service(timeout_sec=1.0):
            get_logger('action').info(
                f"{self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value} not available, waiting again...")

    def __repr__(self) -> str:
        return f"type: {self.type}, duration: {self.duration}"

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        try:
            self.future: Trigger.Response = await asyncio.wait_for(self.reset_imu_client.call_async(Trigger.Request()), timeout=1.0)
            if not self.future.success:
                get_logger('action').error(
                    f"Error while waiting for {self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value}: {self.future.message}")
                return False
        except asyncio.TimeoutError:
            get_logger('action').error(
                f"Wait for {self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value} timed out")
            return False
        return await super().execute()


class EnableStabilizationStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "EnableStabilization",
                 depth: bool = False,
                 roll: bool = False,
                 pitch: bool = False,
                 yaw: bool = False,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)

        self.srv_request = SetStabilization.Request()
        self.srv_request.depth_stabilization = depth
        self.srv_request.roll_stabilization = roll
        self.srv_request.pitch_stabilization = pitch
        self.srv_request.yaw_stabilization = yaw

        self.set_stabilization_client = self.node.create_client(
            SetStabilization, self.node.get_parameter('set_stabilization_srv').get_parameter_value().string_value)

        while not self.set_stabilization_client.wait_for_service(timeout_sec=1.0):
            get_logger('action').info(
                f'set_stabilization_srv not available, waiting again...')

    def __repr__(self) -> str:
        return f"type: {self.type}, depth_stabilization: {self.srv_request.depth_stabilization}, roll_stabilization: {self.srv_request.roll_stabilization}, pitch_stabilization: {self.srv_request.pitch_stabilization}, yaw_stabilization: {self.srv_request.yaw_stabilization}"

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        try:
            self.future: SetStabilization.Response = await asyncio.wait_for(self.set_stabilization_client.call_async(self.srv_request), timeout=1.0)
            if not self.future.success:
                get_logger('action').error(
                    f"Error while waiting for {self.node.get_parameter('set_stabilization_srv').get_parameter_value().string_value}: {self.future.message}")
                return False
        except asyncio.TimeoutError:
            get_logger('action').error(
                f"Wait for {self.node.get_parameter('set_stabilization_srv').get_parameter_value().string_value} timed out")
            return False
        self.executed = True
        return True


class EnableObjectDetectionStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "EnableObjectDetection",
                 camera_topic: str = "",
                 enable: bool = False,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)

        self.msg = EnableObjectDetection()
        self.msg.camera_topic = camera_topic
        self.msg.enable = enable

        self._enable_object_detection_pub: Publisher = self.node.create_publisher(
            EnableObjectDetection,
            self.node.get_parameter(
                'enable_object_detection_topic').get_parameter_value().string_value,
            10)

    def __repr__(self) -> str:
        return f"type: {self.type}, camera_topic: {self.msg.camera_topic}, enable: {self.msg.enable}"

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        for _ in range(5):
            self._enable_object_detection_pub.publish(self.msg)
        self.executed = True
        return True


class ThrusterIndicationStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "ThrusterIndication",
                 repeat: int = 1,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.repeat = repeat

        self.goal = TwistAction.Goal()
        self.goal.surge = 50.0
        self.goal.duration = 0.3

        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, repeat: {self.repeat}"

    def stop(self):
        self.twist_action_client.cancel()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        for i in range(self.repeat):
            if self.stopped:
                return False
            get_logger("action").info(
                f"Thruster indication {i+1}/{self.repeat}")
            result: TwistAction_GetResult_Response = await self.twist_action_client.send_goal_async(self.goal)
            await asyncio.sleep(self.goal.duration)
        self.executed = True
        return result.result.success


class TwistStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "Move",
                 surge: float = 0.0,
                 sway: float = 0.0,
                 depth: float = 0.0,
                 roll: float = 0.0,
                 pitch: float = 0.0,
                 yaw: float = 0.0,
                 duration: float = 0.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.goal = TwistAction.Goal()
        self.goal.surge = float(surge)
        self.goal.sway = float(sway)
        self.goal.depth = float(depth)
        self.goal.roll = float(roll)
        self.goal.pitch = float(pitch)
        self.goal.yaw = float(yaw)
        self.goal.duration = float(duration)

        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, surge: {self.goal.surge}, sway: {self.goal.sway}, depth: {self.goal.depth}, roll: {self.goal.roll}, pitch: {self.goal.pitch}, yaw: {self.goal.yaw}, duration: {self.goal.duration}"

    def stop(self):
        self.twist_action_client.cancel()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        result: TwistAction_GetResult_Response = await self.twist_action_client.send_goal_async(self.goal)
        self.executed = True
        return result.result.success


class BboxCenteringTwistStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "BboxCenteringTwist",
                 bbox_name: str = "",
                 bbox_topic: str = "",
                 distance_threshold: float = 0.0,
                 lost_threshold: int = 0,
                 avoid_bbox_name_array: list[str] = [],
                 avoid_distance_threshold: float = 0.0,
                 avoid_horizontal_threshold: float = 0.0,
                 surge: float = 0.0,
                 sway: float = 0.0,
                 depth: float = 0.0,
                 roll: float = 0.0,
                 pitch: float = 0.0,
                 duration: float = 0.0,
                 centering_rate: float = 0.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.goal = BboxCenteringTwistAction.Goal()
        self.goal.bbox_name = bbox_name
        self.goal.bbox_topic = bbox_topic
        self.goal.distance_threshold = float(distance_threshold)
        self.goal.lost_threshold = int(lost_threshold)
        get_logger("action").info(f"avoid_bbox_name_array {avoid_bbox_name_array}")
        self.goal.avoid_bbox_name_array = avoid_bbox_name_array
        self.goal.avoid_distance_threshold = float(avoid_distance_threshold)
        self.goal.avoid_horizontal_threshold = float(
            avoid_horizontal_threshold)
        self.goal.surge = float(surge)
        self.goal.sway = float(sway)
        self.goal.depth = float(depth)
        self.goal.roll = float(roll)
        self.goal.pitch = float(pitch)
        self.goal.duration = float(duration)
        self.goal.centering_rate = float(centering_rate)

        self.bbox_centering_twist_action_client = AsyncActionClient(
            self.node, BboxCenteringTwistAction, self.node.get_parameter('bbox_centering_twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, bbox_name: {self.goal.bbox_name}"

    def stop(self):
        self.bbox_centering_twist_action_client.cancel()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        result: BboxCenteringTwistAction_GetResult_Response = await self.bbox_centering_twist_action_client.send_goal_async(self.goal)
        self.executed = True
        return result.result.success


class BboxSearchTwistStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "BboxSearchTwist",
                 bbox_name: str = "",
                 bbox_topic: str = "",
                 first_clockwise: bool = True,
                 found_threshold: int = 0,
                 max_yaw: float = 0.0,
                 yaw_step: float = 0.0,
                 depth: float = 0.0,
                 roll: float = 0.0,
                 pitch: float = 0.0,
                 duration: float = 0.0,
                 search_rate: float = 0.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.goal = BboxSearchTwistAction.Goal()
        self.goal.bbox_name = bbox_name
        self.goal.bbox_topic = bbox_topic
        self.goal.first_clockwise = first_clockwise
        self.goal.found_threshold = int(found_threshold)
        self.goal.max_yaw = float(max_yaw)
        self.goal.yaw_step = float(yaw_step)
        self.goal.depth = float(depth)
        self.goal.roll = float(roll)
        self.goal.pitch = float(pitch)
        self.goal.duration = float(duration)
        self.goal.search_rate = float(search_rate)

        self.bbox_search_twist_action_client = AsyncActionClient(
            self.node, BboxSearchTwistAction, self.node.get_parameter('bbox_search_twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, bbox_name: {self.goal.bbox_name}"

    def stop(self):
        self.bbox_search_twist_action_client.cancel()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        result: BboxSearchTwistAction_GetResult_Response = await self.bbox_search_twist_action_client.send_goal_async(self.goal)
        self.executed = True
        return result.result.success


class SetDeviceValueStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "SetDeviceValue",
                 device: int = 0.0,
                 value: int = 0.0,
                 timeout: float = 10.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.goal = DeviceAction.Goal()
        self.goal.device = int(device)
        self.goal.value = int(value)
        self.goal.timeout = float(timeout)

        self.device_action_client = AsyncActionClient(
            self.node, DeviceAction, self.node.get_parameter('device_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, device: {self.goal.device}, value: {self.goal.value}"

    def stop(self):
        self.device_action_client.cancel()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        result: DeviceAction_GetResult_Response = await self.device_action_client.send_goal_async(self.goal)
        self.executed = True
        return result.result.success


def create_action(node: Node, action: dict) -> StateAction:
    if action['type'] == "Duration":
        return DurationStateAction(node=node, **action)
    elif action['type'] == "ResetIMU":
        return ResetIMUStateAction(node=node, **action)
    elif action['type'] == "EnableStabilization":
        return EnableStabilizationStateAction(node=node, **action)
    elif action['type'] == "EnableObjectDetection":
        return EnableObjectDetectionStateAction(node=node, **action)
    elif action['type'] == "ThrusterIndication":
        return ThrusterIndicationStateAction(node=node, **action)
    elif action['type'] == "Twist":
        return TwistStateAction(node=node, **action)
    elif action['type'] == "BboxCenteringTwist":
        return BboxCenteringTwistStateAction(node=node, **action)
    elif action['type'] == "BboxSearchTwist":
        return BboxSearchTwistStateAction(node=node, **action)
    elif action['type'] == "SetDeviceValue":
        return SetDeviceValueStateAction(node=node, **action)
    else:
        raise NotImplementedError(
            f"Action type {action['type']} not implemented")
    return None
