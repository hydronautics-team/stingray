from rclpy.logging import get_logger
import time
import asyncio
from rclpy.node import Node
from rclpy.publisher import Publisher
from stingray_utils.acyncio import AsyncActionClient
from stingray_interfaces.action import TwistAction
from stingray_interfaces.action import BboxCenteringTwistAction
from stingray_interfaces.action import BboxSearchTwistAction
from stingray_interfaces.action import HydroacousticCenteringTwistAction
from stingray_interfaces.action import DeviceAction
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

        while not self.reset_imu_client.wait_for_service(timeout_sec=10.0):
            get_logger('action').info(
                f"{self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value} not available, waiting again...")

    def __repr__(self) -> str:
        return f"type: {self.type}, duration: {self.duration}"

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        try:
            self.future: Trigger.Response = await asyncio.wait_for(self.reset_imu_client.call_async(Trigger.Request()), timeout=10.0)
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

        while not self.set_stabilization_client.wait_for_service(timeout_sec=10.0):
            get_logger('action').info(
                f'set_stabilization_srv not available, waiting again...')

    def __repr__(self) -> str:
        return f"type: {self.type}, depth_stabilization: {self.srv_request.depth_stabilization}, roll_stabilization: {self.srv_request.roll_stabilization}, pitch_stabilization: {self.srv_request.pitch_stabilization}, yaw_stabilization: {self.srv_request.yaw_stabilization}"

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        try:
            self.future: SetStabilization.Response = await asyncio.wait_for(self.set_stabilization_client.call_async(self.srv_request), timeout=10.0)
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
            result = await self.twist_action_client.send_goal_async(self.goal)
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
        result = await self.twist_action_client.send_goal_async(self.goal)
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
        result = await self.bbox_centering_twist_action_client.send_goal_async(self.goal)
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
        result = await self.bbox_search_twist_action_client.send_goal_async(self.goal)
        self.executed = True
        return result.result.success


class SequencePunchBboxTwistStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "SequencePunchBboxTwist",
                 sequence: list[str] = [],
                 bbox_topic: str = "",
                 distance_threshold: float = 0.0,
                 avoid_distance_threshold: float = 0.0,
                 avoid_horizontal_threshold: float = 0.0,
                 lost_threshold: int = 0,
                 first_clockwise: bool = True,
                 found_threshold: int = 0,
                 max_yaw: float = 0.0,
                 yaw_step: float = 0.0,
                 surge: float = 0.0,
                 avoid_sway: float = 0.0,
                 depth: float = 0.0,
                 roll: float = 0.0,
                 pitch: float = 0.0,
                 search_rate: float = 0.0,
                 centering_duration: float = 0.0,
                 punch_duration: float = 0.0,
                 centering_rate: float = 0.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.bbox_topic = bbox_topic
        self.sequence = sequence
        self.distance_threshold = float(distance_threshold)
        self.avoid_distance_threshold = float(avoid_distance_threshold)
        self.avoid_horizontal_threshold = float(avoid_horizontal_threshold)
        self.lost_threshold = int(lost_threshold)
        self.first_clockwise = first_clockwise
        self.found_threshold = int(found_threshold)
        self.max_yaw = float(max_yaw)
        self.yaw_step = float(yaw_step)
        self.surge = float(surge)
        self.avoid_sway = float(avoid_sway)
        self.depth = float(depth)
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.search_rate = float(search_rate)
        self.centering_duration = float(centering_duration)
        self.punch_duration = float(punch_duration)
        self.centering_rate = float(centering_rate)

        self.bbox_centering_twist_action_client = AsyncActionClient(
            self.node, BboxCenteringTwistAction, self.node.get_parameter('bbox_centering_twist_action').get_parameter_value().string_value)
        self.bbox_search_twist_action_client = AsyncActionClient(
            self.node, BboxSearchTwistAction, self.node.get_parameter('bbox_search_twist_action').get_parameter_value().string_value)
        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}"

    def stop(self):
        self.bbox_centering_twist_action_client.cancel()
        self.bbox_search_twist_action_client.cancel()
        return super().stop()

    def get_bbox_name(self, flare_id: str):
        if flare_id == "R":
            return "small_red_flare"
        elif flare_id == "B":
            return "blue_flare"
        elif flare_id == "Y":
            return "yellow_flare"
        else:
            return "yellow_flare"

    def get_avoid_bbox_array(self, flare_id: str):
        if flare_id == "R":
            return ["blue_flare", "yellow_flare"]
        elif flare_id == "B":
            return ["small_red_flare", "yellow_flare"]
        elif flare_id == "Y":
            return ["small_red_flare", "blue_flare"]
        else:
            return []

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        for flare in self.sequence:
            search_goal = BboxSearchTwistAction.Goal()
            search_goal.bbox_name = self.get_bbox_name(flare_id=flare)
            search_goal.bbox_topic = self.bbox_topic
            search_goal.first_clockwise = self.first_clockwise
            search_goal.found_threshold = int(self.found_threshold)
            search_goal.max_yaw = float(self.max_yaw)
            search_goal.yaw_step = float(self.yaw_step)
            search_goal.depth = float(self.depth)
            search_goal.roll = float(self.roll)
            search_goal.pitch = float(self.pitch)
            search_goal.search_rate = float(self.search_rate)
            result = await self.bbox_search_twist_action_client.send_goal_async(search_goal)

            centering_goal = BboxCenteringTwistAction.Goal()
            centering_goal.bbox_name = self.get_bbox_name(flare_id=flare)
            centering_goal.bbox_topic = self.bbox_topic
            centering_goal.distance_threshold = float(self.distance_threshold)
            centering_goal.lost_threshold = int(self.lost_threshold)
            centering_goal.avoid_bbox_name_array = self.get_avoid_bbox_array(
                flare_id=flare)
            centering_goal.avoid_distance_threshold = float(
                self.avoid_distance_threshold)
            centering_goal.avoid_horizontal_threshold = float(
                self.avoid_horizontal_threshold)
            centering_goal.surge = float(self.surge)
            centering_goal.sway = float(self.avoid_sway)
            centering_goal.depth = float(self.depth)
            centering_goal.roll = float(self.roll)
            centering_goal.pitch = float(self.pitch)
            centering_goal.duration = float(self.centering_duration)
            centering_goal.centering_rate = float(self.centering_rate)
            result = await self.bbox_centering_twist_action_client.send_goal_async(centering_goal)

            punch_goal = TwistAction.Goal()
            punch_goal.surge = float(70.0)
            punch_goal.sway = float(0.0)
            punch_goal.depth = float(self.depth)
            punch_goal.roll = float(self.roll)
            punch_goal.pitch = float(self.pitch)
            punch_goal.yaw = float(0.0)
            punch_goal.duration = float(self.punch_duration)
            # result = await self.twist_action_client.send_goal_async(punch_goal)
        self.executed = True
        return result.result.success


class HydroacousticCenteringTwistStateAction(StateAction):
    def __init__(self,
                 node: Node,
                 type: str = "HydroacousticCenteringTwist",
                 bbox_name: str = "",
                 bbox_topic: str = "",
                 hydroacoustic_topic: str = "",
                 angle_threshold: float = 0.0,
                 distance_threshold: float = 0.0,
                 lost_threshold: int = 0,
                 surge: float = 0.0,
                 sway: float = 0.0,
                 depth: float = 0.0,
                 roll: float = 0.0,
                 pitch: float = 0.0,
                 duration: float = 0.0,
                 centering_rate: float = 0.0,
                 **kwargs):
        super().__init__(node=node, type=type, **kwargs)
        self.goal = HydroacousticCenteringTwistAction.Goal()
        self.goal.bbox_name = bbox_name
        self.goal.bbox_topic = bbox_topic
        self.goal.hydroacoustic_topic = hydroacoustic_topic
        self.goal.angle_threshold = float(angle_threshold)
        self.goal.distance_threshold = float(distance_threshold)
        self.goal.lost_threshold = int(lost_threshold)
        self.goal.surge = float(surge)
        self.goal.sway = float(sway)
        self.goal.depth = float(depth)
        self.goal.roll = float(roll)
        self.goal.pitch = float(pitch)
        self.goal.duration = float(duration)
        self.goal.centering_rate = float(centering_rate)

        self.hydroacoustic_centering_twist_action_client = AsyncActionClient(
            self.node, HydroacousticCenteringTwistAction, self.node.get_parameter('hydroacoustic_centering_twist_action').get_parameter_value().string_value)

    def __repr__(self) -> str:
        return f"type: {self.type}, bbox_name: {self.goal.bbox_name}"

    def stop(self):
        self.hydroacoustic_centering_twist_action_client.cancel()
        return super().stop()

    async def execute(self) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        result = await self.hydroacoustic_centering_twist_action_client.send_goal_async(self.goal)
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
        result = await self.device_action_client.send_goal_async(self.goal)
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
    elif action['type'] == "SequencePunchBboxTwist":
        return SequencePunchBboxTwistStateAction(node=node, **action)
    elif action['type'] == "HydroacousticCenteringTwist":
        return HydroacousticCenteringTwistStateAction(node=node, **action)
    elif action['type'] == "SetDeviceValue":
        return SetDeviceValueStateAction(node=node, **action)
    else:
        raise NotImplementedError(
            f"Action type {action['type']} not implemented")
    return None
