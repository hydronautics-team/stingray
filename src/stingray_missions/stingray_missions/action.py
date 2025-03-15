from rclpy.logging import get_logger
import time
import asyncio
from rclpy.node import Node
from rclpy.publisher import Publisher
from stingray_utils.acyncio import AsyncActionClient
from stingray_interfaces.action import TwistAction
from stingray_interfaces.action import BboxCenteringTwistAction
from stingray_interfaces.action import BboxBottomCenteringTwistAction
from stingray_interfaces.action import BboxSearchTwistAction
from stingray_interfaces.action import HydroacousticCenteringTwistAction
from stingray_interfaces.action import DeviceAction
from stingray_interfaces.msg import EnableTopic
from stingray_core_interfaces.srv import SetStabilization
from std_srvs.srv import Trigger, SetBool


class StateActionBase():
    node: Node = None
    type: str = "Default"
    stopped = False
    executed = False

    def __init__(self, node: Node):
        """State class"""
        self.node = node

    def __repr__(self) -> str:
        return f"type: {self.type}"

    def stop(self):
        self.stopped = True
        get_logger("action").info(
            f"Stopped {self.type} action")
        return True

    async def execute(self, **kwargs) -> bool:
        if kwargs:
            get_logger("action").warning(
                f"{self.type} state action unused kwargs: {kwargs}")
        self.executed = True
        get_logger("action").info(
            f"Executed {self.type}")
        return True


class DurationStateAction(StateActionBase):
    type = "Duration"

    def __init__(self,
                 node: Node,
                 ):
        super().__init__(node=node)
        self.expiration_event = asyncio.Event()

    def stop(self):
        self.expiration_event.set()
        return super().stop()

    async def execute(self,
                      duration: float = 60,
                      **kwargs) -> bool:
        get_logger("action").info(
            f"Executing {self.type} state action for {duration} seconds")
        try:
            await asyncio.wait_for(self.expiration_event.wait(), timeout=duration)
            self.expiration_event.clear()
        except asyncio.TimeoutError:
            pass

        get_logger("action").info(
            f"Executed {self.type}")
        return await super().execute(**kwargs)


class ResetIMUStateAction(DurationStateAction):
    type = "ResetIMU"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.reset_imu_client = self.node.create_client(
            Trigger, self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value)

    async def execute(self,
                      duration: float = 60,
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")
        if not self.reset_imu_client.wait_for_service(timeout_sec=1.0):
            get_logger('action').info(
                f"{self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value} not available...")
            return False
        try:
            self.future: Trigger.Response = await asyncio.wait_for(self.reset_imu_client.call_async(Trigger.Request()), timeout=2.0)
            if not self.future.success:
                get_logger('action').error(
                    f"Error while waiting for {self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value}: {self.future.message}")
                return False
        except asyncio.TimeoutError:
            get_logger('action').error(
                f"Wait for {self.node.get_parameter('reset_imu_srv').get_parameter_value().string_value} timed out")
            return False
        return await super().execute(duration, **kwargs)


class EnableStabilizationStateAction(StateActionBase):
    type = "EnableStabilization"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.set_stabilization_client = self.node.create_client(
            SetStabilization, self.node.get_parameter('set_stabilization_srv').get_parameter_value().string_value)

    async def execute(self,
                      depth: bool = False,
                      roll: bool = False,
                      pitch: bool = False,
                      yaw: bool = False,
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")

        self.srv_request = SetStabilization.Request()
        self.srv_request.depth_stabilization = depth
        self.srv_request.roll_stabilization = roll
        self.srv_request.pitch_stabilization = pitch
        self.srv_request.yaw_stabilization = yaw

        if not self.set_stabilization_client.wait_for_service(timeout_sec=1.0):
            get_logger('action').info(
                f"{self.set_stabilization_client.srv_name} not available...")
            return False

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
        return await super().execute(**kwargs)


class EnableObjectDetectionStateAction(StateActionBase):
    type = "EnableObjectDetection"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self._enable_object_detection_pub: Publisher = self.node.create_publisher(
            EnableTopic,
            self.node.get_parameter(
                'enable_object_detection_topic').get_parameter_value().string_value,
            10)

    async def execute(self,
                      camera_topic: str = "",
                      enable: bool = False,
                      **kwargs) -> bool:
        get_logger("action").info(
            f"Executing {self.type} state action. Enable object detection: {enable}")

        self.msg = EnableTopic()
        self.msg.topic_name = camera_topic
        self.msg.enable = enable

        self._enable_object_detection_pub.publish(self.msg)
        return await super().execute(**kwargs)


class EnableVideoRecordingStateAction(StateActionBase):
    type = "EnableVideoRecording"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self._enable_recording_pub: Publisher = self.node.create_publisher(
            EnableTopic,
            self.node.get_parameter(
                'enable_recording_topic').get_parameter_value().string_value,
            10)

    async def execute(self,
                      camera_topic: str = "",
                      enable: bool = False,
                      **kwargs) -> bool:
        get_logger("action").info(
            f"Executing {self.type} state action. Enable recording: {enable}")

        self.msg = EnableTopic()
        self.msg.topic_name = camera_topic
        self.msg.enable = enable

        self._enable_recording_pub.publish(self.msg)
        return await super().execute(**kwargs)


class ThrusterIndicationStateAction(StateActionBase):
    type = "ThrusterIndication"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def stop(self):
        get_logger("action").info(
            f"Stopping {self.type} action")
        self.twist_action_client.cancel()
        return super().stop()

    async def execute(self,
                      repeat: int = 1,
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")

        if not self.twist_action_client.wait_for_server(timeout_sec=1.0):
            get_logger("action").error(
                f"Timeout while waiting for {self.twist_action_client._action_name} action server")
            return False

        self.goal = TwistAction.Goal()
        self.goal.surge = 50.0
        self.goal.duration = 0.3

        for i in range(repeat):
            if self.stopped:
                return False
            get_logger("action").info(
                f"Thruster indication {i+1}/{repeat}")
            result = await self.twist_action_client.send_goal_async(self.goal)
            if not result.result.success:
                get_logger("action").error(
                    f"Error while waiting for {self.node.get_parameter('twist_action').get_parameter_value().string_value}")
                return False
            await asyncio.sleep(self.goal.duration)
        return await super().execute(**kwargs)


class TwistStateAction(StateActionBase):
    type = "Twist"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.twist_action_client = AsyncActionClient(
            self.node, TwistAction, self.node.get_parameter('twist_action').get_parameter_value().string_value)

    def stop(self):
        get_logger("action").info(
            f"Stopping {self.type} action")
        self.twist_action_client.cancel()
        return super().stop()

    async def execute(self,
                      surge: float = 0.0,
                      sway: float = 0.0,
                      depth: float = 0.0,
                      roll: float = 0.0,
                      pitch: float = 0.0,
                      yaw: float = 0.0,
                      duration: float = 0.0,
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")

        if not self.twist_action_client.wait_for_server(timeout_sec=1.0):
            get_logger("action").error(
                f"Timeout while waiting for {self.twist_action_client._action_name} action server")
            return False

        self.goal = TwistAction.Goal()
        self.goal.surge = float(surge)
        self.goal.sway = float(sway)
        self.goal.depth = float(depth)
        self.goal.roll = float(roll)
        self.goal.pitch = float(pitch)
        self.goal.yaw = float(yaw)
        self.goal.duration = float(duration)

        result = await self.twist_action_client.send_goal_async(self.goal)
        if not result.result.success:
            get_logger("action").error(
                f"Error while waiting for {self.node.get_parameter('twist_action').get_parameter_value().string_value}")
            return False
        return await super().execute(**kwargs)


class BboxCenteringTwistStateAction(StateActionBase):
    type = "BboxCenteringTwist"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.bbox_centering_twist_action_client = AsyncActionClient(
            self.node, BboxCenteringTwistAction, self.node.get_parameter('bbox_centering_twist_action').get_parameter_value().string_value)

    def stop(self):
        get_logger("action").info(
            f"Stopping {self.type} action")
        self.bbox_centering_twist_action_client.cancel()
        return super().stop()

    async def execute(self,
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
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")

        if not self.bbox_centering_twist_action_client.wait_for_server(timeout_sec=1.0):
            get_logger("action").error(
                f"Timeout while waiting for {self.bbox_centering_twist_action_client._action_name} action server")
            return False

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

        result = await self.bbox_centering_twist_action_client.send_goal_async(self.goal)
        if not result.result.success:
            get_logger('action').error(
                f"Error while executing {self.node.get_parameter('bbox_centering_twist_action').get_parameter_value().string_value}")
            return False
        return await super().execute(**kwargs)
    
class BboxBottomCenteringTwistStateAction(StateActionBase):
    type = "BboxBottomCenteringTwist"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.bbox_bottom_centering_twist_action_client = AsyncActionClient(
            self.node, BboxBottomCenteringTwistAction, self.node.get_parameter('bbox_bottom_centering_twist_action').get_parameter_value().string_value)

    def stop(self):
        get_logger("action").info(
            f"Stopping {self.type} action")
        self.bbox_bottom_centering_twist_action_client.cancel()
        return super().stop()

    async def execute(self,
                      bbox_name: str = "",
                      bbox_topic: str = "",
                      threshold_x: float = 0.2,
                      threshold_y: float = 0.2,
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
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")

        if not self.bbox_bottom_centering_twist_action_client.wait_for_server(timeout_sec=1.0):
            get_logger("action").error(
                f"Timeout while waiting for {self.bbox_bottom_centering_twist_action_client._action_name} action server")
            return False

        self.goal = BboxBottomCenteringTwistAction.Goal()
        self.goal.bbox_name = bbox_name
        self.goal.bbox_topic = bbox_topic
        self.goal.threshold_x = float(threshold_x)
        self.goal.threshold_y = float(threshold_y)
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

        result = await self.bbox_bottom_centering_twist_action_client.send_goal_async(self.goal)
        if not result.result.success:
            get_logger('action').error(
                f"Error while executing {self.node.get_parameter('bbox_bottom_centering_twist_action').get_parameter_value().string_value}")
            return False
        return await super().execute(**kwargs)


class BboxSearchTwistStateAction(StateActionBase):
    type = "BboxSearchTwist"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.bbox_search_twist_action_client = AsyncActionClient(
            self.node, BboxSearchTwistAction, self.node.get_parameter('bbox_search_twist_action').get_parameter_value().string_value)

    def stop(self):
        get_logger("action").info(
            f"Stopping {self.type} action")
        self.bbox_search_twist_action_client.cancel()
        return super().stop()

    async def execute(self,
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
                      **kwargs) -> bool:
        get_logger("action").info(f"Executing {self.type} state action")

        if not self.bbox_search_twist_action_client.wait_for_server(timeout_sec=1.0):
            get_logger("action").error(
                f"Timeout while waiting for {self.bbox_search_twist_action_client._action_name} action server")
            return False

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

        result = await self.bbox_search_twist_action_client.send_goal_async(self.goal)
        if not result.result.success:
            get_logger('action').error(
                f"Error while executing {self.node.get_parameter('bbox_search_twist_action').get_parameter_value().string_value}")
            return False
        return await super().execute(**kwargs)


class SetDeviceValueStateAction(StateActionBase):
    type = "SetDeviceValue"

    def __init__(self, node: Node):
        super().__init__(node=node)

        self.device_action_client = AsyncActionClient(
            self.node, DeviceAction, self.node.get_parameter('device_action').get_parameter_value().string_value)

    def stop(self):
        get_logger("action").info(
            f"Stopping {self.type} action")
        self.device_action_client.cancel()
        return super().stop()

    async def execute(self,
                      device: int = 0.0,
                      value: int = 0.0,
                      timeout: float = 10.0,
                      **kwargs) -> bool:
        get_logger("action").info(
            f"Executing {self.type}. Device: {device}, Value: {value}")

        if not self.device_action_client.wait_for_server(timeout_sec=1.0):
            get_logger("action").error(
                f"Timeout while waiting for {self.device_action_client._action_name} action server")
            return False

        self.goal = DeviceAction.Goal()
        self.goal.device = int(device)
        self.goal.value = int(value)
        self.goal.timeout = float(timeout)

        result = await self.device_action_client.send_goal_async(self.goal)
        if not result.result.success:
            get_logger('action').error(
                f"Error while executing {self.node.get_parameter('device_action').get_parameter_value().string_value}")
            return False
        return await super().execute(**kwargs)


def load_stingray_actions(node: Node) -> dict[str, StateActionBase]:
    """Load all actions"""
    return {
        DurationStateAction.type: DurationStateAction(node),
        ResetIMUStateAction.type: ResetIMUStateAction(node),
        EnableStabilizationStateAction.type: EnableStabilizationStateAction(node),
        EnableObjectDetectionStateAction.type: EnableObjectDetectionStateAction(node),
        EnableVideoRecordingStateAction.type: EnableVideoRecordingStateAction(node),
        ThrusterIndicationStateAction.type: ThrusterIndicationStateAction(node),
        TwistStateAction.type: TwistStateAction(node),
        BboxCenteringTwistStateAction.type: BboxCenteringTwistStateAction(node),
        BboxBottomCenteringTwistStateAction.type: BboxBottomCenteringTwistStateAction(node),
        BboxSearchTwistStateAction.type: BboxSearchTwistStateAction(node),
        SetDeviceValueStateAction.type: SetDeviceValueStateAction(node),
    }
