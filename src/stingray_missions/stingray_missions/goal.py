import math

from geometry_msgs.msg import PoseStamped
from rclpy.logging import get_logger
from rclpy.node import Node
from stingray_interfaces.srv import SetGoal


class GoalStore:
    def __init__(self, node: Node, service_name: str, is_busy):
        self.node = node
        self.is_busy = is_busy
        self.goal: PoseStamped = None
        self.received_at = None
        self.service = node.create_service(SetGoal, service_name, self._callback)

    def _callback(self, request: SetGoal.Request, response: SetGoal.Response):
        if self.is_busy():
            response.ok = False
            response.message = "mission is active, goal rejected"
            return response

        target = request.target
        if not target.header.frame_id:
            target.header.frame_id = "map"
        if target.header.frame_id != "map":
            response.ok = False
            response.message = f"frame {target.header.frame_id} is not map"
            return response

        p = target.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            response.ok = False
            response.message = "position is not finite"
            return response

        q = target.pose.orientation
        norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
        if norm == 0.0 or not math.isfinite(norm):
            q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
        else:
            q.x, q.y, q.z, q.w = q.x / norm, q.y / norm, q.z / norm, q.w / norm

        self.goal = target
        self.received_at = self.node.get_clock().now()
        get_logger("goal").info(f"Goal stored: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}")
        response.ok = True
        response.message = "goal accepted"
        return response

    def take(self, max_age: float):
        goal, received_at = self.goal, self.received_at
        self.goal = None
        self.received_at = None
        if goal is None:
            return None
        age = (self.node.get_clock().now() - received_at).nanoseconds / 1e9
        if age > max_age:
            get_logger("goal").error(f"Goal is {age:.1f} s old, limit {max_age:.1f} s")
            return None
        return goal
