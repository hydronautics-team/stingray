from typing import List

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class AUVOdometryTf2Broadcaster(Node):
    def __init__(self):
        super().__init__("auv_odometry_tf2_broadcaster")

        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )

        # Публикатор маркера траектории
        self.trajectory_pub = self.create_publisher(
            MarkerArray, "/trajectory_marker", 10
        )

        # Публикатор маркера текущей позиции
        self.pose_marker_pub = self.create_publisher(Marker, "/current_pose_marker", 10)

        # Хранение траектории
        self.trajectory_points: List[Point] = []
        self.marker_id = 0

        # self.sub_imu_linear_accel = self.create_subscription(
        #    Imu, "/vectornav/imu", self.imu_callback, 10
        # )

        self.pub_pose = self.create_publisher(Point, "/test/pose", 10)
        # self.pub_timer = self.create_timer(0.01, self.publish_transform)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_timer = self.create_timer(1.0, self.publish_static_tf)

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0.1:  # TODO: fix it
            dt = 0.01

        self.q_x = msg.orientation.x
        self.q_y = msg.orientation.y
        self.q_z = msg.orientation.z
        self.q_w = msg.orientation.w

        imu_linear_accel_x = msg.linear_acceleration.x
        imu_linear_accel_y = msg.linear_acceleration.y
        imu_linear_accel_z = msg.linear_acceleration.z

        self.get_logger().info(f"imu_linear_accel_x = {imu_linear_accel_x} ")

        self.vx += imu_linear_accel_x * dt
        self.vy += imu_linear_accel_y * dt
        self.vz += imu_linear_accel_z * dt

        self.x += self.vx * dt + 0.5 * imu_linear_accel_x * dt * dt
        self.y += self.vy * dt + 0.5 * imu_linear_accel_y * dt * dt
        self.z += self.vz * dt + 0.5 * imu_linear_accel_z * dt * dt

        point_msg = Point()
        point_msg.x = self.x
        point_msg.y = self.y
        point_msg.z = self.z
        self.pub_pose.publish(point_msg)

        self.last_time = current_time

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "auv"  # базовая система робота
        t.child_frame_id = "imu_link"  # система IMU
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "auv"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = self.q_x
        t.transform.rotation.y = self.q_y
        t.transform.rotation.z = self.q_z
        t.transform.rotation.w = self.q_w

        self.tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg: Odometry):
        # Получаем позицию и ориентацию из EKF
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Обновляем состояние
        self.x = x
        self.y = y
        self.z = z
        self.q_x = msg.pose.pose.orientation.x
        self.q_y = msg.pose.pose.orientation.y
        self.q_z = msg.pose.pose.orientation.z
        self.q_w = msg.pose.pose.orientation.w

        # НЕМЕДЛЕННО ПУБЛИКУЕМ ТРАНСФОРМАЦИЮ
        self.publish_transform()

        # Добавляем точку в траекторию
        self.trajectory_points.append(Point(x=x, y=y, z=z))

        # Ограничиваем длину траектории
        if len(self.trajectory_points) > 1000:
            self.trajectory_points.pop(0)

        # Публикуем маркер траектории
        self.publish_trajectory_marker()

        # Публикуем маркер текущей позиции
        self.publish_current_pose_marker(x, y, z)

    def publish_trajectory_marker(self):
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = self.trajectory_points

        marker.scale.x = 0.05  # Толщина линии

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_array.markers.append(marker)
        self.trajectory_pub.publish(marker_array)

    def publish_current_pose_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_pose"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.pose_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = AUVOdometryTf2Broadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
