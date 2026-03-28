import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


class Plotter(Node):
    def __init__(self):
        super().__init__("plotter")
        self.sub = self.create_subscription(Point, "/test/pose", self.callback, 10)
        self.times = []
        self.x_vals = []
        self.y_vals = []
        self.start_time = self.get_clock().now()

        # Настройка графика
        plt.ion()
        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 8))

    def callback(self, msg):
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.times.append(current_time)
        self.x_vals.append(msg.x)
        self.y_vals.append(msg.y)

        # Очищаем и перерисовываем
        self.ax[0].clear()
        self.ax[0].plot(self.times, self.x_vals, "r-", label="x")
        self.ax[0].plot(self.times, self.y_vals, "g-", label="y")
        self.ax[0].set_xlabel("Time (s)")
        self.ax[0].set_ylabel("Position (m)")
        self.ax[0].legend()
        self.ax[0].grid(True)

        self.ax[1].clear()
        self.ax[1].plot(self.x_vals, self.y_vals, "b-")
        self.ax[1].set_xlabel("X (m)")
        self.ax[1].set_ylabel("Y (m)")
        self.ax[1].set_title("Trajectory")
        self.ax[1].grid(True)
        self.ax[1].axis("equal")

        plt.draw()
        plt.pause(0.01)


def main():
    rclpy.init()
    node = Plotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
