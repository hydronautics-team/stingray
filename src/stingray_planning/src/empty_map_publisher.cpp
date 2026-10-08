#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class EmptyMapPublisher : public rclcpp::Node
{
public:
  EmptyMapPublisher() : Node("empty_map_publisher")
  {
    declare_parameter("topic", "/map/occupancy_local");
    declare_parameter("frame_id", "odom");
    declare_parameter("size_m", 20.0);
    declare_parameter("resolution", 0.1);

    const auto topic = get_parameter("topic").as_string();
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic, qos);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&EmptyMapPublisher::publish_map, this));
  }

private:
  void publish_map()
  {
    const double resolution = std::max(0.01, get_parameter("resolution").as_double());
    const double size_m = std::max(resolution, get_parameter("size_m").as_double());
    const auto cells = static_cast<uint32_t>(size_m / resolution);

    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = now();
    map.header.frame_id = get_parameter("frame_id").as_string();
    map.info.map_load_time = map.header.stamp;
    map.info.resolution = static_cast<float>(resolution);
    map.info.width = cells;
    map.info.height = cells;
    map.info.origin.position.x = -size_m / 2.0;
    map.info.origin.position.y = -size_m / 2.0;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<size_t>(cells) * cells, int8_t{0});
    publisher_->publish(map);
    timer_->cancel();
    RCLCPP_INFO(
      get_logger(), "Published %.1f x %.1f m empty pool map in frame '%s'",
      size_m, size_m, map.header.frame_id.c_str());
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmptyMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
