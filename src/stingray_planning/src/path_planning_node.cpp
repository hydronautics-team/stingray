#include "stingray_planning/Route.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray::planning
{
namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalize_angle_rad(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode() : Node("path_planning")
  {
    declare_parameter("auv_length", 1.0);
    declare_parameter("auv_width", 0.4);
    declare_parameter("max_vel", 1.0);
    declare_parameter("min_vel", 0.1);
    declare_parameter("max_angle_vel", 60.0);
    declare_parameter("min_angle_vel", 5.0);
    declare_parameter("goal_tolerance", 0.25);
    declare_parameter("target_depth", 0.5);
    declare_parameter("control_rate_hz", 10.0);
    declare_parameter("odometry_timeout", 0.5);
    declare_parameter("velocity_observation_timeout", 0.5);
    declare_parameter("require_velocity_observation", true);
    declare_parameter("unknown_is_obstacle", true);
    declare_parameter("shutdown_on_complete", false);
    declare_parameter<std::string>("odometry_topic", "/core/state/odometry");
    declare_parameter<std::string>(
      "velocity_observation_topic", "/stingray_core/sensors/dvl/odometry");
    declare_parameter<std::string>("map_topic", "/map/occupancy_local");
    declare_parameter<std::string>("goal_topic", "/planner/goal");
    declare_parameter<std::string>("command_topic", "/control/data");
    declare_parameter<std::string>("path_topic", "/planner/global_path");

    auv_length_ = get_parameter("auv_length").as_double();
    auv_width_ = get_parameter("auv_width").as_double();
    max_vel_ = get_parameter("max_vel").as_double();
    min_vel_ = get_parameter("min_vel").as_double();
    max_angle_vel_ = get_parameter("max_angle_vel").as_double();
    min_angle_vel_ = get_parameter("min_angle_vel").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    target_depth_ = get_parameter("target_depth").as_double();
    odometry_timeout_ = get_parameter("odometry_timeout").as_double();
    velocity_observation_timeout_ = get_parameter("velocity_observation_timeout").as_double();
    require_velocity_observation_ = get_parameter("require_velocity_observation").as_bool();
    unknown_is_obstacle_ = get_parameter("unknown_is_obstacle").as_bool();

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&PathPlannerNode::odometry_callback, this, std::placeholders::_1));
    sub_velocity_observation_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("velocity_observation_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&PathPlannerNode::velocity_observation_callback, this, std::placeholders::_1));
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&PathPlannerNode::map_callback, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(), command_qos,
      std::bind(&PathPlannerNode::goal_callback, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("command_topic").as_string(), command_qos);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());

    const double rate = std::max(1.0, get_parameter("control_rate_hz").as_double());
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&PathPlannerNode::control_tick, this));
  }

private:
  bool world_to_grid(double world_x, double world_y, Point & point) const
  {
    if (!grid_ || map_resolution_ <= 0.0) {
      return false;
    }
    const double dx = world_x - map_origin_x_;
    const double dy = world_y - map_origin_y_;
    const double local_x = std::cos(map_origin_yaw_) * dx + std::sin(map_origin_yaw_) * dy;
    const double local_y = -std::sin(map_origin_yaw_) * dx + std::cos(map_origin_yaw_) * dy;
    point = {
      static_cast<int>(std::floor(local_x / map_resolution_)),
      static_cast<int>(std::floor(local_y / map_resolution_))};
    return grid_->inside(point.x, point.y);
  }

  geometry_msgs::msg::Point grid_to_world(const Point & point) const
  {
    const double local_x = (static_cast<double>(point.x) + 0.5) * map_resolution_;
    const double local_y = (static_cast<double>(point.y) + 0.5) * map_resolution_;
    geometry_msgs::msg::Point result;
    result.x = map_origin_x_ + std::cos(map_origin_yaw_) * local_x -
      std::sin(map_origin_yaw_) * local_y;
    result.y = map_origin_y_ + std::sin(map_origin_yaw_) * local_x +
      std::cos(map_origin_yaw_) * local_y;
    return result;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odometry_ = *msg;
    last_odometry_time_ = now();
    got_odometry_ = true;
    if (got_map_ && got_goal_ && plan_requested_) {
      plan_route();
    }
  }

  void velocity_observation_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_velocity_observation_stamp_ = rclcpp::Time(msg->header.stamp);
    last_velocity_observation_time_ = now();
    got_velocity_observation_ = true;
  }

  bool stamp_is_fresh(const rclcpp::Time & stamp, double timeout) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    const double age = (now() - stamp).seconds();
    return age >= 0.0 && age <= timeout;
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const auto expected_size = static_cast<size_t>(msg->info.width) * msg->info.height;
    if (msg->info.resolution <= 0.0 || msg->data.size() != expected_size) {
      RCLCPP_ERROR(get_logger(), "Invalid occupancy grid dimensions or resolution");
      stop();
      return;
    }

    map_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    map_resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_origin_yaw_ = yaw_from_quaternion(msg->info.origin.orientation);
    grid_ = std::make_unique<GRID>(
      static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
      std::vector<Point>{}, std::vector<Point>{});
    for (size_t i = 0; i < msg->data.size(); ++i) {
      const int8_t occupancy = msg->data[i];
      grid_->field[i] = (occupancy > 50 || (occupancy < 0 && unknown_is_obstacle_)) ? 1 : 0;
    }
    stop();
    got_map_ = true;
    plan_requested_ = true;
    if (got_odometry_ && got_goal_) {
      plan_route();
    }
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && got_map_ && msg->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Goal frame '%s' differs from map frame '%s'; TF conversion is required",
        msg->header.frame_id.c_str(), map_frame_.c_str());
      stop();
      return;
    }
    goal_ = *msg;
    got_goal_ = true;
    stop();
    plan_requested_ = true;
    if (got_odometry_ && got_map_) {
      plan_route();
    }
  }

  void plan_route()
  {
    plan_requested_ = false;
    if (!odometry_.header.frame_id.empty() && odometry_.header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Odometry frame '%s' differs from map frame '%s'; TF conversion is required",
        odometry_.header.frame_id.c_str(), map_frame_.c_str());
      stop();
      return;
    }

    Point start;
    Point target;
    if (!world_to_grid(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, start) ||
      !world_to_grid(goal_.pose.position.x, goal_.pose.position.y, target))
    {
      RCLCPP_ERROR(get_logger(), "Start or goal lies outside the occupancy grid");
      stop();
      return;
    }
    if (grid_->field[grid_->index(start)] != 0 || grid_->field[grid_->index(target)] != 0) {
      RCLCPP_ERROR(get_logger(), "Start or goal lies in an occupied cell");
      stop();
      return;
    }

    const double cells_per_metre = 1.0 / map_resolution_;
    AUV auv(
      start, metres_to_grid_units(auv_length_, cells_per_metre),
      metres_to_grid_units(auv_width_, cells_per_metre), max_vel_, min_vel_,
      max_angle_vel_, min_angle_vel_);
    route_ = auv.build_full_route({target}, *grid_);
    if (route_.empty()) {
      RCLCPP_ERROR(get_logger(), "Path not found");
      stop();
      return;
    }

    route_index_ = route_.size() > 1 ? 1 : 0;
    route_active_ = true;
    publish_path();
    RCLCPP_INFO(get_logger(), "Planned route with %zu waypoints", route_.size());
  }

  void publish_path()
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.poses.reserve(route_.size());
    for (size_t index = 0; index < route_.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = waypoint_position(index);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    pub_path_->publish(msg);
  }

  geometry_msgs::msg::Point waypoint_position(size_t index) const
  {
    if (index + 1 == route_.size()) {
      return goal_.pose.position;
    }
    return grid_to_world(route_[index]);
  }

  void control_tick()
  {
    if (!route_active_) {
      return;
    }
    const bool odometry_fresh = got_odometry_ &&
      (now() - last_odometry_time_).seconds() <= odometry_timeout_ &&
      stamp_is_fresh(rclcpp::Time(odometry_.header.stamp), odometry_timeout_);
    if (!odometry_fresh) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Odometry timeout; stopping");
      stop();
      return;
    }
    const bool velocity_observation_fresh = got_velocity_observation_ &&
      (now() - last_velocity_observation_time_).seconds() <= velocity_observation_timeout_ &&
      stamp_is_fresh(last_velocity_observation_stamp_, velocity_observation_timeout_);
    if (require_velocity_observation_ && !velocity_observation_fresh) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Velocity observation timeout; stopping");
      stop();
      return;
    }

    const double current_x = odometry_.pose.pose.position.x;
    const double current_y = odometry_.pose.pose.position.y;
    while (route_index_ < route_.size()) {
      const auto waypoint = waypoint_position(route_index_);
      if (std::hypot(waypoint.x - current_x, waypoint.y - current_y) > goal_tolerance_) {
        break;
      }
      ++route_index_;
    }
    if (route_index_ >= route_.size()) {
      RCLCPP_INFO(get_logger(), "Goal reached");
      stop();
      if (get_parameter("shutdown_on_complete").as_bool()) {
        rclcpp::shutdown();
      }
      return;
    }

    const auto waypoint = waypoint_position(route_index_);
    const double dx = waypoint.x - current_x;
    const double dy = waypoint.y - current_y;
    const double distance = std::hypot(dx, dy);
    const double yaw = yaw_from_quaternion(odometry_.pose.pose.orientation);
    const double heading_error = normalize_angle_rad(std::atan2(dy, dx) - yaw);
    const double speed = std::clamp(distance, min_vel_, max_vel_);

    geometry_msgs::msg::Twist command;
    command.linear.z = target_depth_;
    command.linear.x = speed * std::cos(heading_error);
    command.linear.y = speed * std::sin(heading_error);
    if (std::abs(heading_error) > 1e-3) {
      const double angular_speed_deg = std::clamp(
        std::abs(heading_error) * 180.0 / M_PI, min_angle_vel_, max_angle_vel_);
      // stingray_core_control integrates angular.z into degree-based attitude setpoints.
      command.angular.z = std::copysign(angular_speed_deg, heading_error);
    }
    pub_cmd_->publish(command);
  }

  void stop()
  {
    route_active_ = false;
    geometry_msgs::msg::Twist command;
    command.linear.z = target_depth_;
    pub_cmd_->publish(command);
  }

  double auv_length_{1.0};
  double auv_width_{0.4};
  double max_vel_{1.0};
  double min_vel_{0.1};
  double max_angle_vel_{60.0};
  double min_angle_vel_{5.0};
  double goal_tolerance_{0.25};
  double target_depth_{0.5};
  double odometry_timeout_{0.5};
  double velocity_observation_timeout_{0.5};
  bool unknown_is_obstacle_{true};
  bool require_velocity_observation_{true};
  bool got_odometry_{false};
  bool got_map_{false};
  bool got_goal_{false};
  bool plan_requested_{false};
  bool route_active_{false};
  bool got_velocity_observation_{false};
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_observation_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<GRID> grid_;
  std::vector<Point> route_;
  size_t route_index_{0};
  std::string map_frame_{"map"};
  double map_resolution_{0.0};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_origin_yaw_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_velocity_observation_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};
}  // namespace stingray::planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stingray::planning::PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
