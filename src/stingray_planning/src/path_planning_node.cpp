#include "stingray_planning/Route.hpp"
#include "stingray_interfaces/msg/cords_diff.hpp"

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

#include <rclcpp_action/rclcpp_action.hpp>
#include "stingray_interfaces/action/navigate_to_pose.hpp"


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

  using NavigateToPose = stingray_interfaces::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

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
    declare_parameter("use_target_depth", true);
    declare_parameter("fallback_target_depth", 0.0);

    declare_parameter<std::string>("odometry_topic", "/core/state/odometry");
    declare_parameter<std::string>(
      "velocity_observation_topic", "/stingray_core/sensors/dvl/odometry");
    declare_parameter<std::string>("map_topic", "/map/occupancy_local");
    declare_parameter<std::string>("command_topic", "/control/data");
    declare_parameter<std::string>("path_topic", "/planner/global_path");
    declare_parameter<std::string>("cords_diff_topic", "/planner/cords_diff");
    declare_parameter("action_name", "/planner/navigate_to_pose");


    auv_length_ = get_parameter("auv_length").as_double();
    auv_width_ = get_parameter("auv_width").as_double();
    max_vel_ = get_parameter("max_vel").as_double();
    min_vel_ = get_parameter("min_vel").as_double();
    max_angle_vel_ = get_parameter("max_angle_vel").as_double();
    min_angle_vel_ = get_parameter("min_angle_vel").as_double();
    default_goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    goal_tolerance_ = default_goal_tolerance_;
    target_depth_ = get_parameter("target_depth").as_double();
    odometry_timeout_ = get_parameter("odometry_timeout").as_double();
    velocity_observation_timeout_ = get_parameter("velocity_observation_timeout").as_double();
    require_velocity_observation_ = get_parameter("require_velocity_observation").as_bool();
    unknown_is_obstacle_ = get_parameter("unknown_is_obstacle").as_bool();
    use_target_depth_ = get_parameter("use_target_depth").as_bool();
    fallback_target_depth_ = get_parameter("fallback_target_depth").as_double();

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
    

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("command_topic").as_string(), command_qos);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
    pub_cords_ = create_publisher<stingray_interfaces::msg::CordsDiff>(
      get_parameter("cords_diff_topic").as_string(), command_qos);

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, get_parameter("action_name").as_string(),
      std::bind(&PathPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PathPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&PathPlannerNode::handle_accepted, this, std::placeholders::_1));

    const double rate = std::max(1.0, get_parameter("control_rate_hz").as_double());
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&PathPlannerNode::control_tick, this));
    
    RCLCPP_INFO(get_logger(), "Action server ready at %s",
      get_parameter("action_name").as_string().c_str());
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
    result.z = target_depth_;
    return result;
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odometry_ = *msg;
    last_odometry_time_ = now();
    got_odometry_ = true;
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
      safe_stop();
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
    safe_stop();
    got_map_ = true;
    if (active_goal_ && active_goal_->is_active()) {
      finish_goal(NavigateToPose::Result::PLANNING_FAILED,
                  "Map updated during navigation; replan required");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request: (%.2f, %.2f, z=%.2f)",
      goal->target.pose.position.x, goal->target.pose.position.y,
      goal->target.pose.position.z);

    if (!got_map_ || !got_odometry_) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: no map or odometry yet");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_goal_ && active_goal_->is_active()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->timeout <= 0.0f) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: timeout must be > 0");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Cancel request received");
    safe_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    active_goal_ = goal_handle;
    goal_ = goal_handle->get_goal()->target;
    const auto requested_tolerance = goal_handle->get_goal()->position_tolerance;
    goal_tolerance_ = requested_tolerance > 0.0f
      ? requested_tolerance
      : default_goal_tolerance_;
    goal_timeout_ = goal_handle->get_goal()->timeout;
    goal_start_time_ = now();

    target_depth_ = use_target_depth_
      ? goal_.pose.position.z
      : fallback_target_depth_;

    if (!plan_route()) {
      finish_goal(NavigateToPose::Result::PLANNING_FAILED, "Path not found");
      return;
    }
    route_index_ = route_.size() > 1 ? 1 : 0;
    prev_route_index_ = route_index_;
    route_active_ = true;
    publish_path();
    RCLCPP_INFO(get_logger(), "Route accepted: %zu waypoints, depth=%.2f",
                route_.size(), target_depth_);
  }

  bool plan_route()
{
  if (!odometry_.header.frame_id.empty() && odometry_.header.frame_id != map_frame_) {
    RCLCPP_ERROR(get_logger(), "Odometry frame differs from map frame");
    return false;
  }

  Point start, target;
  if (!world_to_grid(odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, start) ||
      !world_to_grid(goal_.pose.position.x, goal_.pose.position.y, target))
  {
    RCLCPP_ERROR(get_logger(), "Start or goal lies outside the occupancy grid");
    return false;
  }
  if (grid_->field[grid_->index(start)] != 0 || grid_->field[grid_->index(target)] != 0) {
    RCLCPP_ERROR(get_logger(), "Start or goal lies in an occupied cell");
    return false;
  }

  const double cells_per_metre = 1.0 / map_resolution_;
  AUV auv(start,
          metres_to_grid_units(auv_length_, cells_per_metre),
          metres_to_grid_units(auv_width_, cells_per_metre),
          max_vel_, min_vel_, max_angle_vel_, min_angle_vel_);
  route_ = auv.build_full_route({target}, *grid_);
  return !route_.empty();
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
      pose.pose.position = grid_to_world(route_[index]);
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
    if (!route_active_ || !active_goal_ || !active_goal_->is_active()) {
      return;
    }
    const bool velocity_observation_fresh = got_velocity_observation_ &&
    (now() - last_velocity_observation_time_).seconds() <= velocity_observation_timeout_ &&
    stamp_is_fresh(last_velocity_observation_stamp_, velocity_observation_timeout_);
    const bool odometry_fresh = got_odometry_ &&
    (now() - last_odometry_time_).seconds() <= odometry_timeout_ &&
    stamp_is_fresh(rclcpp::Time(odometry_.header.stamp), odometry_timeout_);

    if (!odometry_fresh) {
      finish_goal(NavigateToPose::Result::LOCALIZATION_LOST, "Odometry timeout");
      return;
    }
    if (require_velocity_observation_ && !velocity_observation_fresh) {
      finish_goal(NavigateToPose::Result::VELOCITY_FEEDBACK_LOST,
                  "Velocity observation timeout");
      return;
    }
    if ((now() - goal_start_time_).seconds() > goal_timeout_) {
      finish_goal(NavigateToPose::Result::TIMEOUT, "Goal timeout exceeded");
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

    if (route_index_ != prev_route_index_ && route_index_ + 1 < route_.size()){
      const auto reached = waypoint_position(route_index_);
      const auto next = waypoint_position(route_index_ + 1);
      stingray_interfaces::msg::CordsDiff cords;

      cords.x_difference = next.x - reached.x;
      cords.y_difference = next.y - reached.y;
      pub_cords_->publish(cords);
      prev_route_index_ = route_index_;
    }

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose.header.stamp = now();
    feedback->current_pose.header.frame_id = map_frame_;
    feedback->current_pose.pose = odometry_.pose.pose;

    if (route_index_ < route_.size()) {
      const auto waypoint = waypoint_position(route_index_);
      feedback->distance_remaining = static_cast<float>(
        std::hypot(waypoint.x - current_x, waypoint.y - current_y));
    } else {
      feedback->distance_remaining = 0.0f;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header = feedback->current_pose.header;
    path_msg.poses.reserve(route_.size());
    for (const auto & p : route_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position = grid_to_world(p);
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    feedback->path = path_msg;

    active_goal_->publish_feedback(feedback);


    if (route_index_ >= route_.size()) {
      finish_goal(NavigateToPose::Result::SUCCEEDED, "Goal reached");
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

  void safe_stop()
  {
    route_active_ = false;
    pub_cmd_->publish(geometry_msgs::msg::Twist{});
  }

  void finish_goal(uint8_t code, const std::string & message)
  {
    safe_stop();

    if (!active_goal_) return;

    if (active_goal_->is_canceling()) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->code = NavigateToPose::Result::CANCELED;
      result->message = "Canceled";
      active_goal_->canceled(result);
    } else if (code == NavigateToPose::Result::SUCCEEDED) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->code = NavigateToPose::Result::SUCCEEDED;
      result->message = message;
      active_goal_->succeed(result);
    } else {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->code = code;
      result->message = message;
      active_goal_->abort(result);
    }

    active_goal_.reset();
  }

  double auv_length_{1.0};
  double auv_width_{0.4};
  double max_vel_{1.0};
  double min_vel_{0.1};
  double max_angle_vel_{60.0};
  double min_angle_vel_{5.0};
  double default_goal_tolerance_{0.25};
  double goal_tolerance_{0.25};
  double goal_timeout_{30.0};
  double target_depth_{0.5};
  double odometry_timeout_{0.5};
  double velocity_observation_timeout_{0.5};
  double fallback_target_depth_{0.0};
  bool use_target_depth_{true};
  bool unknown_is_obstacle_{true};
  bool require_velocity_observation_{true};
  bool got_odometry_{false};
  bool got_map_{false};
  bool route_active_{false};
  bool got_velocity_observation_{false};
  nav_msgs::msg::Odometry odometry_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time last_odometry_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_observation_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_start_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<GRID> grid_;
  std::vector<Point> route_;
  size_t route_index_{0};
  std::string map_frame_{"map"};
  double map_resolution_{0.0};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  double map_origin_yaw_{0.0};
  size_t prev_route_index_{0};
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_velocity_observation_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<stingray_interfaces::msg::CordsDiff>::SharedPtr pub_cords_;
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
