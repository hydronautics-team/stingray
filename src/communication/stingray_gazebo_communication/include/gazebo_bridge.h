#ifndef STINGRAY_GAZEBO_COMMUNICATION_GAZEBO_BRIDGE_H
#define STINGRAY_GAZEBO_COMMUNICATION_GAZEBO_BRIDGE_H

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stingray_utils/json.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "gazebo_msgs/msg/model_state.hpp"
#include "gazebo_msgs/srv/get_model_state.hpp"
#include "gazebo_msgs/srv/set_model_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "stingray_communication_msgs/srv/set_device_action.hpp"
#include "stingray_communication_msgs/srv/set_float64.hpp"
#include "stingray_communication_msgs/srv/set_horizontal_move.hpp"
#include "stingray_communication_msgs/srv/set_int16.hpp"
#include "stingray_communication_msgs/srv/set_stabilization.hpp"
#include "tf2/utils.h"

using json = nlohmann::json;
using namespace std::chrono_literals;

class GazeboBridge : public rclcpp::Node {
   public:
    GazeboBridge();

   private:
    std::shared_ptr<gazebo_msgs::srv::GetModelState::Response> getModelState(const std::string &model_name);
    void updateModelState(const std::function<void(gazebo_msgs::msg::ModelState &)> &transform);
    std::pair<std_msgs::msg::Int32, std_msgs::msg::Int32> pingerStatus(const std::string &pinger_name, const float &yaw);

    void horizontalMoveCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Request> request,
                                std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Response> response);
    void depthCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetInt16::Request> request,
                       std::shared_ptr<stingray_communication_msgs::srv::SetInt16::Response> response);
    void imuCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void deviceActionCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Request> request,
                              std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Response> response);
    void stabilizationCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Request> request,
                               std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Response> response);
    void timerCallback();

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depthPublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yawPublisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pingerBucketPublisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pingerFlarePublisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;
    // ROS service servers
    rclcpp::Service<stingray_communication_msgs::srv::SetHorizontalMove>::SharedPtr horizontalMoveService;
    rclcpp::Service<stingray_communication_msgs::srv::SetInt16>::SharedPtr depthService;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imuService;
    rclcpp::Service<stingray_communication_msgs::srv::SetDeviceAction>::SharedPtr deviceActionService;
    rclcpp::Service<stingray_communication_msgs::srv::SetStabilization>::SharedPtr stabilizationService;
    // ROS service clients
    rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedPtr getModelStateService;
    rclcpp::Client<gazebo_msgs::srv::SetModelState>::SharedPtr setModelStateService;

    // Message containers
    std_msgs::msg::Float64 depthMessage;
    std_msgs::msg::Float64 yawMessage;
    // Other
    rclcpp::TimerBase::SharedPtr publishingTimer;  // Timer for publishing messages

    bool isReady = false;  // isTopicUpdated flag
    bool depthStabilizationEnabled = false;
    bool pitchStabilizationEnabled = false;
    bool yawStabilizationEnabled = true;
    bool lagStabilizationEnabled = false;

    float currentYaw;
    float currentDepth;
    std_msgs::msg::Int32 pingerBucketMessage;
    std_msgs::msg::Int32 pingerFlareMessage;
    geometry_msgs::msg::Twist currentTwist;

    // get json config
    json ros_config;
    json simulation_config;
};

#endif  // STINGRAY_GAZEBO_COMMUNICATION_GAZEBO_BRIDGE_H
