#ifndef STINGRAY_COMMUNICATION_UDP_BRIDGE_H
#define STINGRAY_COMMUNICATION_UDP_BRIDGE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "std_msgs/msg/string.hpp"

#include <sstream>
#include <string>
#include <vector>

#include "stingray_communication_msgs/msg/upd_info.hpp"
#include "stingray_communication_msgs/srv/set_horizontal_move.hpp"
#include "stingray_communication_msgs/srv/set_float64.hpp"
#include "stingray_communication_msgs/srv/set_int16.hpp"
#include "stingray_communication_msgs/srv/set_device_action.hpp"
#include "stingray_communication_msgs/srv/set_stabilization.hpp"
#include "messages/messages.h"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class udpBridge : public rclcpp::Node
{
public:
    udpBridge();

private:
    void inputMessage_callback(const std_msgs::msg::UInt8MultiArray &msg);
    void horizontalMoveCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Request> request,
                                std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Response> response);
    void depthCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetInt16::Request> request,
                       std::shared_ptr<stingray_communication_msgs::srv::SetInt16::Response> response);
    void imuCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void deviceActionCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Request> request,
                              std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Response> response);
    void stabilizationCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Request> request,
                               std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Response> response);
    void timerCallback();

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr outputMessagePublisher;
    rclcpp::Publisher<stingray_communication_msgs::msg::udpInfo>::SharedPtr updInfoPublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depthPublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yawPublisher;
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inputMessageSubscriber;
    // ROS services
    rclcpp::Service<stingray_communication_msgs::srv::SetHorizontalMove>::SharedPtr horizontalMoveService;
    rclcpp::Service<stingray_communication_msgs::srv::SetInt16>::SharedPtr depthService;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imuService;
    rclcpp::Service<stingray_communication_msgs::srv::SetDeviceAction>::SharedPtr deviceActionService;
    rclcpp::Service<stingray_communication_msgs::srv::SetStabilization>::SharedPtr stabilizationService;
    // Message containers
    stingray_communication_msgs::msg::udpInfo updInfoMessage;
    std_msgs::msg::UInt8MultiArray outputMessage; // upd bridge -> Protocol_bridge
    std_msgs::msg::Float64 depthMessage;
    std_msgs::msg::Float64 yawMessage;
    RequestMessage requestMessage;
    ResponseMessage responseMessage;
    // Other
    rclcpp::TimerBase::SharedPtr publishingTimer; // Timer for publishing messages

    bool isReady = false; // isTopicUpdated flag
    bool depthStabilizationEnabled = false;
    bool pitchStabilizationEnabled = false;
    bool yawStabilizationEnabled = false;
    bool lagStabilizationEnabled = false;

    float currentYaw;
    float currentDepth;

    // get json config
    json ros_config;
};

#endif // STINGRAY_COMMUNICATION_UDP_BRIDGE_H
