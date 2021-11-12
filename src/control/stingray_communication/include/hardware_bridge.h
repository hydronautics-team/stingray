#ifndef STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_H
#define STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include "stingray_communication_msgs/srv/set_device_action.hpp"
#include "stingray_communication_msgs/srv/set_float64.hpp"
#include "stingray_communication_msgs/srv/set_int32.hpp"
#include "stingray_communication_msgs/srv/set_lag_and_march.hpp"
#include "stingray_communication_msgs/srv/set_stabilization.hpp"
#include "messages/messages.h"
#include "topics_and_services.h"

#include <sstream>
#include <string>
#include <vector>


class HardwareBridge : public rclcpp::Node {
public:
    HardwareBridge();

private:
    void inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void lagAndMarchCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetLagAndMarch::Request> request,
                             std::shared_ptr <stingray_communication_msgs::srv::SetLagAndMarch::Response> response);
    void depthCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Request> request,
                       std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Response> response);
    void yawCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Request> request,
                     std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Response> response);
    void imuCallback(const std::shared_ptr <std_srvs::srv::SetBool::Request> request,
                     std::shared_ptr <std_srvs::srv::SetBool::Response> response);
    void
    deviceActionCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetDeviceAction::Request> request,
                         std::shared_ptr <stingray_communication_msgs::srv::SetDeviceAction::Response> response);
    void
    stabilizationCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetStabilization::Request> request,
                          std::shared_ptr <stingray_communication_msgs::srv::SetStabilization::Response> response);
    void timer_callback();

private:
    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr outputMessagePublisher;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr depthPublisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr yawPublisher;
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inputMessageSubscriber;
    // ROS services
    rclcpp::Service<stingray_communication_msgs::srv::SetLagAndMarch>::SharedPtr lagAndMarchService;
    rclcpp::Service<stingray_communication_msgs::srv::SetInt32>::SharedPtr depthService;
    rclcpp::Service<stingray_communication_msgs::srv::SetInt32>::SharedPtr yawService;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imuService;
    rclcpp::Service<stingray_communication_msgs::srv::SetDeviceAction>::SharedPtr deviceActionService;
    rclcpp::Service<stingray_communication_msgs::srv::SetStabilization>::SharedPtr stabilizationService;
    // Other
    rclcpp::TimerBase::SharedPtr publishingTimer; // Timer for publishing messages
    // Message containers
    std_msgs::msg::UInt8MultiArray outputMessage; // Hardware bridge -> Protocol_bridge
    std_msgs::msg::UInt32 depthMessage;
    std_msgs::msg::Int32 yawMessage;
    RequestMessage requestMessage;
    ResponseMessage responseMessage;
    bool isReady = false; // isTopicUpdated flag
    bool depthStabilizationEnabled = false;
    bool yawStabilizationEnabled = false;
    bool lagStabilizationEnabled = false;
};

#endif //STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_H
