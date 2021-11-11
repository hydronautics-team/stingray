#ifndef STINGRAY_DRIVERS_HARDWARE_BRIDGE_H
#define STINGRAY_DRIVERS_HARDWARE_BRIDGE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/UInt16.hpp"
#include "std_msgs/msg/UInt32.hpp"
#include "std_msgs/msg/Int32.hpp"
#include "std_msgs/msg/UInt8MultiArray.hpp"
#include "geometry_msgs/msg/Twist.hpp"
#include <std_srvs/SetBool.h>

#include "stingray_communication_msgs/srv/SetDeviceAction.hpp"
#include "stingray_communication_msgs/srv/SetFloat64.hpp"
#include "stingray_communication_msgs/srv/SetInt32.hpp"
#include "stingray_communication_msgs/srv/SetLagAndMarch.hpp"
#include "stingray_communication_msgs/srv/SetStabilization.hpp"
#include "messages/messages.h"
#include "TopicsAndServices.h"

#include <sstream>
#include <string>
#include <vector>


using std::placeholders::_1;

class HardwareBridge : public rclcpp::Node {
public:
    HardwareBridge() : Node("hardware_bridge");

private:
    void inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const;
    void lagAndMarchCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetLagAndMarch::Request> request,
                             std::shared_ptr <tutorial_interfaces::srv::SetLagAndMarch::Response> response);
    void depthCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Request> request,
                       std::shared_ptr <tutorial_interfaces::srv::SetInt32::Response> response);
    void yawCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Request> request,
                     std::shared_ptr <tutorial_interfaces::srv::SetInt32::Response> response);
    void imuCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetBool::Request> request,
                     std::shared_ptr <tutorial_interfaces::srv::SetBool::Response> response);
    void
    deviceActionCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetDeviceAction::Request> request,
                         std::shared_ptr <tutorial_interfaces::srv::SetDeviceAction::Response> response);
    void
    stabilizationCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetStabilization::Request> request,
                          std::shared_ptr <tutorial_interfaces::srv::SetStabilization::Response> response);
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
    rclcpp::Service<stingray_communication_msgs::srv::SetBool>::SharedPtr imuService;
    rclcpp::Service<tutorial_interfaces::srv::SetDeviceAction>::SharedPtr deviceActionService;
    rclcpp::Service<tutorial_interfaces::srv::SetStabilization>::SharedPtr stabilizationService;
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

#endif //STINGRAY_DRIVERS_HARDWARE_BRIDGE_H
