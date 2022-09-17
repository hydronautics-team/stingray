#ifndef STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_NODELET_H
#define STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_NODELET_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

#include <sstream>
#include <string>
#include <vector>

#include <stingray_communication_msgs/SetHorizontalMove.h>
#include <stingray_communication_msgs/SetFloat64.h>
#include <stingray_communication_msgs/SetInt32.h>
#include <stingray_communication_msgs/SetDeviceAction.h>
#include <stingray_communication_msgs/SetStabilization.h>
#include "messages/messages.h"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;

class hardware_bridge : public nodelet::Nodelet
{
public:
    virtual void onInit();

private:
    void inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg);
    bool horizontalMoveCallback(stingray_communication_msgs::SetHorizontalMove::Request &horizontalMoveRequest,
                             stingray_communication_msgs::SetHorizontalMove::Response &horizontalMoveResponse);
    bool depthCallback(stingray_communication_msgs::SetInt32::Request &depthRequest,
                       stingray_communication_msgs::SetInt32::Response &depthResponse);
    bool imuCallback(std_srvs::SetBool::Request &imuRequest,
                     std_srvs::SetBool::Response &imuResponse);
    bool deviceActionCallback(stingray_communication_msgs::SetDeviceAction::Request &deviceRequest,
                              stingray_communication_msgs::SetDeviceAction::Response &deviceResponse);
    bool stabilizationCallback(stingray_communication_msgs::SetStabilization::Request &stabilizationRequest,
                               stingray_communication_msgs::SetStabilization::Response &stabilizationResponse);
    void timerCallback(const ros::TimerEvent &event);

    // ROS publishers
    ros::Publisher outputMessagePublisher;
    ros::Publisher depthPublisher;
    ros::Publisher yawPublisher;
    // ROS subscribers
    ros::Subscriber inputMessageSubscriber;
    // ROS services
    ros::ServiceServer horizontalMoveService;
    ros::ServiceServer depthService;
    ros::ServiceServer imuService;
    ros::ServiceServer stabilizationService;
    ros::ServiceServer deviceActionService;
    // Message containers
    std_msgs::UInt8MultiArray outputMessage; // Hardware bridge -> Protocol_bridge
    std_msgs::Int32 depthMessage;
    std_msgs::Int32 yawMessage;
    RequestMessage requestMessage;
    ResponseMessage responseMessage;
    // Other
    ros::Timer publishingTimer; // Timer for publishing messages

    bool isReady = false; // isTopicUpdated flag
    bool depthStabilizationEnabled = false;
    bool pitchStabilizationEnabled = false;
    bool yawStabilizationEnabled = false;
    bool lagStabilizationEnabled = false;

    // get json config
    json ros_config;
};

#endif // STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_NODELET_H
