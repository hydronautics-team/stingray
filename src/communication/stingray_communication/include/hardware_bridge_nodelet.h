#ifndef STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_NODELET_H
#define STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_NODELET_H

#include <ros/ros.h>
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

#include <stingray_communication_msgs/SetLagAndMarch.h>
#include <stingray_communication_msgs/SetFloat64.h>
#include <stingray_communication_msgs/SetInt32.h>
#include <stingray_communication_msgs/SetDeviceAction.h>
#include <stingray_communication_msgs/SetStabilization.h>
#include "messages/messages.h"
#include "TopicsAndServices.h"

class hardware_bridge : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    void inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg);
    bool lagAndMarchCallback(stingray_communication_msgs::SetLagAndMarch::Request& lagAndMarchRequest,
                             stingray_communication_msgs::SetLagAndMarch::Response& lagAndMarchResponse);
    bool depthCallback(stingray_communication_msgs::SetInt32::Request& depthRequest,
                       stingray_communication_msgs::SetInt32::Response& depthResponse);
    bool yawCallback(stingray_communication_msgs::SetInt32::Request& yawRequest,
                     stingray_communication_msgs::SetInt32::Response& yawResponse);
    bool imuCallback(std_srvs::SetBool::Request& imuRequest,
                     std_srvs::SetBool::Response& imuResponse);
    bool deviceActionCallback(stingray_communication_msgs::SetDeviceAction::Request& deviceRequest,
                              stingray_communication_msgs::SetDeviceAction::Response& deviceResponse);
    bool stabilizationCallback(stingray_communication_msgs::SetStabilization::Request& stabilizationRequest,
                               stingray_communication_msgs::SetStabilization::Response& stabilizationResponse);
    void timerCallback(const ros::TimerEvent& event);

    // ROS publishers
    ros::Publisher outputMessagePublisher;
    ros::Publisher depthPublisher;
    ros::Publisher yawPublisher;
    // ROS subscribers
    ros::Subscriber inputMessageSubscriber;
    // ROS services
    ros::ServiceServer lagAndMarchService;
    ros::ServiceServer depthService;
    ros::ServiceServer yawService;
    ros::ServiceServer imuService;
    ros::ServiceServer stabilizationService;
    ros::ServiceServer deviceActionService;
    // Message containers
    std_msgs::UInt8MultiArray outputMessage; // Hardware bridge -> Protocol_bridge
    std_msgs::UInt32 depthMessage;
    std_msgs::Int32 yawMessage;
    RequestMessage requestMessage;
    ResponseMessage responseMessage;
    // Other
    ros::Timer publishingTimer; // Timer for publishing messages

    bool isReady = false; // isTopicUpdated flag
    bool depthStabilizationEnabled = false;
    bool yawStabilizationEnabled = false;
    bool lagStabilizationEnabled = false;
};

#endif //STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_NODELET_H
