//
// Created by VLADUSHKED on 25.07.2019.
//

#ifndef STINGRAY_DRIVERS_HARDWARE_BRIDGE_NODELET_H
#define STINGRAY_DRIVERS_HARDWARE_BRIDGE_NODELET_H

#include "ros/ros.h"
#include <nodelet/nodelet.h>

#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"

#include <stingray_msgs/SetLagAndMarch.h>
#include <stingray_msgs/SetFloat64.h>
#include <stingray_msgs/SetInt32.h>
#include <stingray_msgs/SetDeviceAction.h>
#include <stingray_msgs/SetStabilization.h>

#include <sstream>
#include <string>
#include <vector>
#include <std_msgs/UInt32.h>
#include <boost/shared_ptr.hpp>

#include "messages/messages.h"

class hardware_bridge : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    void set_bit(uint8_t &byte, uint8_t bit, bool state);
    void inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg);

    bool lagAndMarchCallback(stingray_msgs::SetLagAndMarch::Request& lagAndMarchRequest,
                              stingray_msgs::SetLagAndMarch::Response& lagAndMarchResponse);
    bool depthCallback(stingray_msgs::SetFloat64::Request& depthRequest,
                             stingray_msgs::SetFloat64::Response& depthResponse);
    bool yawCallback(stingray_msgs::SetFloat64::Request& yawRequest,
                       stingray_msgs::SetFloat64::Response& yawResponse);
    bool imuCallback(stingray_msgs::SetFloat64::Request& imuRequest,
                     stingray_msgs::SetFloat64::Response& imuResponse);
    bool deviceActionCallback(stingray_msgs::SetDeviceAction::Request& deviceRequest,
                              stingray_msgs::SetDeviceAction::Response& deviceResponse);
    bool stabilizationCallback(stingray_msgs::SetStabilization::Request& stabilizationRequest,
                                stingray_msgs::SetStabilization::Response& stabilizationResponse);

    void timerCallback(const ros::TimerEvent& event);

    ros::Publisher outputMessage_pub;
    ros::Publisher depth_pub;
    ros::Subscriber inputMessage_sub;

    ros::ServiceServer lagAndMarchSrv;
    ros::ServiceServer depthSrv;
    ros::ServiceServer yawSrv;
    ros::ServiceServer imuSrv;
    ros::ServiceServer deviceActionSrv;
    ros::ServiceServer stabilizationSrv;

    // Hardware bridge -> Protocol_bridge
    std_msgs::UInt8MultiArray msg_out;
    std_msgs::UInt32 depth_message;

    RequestMessage request;
    ResponseMessage response;
    ros::Timer timer;

    int current_depth;
    bool isTopicUpdated = false;
};

#endif //STINGRAY_DRIVERS_HARDWARE_BRIDGE_NODELET_H
