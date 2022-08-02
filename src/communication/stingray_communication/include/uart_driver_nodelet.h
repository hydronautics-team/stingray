#ifndef STINGRAY_COMMUNICATION_UART_DRIVER_NODELET_H
#define STINGRAY_COMMUNICATION_UART_DRIVER_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <vector>
#include "messages/messages.h"

class uart_driver : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    void portInitialize(ros::NodeHandle& nodeHandle);
    bool sendData();
    bool receiveData();
    void inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg);

    // ROS publishers
    ros::Publisher outputMessage_pub;
    // ROS subscribers
    ros::Subscriber inputMessage_sub;
    // Other
    serial::Serial port;    //serial port
    // Message containers
    std_msgs::UInt8MultiArray inputMessage;// Hardware bridge -> Protocol_driver
    std_msgs::UInt8MultiArray outputMessage;// Protocol_driver -> Hardware bridge
};

#endif //STINGRAY_COMMUNICATION_UART_DRIVER_NODELET_H
