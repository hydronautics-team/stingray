//
// Created by VLADUSHKED on 25.07.2019.
//

#ifndef STINGRAY_DRIVERS_UART_DRIVER_NODELET_H
#define STINGRAY_DRIVERS_UART_DRIVER_NODELET_H

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8MultiArray.h"
#include "serial/serial.h"
#include "messages/messages.h"

#include <sstream>
#include <string>
#include <vector>

class uart_driver : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    bool sendData();
    bool receiveData();
    void inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg);

    ros::Publisher outputMessage_pub;
    ros::Subscriber inputMessage_sub;
    serial::Serial port;    //serial port
    // Hardware bridge -> Protocol_bridge
    std_msgs::UInt8MultiArray msg_in;
    // Protocol_bridge -> Hardware bridge
    std_msgs::UInt8MultiArray msg_out;
};

#endif //STINGRAY_DRIVERS_UART_DRIVER_NODELET_H
