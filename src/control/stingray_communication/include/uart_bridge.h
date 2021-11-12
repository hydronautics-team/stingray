#ifndef STINGRAY_COMMUNICATION_UART_BRIDGE_H
#define STINGRAY_COMMUNICATION_UART_BRIDGE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <serial/serial.h>
#include <sstream>
#include <string>
#include <vector>
#include "messages/messages.h"
#include "topics_and_services.h"

using std::placeholders::_1;


class UartBridge : public rclcpp::Node {
public:
    UartBridge();

private:
    void inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const;
    void portInitialize();
    bool sendData();
    bool receiveData();

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr outputMessage_pub;
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inputMessage_sub;
    // Other
    serial::Serial port;    //serial port
    // Message containers
    std_msgs::msg::UInt8MultiArray inputMessage;// Hardware bridge -> Protocol_driver
    std_msgs::msg::UInt8MultiArray outputMessage;// Protocol_driver -> Hardware bridge
};

#endif //STINGRAY_COMMUNICATION_UART_BRIDGE_H
