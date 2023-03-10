#ifndef STINGRAY_COMMUNICATION_UDP_RECEIVER_H
#define STINGRAY_COMMUNICATION_UDP_RECEIVER_H

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

#include "stingray_communication_msgs/msg/hardware_info.hpp"
#include "stingray_communication_msgs/srv/set_float64.hpp"
#include "stingray_communication_msgs/srv/set_int16.hpp"
#include "messages/messages.h"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class UpdReceiver : public rclcpp::Node
{
public:
    UpdReceiver();

private:
    void UdpReceiver::timerCallback();
    void UdpReceiver::wait();
    void UdpReceiver::Receiver();

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr outputMessagePublisher;
    
    std_msgs::msg::UInt8MultiArray outputMessage;
    
    // get json config
    json ros_config;
    json udp_config;
};

#endif // STINGRAY_COMMUNICATION_UDP_RECEIVER_H
