#ifndef STINGRAY_COMMUNICATION_UPD_SENDER
#define STINGRAY_COMMUNICATION_UPD_SENDER

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
#include "stingray_communication_msgs/srv/set_float64.hpp"
#include "stingray_communication_msgs/srv/set_int16.hpp"
#include "messages/messages.h"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class UdpSender : public rclcpp::Node
{
public:
    UdpSender();

private:
    void udpSender_callback(const std_msgs::msg::UInt8MultiArray &msg);

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inputMessageSubscriber;
  // Message containers
    stingray_communication_msgs::msg::udpInfo updInfoMessage;
    std_msgs::msg::UInt8MultiArray outputMessage;
    RequestMessage requestMessage;
    ResponseMessage responseMessage;

    // get json config
    json ros_config;
    json udp_config;
};

#endif // STINGRAY_COMMUNICATION_UPD_SENDER
