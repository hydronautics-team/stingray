#ifndef STINGRAY_COMMUNICATION_UPD_SENDER
#define STINGRAY_COMMUNICATION_UPD_SENDER

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>

#include "messages/messages.h"
#include "std_msgs/msg/string.hpp"
#include "stingray_communication_msgs/srv/set_float64.hpp"
#include "stingray_communication_msgs/srv/set_int16.hpp"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class UdpSender : public rclcpp::Node {
   public:
    UdpSender();
    udp::socket socket;

   private:
    void udpSender_callback(const std_msgs::msg::UInt8MultiArray &msg);

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inputMessageSubscriber;
    // Message containers
    std_msgs::msg::UInt8MultiArray outputMessage;
    RequestMessage requestMessage;
    ResponseMessage responseMessage;
    GuiMessage guiMessage;

    // get json config
    json ros_config;
    json udp_config;

    // udp
    boost::asio::io_service io_service;
    udp::endpoint remote_endpoint;
};

#endif  // STINGRAY_COMMUNICATION_UPD_SENDER
