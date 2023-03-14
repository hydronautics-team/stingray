#ifndef STINGRAY_COMMUNICATION_UPD_SENDER
#define STINGRAY_COMMUNICATION_UPD_SENDER

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <vector>

#include "messages/messages.h"
#include "std_msgs/msg/string.hpp"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;

class UdpSender : public rclcpp::Node {
   public:
    UdpSender();
    ~UdpSender();

   private:
    void udpSender_callback(const std_msgs::msg::UInt8MultiArray &msg);

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr inputMessageSubscriber;
    // Message containers
    std_msgs::msg::UInt8MultiArray outputMessage;
    FromDriverMessage responseMessage;
    ToGuiMessage guiMessage;

    // get json config
    json ros_config;
    json udp_config;

    // udp
    boost::asio::io_service io_service;
    udp::endpoint remote_endpoint;
    udp::socket socket;
};

#endif  // STINGRAY_COMMUNICATION_UPD_SENDER
