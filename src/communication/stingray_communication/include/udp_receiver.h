#ifndef STINGRAY_COMMUNICATION_UDP_RECEIVER_H
#define STINGRAY_COMMUNICATION_UDP_RECEIVER_H

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <fstream>
#include <thread>
#include "messages/messages.h"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class UdpReceiver : public rclcpp::Node
{
public:
    UdpReceiver();
    void wait();
    udp::socket socket;

private:
    void udpReceive_callback(const boost::system::error_code &error, size_t bytes_transferred);

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr outputMessagePublisher;

    std_msgs::msg::UInt8MultiArray outputMessage;
    RequestMessage requestMessage;
    GuiRequestMessage guiRequestMessage;

    // get json config
    json ros_config;
    json udp_config;

    // UPD receiver
    boost::asio::io_service io_service;
    boost::array<uint8_t, 1024> recv_buffer;
    udp::endpoint remote_endpoint;
};

#endif // STINGRAY_COMMUNICATION_UDP_RECEIVER_H
