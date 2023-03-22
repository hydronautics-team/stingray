#ifndef STINGRAY_COMMUNICATION_GUI_BRIDGE
#define STINGRAY_COMMUNICATION_GUI_BRIDGE

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <thread>
#include <vector>
#include <serial/serial.h>

#include "messages/messages.h"
#include "std_msgs/msg/string.hpp"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;

class GuiBridgeSender : public rclcpp::Node {
   public:
    GuiBridgeSender(boost::asio::io_service &io_service);
    ~GuiBridgeSender();

   private:
    void from_driver_callback(const std_msgs::msg::UInt8MultiArray &msg);
    void timerCallback();

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr fromDriverMessageSubscriber;

    // Message containers
    FromDriverMessage fromDriverMessage;
    ToGuiMessage toGuiMessage;

    // get json config
    json ros_config;
    json com_config;

    // udp connection
    boost::asio::io_service &_io_service;
    udp::endpoint _send_endpoint;
    udp::socket _send_socket;

    rclcpp::TimerBase::SharedPtr publishingTimer;  // Timer for publishing messages

    serial::Serial ser;
};

class GuiBridgeReceiver : public rclcpp::Node {
   public:
    GuiBridgeReceiver(boost::asio::io_service &io_service);
    ~GuiBridgeReceiver();
    void try_receive();

   private:
    void from_gui_callback(const boost::system::error_code &error, size_t bytes_transferred);

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr toDriverMessagePublisher;

    // Message containers
    std_msgs::msg::UInt8MultiArray toDriverMessageContainer;
    ToDriverMessage toDriverMessage;
    FromGuiMessage fromGuiMessage;

    // get json config
    json ros_config;
    json com_config;

    // udp connection
    boost::asio::io_service &_io_service;
    udp::endpoint _receive_endpoint;
    udp::socket _receive_socket;
    boost::array<uint8_t, 1024> from_gui_buffer;
};

#endif  // STINGRAY_COMMUNICATION_GUI_BRIDGE
