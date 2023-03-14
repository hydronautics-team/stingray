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
#include <vector>
#include <thread>

#include "messages/messages.h"
#include "std_msgs/msg/string.hpp"
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;

class GuiBridge : public rclcpp::Node {
   public:
    GuiBridge();
    ~GuiBridge();
    void receive_loop();

   private:
    void from_driver_callback(const std_msgs::msg::UInt8MultiArray &msg);
    void from_gui_callback(const boost::system::error_code &error, size_t bytes_transferred);
    void timerCallback();

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr fromDriverMessageSubscriber;
    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr toDriverMessagePublisher;

    rclcpp::CallbackGroup::SharedPtr receiver_group_;
    rclcpp::CallbackGroup::SharedPtr sender_group_;

    // Message containers
    std_msgs::msg::UInt8MultiArray toDriverMessageContainer;
    ToDriverMessage toDriverMessage;
    FromDriverMessage fromDriverMessage;
    ToGuiMessage toGuiMessage;
    FromGuiMessage fromGuiMessage;

    // get json config
    json ros_config;
    json com_config;

    // udp connection
    boost::asio::io_service _io_service;
    udp::endpoint _send_endpoint;
    udp::endpoint _receive_endpoint;
    udp::socket _send_socket;
    udp::socket _receive_socket;
    boost::array<uint8_t, 1024> from_gui_buffer;


    rclcpp::TimerBase::SharedPtr publishingTimer; // Timer for publishing messages

};

#endif  // STINGRAY_COMMUNICATION_GUI_BRIDGE
