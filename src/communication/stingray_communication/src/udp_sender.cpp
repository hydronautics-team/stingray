#include "udp_sender.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <fstream>

UdpSender::UdpSender() : Node("UdpSender") {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    udp_config = json::parse(std::ifstream("resources/configs/udp.json"));

    // ROS subscribers
    this->inputMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        ros_config["topics"]["input_parcel"], 1000, std::bind(&UdpSender::udpSender_callback, \
                                                                this, std::placeholders::_1));

    // UPD sender
    using boost::asio::ip::udp;
    using boost::asio::ip::address;
    boost::asio::io_service io_service;
    udp::socket socket(io_service);
    udp::endpoint remote_endpoint = udp::endpoint(address::from_string(udp_config["udp_sender"]["ip_address"]), \
                                                                        udp_config["udp_sender"]["udp_port"]);
    socket.open(udp::v4());
}

void UdpSender::udpSender_callback(const std_msgs::msg::UInt8MultiArray &msg) {
    std::vector<uint8_t> gui_vector;
    for (int i = 0; i < GuiMessage::lengthResponse; i++) {
        gui_vector.push_back(msg.data[i]);
    }
    bool ok = guiMessage.parseVector(gui_vector);
    if (ok) {
        depthMessage.data = guiMessage.depth;  // Convert metres to centimetres
        RCLCPP_INFO(this->get_logger(), "Received depth: %f", guiMessage.depth);
        // TODO: Test yaw obtaining
        yawMessage.data = guiMessage.yaw;
        RCLCPP_INFO(this->get_logger(), "Received yaw: %f", guiMessage.yaw);

        
        boost::system::error_code err;
        auto sent = socket.send_to(boost::asio::buffer(guiMessage), remote_endpoint, 0, err);
    } else
        RCLCPP_WARN(this->get_logger(), "Wrong checksum");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<UdpSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    socket.close();
    return 0;
}