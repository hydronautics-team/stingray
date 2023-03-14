#include "udp_sender.h"

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <fstream>
#include <thread>

UdpSender::UdpSender() : Node("UdpSender"), io_service(), socket(io_service) {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    udp_config = json::parse(std::ifstream("resources/configs/udp.json"));

    // UDP sender
    remote_endpoint = udp::endpoint(address::from_string(udp_config["udp_sender"]["ip_address"]), udp_config["udp_sender"]["udp_port"].get<int>());
    socket.open(udp::v4());
    RCLCPP_INFO(this->get_logger(), "UdpSender: socket opened. Address: %s, port: %d",
                udp_config["udp_sender"]["ip_address"].get<std::string>().c_str(), udp_config["udp_sender"]["udp_port"].get<int>());

    // ROS subscribers
    this->inputMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        ros_config["topics"]["from_driver_parcel"], 1000, std::bind(&UdpSender::udpSender_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "UdpSender: Constructed");
}

UdpSender::~UdpSender() { socket.close(); }

void UdpSender::udpSender_callback(const std_msgs::msg::UInt8MultiArray &msg) {
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < ToGuiMessage::length; i++) {
        received_vector.push_back(msg.data[i]);
    }
    bool ok = guiMessage.parseVector(received_vector);
    if (ok) {
        boost::system::error_code err;
        socket.send_to(boost::asio::buffer(received_vector), remote_endpoint, 0, err);
    } else
        RCLCPP_WARN(this->get_logger(), "Wrong checksum");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<UdpSender> node = std::make_shared<UdpSender>();
    rclcpp::spin(node);
    // node.get()->socket.close();
    rclcpp::shutdown();
    return 0;
}