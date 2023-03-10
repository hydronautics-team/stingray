#include "udp_sender.h"

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <fstream>
#include <thread>

UdpSender::UdpSender() : Node("UdpSender") {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    udp_config = json::parse(std::ifstream("resources/configs/udp.json"));

    // UDP sender
    socket = udp::socket(io_service);
    remote_endpoint = udp::endpoint(address::from_string(udp_config["udp_sender"]["ip_address"]), udp_config["udp_sender"]["udp_port"]);
    socket.open(udp::v4());

    // ROS subscribers
    this->inputMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        ros_config["topics"]["input_parcel"], 1000, std::bind(&UdpSender::udpSender_callback, this, std::placeholders::_1));
}

void UdpSender::udpSender_callback(const std_msgs::msg::UInt8MultiArray &msg) {
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < GuiMessage::length; i++) {
        received_vector.push_back(msg.data[i]);
    }
    bool ok = guiMessage.parseVector(received_vector);
    if (ok) {
        boost::system::error_code err;
        auto sent = socket.send_to(boost::asio::buffer(received_vector), remote_endpoint, 0, err);
    } else
        RCLCPP_WARN(this->get_logger(), "Wrong checksum");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<UdpSender>();
    rclcpp::spin(node);
    std::dynamic_pointer_cast<UdpSender>(node).get()->socket.close();
    rclcpp::shutdown();
    return 0;
}