#include "udp_receiver.h"

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <fstream>

UdpReceiver::UdpReceiver() : Node("UdpReceiver") {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    udp_config = json::parse(std::ifstream("resources/configs/udp.json"));

    // ROS publishers
    this->outputMessagePublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["output_parcel"], 1000);

    // UPD receiver
    boost::asio::io_service io_service;
    udp::socket socket{io_service};
    boost::array<char, 1024> recv_buffer;
    udp::endpoint remote_endpoint;
   }

void UdpReceiver::timerCallback() {
    RCLCPP_INFO(this->get_logger(), "Timer callback");
    if (isReady) {
        // Make output message
        std::vector<uint8_t> output_vector = guiMessage.formVector();
        outputMessage.data.clear();
        for (int i = 0; i < GuiMessage::lengthRequest; i++) {
            outputMessage.data.push_back(output_vector[i]);
        }
        // Publish messages
        outputMessagePublisher->publish(outputMessage);
        RCLCPP_INFO(this->get_logger(), "Udp receiver publishing ...");
    } else
        RCLCPP_INFO(this->get_logger(), "Wait for topic updating");
}

void UdpReceiver::wait() {
    socket.async_receive_from(boost::asio::buffer(recv_buffer), \
        remote_endpoint, \
        boost::bind(&UdpReceiver::timerCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UdpReceiver::Receiver()
{
    socket.open(udp::v4());
    socket.bind(udp::endpoint(address::from_string(udp_config["udp_receiver"]["ip_address"]), udp_config["udp_receiver"]["udp_port"]));

    wait();

    RCLCPP_INFO(this->get_logger(), "Receiving");
    io_service.run();
    RCLCPP_INFO(this->get_logger(), "Receiver exit");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<UdpReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}