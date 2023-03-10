#include "udp_receiver.h"

UdpReceiver::UdpReceiver() : Node("UdpReceiver"), io_service(), socket(io_service)
{
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    udp_config = json::parse(std::ifstream("resources/configs/udp.json"));

    socket.open(udp::v4());
    socket.bind(udp::endpoint(address::from_string(udp_config["udp_receiver"]["ip_address"]), udp_config["udp_receiver"]["udp_port"]));

    // ROS publishers
    this->outputMessagePublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["output_parcel"], 1000);
}

void UdpReceiver::udpReceive_callback(const boost::system::error_code &error, size_t bytes_transferred)
{
    // Make output message
    if (error)
    {
        RCLCPP_ERROR(this->get_logger(), "Receive failed: %s", error.message().c_str());
        return;
    }

    std::vector<uint8_t> gui_vector;
    for (int i = 0; i < GuiRequestMessage::length; i++)
    {
        gui_vector.push_back(recv_buffer[i]);
    }
    bool ok = guiRequestMessage.parseVector(gui_vector);

    requestMessage.flags = guiRequestMessage.flags;
    requestMessage.march = guiRequestMessage.march;
    requestMessage.lag = guiRequestMessage.lag;
    requestMessage.depth = guiRequestMessage.depth;
    requestMessage.roll = guiRequestMessage.roll;
    requestMessage.pitch = guiRequestMessage.pitch;
    requestMessage.yaw = guiRequestMessage.yaw;
    for (int i = 0; i < DevAmount; i++)
    {
        requestMessage.dev[i] = guiRequestMessage.dev[i];
    }

    std::vector<uint8_t> output_vector = requestMessage.formVector();
    outputMessage.data.clear();
    for (int i = 0; i < RequestMessage::length; i++)
    {
        outputMessage.data.push_back(output_vector[i]);
    }
    // Publish messages
    outputMessagePublisher->publish(outputMessage);
    RCLCPP_INFO(this->get_logger(), "Udp receiver publishing ...");
}

void UdpReceiver::wait()
{
    socket.async_receive_from(
        boost::asio::buffer(recv_buffer), remote_endpoint,
        boost::bind(&UdpReceiver::udpReceive_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<UdpReceiver> node = std::make_shared<UdpReceiver>();
    while (rclcpp::ok())
    {
        node.get()->wait();
        rclcpp::spin_some(node);
    }
    node.get()->socket.close();
    rclcpp::shutdown();
    return 0;
}