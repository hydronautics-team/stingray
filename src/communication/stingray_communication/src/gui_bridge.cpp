#include "gui_bridge.h"

GuiBridgeSender::GuiBridgeSender(boost::asio::io_service &io_service) : Node("GuiBridgeSender"), _io_service(io_service), _send_socket(io_service) {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    com_config = json::parse(std::ifstream("resources/configs/communication.json"));

    // UDP sender
    _send_endpoint =
        udp::endpoint(address::from_string(com_config["bridges"]["gui"]["send_to_ip"]), com_config["bridges"]["gui"]["send_to_port"].get<int>());
    _send_socket.open(udp::v4());
    RCLCPP_INFO(this->get_logger(), "GuiBridgeSender: socket opened. Address: %s, port: %d",
                com_config["bridges"]["gui"]["send_to_ip"].get<std::string>().c_str(), com_config["bridges"]["gui"]["send_to_port"].get<int>());

    // ROS subscribers
    this->fromDriverMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        ros_config["topics"]["from_driver_parcel"], 1000, std::bind(&GuiBridgeSender::from_driver_callback, this, std::placeholders::_1));

    this->publishingTimer = this->create_wall_timer(1s, std::bind(&GuiBridgeSender::timerCallback, this));
}

GuiBridgeSender::~GuiBridgeSender() { _send_socket.close(); }

void GuiBridgeSender::from_driver_callback(const std_msgs::msg::UInt8MultiArray &msg) {
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < FromDriverMessage::length; i++) {
        received_vector.push_back(msg.data[i]);
    }
    bool ok = fromDriverMessage.parseVector(received_vector);
    RCLCPP_INFO(this->get_logger(), "Received from driver");

    if (ok) {
        toGuiMessage.roll = fromDriverMessage.roll;
        toGuiMessage.pitch = fromDriverMessage.pitch;
        toGuiMessage.yaw = fromDriverMessage.yaw;
        toGuiMessage.depth = fromDriverMessage.depth;
        toGuiMessage.rollSpeed = fromDriverMessage.rollSpeed;
        toGuiMessage.pitchSpeed = fromDriverMessage.pitchSpeed;
        toGuiMessage.yawSpeed = fromDriverMessage.yawSpeed;

        boost::system::error_code err;
        _send_socket.send_to(boost::asio::buffer(toGuiMessage.formVector()), _send_endpoint, 0, err);
        RCLCPP_INFO(this->get_logger(), "Sent to gui %s", err.message().c_str());
    } else
        RCLCPP_WARN(this->get_logger(), "Receive from driver: wrong checksum");
}

void GuiBridgeSender::timerCallback() {
    boost::system::error_code err;
    _send_socket.send_to(boost::asio::buffer(toGuiMessage.formVector()), _send_endpoint, 0, err);
    RCLCPP_INFO(this->get_logger(), "Sent to gui %s", err.message().c_str());
}

GuiBridgeReceiver::GuiBridgeReceiver(boost::asio::io_service &io_service)
    : Node("GuiBridgeReceiver"), _io_service(io_service), _receive_socket(io_service) {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    com_config = json::parse(std::ifstream("resources/configs/communication.json"));

    // ROS publishers
    this->toDriverMessagePublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["to_driver_parcel"], 1000);

    // UDP receiver
    _receive_socket.open(udp::v4());
    _receive_socket.bind(udp::endpoint(address::from_string(com_config["bridges"]["gui"]["receive_from_ip"]),
                                       com_config["bridges"]["gui"]["receive_from_port"].get<int>()));
    RCLCPP_INFO(this->get_logger(), "GuiBridgeReceiver: socket binded to address: %s, port: %d",
                com_config["bridges"]["gui"]["receive_from_ip"].get<std::string>().c_str(),
                com_config["bridges"]["gui"]["receive_from_port"].get<int>());
}

GuiBridgeReceiver::~GuiBridgeReceiver() { _receive_socket.close(); }

void GuiBridgeReceiver::from_gui_callback(const boost::system::error_code &error, size_t bytes_transferred) {
    // Make output message
    if (error) {
        RCLCPP_ERROR(this->get_logger(), "Receive failed: %s", error.message().c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Received from gui %ld", bytes_transferred);

    std::vector<uint8_t> gui_vector;
    for (int i = 0; i < FromGuiMessage::length; i++) {
        gui_vector.push_back(from_gui_buffer[i]);
    }
    fromGuiMessage.parseVector(gui_vector);

    toDriverMessage.flags = fromGuiMessage.flags;
    toDriverMessage.march = fromGuiMessage.march;
    toDriverMessage.lag = fromGuiMessage.lag;
    toDriverMessage.depth = fromGuiMessage.depth;
    toDriverMessage.roll = fromGuiMessage.roll;
    toDriverMessage.pitch = fromGuiMessage.pitch;
    toDriverMessage.yaw = fromGuiMessage.yaw;
    for (int i = 0; i < DevAmount; i++) {
        toDriverMessage.dev[i] = fromGuiMessage.dev[i];
    }

    std::vector<uint8_t> output_vector = toDriverMessage.formVector();
    toDriverMessageContainer.data.clear();
    for (int i = 0; i < ToDriverMessage::length; i++) {
        toDriverMessageContainer.data.push_back(output_vector[i]);
    }
    // Publish messages
    toDriverMessagePublisher->publish(toDriverMessageContainer);
    RCLCPP_INFO(this->get_logger(), "Udp publishing to driver ...");
    try_receive();
}

void GuiBridgeReceiver::try_receive() {
    RCLCPP_INFO(this->get_logger(), "Trying to receive from gui...");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    _receive_socket.async_receive_from(
        boost::asio::buffer(from_gui_buffer), _receive_endpoint,
        boost::bind(&GuiBridgeReceiver::from_gui_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    boost::asio::io_service io_service;
    auto sender = std::make_shared<GuiBridgeSender>(io_service);
    auto receiver = std::make_shared<GuiBridgeReceiver>(io_service);
    std::thread s([&] {
        receiver->try_receive();
        io_service.run();
    });
    executor.add_node(sender);
    executor.add_node(receiver);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}