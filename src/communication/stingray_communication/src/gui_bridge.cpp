#include "gui_bridge.h"

GuiBridge::GuiBridge() : Node("GuiBridge"), _io_service(), _send_socket(_io_service), _receive_socket(_io_service) {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    com_config = json::parse(std::ifstream("resources/configs/communication.json"));

    // UDP sender
    _send_endpoint =
        udp::endpoint(address::from_string(com_config["bridges"]["gui"]["send_to_ip"]), com_config["bridges"]["gui"]["send_to_port"].get<int>());
    _send_socket.open(udp::v4());
    RCLCPP_INFO(this->get_logger(), "GuiBridge: socket opened. Address: %s, port: %d",
                com_config["bridges"]["gui"]["send_to_ip"].get<std::string>().c_str(), com_config["bridges"]["gui"]["send_to_port"].get<int>());

    // ROS multithreading
    receiver_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto receiver_pub_options = rclcpp::PublisherOptions();
    receiver_pub_options.callback_group = receiver_group_;

    sender_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sender_sub_options = rclcpp::SubscriptionOptions();
    sender_sub_options.callback_group = sender_group_;

    // ROS subscribers
    this->fromDriverMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        ros_config["topics"]["from_driver_parcel"], 1000, std::bind(&GuiBridge::from_driver_callback, this, std::placeholders::_1),
        sender_sub_options);
    // ROS publishers
    this->toDriverMessagePublisher =
        this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["to_driver_parcel"], 1000, receiver_pub_options);

    // UDP receiver
    _receive_socket.open(udp::v4());
    _receive_socket.bind(udp::endpoint(address::from_string(com_config["bridges"]["gui"]["receive_from_ip"]),
                                       com_config["bridges"]["gui"]["receive_from_port"].get<int>()));
    RCLCPP_INFO(this->get_logger(), "GuiBridge: socket binded to address: %s, port: %d",
                com_config["bridges"]["gui"]["receive_from_ip"].get<std::string>().c_str(),
                com_config["bridges"]["gui"]["receive_from_port"].get<int>());
    this->publishingTimer = this->create_wall_timer(50ms, std::bind(&GuiBridge::timerCallback, this));
}

GuiBridge::~GuiBridge() {
    _send_socket.close();
    _receive_socket.close();
}

void GuiBridge::from_driver_callback(const std_msgs::msg::UInt8MultiArray &msg) {
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
        RCLCPP_INFO(this->get_logger(), "Sent to gui");
        RCLCPP_INFO(this->get_logger(), "Error: %s", err.message().c_str());
    } else
        RCLCPP_WARN(this->get_logger(), "Receive from driver: wrong checksum");
}

void GuiBridge::from_gui_callback(const boost::system::error_code &error, size_t bytes_transferred) {
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
}

void GuiBridge::receive_loop() {
    RCLCPP_INFO(this->get_logger(), "Send loop started");
    // RCLCPP_INFO(this->get_logger(), "Ros ok %d", rclcpp::ok());

    while (rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        _receive_socket.async_receive_from(
            boost::asio::buffer(from_gui_buffer), _receive_endpoint,
            boost::bind(&GuiBridge::from_gui_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void GuiBridge::timerCallback() {
    boost::system::error_code err;
    _send_socket.send_to(boost::asio::buffer(toGuiMessage.formVector()), _send_endpoint, 0, err);
    RCLCPP_INFO(this->get_logger(), "Sent to gui");
    RCLCPP_INFO(this->get_logger(), "Error: %s", err.message().c_str());
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GuiBridge>();
    std::thread s([&] { node->receive_loop(); });
    // executor.add_node(node);
    // executor.spin();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}