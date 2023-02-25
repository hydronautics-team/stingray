#include "udp_bridge.h"

#include <fstream>

udpBridge::udpBridge() : Node("udpBridge") {
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));

    // ROS publishers
    this->outputMessagePublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["output_parcel"], 1000);
    this->udpInfoPublisher = this->create_publisher<stingray_communication_msgs::msg::udpInfo>(ros_config["topics"]["robot_info"], 1000);
    this->depthPublisher = this->create_publisher<std_msgs::msg::Float64>(ros_config["topics"]["depth"], 1000);
    this->yawPublisher = this->create_publisher<std_msgs::msg::Float64>(ros_config["topics"]["yaw"], 20);
    // ROS subscribers
    this->inputMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        ros_config["topics"]["input_parcel"], 1000, std::bind(&udpBridge::inputMessage_callback, this, std::placeholders::_1));
    // ROS services
    this->horizontalMoveService = this->create_service<stingray_communication_msgs::srv::SetHorizontalMove>(
        ros_config["services"]["set_horizontal_move"],
        std::bind(&udpBridge::horizontalMoveCallback, this, std::placeholders::_1, std::placeholders::_2));

    this->depthService = this->create_service<stingray_communication_msgs::srv::SetInt16>(
        ros_config["services"]["set_depth"], std::bind(&udpBridge::depthCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->imuService = this->create_service<std_srvs::srv::SetBool>(
        ros_config["services"]["set_imu_enabled"], std::bind(&udpBridge::imuCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->stabilizationService = this->create_service<stingray_communication_msgs::srv::SetStabilization>(
        ros_config["services"]["set_stabilization_enabled"],
        std::bind(&udpBridge::stabilizationCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->deviceActionService = this->create_service<stingray_communication_msgs::srv::SetDeviceAction>(
        ros_config["services"]["updown"], std::bind(&udpBridge::deviceActionCallback, this, std::placeholders::_1, std::placeholders::_2));
    // Output message container
    outputMessage.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    outputMessage.layout.dim[0].size = RequestMessage::length;
    outputMessage.layout.dim[0].stride = RequestMessage::length;
    outputMessage.layout.dim[0].label = "outputMessage";
    // Initializing timer for publishing messages. Callback interval: 0.05 ms
    this->publishingTimer = this->create_wall_timer(50ms, std::bind(&udpBridge::timerCallback, this));
}


void udpBridge::inputMessage_callback(const std_msgs::msg::UInt8MultiArray &msg) {
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < ResponseMessage::length; i++) {
        received_vector.push_back(msg.data[i]);
    }
    bool ok = responseMessage.parseVector(received_vector);
    if (ok) {
        depthMessage.data = responseMessage.depth;  // Convert metres to centimetres
        RCLCPP_INFO(this->get_logger(), "Received depth: %f", responseMessage.depth);
        // TODO: Test yaw obtaining
        yawMessage.data = responseMessage.yaw;
        RCLCPP_INFO(this->get_logger(), "Received yaw: %f", responseMessage.yaw);

        UDPInfoMessage.roll = responseMessage.roll;
        UDPInfoMessage.pitch = responseMessage.pitch;
        UDPInfoMessage.yaw = responseMessage.yaw;
        UDPInfoMessage.roll_speed = responseMessage.rollSpeed;
        UDPInfoMessage.pitch_speed = responseMessage.pitchSpeed;
        UDPInfoMessage.yaw_speed = responseMessage.yawSpeed;
        UDPInfoMessage.depth = responseMessage.depth;
    } else
        RCLCPP_WARN(this->get_logger(), "Wrong checksum");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<udpBridge>();
    // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr horizontalMoveService = node->create_service<std_srvs::srv::SetBool>("add_two_ints",
    // &horizontalMoveCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}