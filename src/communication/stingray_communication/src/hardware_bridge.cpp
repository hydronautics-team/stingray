/**
 * This node:
 * - receives movement data from pilot and transforms it into byte array and publishes it
 * - receives byte array from protocol_bridge, parses it and publishes it
 */

#include "hardware_bridge.h"
#include <fstream>

HardwareBridge::HardwareBridge() : Node("HardwareBridge")
{
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));

    // ROS publishers
    this->outputMessagePublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["output_parcel"], 1000);
    this->hardwareInfoPublisher = this->create_publisher<stingray_communication_msgs::msg::HardwareInfo>(ros_config["topics"]["robot_info"], 1000);
    this->depthPublisher = this->create_publisher<std_msgs::msg::UInt32>(ros_config["topics"]["depth"], 1000);
    this->yawPublisher = this->create_publisher<std_msgs::msg::Int32>(ros_config["topics"]["yaw"], 20);
    // ROS subscribers
    this->inputMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(ros_config["topics"]["input_parcel"], 1000,
                                                                                       std::bind(&HardwareBridge::inputMessage_callback, this, std::placeholders::_1));
    // ROS services
    this->horizontalMoveService = this->create_service<stingray_communication_msgs::srv::SetHorizontalMove>(ros_config["services"]["set_horizontal_move"],
                                                                                                      std::bind(
                                                                                                          &HardwareBridge::horizontalMoveCallback, this, std::placeholders::_1, std::placeholders::_2));

    this->depthService = this->create_service<stingray_communication_msgs::srv::SetInt32>(ros_config["services"]["set_depth"], std::bind(
                                                                                                                             &HardwareBridge::depthCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->imuService = this->create_service<std_srvs::srv::SetBool>(ros_config["services"]["set_imu_enabled"], std::bind(
                                                                                                             &HardwareBridge::imuCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->stabilizationService = this->create_service<stingray_communication_msgs::srv::SetStabilization>(ros_config["services"]["set_stabilization_enabled"],
                                                                                                    std::bind(
                                                                                                        &HardwareBridge::stabilizationCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->deviceActionService = this->create_service<stingray_communication_msgs::srv::SetDeviceAction>(ros_config["services"]["updown"],
                                                                                                  std::bind(
                                                                                                      &HardwareBridge::deviceActionCallback,
                                                                                                      this,
                                                                                                      std::placeholders::_1,
                                                                                                      std::placeholders::_2));
    // Output message container
    outputMessage.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    outputMessage.layout.dim[0].size = RequestMessage::length;
    outputMessage.layout.dim[0].stride = RequestMessage::length;
    outputMessage.layout.dim[0].label = "outputMessage";
    // Initializing timer for publishing messages. Callback interval: 0.05 ms
    this->publishingTimer = this->create_wall_timer(50ms, std::bind(&HardwareBridge::timerCallback, this));
}

void HardwareBridge::inputMessage_callback(const std_msgs::msg::UInt8MultiArray &msg)
{
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < ResponseMessage::length; i++)
    {
        received_vector.push_back(msg.data[i]);
    }
    bool ok = responseMessage.parseVector(received_vector);
    if (ok)
    {
        depthMessage.data = static_cast<int>(responseMessage.depth); // Convert metres to centimetres
        RCLCPP_INFO(this->get_logger(), "Received depth: %f", responseMessage.depth);
        // TODO: Test yaw obtaining
        yawMessage.data = static_cast<int>(responseMessage.yaw);
        RCLCPP_INFO(this->get_logger(), "Received yaw: %f", responseMessage.yaw);

        hardwareInfoMessage.roll = responseMessage.roll;
        hardwareInfoMessage.pitch = responseMessage.pitch;
        hardwareInfoMessage.yaw = responseMessage.yaw;
        hardwareInfoMessage.roll_speed = responseMessage.rollSpeed;
        hardwareInfoMessage.pitch_speed = responseMessage.pitchSpeed;
        hardwareInfoMessage.yaw_speed = responseMessage.yawSpeed;
        hardwareInfoMessage.depth = responseMessage.depth;
    }
    else
        RCLCPP_WARN(this->get_logger(), "Wrong checksum");
}

void HardwareBridge::horizontalMoveCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Request> request,
                                            std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Response> response)
{
    if (lagStabilizationEnabled)
    {
        requestMessage.march = static_cast<int16_t>(0.0);
        requestMessage.lag = static_cast<int16_t>(0.0);
        requestMessage.lag_error = static_cast<int16_t>(request->lag);
    }
    else
    {
        requestMessage.march = static_cast<int16_t>(request->march);
        requestMessage.lag = static_cast<int16_t>(request->lag);
    }

    if (!yawStabilizationEnabled)
    {
        response->success = false;
        response->message = "Yaw stabilization is not enabled";
        return;
    }
    currentYaw += request->yaw;
    requestMessage.yaw = static_cast<int16_t>(currentYaw);

    isReady = true;
    response->success = true;
}

void HardwareBridge::depthCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetInt32::Request> request,
                                   std::shared_ptr<stingray_communication_msgs::srv::SetInt32::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting depth to %d", request->value);
    if (!depthStabilizationEnabled)
    {
        response->success = false;
        response->message = "Depth stabilization is not enabled";
        return;
    }
    requestMessage.depth = (static_cast<int16_t>(request->value)); // For low-level stabilization purposes
    RCLCPP_INFO(this->get_logger(), "Sending to STM32 depth value: %d", requestMessage.depth);

    isReady = true;
    response->success = true;
}

void HardwareBridge::imuCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Hardware bridge: Setting SHORE_STABILIZE_IMU_BIT to %d", request->data);
    setStabilizationState(requestMessage, SHORE_STABILIZE_IMU_BIT, request->data);

    isReady = true;
    response->success = true;
}

void HardwareBridge::stabilizationCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Request> request,
                                           std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Response> response)
{
    // set current yaw
    currentYaw = responseMessage.yaw;
    requestMessage.yaw = currentYaw;
    RCLCPP_INFO(this->get_logger(), "Hardware bridge: Setting initial yaw: %d", currentYaw);
    // set current depth
    currentDepth = responseMessage.depth;
    requestMessage.depth = currentDepth;
    RCLCPP_INFO(this->get_logger(), "Hardware bridge: Setting initial depth: %d", currentDepth);

    RCLCPP_INFO(this->get_logger(), "Setting depth stabilization %d", request->depth_stabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_DEPTH_BIT, request->depth_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting pitch stabilization %d", request->pitch_stabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_PITCH_BIT, request->pitch_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting yaw stabilization %d", request->yaw_stabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_YAW_BIT, request->yaw_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting lag stabilization %d", request->lag_stabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_LAG_BIT, request->lag_stabilization);
    depthStabilizationEnabled = request->depth_stabilization;
    pitchStabilizationEnabled = request->pitch_stabilization;
    yawStabilizationEnabled = request->yaw_stabilization;
    lagStabilizationEnabled = request->lag_stabilization;

    isReady = true;
    response->success = true;
}

void HardwareBridge::deviceActionCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Request> request,
                                          std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Setting device [%d] action value to %d", request->device, request->value);
    requestMessage.dev[request->device] = request->value;

    isReady = true;
    response->success = true;
}

/** @brief Timer callback. Make byte array to publish for protocol_node and publishes it
 *
 */
void HardwareBridge::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback");
    if (isReady)
    {
        // Make output message
        std::vector<uint8_t> output_vector = requestMessage.formVector();
        outputMessage.data.clear();
        for (int i = 0; i < RequestMessage::length; i++)
        {
            outputMessage.data.push_back(output_vector[i]);
        }
        // Publish messages
        // hardwareInfoPublisher->publish(hardwareInfoMessage);
        // outputMessagePublisher->publish(outputMessage);
        // depthPublisher->publish(depthMessage);
        // yawPublisher->publish(yawMessage);
        RCLCPP_INFO(this->get_logger(), "Hardware bridge publishing ...");
    }
    else
        RCLCPP_INFO(this->get_logger(), "Wait for topic updating");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<HardwareBridge>();
    // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr horizontalMoveService = node->create_service<std_srvs::srv::SetBool>("add_two_ints", &horizontalMoveCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}