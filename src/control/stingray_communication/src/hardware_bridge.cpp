/**
 * This node:
 * - receives movement data from pilot and transforms it into byte array and publishes it
 * - receives byte array from protocol_bridge, parses it and publishes it
 */

#include "../include/HardwareBridge.h"


void HardwareBridge::HardwareBridge() {

    // ROS publishers
    outputMessagePublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(OUTPUT_PARCEL_TOPIC, 1000);
    depthPublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(DEPTH_PUBLISH_TOPIC, 1000);
    yawPublisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(YAW_PUBLISH_TOPIC, 20);

    // ROS subscribers
    inputMessageSubscriber = this->create_subscription<std_msgs::msg::UInt8MultiArray>(INPUT_PARCEL_TOPIC, 1000,
                                                                                       std::bind(
                                                                                               &HardwareBridge::inputMessage_callback,
                                                                                               this, _1));

    // ROS services
    lagAndMarchService = this->create_service<stingray_communication_msgs::srv::SetLagAndMarch>(
            SET_LAG_AND_MARCH_SERVICE, &HardwareBridge::lagAndMarchCallback);
    depthService = this->create_service<stingray_communication_msgs::srv::SetInt32>(SET_DEPTH_SERVICE,
                                                                                    &HardwareBridge::depthCallback);
    yawService = this->create_service<stingray_communication_msgs::srv::SetInt32>(SET_YAW_SERVICE,
                                                                                  &HardwareBridge::yawCallback);
    imuService = this->create_service<stingray_communication_msgs::srv::SetBool>(SET_IMU_ENABLED_SERVICE,
                                                                                 &HardwareBridge::imuCallback);
    stabilizationService = this->create_service<stingray_communication_msgs::srv::SetDeviceAction>(
            SET_STABILIZATION_SERVICE,
            &HardwareBridge::stabilizationCallback);
    deviceActionService = this->create_service<stingray_communication_msgs::srv::SetStabilization>(SET_DEVICE_SERVICE,
                                                                                                   &HardwareBridge::deviceActionCallback);

    // Output message container
    outputMessage.layout.dim.push_back(std_msgs::MultiArrayDimension());
    outputMessage.layout.dim[0].size = RequestMessage::length;
    outputMessage.layout.dim[0].stride = RequestMessage::length;
    outputMessage.layout.dim[0].label = "outputMessage";

    // Initializing timer for publishing messages. Callback interval: 0.05 ms
    timer_ = this->create_wall_timer(50ms, std::bind(&HardwareBridge::timer_callback, this));
}


/*!
 * @brief Input message callback
 * @param msg
 */
void HardwareBridge::inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) const {

    std::vector <uint8_t> received_vector;
    for (int i = 0; i < ResponseMessage::length; i++) {
        received_vector.push_back(msg->data[i]);
    }
    bool ok = responseMessage.parseVector(received_vector);
    if (ok) {
        RCLCPP_INFO(this->get_logger(), "Received depth: %f", responseMessage.depth);
        depthMessage.data = std::abs(static_cast<int>(responseMessage.depth * 100.0f)); // Convert metres to centimetres
        // TODO: Test yaw obtaining
        yawMessage.data = static_cast<int>(responseMessage.yaw * 100.0f);
    } else
        RCLCPP_WARN(this->get_logger(), "Wrong checksum");
}


/*!
 * @brief Lag and march callback
 * @param request
 * @param response
 * @return
 */
bool HardwareBridge::lagAndMarchCallback(
        const std::shared_ptr <stingray_communication_msgs::srv::SetLagAndMarch::Request> request,
        std::shared_ptr <tutorial_interfaces::srv::SetLagAndMarch::Response> response) {

    if (lagStabilizationEnabled) {
        request.march = static_cast<int16_t> (0.0);
        request.lag = static_cast<int16_t> (0.0);
        request.lag_error = static_cast<int16_t> (lagAndMarchRequest.lag);
    } else {
        request.march = static_cast<int16_t> (lagAndMarchRequest.march);
        request.lag = static_cast<int16_t> (lagAndMarchRequest.lag);
    }

    isReady = true;
    response.success = true;
    return true;
}


/*!
 * @brief Depth callback
 * @param request
 * @param response
 * @return
 */
bool HardwareBridge::depthCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Request> request,
                                   std::shared_ptr <tutorial_interfaces::srv::SetInt32::Response> response) {

    if (!depthStabilizationEnabled) {
        response.success = false;
        response.message = "Depth stabilization is not enabled";
        return true;
    }
    RCLCPP_INFO(this->get_logger(), "Setting depth to %d", request.value);
    requestMessage.depth = -(static_cast<int16_t> (request.value * 100)); // For low-level stabilization purposes
    RCLCPP_INFO(this->get_logger(), "Sending to STM32 depth value: %d", requestMessage.depth);

    isReady = true;
    response.success = true;
    return true;
}


/*!
 * @brief Yaw callback
 * @param request
 * @param response
 * @return
 */
bool HardwareBridge::yawCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetInt32::Request> request,
                                 std::shared_ptr <tutorial_interfaces::srv::SetInt32::Response> response) {

    if (!yawStabilizationEnabled) {
        response.success = false;
        response.message = "Yaw stabilization is not enabled";
        return true;
    }
    RCLCPP_INFO(this->get_logger(), "Setting depth to %d", request.value);
    requestMessage.yaw = request.value;
    RCLCPP_INFO(this->get_logger(), "Sending to STM32 depth value: %d", requestMessage.yaw);

    isReady = true;
    response.success = true;
    return true;
}


/*!
 * @brief Imu callback
 * @param request
 * @param response
 * @return
 */
bool HardwareBridge::imuCallback(const std::shared_ptr <stingray_communication_msgs::srv::SetBool::Request> request,
                                 std::shared_ptr <tutorial_interfaces::srv::SetBool::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Setting SHORE_STABILIZE_IMU_BIT to %d", request.data);
    setStabilizationState(requestMessage, SHORE_STABILIZE_IMU_BIT, request.data);

    isReady = true;
    response.success = true;

    return true;
}


/*!
 * @brief Device callback
 * @param request
 * @param response
 * @return
 */
bool HardwareBridge::deviceActionCallback(
        const std::shared_ptr <stingray_communication_msgs::srv::SetDeviceAction::Request> request,
        std::shared_ptr <tutorial_interfaces::srv::SetDeviceAction::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Setting device [%d] action value to %d", request.device, request.value);
    requestMessage.dev[request.device] = request.value;

    isReady = true;
    response.success = true;
    return true;
}


/*!
 * @brief Stabilization callback
 * @param request
 * @param response
 * @return
 */
bool HardwareBridge::stabilizationCallback(
        const std::shared_ptr <stingray_communication_msgs::srv::SetStabilization::Request> request,
        std::shared_ptr <tutorial_interfaces::srv::SetStabilization::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Setting depth stabilization %d", request.depthStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_DEPTH_BIT, request.depthStabilization);
    RCLCPP_INFO(this->get_logger(), "Setting yaw stabilization %d", request.yawStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_YAW_BIT, request.yawStabilization);
    RCLCPP_INFO(this->get_logger(), "Setting lag stabilization %d", request.lagStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_LAG_BIT, request.lagStabilization);
    depthStabilizationEnabled = request.depthStabilization;
    yawStabilizationEnabled = request.yawStabilization;
    lagStabilizationEnabled = request.lagStabilization;

    isReady = true;
    response.success = true;
    return true;
}


/** @brief Timer callback. Make byte array to publish for protocol_node and publishes it
  */
void HardwareBridge::timerCallback() {

    RCLCPP_INFO(this->get_logger(), "Timer callback");
    if (isReady) {
        // Make output message
        std::vector <uint8_t> output_vector = requestMessage.formVector();
        outputMessage.data.clear();
        for (int i = 0; i < RequestMessage::length; i++) {
            outputMessage.data.push_back(output_vector[i]);
        }
        // Publish messages
        outputMessagePublisher.publish(outputMessage);
        depthPublisher.publish(depthMessage);
        yawPublisher.publish(yawMessage);
        RCLCPP_INFO(this->get_logger(), "Hardware bridge publishing ...");
    } else RCLCPP_INFO(this->get_logger(), "Wait for topic updating");

}

/*!
 * @brief Main loop
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareBridge>());
    rclcpp::shutdown();
}