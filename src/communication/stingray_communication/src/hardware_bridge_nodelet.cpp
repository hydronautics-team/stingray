/**
 * This node:
 * - receives movement data from pilot and transforms it into byte array and publishes it
 * - receives byte array from protocol_bridge, parses it and publishes it
 */

#include <pluginlib/class_list_macros.h>
#include "hardware_bridge_nodelet.h"
#include <fstream>

void hardware_bridge::onInit()
{
    ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
    // Initializing nodelet and parameters
    NODELET_INFO("Hardware bridge: Initializing nodelet: hardware_bridge");
    ros::NodeHandle &nodeHandle = getNodeHandle();
    // ROS publishers
    outputMessagePublisher = nodeHandle.advertise<std_msgs::UInt8MultiArray>(ros_config["topics"]["output_parcel"], 1000);
    hardwareInfoPublisher = nodeHandle.advertise<stingray_communication_msgs::HardwareInfo>(ros_config["topics"]["robot_info"], 1000);
    depthPublisher = nodeHandle.advertise<std_msgs::Int32>(ros_config["topics"]["depth"], 1000);
    yawPublisher = nodeHandle.advertise<std_msgs::Int32>(ros_config["topics"]["yaw"], 20);
    // ROS subscribers
    inputMessageSubscriber = nodeHandle.subscribe(ros_config["topics"]["input_parcel"], 1000,
                                                  &hardware_bridge::inputMessage_callback, this);
    // ROS services
    horizontalMoveService = nodeHandle.advertiseService(ros_config["services"]["set_horizontal_move"],
                                                        &hardware_bridge::horizontalMoveCallback, this);
    depthService = nodeHandle.advertiseService(ros_config["services"]["set_depth"], &hardware_bridge::depthCallback, this);
    imuService = nodeHandle.advertiseService(ros_config["services"]["set_imu_enabled"], &hardware_bridge::imuCallback, this);
    stabilizationService = nodeHandle.advertiseService(ros_config["services"]["set_stabilization_enabled"],
                                                       &hardware_bridge::stabilizationCallback, this);
    deviceActionService = nodeHandle.advertiseService(ros_config["services"]["updown"],
                                                      &hardware_bridge::deviceActionCallback, this);
    // Output message container
    outputMessage.layout.dim.push_back(std_msgs::MultiArrayDimension());
    outputMessage.layout.dim[0].size = RequestMessage::length;
    outputMessage.layout.dim[0].stride = RequestMessage::length;
    outputMessage.layout.dim[0].label = "outputMessage";
    // Initializing timer for publishing messages. Callback interval: 0.05 ms
    publishingTimer = nodeHandle.createTimer(ros::Duration(0.1),
                                             &hardware_bridge::timerCallback, this);
}

void hardware_bridge::inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg)
{
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < ResponseMessage::length; i++)
    {
        received_vector.push_back(msg->data[i]);
    }
    bool ok = responseMessage.parseVector(received_vector);
    if (ok)
    {
        depthMessage.data = static_cast<int>(responseMessage.depth); // Convert metres to centimetres
        // NODELET_INFO("Hardware bridge: Received depth: %f", responseMessage.depth);
        // TODO: Test yaw obtaining
        yawMessage.data = static_cast<int>(responseMessage.yaw);
        // NODELET_INFO("Hardware bridge: Received yaw: %f", responseMessage.yaw);

        hardwareInfoMessage.roll = responseMessage.roll;
        hardwareInfoMessage.pitch = responseMessage.pitch;
        hardwareInfoMessage.yaw = responseMessage.yaw;
        hardwareInfoMessage.rollSpeed = responseMessage.rollSpeed;
        hardwareInfoMessage.pitchSpeed = responseMessage.pitchSpeed;
        hardwareInfoMessage.yawSpeed = responseMessage.yawSpeed;
        hardwareInfoMessage.depth = responseMessage.depth;
    }
    else
        NODELET_ERROR("Wrong checksum");
}

bool hardware_bridge::horizontalMoveCallback(stingray_communication_msgs::SetHorizontalMove::Request &horizontalMoveRequest,
                                             stingray_communication_msgs::SetHorizontalMove::Response &horizontalMoveResponse)
{

    if (lagStabilizationEnabled)
    {
        requestMessage.march = static_cast<int16_t>(0.0);
        requestMessage.lag = static_cast<int16_t>(0.0);
        requestMessage.lag_error = static_cast<int16_t>(horizontalMoveRequest.lag);
    }
    else
    {
        NODELET_INFO("Hardware bridge: Setting march to %f", horizontalMoveRequest.march);
        requestMessage.march = static_cast<int16_t>(horizontalMoveRequest.march);
        NODELET_INFO("Hardware bridge: Setting lag to %f", horizontalMoveRequest.lag);
        requestMessage.lag = static_cast<int16_t>(horizontalMoveRequest.lag);
    }

    if (!yawStabilizationEnabled)
    {
        horizontalMoveResponse.success = false;
        horizontalMoveResponse.message = "Yaw stabilization is not enabled";
        return true;
    }
    currentYaw += horizontalMoveRequest.yaw;
    NODELET_INFO("Hardware bridge: Setting yaw to %d", currentYaw);
    requestMessage.yaw = static_cast<int16_t>(currentYaw);

    isReady = true;
    horizontalMoveResponse.success = true;
    return true;
}

bool hardware_bridge::depthCallback(stingray_communication_msgs::SetInt32::Request &depthRequest,
                                    stingray_communication_msgs::SetInt32::Response &depthResponse)
{
    if (!depthStabilizationEnabled)
    {
        depthResponse.success = false;
        depthResponse.message = "Depth stabilization is not enabled";
        NODELET_INFO("Hardware bridge: Setting depth to %d", depthRequest.value);
        return true;
    }
    NODELET_INFO("Hardware bridge: Setting depth to %d", depthRequest.value);
    requestMessage.depth = (static_cast<int16_t>(depthRequest.value)); // For low-level stabilization purposes
    NODELET_DEBUG("Sending to STM32 depth value: %d", requestMessage.depth);

    isReady = true;
    depthResponse.success = true;
    return true;
}

bool hardware_bridge::imuCallback(std_srvs::SetBool::Request &imuRequest,
                                  std_srvs::SetBool::Response &imuResponse)
{
    NODELET_INFO("Hardware bridge: Setting SHORE_STABILIZE_IMU_BIT to %d", imuRequest.data);
    setStabilizationState(requestMessage, SHORE_STABILIZE_IMU_BIT, imuRequest.data);

    isReady = true;
    imuResponse.success = true;

    return true;
}

bool hardware_bridge::stabilizationCallback(stingray_communication_msgs::SetStabilization::Request &stabilizationRequest,
                                            stingray_communication_msgs::SetStabilization::Response &stabilizationResponse)
{
    // set current yaw
    currentYaw = responseMessage.yaw;
    requestMessage.yaw = currentYaw;
    NODELET_INFO("Hardware bridge: Setting initial yaw: %d", currentYaw);
    // set current depth
    currentDepth = responseMessage.depth;
    requestMessage.depth = currentDepth;
    NODELET_INFO("Hardware bridge: Setting initial depth: %d", currentDepth);

    NODELET_INFO("Hardware bridge: Setting depth stabilization %d", stabilizationRequest.depthStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_DEPTH_BIT, stabilizationRequest.depthStabilization);
    NODELET_INFO("Hardware bridge: Setting pitch stabilization %d", stabilizationRequest.pitchStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_PITCH_BIT, stabilizationRequest.pitchStabilization);
    NODELET_INFO("Hardware bridge: Setting yaw stabilization %d", stabilizationRequest.yawStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_YAW_BIT, stabilizationRequest.yawStabilization);
    NODELET_INFO("Hardware bridge: Setting lag stabilization %d", stabilizationRequest.lagStabilization);
    setStabilizationState(requestMessage, SHORE_STABILIZE_LAG_BIT, stabilizationRequest.lagStabilization);
    depthStabilizationEnabled = stabilizationRequest.depthStabilization;
    pitchStabilizationEnabled = stabilizationRequest.pitchStabilization;
    yawStabilizationEnabled = stabilizationRequest.yawStabilization;
    lagStabilizationEnabled = stabilizationRequest.lagStabilization;

    isReady = true;
    stabilizationResponse.success = true;
    return true;
}

bool hardware_bridge::deviceActionCallback(stingray_communication_msgs::SetDeviceAction::Request &deviceRequest,
                                           stingray_communication_msgs::SetDeviceAction::Response &deviceResponse)
{
    ROS_INFO("Setting device [%d] action value to %d", deviceRequest.device, deviceRequest.value);
    requestMessage.dev[deviceRequest.device] = deviceRequest.value;

    isReady = true;
    deviceResponse.success = true;
    return true;
}

/** @brief Timer callback. Make byte array to publish for protocol_node and publishes it
 *
 */
void hardware_bridge::timerCallback(const ros::TimerEvent &event)
{
    // NODELET_DEBUG("Timer callback");
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
        hardwareInfoPublisher.publish(hardwareInfoMessage);
        outputMessagePublisher.publish(outputMessage);
        depthPublisher.publish(depthMessage);
        yawPublisher.publish(yawMessage);
        // NODELET_DEBUG("HARDWARE BRIDGE PUBLISH");
    }
    else
        NODELET_DEBUG("Wait for topic updating");
}

PLUGINLIB_EXPORT_CLASS(hardware_bridge, nodelet::Nodelet);
