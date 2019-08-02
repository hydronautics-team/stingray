//
// Created by VLADUSHKED on 04.07.19.
//

/**
 * This node:
 * - receives movement data from pilot and transforms it into byte array and publishes it
 * - receives byte array from protocol_bridge, parses it and publishes it
 */

#include <pluginlib/class_list_macros.h>
#include "../include/hardware_bridge_nodelet.h"

#define SHORE_STABILIZE_DEPTH_BIT       0
#define SHORE_STABILIZE_ROLL_BIT        1
#define SHORE_STABILIZE_PITCH_BIT       2
#define SHORE_STABILIZE_YAW_BIT         3
#define SHORE_STABILIZE_IMU_BIT         4
#define SHORE_STABILIZE_SAVE_BIT		5

void hardware_bridge::onInit() {

    ros::NodeHandle& n = getNodeHandle();

    current_depth = 0;

    // ROS publishers
    outputMessage_pub = n.advertise<std_msgs::UInt8MultiArray>("/hard_bridge/parcel", 1000);
    depth_pub = n.advertise<std_msgs::UInt32>("/perception/depth", 1000);
    // **************

    // ROS subscribers
    inputMessage_sub = n.subscribe("/hard_bridge/uart", 1000, &hardware_bridge::inputMessage_callback, this);
    // **************

    // ROS services
    lagAndMarchSrv = n.advertiseService("lag_and_march_service", &hardware_bridge::lagAndMarchCallback, this);
    depthSrv = n.advertiseService("depth_service", &hardware_bridge::depthCallback, this);
    yawSrv = n.advertiseService("yaw_service", &hardware_bridge::yawCallback, this);
    imuSrv = n.advertiseService("imu_service", &hardware_bridge::imuCallback, this);
    deviceActionSrv = n.advertiseService("device_action_service", &hardware_bridge::deviceActionCallback, this);
    stabilizationSrv = n.advertiseService("stabilization_service", &hardware_bridge::stabilizationCallback, this);
    // **************

    // Output message container
    msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_out.layout.dim[0].size = RequestMessage::length;
    msg_out.layout.dim[0].stride = RequestMessage::length;
    msg_out.layout.dim[0].label = "msg_out";

    // Initializing hardware_bridge timer, 0.05 ms
    timer = n.createTimer(ros::Duration(0.05), &hardware_bridge::timerCallback, this);
}

void hardware_bridge::set_bit(uint8_t &byte, uint8_t bit, bool state)
{
    uint8_t value = 1;
    if(state) {
        byte = byte | (value << bit);
    }
    else {
        byte = byte & ~(value << bit);
    }
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void hardware_bridge::inputMessage_callback(const std_msgs::UInt8MultiArrayConstPtr msg)
{
    std::vector<uint8_t> received_vector;
    for(int i = 0; i < ResponseMessage::length; i++) {
        received_vector.push_back(msg->data[i]);
    }
    bool ok = response.parseVector(received_vector);
    if (ok) {
        NODELET_DEBUG("Received depth: %f", response.depth);
        current_depth = std::abs(static_cast<int>(response.depth * 100.0f)); // Convert metres to centimetres
    }
    else
        NODELET_ERROR("Wrong checksum");
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
bool hardware_bridge::lagAndMarchCallback(stingray_msgs::SetLagAndMarch::Request& lagAndMarchRequest,
                                          stingray_msgs::SetLagAndMarch::Response& lagAndMarchResponse) {
    //request.roll	= static_cast<int16_t> (lagAndMarchRequest.twist.angular.x);
    //request.yaw		= static_cast<int16_t> (lagAndMarchRequest.twist.angular.y);
    //request.pitch	= static_cast<int16_t> (lagAndMarchRequest.twist.angular.z);
    request.march	= static_cast<int16_t> (lagAndMarchRequest.march);
    request.lag	    = static_cast<int16_t> (lagAndMarchRequest.lag);

    lagAndMarchResponse.success = true;
    isTopicUpdated = true;

    return true;
}

bool hardware_bridge::depthCallback(stingray_msgs::SetFloat64::Request& depthRequest,
                                    stingray_msgs::SetFloat64::Response& depthResponse) {
    NODELET_DEBUG("Setting depth to %f", depthRequest.value);
    request.depth	= -(static_cast<int16_t> (depthRequest.value * 10)); // For low-level stabilization purposes
    NODELET_DEBUG("Sending to STM32 depth value: %d", request.depth);

    depthResponse.success = true;
    isTopicUpdated = true;

    return true;
}

bool hardware_bridge::yawCallback(stingray_msgs::SetFloat64::Request& yawRequest,
                                  stingray_msgs::SetFloat64::Response& yawResponse) {
    NODELET_DEBUG("Setting depth to %f", yawRequest.value);
    request.yaw		= static_cast<int16_t> (yawRequest.value);
    NODELET_DEBUG("Sending to STM32 depth value: %d", request.depth);

    yawResponse.success = true;
    isTopicUpdated = true;

    return true;
}

bool hardware_bridge::imuCallback(stingray_msgs::SetFloat64::Request& imuRequest,
                                  stingray_msgs::SetFloat64::Response& imuResponse) {
    // TODO: IMPLEMENT

    imuResponse.success = true;
    isTopicUpdated = true;

    return true;
}

bool hardware_bridge::deviceActionCallback(stingray_msgs::SetDeviceAction::Request& deviceRequest,
                                           stingray_msgs::SetDeviceAction::Response& deviceResponse) {
    // TODO: IMPLEMENT

    deviceResponse.success = true;
    isTopicUpdated = true;

    return true;
}

bool hardware_bridge::stabilizationCallback(stingray_msgs::SetStabilization::Request& stabilizationRequest,
                                            stingray_msgs::SetStabilization::Response& stabilizationResponse) {
    NODELET_DEBUG("Setting depth stabilization %d", stabilizationRequest.depthStabilization);
    NODELET_DEBUG("Setting yaw stabilization %d", stabilizationRequest.yawStabilization);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT, stabilizationRequest.depthStabilization);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_YAW_BIT, stabilizationRequest.yawStabilization);

    stabilizationResponse.success = true;
    isTopicUpdated = true;

    return true;
}

/** @brief Timer callback. Make byte array to publish for protocol_node and publishes it
  *
  */
void hardware_bridge::timerCallback(const ros::TimerEvent& event)
{
    NODELET_DEBUG("Timer callback");
    if (isTopicUpdated){
        std::vector<uint8_t> output_vector = request.formVector();

        msg_out.data.clear();
        for(int i=0; i<RequestMessage::length; i++) {
            msg_out.data.push_back(output_vector[i]);
        }
        outputMessage_pub.publish(msg_out);
        depth_message.data = current_depth;
        depth_pub.publish(depth_message);
        NODELET_DEBUG("HARDWARE BRIDGE PUBLISH");
    }
    else NODELET_DEBUG("Wait for topic updating");

}

PLUGINLIB_EXPORT_CLASS(hardware_bridge, nodelet::Nodelet);