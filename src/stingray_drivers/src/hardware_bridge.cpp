#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>

#include <sstream>
#include <string>
#include <vector>

#include <stingray_msgs/SetTwist.h>
#include <stingray_msgs/SetFloat64.h>
#include <stingray_msgs/SetInt32.h>
#include <stingray_msgs/SetStabilization.h>
#include <stingray_msgs/SetDeviceAction.h>

#include "messages/messages.h"
#include "TopicsAndServices.h"

#define SHORE_STABILIZE_DEPTH_BIT 0
#define SHORE_STABILIZE_ROLL_BIT 1
#define SHORE_STABILIZE_PITCH_BIT 2
#define SHORE_STABILIZE_YAW_BIT 3
#define SHORE_STABILIZE_IMU_BIT 4
#define SHORE_STABILIZE_SAVE_BIT 5

static const std::string HARDWARE_BRIDGE_NODE_NAME = "hardware_bridge";

static const uint32_t  COMMUNICATION_DELAY_MILLISECONDS = 100;

// Hardware bridge -> Protocol_bridge
std_msgs::UInt8MultiArray msg_out;

// Protocol_bridge -> Hardware bridge
std_msgs::UInt8MultiArray msg_in;

RequestMessage requestMessage;
ResponseMessage responseMessage;

std_msgs::UInt32 depthMessage;

bool isReady = false;

bool depthStabilizationEnabled = false;
bool yawStabilizationEnabled = false;

bool pickBit(uint8_t &input, uint8_t bit) {
  return static_cast<bool>((input << (8 - bit)) >> 8);
}

void setBit(uint8_t &byte, uint8_t bit, bool state) {
  uint8_t value = 1;
  if (state) {
    byte = byte | (value << bit);
  } else {
    byte = byte & ~(value << bit);
  }
}

void makeOutputMessage() {
  std::vector<uint8_t> output_vector = requestMessage.formVector();

  msg_out.data.clear();
  for (int i = 0; i < RequestMessage::length; i++) {
    msg_out.data.push_back(output_vector[i]);
  }
}

void inputMessageCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg) {
  std::vector<uint8_t> received_vector;
  for (int i = 0; i < ResponseMessage::length; i++) {
    received_vector.push_back(msg->data[i]);
  }
  bool ok = responseMessage.parseVector(received_vector);
  if (ok) {
    depthMessage.data = std::abs(static_cast<int>(responseMessage.depth * 100.0f)); // Convert metres to centimetres
  } else
    ROS_ERROR("Wrong checksum");
}

bool movementCallback(stingray_msgs::SetTwist::Request &request, stingray_msgs::SetTwist::Response &response) {
  requestMessage.roll = static_cast<int16_t>(request.twist.angular.x);
  requestMessage.pitch = static_cast<int16_t>(request.twist.angular.z);
  requestMessage.march = static_cast<int16_t>(request.twist.linear.x);
  requestMessage.lag = static_cast<int16_t>(request.twist.linear.z);

  if (!depthStabilizationEnabled)
    requestMessage.depth = static_cast<int16_t>(request.twist.linear.y);
  if (!yawStabilizationEnabled)
    requestMessage.yaw = static_cast<int16_t>(request.twist.angular.y);

  isReady = true;

  response.success = true;

  return true;
}

bool depthCallback(stingray_msgs::SetInt32::Request &request, stingray_msgs::SetInt32::Response &response) {
  if (!depthStabilizationEnabled) {
    response.success = false;
    response.message = "Depth stabilization is not enabled";
    return true;
  }

  ROS_INFO("Setting depth to %d", request.value);
  requestMessage.depth = -(static_cast<int16_t>(request.value * 10)); // For low-level stabilization purposes
  ROS_INFO("Sending to STM32 depth value: %d", requestMessage.depth);

  isReady = true;

  response.success = true;

  return true;
}

bool yawCallback(stingray_msgs::SetInt32::Request &request, stingray_msgs::SetInt32::Response &response) {
  if (!yawStabilizationEnabled) {
    response.success = false;
    response.message = "Yaw stabilization is not enabled";
    return true;
  }

  ROS_INFO("Setting yaw to %d", request.value);
  requestMessage.yaw = request.value;
  ROS_INFO("Sending to STM32 yaw value: %d", requestMessage.depth);

  isReady = true;

  response.success = true;

  return true;
}

bool imuCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  ROS_INFO("Setting SHORE_STABILIZE_IMU_BIT to %d", request.data);
  setBit(requestMessage.stabilize_flags, SHORE_STABILIZE_IMU_BIT, request.data);

  isReady = true;

  response.success = true;

  return true;
}

bool stabilizationCallback(stingray_msgs::SetStabilization::Request &request,
                           stingray_msgs::SetStabilization::Response &response) {
  ROS_INFO("Setting depth stabilization bits to %d", request.depthStabilization);
  setBit(requestMessage.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT, request.depthStabilization);
  ROS_INFO("Setting yaw stabilization bits to %d", request.yawStabilization);
  setBit(requestMessage.stabilize_flags, SHORE_STABILIZE_YAW_BIT, request.yawStabilization);

  depthStabilizationEnabled = request.depthStabilization;
  yawStabilizationEnabled = request.yawStabilization;

  isReady = true;

  response.success = true;

  return true;
}

bool deviceActionCallback(stingray_msgs::SetDeviceAction::Request &request,
                          stingray_msgs::SetDeviceAction::Response &response) {
  ROS_INFO("Setting device [%d] action value to %d", request.device, request.value);
  requestMessage.dev[request.device]  = request.value;

  isReady = true;

  response.success = true;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, HARDWARE_BRIDGE_NODE_NAME);
  ros::NodeHandle nodeHandle;
  ros::Rate communicationDelay(1000.0 / COMMUNICATION_DELAY_MILLISECONDS);

  msg_in.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_in.layout.dim[0].size = ResponseMessage::length;
  msg_in.layout.dim[0].stride = ResponseMessage::length;
  msg_in.layout.dim[0].label = "msg_in";

  msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_out.layout.dim[0].size = RequestMessage::length;
  msg_out.layout.dim[0].stride = RequestMessage::length;
  msg_out.layout.dim[0].label = "msg_out";

  requestMessage.roll = 0;
  requestMessage.yaw	= 0;
  requestMessage.pitch = 0;
  requestMessage.depth = 0;
  requestMessage.march = 0;
  requestMessage.lag	= 0;
  setBit(requestMessage.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT, false);
  setBit(requestMessage.stabilize_flags, SHORE_STABILIZE_YAW_BIT, false);
  setBit(requestMessage.stabilize_flags, SHORE_STABILIZE_IMU_BIT, false);

  ros::Publisher outputMessagePublisher = nodeHandle.advertise<std_msgs::UInt8MultiArray>(OUTPUT_PARCEL_TOPIC, 100);
  ros::Publisher depthPublisher = nodeHandle.advertise<std_msgs::UInt32>(DEPTH_PUBLISH_TOPIC, 20);

  ros::Subscriber inputMessageSubscriber = nodeHandle.subscribe(INPUT_PARCEL_TOPIC, 100, inputMessageCallback);

  ros::ServiceServer velocityService = nodeHandle.advertiseService(SET_VELOCITY_SERVICE, movementCallback);
  ros::ServiceServer depthService = nodeHandle.advertiseService(SET_DEPTH_SERVICE, depthCallback);
  ros::ServiceServer yawService = nodeHandle.advertiseService(SET_YAW_SERVICE, yawCallback);
  ros::ServiceServer imuService = nodeHandle.advertiseService(SET_IMU_ENABLED_SERVICE, imuCallback);
  ros::ServiceServer stabilizationService = nodeHandle.advertiseService(SET_STABILIZATION_SERVICE, stabilizationCallback);
  ros::ServiceServer deviceService = nodeHandle.advertiseService(SET_DEVICE_SERVICE, deviceActionCallback);

  while (ros::ok()) {
    if (isReady) {
      makeOutputMessage();
      outputMessagePublisher.publish(msg_out);
      depthPublisher.publish(depthMessage);
    }

    ros::spinOnce();
    communicationDelay.sleep();
  }

  return 0;
}