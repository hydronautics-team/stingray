#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>

#include <sstream>
#include <string>
#include <vector>

#include <stingray_common/SetTwist.h>
#include <stingray_common/SetFloat64.h>
#include <stingray_common/SetInt32.h>
#include <stingray_common/SetStabilization.h>
#include <stingray_common/SetDeviceAction.h>

#include "messages/messages.h"
#include "TopicsAndServices.h"

static const std::string GAZEBO_BRIDGE_NODE_NAME = "gazebo_bridge";

static const uint32_t COMMUNICATION_DELAY_MILLISECONDS = 100;

nav_msgs::Odometry currentOdometry;

std_msgs::UInt32 depthMessage;

bool isReady = false;

bool depthStabilizationEnabled = false;
bool yawStabilizationEnabled = false;

bool movementCallback(stingray_common::SetTwist::Request &request, stingray_common::SetTwist::Response &response) {
  currentOdometry.twist.twist.linear.x = request.twist.linear.x;
  currentOdometry.twist.twist.linear.y = request.twist.linear.y;
  currentOdometry.twist.twist.angular.x = request.twist.angular.x;
  currentOdometry.twist.twist.angular.y = request.twist.angular.y;

  if (!depthStabilizationEnabled)
    currentOdometry.twist.twist.linear.z = request.twist.linear.z;
  if (!yawStabilizationEnabled)
    currentOdometry.twist.twist.angular.z = request.twist.angular.z;

  isReady = true;

  response.success = true;

  return true;
}

bool depthCallback(stingray_common::SetInt32::Request &request, stingray_common::SetInt32::Response &response) {
  if (!depthStabilizationEnabled) {
    response.success = false;
    response.message = "Depth stabilization is not enabled";
    return true;
  }

  // TODO: Check it
  currentOdometry.pose.pose.position.z = request.value;

  isReady = true;

  response.success = true;

  return true;

}

bool yawCallback(stingray_common::SetInt32::Request &request, stingray_common::SetInt32::Response &response) {
  if (!yawStabilizationEnabled) {
    response.success = false;
    response.message = "Yaw stabilization is not enabled";
    return true;
  }

  // TODO: Check it
  currentOdometry.pose.pose.orientation.z = request.value;

  isReady = true;

  response.success = true;

  return true;

}

bool imuCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  isReady = true;
  response.success = true;
  return true;
}

bool stabilizationCallback(stingray_common::SetStabilization::Request &request,
                           stingray_common::SetStabilization::Response &response) {
  depthStabilizationEnabled = request.depthStabilization;
  yawStabilizationEnabled = request.yawStabilization;

  isReady = true;

  response.success = true;

  return true;
}

bool deviceActionCallback(stingray_common::SetDeviceAction::Request &request,
                          stingray_common::SetDeviceAction::Response &response) {
  isReady = true;
  response.success = true;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, GAZEBO_BRIDGE_NODE_NAME);
  ros::NodeHandle nodeHandle;
  ros::Rate communicationDelay(1000.0 / COMMUNICATION_DELAY_MILLISECONDS);

  currentOdometry.twist.twist.linear.x = 0;
  currentOdometry.twist.twist.linear.y = 0;
  currentOdometry.twist.twist.linear.z = 0;
  currentOdometry.twist.twist.angular.x = 0;
  currentOdometry.twist.twist.angular.y = 0;
  currentOdometry.twist.twist.angular.z = 0;

  ros::Publisher gazeboOdometryPublisher = nodeHandle.advertise<std_msgs::UInt8MultiArray>(GAZEBO_ODOMETRY_PUBLISH_TOPIC, 100);

  // TODO: Optain depth from simulator
  //ros::Publisher depthPublisher = nodeHandle.advertise<std_msgs::UInt32>(DEPTH_PUBLISH_TOPIC, 20);

  ros::ServiceServer velocityService = nodeHandle.advertiseService(SET_VELOCITY_SERVICE, movementCallback);
  ros::ServiceServer depthService = nodeHandle.advertiseService(SET_DEPTH_SERVICE, depthCallback);
  ros::ServiceServer yawService = nodeHandle.advertiseService(SET_YAW_SERVICE, yawCallback);
  ros::ServiceServer imuService = nodeHandle.advertiseService(SET_IMU_ENABLED_SERVICE, imuCallback);
  ros::ServiceServer stabilizationService = nodeHandle.advertiseService(SET_STABILIZATION_SERVICE, stabilizationCallback);
  ros::ServiceServer deviceService = nodeHandle.advertiseService(SET_DEVICE_SERVICE, deviceActionCallback);

  while (ros::ok()) {
    if (isReady) {
      gazeboOdometryPublisher.publish(currentOdometry);
      //depthPublisher.publish(depthMessage);
    }

    ros::spinOnce();
    communicationDelay.sleep();
  }

  return 0;
}