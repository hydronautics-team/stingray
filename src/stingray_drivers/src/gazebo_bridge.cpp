#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>

#include <sstream>
#include <string>
#include <vector>

#include <stingray_msgs/SetTwist.h>
#include <stingray_msgs/SetFloat64.h>
#include <stingray_msgs/SetInt32.h>
#include <stingray_msgs/SetStabilization.h>
#include <stingray_msgs/SetDeviceAction.h>
#include <stingray_msgs/SetLagAndMarch.h>

#include "messages/messages.h"
#include "TopicsAndServices.h"

static const std::string GAZEBO_BRIDGE_NODE_NAME = "gazebo_bridge";

static const uint32_t COMMUNICATION_DELAY_MILLISECONDS = 100;

nav_msgs::Odometry currentOdometry;

std_msgs::UInt32 depthMessage;
std_msgs::Int32 yawMessage;

bool isReady = false;

bool depthStabilizationEnabled = false;
bool yawStabilizationEnabled = false;

bool lagAndMarchCallback(stingray_msgs::SetLagAndMarch::Request &request,
                         stingray_msgs::SetLagAndMarch::Response &response) {
  currentOdometry.twist.twist.linear.x = request.march;
  currentOdometry.twist.twist.linear.y = request.lag;

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

  // TODO: Test it
  currentOdometry.pose.pose.position.z = request.value;

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

  // TODO: Test it
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

bool stabilizationCallback(stingray_msgs::SetStabilization::Request &request,
                           stingray_msgs::SetStabilization::Response &response) {
  depthStabilizationEnabled = request.depthStabilization;
  yawStabilizationEnabled = request.yawStabilization;

  isReady = true;
  response.success = true;
  return true;
}

bool deviceActionCallback(stingray_msgs::SetDeviceAction::Request &request,
                          stingray_msgs::SetDeviceAction::Response &response) {
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

  // TODO: Obtain depth and yaw from simulator
  ros::Publisher depthPublisher = nodeHandle.advertise<std_msgs::UInt32>(DEPTH_PUBLISH_TOPIC, 20);
  ros::Publisher yawPublisher = nodeHandle.advertise<std_msgs::Int32>(YAW_PUBLISH_TOPIC, 20);

  ros::ServiceServer velocityService = nodeHandle.advertiseService(SET_LAG_AND_MARCH_SERVICE, lagAndMarchCallback);
  ros::ServiceServer depthService = nodeHandle.advertiseService(SET_DEPTH_SERVICE, depthCallback);
  ros::ServiceServer yawService = nodeHandle.advertiseService(SET_YAW_SERVICE, yawCallback);
  ros::ServiceServer imuService = nodeHandle.advertiseService(SET_IMU_ENABLED_SERVICE, imuCallback);
  ros::ServiceServer stabilizationService = nodeHandle.advertiseService(SET_STABILIZATION_SERVICE, stabilizationCallback);
  ros::ServiceServer deviceService = nodeHandle.advertiseService(SET_DEVICE_SERVICE, deviceActionCallback);

  while (ros::ok()) {
    if (isReady) {
      gazeboOdometryPublisher.publish(currentOdometry);
      //depthPublisher.publish(depthMessage);
      //yawPublisher.publish(yawMessage);
    }

    ros::spinOnce();
    communicationDelay.sleep();
  }

  return 0;
}