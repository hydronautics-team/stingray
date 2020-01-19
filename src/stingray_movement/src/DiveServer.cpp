#include <DiveServer.h>

#include <cmath>

#include <stingray_msgs/SetInt32.h>
#include <std_msgs/UInt32.h>

DiveServer::DiveServer(const std::string &actionName) :
    AbstractMovementActionServer<stingray_movement_msgs::DiveAction,
                                 stingray_movement_msgs::DiveGoalConstPtr>(actionName, 0.0) {};

void DiveServer::goalCallback(const stingray_movement_msgs::DiveGoalConstPtr &goal) {
  stingray_msgs::SetInt32 serviceCall;
  serviceCall.request.value = goal->depth;
  auto result = ros::service::call(DEPTH_SERVICE, serviceCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set depth: %s", serviceCall.response.message.c_str());
    actionServer.setAborted(stingray_movement_msgs::DiveResult(),
                            "Unable to set depth: %s" + serviceCall.response.message);
    return;
  }

  int desiredDepth = goal->depth;
  while (true) {
    std_msgs::UInt32 depthMessage = *ros::topic::waitForMessage<std_msgs::UInt32>(DEPTH_TOPIC, nodeHandle);
    int currentDepth = depthMessage.data;
    auto delta = std::abs(desiredDepth - currentDepth);
    ROS_INFO("Current depth: %d, desired depth: %d", currentDepth, desiredDepth);
    if (delta <= DEPTH_ERROR_RANGE) {
      break;
    }
  }

  actionServer.setSucceeded();
}

