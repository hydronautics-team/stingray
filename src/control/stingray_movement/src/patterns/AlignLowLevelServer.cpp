#include <patterns/AlignLowLevelServer.h>

#include <stingray_drivers_msgs/SetLagAndMarch.h>
#include <stingray_drivers_msgs/SetStabilization.h>
#include <stingray_movement_msgs/AlignValue.h>


const std::string  AlignLowLevelServer::STABILIZATION_SERVICE = "/stingray/services/control/set_stabilization";

AlignLowLevelServer::AlignLowLevelServer(const std::string &actionName, double velocityCoefficient) :
    AbstractMovementActionServer<stingray_movement_msgs::AlignAction,
                                 stingray_movement_msgs::AlignGoalConstPtr>(actionName, velocityCoefficient) {};

void AlignLowLevelServer::goalCallback(const stingray_movement_msgs::AlignGoalConstPtr &goal) {
  stingray_drivers_msgs::SetLagAndMarch serviceCall;
  serviceCall.request.lag = serviceCall.request.march = 0.0;
  auto result = ros::service::call(LAG_MARCH_SERVICE, serviceCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
    actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                            "Unable to set march and lag: %s" + serviceCall.response.message);
    return;
  }

  stingray_drivers_msgs::SetStabilization stabilizationCall;
  stabilizationCall.request.depthStabilization = stabilizationCall.request.yawStabilization =
      stabilizationCall.request.lagStabilization = true;
  result = ros::service::call(STABILIZATION_SERVICE, stabilizationCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set stabilization!");
    actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                            "Unable to set stabilization!");
    return;
  }

  bool inRange = false;
  int direction = 0;
  while (!inRange) {
    auto message = ros::topic::waitForMessage<stingray_movement_msgs::AlignValue>(goal->topicName);

    if (message->hasValue) {
      if (std::abs(message->difference) <= goal->delta) {
        inRange = true;
      } else {
        direction = message->difference;
      }
      ROS_INFO("Difference: %f, direction: %d", message->difference, direction);
    } else {
      direction = -direction;
    }
    serviceCall.request.lag = direction;

    result = ros::service::call(LAG_MARCH_SERVICE, serviceCall);
    if (!result || !serviceCall.response.success) {
      ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
      actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                              "Unable to set march and lag: %s" + serviceCall.response.message);
      return;
    }

  }

  stabilizationCall.request.lagStabilization = false;
  result = ros::service::call(STABILIZATION_SERVICE, stabilizationCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set stabilization!");
    actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                            "Unable to set stabilization!");
    return;
  }

  serviceCall.request.lag = serviceCall.request.march = 0.0;
  result = ros::service::call(LAG_MARCH_SERVICE, serviceCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
    actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                            "Unable to set march and lag: %s" + serviceCall.response.message);
    return;
  }

  actionServer.setSucceeded();
}