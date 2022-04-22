#include <basic/RotateServer.h>

#include <cmath>

#include <stingray_communication_msgs/SetInt32.h>
#include <std_msgs/Int32.h>

RotateServer::RotateServer(const std::string &actionName) :
    AbstractMovementActionServer<stingray_movement_msgs::RotateAction,
                                 stingray_movement_msgs::RotateGoalConstPtr>(actionName, 0.0) {};

void RotateServer::goalCallback(const stingray_movement_msgs::RotateGoalConstPtr &goal) {

  stingray_communication_msgs::SetInt32 serviceCall;
  serviceCall.request.value = goal->yaw;
  auto result = ros::service::call(YAW_SERVICE, serviceCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set yaw: %s", serviceCall.response.message.c_str());
    actionServer.setAborted(stingray_movement_msgs::RotateResult(),
                            "Unable to set yaw: %s" + serviceCall.response.message);
    return;
  }

  int desiredYaw = goal->yaw;
  while (true) {
    auto yawMessage = *ros::topic::waitForMessage<std_msgs::Int32>(YAW_TOPIC, nodeHandle);
    auto currentYaw = yawMessage.data;

    ROS_INFO("Current yaw: %d, desired yaw: %d", currentYaw, desiredYaw);

    // Special case for the simulator
    if (currentYaw < 0 && desiredYaw > 0) {
      currentYaw = 360 + currentYaw;
    }
    // TODO: Handle case when desiredYaw > 360 for simulator

    auto delta = std::abs(desiredYaw - currentYaw);
    if (delta <= YAW_ERROR_RANGE) {
      break;
    }
  }

  actionServer.setSucceeded();
}
