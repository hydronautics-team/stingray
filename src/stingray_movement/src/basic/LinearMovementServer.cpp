#include <basic/LinearMovementServer.h>

#include <stingray_msgs/SetLagAndMarch.h>


LinearMovementServer::LinearMovementServer(const std::string &actionName, double velocityCoefficient) :
    AbstractMovementActionServer<stingray_movement_msgs::LinearMoveAction,
                                 stingray_movement_msgs::LinearMoveGoalConstPtr>(actionName, velocityCoefficient) {};

void LinearMovementServer::goalCallback(const stingray_movement_msgs::LinearMoveGoalConstPtr &goal) {
  stingray_msgs::SetLagAndMarch serviceCall;
  if (goal->velocity < 0.0 || goal->velocity > 1.0) {
    actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
                            "Velocity value must be between 0.0 and 1.0");
    return;
  }
  auto velocity = velocityCoefficient * goal->velocity;
  serviceCall.request.lag = serviceCall.request.march = 0.0;

  if (goal->duration < 0.0) {
    actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
                            "Duration value must not be less than 0.0");
    return;
  }
  auto duration = goal->duration / 1000.0;

  auto isStop = false;

  switch (goal->direction) {
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_MARCH_FORWARD:
      serviceCall.request.march = velocity;
      break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_MARCH_BACKWARDS:
      serviceCall.request.march = -velocity;
      break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_LAG_LEFT:
      serviceCall.request.lag = velocity;
      break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_LAG_RIGHT:
      serviceCall.request.lag = -velocity;
      break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_STOP:
      isStop = true;
      break;
    default:
      actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(), "Wrong direction value");
      return;;
  }

  auto result = ros::service::call(LAG_MARCH_SERVICE, serviceCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
    actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
                            "Unable to set march and lag: %s" + serviceCall.response.message);
    return;
  }
  if (isStop) {
    actionServer.setSucceeded();
    return;
  }

  ros::Duration(duration).sleep();

  serviceCall.request.march = 0.0;
  serviceCall.request.lag = 0.0;
  result = ros::service::call(LAG_MARCH_SERVICE, serviceCall);
  if (!result || !serviceCall.response.success) {
    ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
    actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
                            "Unable to set march and lag: %s" + serviceCall.response.message);
    return;
  }

  actionServer.setSucceeded();
}