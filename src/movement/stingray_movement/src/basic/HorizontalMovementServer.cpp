#include <basic/HorizontalMovementServer.h>

#include <stingray_communication_msgs/SetHorizontalMove.h>

HorizontalMovementServer::HorizontalMovementServer(const std::string &actionName, double velocityCoefficient) : AbstractMovementActionServer<stingray_movement_msgs::HorizontalMoveAction,
                                                                                                                                             stingray_movement_msgs::HorizontalMoveGoalConstPtr>(actionName, velocityCoefficient){};

void HorizontalMovementServer::goalCallback(const stingray_movement_msgs::HorizontalMoveGoalConstPtr &goal)
{
    ROS_INFO("March: %f", goal->march);
    ROS_INFO("Lag: %f", goal->lag);
    ROS_INFO("Yaw: %i", goal->yaw);

    stingray_communication_msgs::SetHorizontalMove serviceCall;
    if (goal->march < 0.0 || goal->march > 1.0)
    {
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "March value must be between 0.0 and 1.0");
        return;
    }

    if (goal->lag < 0.0 || goal->lag > 1.0)
    {
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "Lag value must be between 0.0 and 1.0");
        return;
    }

    serviceCall.request.march = velocityCoefficient * goal->march;
    serviceCall.request.lag = velocityCoefficient * goal->lag;
    serviceCall.request.yaw = goal->yaw;

    auto result = ros::service::call(ros_config["services"]["set_horizontal_move"], serviceCall);

    if (!result || !serviceCall.response.success)
    {
        ROS_ERROR("Unable to set horizontal move: %s", serviceCall.response.message.c_str());
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "Unable to set march and lag: %s" + serviceCall.response.message);
        return;
    }
    else
    {
        ROS_INFO("Success set horizontal move");
        actionServer.setSucceeded();
    }
}