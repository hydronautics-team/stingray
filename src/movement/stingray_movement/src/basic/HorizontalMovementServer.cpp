#include <basic/HorizontalMovementServer.h>

#include "common/AsyncTimer.h"
#include <stingray_communication_msgs/SetHorizontalMove.h>

HorizontalMovementServer::HorizontalMovementServer(const std::string &actionName, double velocityCoefficient) : AbstractMovementActionServer<stingray_movement_msgs::HorizontalMoveAction,
                                                                                                                                             stingray_movement_msgs::HorizontalMoveGoalConstPtr>(actionName, velocityCoefficient){};

void HorizontalMsovementServer::goalCallback(const stingray_movement_msgs::HorizontalMoveGoalConstPtr &goal)
{
    ROS_INFO("Velosity: %f, direction: %d", goal->velocity, goal->direction);

    stingray_communication_msgs::SetHorizontalMove serviceCall;
    if (goal->velocity < 0.0 || goal->velocity > 1.0)
    {
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "Velocity value must be between 0.0 and 1.0");
        return;
    }
    auto velocity = velocityCoefficient * goal->velocity;
    serviceCall.request.lag = serviceCall.request.march = 0.0;

    if (goal->duration < 0.0)
    {
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "Duration value must not be less than 0.0");
        return;
    }

    ros::Rate checkRate(5);
    AsyncTimer timer(goal->duration);

    auto isStop = false;

    switch (goal->direction)
    {
    case stingray_movement_msgs::HorizontalMoveGoal::DIRECTION_MARCH_FORWARD:
        serviceCall.request.march = velocity;
        break;
    case stingray_movement_msgs::HorizontalMoveGoal::DIRECTION_MARCH_BACKWARDS:
        serviceCall.request.march = -velocity;
        break;
    case stingray_movement_msgs::HorizontalMoveGoal::DIRECTION_LAG_RIGHT:
        serviceCall.request.lag = velocity;
        break;
    case stingray_movement_msgs::HorizontalMoveGoal::DIRECTION_LAG_LEFT:
        serviceCall.request.lag = -velocity;
        break;
    case stingray_movement_msgs::HorizontalMoveGoal::DIRECTION_STOP:
        isStop = true;
        break;
    default:
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(), "Wrong direction value");
        return;
        ;
    }

    auto result = ros::service::call(ros_config["services"]["set_lag_march"], serviceCall);

    if (!result || !serviceCall.response.success)
    {
        ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "Unable to set march and lag: %s" + serviceCall.response.message);
        return;
    }
    if (isStop)
    {
        actionServer.setSucceeded();
        return;
    }

    timer.start();
    bool preempted = false;
    while (timer.isBusy() && !preempted)
    {
        preempted = actionServer.isPreemptRequested() || !ros::ok();
        checkRate.sleep();
    }

    serviceCall.request.march = 0.0;
    serviceCall.request.lag = 0.0;
    result = ros::service::call(ros_config["services"]["set_lag_march"], serviceCall);
    if (!result || !serviceCall.response.success)
    {
        ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
        actionServer.setAborted(stingray_movement_msgs::HorizontalMoveResult(),
                                "Unable to set march and lag: %s" + serviceCall.response.message);
        return;
    }

    if (!preempted)
    {
        actionServer.setSucceeded();
    }
    else
    {
        actionServer.setPreempted();
    }
}