#include <basic/LinearMovementServer.h>

#include "common/AsyncTimer.h"
#include <stingray_communication_msgs/SetLagAndMarch.h>

LinearMovementServer::LinearMovementServer(const std::string &actionName, double velocityCoefficient) : AbstractMovementActionServer<stingray_movement_msgs::LinearMoveAction,
                                                                                                                                     stingray_movement_msgs::LinearMoveGoalConstPtr>(actionName, velocityCoefficient){};

void LinearMovementServer::goalCallback(const stingray_movement_msgs::LinearMoveGoalConstPtr &goal)
{
    ROS_INFO("Velosity: %f, direction: %d", goal->velocity, goal->direction);

    stingray_communication_msgs::SetLagAndMarch serviceCall;
    if (goal->velocity < 0.0 || goal->velocity > 1.0)
    {
        actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
                                "Velocity value must be between 0.0 and 1.0");
        ROS_INFO("pidor");
        return;
    }
    auto velocity = velocityCoefficient * goal->velocity;
    ROS_INFO("Velositi suka %d", velocity);
    serviceCall.request.lag = serviceCall.request.march = 0.0;

    if (goal->duration < 0.0)
    {
        actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
                                "Duration value must not be less than 0.0");
        ROS_INFO("pidor2");
        return;
    }

    ros::Rate checkRate(5);
    AsyncTimer timer(goal->duration);

    auto isStop = false;

    switch (goal->direction)
    {
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_MARCH_FORWARD:
        serviceCall.request.march = velocity;
        break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_MARCH_BACKWARDS:
        serviceCall.request.march = -velocity;
        break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_LAG_RIGHT:
        serviceCall.request.lag = velocity;
        break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_LAG_LEFT:
        serviceCall.request.lag = -velocity;
        break;
    case stingray_movement_msgs::LinearMoveGoal::DIRECTION_STOP:
        isStop = true;
        break;
    default:
        actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(), "Wrong direction value");
        ROS_INFO("pidor");
        return;
        ;
    }

    auto result = ros::service::call(ros_config["services"]["set_lag_march"], serviceCall);
    ROS_INFO("LAG_MARCH_SERVICE called");

    if (!result || !serviceCall.response.success)
    {
        ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
        actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
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
        actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
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