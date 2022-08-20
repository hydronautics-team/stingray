#include <patterns/AlignServer.h>

#include <stingray_communication_msgs/SetLagAndMarch.h>
#include <stingray_movement_msgs/AlignValue.h>

AlignServer::AlignServer(const std::string &actionName, double velocityCoefficient) : AbstractMovementActionServer<stingray_movement_msgs::AlignAction,
                                                                                                                   stingray_movement_msgs::AlignGoalConstPtr>(actionName, velocityCoefficient){};

void AlignServer::goalCallback(const stingray_movement_msgs::AlignGoalConstPtr &goal)
{
    stingray_communication_msgs::SetLagAndMarch serviceCall;
    serviceCall.request.lag = serviceCall.request.march = 0.0;
    auto result = ros::service::call(ros_config["services"]["set_lag_march"], serviceCall);
    if (!result || !serviceCall.response.success)
    {
        ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
        actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                                "Unable to set march and lag: %s" + serviceCall.response.message);
        return;
    }

    bool inRange = false;
    int direction = 0;
    while (!inRange)
    {
        auto message = ros::topic::waitForMessage<stingray_movement_msgs::AlignValue>(goal->topicName);

        if (message->hasValue)
        {
            if (std::abs(message->difference) <= goal->delta)
            {
                serviceCall.request.lag = 0.0;
                inRange = true;
            }
            else
            {
                direction = message->difference > 0.0 ? -1 : 1;
                serviceCall.request.lag = 0.1 * direction;
            }
            ROS_INFO("Difference: %f, direction: %d", message->difference, direction);
        }
        else
        {
            direction = -direction;
            serviceCall.request.lag = 0.1 * direction;
        }

        result = ros::service::call(ros_config["services"]["set_lag_march"], serviceCall);
        if (!result || !serviceCall.response.success)
        {
            ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
            actionServer.setAborted(stingray_movement_msgs::AlignResult(),
                                    "Unable to set march and lag: %s" + serviceCall.response.message);
            return;
        }
    }

    actionServer.setSucceeded();
}