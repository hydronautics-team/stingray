#include <basic/DiveServer.h>

#include <cmath>

#include <stingray_communication_msgs/SetInt32.hpp>
#include <std_msgs/UInt32.hpp>
#include <stingray_utils/json.hpp>

using json = nlohmann::json;

// get json configs
static const json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));

DiveServer::DiveServer(const std::string &actionName) : AbstractMovementActionServer<stingray_movement_msgs::DiveAction,
                                                                                     stingray_movement_msgs::DiveGoalConstPtr>(actionName, 0.0){};

void DiveServer::goalCallback(const stingray_movement_msgs::DiveGoalConstPtr &goal)
{
    stingray_communication_msgs::SetInt32 serviceCall;
    serviceCall.request.value = goal->depth;
    auto result = ros::service::call(ros_config["services"]["set_depth"], serviceCall);
    if (!result || !serviceCall.response.success)
    {
        RCL_ERROR("Unable to set depth: %s", serviceCall.response.message.c_str());
        actionServer.setAborted(stingray_movement_msgs::DiveResult(),
                                "Unable to set depth: %s" + serviceCall.response.message);
        return;
    }

    if (goal->check_depth)
    {
        int desiredDepth = goal->depth;
        while (true)
        {
            std_msgs::UInt32 depthMessage = *ros::topic::waitForMessage<std_msgs::UInt32>(ros_config["topics"]["depth"], nodeHandle);
            int currentDepth = depthMessage.data;
            auto delta = std::abs(desiredDepth - currentDepth);
            RCL_INFO("Current depth: %d, desired depth: %d", currentDepth, desiredDepth);
            if (delta <= control_config["movement"]["depth_error_range"])
            {
                break;
            }
        }
    }

    actionServer.setSucceeded();
}
