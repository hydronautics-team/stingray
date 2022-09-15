#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <std_srvs/SetBool.h>
#include <ros/package.h>

#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>

#include <stingray_communication_msgs/SetInt32.h>
#include <stingray_communication_msgs/SetStabilization.h>
#include <stingray_communication_msgs/SetDeviceAction.h>
#include <stingray_communication_msgs/SetHorizontalMove.h>
#include <stingray_utils/json.hpp>

using json = nlohmann::json;

// get json configs
static const json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
static const json simulation_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/simulation.json"));

std_msgs::Int32 depthMessage;
std_msgs::Int32 yawMessage;
geometry_msgs::Twist currentTwist;
bool depthStabilizationEnabled = false;
bool yawStabilizationEnabled = true;

/**
 * Obtains model state from Gazebo, transforms and updates it.
 * @param transform Function that transforms current model state
 * @throws {@code std::runtime_error} if fails to get or set model state in Gazebo
 */
void updateModelState(const std::function<void(gazebo_msgs::ModelState &)> &transform)
{
    gazebo_msgs::GetModelState modelState;
    modelState.request.model_name = simulation_config["model_name"];
    bool result = ros::service::call(ros_config["services"]["gazebo_get_state"], modelState);
    if (!result || !modelState.response.success)
    {
        throw std::runtime_error("Failed to obtain state in Gazebo: " + modelState.response.status_message);
    }

    gazebo_msgs::SetModelState newModelState;
    newModelState.request.model_state.pose = modelState.response.pose;
    newModelState.request.model_state.twist = modelState.response.twist;
    newModelState.request.model_state.model_name = simulation_config["model_name"];

    transform(newModelState.request.model_state);

    result = ros::service::call(ros_config["services"]["gazebo_set_state"], newModelState);
    if (!result || !newModelState.response.success)
    {
        throw std::runtime_error("Failed to update state in Gazebo: " + modelState.response.status_message);
    }
}

/**
 * Sets vehicle lag and march speed
 * @param request Service request with lag and march speed in Gazebo-related units
 * @param response Service response
 * @return {@code true} if service call didn't fail
 */
bool horizontalMoveCallback(stingray_communication_msgs::SetHorizontalMove::Request &request,
                            stingray_communication_msgs::SetHorizontalMove::Response &response)
{

    // ROS_INFO("horizontalMoveCallback in gazebo bridge");

    currentTwist.linear.x = request.lag;
    currentTwist.linear.y = -request.march;

    if (!yawStabilizationEnabled)
    {
        response.success = false;
        response.message = "Yaw stabilization is not enabled";
        return true;
    }

    try
    {
        updateModelState([request](gazebo_msgs::ModelState &modelState)
                         {
      double desiredYaw = -request.yaw % 360;
    //   ROS_INFO("desiredYaw gazebo %f", desiredYaw);
      double newYaw = simulation_config["initial_yaw"].get<double>() + desiredYaw * M_PI / 180.0;
    //   ROS_INFO("newYaw gazebo %f", newYaw);
      modelState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(simulation_config["initial_roll"].get<double>(), simulation_config["initial_pitch"].get<double>(), newYaw); });
    }
    catch (std::runtime_error &e)
    {
        response.success = false;
        response.message = "Failed to set depth in Gazebo: " + std::string(e.what());
        return true;
    }

    response.success = true;
    return true;
}

/**
 * Dives vehicle on specified depth
 * @param request Service request with depth in centimetres
 * @param response Service response
 * @return {@code true} if service call didn't fail
 */
bool depthCallback(stingray_communication_msgs::SetInt32::Request &request,
                   stingray_communication_msgs::SetInt32::Response &response)
{
    /*
     * Here we simulate enabled depth stabilization: we just pass desired depth
     * for Gazebo like it is low-level control system that stabilizes this depth.
     */

    if (!depthStabilizationEnabled)
    {
        response.success = false;
        response.message = "Depth stabilization is not enabled";
        return true;
    }

    try
    {
        updateModelState([request](gazebo_msgs::ModelState &modelState)
                         {
      /* In our simulator scale is 1.0 = 1 metre, and target depth is passed in centimetres.
       * Bias is needed due to simulator implementation details. */
      modelState.pose.position.z = simulation_config["initial_depth"].get<double>() - request.value / 100.0; });
    }
    catch (std::runtime_error &e)
    {
        response.success = false;
        response.message = "Failed to set depth in Gazebo: " + std::string(e.what());
        return true;
    }

    response.success = true;
    return true;
}

bool imuCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
    response.success = true;
    return true;
}

bool stabilizationCallback(stingray_communication_msgs::SetStabilization::Request &request,
                           stingray_communication_msgs::SetStabilization::Response &response)
{
    depthStabilizationEnabled = request.depthStabilization;
    yawStabilizationEnabled = request.yawStabilization;

    response.success = true;
    return true;
}

bool deviceActionCallback(stingray_communication_msgs::SetDeviceAction::Request &request,
                          stingray_communication_msgs::SetDeviceAction::Response &response)
{
    response.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_bridge");
    ros::NodeHandle nodeHandle;

    ros::Rate communicationDelay(1000.0 / simulation_config["communication_delay"].get<uint32_t>());

    ros::Publisher depthPublisher = nodeHandle.advertise<std_msgs::Int32>(ros_config["topics"]["depth"], 20);
    ros::Publisher yawPublisher = nodeHandle.advertise<std_msgs::Int32>(ros_config["topics"]["yaw"], 20);
    ros::Publisher velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(ros_config["topics"]["gazebo_velocity"], 20);

    ros::ServiceServer horizontalMoveService = nodeHandle.advertiseService(ros_config["services"]["set_horizontal_move"], horizontalMoveCallback);
    ros::ServiceServer depthService = nodeHandle.advertiseService(ros_config["services"]["set_depth"], depthCallback);
    ros::ServiceServer imuService = nodeHandle.advertiseService(ros_config["services"]["set_imu_enabled"], imuCallback);
    ros::ServiceServer stabilizationService = nodeHandle.advertiseService(ros_config["services"]["set_stabilization_enabled"], stabilizationCallback);
    ros::ServiceServer deviceService = nodeHandle.advertiseService(ros_config["services"]["set_device"], deviceActionCallback);

    gazebo_msgs::GetModelState modelState;
    modelState.request.model_name = simulation_config["model_name"];

    currentTwist.linear.x = currentTwist.linear.y = currentTwist.linear.z =
        currentTwist.angular.x = currentTwist.angular.y = currentTwist.angular.z = 0;

    while (ros::ok())
    {

        bool result = ros::service::call(ros_config["services"]["gazebo_get_state"], modelState);
        if (!result || !modelState.response.success)
        {
            ROS_ERROR("Failed to obtain current model state from Gazebo!");
        }
        else
        {
            // Convert back to initial values
            depthMessage.data = -(modelState.response.pose.position.z - simulation_config["initial_depth"].get<double>()) * 100;
            float yaw_postprocessed = -(tf::getYaw(modelState.response.pose.orientation) - simulation_config["initial_yaw"].get<double>()) * 180.0 / M_PI;
            if (yaw_postprocessed > 180)
            {
                yaw_postprocessed -= 360;
            }
            else if (yaw_postprocessed < -180)
            {
                yaw_postprocessed += 360;
            }
            yawMessage.data = yaw_postprocessed;
            // ROS_INFO("Yaw gazebo %f", yaw_postprocessed);
        }

        depthPublisher.publish(depthMessage);
        yawPublisher.publish(yawMessage);

        velocityPublisher.publish(currentTwist);

        ros::spinOnce();
        communicationDelay.sleep();
    }

    return 0;
}