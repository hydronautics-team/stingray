#include <ros/ros.h>
#include <std_msgs/UInt32.h>
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
#include <stingray_communication_msgs/SetLagAndMarch.h>
#include <stingray_utils/json.hpp>

using json = nlohmann::json;

// get json config
static const json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
static const json common_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/common.json"));

// get constants from config
static const std::string MODEL_NAME = common_config["simulation"]["model_name"].get<std::string>();
static const uint32_t COMMUNICATION_DELAY_MILLISECONDS = common_config["simulation"]["communication_delay"].get<uint32_t>();
static const double INITIAL_YAW = common_config["simulation"]["initial_yaw"].get<double>();
static const double INITIAL_ROLL = common_config["simulation"]["initial_roll"].get<double>();
static const double INITIAL_PITCH = common_config["simulation"]["initial_pitch"].get<double>();
static const double INITIAL_DEPTH = common_config["simulation"]["initial_depth"].get<double>();

std_msgs::UInt32 depthMessage;
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
    modelState.request.model_name = MODEL_NAME;
    bool result = ros::service::call(ros_config["services"]["GAZEBO_GET_STATE_SERVICE"].get<std::string>(), modelState);
    if (!result || !modelState.response.success)
    {
        throw std::runtime_error("Failed to obtain state in Gazebo: " + modelState.response.status_message);
    }

    gazebo_msgs::SetModelState newModelState;
    newModelState.request.model_state.pose = modelState.response.pose;
    newModelState.request.model_state.twist = modelState.response.twist;
    newModelState.request.model_state.model_name = MODEL_NAME;

    transform(newModelState.request.model_state);

    result = ros::service::call(ros_config["services"]["GAZEBO_SET_STATE_SERVICE"].get<std::string>(), newModelState);
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
bool lagAndMarchCallback(stingray_communication_msgs::SetLagAndMarch::Request &request,
                         stingray_communication_msgs::SetLagAndMarch::Response &response)
{

    ROS_INFO("lagAndMarchCallback in gazebo bridge");

    currentTwist.linear.x = request.march;
    currentTwist.linear.y = -request.lag;

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
      modelState.pose.position.z = INITIAL_DEPTH - request.value / 100.0; });
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
 * Rotates vehicle on specified yaw angle
 * @param request Service request with yaw angle in degrees
 * @param response Service response
 * @return {@code true} if service call didn't fail
 */
bool yawCallback(stingray_communication_msgs::SetInt32::Request &request,
                 stingray_communication_msgs::SetInt32::Response &response)
{
    /*
     * Here we simulate enabled yaw stabilization: we just pass desired yaw angle
     * for Gazebo like it is low-level control system that stabilizes this angle.
     */

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
      double desiredYaw = request.value % 360;
      double newYaw = INITIAL_YAW + desiredYaw * M_PI / 180.0;
      modelState.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(INITIAL_ROLL, INITIAL_PITCH, newYaw); });
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

    ros::Rate communicationDelay(1000.0 / COMMUNICATION_DELAY_MILLISECONDS);

    ros::Publisher depthPublisher = nodeHandle.advertise<std_msgs::UInt32>(ros_config["topics"]["DEPTH_PUBLISH_TOPIC"].get<std::string>(), 20);
    ros::Publisher yawPublisher = nodeHandle.advertise<std_msgs::Int32>(ros_config["topics"]["YAW_PUBLISH_TOPIC"].get<std::string>(), 20);
    ros::Publisher velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(ros_config["topics"]["GAZEBO_VELOCITY_TOPIC"].get<std::string>(), 20);

    ros::ServiceServer velocityService = nodeHandle.advertiseService(ros_config["services"]["SET_LAG_AND_MARCH_SERVICE"].get<std::string>(), lagAndMarchCallback);
    ros::ServiceServer depthService = nodeHandle.advertiseService(ros_config["services"]["SET_DEPTH_SERVICE"].get<std::string>(), depthCallback);
    ros::ServiceServer yawService = nodeHandle.advertiseService(ros_config["services"]["SET_YAW_SERVICE"].get<std::string>(), yawCallback);
    ros::ServiceServer imuService = nodeHandle.advertiseService(ros_config["services"]["SET_IMU_ENABLED_SERVICE"].get<std::string>(), imuCallback);
    ros::ServiceServer stabilizationService = nodeHandle.advertiseService(ros_config["services"]["SET_STABILIZATION_SERVICE"].get<std::string>(), stabilizationCallback);
    ros::ServiceServer deviceService = nodeHandle.advertiseService(ros_config["services"]["SET_DEVICE_SERVICE"].get<std::string>(), deviceActionCallback);

    gazebo_msgs::GetModelState modelState;
    modelState.request.model_name = MODEL_NAME;

    currentTwist.linear.x = currentTwist.linear.y = currentTwist.linear.z =
        currentTwist.angular.x = currentTwist.angular.y = currentTwist.angular.z = 0;

    while (ros::ok())
    {

        bool result = ros::service::call(ros_config["services"]["GAZEBO_GET_STATE_SERVICE"].get<std::string>(), modelState);
        if (!result || !modelState.response.success)
        {
            ROS_ERROR("Failed to obtain current model state from Gazebo!");
        }
        else
        {
            // Convert back to initial values
            depthMessage.data = -(modelState.response.pose.position.z - INITIAL_DEPTH) * 100;
            yawMessage.data = (tf::getYaw(modelState.response.pose.orientation) - INITIAL_YAW) * 180.0 / M_PI;
        }

        depthPublisher.publish(depthMessage);
        yawPublisher.publish(yawMessage);

        velocityPublisher.publish(currentTwist);

        ros::spinOnce();
        communicationDelay.sleep();
    }

    return 0;
}