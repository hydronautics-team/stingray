#include <patterns/TackServer.h>

#include "ros/ros.h"
#include <stingray_movement_msgs/TackAction.h>
#include <stingray_communication_msgs/SetLagAndMarch.h>
#include <stingray_communication_msgs/SetInt32.h>
#include <std_msgs/Int32.h>

TackServer::TackServer(const std::string& actionName, double velocityCoefficient) :
	AbstractMovementActionServer<stingray_movement_msgs::TackAction,
	stingray_movement_msgs::TackGoalConstPtr>(actionName, velocityCoefficient)
{};

void TackServer::goalCallback(const stingray_movement_msgs::TackGoalConstPtr& goal)
{
    stingray_communication_msgs::SetLagAndMarch lagAndMarchService;
    stingray_communication_msgs::SetLagAndMarch stop;
    stingray_communication_msgs::SetInt32 rotateService;

	stop.request.lag = 0.0;
	stop.request.march = 0.0;
	auto velocity = goal->velocity * velocityCoefficient;
	lagAndMarchService.request.march = velocity;

	std_msgs::Int32 yawMessage = *ros::topic::waitForMessage<std_msgs::Int32>(ros_config["topics"]["yaw"], nodeHandle);
	int currentYaw = yawMessage.data;
	int rotation;

	switch (goal->direction)
	{
		case stingray_movement_msgs::TackGoal::DIRECTION_RIGHT:
			rotation = -90;
			break;
		case stingray_movement_msgs::TackGoal::DIRECTION_LEFT:
			rotation = 90;
			break;
		default:
			actionServer.setAborted(stingray_movement_msgs::TackResult(), "Wrong direction value");
			return ;
	}
	rotateService.request.value = currentYaw + rotation;

	stingray_movement_msgs::TackFeedback feedback;
	feedback.currentNumber = 0;

	int i;
	for (i = 0; i < goal->number; i++)
	{
		auto result = ros::service::call(ros_config["services"]["set_lag_march"], lagAndMarchService);

		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->lenght).sleep();
		result = ros::service::call(ros_config["services"]["set_lag_march"], stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(ros_config["services"]["set_yaw"], rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}
		rotateService.request.value += rotation;

		result = ros::service::call(ros_config["services"]["set_lag_march"], lagAndMarchService);
		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->width).sleep();
		result = ros::service::call(ros_config["services"]["set_lag_march"], stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(ros_config["services"]["set_yaw"], rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}
		rotateService.request.value -= rotation;

		result = ros::service::call(ros_config["services"]["set_lag_march"], lagAndMarchService);
		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->lenght).sleep();
		result = ros::service::call(ros_config["services"]["set_lag_march"], stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(ros_config["services"]["set_yaw"], rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}
		rotateService.request.value -= rotation;
		result = ros::service::call(ros_config["services"]["set_lag_march"], lagAndMarchService);
		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->width).sleep();
		result = ros::service::call(ros_config["services"]["set_lag_march"], stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(ros_config["services"]["set_yaw"], rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}
		feedback.currentNumber++;
		actionServer.publishFeedback(feedback);
	}

	stingray_movement_msgs::TackResult result;
	result.number = i;
	actionServer.setSucceeded(result);
}
