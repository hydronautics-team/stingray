#include <patterns/TackServer.h>

#include "ros/ros.h"
#include <stingray_movement_msgs/TackAction.h>
#include <stingray_msgs/SetLagAndMarch.h>
#include <stingray_msgs/SetInt32.h>

TackServer::TackServer(const std::string& actionName, double velocityCoefficient) :
	AbstractMovementActionServer<stingray_movement_msgs::TackAction,
	stingray_movement_msgs::TackGoalConstPtr>(actionName, velocityCoefficient)
{};

void TackServer::goalCallback(const stingray_movement_msgs::TackGoalConstPtr& goal)
{
	stingray_msgs::SetLagAndMarch lagAndMarchService;
	stingray_msgs::SetLagAndMarch stop;
	stingray_msgs::SetInt32 rotateService;

	stop.request.lag = 0.0;
	stop.request.march = 0.0;
	auto velocity = goal->velocity * velocityCoefficient;
	lagAndMarchService.request.march = velocity;

	switch (goal->direction)
	{
		case stingray_movement_msgs::TackGoal::DIRECTION_RIGHT:
			rotateService.request.value = -90;
			break;
		case stingray_movement_msgs::TackGoal::DIRECTION_LEFT:
			rotateService.request.value = 90;
			break;
		default:
			actionServer.setAborted(stingray_movement_msgs::TackResult(), "Wrong direction value");
			return ;
	}

	stingray_movement_msgs::TackFeedback feedback;
	feedback.currentNumber = 0;

	int i;
	for (i = 0; i < goal->number; i++)
	{
		auto result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);

		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->lenght).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(YAW_SERVICE, rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}

		result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->width).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(YAW_SERVICE, rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}

		result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->lenght).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		rotateService.request.value = -rotateService.request.value;

		result = ros::service::call(YAW_SERVICE, rotateService);

		if (!result || !rotateService.response.success) {
			ROS_ERROR("Unable to set yaw: %s", rotateService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set yaw: %s" + rotateService.response.message);
			return;
		}
		result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !lagAndMarchService.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", lagAndMarchService.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + lagAndMarchService.response.message);
			return;
		}
		ros::Duration(goal->width).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !stop.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", stop.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::TackResult(),
					"Unable to set march and lag: %s" + stop.response.message);
			return;
		}

		result = ros::service::call(YAW_SERVICE, rotateService);

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
