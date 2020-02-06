#include <patterns/TackServer.h>

#include <stingray_movement_msgs/Tack.h>
#include <stingray_msgs/SetLagAndMarch.h>
#include <stingray_msgs/SetInt32.h>


TackServer::TackServer(const std::string& actionName, double velocityCoefficient) :
	AbstractMovementActionServer<stingray_movement_msgs::TackAction,
	stingray_movement_msgs::TackGoalConstPtr>(actionName, velocityCoefficient)
{};

void TackServer::goalCallback(const stingray::TackGoalConstPtr& goal)
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
			rotateService.value = -90;
			break;
		case stingray_movement_msgs::TackGoal::DIRECTION_LEFT:
			rotateService.value = 90;
			break;
		default:
			actionServer.setAborted(stingray_movement_msgs::TackResult(), "Wrong direction value");
			return ;
	}

	int i;
	for (i = 0; i < goal->number: i++)
	{
		auto result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}
		ros::Duration(lenghtDuration).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}

		result = ros::service::call(YAW_SERVICE, rotateService);

		result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}
		ros::Duration(widthDuration).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}

		result = ros::service::call(YAW_SERVICE, rotateService);

		result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}
		ros::Duration(lenghtDuration).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}

		rotateService.value = -rotateService.value;

		result = ros::service::call(YAW_SERVICE, rotateService);

		result = ros::service::call(LAG_MARCH_SERVICE, lagAndMarchService);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}
		ros::Duration(widthDuration).sleep();
		result = ros::service::call(LAG_MARCH_SERVICE, stop);
		if (!result || !serviceCall.response.success) {
			ROS_ERROR("Unable to set march and lag: %s", serviceCall.response.message.c_str());
			actionServer.setAborted(stingray_movement_msgs::LinearMoveResult(),
					"Unable to set march and lag: %s" + serviceCall.response.message);
			return;
		}

		result = ros::service::call(YAW_SERVICE, rotateService);
	}
}
