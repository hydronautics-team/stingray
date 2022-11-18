#include <patterns/TackServer.h>

#include "ros/ros.h"
#include <stingray_movement_msgs/TackAction.h>
#include <stingray_communication_msgs/SetInt32.h>
#include <std_msgs/Int32.h>

TackServer::TackServer(const std::string& actionName, double velocityCoefficient) :
	AbstractMovementActionServer<stingray_movement_msgs::TackAction,
	stingray_movement_msgs::TackGoalConstPtr>(actionName, velocityCoefficient)
{};

void TackServer::goalCallback(const stingray_movement_msgs::TackGoalConstPtr& goal)
{
    
}
