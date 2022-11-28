#include <patterns/TackServer.h>

#include "rclcpp/rclcpp.hpp"
#include <stingray_movement_msgs/TackAction.h>
#include <stingray_communication_msgs/SetInt32.hpp>
#include <std_msgs/Int32.hpp>

TackServer::TackServer(const std::string& actionName, double velocityCoefficient) :
	AbstractMovementActionServer<stingray_movement_msgs::TackAction,
	stingray_movement_msgs::TackGoalConstPtr>(actionName, velocityCoefficient)
{};

void TackServer::goalCallback(const stingray_movement_msgs::TackGoalConstPtr& goal)
{
    
}
