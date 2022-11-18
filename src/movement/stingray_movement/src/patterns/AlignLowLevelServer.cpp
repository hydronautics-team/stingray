#include <patterns/AlignLowLevelServer.h>

#include <stingray_communication_msgs/SetStabilization.h>
#include <stingray_movement_msgs/AlignValue.h>

AlignLowLevelServer::AlignLowLevelServer(const std::string &actionName, double velocityCoefficient) : AbstractMovementActionServer<stingray_movement_msgs::AlignAction,
                                                                                                                                   stingray_movement_msgs::AlignGoalConstPtr>(actionName, velocityCoefficient){};

void AlignLowLevelServer::goalCallback(const stingray_movement_msgs::AlignGoalConstPtr &goal)
{
    
}