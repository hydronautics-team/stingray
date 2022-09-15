#include <patterns/AlignServer.h>

#include <stingray_movement_msgs/AlignValue.h>

AlignServer::AlignServer(const std::string &actionName, double velocityCoefficient) : AbstractMovementActionServer<stingray_movement_msgs::AlignAction,
                                                                                                                   stingray_movement_msgs::AlignGoalConstPtr>(actionName, velocityCoefficient){};

void AlignServer::goalCallback(const stingray_movement_msgs::AlignGoalConstPtr &goal)
{
    
}