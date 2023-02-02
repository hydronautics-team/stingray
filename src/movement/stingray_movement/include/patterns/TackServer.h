#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_TACKSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_TACKSERVER_H_

#include <stingray_movement_msgs/msg/TackAction.h>

#include "common/AbstractMovementActionServer.h"

class TackServer : AbstractMovementActionServer<stingray_movement_msgs::TackAction,
						stingray_movement_msgs::TackGoalConstPtr>
{
protected:
	void goalCallback(const stingray_movement_msgs::TackGoalConstPtr& goal) override;

public:
	TackServer(const std::string& actionName, double velocityCoefficient);
	~TackServer() = default;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_TACKSERVER_H_
