#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_

#include <stingray_movement_msgs/HorizontalMoveAction.h>

#include "common/AbstractMovementActionServer.h"

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
class HorizontalMovementServer : AbstractMovementActionServer<stingray_movement_msgs::HorizontalMoveAction,
                                                              stingray_movement_msgs::HorizontalMoveGoalConstPtr>
{

protected:
    void goalCallback(const stingray_movement_msgs::HorizontalMoveGoalConstPtr &goal) override;

public:
    HorizontalMovementServer(const std::string &actionName, double velocityCoefficient);
    ~HorizontalMovementServer() = default;
};

#endif // STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
