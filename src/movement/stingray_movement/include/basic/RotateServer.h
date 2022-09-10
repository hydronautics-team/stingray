#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ROTATESERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ROTATESERVER_H_

#include <stingray_movement_msgs/RotateAction.h>

#include "common/AbstractMovementActionServer.h"

/**
 * Action server that is responsible for rotating the vehicle.
 */
class RotateServer : AbstractMovementActionServer<stingray_movement_msgs::RotateAction,
                                                  stingray_movement_msgs::RotateGoalConstPtr>
{

protected:
    void goalCallback(const stingray_movement_msgs::RotateGoalConstPtr &goal) override;

public:
    RotateServer(const std::string &actionName, double rotateCoefficient);
    ~RotateServer() = default;
};

#endif // STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ROTATESERVER_H_
