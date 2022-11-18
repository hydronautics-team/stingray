#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_DIVESERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_DIVESERVER_H_

#include <stingray_movement_msgs/DiveAction.h>

#include "common/AbstractMovementActionServer.h"

/**
 * Action server that is responsible for diving the vehicle.
 */
class DiveServer : AbstractMovementActionServer<stingray_movement_msgs::DiveAction,
                                                stingray_movement_msgs::DiveGoalConstPtr>
{

protected:
    void goalCallback(const stingray_movement_msgs::DiveGoalConstPtr &goal) override;

public:
    DiveServer(const std::string &actionName);
    ~DiveServer() = default;
};

#endif // STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_DIVESERVER_H_
