#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_

#include <stingray_movement_msgs/LinearMoveAction.h>

#include "AbstractMovementActionServer.h"

class LinearMovementServer: AbstractMovementActionServer<stingray_movement_msgs::LinearMoveAction,
                                                         stingray_movement_msgs::LinearMoveGoalConstPtr> {

 protected:

  void goalCallback(const stingray_movement_msgs::LinearMoveGoalConstPtr& goal) override;

 public:

  LinearMovementServer(const std::string& actionName, double velocityCoefficient);
  ~LinearMovementServer() = default;

};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
