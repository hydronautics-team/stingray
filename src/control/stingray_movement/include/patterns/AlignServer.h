#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_ALIGNSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_ALIGNSERVER_H_

#include <stingray_movement_msgs/AlignAction.h>

#include "common/AbstractMovementActionServer.h"

/**
 * Action server that is responsible for aligning
 * vehicle according to value obtained from topic
 */
class AlignServer: AbstractMovementActionServer<stingray_movement_msgs::AlignAction,
                                                         stingray_movement_msgs::AlignGoalConstPtr> {

 protected:

  void goalCallback(const stingray_movement_msgs::AlignGoalConstPtr& goal) override;

 public:

  AlignServer(const std::string& actionName, double velocityCoefficient);
  ~AlignServer() = default;

};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_ALIGNSERVER_H_
