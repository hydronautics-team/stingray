#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_ALIGNLOWLEVELSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_ALIGNLOWLEVELSERVER_H_

#include <stingray_movement_msgs/AlignAction.h>

#include "common/AbstractMovementActionServer.h"

/**
 * Action server that is responsible for aligning
 * vehicle according to value obtained from topic
 * using low-level CS stabilization
 */
class AlignLowLevelServer: AbstractMovementActionServer<stingray_movement_msgs::AlignAction,
                                                        stingray_movement_msgs::AlignGoalConstPtr> {
 private:
  static const std::string STABILIZATION_SERVICE;

 protected:

  void goalCallback(const stingray_movement_msgs::AlignGoalConstPtr& goal) override;

 public:

  AlignLowLevelServer(const std::string& actionName, double velocityCoefficient);
  ~AlignLowLevelServer() = default;

};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_PATTERNS_ALIGNLOWLEVELSERVER_H_
