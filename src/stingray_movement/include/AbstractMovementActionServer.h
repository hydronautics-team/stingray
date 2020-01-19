#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <stingray_movement_msgs/LinearMoveAction.h>

/**
 * Abstract base for implementing action servers
 * related to vehicle movement.
 *
 * @tparam TAction Action type
 * @tparam TGoalPtr Action goal const pointer type
 */
template <class TAction, class TGoalPtr>
class AbstractMovementActionServer {

 protected:

  static const std::string LAG_MARCH_SERVICE;

  ros::NodeHandle nodeHandle;
  actionlib::SimpleActionServer<TAction> actionServer;

  double velocityCoefficient;

  /** Implement your goal processing logic */
  virtual void goalCallback(const TGoalPtr& goal) = 0;

 public:

  AbstractMovementActionServer(const std::string& actionName, double velocityCoefficient);
  ~AbstractMovementActionServer() = default;

};


template <class TAction, class TGoalPtr>
const std::string AbstractMovementActionServer<TAction, TGoalPtr>::LAG_MARCH_SERVICE =
    "/stingray/services/control/set_lag_and_march";

template <class TAction, class TGoalPtr>
AbstractMovementActionServer<TAction, TGoalPtr>::AbstractMovementActionServer(const std::string &actionName,
                                                                              double velocityCoefficient): actionServer(
    nodeHandle,
    actionName,
    boost::bind(&AbstractMovementActionServer<TAction, TGoalPtr>::goalCallback,
                this, _1),
    false), velocityCoefficient(velocityCoefficient) {}


#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
