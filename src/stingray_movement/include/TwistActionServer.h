#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_

#include <stingray_interfaces/action/twist_action.hpp>

#include "AbstractActionServer.h"


/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
class TwistActionServer : AbstractActionServer<stingray_interfaces::action::TwistAction> {

protected:

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::TwistAction>> goal_handle) override;

public:

  TwistActionServer(const std::string &actionName);
  ~TwistActionServer() = default;

};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
