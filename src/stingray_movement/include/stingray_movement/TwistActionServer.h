#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_TWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_TWISTACTIONSERVER_H_

#include <stingray_interfaces/action/twist_action.hpp>

#include "stingray_movement/AbstractTwistActionServer.h"
#include "stingray_utils/AsyncTimer.h"

using namespace std::chrono_literals;

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
class TwistActionServer : public AbstractTwistActionServer<stingray_interfaces::action::TwistAction, stingray_interfaces::action::TwistAction_Goal> {

protected:

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::TwistAction>> goal_handle) override;
    bool isTwistDone(const std::shared_ptr<const stingray_interfaces::action::TwistAction_Goal> goal);

public:

    TwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~TwistActionServer() = default;

private:
    float target_yaw;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_TWISTACTIONSERVER_H_
