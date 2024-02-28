#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_

#include <stingray_interfaces/action/twist_action.hpp>

#include "AbstractActionServer.h"
#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"

using namespace std::chrono_literals;

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
class TwistActionServer : AbstractActionServer<stingray_interfaces::action::TwistAction, stingray_interfaces::action::TwistAction_Goal> {

protected:

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::TwistAction>> goal_handle) override;

public:

    TwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~TwistActionServer() = default;

private:
    void uvStateCallback(const stingray_core_interfaces::msg::UVState &msg);
    rclcpp::Subscription<stingray_core_interfaces::msg::UVState>::SharedPtr uvStateSub;
    rclcpp::Client<stingray_core_interfaces::srv::SetTwist>::SharedPtr twistSrvClient;

    float current_depth;
    float current_yaw;
    float current_pitch;
    float current_roll;

    bool depth_stabilization;
    bool roll_stabilization;
    bool pitch_stabilization;
    bool yaw_stabilization;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
