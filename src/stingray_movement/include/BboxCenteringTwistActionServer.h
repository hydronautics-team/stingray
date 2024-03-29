#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_

#include <stingray_interfaces/action/bbox_centering_twist_action.hpp>

#include "stingray_utils/AbstractActionServer.h"
#include "stingray_utils/AsyncTimer.h"
#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"
#include "stingray_interfaces/msg/bbox_array.hpp"

using namespace std::chrono_literals;

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
class BboxCenteringTwistActionServer : AbstractActionServer<stingray_interfaces::action::BboxCenteringTwistAction, stingray_interfaces::action::BboxCenteringTwistAction_Goal> {

protected:

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxCenteringTwistAction>> goal_handle) override;

public:

    BboxCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~BboxCenteringTwistActionServer() = default;

private:
    void uvStateCallback(const stingray_core_interfaces::msg::UVState &msg);
    void bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg);
    bool isTwistDone(const std::shared_ptr<const stingray_interfaces::action::BboxCenteringTwistAction_Goal> goal);
    rclcpp::Client<stingray_core_interfaces::srv::SetTwist>::SharedPtr twistSrvClient;
    rclcpp::Subscription<stingray_core_interfaces::msg::UVState>::SharedPtr uvStateSub;
    rclcpp::Subscription<stingray_interfaces::msg::BboxArray>::SharedPtr bboxArraySub;

    float depth_tolerance = 100.0;
    float yaw_tolerance = 10.0;
    float pitch_tolerance = 10.0;
    float roll_tolerance = 10.0;

    float current_depth;
    float current_yaw;
    float current_pitch;
    float current_roll;

    bool depth_stabilization;
    bool roll_stabilization;
    bool pitch_stabilization;
    bool yaw_stabilization;

    bool move_in_progress = false;
    std::string target_name = "";
    int image_width = 640;
    int image_height = 480;
    int camera_fov = 60;
    float centering_angle_difference = 0.0;

    int target_disappeared_counter = 0;
    bool target_big = false;
    bool target_lost = false;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BASICMOVEMENTSERVER_H_