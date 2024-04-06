#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BBOXCENTERINGTWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BBOXCENTERINGTWISTACTIONSERVER_H_

#include <stingray_interfaces/action/bbox_centering_twist_action.hpp>

#include "stingray_movement/AbstractCenteringTwistActionServer.h"
#include "stingray_utils/AsyncTimer.h"
#include "stingray_interfaces/msg/bbox_array.hpp"

using namespace std::chrono_literals;

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
class HydroacousticCenteringTwistActionServer : public AbstractCenteringTwistActionServer<stingray_interfaces::action::HydroacousticCenteringTwistAction, stingray_interfaces::action::HydroacousticCenteringTwistAction_Goal> {

public:
    HydroacousticCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~HydroacousticCenteringTwistActionServer() = default;

private:
    bool isTwistDone(const std::shared_ptr<const stingray_interfaces::action::HydroacousticCenteringTwistAction_Goal> goal)  override;
    bool isCenteringTwistDone() override;
    bool isTargetLost() override;
    void bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::HydroacousticCenteringTwistAction>> goal_handle) override;
    rclcpp::Subscription<stingray_interfaces::msg::BboxArray>::SharedPtr bboxArraySub;

    std::string target_bbox_name = "";
    std::vector<std::string> target_avoid_bbox_name_array;
    stingray_interfaces::msg::Bbox current_target_bbox;
    stingray_interfaces::msg::Bbox current_avoid_target_bbox;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BBOXCENTERINGTWISTACTIONSERVER_H_
