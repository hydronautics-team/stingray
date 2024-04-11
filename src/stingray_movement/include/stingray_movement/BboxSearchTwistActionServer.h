#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BBOXSEARCHTWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BBOXSEARCHTWISTACTIONSERVER_H_

#include <stingray_interfaces/action/bbox_search_twist_action.hpp>

#include "stingray_movement/AbstractSearchTwistActionServer.h"
#include "stingray_utils/AsyncTimer.h"
#include "stingray_interfaces/msg/bbox_array.hpp"

using namespace std::chrono_literals;


class BboxSearchTwistActionServer : public AbstractSearchTwistActionServer<stingray_interfaces::action::BboxSearchTwistAction, stingray_interfaces::action::BboxSearchTwistAction_Goal> {

public:

    BboxSearchTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~BboxSearchTwistActionServer() = default;

private:
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxSearchTwistAction>> goal_handle) override;
    bool isTwistDone(const std::shared_ptr<const stingray_interfaces::action::BboxSearchTwistAction_Goal> goal) override;
    bool isSearchTwistDone() override;
    void bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg);
    rclcpp::Subscription<stingray_interfaces::msg::BboxArray>::SharedPtr bboxArraySub;

    std::string target_bbox_name = "";
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_BBOXSEARCHTWISTACTIONSERVER_H_
