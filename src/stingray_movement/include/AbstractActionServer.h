#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"

/**
 * Abstract base for implementing action servers
 * related to vehicle movement.
 *
 * @tparam TAction Action type
 */
template <class TAction>
class AbstractActionServer {

protected:
    rclcpp_action::Server<TAction>::SharedPtr action_server_;
    virtual void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TAction>> goal_handle) = 0;

public:
    AbstractActionServer(const std::string &actionName);
    ~AbstractActionServer() = default;

private:
    virtual rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    virtual rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TAction>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    virtual void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TAction>> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread { std::bind(&AbstractActionServer::execute, this, _1), goal_handle }.detach();
    }

};


template <class TAction>
AbstractActionServer<TAction>::AbstractActionServer(const std::string &actionName) {
    this->action_server_ = rclcpp_action::create_server<TAction>(
        this,
        actionName,
        std::bind(&AbstractActionServer<TAction>::handle_goal, this, _1, _2),
        std::bind(&AbstractActionServer<TAction>::handle_cancel, this, _1),
        std::bind(&AbstractActionServer<TAction>::handle_accepted, this, _1));

}


#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
