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
template <class TAction, class TActionGoal>
class AbstractActionServer {

protected:
    typename rclcpp_action::Server<TAction>::SharedPtr _action_server;
    std::shared_ptr<rclcpp::Node> _node;
    virtual void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TAction>> goal_handle) = 0;

public:
    AbstractActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : _node(_node) {
        _action_server = rclcpp_action::create_server<TAction>(
            _node,
            actionName,
            std::bind(&AbstractActionServer<TAction, TActionGoal>::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AbstractActionServer<TAction, TActionGoal>::handle_cancel, this, std::placeholders::_1),
            std::bind(&AbstractActionServer<TAction, TActionGoal>::handle_accepted, this, std::placeholders::_1));
    }
    ~AbstractActionServer() = default;

private:
    virtual rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const TActionGoal> goal) {
        RCLCPP_INFO(_node->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    virtual rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<TAction>> goal_handle) {
        RCLCPP_INFO(_node->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    virtual void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TAction>> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread { std::bind(&AbstractActionServer::execute, this, _1), goal_handle }.detach();
    }

};


#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
