#include "stingray_movement/TwistActionServer.h"

TwistActionServer::TwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractTwistActionServer<stingray_interfaces::action::TwistAction, stingray_interfaces::action::TwistAction_Goal>(_node, actionName) {};

bool TwistActionServer::isTwistDone(const std::shared_ptr<const stingray_interfaces::action::TwistAction_Goal> goal) {
    
    return isDepthDone(goal->depth) && isRollDone(goal->roll) && isPitchDone(goal->pitch) && isYawDone(target_yaw);
};

void TwistActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::TwistAction>> goal_handle) {

    auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();

    RCLCPP_INFO(_node->get_logger(), "Execute action");
    if (!twistSrvClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service %s not available!", _node->get_parameter("set_twist_srv").as_string().c_str());
    }

    // get goal data
    const auto goal = goal_handle->get_goal();
    auto goal_result = std::make_shared<stingray_interfaces::action::TwistAction::Result>();
    goal_result->success = false;

    // check duration
    if (goal->duration < 0.0) {
        goal_result->success = false;
        goal_handle->abort(goal_result);
        RCLCPP_ERROR(_node->get_logger(), "Duration value must be greater than 0.0");
        return;
    }

    // send service request
    twistSrvRequest->surge = goal->surge;
    twistSrvRequest->sway = goal->sway;
    twistSrvRequest->depth = goal->depth;
    twistSrvRequest->roll = goal->roll;
    twistSrvRequest->pitch = goal->pitch;
    twistSrvRequest->yaw = goal->yaw;
    target_yaw = current_uv_state.yaw + goal->yaw;

    RCLCPP_INFO(_node->get_logger(), "request target_yaw: %f, surge: %f", target_yaw, twistSrvRequest->surge);
    // check if service success
    twistSrvClient->async_send_request(twistSrvRequest).wait();

    rclcpp::Rate checkRate(100ms);
    AsyncTimer timer(goal->duration * 1000);
    timer.start();

    while (rclcpp::ok()) {
        if (!timer.isBusy() && isTwistDone(goal)) {
            break;
        }
        if (goal_handle->is_canceling()) {
            goal_result->success = false;
            goal_handle->canceled(goal_result);
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            return;
        }
        // rclcpp::spin_some(_node);
        checkRate.sleep();
    }

    // stop maneuvr service request
    stopTwist();
    
    if (rclcpp::ok()) {
        goal_result->success = true;
        goal_handle->succeed(goal_result);
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("twist_action_server");
    node->declare_parameter("twist_action", "/stingray/actions/twist");
    TwistActionServer server = TwistActionServer(node, node->get_parameter("twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};