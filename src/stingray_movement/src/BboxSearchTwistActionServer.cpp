#include "stingray_movement/BboxSearchTwistActionServer.h"

BboxSearchTwistActionServer::BboxSearchTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractSearchTwistActionServer<stingray_interfaces::action::BboxSearchTwistAction, stingray_interfaces::action::BboxSearchTwistAction_Goal>(_node, actionName) {};

void BboxSearchTwistActionServer::bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg) {
    bool found_target = false;
    for (auto bbox : msg.bboxes) {
        if (bbox.name == target_bbox_name) {
            target_found_counter++;
            if (target_found_counter > target_found_threshold) {
                found_target_yaw = bbox.horizontal_angle;
            }
        }
    }
    if (!found_target) {
        target_found_counter = 0;
    }
};

bool BboxSearchTwistActionServer::isTwistDone(const std::shared_ptr<const stingray_interfaces::action::BboxSearchTwistAction_Goal> goal) {

    return isDepthDone(goal->depth) && isRollDone(goal->roll) && isPitchDone(goal->pitch);
};

bool BboxSearchTwistActionServer::isSearchTwistDone() {
    return target_found_counter > target_found_threshold;
};

void BboxSearchTwistActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxSearchTwistAction>> goal_handle) {

    auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();

    RCLCPP_INFO(_node->get_logger(), "Execute action");
    while (!twistSrvClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service %s not available!", _node->get_parameter("set_twist_srv").as_string().c_str());
        // return;
    }

    // get goal data
    const auto goal = goal_handle->get_goal();
    auto goal_result = std::make_shared<stingray_interfaces::action::BboxSearchTwistAction::Result>();
    goal_result->success = false;

    bboxArraySub = _node->create_subscription<stingray_interfaces::msg::BboxArray>(
        goal->bbox_topic, 10,
        std::bind(&BboxSearchTwistActionServer::bboxArrayCallback, this, std::placeholders::_1));

    // check duration
    if (goal->duration < 0.0) {
        goal_result->success = false;
        goal_handle->abort(goal_result);
        RCLCPP_ERROR(_node->get_logger(), "Duration value must be greater than 0.0");
        return;
    }

    // send service request
    target_bbox_name = goal->bbox_name;
    target_found_threshold = goal->found_threshold;
    twistSrvRequest->depth = goal->depth;
    twistSrvRequest->roll = goal->roll;
    twistSrvRequest->pitch = goal->pitch;

    if (goal->first_clockwise) {
        target_yaw_step = goal->yaw_step;
    } else {
        target_yaw_step = - goal->yaw_step;
    }
    rclcpp::Rate checkRate(goal->search_rate * 1000);
    AsyncTimer timer(goal->duration * 1000);
    timer.start();

    while (rclcpp::ok()) {
        if (!timer.isBusy()) {
            RCLCPP_ERROR(_node->get_logger(), "Twist done by duration %f, target not found!", goal->duration);
            break;
        }

        if (isTwistDone(goal) && isSearchTwistDone()) {
            RCLCPP_INFO(_node->get_logger(), "Twist done, target found, target yaw: %f", found_target_yaw);
            twistSrvRequest->yaw = found_target_yaw;
            twistSrvClient->async_send_request(twistSrvRequest).wait();
            break;
        }
        twistSrvRequest->yaw += target_yaw_step;
        if (abs(twistSrvRequest->yaw) > goal->max_yaw) {
            break;
        }
        RCLCPP_INFO(_node->get_logger(), "Twist action current yaw: %f, request diff: %f, surge: %f", current_uv_state.yaw, twistSrvRequest->yaw, twistSrvRequest->surge);
        // check if service success
        twistSrvClient->async_send_request(twistSrvRequest).wait();
        twistSrvRequest->yaw = 0.0;

        if (goal_handle->is_canceling()) {
            goal_result->success = false;
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            goal_handle->canceled(goal_result);
            return;
        }
        // rclcpp::spin_some(_node);
        checkRate.sleep();
    }

    while (rclcpp::ok()) {
        if (!timer.isBusy()) {
            RCLCPP_ERROR(_node->get_logger(), "Twist done by duration %f, target not found!", goal->duration);
            break;
        }

        if (isTwistDone(goal) && isSearchTwistDone()) {
            RCLCPP_INFO(_node->get_logger(), "Twist done, target found, target yaw: %f", found_target_yaw);
            twistSrvRequest->yaw = found_target_yaw;
            twistSrvClient->async_send_request(twistSrvRequest).wait();
            break;
        }
        twistSrvRequest->yaw -= target_yaw_step;
        if (abs(twistSrvRequest->yaw) < goal->max_yaw) {
            break;
        }
        RCLCPP_INFO(_node->get_logger(), "Twist action current yaw: %f, request diff: %f, surge: %f", current_uv_state.yaw, twistSrvRequest->yaw, twistSrvRequest->surge);
        // check if service success
        twistSrvClient->async_send_request(twistSrvRequest).wait();
        twistSrvRequest->yaw = 0.0;

        if (goal_handle->is_canceling()) {
            goal_result->success = false;
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            goal_handle->canceled(goal_result);
            return;
        }
        // rclcpp::spin_some(_node);
        checkRate.sleep();
    }

    target_yaw_step = 0.0;
    target_found_threshold = 0;
    target_found_counter = 0;
    target_bbox_name = "";
    bboxArraySub.reset();

    RCLCPP_INFO(_node->get_logger(), "Done moving");

    // stop maneuvr service request
    stopTwist();

    if (rclcpp::ok()) {
        goal_result->success = true;
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
        goal_handle->succeed(goal_result);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bbox_search_twist_action_server");
    node->declare_parameter("bbox_search_twist_action", "/stingray/actions/bbox_search_twist");
    BboxSearchTwistActionServer server = BboxSearchTwistActionServer(node, node->get_parameter("bbox_search_twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};