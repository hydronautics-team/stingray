#include "stingray_movement/HydroacousticCenteringTwistActionServer.h"

HydroacousticCenteringTwistActionServer::HydroacousticCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractCenteringTwistActionServer<stingray_interfaces::action::HydroacousticCenteringTwistAction, stingray_interfaces::action::HydroacousticCenteringTwistAction_Goal>(_node, actionName) {
    current_target_bbox.pos_x = 1000.0;
    current_target_bbox.pos_y = 1000.0;
    current_target_bbox.pos_z = 1000.0;

    current_avoid_target_bbox.pos_x = 1000.0;
    current_avoid_target_bbox.pos_y = 1000.0;
    current_avoid_target_bbox.pos_z = 1000.0;
};


void HydroacousticCenteringTwistActionServer::bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg) {
    bool found_target = false;
    for (auto bbox : msg.bboxes) {
        if (std::find(target_avoid_bbox_name_array.begin(), target_avoid_bbox_name_array.end(), bbox.name) != target_avoid_bbox_name_array.end()) {
            if (bbox.pos_z < current_avoid_target_bbox.pos_z) {
                current_avoid_target_bbox = bbox;
            }
        }

        if (bbox.name == target_bbox_name) {
            current_target_bbox = bbox;
            found_target = true;
            target_disappeared_counter = 0;
        }
    }
    if (!found_target) {
        target_disappeared_counter++;
    }
};

void HydroacousticCenteringTwistActionServer::hydroacousticCallback(const std_msgs::msg::Float64 &msg) {

        if (bbox.name == target_bbox_name) {
            current_target_bbox = *msg;
            found_target = true;
            target_disappeared_counter = 0;
        }
    }
    if (!found_target) {
        target_disappeared_counter++;
    }
};

bool HydroacousticCenteringTwistActionServer::isTwistDone(const std::shared_ptr<const stingray_interfaces::action::BboxCenteringTwistAction_Goal> goal) {
    return isDepthDone(goal->depth) && isRollDone(goal->roll) && isPitchDone(goal->pitch);
};

bool HydroacousticCenteringTwistActionServer::isCenteringTwistDone() {
    return current_target_bbox.pos_z < target_distance_threshold;
};

bool HydroacousticCenteringTwistActionServer::isTargetLost() {
    return target_disappeared_counter > target_lost_thresh;
};

void HydroacousticCenteringTwistActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxCenteringTwistAction>> goal_handle) {

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
    auto goal_result = std::make_shared<stingray_interfaces::action::HydroacousticCenteringTwistAction::Result>();
    goal_result->success = false;

    bboxArraySub = _node->create_subscription<stingray_interfaces::msg::BboxArray>(
        goal->bbox_topic, 10,
        std::bind(&HydroacousticCenteringTwistActionServer::bboxArrayCallback, this, std::placeholders::_1));

    AngleHydroacousticSub = _node->create_subscription<std_msgs::msg::Float64>(goal->hydroacoustic_topic, 10,
            std::bind(&HydroacousticCenteringTwistActionServer::hydroacousticCallback, this, std::placeholders::_1));

    // check duration
    if (goal->duration < 0.0) {
        goal_result->success = false;
        goal_handle->abort(goal_result);
        RCLCPP_ERROR(_node->get_logger(), "Duration value must be greater than 0.0");
        return;
    }

    // send service request
    target_bbox_name = goal->bbox_name;
    target_avoid_bbox_name_array = goal->avoid_bbox_name_array;
    target_distance_threshold = goal->distance_threshold;
    target_lost_thresh = goal->lost_threshold;
    twistSrvRequest->surge = goal->surge;
    twistSrvRequest->depth = goal->depth;
    twistSrvRequest->roll = goal->roll;
    twistSrvRequest->pitch = goal->pitch;

    rclcpp::Rate checkRate(goal->centering_rate);
    AsyncTimer timer(goal->duration * 1000);
    timer.start();

    while (rclcpp::ok()) {
        if (isTargetLost()) {
            RCLCPP_ERROR(_node->get_logger(), "Target lost!");
        }

        if (!timer.isBusy() && isTargetLost()) {
            RCLCPP_ERROR(_node->get_logger(), "Twist done by duration %f, target lost!", goal->duration);
            break;
        }

        if (isTwistDone(goal) && isCenteringTwistDone()) {
            RCLCPP_INFO(_node->get_logger(), "Twist done, target distance: %f, closer than: %f", current_target_bbox.pos_z, target_distance_threshold);
            break;
        }
        if (current_avoid_target_bbox.pos_z < goal->avoid_distance_threshold && abs(current_avoid_target_bbox.pos_x) < goal->avoid_horizontal_threshold) {
            if (current_avoid_target_bbox.pos_x < 0.0) {
                twistSrvRequest->sway = - goal->sway;
            } else {
                twistSrvRequest->sway = goal->sway;
            }
        } 
        twistSrvRequest->yaw = current_target_angle;
        RCLCPP_INFO(_node->get_logger(), "Twist action current yaw: %f, request diff: %f, surge: %f", current_uv_state.yaw, twistSrvRequest->yaw, twistSrvRequest->surge);
        // check if service success
        twistSrvClient->async_send_request(twistSrvRequest).wait();
        current_target_angle = 0.0;
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
    current_target_angle = 0.0;
    target_disappeared_counter = 0;
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
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hydronautics_centering_twist_action_server");
    node->declare_parameter("bbox_centering_twist_action", "/stingray/actions/bbox_centering_twist");
    HydroacousticCenteringTwistActionServer server = HydroacousticCenteringTwistActionServer(node, node->get_parameter("hydronautics_centering_twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};