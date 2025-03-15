#include "stingray_movement/BboxBottomCenteringTwistActionServer.h"

BboxBottomCenteringTwistActionServer::BboxBottomCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractBottomCenteringTwistActionServer<stingray_interfaces::action::BboxBottomCenteringTwistAction, stingray_interfaces::action::BboxBottomCenteringTwistAction_Goal>(_node, actionName) {
    current_target_bbox.pos_x = 1000.0;
    current_target_bbox.pos_y = 1000.0;
    current_target_bbox.pos_z = 1000.0;
    current_target_bbox.horizontal_angle = 1000.0;
    current_target_bbox.vertical_angle = 1000.0;
    current_avoid_target_bbox.pos_x = 1000.0;
    current_avoid_target_bbox.pos_y = 1000.0;
    current_avoid_target_bbox.pos_z = 1000.0;
};

void BboxBottomCenteringTwistActionServer::bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg) {
    bool found_target = false;
    bool found_avoid_target = false;
    current_avoid_target_bbox.pos_x = 1000.0;
    current_avoid_target_bbox.pos_y = 1000.0;
    current_avoid_target_bbox.pos_z = 1000.0;
    //current_target_bbox.horizontal_angle = 1000.0;
    //current_target_bbox.vertical_angle = 1000.0;

    for (auto bbox : msg.bboxes) {
        // RCLCPP_INFO(_node->get_logger(), "\nName x: %s", bbox.name.c_str());
        // RCLCPP_INFO(_node->get_logger(), "Avoid x: %f, y: %f, z: %f", bbox.pos_x, bbox.pos_y, bbox.pos_z);
        // RCLCPP_INFO(_node->get_logger(), "Current Avoid x: %f, y: %f, z: %f", current_avoid_target_bbox.pos_x, current_avoid_target_bbox.pos_y, current_avoid_target_bbox.pos_z);
        
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

bool BboxBottomCenteringTwistActionServer::isTwistDone(const std::shared_ptr<const stingray_interfaces::action::BboxBottomCenteringTwistAction_Goal> goal) {
    return isDepthDone(goal->depth) && isRollDone(goal->roll) && isPitchDone(goal->pitch);
};

bool BboxBottomCenteringTwistActionServer::isCenteringTwistDone() {
    return abs(current_target_bbox.horizontal_angle) < target_threshold_x && abs(current_target_bbox.vertical_angle) < target_threshold_y;
};

bool BboxBottomCenteringTwistActionServer::isTargetLost() {
    return target_disappeared_counter > target_lost_thresh;
};

void BboxBottomCenteringTwistActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxBottomCenteringTwistAction>> goal_handle) {

    auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();

    RCLCPP_INFO(_node->get_logger(), "Execute action");
    while (!twistSrvClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service %s not available!", _node->get_parameter("set_twist_srv").as_string().c_str());
        return;
    }

    // get goal data
    const auto goal = goal_handle->get_goal();
    auto goal_result = std::make_shared<stingray_interfaces::action::BboxBottomCenteringTwistAction::Result>();
    goal_result->success = false;

    // check duration
    if (goal->duration < 0.0) {
        goal_result->success = false;
        goal_handle->abort(goal_result);
        RCLCPP_ERROR(_node->get_logger(), "Duration value must be greater than 0.0");
        return;
    }

    bboxArraySub = _node->create_subscription<stingray_interfaces::msg::BboxArray>(
        goal->bbox_topic, 10,
        std::bind(&BboxBottomCenteringTwistActionServer::bboxArrayCallback, this, std::placeholders::_1));

    // send service request
    target_bbox_name = goal->bbox_name;
    //target_distance_threshold = goal->distance_threshold;
    target_lost_thresh = goal->lost_threshold;
    twistSrvRequest->surge = goal->surge;
    twistSrvRequest->sway = goal->sway;
    twistSrvRequest->depth = goal->depth;
    twistSrvRequest->roll = goal->roll;
    target_threshold_x = goal->threshold_x;
    target_threshold_y = goal->threshold_y;
    float e = 2.71828;

    rclcpp::Rate checkRate(goal->centering_rate);
    AsyncTimer timer(goal->duration * 1000);
    timer.start();

    while (rclcpp::ok()) {
        if (isTargetLost()) {
            RCLCPP_ERROR(_node->get_logger(), "Target lost!");
            goal_result->success = false;
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            
            target_disappeared_counter = 0;
            current_target_bbox.pos_x = 1000.0;
            current_target_bbox.pos_y = 1000.0;
            current_target_bbox.pos_z = 1000.0;
            current_target_bbox.horizontal_angle = 1000.0;
            current_target_bbox.vertical_angle = 1000.0;
            current_avoid_target_bbox.pos_x = 1000.0;
            current_avoid_target_bbox.pos_y = 1000.0;
            current_avoid_target_bbox.pos_z = 1000.0;
            target_bbox_name = "";
            bboxArraySub.reset();
            
            // stop maneuvr service request
            stopTwist(twistSrvRequest);
            goal_handle->succeed(goal_result);
            return;
        }

        if (!timer.isBusy() && isTargetLost()) {
            RCLCPP_ERROR(_node->get_logger(), "Twist done by duration %f, target lost!", goal->duration);
            goal_result->success = false;
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            
            target_disappeared_counter = 0;
            current_target_bbox.pos_x = 1000.0;
            current_target_bbox.pos_y = 1000.0;
            current_target_bbox.pos_z = 1000.0;
            current_target_bbox.horizontal_angle = 1000.0;
            current_target_bbox.vertical_angle = 1000.0;
            current_avoid_target_bbox.pos_x = 1000.0;
            current_avoid_target_bbox.pos_y = 1000.0;
            current_avoid_target_bbox.pos_z = 1000.0;
            target_bbox_name = "";
            bboxArraySub.reset();
            
            // stop maneuvr service request
            stopTwist(twistSrvRequest);
            goal_handle->succeed(goal_result);
            return;
        }

        if (isCenteringTwistDone()) {
            RCLCPP_INFO(_node->get_logger(), "Twist done OK");
            break;
        }
        
        twistSrvRequest->sway = 0;

        float distance_surge = 8;
        float new_speed_surge = fmin((pow(e, abs(current_target_bbox.vertical_angle)) / pow(e, distance_surge)) * abs(goal->surge), abs(goal->surge));
        if (current_target_bbox.vertical_angle < -target_threshold_y) {
            twistSrvRequest->surge = new_speed_surge;
        }
        else if (current_target_bbox.vertical_angle > target_threshold_y) {
            twistSrvRequest->surge = -new_speed_surge;
        }

        float distance_sway = 8;
        float new_speed_sway = fmin((pow(e, abs(current_target_bbox.horizontal_angle)) / pow(e, distance_sway)) * abs(goal->sway), abs(goal->sway));
        if (current_target_bbox.horizontal_angle < -target_threshold_x) {
            twistSrvRequest->sway = -new_speed_sway;
        }
        else if (current_target_bbox.horizontal_angle > target_threshold_x) {
            twistSrvRequest->sway = new_speed_sway;
        }


        RCLCPP_INFO(_node->get_logger(), "Current (x, y) to center: (%f, %f)", current_target_bbox.vertical_angle, current_target_bbox.horizontal_angle);
        RCLCPP_INFO(_node->get_logger(), "Current (sway, surge): (%f, %f)", twistSrvRequest->sway, twistSrvRequest->surge);
        // check if service success
        twistSrvClient->async_send_request(twistSrvRequest).wait();

        if (goal_handle->is_canceling()) {
            goal_result->success = false;
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            goal_handle->canceled(goal_result);

            target_disappeared_counter = 0;
            current_target_bbox.pos_x = 1000.0;
            current_target_bbox.pos_y = 1000.0;
            current_target_bbox.pos_z = 1000.0;
            current_target_bbox.horizontal_angle = 0.0;
            current_avoid_target_bbox.pos_x = 1000.0;
            current_avoid_target_bbox.pos_y = 1000.0;
            current_avoid_target_bbox.pos_z = 1000.0;
            target_bbox_name = "";
            bboxArraySub.reset();

            // stop maneuvr service request
            stopTwist(twistSrvRequest);
            return;
        }
        // rclcpp::spin_some(_node);
        checkRate.sleep();
    }
    target_disappeared_counter = 0;
    current_target_bbox.pos_x = 1000.0;
    current_target_bbox.pos_y = 1000.0;
    current_target_bbox.pos_z = 1000.0;
    current_target_bbox.horizontal_angle = 0.0;
    current_avoid_target_bbox.pos_x = 1000.0;
    current_avoid_target_bbox.pos_y = 1000.0;
    current_avoid_target_bbox.pos_z = 1000.0;
    target_bbox_name = "";
    bboxArraySub.reset();

    RCLCPP_INFO(_node->get_logger(), "Done moving");

    // stop maneuvr service request
    stopTwist(twistSrvRequest);

    if (rclcpp::ok()) {
        goal_result->success = true;
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
        goal_handle->succeed(goal_result);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bbox_bottom_centering_twist_action_server");
    node->declare_parameter("bbox_bottom_centering_twist_action", "/stingray/actions/bbox_bottom_centering_twist");
    BboxBottomCenteringTwistActionServer server = BboxBottomCenteringTwistActionServer(node, node->get_parameter("bbox_bottom_centering_twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};