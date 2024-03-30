#include <BboxCenteringTwistActionServer.h>

BboxCenteringTwistActionServer::BboxCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<stingray_interfaces::action::BboxCenteringTwistAction, stingray_interfaces::action::BboxCenteringTwistAction_Goal>(_node, actionName) {

    _node->declare_parameter("set_twist_srv", "/stingray/services/set_twist");
    _node->declare_parameter("uv_state_topic", "/stingray/topics/uv_state");
    _node->declare_parameter("bbox_array_topic", "/stingray/topics/camera/bbox_array");
    _node->declare_parameter("target_close_thresh", 1.5);

    target_close_thresh = _node->get_parameter("target_close_thresh").as_double();

    // ROS service clients
    twistSrvClient = _node->create_client<stingray_core_interfaces::srv::SetTwist>(_node->get_parameter("set_twist_srv").as_string());
    // ROS subscribers
    uvStateSub = _node->create_subscription<stingray_core_interfaces::msg::UVState>(
        _node->get_parameter("uv_state_topic").as_string(), 10,
        std::bind(&BboxCenteringTwistActionServer::uvStateCallback, this, std::placeholders::_1));
    bboxArraySub = _node->create_subscription<stingray_interfaces::msg::BboxArray>(
        _node->get_parameter("bbox_array_topic").as_string(), 10,
        std::bind(&BboxCenteringTwistActionServer::bboxArrayCallback, this, std::placeholders::_1));
};

void BboxCenteringTwistActionServer::uvStateCallback(const stingray_core_interfaces::msg::UVState &msg) {
    current_depth = msg.depth;
    current_roll = msg.roll;
    current_pitch = msg.pitch;
    current_yaw = msg.yaw;
    depth_stabilization = msg.depth_stabilization;
    roll_stabilization = msg.roll_stabilization;
    pitch_stabilization = msg.pitch_stabilization;
    yaw_stabilization = msg.yaw_stabilization;
};

void BboxCenteringTwistActionServer::bboxArrayCallback(const stingray_interfaces::msg::BboxArray &msg) {
    if (move_in_progress) {
        bool found_target = false;
        for (auto bbox : msg.bboxes) {
            if (bbox.name == target_name) {
                centering_angle_difference = bbox.horizontal_angle;
                found_target = true;
                target_disappeared_counter = 0;
                target_close = bbox.pos_z < target_close_thresh;
            }
        }
        if (!found_target) {
            target_disappeared_counter++;
        }
    }
};

bool BboxCenteringTwistActionServer::isTwistDone(const std::shared_ptr<const stingray_interfaces::action::BboxCenteringTwistAction_Goal> goal) {
    bool depth_done = false;
    bool roll_done = false;
    bool pitch_done = false;

    target_lost = target_disappeared_counter > 10;

    if (depth_stabilization) {
        auto depth_delta = abs(current_depth - goal->depth);
        depth_done = depth_delta < depth_tolerance;
        if (!depth_done) {
            RCLCPP_ERROR(_node->get_logger(), "Depth not reached %d", depth_delta);
        }
    } else {
        depth_done = true;
    }
    if (roll_stabilization) {
        auto roll_delta = abs(current_roll - goal->roll);
        roll_done = roll_delta < roll_tolerance;
        if (!roll_done) {
            RCLCPP_ERROR(_node->get_logger(), "Roll not reached %d", roll_delta);
        }
    } else {
        roll_done = true;
    }
    if (pitch_stabilization) {
        auto pitch_delta = abs(current_pitch - goal->pitch);
        pitch_done = pitch_delta < pitch_tolerance;
        if (!pitch_done) {
            RCLCPP_ERROR(_node->get_logger(), "Pitch not reached %d", pitch_delta);
        }
    } else {
        pitch_done = true;
    }

    RCLCPP_INFO(_node->get_logger(), "Target big %d", target_close);
    RCLCPP_INFO(_node->get_logger(), "Target disappeared %d", target_disappeared_counter);

    return depth_done && roll_done && pitch_done && target_close;
};

void BboxCenteringTwistActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::BboxCenteringTwistAction>> goal_handle) {

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
    auto goal_result = std::make_shared<stingray_interfaces::action::BboxCenteringTwistAction::Result>();
    goal_result->success = false;

    // // check duration
    // if (goal->duration < 0.0) {
    //     goal_result->success = false;
    //     goal_handle->abort(goal_result);
    //     RCLCPP_ERROR(_node->get_logger(), "Duration value must be greater than 0.0");
    //     return;
    // }

    // send service request
    target_name = goal->bbox_name;
    twistSrvRequest->surge = goal->surge;
    twistSrvRequest->depth = goal->depth;
    twistSrvRequest->roll = goal->roll;
    twistSrvRequest->pitch = goal->pitch;

    rclcpp::Rate checkRate(2s);

    while (rclcpp::ok() && !isTwistDone(goal)) {
        move_in_progress = true;
        RCLCPP_INFO(_node->get_logger(), "Send twist srv request");
        // if (!target_big && target_lost) {
        //     goal_result->success = false;
        //     goal_handle->abort(goal_result);
        //     RCLCPP_ERROR(_node->get_logger(), "Target lost!");
        //     return;
        // }
        RCLCPP_INFO(_node->get_logger(), "Centering angle difference: %f", centering_angle_difference);
        // check if service success
        twistSrvRequest->yaw = current_yaw + centering_angle_difference;
        twistSrvClient->async_send_request(twistSrvRequest).wait();

        if (goal_handle->is_canceling()) {
            goal_result->success = false;
            goal_handle->canceled(goal_result);
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            return;
        }
        // rclcpp::spin_some(_node);
        checkRate.sleep();
    }
    move_in_progress = false;
    centering_angle_difference = 0.0;
    target_name = "";

    RCLCPP_INFO(_node->get_logger(), "Done moving");

    // stop maneuvr service request
    twistSrvRequest->surge = 0.0;
    twistSrvRequest->sway = 0.0;
    if (!depth_stabilization)
        twistSrvRequest->depth = 0.0;
    if (!roll_stabilization)
        twistSrvRequest->roll = 0.0;
    if (!pitch_stabilization)
        twistSrvRequest->pitch = 0.0;
    if (!yaw_stabilization)
        twistSrvRequest->yaw = 0.0;

    RCLCPP_INFO(_node->get_logger(), "Send twist srv request stop");
    twistSrvClient->async_send_request(twistSrvRequest).wait();

    if (rclcpp::ok()) {
        goal_result->success = true;
        goal_handle->succeed(goal_result);
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bbox_centering_twist_action_server");
    node->declare_parameter("bbox_centering_twist_action", "/stingray/actions/bbox_centering_twist");
    BboxCenteringTwistActionServer server = BboxCenteringTwistActionServer(node, node->get_parameter("bbox_centering_twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};