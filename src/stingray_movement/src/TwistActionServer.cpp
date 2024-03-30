#include <TwistActionServer.h>

TwistActionServer::TwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<stingray_interfaces::action::TwistAction, stingray_interfaces::action::TwistAction_Goal>(_node, actionName) {

    _node->declare_parameter("uv_state_topic", "/stingray/topics/uv_state");
    _node->declare_parameter("set_twist_srv", "/stingray/services/set_twist");

    // ROS service clients
    twistSrvClient = _node->create_client<stingray_core_interfaces::srv::SetTwist>(_node->get_parameter("set_twist_srv").as_string());
    // ROS subscribers
    uvStateSub = _node->create_subscription<stingray_core_interfaces::msg::UVState>(
        _node->get_parameter("uv_state_topic").as_string(), 1000,
        std::bind(&TwistActionServer::uvStateCallback, this, std::placeholders::_1));
};

void TwistActionServer::uvStateCallback(const stingray_core_interfaces::msg::UVState &msg) {
    current_depth = msg.depth;
    current_roll = msg.roll;
    current_pitch = msg.pitch;
    current_yaw = msg.yaw;
    depth_stabilization = msg.depth_stabilization;
    roll_stabilization = msg.roll_stabilization;
    pitch_stabilization = msg.pitch_stabilization;
    yaw_stabilization = msg.yaw_stabilization;
};

bool TwistActionServer::isTwistDone(const std::shared_ptr<const stingray_interfaces::action::TwistAction_Goal> goal) {
    bool depth_done = false;
    bool roll_done = false;
    bool pitch_done = false;
    bool yaw_done = false;

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
    if (yaw_stabilization) {
        auto yaw_delta = abs(target_yaw - goal->yaw);
        yaw_done = yaw_delta < yaw_tolerance;
        if (!yaw_done) {
            RCLCPP_ERROR(_node->get_logger(), "Yaw not reached %d", yaw_delta);
        }
    } else {
        yaw_done = true;
    }
    return depth_done && roll_done && pitch_done && yaw_done;
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
        return;
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
    target_yaw = current_yaw + goal->yaw;

    RCLCPP_INFO(_node->get_logger(), "Send twist srv request");
    // check if service success
    twistSrvClient->async_send_request(twistSrvRequest).wait();

    rclcpp::Rate checkRate(10ms);
    AsyncTimer timer(goal->duration * 1000);
    timer.start();

    while (rclcpp::ok()) {
        // RCLCPP_INFO(_node->get_logger(), "isTwistDone %d", isTwistDone(goal));
        // RCLCPP_INFO(_node->get_logger(), "timer.isBusy %d", timer.isBusy());
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
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("twist_action_server");
    node->declare_parameter("twist_action", "/stingray/actions/twist");
    TwistActionServer server = TwistActionServer(node, node->get_parameter("twist_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};