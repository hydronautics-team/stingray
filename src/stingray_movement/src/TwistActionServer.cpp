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

void TwistActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::TwistAction>> goal_handle) {


    auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();

    if (!twistSrvClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service %s not available!", _node->get_parameter("set_twist_srv").as_string());
        return;
    }
    const auto goal = goal_handle->get_goal();
    auto goal_result = std::make_shared<stingray_interfaces::action::TwistAction::Result>();

    // check duration
    if (goal->duration < 0.0) {
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

    auto twistSrvResult = twistSrvClient->async_send_request(twistSrvRequest);

    // check if service done
    auto twistSrvResultCode = rclcpp::spin_until_future_complete(_node, twistSrvResult);
    if (twistSrvResultCode != rclcpp::FutureReturnCode::SUCCESS) {
        goal_handle->abort(goal_result);
        RCLCPP_ERROR(_node->get_logger(), "Unable to set twist");
        return;
    }

    // rclcpp::Rate checkRate(5);
    // AsyncTimer timer(goal->duration);


    // timer.start();
    // bool preempted = false;
    // while (timer.isBusy() && !preempted) {
    //     preempted = actionServer.isPreemptRequested() || !rclcpp::ok();
    //     checkRate.sleep();
    // }

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