#include "../include/EnableDeviceNode.h"

UpDownServer::UpDownServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<stingray_interfaces::action::UpDownAction, stingray_interfaces::action::UpDownAction_Goal>(_node, actionName) {

    _node->declare_parameter("device_enable_topic", "/stingray/topics/devices/updown");
    _node->declare_parameter("set_enable_device", "/stingray/services/devices/updown");

    // ROS service clients
    stingray_comClient = _node->create_client<stingray_core_interfaces::srv::SetDeviceAction>(_node->get_parameter("set_enable_device").as_string());
    // ROS subscribers
    lowLevelSub = _node->create_subscription<stingray_core_interfaces::msg::DeviceState>(
        _node->get_parameter("device_enable_topic").as_string(), 1000,
        std::bind(&UpDownServer::deviceStateCallback, this, std::placeholders::_1));
};

void UpDownServer::deviceStateCallback(const stingray_core_interfaces::msg::DeviceState &msg) {
    current_device = msg.device;
    current_velocity = msg.velocity;
    current_opened = msg.opened;
};

bool UpDownServer::isSwitchDone(const std::shared_ptr<const stingray_interfaces::action::UpDownAction_Goal> goal) {
    bool device_correct = false;
    bool velocity_done = false;
    bool open_done = false;

    device_correct = (current_device == goal->device);

    auto velocity_delta = abs(current_velocity - goal->velocity);
    velocity_done = velocity_delta < velocity_tolerance;

    open_done = (current_opened == goal->opened);
    
    return device_correct && velocity_done && open_done;
};

void UpDownServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::UpDownAction>> goal_handle) {

    auto stingray_comRequest = std::make_shared<stingray_core_interfaces::srv::SetDeviceAction::Request>();

    if (!stingray_comClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service %s not available!", _node->get_parameter("set_enable_device").as_string().c_str());
        return;
    }

    // get goal data
    const auto goal = goal_handle->get_goal();
    auto goal_result = std::make_shared<stingray_interfaces::action::UpDownAction::Result>();
    goal_result->done = false;

    // check duration
    if (goal->duration < 0.0) {
        goal_result->done = false;
        goal_handle->abort(goal_result);
        RCLCPP_ERROR(_node->get_logger(), "Duration value must be greater than 0.0");
        return;
    }

    // send service request
    stingray_comRequest->device = goal->device;
    stingray_comRequest->velocity = goal->velocity;
    stingray_comRequest->opened = goal->opened;

    // check if service done
    stingray_comClient->async_send_request(stingray_comRequest).wait();

    rclcpp::Rate checkRate(10ms);
    AsyncTimer timer(goal->pause_common * 1000);
    timer.start();

    while (rclcpp::ok()) {
        if (!timer.isBusy() && isSwitchDone(goal)) {
            break;
        }
        if (goal_handle->is_canceling()) {
            goal_result->done = false;
            goal_handle->canceled(goal_result);
            RCLCPP_INFO(_node->get_logger(), "Goal canceled");
            return;
        }
        // rclcpp::spin_some(_node);
        checkRate.sleep();
    }

    if(goal->pause_optional > 0){
        RCLCPP_INFO("Keeping lifter down for %f ms", goal->pause_optional);
        stingray_comRequest->velocity = 0.0;
        stingray_comClient->async_send_request(stingray_comRequest).wait();
        AsyncTimer timer_opt(goal->pause_optional * 1000);
        timer_opt.start();

        while (rclcpp::ok()) {
            if (!timer_opt.isBusy()) {
                break;
            }
            if (goal_handle->is_canceling()) {
                goal_result->done = false;
                goal_handle->canceled(goal_result);
                RCLCPP_INFO(_node->get_logger(), "Goal canceled");
                return;
            }
            // rclcpp::spin_some(_node);
            checkRate.sleep();
        }
    }

    // stop  service request
    stingray_comRequest->velocity = 0.0;

    stingray_comClient->async_send_request(stingray_comRequest).wait();

    if (rclcpp::ok()) {
        goal_result->done = true;
        goal_handle->succeed(goal_result);
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("updown_device");
    node->declare_parameter("updown_action", "/stingray/actions/devices/updown");
    UpDownServer server = UpDownServer(node, node->get_parameter("updown_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};

