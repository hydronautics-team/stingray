#include <OpenCloseActionServer.h>

OpenCloseActionServer::OpenCloseActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<stingray_interfaces::action::OpenCloseAction, stingray_interfaces::action::OpenCloseAction_Goal>(_node, actionName) {

    _node->declare_parameter("open_close_state_topic", "/stingray/topics/devices/updown");
    _node->declare_parameter("set_device_action_srv", "/stingray/services/devices/openclose");

    // ROS service clients
    setDeviceSrvClient = _node->create_client<stingray_core_interfaces::srv::SetDeviceAction>(_node->get_parameter("set_enable_device").as_string());
    // ROS subscribers
    lowLevelSub = _node->create_subscription<stingray_core_interfaces::msg::OpenCloseState>(
        _node->get_parameter("open_close_state_topic").as_string(), 1000,
        std::bind(&OpenCloseActionServer::openCloseStateCallback, this, std::placeholders::_1));
};

void OpenCloseActionServer::openCloseStateCallback(const stingray_core_interfaces::msg::OpenCloseState &msg) {
    current_device = msg.device;
    current_is_opened = msg.is_opened;
};

bool OpenCloseActionServer::isSwitchDone(const std::shared_ptr<const stingray_interfaces::action::OpenCloseAction_Goal> goal) {
    bool device_correct = false;
    bool open_done = false;

    device_correct = (current_device == goal->device);

    open_done = (current_is_opened == goal->open);

    return device_correct && open_done;
};

void OpenCloseActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::OpenCloseAction>> goal_handle) {

    auto setDeviceSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetDeviceAction::Request>();

    if (!setDeviceSrvClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service %s not available!", _node->get_parameter("set_enable_device").as_string().c_str());
        return;
    }

    // get goal data
    const auto goal = goal_handle->get_goal();
    auto goal_result = std::make_shared<stingray_interfaces::action::OpenCloseAction::Result>();
    goal_result->done = false;

    if (isSwitchDone(goal)) {
        goal_result->done = true;
        goal_handle->succeed(goal_result);
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
        return;
    }

    // send service request
    setDeviceSrvRequest->device = goal->device;
    setDeviceSrvRequest->value = goal->open;

    // check if service done
    setDeviceSrvClient->async_send_request(setDeviceSrvRequest).wait();

    rclcpp::Rate checkRate(10ms);
    AsyncTimer timer(goal->timeout * 1000);
    timer.start();

    while (rclcpp::ok()) {
        if (!timer.isBusy()) {
            goal_result->done = false;
            goal_handle->abort(goal_result);
            RCLCPP_ERROR(_node->get_logger(), "Timeout reached while open/close device!");
            return;
        }
        if (isSwitchDone(goal)) {
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

    if (rclcpp::ok()) {
        goal_result->done = true;
        goal_handle->succeed(goal_result);
        RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("updown_device");
    node->declare_parameter("open_close_action", "/stingray/actions/open_close");
    OpenCloseActionServer server = OpenCloseActionServer(node, node->get_parameter("open_close_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};

