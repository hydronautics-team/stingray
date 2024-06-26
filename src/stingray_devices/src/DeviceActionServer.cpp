#include <DeviceActionServer.h>

DeviceActionServer::DeviceActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<stingray_interfaces::action::DeviceAction, stingray_interfaces::action::DeviceAction_Goal>(_node, actionName) {

    _node->declare_parameter("device_state_array_topic", "/stingray/topics/device_state_array");
    _node->declare_parameter("set_device_srv", "/stingray/services/set_device");

    // ROS service clients
    setDeviceSrvClient = _node->create_client<stingray_core_interfaces::srv::SetDevice>(_node->get_parameter("set_device_srv").as_string());
    // ROS subscribers
    deviceStateArraySub = _node->create_subscription<stingray_core_interfaces::msg::DeviceStateArray>(
        _node->get_parameter("device_state_array_topic").as_string(), 1000,
        std::bind(&DeviceActionServer::deviceStateCallback, this, std::placeholders::_1));
};

void DeviceActionServer::deviceStateCallback(const stingray_core_interfaces::msg::DeviceStateArray &msg) {
    currentDeviceStates.clear();
    for (auto &device : msg.states) {
        currentDeviceStates.push_back(device);
    }
};

bool DeviceActionServer::isSwitchDone(const std::shared_ptr<const stingray_interfaces::action::DeviceAction_Goal> goal) {
    bool value_correct = false;
    if (currentDeviceStates.empty()) {
        RCLCPP_ERROR(_node->get_logger(), "No device states");
        return false;
    }
    if (currentDeviceStates.size() < goal->device + 1) {
        RCLCPP_ERROR(_node->get_logger(), "Device not found. Device states size: %d", currentDeviceStates.size());
        return false;
    }
    value_correct = (currentDeviceStates[goal->device].value == goal->value);
    return value_correct;
};

void DeviceActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::DeviceAction>> goal_handle) {

    auto setDeviceSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetDevice::Request>();

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
    auto goal_result = std::make_shared<stingray_interfaces::action::DeviceAction::Result>();
    goal_result->success = false;

    // if (isSwitchDone(goal)) {
    //     goal_result->success = true;
    //     goal_handle->succeed(goal_result);
    //     RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    //     return;
    // }

    // send service request
    setDeviceSrvRequest->device = goal->device;
    setDeviceSrvRequest->value = goal->value;

    // check if service done
    setDeviceSrvClient->async_send_request(setDeviceSrvRequest).wait();

    // rclcpp::Rate checkRate(10ms);
    // AsyncTimer timer(goal->timeout * 1000);
    // timer.start();

    goal_result->success = true;
    goal_handle->succeed(goal_result);

    // while (rclcpp::ok()) {
    //     if (!timer.isBusy()) {
    //         RCLCPP_ERROR(_node->get_logger(), "Timeout reached during device action!");
    //         goal_result->success = false;
    //         goal_handle->abort(goal_result);
    //         return;
    //     }
    //     // if (isSwitchDone(goal)) {
    //     //     RCLCPP_INFO(_node->get_logger(), "Goal succeeded");
    //     //     goal_result->success = true;
    //     //     goal_handle->succeed(goal_result);
    //     //     return;
    //     // }

    //     if (goal_handle->is_canceling()) {
    //         RCLCPP_INFO(_node->get_logger(), "Goal canceled");
    //         goal_result->success = false;
    //         goal_handle->canceled(goal_result);
    //         return;
    //     }
    //     // rclcpp::spin_some(_node);
    //     checkRate.sleep();
    // }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("device_action_server");
    node->declare_parameter("device_action", "/stingray/actions/device");
    DeviceActionServer server = DeviceActionServer(node, node->get_parameter("device_action").as_string());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};

