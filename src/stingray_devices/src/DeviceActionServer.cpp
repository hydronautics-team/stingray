#include <DeviceActionServer.h>

DeviceActionServer::DeviceActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<stingray_interfaces::action::DeviceAction, stingray_interfaces::action::DeviceAction_Goal>(_node, actionName) {
};

void DeviceActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::DeviceAction>> goal_handle) {

    // get goal data
    const auto goal = goal_handle->get_goal();
    (void)goal;
    auto goal_result = std::make_shared<stingray_interfaces::action::DeviceAction::Result>();
    goal_result->success = true;
    goal_handle->succeed(goal_result);
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

