#ifndef STINGRAY_DEVICE_ACTION_SERVER_H
#define STINGRAY_DEVICE_ACTION_SERVER_H


#include <stingray_utils/AbstractActionServer.h>
#include <stingray_utils/AsyncTimer.h>

#include <stingray_core_interfaces/srv/set_device.hpp>
#include <stingray_core_interfaces/msg/device_state.hpp>
#include <stingray_core_interfaces/msg/device_state_array.hpp>
#include <stingray_interfaces/action/device_action.hpp>

using namespace std::chrono_literals;


class DeviceActionServer : AbstractActionServer<stingray_interfaces::action::DeviceAction, stingray_interfaces::action::DeviceAction_Goal> {

protected:

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::DeviceAction>> goal_handle) override;

public:

    DeviceActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~DeviceActionServer() = default;

private:
    void deviceStateCallback(const stingray_core_interfaces::msg::DeviceStateArray &msg);
    bool isSwitchDone(const std::shared_ptr<const stingray_interfaces::action::DeviceAction_Goal> goal);
    rclcpp::Subscription<stingray_core_interfaces::msg::DeviceStateArray>::SharedPtr deviceStateArraySub;
    rclcpp::Client<stingray_core_interfaces::srv::SetDevice>::SharedPtr setDeviceSrvClient;

    std::vector<stingray_core_interfaces::msg::DeviceState> currentDeviceStates;
};


#endif //STINGRAY_DEVICE_ACTION_SERVER_H

