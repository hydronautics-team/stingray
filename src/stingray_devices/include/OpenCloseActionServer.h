#ifndef STINGRAY_OPEN_CLOSE_ACTION_SERVER_H
#define STINGRAY_OPEN_CLOSE_ACTION_SERVER_H


#include <stingray_utils/AbstractActionServer.h>
#include <stingray_utils/AsyncTimer.h>

#include <stingray_core_interfaces/srv/set_device_action.hpp>
#include <stingray_core_interfaces/msg/device_state.hpp>
#include <stingray_interfaces/action/up_down_action.hpp>

using namespace std::chrono_literals;


class OpenCloseActionServer : AbstractActionServer<stingray_interfaces::action::OpenCloseAction, stingray_interfaces::action::OpenCloseAction_Goal>{

protected:

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::OpenCloseAction>> goal_handle) override;

public:

    OpenCloseActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~OpenCloseActionServer() = default;

private:
    void deviceStateCallback(const stingray_core_interfaces::msg::DeviceState &msg);
    bool isSwitchDone(const std::shared_ptr<const stingray_interfaces::action::OpenCloseAction_Goal> goal);
    rclcpp::Subscription<stingray_core_interfaces::msg::DeviceState>::SharedPtr lowLevelSub;
    rclcpp::Client<stingray_core_interfaces::srv::SetDeviceAction>::SharedPtr stingray_comClient;
    
    int current_device;
    int current_velocity;
    bool current_opened;
};


#endif //STINGRAY_OPEN_CLOSE_ACTION_SERVER_H

