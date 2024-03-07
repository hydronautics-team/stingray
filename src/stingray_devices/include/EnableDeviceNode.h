#ifndef PROJECT_UPDOWNSERVER_H
#define PROJECT_UPDOWNSERVER_H

#include <stingray_interfaces/action/up_down_action.hpp>

#include "../../stingray_movement/include/AbstractActionServer.h"
#include "stingray_core_interfaces/srv/set_device_action.hpp"
#include "stingray_core_interfaces/msg/device_state.hpp"
#include "../../stingray_movement/include/AsyncTimer.h"

//#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;


class UpDownServer : AbstractActionServer<stingray_interfaces::action::UpDownAction, stingray_interfaces::action::UpDownAction_Goal>{

protected:

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<stingray_interfaces::action::UpDownAction>> goal_handle) override;

public:

    UpDownServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName);
    ~UpDownServer() = default;

private:
    void deviceStateCallback(const stingray_core_interfaces::msg::DeviceState &msg);
    bool isSwitchDone(const std::shared_ptr<const stingray_interfaces::action::UpDownAction_Goal> goal);
    rclcpp::Subscription<stingray_core_interfaces::msg::DeviceState>::SharedPtr lowLevelSub;
    rclcpp::Client<stingray_core_interfaces::srv::SetDeviceAction>::SharedPtr stingray_comClient;

    float velocity_tolerance = 0.2;
    
    int current_device;
    int current_velocity;
    bool current_opened;
};
/*
private:

    std::string deviceActionService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<stingray_devices_msgs::UpDownAction> actionServer;

    void goalCallback(const stingray_devices_msgs::UpDownGoalConstPtr &goal);

public:

    UpDownServer(const std::string& actionName, const std::string& deviceActionService);
    ~UpDownServer() = default;
};
*/

#endif //PROJECT_UPDOWNSERVER_H

