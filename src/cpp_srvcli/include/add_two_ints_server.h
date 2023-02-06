#ifndef COMMUNICATION_HARDWARE_BRIDGE_H
#define COMMUNICATION_HARDWARE_BRIDGE_H

#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <memory>

class AddTwo : public rclcpp::Node
{
public:
    AddTwo();

private:
    void add(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, // CHANGE
             std::shared_ptr<std_srvs::srv::SetBool::Response> response);    // CHANGE
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;
};

#endif // COMMUNICATION_HARDWARE_BRIDGE_H
