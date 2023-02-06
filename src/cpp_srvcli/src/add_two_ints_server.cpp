#include "add_two_ints_server.h"

AddTwo::AddTwo() : Node("add_two_ints_server")
{
    service = this->create_service<std_srvs::srv::SetBool>("add_two_ints", &AddTwo::add); // CHANGE
}

void AddTwo::add(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, // CHANGE
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)     // CHANGE
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddTwo>());
    rclcpp::shutdown();

    return 0;
}