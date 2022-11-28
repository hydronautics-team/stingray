#include <basic/HorizontalMovementServer.h>
#include <basic/DiveServer.h>

static const std::string NODE_NAME = "basic_movement";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(NODE_NAME);
    

    // get json configs
    json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
    json control_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/control.json"));

    double velocityCoefficient = control_config["movement"]["velocity_coefficient"];
    RCLCPP_INFO(node->get_logger(), "Velocity coefficient %f", velocityCoefficient);

    if (velocityCoefficient < 1.0)
    {
        RCL_ERROR("Velocity coefficient must be larger than 1.0");
        return 0;
    }

    HorizontalMovementServer horizontalMovementServer(ros_config["actions"]["movement"]["horizontal"], velocityCoefficient);
    DiveServer diveServer(ros_config["actions"]["movement"]["dive"]);

    rclcpp::spin(node);

    return 0;
}