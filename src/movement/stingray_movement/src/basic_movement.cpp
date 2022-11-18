#include <basic/HorizontalMovementServer.h>
#include <basic/DiveServer.h>

static const std::string NODE_NAME = "basic_movement";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle(NODE_NAME);

    // get json configs
    json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
    json control_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/control.json"));

    double velocityCoefficient = control_config["movement"]["velocity_coefficient"];
    ROS_INFO("Velocity coefficient %f", velocityCoefficient);

    if (velocityCoefficient < 1.0)
    {
        ROS_ERROR("Velocity coefficient must be larger than 1.0");
        return 0;
    }

    HorizontalMovementServer horizontalMovementServer(ros_config["actions"]["movement"]["horizontal"], velocityCoefficient);
    DiveServer diveServer(ros_config["actions"]["movement"]["dive"]);

    ros::spin();

    return 0;
}