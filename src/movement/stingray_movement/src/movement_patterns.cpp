#include <patterns/AlignServer.h>
#include <patterns/AlignLowLevelServer.h>

#include "patterns/TackServer.h"

static const std::string NODE_NAME = "movement_patterns";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle(NODE_NAME);

    // get json configs
    json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
    json control_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/control.json"));

    double velocityCoefficient = control_config["movement"]["velocity_coefficient"];

    if (velocityCoefficient < 1.0)
    {
        ROS_ERROR("Velocity coefficient must be larger than 1.0");
        return 0;
    }

    AlignServer alignServer(ros_config["actions"]["movement"]["align"], velocityCoefficient);
    AlignLowLevelServer alignLowLevelServer(ros_config["actions"]["movement"]["align_low_level"], velocityCoefficient);
    TackServer tackServer(ros_config["actions"]["movement"]["tack"], 1.0);

    ros::spin();

    return 0;
}
