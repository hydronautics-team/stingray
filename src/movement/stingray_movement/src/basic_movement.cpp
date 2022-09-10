#include <basic/LinearMovementServer.h>
#include <basic/DiveServer.h>
#include <basic/RotateServer.h>

static const std::string NODE_NAME = "basic_movement";

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle(NODE_NAME);

    // get json configs
    json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
    json control_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/control.json"));

    double velocityCoefficient = control_config["movement"]["velocity_coefficient"];
    double rotateCoefficient = control_config["movement"]["rotate_coefficient"];
    ROS_INFO("Velocity coefficient %f", velocityCoefficient);
    ROS_INFO("Rotate coefficient %f", rotateCoefficient);

    if (velocityCoefficient < 1.0)
    {
        ROS_ERROR("Velocity coefficient must be larger than 1.0");
        return 0;
    }

    LinearMovementServer linearMovementServer(ros_config["actions"]["movement"]["linear"], velocityCoefficient);
    DiveServer diveServer(ros_config["actions"]["movement"]["dive"]);
    RotateServer rotateServer(ros_config["actions"]["movement"]["rotate"], rotateCoefficient);

    ros::spin();

    return 0;
}