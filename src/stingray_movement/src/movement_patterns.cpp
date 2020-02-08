#include <patterns/AlignServer.h>

static const std::string NODE_NAME = "movement_patterns";

static const std::string PARAM_VELOCITY_COEFFICIENT = "velocity_coefficient";

static const std::string ACTION_ALIGN = "stingray_action_align";

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nodeHandle(NODE_NAME);

  double velocityCoefficient = 0.0;

  bool hasParameter = nodeHandle.param(PARAM_VELOCITY_COEFFICIENT, velocityCoefficient, 0.0);
  if (!hasParameter) {
    ROS_ERROR("Parameter %s is not specified!", PARAM_VELOCITY_COEFFICIENT.c_str());
    return 0;
  }

  if (velocityCoefficient < 1.0) {
    ROS_ERROR("Velocity coefficient must be larger than 1.0");
    return 0;
  }

  AlignServer linearMovementServer(ACTION_ALIGN, velocityCoefficient);

  ros::spin();

  return 0;
}