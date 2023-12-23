#include <patterns/AlignServer.h>
#include <patterns/AlignLowLevelServer.h>

#include "patterns/TackServer.h"

static const std::string NODE_NAME = "movement_patterns";

static const std::string PARAM_VELOCITY_COEFFICIENT = "velocity_coefficient";

static const std::string ACTION_ALIGN = "stingray_action_align";

static const std::string ACTION_ALIGN_LOW_LEVEL = "stingray_action_align_low_level";


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

  AlignServer alignServer(ACTION_ALIGN, velocityCoefficient);
  AlignLowLevelServer alignLowLevelServer(ACTION_ALIGN_LOW_LEVEL, velocityCoefficient);
  TackServer server("stingray_action_tack", 1.0);

  ros::spin();

  return 0;
}
