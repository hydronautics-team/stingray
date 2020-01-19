#include <LinearMovementServer.h>
#include <DiveServer.h>
#include <RotateServer.h>

static const std::string NODE_NAME = "basic_movement";

static const std::string PARAM_VELOCITY_COEFFICIENT = "velocity_coefficient";

static const std::string ACTION_LINEAR_MOVEMENT = "stingray_action_linear_movement";
static const std::string ACTION_DIVE = "stingray_action_dive";
static const std::string ACTION_ROTATE = "stingray_action_rotate";

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

  LinearMovementServer linearMovementServer(ACTION_LINEAR_MOVEMENT, velocityCoefficient);
  DiveServer diveServer(ACTION_DIVE);
  RotateServer rotateServer(ACTION_ROTATE);

  ros::spin();

  return 0;
}