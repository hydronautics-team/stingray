#include <ros/ros.h>

static const std::string NODE_NAME = "movement_patterns";

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nodeHandle(NODE_NAME);

  // TODO: Instantiate movement patterns servers

  ros::spin();

  return 0;
}