#include <ros/ros.h>

#include "patterns/TackServer.h"

static const std::string NODE_NAME = "movement_patterns";

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nodeHandle(NODE_NAME);

  // TODO: Instantiate movement patterns servers
  

  TackServer server("stingray_action_tack", 1.0); 
  ros::spin();

  return 0;
}
