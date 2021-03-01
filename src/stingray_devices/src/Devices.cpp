#include <ros/ros.h>
#include <string>
#include "../include/UpDownServer.h"

static const std::string NODE_NAME = "stingray_devices";

static const std::string UPDOWN_ACTION = "stingray_action_updown";

static const std::string UPDOWN_SERVICE = "updown_service";


int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nodeHandle(NODE_NAME);

  UpDownServer UpDownServer(UPDOWN_ACTION, UPDOWN_SERVICE);

  ros::spin();

  return 0;
}