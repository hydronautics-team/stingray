#include <ros/ros.h>
#include <string>
#include "../include/UpDownServer.h"
#include <ros/package.h>
#include <fstream>
#include "stingray_utils/json.hpp"

using json = nlohmann::json;
// get json config
static const json ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));

static const std::string NODE_NAME = "updown_device";

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nodeHandle(NODE_NAME);

  UpDownServer UpDownServer(ros_config["actions"]["updown"], ros_config["services"]["updown"]);

  ros::spin();

  return 0;
}