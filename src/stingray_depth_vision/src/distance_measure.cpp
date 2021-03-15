#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "depthcam.h"
#include "stingray_depth_vision/Object.h"

void frameCallback(const distance_measure::Object::ConstPtr& msg) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_subscriber");
    ros::NodeHandle handle;
    ros::Subscriber subDepth = handle.subscribe("/zedm/zed_node/depth/depth_registered", 10, depthCallback);
    // TODO: publish distance
    ros::spin();
    return 0;
}
