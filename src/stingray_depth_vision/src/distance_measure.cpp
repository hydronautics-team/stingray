#include <ros/ros.h>
#include <array>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <sensor_msgs/Image.h>
#include "stingray_depth_vision/Object.h"

using namespace boost::accumulators;

class CameraActions {

private:
    int16_t top_left_x;
    int16_t top_left_y;
    int16_t bottom_right_x;
    int16_t bottom_right_y;
};

void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    float* depths = (float*)(&msg->data[0]);

    // TODO: subscribe to these values
    uint32_t left_x = 0;
    uint32_t right_x = msg->width;
    uint32_t upper_y = 0;
    uint32_t lower_y = msg->height;

    accumulator_set<float, features<tag::mean, tag::count>> acc;

    for (uint32_t h = upper_y; h < lower_y; ++h) {
        for (uint32_t w = left_x; w < right_x; ++w) {
            if (!std::isnan(depths[w + (h - upper_y)*w]) && !std::isinf(depths[w + (h - upper_y)*w])) {
                acc(depths[w + (h - upper_y)*w]);
            }
        }
    }

    if (count(acc) == 0)
        ROS_WARN("All data is nan or inf");
    else
        ROS_INFO("Mean distance: %g, Counted: %ld", mean(acc), count(acc));
}

void netCallback(const distance_measure::Object::ConstPtr& msg) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_subscriber");

    ros::NodeHandle n;

    ros::Subscriber subDepth = n.subscribe("/zedm/zed_node/depth/depth_registered", 10, depthCallback);

    ros::spin();

    return 0;
}
