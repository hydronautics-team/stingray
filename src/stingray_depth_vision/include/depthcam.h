#ifndef STINGRAY_DEPTH_VISION_INCLUDE_DEPTHCAM_H_
#define STINGRAY_DEPTH_VISION_INCLUDE_DEPTHCAM_H_

#include <memory>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace depth {

typedef struct Point {
    int32_t x;
    int32_t y;
};

typedef struct ImageFrame {
    Point topLeft;
    Point bottomRight;
};

class DepthCamera {

    static void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        float* depths = std::reinterpret_cast<float*>(&msg->data[0]);
    }

    static size_t getMean(const float const *image, const std::optional<ImageFrame> &frame, size_t stride=1) {
#ifdef DEBUG
        boost::accumulators::accumulator_set<float, features<boost::accumulators::tag::mean, boost::accumulators::tag::count>> acc;
#else
        boost::accumulators::accumulator_set<float, boost::accumulators::features<tag::mean>> acc;
#endif
        // TODO: create iterator on frame
        for (size_t h = upper_y; h < lower_y; ++h) {
            for (size_t w = frame.top_left_x; w < frame.bottom_right_x; ++w) {
                if (depthIsValid(depths[w + (h - upper_y)*w])) {
                    acc(depths[w + (h - upper_y)*w]);
                }
            }
        }

        if (count(acc) == 0)
            ROS_WARN("All data is nan or inf");
        else
#ifdef DEBUG
            ROS_INFO("Mean distance: %g, Counted: %ld", boost::accumulators::mean(acc), boost::accumulators::count(acc));
#endif
    }

private:
    std::unique_ptr<ImageFrame> frame;
    std::unique_ptr<float> depths;

    bool dataIsInitialized() const {
        return (depths != nullptr && frame != nullptr);
    }

    bool depthIsValid(float value) const {
        return (!std::isnan(value) && !std::isinf(value));
    }
};

}  // namespace depth

#endif  // STINGRAY_DEPTH_VISION_INCLUDE_DEPTHCAM_H_
