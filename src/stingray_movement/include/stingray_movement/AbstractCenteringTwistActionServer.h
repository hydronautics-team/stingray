#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTCENTERINGTWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTCENTERINGTWISTACTIONSERVER_H_

#include "stingray_movement/AbstractTwistActionServer.h"
#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
template <class TCenteringTwistAction, class TCenteringTwistActionGoal>
class AbstractCenteringTwistActionServer : public AbstractTwistActionServer<TCenteringTwistAction, TCenteringTwistActionGoal> {

protected:
    virtual bool isCenteringTwistDone() = 0;
    virtual bool isTargetLost() = 0;
    
    float target_distance_threshold;
    int target_lost_thresh;

    int target_disappeared_counter = 0;

public:

    AbstractCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractTwistActionServer<TCenteringTwistAction, TCenteringTwistActionGoal>(_node, actionName) {
    // ROS subscribers
        uvStateSub = _node->create_subscription<stingray_core_interfaces::msg::UVState>(
            _node->get_parameter("uv_state_topic").as_string(), 1000,
            std::bind(&AbstractCenteringTwistActionServer::uvStateCallback, this, std::placeholders::_1));
    };
    ~AbstractCenteringTwistActionServer() = default;
    
    stingray_core_interfaces::msg::UVState current_uv_state;
    
private:
    void uvStateCallback(const stingray_core_interfaces::msg::UVState &msg) {
        current_uv_state = msg;
    }

    rclcpp::Subscription<stingray_core_interfaces::msg::UVState>::SharedPtr uvStateSub;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTCENTERINGTWISTACTIONSERVER_H_
