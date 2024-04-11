#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTCENTERINGTWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTCENTERINGTWISTACTIONSERVER_H_

#include "stingray_movement/AbstractTwistActionServer.h"

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

    AbstractCenteringTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractTwistActionServer<TCenteringTwistAction, TCenteringTwistActionGoal>(_node, actionName) {};
    ~AbstractCenteringTwistActionServer() = default;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTCENTERINGTWISTACTIONSERVER_H_
