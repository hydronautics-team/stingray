#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTSEARCHTWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTSEARCHTWISTACTIONSERVER_H_

#include "stingray_movement/AbstractTwistActionServer.h"

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
template <class TSearchTwistAction, class TSearchTwistActionGoal>
class AbstractSearchTwistActionServer : public AbstractTwistActionServer<TSearchTwistAction, TSearchTwistActionGoal> {

protected:
    virtual bool isSearchTwistDone() = 0;
    int target_found_counter = 0;
    int target_found_threshold = 0;
    float target_yaw_step = 0.0;
    float found_target_yaw = 0.0;
    
public:

    AbstractSearchTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractTwistActionServer<TSearchTwistAction, TSearchTwistActionGoal>(_node, actionName) {};
    ~AbstractSearchTwistActionServer() = default;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTSEARCHTWISTACTIONSERVER_H_
