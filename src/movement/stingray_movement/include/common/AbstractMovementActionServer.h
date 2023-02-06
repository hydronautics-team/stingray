#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <fstream>

#include <stingray_utils/json.hpp>

using json = nlohmann::json;

/**
 * Abstract base for implementing action servers
 * related to vehicle movement.
 *
 * @tparam TAction Action type
 * @tparam TGoalPtr Action goal const pointer type
 */
template <class TAction, class TGoalPtr>
class AbstractMovementActionServer
{

protected:
    json ros_config;
    json control_config;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<TAction> actionServer;

    double velocityCoefficient;

    /** Implement your goal processing logic */
    virtual void goalCallback(const TGoalPtr &goal) = 0;

public:
    AbstractMovementActionServer(const std::string &actionName, double velocityCoefficient);
    ~AbstractMovementActionServer() = default;
};

template <class TAction, class TGoalPtr>
AbstractMovementActionServer<TAction, TGoalPtr>::AbstractMovementActionServer(const std::string &actionName,
                                                                              double velocityCoefficient) : actionServer(nodeHandle,
                                                                                                                         actionName,
                                                                                                                         boost::bind(&AbstractMovementActionServer<TAction, TGoalPtr>::goalCallback,
                                                                                                                                     this, _1),
                                                                                                                         false),
                                                                                                            velocityCoefficient(velocityCoefficient)
{
    actionServer.start();
    ros_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/ros.json"));
    control_config = json::parse(std::ifstream(ros::package::getPath("stingray_resources") + "/configs/control.json"));
}

#endif // STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTMOVEMENTACTIONSERVER_H_
