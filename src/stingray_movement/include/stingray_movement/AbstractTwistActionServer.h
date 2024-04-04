#ifndef STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTTWISTACTIONSERVER_H_
#define STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTTWISTACTIONSERVER_H_

#include "stingray_utils/AbstractActionServer.h"
#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"

/**
 * Action server that is responsible for moving vehicle
 * by march and lag.
 */
template <class TTwistAction, class TTwistActionGoal>
class AbstractTwistActionServer : public AbstractActionServer<TTwistAction, TTwistActionGoal> {

public:

    virtual bool isTwistDone(const std::shared_ptr<const TTwistActionGoal> goal) = 0;

    virtual bool isDepthDone(const float &goal_depth) {
        if (current_uv_state.depth_stabilization) {
            float depth_delta = abs(current_uv_state.depth - goal_depth);
            bool depth_done = depth_delta < depth_tolerance;
            if (!depth_done) {
                RCLCPP_ERROR(this->_node->get_logger(), "Depth not reached current_depth %f", current_uv_state.depth);
                RCLCPP_ERROR(this->_node->get_logger(), "Depth not reached depth_tolerance %f", depth_tolerance);
                RCLCPP_ERROR(this->_node->get_logger(), "Depth not reached depth_delta %f", depth_delta);
            }

            return depth_done;
        } else {
            return true;
        }
    };

    virtual bool isRollDone(const float &goal_roll) {
        if (current_uv_state.roll_stabilization) {
            float roll_delta = abs(current_uv_state.roll - goal_roll);
            bool roll_done = roll_delta < angle_tolerance;
            if (!roll_done) {
                RCLCPP_ERROR(this->_node->get_logger(), "Roll not reached current_roll %f", current_uv_state.roll);
                RCLCPP_ERROR(this->_node->get_logger(), "Roll not reached angle_tolerance %f", angle_tolerance);
                RCLCPP_ERROR(this->_node->get_logger(), "Roll not reached roll_delta %f", roll_delta);
            }
            return roll_done;
        } else {
            return true;
        }
    }

    virtual bool isPitchDone(const float &goal_pitch) {
        if (current_uv_state.pitch_stabilization) {
            float pitch_delta = abs(current_uv_state.pitch - goal_pitch);
            bool pitch_done = pitch_delta < angle_tolerance;
            if (!pitch_done) {
                RCLCPP_ERROR(this->_node->get_logger(), "Pitch not reached current_pitch %f", current_uv_state.pitch);
                RCLCPP_ERROR(this->_node->get_logger(), "Pitch not reached angle_tolerance %f", angle_tolerance);
                RCLCPP_ERROR(this->_node->get_logger(), "Pitch not reached pitch_delta %f", pitch_delta);
            }
            return pitch_done;
        } else {
            return true;
        }
    }

    virtual bool isYawDone(const float &goal_yaw) {
        if (current_uv_state.yaw_stabilization) {
            float yaw_delta = abs(current_uv_state.yaw - goal_yaw);
            bool yaw_done = yaw_delta < angle_tolerance;
            if (!yaw_done) {
                RCLCPP_ERROR(this->_node->get_logger(), "Yaw not reached current_yaw %f", current_uv_state.yaw);
                RCLCPP_ERROR(this->_node->get_logger(), "Yaw not reached yaw_tolerance %f", angle_tolerance);
                RCLCPP_ERROR(this->_node->get_logger(), "Yaw not reached yaw_delta %f", yaw_delta);
            }
            return yaw_done;
        } else {
            return true;
        }
    }

    virtual void stopTwist() {
        auto twistSrvRequest = std::make_shared<stingray_core_interfaces::srv::SetTwist::Request>();
        twistSrvRequest->surge = 0.0;
        twistSrvRequest->sway = 0.0;
        twistSrvRequest->yaw = 0.0;
        if (!current_uv_state.depth_stabilization)
            twistSrvRequest->depth = 0.0;
        if (!current_uv_state.roll_stabilization)
            twistSrvRequest->roll = 0.0;
        if (!current_uv_state.pitch_stabilization)
            twistSrvRequest->pitch = 0.0;

        RCLCPP_INFO(this->_node->get_logger(), "Twist action request stop yaw: %f, surge: %f", twistSrvRequest->yaw, twistSrvRequest->surge);
        twistSrvClient->async_send_request(twistSrvRequest).wait();
    }

    rclcpp::Client<stingray_core_interfaces::srv::SetTwist>::SharedPtr twistSrvClient;

    float depth_tolerance;
    float angle_tolerance;
    stingray_core_interfaces::msg::UVState current_uv_state;

    AbstractTwistActionServer(std::shared_ptr<rclcpp::Node> _node, const std::string &actionName) : AbstractActionServer<TTwistAction, TTwistActionGoal>(_node, actionName) {

        _node->declare_parameter("uv_state_topic", "/stingray/topics/uv_state");
        _node->declare_parameter("set_twist_srv", "/stingray/services/set_twist");
        _node->declare_parameter("depth_tolerance", 0.1);
        _node->declare_parameter("angle_tolerance", 10.0);

        // ROS service clients
        twistSrvClient = _node->create_client<stingray_core_interfaces::srv::SetTwist>(_node->get_parameter("set_twist_srv").as_string());
        // ROS subscribers
        uvStateSub = _node->create_subscription<stingray_core_interfaces::msg::UVState>(
            _node->get_parameter("uv_state_topic").as_string(), 1000,
            std::bind(&AbstractTwistActionServer::uvStateCallback, this, std::placeholders::_1));
        depth_tolerance = _node->get_parameter("depth_tolerance").as_double();
        angle_tolerance = _node->get_parameter("angle_tolerance").as_double();
    }
    ~AbstractTwistActionServer() = default;

private:
    void uvStateCallback(const stingray_core_interfaces::msg::UVState &msg) {
        current_uv_state = msg;
    }

    rclcpp::Subscription<stingray_core_interfaces::msg::UVState>::SharedPtr uvStateSub;
};

#endif //STINGRAY_SRC_STINGRAY_MOVEMENT_INCLUDE_ABSTRACTTWISTACTIONSERVER_H_
