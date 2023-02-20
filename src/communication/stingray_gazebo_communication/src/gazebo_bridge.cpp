#include "gazebo_bridge.h"

#include <fstream>

GazeboBridge::GazeboBridge() : Node("GazeboBridge") {
    RCLCPP_INFO(this->get_logger(), "okay good");
    ros_config = json::parse(std::ifstream("resources/configs/ros.json"));
    simulation_config = json::parse(std::ifstream("resources/configs/simulation.json"));
    // Initializing timer for publishing messages. Callback interval: 0.05 ms
    this->publishingTimer = this->create_wall_timer(50ms, std::bind(&GazeboBridge::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "timer okay good");
    // ROS publishers
    this->depthPublisher = this->create_publisher<std_msgs::msg::Float64>(ros_config["topics"]["depth"], 1000);
    this->yawPublisher = this->create_publisher<std_msgs::msg::Float64>(ros_config["topics"]["yaw"], 20);
    this->pingerBucketPublisher = this->create_publisher<std_msgs::msg::Int32>(ros_config["topics"]["pinger_buckets"], 20);
    this->pingerFlarePublisher = this->create_publisher<std_msgs::msg::Int32>(ros_config["topics"]["pinger_flare"], 20);
    this->velocityPublisher = this->create_publisher<geometry_msgs::msg::Twist>(ros_config["topics"]["gazebo_velocity"], 20);
    RCLCPP_INFO(this->get_logger(), "publishers okay good");
    // ROS services
    this->horizontalMoveService = this->create_service<stingray_communication_msgs::srv::SetHorizontalMove>(
        ros_config["services"]["set_horizontal_move"],
        std::bind(&GazeboBridge::horizontalMoveCallback, this, std::placeholders::_1, std::placeholders::_2));

    this->depthService = this->create_service<stingray_communication_msgs::srv::SetInt16>(
        ros_config["services"]["set_depth"], std::bind(&GazeboBridge::depthCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->imuService = this->create_service<std_srvs::srv::SetBool>(
        ros_config["services"]["set_imu_enabled"], std::bind(&GazeboBridge::imuCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->stabilizationService = this->create_service<stingray_communication_msgs::srv::SetStabilization>(
        ros_config["services"]["set_stabilization_enabled"],
        std::bind(&GazeboBridge::stabilizationCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->deviceActionService = this->create_service<stingray_communication_msgs::srv::SetDeviceAction>(
        ros_config["services"]["updown"], std::bind(&GazeboBridge::deviceActionCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "services okay good");
    // ROS service clients
    this->getModelStateService = this->create_client<gazebo_msgs::srv::GetModelState>(ros_config["services"]["gazebo_get_state"]);
    this->setModelStateService = this->create_client<gazebo_msgs::srv::SetModelState>(ros_config["services"]["gazebo_set_state"]);
    currentTwist.linear.x = currentTwist.linear.y = currentTwist.linear.z = currentTwist.angular.x = currentTwist.angular.y = currentTwist.angular.z =
        0;
    RCLCPP_INFO(this->get_logger(), "service clients okay good");
}

std::shared_ptr<gazebo_msgs::srv::GetModelState::Response> GazeboBridge::getModelState(const std::string &model_name) {
    auto getModelStateRequest = std::make_shared<gazebo_msgs::srv::GetModelState::Request>();
    getModelStateRequest->model_name = model_name;
    while (!getModelStateService->wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the getModelStateService service. Exiting.");
            return nullptr;
        }
        RCLCPP_INFO(this->get_logger(), "getModelStateService service not available, waiting again...");
    }
    auto getModelStateResponse = getModelStateService->async_send_request(getModelStateRequest);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), getModelStateResponse) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to obtain state for model '%s' in Gazebo: '%s'", model_name.c_str(),
                     getModelStateResponse.get()->status_message.c_str());
        return nullptr;
    }

    return getModelStateResponse.get();
}

/**
 * Obtains model state from Gazebo, transforms and updates it.
 * @param transform Function that transforms current model state
 * @throws {@code std::runtime_error} if fails to get or set model state in Gazebo
 */
void GazeboBridge::updateModelState(const std::function<void(gazebo_msgs::msg::ModelState &)> &transform) {
    auto getModelStateResponse = getModelState(simulation_config["vehicle_model_name"]);
    auto setModelStateRequest = std::make_shared<gazebo_msgs::srv::SetModelState::Request>();
    setModelStateRequest->model_state.model_name = simulation_config["vehicle_model_name"];
    setModelStateRequest->model_state.pose = getModelStateResponse.get()->pose;
    setModelStateRequest->model_state.twist = getModelStateResponse.get()->twist;

    // do transform
    transform(setModelStateRequest->model_state);

    while (!setModelStateService->wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the setModelStateService service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "setModelStateService service not available, waiting again...");
    }
    auto setModelStateResponse = setModelStateService->async_send_request(setModelStateRequest);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), setModelStateResponse) != rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_ERROR(this->get_logger(), "Failed to update state in Gazebo: '%s'", setModelStateResponse.get()->status_message.c_str());
}

/**
 * This method allows you to determine the angles to the pinger
 * @param pinger pinger name
 * @param yaw corner robot now
 * @return {@code pair} Angle by xy and z to pinger
 */
std::pair<std_msgs::msg::Int32, std_msgs::msg::Int32> GazeboBridge::pingerStatus(const std::string &pinger_name, const float &yaw = 0) {
    auto f90 = [](float corner) {
        if (corner > 0)
            corner -= 90;
        else if (corner < -0)
            corner += 90;
        return corner;
    };

    // get robot position
    auto getModelStateResponse = getModelState(simulation_config["vehicle_model_name"]);
    // get pinger position
    auto getPingerModelStateResponse = getModelState(simulation_config[pinger_name]);

    double path_x = getPingerModelStateResponse.get()->pose.position.x - getModelStateResponse.get()->pose.position.x;
    double path_y = getPingerModelStateResponse.get()->pose.position.y - getModelStateResponse.get()->pose.position.y;
    double path_z = getModelStateResponse.get()->pose.position.z - getPingerModelStateResponse.get()->pose.position.z;

    double r_xy = std::sqrt(path_x * path_x + path_y * path_y);
    std_msgs::msg::Int32 corner_XY;
    std_msgs::msg::Int32 corner_Z;
    float corner_XY_data = std::atan(path_y / path_x) * 180 / M_PI;
    /*
     * I'm not sure if the angle of rotation of the device should be added or subtracted
     * */
    corner_XY.data = f90(corner_XY_data) + yaw;  // ???
    float corner_Z_data = std::atan(path_z / r_xy) * 180 / M_PI;
    corner_Z.data = -corner_Z_data;

    /*
     * For tests
     * */
    //    ROS_INFO("XY %f", corner_XY_data);
    //    ROS_INFO("Z %f", corner_Z_data);

    std::pair<std_msgs::msg::Int32, std_msgs::msg::Int32> df(corner_XY, corner_Z);
    return df;
}

/**
 * Sets vehicle lag and march speed
 * @param request Service request with lag and march speed in Gazebo-related units
 * @param response Service response
 * @return {@code true} if service call didn't fail
 */
void GazeboBridge::horizontalMoveCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Request> request,
                                          std::shared_ptr<stingray_communication_msgs::srv::SetHorizontalMove::Response> response) {
    // ROS_INFO("horizontalMoveCallback in gazebo bridge");

    currentTwist.linear.x = -request->lag;
    RCLCPP_INFO(this->get_logger(), "request.lag %f", request->lag);
    currentTwist.linear.y = -request->march;

    if (!yawStabilizationEnabled) {
        response->success = false;
        response->message = "Yaw stabilization is not enabled";
    }

    try {
        updateModelState([&](gazebo_msgs::msg::ModelState &modelState) {
            double desiredYaw = -request->yaw;
            RCLCPP_INFO(this->get_logger(), "currentYaw %f", currentYaw);
            RCLCPP_INFO(this->get_logger(), "desiredYaw %f", desiredYaw);
            this->currentYaw += desiredYaw;
            RCLCPP_INFO(this->get_logger(), "currentYaw %f", currentYaw);
            double newYaw = currentYaw * M_PI / 180.0 + simulation_config["initial_yaw"].get<double>();
            RCLCPP_INFO(this->get_logger(), "newYaw %f", newYaw);
            tf2::Quaternion tf2_quat;
            tf2_quat.setRPY(simulation_config["initial_roll"].get<double>(), simulation_config["initial_pitch"].get<double>(), newYaw);
            modelState.pose.orientation = tf2::toMsg(tf2_quat);
        });
    } catch (std::runtime_error &e) {
        response->success = false;
        response->message = "Failed to set depth in Gazebo: " + std::string(e.what());
    }

    response->success = true;
}

/**
 * Dives vehicle on specified depth
 * @param request Service request with depth in centimetres
 * @param response Service response
 * @return {@code true} if service call didn't fail
 */
void GazeboBridge::depthCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetInt16::Request> request,
                                 std::shared_ptr<stingray_communication_msgs::srv::SetInt16::Response> response) {
    /*
     * Here we simulate enabled depth stabilization: we just pass desired depth
     * for Gazebo like it is low-level control system that stabilizes this depth.
     */

    if (!depthStabilizationEnabled) {
        response->success = false;
        response->message = "Depth stabilization is not enabled";
    }

    try {
        updateModelState([&](gazebo_msgs::msg::ModelState &modelState) {
            /* In our simulator scale is 1.0 = 1 metre, and target depth is passed in centimetres.
             * Bias is needed due to simulator implementation details. */
            float initialDepth = simulation_config["initial_depth"].get<float>();
            float depthDelta = request->value / 1000.0;
            float fullDepth = initialDepth - depthDelta;
            RCLCPP_INFO(this->get_logger(), "depth %f", depthDelta);
            RCLCPP_INFO(this->get_logger(), "initial depth %f", initialDepth);
            RCLCPP_INFO(this->get_logger(), "full depth %f", fullDepth);
            modelState.pose.position.z = fullDepth;
        });
    } catch (std::runtime_error &e) {
        response->success = false;
        response->message = "Failed to set depth in Gazebo: " + std::string(e.what());
    }

    response->success = true;
}
void GazeboBridge::imuCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Setting SHORE_STABILIZE_IMU_BIT to %d", request->data);
    response->success = true;
}
void GazeboBridge::deviceActionCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Request> request,
                                        std::shared_ptr<stingray_communication_msgs::srv::SetDeviceAction::Response> response) {
    (void)request;
    response->success = true;
}
void GazeboBridge::stabilizationCallback(const std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Request> request,
                                         std::shared_ptr<stingray_communication_msgs::srv::SetStabilization::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Setting depth stabilization %d", request->depth_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting yaw stabilization %d", request->yaw_stabilization);
    depthStabilizationEnabled = request->depth_stabilization;
    yawStabilizationEnabled = request->yaw_stabilization;
    currentYaw = yawMessage.data;

    response->success = true;
}
void GazeboBridge::timerCallback() {
    RCLCPP_INFO(this->get_logger(), "Timer callback" );
    auto getModelStateResponse = getModelState(simulation_config["vehicle_model_name"]);
    if (getModelStateResponse) {
        // Convert back to initial values
        depthMessage.data = -(getModelStateResponse.get()->pose.position.z - simulation_config["initial_depth"].get<double>()) * 100;
        float yaw_postprocessed =
            -(tf2::getYaw(getModelStateResponse.get()->pose.orientation) - simulation_config["initial_yaw"].get<double>()) * 180.0 / M_PI;
        if (yaw_postprocessed > 180) {
            yaw_postprocessed -= 360;
        } else if (yaw_postprocessed < -180) {
            yaw_postprocessed += 360;
        }
        yawMessage.data = static_cast<int16_t>(yaw_postprocessed);
        auto df_pinger_bucket = pingerStatus(simulation_config["pinger_buckets_model_name"], yaw_postprocessed);
        auto df_pinger_flare = pingerStatus(simulation_config["pinger_flare_model_name"], yaw_postprocessed);
        pingerBucketMessage = df_pinger_bucket.first;
        pingerFlareMessage = df_pinger_flare.first;
    }

    depthPublisher->publish(depthMessage);
    yawPublisher->publish(yawMessage);
    pingerBucketPublisher->publish(pingerBucketMessage);
    pingerFlarePublisher->publish(pingerFlareMessage);

    velocityPublisher->publish(currentTwist);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<GazeboBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
