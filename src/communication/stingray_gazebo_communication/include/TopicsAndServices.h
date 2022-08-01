#ifndef STINGRAY_SRC_STINGRAY_COMMUNICATION_INCLUDE_TOPICSANDSERVICES_H_
#define STINGRAY_SRC_STINGRAY_COMMUNICATION_INCLUDE_TOPICSANDSERVICES_H_

#include <string>

// TODO: Documentation

static const std::string    OUTPUT_PARCEL_TOPIC             = "/stingray/topics/hardware_bridge/parcels";
static const std::string    INPUT_PARCEL_TOPIC              = "/stingray/topics/drivers/parcels";
static const std::string    DEPTH_PUBLISH_TOPIC             = "/stingray/topics/position/depth";
static const std::string    YAW_PUBLISH_TOPIC               = "/stingray/topics/position/yaw";
static const std::string    GAZEBO_VELOCITY_TOPIC           = "/cmd_vel";
static const std::string    GAZEBO_ODOMETRY_PUBLISH_TOPIC   = "/odom";
static const std::string    SET_LAG_AND_MARCH_SERVICE       = "/stingray/services/control/set_lag_and_march";
static const std::string    SET_DEPTH_SERVICE               = "/stingray/services/control/set_depth";
static const std::string    SET_YAW_SERVICE                 = "/stingray/services/control/set_yaw";
static const std::string    SET_IMU_ENABLED_SERVICE         = "/stingray/services/control/set_imu_enabled";
static const std::string    SET_STABILIZATION_SERVICE       = "/stingray/services/control/set_stabilization";
static const std::string    SET_DEVICE_SERVICE              = "/stingray/services/control/set_device";
static const std::string    GAZEBO_GET_STATE_SERVICE        = "/gazebo/get_model_state";
static const std::string    GAZEBO_SET_STATE_SERVICE        = "/gazebo/set_model_state";

#endif //STINGRAY_SRC_STINGRAY_COMMUNICATION_INCLUDE_TOPICSANDSERVICES_H_
