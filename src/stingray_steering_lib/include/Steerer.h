#ifndef STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
#define STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <stingray_msgs/SetLagAndMarch.h>
#include <ros/node_handle.h>

typedef enum {
  LINEAR_LEVEL_SLOW,
  LINEAR_LEVEL_MEDIUM,
  LINEAR_LEVEL_FAST,
  LINEAR_LEVEL_TURBO
} LinearVelocityLevel;

typedef enum {
  MARCH_FORWARD,
  MARCH_BACKWARDS
} MarchDirection;

typedef enum {
  LAG_RIGHT,
  LAG_LEFT
} LagDirection;

static const std::string DEPTH_SUBSCRIBE_TOPIC = "/stingray/topics/position/depth";
static const std::string YAW_SUBSCRIBE_TOPIC = "/stingray/topics/position/yaw";

static const std::string SET_LAG_AND_MARCH_SERVICE = "/stingray/services/control/set_lag_and_march";
static const std::string SET_DEPTH_SERVICE = "/stingray/services/control/set_depth";
static const std::string SET_YAW_SERVICE = "/stingray/services/control/set_yaw";

static const int LINEAR_VELOCITY_LEVELS = 4;

static const int DEPTH_DELTA = 10;
static const int YAW_DELTA = 5;

class Steerer {

 private:

  double linearVelocityValues[LINEAR_VELOCITY_LEVELS];

 protected:

  ros::NodeHandle nodeHandle;

  double getVelocity(LinearVelocityLevel velocityLevel);

  void move(double marchVelocity, double lagVelocity, int durationSeconds);

 public:

  Steerer(const ros::NodeHandle& nodeHandle, double linearSlow, double linearMedium,
      double linearFast, double linearTurbo);
  Steerer(Steerer& other) = default;
  ~Steerer() = default;

  // TODO: Add async API

  void moveMarch(int durationSeconds, LinearVelocityLevel velocityLevel, MarchDirection direction);
  void moveMarchForward(int durationSeconds, LinearVelocityLevel velocityLevel);
  void moveMarchBackwards(int durationSeconds, LinearVelocityLevel velocityLevel);

  void moveLag(int durationSeconds, LinearVelocityLevel velocityLevel, LagDirection direction);
  void moveLagRight(int durationSeconds, LinearVelocityLevel velocityLevel);
  void moveLagLeft(int durationSeconds, LinearVelocityLevel velocityLevel);

  void stop();

  void rotateToAngle(int angleDegree);

  void diveToDepth(int depthCm);

};

#endif //STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
