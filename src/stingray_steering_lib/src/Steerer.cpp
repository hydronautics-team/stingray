#include "stingray_steering_lib/Steerer.h"

#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>

#include <stingray_msgs/SetFloat64.h>
#include <stingray_msgs/SetInt32.h>

const std::string Steerer::DEPTH_SUBSCRIBE_TOPIC = "/stingray/topics/position/depth";
const std::string Steerer::YAW_SUBSCRIBE_TOPIC = "/stingray/topics/position/yaw";

const std::string Steerer::SET_LAG_AND_MARCH_SERVICE = "/stingray/services/control/set_lag_and_march";
const std::string Steerer::SET_DEPTH_SERVICE = "/stingray/services/control/set_depth";
const std::string Steerer::SET_YAW_SERVICE = "/stingray/services/control/set_yaw";

Steerer::Steerer(const ros::NodeHandle& nodeHandle, double linearSlow, double linearMedium,
    double linearFast, double linearTurbo) : nodeHandle(nodeHandle) {
  linearVelocityValues[LinearVelocityLevel::LINEAR_LEVEL_SLOW] = linearSlow;
  linearVelocityValues[LinearVelocityLevel::LINEAR_LEVEL_MEDIUM] = linearMedium;
  linearVelocityValues[LinearVelocityLevel::LINEAR_LEVEL_FAST] = linearFast;
  linearVelocityValues[LinearVelocityLevel::LINEAR_LEVEL_TURBO] = linearTurbo;
}

double Steerer::getVelocity(LinearVelocityLevel velocityLevel) {
  return linearVelocityValues[velocityLevel];
}

void Steerer::move(double marchVelocity, double lagVelocity, int durationSeconds) {
  stingray_msgs::SetLagAndMarch lagAndMarch;
  lagAndMarch.request.march = marchVelocity;
  lagAndMarch.request.lag = lagVelocity;

  // TODO: Add error handling
  ros::service::call(SET_LAG_AND_MARCH_SERVICE, lagAndMarch);

  ros::Duration(durationSeconds, 0).sleep();

  lagAndMarch.request.lag = 0.0;
  lagAndMarch.request.march = 0.0;
  ros::service::call(SET_LAG_AND_MARCH_SERVICE, lagAndMarch);
}

void Steerer::moveMarch(int durationSeconds, LinearVelocityLevel velocityLevel, MarchDirection direction) {
  double speed = getVelocity(velocityLevel);
  if (direction == MarchDirection::MARCH_BACKWARDS)
    speed = -speed;
  move(speed, 0.0, durationSeconds);
}

void Steerer::moveMarchForward(int durationSeconds, LinearVelocityLevel velocityLevel) {
  moveMarch(durationSeconds, velocityLevel, MarchDirection::MARCH_FORWARD);
}

void Steerer::moveMarchBackwards(int durationSeconds, LinearVelocityLevel velocityLevel) {
  moveMarch(durationSeconds, velocityLevel, MarchDirection::MARCH_BACKWARDS);
}

void Steerer::moveLag(int durationSeconds, LinearVelocityLevel velocityLevel, LagDirection direction) {
  double speed = getVelocity(velocityLevel);
  if (direction == LagDirection::LAG_LEFT)
    speed = -speed;
  move(0.0, speed, durationSeconds);
}

void Steerer::moveLagRight(int durationSeconds, LinearVelocityLevel velocityLevel) {
  moveLag(durationSeconds, velocityLevel, LagDirection::LAG_RIGHT);
}

void Steerer::moveLagLeft(int durationSeconds, LinearVelocityLevel velocityLevel) {
  moveLag(durationSeconds, velocityLevel, LagDirection::LAG_LEFT);
}

void Steerer::stop() {
  stingray_msgs::SetLagAndMarch lagAndMarch;
  lagAndMarch.request.march = 0.0;
  lagAndMarch.request.lag = 0.0;
  ros::service::call(SET_LAG_AND_MARCH_SERVICE, lagAndMarch);
}

void Steerer::diveToDepth(int depthCm) {
  bool depthReached = false;
  boost::function<void (const std_msgs::UInt32::ConstPtr&)> callback =
      [&depthReached, depthCm](const std_msgs::UInt32::ConstPtr& depthMessage) {
    if (!depthReached) {
      int currentDepth = static_cast<int>((*depthMessage).data);
      depthReached = std::abs(depthCm - currentDepth) < DEPTH_DELTA;
    }
  };

  // TODO: Add error handling
  stingray_msgs::SetInt32 depthToSet;
  depthToSet.request.value = depthCm;
  ros::service::call(SET_DEPTH_SERVICE, depthToSet);

  ros::Subscriber sub = nodeHandle.subscribe(DEPTH_SUBSCRIBE_TOPIC, 1, callback);
  while (!depthReached);
  sub.shutdown();
}

void Steerer::rotateToAngle(int angleDegree) {
  bool yawReached = false;
  boost::function<void (const std_msgs::Int32::ConstPtr&)> callback =
      [&yawReached, angleDegree](const std_msgs::Int32::ConstPtr& yawMessage) {
        if (!yawReached) {
          int currentYaw = (*yawMessage).data;
          yawReached = std::abs(angleDegree - currentYaw) < YAW_DELTA;
        }
  };

  // TODO: Add error handling
  stingray_msgs::SetInt32 yawToSet;
  yawToSet.request.value = angleDegree;
  ros::service::call(SET_YAW_SERVICE, yawToSet);

  ros::Subscriber sub = nodeHandle.subscribe(YAW_SUBSCRIBE_TOPIC, 1, callback);
  while (!yawReached);
  sub.shutdown();
}