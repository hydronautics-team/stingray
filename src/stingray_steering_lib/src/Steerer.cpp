#include "Steerer.h"

#include <ros/ros.h>

Steerer::Steerer(double linearSlow, double linearMedium, double linearFast, double linearTurbo) {
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