#ifndef STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
#define STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_

#include <geometry_msgs/Twist.h>

typedef enum {
  LINEAR_LEVEL_SLOW,
  LINEAR_LEVEL_MEDIUM,
  LINEAR_LEVEL_FAST,
  LINEAR_LEVEL_VERY_FAST
} LinearVelocityLevel;

typedef enum {
  ANGULAR_LEVEL_SLOW,
  ANGULAR_LEVEL_MEDIUM,
  ANGULAR_LEVEL_FAST
} AngularVelocityLevel;

static const int LINEAR_VELOCITIES_LEVELS = 4;
static const int ANGULAR_VELOCITIES_LEVELS = 3;

class Steerer {

 protected:

  double linearVelocityValues[LINEAR_VELOCITIES_LEVELS];
  double angularVelocityValues[ANGULAR_VELOCITIES_LEVELS];

  /** Sets twist via specific service */
  void setTwist(geometry_msgs::Twist& twist);

  /** Initializes twist */
  virtual geometry_msgs::Twist createTwist(double x, double y, double z, double roll, double pitch, double yaw) = 0;

 public:

  Steerer(double linearVelocityValues[LINEAR_VELOCITIES_LEVELS], double angularVelocityValues[ANGULAR_VELOCITIES_LEVELS]);
  Steerer(Steerer& other) = default;
  ~Steerer() = default;

  // TODO: Add async API

  void moveMarch(int duration, LinearVelocityLevel velocityLevel);

  void moveLag(int duration, LinearVelocityLevel velocityLevel);

  void rotateToAngle(int angle, AngularVelocityLevel velocityLevel);

  void diveToDepth(int depth);

};

#endif //STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
