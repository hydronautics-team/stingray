#ifndef STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
#define STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_

#include <geometry_msgs/Twist.h>

typedef enum {
  MARCH_FORWARD, // Forward
  MARCH_BACKWARDS, // Backwards
  LAG_RIGHT, // Right
  LAG_LEFT, // Left
  DEPTH_UP, // Up
  DEPTH_DOWN, // Down
  YAW_CW, // Turn clockwise
  YAW_CCW, // Turn counterclockwise
  STOP // Stop
} Direction;

typedef enum {
  LINEAR_LEVEL_1,
  LINEAR_LEVEL_2,
  LINEAR_LEVEL_3,
  LINEAR_LEVEL_4,
  ANGULAR_LEVEL_1,
  ANGULAR_LEVEL_2,
  ANGULAR_LEVEL_3,
  ANGULAR_LEVEL_4
} VelocityLevel;

class Steerer {

 protected:

  double linearVelocityValues[4];
  double angularVelocityValues[4];

  /** Initializes twist */
  geometry_msgs::Twist createTwist(double x, double y, double z, double roll, double pitch, double yaw);

  /** Initializes twist with zero angular velocities */
  geometry_msgs::Twist createLinearTwist(double x, double y, double z);

  /** Initializes twist with zero linear velocities */
  geometry_msgs::Twist createAngularTwist(double roll, double pitch, double yaw);

  /** Initializes twist for specified direction and velocity level */
  geometry_msgs::Twist createDirectionTwist(Direction direction, VelocityLevel velocityLevel);

  /** Creates twist to stop vehicle */
  geometry_msgs::Twist createStopTwist();

  /** Publishes twist to specific topic */
  void publishTwist(geometry_msgs::Twist& twist, std::string& topic);

  /** Initializes twist for specified direction and velocity */
  virtual geometry_msgs::Twist createDirectionTwist(Direction direction, double velocity) = 0;

 public:

  Steerer(double linearVelocityValues[4], double angularVelocityValues[4]);
  Steerer(Steerer& other) = default;
  ~Steerer() = default;

  void moveMarch(int duration);

  void moveLag(int duration);

  void rotateToAngle(int angle);

  void diveToDepth(int depth);

};

#endif //STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
