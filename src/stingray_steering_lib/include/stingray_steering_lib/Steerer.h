#ifndef STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
#define STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <stingray_msgs/SetLagAndMarch.h>
#include <ros/node_handle.h>

/**
 * Represents available linear velocity levels.
 */
typedef enum {
  LINEAR_LEVEL_SLOW,
  LINEAR_LEVEL_MEDIUM,
  LINEAR_LEVEL_FAST,
  LINEAR_LEVEL_TURBO
} LinearVelocityLevel;

/**
 * Represents possible march directions.
 */
typedef enum {
  MARCH_FORWARD,
  MARCH_BACKWARDS
} MarchDirection;

/**
 * Reprsenets possible lag directions.
 */
typedef enum {
  LAG_RIGHT,
  LAG_LEFT
} LagDirection;

/**
 * <p>Class provides API (steering API) to control vehicle's movement.</p>
 *
 * <p>Users of Stingray platform should use this class to perform all actions,
 * related to movement of the vehicle. Underlying service calls are not intended
 * to be called by users of the platform.</p>
 *
 * <p>Currently, this class provides following movement actions:
 * <ul>
 *  <li>March moving for specified time with specified velocity level</li>
 *  <li>Lag moving for specified time with specified velocity level</li>
 *  <li>Dive to specified depth</li>
 *  <li>Rotate to specified angle</li>
 * </ul></p>
 *
 * <p>Rotation and diving use low-level stabilization contours and feedback loops.</p>
 *
 * <p>There are 4 available velocity levels: slow, medium, fast and turbo. Users should
 * specify velocity values for those levels when constructing instances.</p>
 *
 * <p>Currently, there is no synchronization in steering mechanism, so calling steering API
 * from multiple threads or processes at the same time can cause big problem. Users should
 * avoid such situations by themselves. The safe scenario is when one node uses steering API
 * to perform some actions, only after that another node can use API and so on - two or more
 * nodes should not use API simultaneously.</p>
 */
class Steerer {

 private:

  static const std::string DEPTH_SUBSCRIBE_TOPIC;
  static const std::string YAW_SUBSCRIBE_TOPIC;

  static const std::string SET_LAG_AND_MARCH_SERVICE;
  static const std::string SET_DEPTH_SERVICE;
  static const std::string SET_YAW_SERVICE;

  static const int LINEAR_VELOCITY_LEVELS = 4;

  static const int DEPTH_DELTA = 10;
  static const int YAW_DELTA = 5;

  double linearVelocityValues[LINEAR_VELOCITY_LEVELS];

 protected:

  ros::NodeHandle nodeHandle;

  /**
   * Obtains velocity value for specified velocity level.
   *
   * @param velocityLevel Velocity level
   *
   * @return Value for specified velocity level
   */
  double getVelocity(LinearVelocityLevel velocityLevel);

  /**
   * Moves vehicle with specified march and lag velocities
   * during specified time. Blocks calling thread until
   * action is not completed.
   *
   * @param marchVelocity March velocity, not negative
   * @param lagVelocity Lag velocity, not negative
   * @param durationSeconds Duration in seconds, not negative
   */
  void move(double marchVelocity, double lagVelocity, int durationSeconds);

 public:

  /**
   * @param nodeHandle Node handle instance
   * @param linearSlow Velocity value for slow level, not negative
   * @param linearMedium Velocity value for medium level, not negative
   * @param linearFast Velocity value for fast level, not negative
   * @param linearTurbo Velocity value for turbo level, not negative
   */
  Steerer(const ros::NodeHandle& nodeHandle, double linearSlow, double linearMedium,
      double linearFast, double linearTurbo);

  Steerer(Steerer& other) = default;
  ~Steerer() = default;

  // TODO: Add non-blocking API

  /**
   * Moves vehicle by march during specified time. Blocks calling thread
   * until action is not completed.
   *
   * @param durationSeconds Duration in seconds, not negative
   * @param velocityLevel Velocity level
   * @param direction Direction of the movement
   */
  void moveMarch(int durationSeconds, LinearVelocityLevel velocityLevel, MarchDirection direction);

  /**
   * Moves vehicle forward by march. Equals to calling:
   * <br><code>moveMarch(duration, level, MarchDirection::MARCH_FORWARD)</code>
   *
   * @param durationSeconds Duration in seconds, not negative
   * @param velocityLevel Velocity level
   */
  void moveMarchForward(int durationSeconds, LinearVelocityLevel velocityLevel);

  /**
   * Moves vehicle backwards by march. Equals to calling:
   * <br><code>moveMarch(duration, level, MarchDirection::MARCH_BACKWARDS)</code>
   *
   * @param durationSeconds Duration in seconds, not negative
   * @param velocityLevel Velocity level
   */
  void moveMarchBackwards(int durationSeconds, LinearVelocityLevel velocityLevel);

  /**
   * Moves vehicle by lag during specified time. Blocks calling thread
   * until action is not performed.
   *
   * @param durationSeconds Duration in seconds, not negative
   * @param velocityLevel Velocity level
   * @param direction Direction of the movement
   */
  void moveLag(int durationSeconds, LinearVelocityLevel velocityLevel, LagDirection direction);

  /**
   * Moves vehicle right by lag. Equals to calling:
   * <br><code>moveLag(duration, level, LagDirection::LAG_RIGHT)</code>
   *
   * @param durationSeconds Duration in seconds, not negative
   * @param velocityLevel Velocity level
   */
  void moveLagRight(int durationSeconds, LinearVelocityLevel velocityLevel);

  /**
   * Moves vehicle left by lag. Equals to calling:
   * <br><code>moveLag(duration, level, LagDirection::LAG_LEFT)</code>
   *
   * @param durationSeconds Duration in seconds, not negative
   * @param velocityLevel Velocity level
   */
  void moveLagLeft(int durationSeconds, LinearVelocityLevel velocityLevel);

  /**
   * Stops vehicle's movement by lag and march.
   */
  void stop();

  /**
   * Rotates vehicle to specified degree by yaw. Blocks calling
   * thread until angle is not reached.
   *
   * @param angleDegree Angle in degree to rotate on, positive - counterclockwise, negative - clockwise.
   */
  void rotateToAngle(int angleDegree);

  /**
   * Dives vehicle on specified depth. Blocks calling thread
   * until depth is not reached.
   *
   * @param depthCm Depth to dive on in centimeters, not negative.
   */
  void diveToDepth(int depthCm);

};

#endif //STINGRAY_SRC_STINGRAY_STEERING_INCLUDE_STEERER_H_
