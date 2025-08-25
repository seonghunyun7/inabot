#ifndef MOTION__VELOCITY_PROFILE_HPP_
#define MOTION__VELOCITY_PROFILE_HPP_

#include <memory>
#include <geometry_msgs/msg/twist.hpp>

namespace motion {

class VelocityProfile
{
public:
  VelocityProfile(geometry_msgs::msg::Twist acc, double control_hz);
  ~VelocityProfile();

  void resetSpeed(geometry_msgs::msg::Twist init_speed);
  void enableSoftStop(bool enable, double ratio = 1.0);
  void emergencyStop();
  void releaseEmergencyStop();

  geometry_msgs::msg::Twist calc(geometry_msgs::msg::Twist vel);
  geometry_msgs::msg::Twist reduceVelocity(geometry_msgs::msg::Twist current_vel, double ratio);

private:
  double calcVel(double current_speed, double acc);
  bool checkAcceleration(geometry_msgs::msg::Twist current_speed,
                         geometry_msgs::msg::Twist target_speed);

  geometry_msgs::msg::Twist acc;
  double control_hz;

  std::shared_ptr<geometry_msgs::msg::Twist> target_speed;
  std::shared_ptr<geometry_msgs::msg::Twist> curret_speed;

  bool emergency_stop_;
  bool soft_stop_enabled_;
  double soft_stop_ratio_;
};

}  // namespace motion

#endif  // MOTION__VELOCITY_PROFILE_HPP_
