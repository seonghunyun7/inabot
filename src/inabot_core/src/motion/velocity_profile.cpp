#include "motion/velocity_profile.hpp"
#include <cmath>
#include <iostream>

namespace motion {

VelocityProfile::VelocityProfile(geometry_msgs::msg::Twist acc, double control_hz)
  : acc(acc), control_hz(control_hz),
    target_speed(std::make_shared<geometry_msgs::msg::Twist>()),
    curret_speed(std::make_shared<geometry_msgs::msg::Twist>()),
    emergency_stop_(false),
    soft_stop_enabled_(false),
    soft_stop_ratio_(1.0)
{
}

VelocityProfile::~VelocityProfile()
{
    target_speed.reset();
    curret_speed.reset();
}

void VelocityProfile::resetSpeed(geometry_msgs::msg::Twist init_speed)
{
    *curret_speed = init_speed;
}

void VelocityProfile::enableSoftStop(bool enable, double ratio)
{
    soft_stop_enabled_ = enable;
    soft_stop_ratio_ = (ratio > 0.0 && ratio <= 1.0) ? ratio : 1.0;
}

void VelocityProfile::emergencyStop()
{
    emergency_stop_ = true;
}

void VelocityProfile::releaseEmergencyStop()
{
    emergency_stop_ = false;
}

double VelocityProfile::calcVel(double current_speed, double acc)
{
    return current_speed + (acc * (1.0 / control_hz));
}

geometry_msgs::msg::Twist VelocityProfile::calc(geometry_msgs::msg::Twist vel)
{
    if (emergency_stop_) {
        curret_speed->linear.x = 0.0;
        curret_speed->linear.y = 0.0;
        curret_speed->angular.z = 0.0;
        *target_speed = *curret_speed;
        return *curret_speed;
    }

    *target_speed = vel;
    double dt = 1.0 / control_hz;

    const double max_linear_speed = 1.0;
    const double max_angular_speed = 1.0;

    curret_speed->linear.x = std::clamp(curret_speed->linear.x, -max_linear_speed, max_linear_speed);
    curret_speed->angular.z = std::clamp(curret_speed->angular.z, -max_angular_speed, max_angular_speed);

    if (!checkAcceleration(*curret_speed, *target_speed)) {
        std::cout << "[VelocityProfile] Acceleration limit exceeded!" << std::endl;
    }

    double check_speed = target_speed->linear.x - curret_speed->linear.x;
    curret_speed->linear.x = std::fabs(check_speed) < acc.linear.x * dt
        ? target_speed->linear.x
        : calcVel(curret_speed->linear.x, std::copysign(acc.linear.x, check_speed));

    check_speed = target_speed->linear.y - curret_speed->linear.y;
    curret_speed->linear.y = std::fabs(check_speed) < acc.linear.y * dt
        ? target_speed->linear.y
        : calcVel(curret_speed->linear.y, std::copysign(acc.linear.y, check_speed));

    check_speed = target_speed->angular.z - curret_speed->angular.z;
    curret_speed->angular.z = std::fabs(check_speed) < acc.angular.z * dt
        ? target_speed->angular.z
        : calcVel(curret_speed->angular.z, std::copysign(acc.angular.z, check_speed));

    if (soft_stop_enabled_) {
        curret_speed->linear.x *= soft_stop_ratio_;
        curret_speed->linear.y *= soft_stop_ratio_;
        curret_speed->angular.z *= soft_stop_ratio_;
    }

    return *curret_speed;
}

bool VelocityProfile::checkAcceleration(geometry_msgs::msg::Twist current_speed,
                                        geometry_msgs::msg::Twist target_speed)
{
    bool result = true;

    result &= std::fabs(target_speed.linear.x - current_speed.linear.x) <= acc.linear.x;
    result &= std::fabs(target_speed.linear.y - current_speed.linear.y) <= acc.linear.y;
    result &= std::fabs(target_speed.angular.z - current_speed.angular.z) <= acc.angular.z;

    return result;
}

geometry_msgs::msg::Twist VelocityProfile::reduceVelocity(geometry_msgs::msg::Twist current_vel, double ratio)
{
    geometry_msgs::msg::Twist reduced_vel;
    reduced_vel.linear.x = current_vel.linear.x * ratio;
    reduced_vel.linear.y = current_vel.linear.y * ratio;
    reduced_vel.angular.z = current_vel.angular.z * ratio;
    return reduced_vel;
}

}  // namespace motion
