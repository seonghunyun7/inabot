#include "motor/kinematics/motor_kinematics.hpp"
#include <cmath>

namespace motor_kinematics {

MotorKinematics::MotorKinematics(double wheel_radius, double chassis_to_wheel_distance, int pulse_per_revolution, double gear_ratio)
    : wheel_radius_(wheel_radius),
      wheel_separation_(chassis_to_wheel_distance * 2.0),
      pulse_per_revolution_(pulse_per_revolution),
      gear_ratio_(gear_ratio)
{
    //모터가 1회전할 때 엔코더가 발생하는 펄스 수(pulse_per_revolution_)에 기어비(gear_ratio_)를 곱해 휠 1회전 당 펄스 수를 산출하는 방식
    pulse_per_meter_ = gear_ratio_ * pulse_per_revolution_ / (2.0 * M_PI * wheel_radius_);
    
    meter_per_pulse_ = 1.0 / pulse_per_meter_;
}

//펄스 → 미터 변환
double MotorKinematics::pulsesToMeters(int32_t pulses) const {
    return pulses * meter_per_pulse_;
}

//미터 → 펄스 변환
int32_t MotorKinematics::metersToPulses(double meters) const {
    return static_cast<int32_t>(meters * pulse_per_meter_);
}

// RPM → m/s 변환
//공식: v=ω×r= (rpm×2π60×r)/60
double MotorKinematics::rpmToLinearVelocity(double rpm) const {
    return (rpm * 2.0 * M_PI / 60.0) * wheel_radius_;
}

double MotorKinematics::rpmToAngularVelocity(double rpm) const {
    // rpm을 rad/s로 변환: rpm * (2π / 60)
    return rpm * 2.0 * M_PI / 60.0;
}

//펄스/초 → m/s 변환
double MotorKinematics::pulsesPerSecondToLinearVelocity(double pulses_per_sec) const {
    return pulses_per_sec * meter_per_pulse_;
}

}  // namespace motor_kinematics
