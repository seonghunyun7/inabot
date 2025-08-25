#pragma once

#include <cstdint>

namespace motor_kinematics {

class MotorKinematics {
public:
    MotorKinematics(double wheel_radius, double chassis_to_wheel_distance, int pulse_per_revolution, double gear_ratio);

    double pulsesToMeters(int32_t pulses) const;
    int32_t metersToPulses(double meters) const;

    double rpmToLinearVelocity(double rpm) const;
    double rpmToAngularVelocity(double rpm) const;

    // 추가: 펄스 속도(pulses/s) -> 선속도(m/s)
    double pulsesPerSecondToLinearVelocity(double pulses_per_sec) const;

    // 기타 필요 함수 선언

private:
    double wheel_radius_;
    double wheel_separation_;
    int pulse_per_revolution_; //// 1회전당 1024 펄스
    double gear_ratio_;   // 기어비 또는 보정 계수

    double pulse_per_meter_;
    double meter_per_pulse_;
};

}  // namespace motor_kinematics
