#pragma once
#include "base_kinematics.hpp"
#include <vector>
#include <cmath>

namespace kinematics {

class SteerableWheelDrive : public BaseKinematics
{
public:
    SteerableWheelDrive() = default;
    virtual ~SteerableWheelDrive() = default;

    // twistToWheelVelocities: 각 바퀴 (전방, 후방) 순서로 [steering_angle(rad), wheel_velocity(m/s)] * 2개를
    // vector<double> 로 리턴 (순서: front_steer_angle, front_wheel_vel, rear_steer_angle, rear_wheel_vel)
    std::vector<double> twistToWheelVelocities(double linear_x, double /*linear_y*/, double angular_z) const override;

    // wheelVelocitiesToTwist: vector<double>에서 위 순서대로 값을 읽어 twist 재계산
    void wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                double& linear_x, double& linear_y, double& angular_z) const override;

    std::string getDriveType() const override { return "SteerableWheelDrive"; }

    // wheel_velocities → CAN 프레임 변환 함수 추가
    std::vector<uint8_t> convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const override;
};

}  // namespace kinematics
