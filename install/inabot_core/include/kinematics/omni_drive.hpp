#pragma once
#include "base_kinematics.hpp"
#include <vector>
#include <string>

namespace kinematics {

class OmniDrive : public BaseKinematics
{
public:
    OmniDrive() = default;

    std::vector<double> twistToWheelVelocities(double linear_x, double linear_y, double angular_z) const override;
    void wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                double& linear_x, double& linear_y, double& angular_z) const override;
    std::string getDriveType() const override { return "OmniDrive"; }

   // wheel_velocities → CAN 프레임 변환 함수 추가
    std::vector<uint8_t> convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const override;
};

} // namespace kinematics
