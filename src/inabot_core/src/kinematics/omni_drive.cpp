#include "omni_drive.hpp"
#include <cmath>

namespace kinematics {

// 3륜 옴니 로봇 예시, 120도 간격 바퀴 기준

std::vector<double> OmniDrive::twistToWheelVelocities(double linear_x, double linear_y, double angular_z) const
{
    double r = wheel_radius_;
    double L = wheel_base_;

    double w1 = (1.0 / r) * (linear_x - L * angular_z);
    double w2 = (1.0 / r) * (-0.5 * linear_x + std::sqrt(3)/2 * linear_y - L * angular_z);
    double w3 = (1.0 / r) * (-0.5 * linear_x - std::sqrt(3)/2 * linear_y - L * angular_z);

    return {w1, w2, w3};
}

void OmniDrive::wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                      double& linear_x, double& linear_y, double& angular_z) const
{
    if (wheel_velocities.size() != 3) {
        linear_x = 0; linear_y = 0; angular_z = 0;
        return;
    }

    double r = wheel_radius_;
    double L = wheel_base_;

    linear_x = (r / 3.0) * (wheel_velocities[0] - 0.5 * wheel_velocities[1] - 0.5 * wheel_velocities[2]);
    linear_y = (r / std::sqrt(3)) * (wheel_velocities[1] - wheel_velocities[2]);
    angular_z = (r / (3.0 * L)) * (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2]);
}

std::vector<uint8_t> OmniDrive::convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const
{
    std::vector<uint8_t> can_data(8, 0);

    if (wheel_velocities.size() != 3) {
        return can_data;
    }

    auto encodeInt16 = [](double val) -> std::pair<uint8_t, uint8_t> {
        int16_t tmp = static_cast<int16_t>(val * 1000);
        return {static_cast<uint8_t>((tmp >> 8) & 0xFF), static_cast<uint8_t>(tmp & 0xFF)};
    };

    // 각 바퀴 속도 인코딩
    auto [w1_hi, w1_lo] = encodeInt16(wheel_velocities[0]);
    auto [w2_hi, w2_lo] = encodeInt16(wheel_velocities[1]);
    auto [w3_hi, w3_lo] = encodeInt16(wheel_velocities[2]);

    can_data[0] = w1_hi;
    can_data[1] = w1_lo;
    can_data[2] = w2_hi;
    can_data[3] = w2_lo;
    can_data[4] = w3_hi;
    can_data[5] = w3_lo;

    // can_data[6], can_data[7] 은 0으로 유지

    return can_data;
}

}  // namespace kinematics
