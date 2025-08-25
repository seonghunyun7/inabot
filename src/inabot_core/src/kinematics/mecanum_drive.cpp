#include "mecanum_drive.hpp"
#include <cmath>

namespace kinematics {

std::vector<double> MecanumDrive::twistToWheelVelocities(double linear_x, double linear_y, double angular_z) const
{
    double L = wheel_base_;
    double W = wheel_separation_;
    double r = wheel_radius_;

    double v1 = (1.0 / r) * (linear_x - linear_y - (L + W) * angular_z);
    double v2 = (1.0 / r) * (linear_x + linear_y + (L + W) * angular_z);
    double v3 = (1.0 / r) * (linear_x + linear_y - (L + W) * angular_z);
    double v4 = (1.0 / r) * (linear_x - linear_y + (L + W) * angular_z);

    return {v1, v2, v3, v4};
}

void MecanumDrive::wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                         double& linear_x, double& linear_y, double& angular_z) const
{
    if (wheel_velocities.size() != 4) {
        linear_x = 0; linear_y = 0; angular_z = 0;
        return;
    }

    double r = wheel_radius_;
    double L = wheel_base_;
    double W = wheel_separation_;

    linear_x = (r / 4.0) * (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]);
    linear_y = (r / 4.0) * (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]);
    angular_z = (r / (4.0 * (L + W))) * (-wheel_velocities[0] + wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3]);
}

std::vector<uint8_t> MecanumDrive::convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const
{
    std::vector<uint8_t> can_data(8, 0);

    if (wheel_velocities.size() != 4) {
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
    auto [w4_hi, w4_lo] = encodeInt16(wheel_velocities[3]);

    can_data[0] = w1_hi;
    can_data[1] = w1_lo;
    can_data[2] = w2_hi;
    can_data[3] = w2_lo;
    can_data[4] = w3_hi;
    can_data[5] = w3_lo;
    can_data[6] = w4_hi;
    can_data[7] = w4_lo;

    return can_data;
}

}  // namespace kinematics
