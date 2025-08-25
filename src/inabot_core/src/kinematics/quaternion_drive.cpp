#include "quaternion_drive.hpp"
#include <cmath>
#include <vector>
#include <cstdint>
#include <utility>

namespace kinematics {

// 2륜 쿼터니언(예시)
std::vector<double> QuaternionDrive::twistToWheelVelocities(double linear_x, double /*linear_y*/, double angular_z) const
{
    double v_l = (linear_x - (angular_z * wheel_base_ / 2.0)) / wheel_radius_;
    double v_r = (linear_x + (angular_z * wheel_base_ / 2.0)) / wheel_radius_;
    return {v_l, v_r};
}

void QuaternionDrive::wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                            double& linear_x, double& linear_y, double& angular_z) const
{
    if (wheel_velocities.size() != 2) {
        linear_x = 0;
        linear_y = 0;
        angular_z = 0;
        return;
    }

    double v_l = wheel_velocities[0] * wheel_radius_;
    double v_r = wheel_velocities[1] * wheel_radius_;

    linear_x = (v_l + v_r) / 2.0;
    linear_y = 0.0;  // 측방향 없음
    angular_z = (v_r - v_l) / wheel_base_;
}

std::vector<uint8_t> QuaternionDrive::convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const
{
    std::vector<uint8_t> can_data(8, 0);
    if (wheel_velocities.size() < 2) {
        // 로그 경고
        return can_data;
    }

    // differential drive의 경우, wheel_velocities는 [v_l, v_r] (rad/s)
    // 예시: 1000배 스케일 후 int16_t로 변환
    int16_t left_cmd = static_cast<int16_t>(wheel_velocities[0] * 1000);
    int16_t right_cmd = static_cast<int16_t>(wheel_velocities[1] * 1000);

    can_data[0] = (left_cmd >> 8) & 0xFF;
    can_data[1] = left_cmd & 0xFF;
    can_data[2] = (right_cmd >> 8) & 0xFF;
    can_data[3] = right_cmd & 0xFF;

    // 나머지는 0
    return can_data;
}


}  // namespace kinematics
