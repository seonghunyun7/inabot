#include "steerable_wheel_drive.hpp"
#include <cmath>

#include <vector>
#include <cstdint>
#include <utility>

namespace kinematics {

std::vector<double> SteerableWheelDrive::twistToWheelVelocities(double linear_x, double /*linear_y*/, double angular_z) const
{
    double half_base = wheel_base_ / 2.0;

    // 앞바퀴 속도 및 조향각
    double front_wheel_velocity = linear_x + angular_z * half_base;
    double front_steering_angle = std::atan2(angular_z * half_base, linear_x);

    // 뒷바퀴 속도 및 조향각
    double rear_wheel_velocity = linear_x - angular_z * half_base;
    double rear_steering_angle = std::atan2(-angular_z * half_base, linear_x);

    return {front_steering_angle, front_wheel_velocity,
            rear_steering_angle,  rear_wheel_velocity};
}

void SteerableWheelDrive::wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                                 double& linear_x, double& linear_y, double& angular_z) const
{
    if (wheel_velocities.size() != 4) {
        linear_x = 0;
        linear_y = 0;
        angular_z = 0;
        return;
    }

    double front_steering_angle = wheel_velocities[0];
    double front_wheel_velocity = wheel_velocities[1];
    double rear_steering_angle = wheel_velocities[2];
    double rear_wheel_velocity = wheel_velocities[3];

    // 선속도 계산 (두 바퀴 속도 및 조향각 평균)
    linear_x = (front_wheel_velocity * std::cos(front_steering_angle) +
                rear_wheel_velocity * std::cos(rear_steering_angle)) / 2.0;

    // 측면 속도 없음 (2륜 조향 기반)
    linear_y = 0.0;

    // 각속도 계산 (두 바퀴 조향각에 따른 y방향 성분 차)
    double half_base = wheel_base_ / 2.0;
    angular_z = (front_wheel_velocity * std::sin(front_steering_angle) -
                 rear_wheel_velocity * std::sin(rear_steering_angle)) / wheel_base_;
}

std::vector<uint8_t> SteerableWheelDrive::convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const
{
    std::vector<uint8_t> can_data(16, 0);
    if (wheel_velocities.size() < 4) {
        // 로그 경고
        return can_data;
    }

    // wheel_velocities: {front_steering_angle, front_wheel_velocity, rear_steering_angle, rear_wheel_velocity}
    // 각도는 rad 단위, 속도는 m/s (예상)
    // 여기서 각도를 degree 혹은 적절한 단위로 변환할 수도 있음

    // 예: steering angle * 1000 → int16_t, velocity * 1000 → int16_t 변환 (임의 스케일)
    //상위 8비트만 추출 =>  하위 8비트만 추출
    auto encodeInt16 = [](double val) -> std::pair<uint8_t, uint8_t> {
        int16_t tmp = static_cast<int16_t>(val * 1000);
        return {static_cast<uint8_t>((tmp >> 8) & 0xFF), static_cast<uint8_t>(tmp & 0xFF)};
    };

    // front steering angle
    auto [fsa_h, fsa_l] = encodeInt16(wheel_velocities[0]);
    // front wheel velocity
    auto [fwv_h, fwv_l] = encodeInt16(wheel_velocities[1]);
    // rear steering angle
    auto [rsa_h, rsa_l] = encodeInt16(wheel_velocities[2]);
    // rear wheel velocity
    auto [rwv_h, rwv_l] = encodeInt16(wheel_velocities[3]);

    can_data[0] = fsa_h; can_data[1] = fsa_l;
    can_data[2] = fwv_h; can_data[3] = fwv_l;
    can_data[4] = rsa_h; can_data[5] = rsa_l;
    can_data[6] = rwv_h; can_data[7] = rwv_l;

    // 나머지 바이트 0으로 유지
    return can_data;
}


}  // namespace kinematics
