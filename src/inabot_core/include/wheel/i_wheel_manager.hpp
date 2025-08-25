// IWheelManager.hpp
// i_wheel_manager.hpp
#pragma once
#include <vector>

class IWheelManager {
public:
    virtual ~IWheelManager() = default;

    // 엔코더 pulse와 속도 벡터 입력 (길이는 휠 개수와 매칭되어야 함)
    virtual void updateWheelData(const std::vector<int>& pulses, const std::vector<double>& velocities) = 0;

    // 바퀴 속도를 기구학에 맞게 반환 (ex: [front_steering_angle, front_velocity, rear_steering_angle, rear_velocity])
    virtual std::vector<double> getWheelVelocities() const = 0;

    // 회전량(pulse) 조회 함수 (속도와 쌍으로 맞춘 벡터 반환)
    virtual std::vector<int> getWheelPulses() const = 0;
};
