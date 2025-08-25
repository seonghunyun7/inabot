#include "wheel/differential_wheel_manager.hpp"
#include <cstddef> // <vector>, <cstdlib>

void DifferentialWheelManager::updateWheelData(const std::vector<int>& pulses, const std::vector<double>& velocities) {
    wheels_.clear();
    for (size_t i = 0; i < pulses.size(); ++i) {
        wheels_.push_back(WheelData{pulses[i], velocities[i]});
    }
}

std::vector<double> DifferentialWheelManager::getWheelVelocities() const {
    if (wheels_.size() < 2) return {};
    return { wheels_[0].velocity, wheels_[1].velocity }; // 왼쪽, 오른쪽 바퀴 속도 순서
}

std::vector<int> DifferentialWheelManager::getWheelPulses() const {
    std::vector<int> pulses;
    for (const auto& wheel : wheels_) {
        pulses.push_back(wheel.pulse);
    }
    return pulses;
}