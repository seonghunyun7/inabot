// SteerableWheelManager.hpp
// steerable_wheel_manager.hpp
#pragma once
#include "wheel/i_wheel_manager.hpp"
#include "wheel/steerable_wheel_pair_data.hpp"
#include <vector>

class SteerableWheelManager : public IWheelManager {
public:
    void updateWheelData(const std::vector<int>& pulses, const std::vector<double>& velocities) override;

    void setSteeringAngles(const std::vector<double>& angles);  // 각도 rad 단위

    std::vector<double> getWheelVelocities() const override;

    std::vector<int> getWheelPulses() const override;

private:
    std::vector<SteerableWheelPairData> wheels_;  // 전방, 후방 쌍
    std::vector<double> steering_angles_;         // 각 바퀴 조향 각도 (rad)
};
