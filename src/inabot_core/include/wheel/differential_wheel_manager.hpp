// DifferentialWheelManager.hpp
// differential_wheel_manager.hpp
#pragma once
#include "wheel/i_wheel_manager.hpp"
//#include "wheel/WheelData.hpp"
#include "wheel/wheel_data.hpp"
#include <vector>

class DifferentialWheelManager : public IWheelManager {
public:
    void updateWheelData(const std::vector<int>& pulses, const std::vector<double>& velocities) override;

    std::vector<double> getWheelVelocities() const override;

    std::vector<int> getWheelPulses() const override;
private:
    std::vector<WheelData> wheels_;  // 왼쪽, 오른쪽 바퀴
};
