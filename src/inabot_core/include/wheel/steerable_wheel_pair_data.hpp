// SteerableWheelPairData.hpp
// steerable_wheel_pair_data.hpp
#pragma once
//#include "wheel/WheelData.hpp"
#include "wheel/wheel_data.hpp"

struct SteerableWheelPairData {
    WheelData traction;  // 구동 바퀴 데이터
    WheelData steering;  // 조향 바퀴 데이터
};
