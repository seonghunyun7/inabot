#pragma once

#include <vector>

/// @brief 모터의 엔코더 기반 부하를 추정하는 클래스
class MotorLoadMonitor {
private:
    std::vector<double> previous_distances_; ///< 이전 주기의 바퀴 누적 거리 값 (m)
    float smoothed_pulse_load_ = 0.0f;       ///< 지연 필터를 적용한 거리 기반 부하 추정 값

public:
    /**
     * @brief 주어진 바퀴 누적 거리 및 속도 값으로부터 모터 부하를 추정
     * 
     * @param distances  현재 바퀴 누적 거리 값 배열 (최소 4개, 단위: m)
     * @param velocities 현재 바퀴 속도 값 배열 (최소 4개, 단위: m/s)
     * @param dt         샘플링 주기 (초)
     * @return 추정된 모터 부하 값 (0 이상 float)
     */
    float calculate(const std::vector<double>& distances,  // m 단위
                    const std::vector<double>& velocities, // m/s 단위
                    double dt);
};
