#include "motor/utils/motor_load_monitor.hpp"
#include <cmath>

float MotorLoadMonitor::calculate(const std::vector<double>& distances,
                                  const std::vector<double>& velocities,
                                  double dt) {
    if (velocities.size() < 4 || distances.size() < 4) return 0.0f;

    if (previous_distances_.empty()) {
        previous_distances_ = distances;
        return 0.0f;
    }

    // 1. 속도 기반 부하 (m/s)
    double front_vel = velocities[0]; // front traction
    double rear_vel  = velocities[2]; // rear traction
    float load_from_velocity = static_cast<float>(std::abs(front_vel - rear_vel));

    // 2. 거리 차이 → 속도(m/s) 변환
    double delta_front = distances[0] - previous_distances_[0];
    double delta_rear  = distances[2] - previous_distances_[2];
    double pulse_diff_mps = std::abs(delta_front - delta_rear) / dt;

    // 3. 지연 필터 -> 1차 지수 이동 평균(EMA, Exponential Moving Average)
    //0.8f → 이전 값(과거) 가중치
    //0.2f → 새 값(현재 측정치) 가중치
    //새로운 측정값이 갑자기 튀어도, **이전 값 80% + 현재 값 20%**만 반영 → 값 변화가 완만해짐
    smoothed_pulse_load_ = 0.8f * smoothed_pulse_load_ + 0.2f * pulse_diff_mps;

    // 4. 최종 부하 : 속도 기반 부하와 거리 기반 부하 -> 최종 부하율
    float combined_load = 0.5f * load_from_velocity + 0.5f * smoothed_pulse_load_;

    combined_load *= 100.0f; ///퍼센트 단위 변환

    // 노이즈 컷오프 (예: 0.01% 이하면 0으로 처리)
    if (combined_load < 0.01f) {
        combined_load = 0.0f;
    }

    previous_distances_ = distances;
    return combined_load;
}
