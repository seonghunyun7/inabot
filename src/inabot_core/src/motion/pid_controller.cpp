#include "motion/pid_controller.hpp"

namespace motion {

/**
 * @brief PID 파라미터를 초기화합니다.
 * 
 * @param kp 비례 게인
 * @param ki 적분 게인
 * @param kd 미분 게인
 * @param i_max 적분 최대값 (anti-windup)
 * @param i_min 적분 최소값 (anti-windup)
 */
void PidController::init(double kp, double ki, double kd, double i_max, double i_min)
{
    pid_.initPid(kp, ki, kd, i_max, i_min);
}

/**
 * @brief 목표값과 현재값의 차이를 기반으로 제어 출력을 계산합니다.
 * 
 * @param target 목표값 (예: 속도 또는 위치)
 * @param current 현재값 (예: 현재 속도 또는 위치)
 * @param dt 시간 간격 (초 단위)
 * @return double PID 제어 출력값
 */
double PidController::computeCommand(double target, double current, double dt)
{
    double error = target - current;  // 오차 계산

    // dt 초를 마이크로초 단위 uint64_t 로 변환
    uint64_t dt_us = static_cast<uint64_t>(dt * 1e6);

    return pid_.computeCommand(error, dt_us);  // PID 계산
}

/**
 * @brief 내부 상태(예: 적분 누적값 등)를 초기화합니다.
 */
void PidController::reset()
{
    pid_.reset();
}

}  // namespace motion
