#ifndef MOTION__PID_CONTROLLER_HPP_
#define MOTION__PID_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <control_toolbox/pid.hpp>

namespace motion {

class PidController
{
public:
    PidController() = default;
    ~PidController() = default;

    /**
     * @brief PID 파라미터 초기화
     */
    void init(double kp, double ki, double kd, double i_max = 1.0, double i_min = -1.0);

    /**
     * @brief 제어 출력 계산
     * @param target 목표값
     * @param current 현재값
     * @param dt 제어 주기 (초)
     * @return 제어 명령 (예: 속도, 토크)
     */
    double computeCommand(double target, double current, double dt);

    /**
     * @brief 내부 PID 상태 초기화
     */
    void reset();

private:
    control_toolbox::Pid pid_;
};

}  // namespace motion

#endif  // MOTION__PID_CONTROLLER_HPP_