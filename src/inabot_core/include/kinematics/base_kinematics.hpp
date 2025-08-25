#ifndef BASE_KINEMATICS_HPP_
#define BASE_KINEMATICS_HPP_

#include <string>
#include <vector>
#include <cmath>

namespace kinematics {

/**
 * @brief Base class for kinematics models
 */
class BaseKinematics
{
public:
    virtual ~BaseKinematics() = default;

    /**
     * @brief 변속 명령을 받아 각 바퀴 속도 계산
     * @param linear_x 직진 속도 (m/s)
     * @param linear_y 측면 속도 (m/s) — 메카넘용 등
     * @param angular_z 회전 속도 (rad/s)
     * @return 각 바퀴 속도 (rad/s)
     */
    virtual std::vector<double> twistToWheelVelocities(double linear_x, double linear_y, double angular_z) const = 0;

    /**
     * @brief 바퀴 속도로부터 실제 선속도 및 각속도 추정
     * @param wheel_velocities 각 바퀴 속도 (rad/s)
     * @param linear_x 출력: 선속도 x
     * @param linear_y 출력: 선속도 y
     * @param angular_z 출력: 각속도 z
     */
    virtual void wheelVelocitiesToTwist(const std::vector<double>& wheel_velocities,
                                        double& linear_x, double& linear_y, double& angular_z) const = 0;

    //   wheel_velocities → CAN frame 변환
    virtual std::vector<uint8_t> convertWheelVelocitiesToCanFrame(const std::vector<double>& wheel_velocities) const = 0;

    /**
     * @brief 현재 구동 방식 타입 반환 (예: "DifferentialDrive")
     */
    virtual std::string getDriveType() const = 0;


    /**
     * @brief 펄스를 거리로 변환 (기본 구현)
     * @param pulse 인코더 펄스 수
     * @param ppr 펄스 퍼 레볼루션
     * @param slip_factor 각 지면별로 실험해서 보정 계수
     * @return 이동 거리 (m)
     */
    virtual double pulseToDistance(int pulse, int ppr, double slip_factor = 1.0) const {
        double circumference = 2.0 * M_PI * wheel_radius_;
        return (static_cast<double>(pulse) / ppr) * circumference * slip_factor;
    }

    /**
     * @brief 이동 거리를 펄스로 변환 (기본 구현)
     * @param distance 이동 거리 (m)
     * @param ppr 펄스 퍼 레볼루션
     * @return 인코더 펄스 수
     */
    virtual int distanceToPulse(double distance, int ppr) const {
        double circumference = 2.0 * M_PI * wheel_radius_;
        return static_cast<int>((distance / circumference) * ppr);
    }

protected:
    BaseKinematics() = default;

    // 예: 바퀴 반경, 차폭 (상속 클래스에서 접근 가능)
    double wheel_radius_ = 0.05;     // 기본값 (m)
    double wheel_base_ = 0.3;        // 바퀴 사이 거리 (m)
    double wheel_separation_ = 0.3;  // (옵션) 좌우 바퀴 간 거리 등

public:
    void setWheelRadius(double r) { wheel_radius_ = r; }
    void setWheelBase(double b) { wheel_base_ = b; }
    void setWheelSeparation(double s) { wheel_separation_ = s; }

    double getWheelRadius() const { return wheel_radius_; }
    double getWheelBase() const { return wheel_base_; }
    double getWheelSeparation() const { return wheel_separation_; }
};

}  // namespace kinematics

#endif  // BASE_KINEMATICS_HPP_
