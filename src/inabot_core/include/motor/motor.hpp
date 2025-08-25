#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>
#include <mutex>

#include <linux/can.h>
//#include "can_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <std_msgs/msg/float32.hpp>

#include "kinematics/kinematics_factory.hpp"
#include "kinematics/base_kinematics.hpp"
#include "motor/kinematics/motor_kinematics.hpp"

#include "motion/pid_controller.hpp"
#include "motion/velocity_profile.hpp"  // 추가

#include "motor/controller/motor_controller_factory.hpp"
#include "motor/can_driver/can_driver_factory.hpp"

#include "motor/utils/motor_load_monitor.hpp"

#include "motor_encoder_data.hpp"  // MotorEncoderData 정의된 헤더 파일 
#include "motor_node_id.hpp"

using namespace Inabot;
using Inabot::IMotorController;

class MotorDriver : public rclcpp::Node
{
public:
    explicit MotorDriver(const rclcpp::NodeOptions& options);

    bool init();

private:
    void initParameters();

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void encoderTimerCallback();
    void loadMonitorTimerCallback();

    // motion_type 별 처리 함수 선언
    void processPidMotion(const std::vector<double>& wheel_velocities);
    void processDirectMotion(const std::vector<double>& wheel_velocities);

    std::vector<std::pair<NodeId, std::vector<uint8_t>>> convertPidOutputToCanFrame(double traction_output, double steering_output);

    MotorEncoderData parseEncoderFrame(const struct can_frame& frame);

    // motion_type 별 처리 함수 타입
    using MotionFunc = std::function<void(const std::vector<double>& wheel_velocities)>;

    // motion_type 별 처리 함수 맵
    std::unordered_map<std::string, MotionFunc> motion_func_map_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr load_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr encoder_timer_;

    //std::shared_ptr<CanInterface> socket_can_driver_;
    std::unique_ptr<kinematics::BaseKinematics> kinematics_;
    std::shared_ptr<motor_kinematics::MotorKinematics> motor_kinematics_;
    std::shared_ptr<IMotorController> motor_controller_;
    std::shared_ptr<ICanInterface> can_driver_;

    MotorLoadMonitor load_monitor_;

    // 위치+속도 모두 포함하는 구조체
    //MotorEncoderData current_wheel_data_;

    // PID 제어용, m/s 단위 속도 벡터 (보통 2개: left, right)
    std::vector<double> current_wheel_velocities_;

    std::string interface_name_;
    std::string can_type_;
    std::string drive_type_;
    std::string controller_type_;  // 드라이버 타입을 저장하는 멤버 변수
    std::string motion_type_;      // 제어 모션 타입 ("pid", "open_loop", "mpc" 등)
    //Kinematics 파라미터
    double wheel_radius_;
    double wheel_base_;
    double wheel_separation_;

    // MotorKinematics
    int pulse_per_revolution_;
    double gear_ratio_;
    bool use_motor_load_monitor_;

    bool use_velocity_profile_ = false;
    double control_dt_;  // 예: 0.02 (초)

    // PID 컨트롤러 객체 (motion_type이 "pid"일 때 사용)
    motion::PidController traction_pid_controller_;
    motion::PidController steering_pid_controller_;

    struct PidParams
    {
        double p;
        double i;
        double d;
        double i_clamp;
        double max;
    };

    PidParams traction_pid_;
    PidParams steering_pid_;
 
    //VelocityProfile 추가
    std::shared_ptr<motion::VelocityProfile> velocity_profile_;

    double acc_linear_x_;
    double acc_linear_y_;
    double control_hz_;

    // MotorDriver 클래스 내부 멤버 변수
    std::mutex encoder_data_mutex_;

    MotorEncoderData front_encoder_data_;  // 전방 휠 데이터 저장용
    MotorEncoderData rear_encoder_data_;   // 후방 휠 데이터 저장용

    // 부하 계산용 복사본 저장 공간
    std::vector<double> latest_distances_;   ///< m 단위 누적 거리
    std::vector<double> latest_velocities_;  ///< m/s 단위 속도
    rclcpp::TimerBase::SharedPtr load_monitor_timer_;
};

#endif  // MOTOR_HPP_
