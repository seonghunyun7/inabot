#include "motor.hpp"
#include <iostream>

#include <utils/logger.h>

#define _LOG_ 0

MotorDriver::MotorDriver(const rclcpp::NodeOptions& options)
: Node("motor_driver", options)
{
    RCLCPP_INFO(this->get_logger(), "MotorDriver 노드 생성");
    
    initParameters();

    can_driver_ = CanDriverFactory::create(can_type_);

    kinematics_ = kinematics::KinematicsFactory::create(drive_type_);
    if (!kinematics_) {
        RCLCPP_ERROR(this->get_logger(), "Unknown drive type: %s", drive_type_.c_str());
        // 필요시 예외처리 또는 노드 종료
    }

    kinematics_->setWheelRadius(wheel_radius_);
    kinematics_->setWheelBase(wheel_base_);
    kinematics_->setWheelSeparation(wheel_separation_);
    
    //motor_kinematics
    motor_kinematics_ = std::make_shared<motor_kinematics::MotorKinematics>(
        wheel_radius_,     // 바퀴 반경
        wheel_base_ / 2.0, // chassis_to_wheel_distance
        pulse_per_revolution_,  // 모터 1회전당 펄스 수
        gear_ratio_        // 기어비 (필요시 조정)
    );

    //if (use_velocity_profile)
    //acc.linear.x	0.1 ~ 0.2 m/s²
    //acc.linear.y	0.02 ~ 0.05 m/s²
    //control_hz	50Hz 이상 유지 (20ms)
    geometry_msgs::msg::Twist acc;
    geometry_msgs::msg::Twist init_speed;
    acc.linear.x = acc_linear_x_;   // x축 직선 가속도 0.5 m/s^2 -> 0.2
    acc.linear.y = acc_linear_y_;   // y축 직선 가속도 0.1 m/s^2 (좌우 이동이 느리게, 부드럽게) -> 0.02

    double control_hz = control_hz_;  // 제어 주기 (50Hz) -> 20ms
    velocity_profile_ = std::make_shared<motion::VelocityProfile>(acc, control_hz);  

    init_speed.linear.x = 0.0;  // 초기 x축 속도
    init_speed.linear.y = 0.0;  // 초기 y축 속도
    init_speed.angular.z = 0.0; // 초기 각속도
    velocity_profile_->resetSpeed(init_speed);
 
    //(motion_type_ == "pid")
    traction_pid_controller_.init(traction_pid_.p, traction_pid_.i, traction_pid_.d,
                                    traction_pid_.i_clamp, -traction_pid_.i_clamp);
    
    steering_pid_controller_.init(steering_pid_.p, steering_pid_.i, steering_pid_.d,
                                    steering_pid_.i_clamp, -steering_pid_.i_clamp);
    
    // motion_type 별 처리 함수 맵 초기화
    motion_func_map_["pid"] = std::bind(&MotorDriver::processPidMotion, this, std::placeholders::_1);
    motion_func_map_["direct"] = std::bind(&MotorDriver::processDirectMotion, this, std::placeholders::_1);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

    // QoS settings for encoder
    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    default_qos.keep_last(10);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10/*default_qos*/);

    load_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor_load", 10);

    control_dt_ = 1.0 / control_hz_; // (e.g., 50.0 = 20ms, 100 = 10ms)
    encoder_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(control_dt_),
        std::bind(&MotorDriver::encoderTimerCallback, this));

    if (use_motor_load_monitor_) {
        load_monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorDriver::loadMonitorTimerCallback, this)
        );
    }
}

void MotorDriver::initParameters()
{
    this->declare_parameter<std::string>("interface_name", "can0");
    this->declare_parameter<std::string>("can_type", "socket");
    this->declare_parameter<std::string>("controller_type", "roboteq");
    this->declare_parameter<std::string>("drive_type", "differential");
    this->declare_parameter<std::string>("motion_type", "pid");
    this->declare_parameter<bool>("use_velocity_profile", false);

    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_base", 0.3);
    this->declare_parameter<double>("wheel_separation", 0.3);

    this->declare_parameter<int>("pulse_per_revolution", 4096);
    this->declare_parameter<double>("gear_ratio", 30.0);
    this->declare_parameter<bool>("use_motor_load_monitor", false);

    this->declare_parameter<double>("pid.traction_pid.p", 1.0);
    this->declare_parameter<double>("pid.traction_pid.i", 0.1);
    this->declare_parameter<double>("pid.traction_pid.d", 0.01);
    this->declare_parameter<double>("pid.traction_pid.i_clamp", 1.0);
    this->declare_parameter<double>("pid.traction_pid.max_linear", 1.0);

    this->declare_parameter<double>("pid.steering_pid.p", 1.5);
    this->declare_parameter<double>("pid.steering_pid.i", 0.2);
    this->declare_parameter<double>("pid.steering_pid.d", 0.02);
    this->declare_parameter<double>("pid.steering_pid.i_clamp", 1.0);
    this->declare_parameter<double>("pid.traction_pid.max_angular", 1.0);

    this->declare_parameter("acc.linear.x", 0.2);
    this->declare_parameter("acc.linear.y", 0.02);
    this->declare_parameter("control_hz", 50.0);


    this->get_parameter("interface_name", interface_name_);
    this->get_parameter("can_type", can_type_);

    this->get_parameter("controller_type", controller_type_);
    this->get_parameter("drive_type", drive_type_);
    this->get_parameter("motion_type", motion_type_);
    this->get_parameter("use_velocity_profile", use_velocity_profile_);

    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("wheel_separation", wheel_separation_);

    this->get_parameter("pulse_per_revolution", pulse_per_revolution_);
    this->get_parameter("gear_ratio", gear_ratio_);
    this->get_parameter("use_motor_load_monitor", use_motor_load_monitor_);

    this->get_parameter("pid.traction_pid.p", traction_pid_.p);
    this->get_parameter("pid.traction_pid.i", traction_pid_.i);
    this->get_parameter("pid.traction_pid.d", traction_pid_.d);
    this->get_parameter("pid.traction_pid.i_clamp", traction_pid_.i_clamp);
    this->get_parameter("pid.traction_pid.max_linear", traction_pid_.max);

    this->get_parameter("pid.steering_pid.p", steering_pid_.p);
    this->get_parameter("pid.steering_pid.i", steering_pid_.i);
    this->get_parameter("pid.steering_pid.d", steering_pid_.d);
    this->get_parameter("pid.steering_pid.i_clamp", steering_pid_.i_clamp);  // 누락된 부분 보완
    this->get_parameter("pid.steering_pid.max_angular", steering_pid_.max);

    this->get_parameter("acc.linear.x", acc_linear_x_);
    this->get_parameter("acc.linear.y", acc_linear_y_);
    this->get_parameter("control_hz", control_hz_);

    RCLCPP_INFO(this->get_logger(), "interface_name_: %s", interface_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "can_type_: %s", can_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "controller_type_: %s", controller_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "drive_type: %s", drive_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "motion_type: %s", motion_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f", wheel_radius_);
    RCLCPP_INFO(this->get_logger(), "wheel_base: %.3f", wheel_base_);
    RCLCPP_INFO(this->get_logger(), "wheel_separation: %.3f", wheel_separation_);
}

bool MotorDriver::init()
{
    try {
        if (!can_driver_ || !can_driver_->initialize(interface_name_)) {
            std::cerr << "[ERROR][Motor] Failed to initialize CAN driver!" << std::endl;
            return false;
        }

        std::cout << "[INFO][Motor] CAN driver initialized successfully." << std::endl;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "SocketCAN 인터페이스 열기 중 예외 발생: %s", e.what());
        return false;
    }

    // 공통 인터페이스로 모터 컨트롤러 생성
    motor_controller_ = MotorControllerFactory::create(controller_type_, can_driver_);
    if (!motor_controller_) {
        RCLCPP_ERROR(this->get_logger(), "모터 컨트롤러 생성 실패 (타입: %s)", controller_type_.c_str());
        return false;
    }

    if (!motor_controller_->initialize()) {
        RCLCPP_ERROR(this->get_logger(), "모터 컨트롤러 초기화 실패");
        //socket_can_driver_->close();  // 필요 시 정리
        //can_driver_->close();  // 필요 시 정리
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//NODE ID	데이터 내용	설명
//CAN ID 구성 요소	설명	예시
//베이스 ID	명령 종류/데이터 타입 구분	0x100: 엔코더 데이터, 0x200: 속도 명령 등
//노드 ID	각 장치별 고유 ID (1~127)	0x01, 0x02, 0x03 등
//확장 플래그 (EFF)	29비트 확장 ID 사용 시 플래그	필요에 따라 설정
//예를 들어,
//    엔코더 데이터 수신 CAN ID: 0x100 + node_id
//    속도 명령 전송 CAN ID: 0x200 + node_id
//2. CAN ID 조합 예시
//uint32_t can_id = BASE_ID_ENCODER + node_id; // 예: 0x100 + 0x01 = 0x101
//3. 데이터 내용 예시
//엔코더 데이터는 보통 다음과 같이 8바이트에 담겨 옵니다:
//바이트	내용	비고
//0~3	엔코더 위치 (int32_t)	회전량, 펄스 수 등
//4~7	엔코더 속도 (int32_t)	속도 값 (펄스/초 등)
//CAN ID: 명령/데이터 타입 + 노드 ID 합친 값 (예: 0x100 + node_id)
//데이터: 엔코더 위치와 속도 4바이트씩 (총 8바이트)
//수신 시: CAN ID에서 노드 ID 추출 후 데이터 파싱 및 저장
//0x01	traction_position (4바이트) + traction_velocity (4바이트)	전륜 견인 바퀴 위치 + 속도
//0x02	steering_position (4바이트) + steering_velocity (4바이트)	전륜 조향 바퀴 위치 + 속도
//0x03	traction_position (4바이트) + traction_velocity (4바이트)	후륜 견인 바퀴 위치 + 속도
//0x04	steering_position (4바이트) + steering_velocity (4바이트)   후륜 조향 바퀴 위치 + 속도
MotorEncoderData MotorDriver::parseEncoderFrame(const struct can_frame& frame)
{
    MotorEncoderData data;

    if (frame.can_dlc < 8) {
        std::cout << "[WARN] CAN 프레임 데이터 부족" << std::endl;
        return data;
    }

    uint8_t node_id = static_cast<uint8_t>(frame.can_id & 0xFF);
#if _LOG_
    std::cout << "[INFO] parseEncoderFrame [CAN ID] 0x" << std::hex << frame.can_id
              << ", [Node ID] 0x" << static_cast<int>(node_id) << std::dec << std::endl;
#endif
    auto parseInt32FromData = [](const uint8_t* d) -> int32_t {
        return static_cast<int32_t>(
            (static_cast<uint32_t>(d[0]) << 24) |
            (static_cast<uint32_t>(d[1]) << 16) |
            (static_cast<uint32_t>(d[2]) << 8)  |
            static_cast<uint32_t>(d[3]));
    };

    switch (static_cast<NodeId>(node_id)) {
        case NodeId::FrontTraction:
            data.channel = EncoderChannel::FRONT;
            data.traction_position_raw = parseInt32FromData(&frame.data[0]);
            data.traction_velocity_raw = parseInt32FromData(&frame.data[4]);
#if _LOG_
            std::cout << "[INFO]   FrontTraction - position_raw: " << data.traction_position_raw
                      << ", velocity_raw: " << data.traction_velocity_raw << std::endl;
#endif
            break;

        case NodeId::FrontSteering:
            data.channel = EncoderChannel::FRONT;
            data.steering_position_raw = parseInt32FromData(&frame.data[0]);
            data.steering_velocity_raw = parseInt32FromData(&frame.data[4]);
#if _LOG_
            std::cout << "[INFO]   FrontSteering - position_raw: " << data.steering_position_raw
                      << ", velocity_raw: " << data.steering_velocity_raw << std::endl;
#endif
            break;

        case NodeId::RearTraction:
            data.channel = EncoderChannel::REAR;
            data.traction_position_raw = parseInt32FromData(&frame.data[0]);
            data.traction_velocity_raw = parseInt32FromData(&frame.data[4]);
#if _LOG_
            std::cout << "[INFO]   RearTraction - position_raw: " << data.traction_position_raw
                      << ", velocity_raw: " << data.traction_velocity_raw << std::endl;
#endif
            break;

        case NodeId::RearSteering:
            data.channel = EncoderChannel::REAR;
            data.steering_position_raw = parseInt32FromData(&frame.data[0]);
            data.steering_velocity_raw = parseInt32FromData(&frame.data[4]);
#if _LOG_
            std::cout << "[INFO]   RearSteering - position_raw: " << data.steering_position_raw
                      << ", velocity_raw: " << data.steering_velocity_raw << std::endl;
#endif
            break;

        default:
        std::cout << "[WARN] 알 수 없는 Node ID: 0x" << std::hex << static_cast<int>(node_id) << std::dec << std::endl;
        break;
    }

    return data;
}

void MotorDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::Twist input_vel = *msg;

    RCLCPP_INFO(this->get_logger(), "[MotorDriver] cmdVelCallback called");

    // 1. VelocityProfile 적용 여부 확인
    if (use_velocity_profile_ && velocity_profile_) {
        input_vel = velocity_profile_->calc(input_vel); // 부드럽게 처리된 속도 반환
    }

    // 2. twist → 바퀴 속도 변환
    //input_vel.linear.x는 로봇의 직진 속도 (m/s),
    //input_vel.angular.z는 로봇의 각속도 (rad/s)
    auto wheel_velocities = kinematics_->twistToWheelVelocities(
        input_vel.linear.x, input_vel.linear.y, input_vel.angular.z);

    // 3. motion_type_ 에 따른 분기 처리 (예: "pid", "direct" 등)
    auto it = motion_func_map_.find(motion_type_);
    if (it != motion_func_map_.end()) {
        // 해당 motion_type 에 매핑된 처리 함수 호출
        it->second(wheel_velocities);
    } else {
        RCLCPP_WARN(this->get_logger(), "알 수 없는 motion_type: %s", motion_type_.c_str());
    }
}

void MotorDriver::loadMonitorTimerCallback()
{
    std::vector<double> distances_copy;
    std::vector<double> velocities_copy;

    {
        std::lock_guard<std::mutex> lock(encoder_data_mutex_);
        distances_copy = latest_distances_;
        velocities_copy = latest_velocities_;
    }

    static rclcpp::Time last_time = this->now();
    rclcpp::Time now_time = this->now();
    double dt = (now_time - last_time).seconds();
    last_time = now_time;

    float load = load_monitor_.calculate(distances_copy, velocities_copy, dt);

    std_msgs::msg::Float32 load_msg;
    load_msg.data = load;
    load_pub_->publish(load_msg);
}

void MotorDriver::encoderTimerCallback()
{
    //struct can_frame frame = socket_can_driver_->readFrame();
    struct can_frame frame = can_driver_->readFrame();
    MotorEncoderData wheel_data = parseEncoderFrame(frame);

    if (wheel_data.channel == EncoderChannel::UNKNOWN) {
        // 알 수 없는 채널, 무시
        return;
    }
  
    // CAN ID 하위 8비트를 노드 ID로 가정
    uint8_t node_id = static_cast<uint8_t>(frame.can_id & 0xFF);
    #if _LOG_
    std::cout << "encoderTimerCallback [Node ID] 0x" << std::hex << static_cast<int>(node_id) << std::dec << std::endl;
    #endif
    double traction_pos_m = 0.0;
    double steering_pos_m = 0.0;
    double traction_vel_mps = 0.0;
    double steering_vel_mps = 0.0;

    //velocity_raw	pulse/sec	엔코더의 원시 속도 값
    //traction_velocity	m/s	변환된 실제 속도
    //position_raw	pulse	누적 펄스 (거리)
    //position	m	변환된 거리
    {
        // 뮤텍스 잠금: 엔코더 데이터 업데이트 중 데이터 보호
        std::lock_guard<std::mutex> lock(encoder_data_mutex_);

        switch (static_cast<NodeId>(node_id)) {
            case NodeId::FrontTraction: // 0x01
                front_encoder_data_.traction_position_raw = wheel_data.traction_position_raw;
                front_encoder_data_.traction_velocity_raw = wheel_data.traction_velocity_raw;

                traction_pos_m = motor_kinematics_->pulsesToMeters(static_cast<int32_t>(wheel_data.traction_position_raw));
                traction_vel_mps = motor_kinematics_->rpmToLinearVelocity(static_cast<int32_t>(wheel_data.traction_velocity_raw));

                front_encoder_data_.traction_position = traction_pos_m;
                front_encoder_data_.traction_velocity = traction_vel_mps;
                #if _LOG_
                std::cout << "[Front Traction] "
                        << "pos_raw=" << front_encoder_data_.traction_position_raw << " pulses, "
                        << "vel_raw=" << front_encoder_data_.traction_velocity_raw << " rpm, "
                        << "pos_m=" << front_encoder_data_.traction_position << " m, "
                        << "vel_mps=" << front_encoder_data_.traction_velocity << " m/s"
                        << std::endl;
                #endif
                break;

            case NodeId::FrontSteering: //0x02
                front_encoder_data_.steering_position_raw = wheel_data.steering_position_raw;
                front_encoder_data_.steering_velocity_raw = wheel_data.steering_velocity_raw;

                steering_pos_m = motor_kinematics_->pulsesToMeters(static_cast<int32_t>(wheel_data.steering_position_raw));
                steering_vel_mps = motor_kinematics_->rpmToLinearVelocity(static_cast<int32_t>(wheel_data.steering_velocity_raw));

                front_encoder_data_.steering_position = steering_pos_m;
                front_encoder_data_.steering_velocity = steering_vel_mps;
                #if _LOG_
                std::cout << "[Front Steering] "
                    << "pos_raw=" << front_encoder_data_.steering_position_raw << " pulses, "
                    << "vel_raw=" << front_encoder_data_.steering_velocity_raw << " rpm, "
                    << "pos_m=" << front_encoder_data_.steering_position << " m, "
                    << "vel_mps=" << front_encoder_data_.steering_velocity << " m/s"
                    << std::endl;
                #endif
                break;

            case NodeId::RearTraction: // 0x03
                rear_encoder_data_.traction_position_raw = wheel_data.traction_position_raw;
                rear_encoder_data_.traction_velocity_raw = wheel_data.traction_velocity_raw;

                traction_pos_m = motor_kinematics_->pulsesToMeters(static_cast<int32_t>(wheel_data.traction_position_raw));
                traction_vel_mps = motor_kinematics_->rpmToLinearVelocity(static_cast<int32_t>(wheel_data.traction_velocity_raw));

                rear_encoder_data_.traction_position = traction_pos_m;
                rear_encoder_data_.traction_velocity = traction_vel_mps;
                #if _LOG_
                std::cout << "[Rear Traction] "
                        << "pos_raw=" << rear_encoder_data_.traction_position_raw << " pulses, "
                        << "vel_raw=" << rear_encoder_data_.traction_velocity_raw << " rpm, "
                        << "pos_m=" << rear_encoder_data_.traction_position << " m, "
                        << "vel_mps=" << rear_encoder_data_.traction_velocity << " m/s"
                        << std::endl;
                #endif
                break;

            case NodeId::RearSteering: //0x04
                rear_encoder_data_.steering_position_raw = wheel_data.steering_position_raw;
                rear_encoder_data_.steering_velocity_raw = wheel_data.steering_velocity_raw;

                steering_pos_m = motor_kinematics_->pulsesToMeters(static_cast<int32_t>(wheel_data.steering_position_raw));
                steering_vel_mps = motor_kinematics_->rpmToLinearVelocity(static_cast<int32_t>(wheel_data.steering_velocity_raw));

                rear_encoder_data_.steering_position = steering_pos_m;
                rear_encoder_data_.steering_velocity = steering_vel_mps;
                #if  _LOG_
                std::cout << "[Rear Steering] "
                        << "pos_raw=" << rear_encoder_data_.steering_position_raw << " pulses, "
                        << "vel_raw=" << rear_encoder_data_.steering_velocity_raw << " rpm, "
                        << "pos_m=" << rear_encoder_data_.steering_position << " m, "
                        << "vel_mps=" << rear_encoder_data_.steering_velocity << " m/s"
                        << std::endl;
                #endif
                break;

            default:
                // 알 수 없는 노드 ID 무시
                return;
        }

        // 부하 계산용 - 누적 회전량 및 누적 거리
        latest_distances_ = {
            #if 0
            front_encoder_data_.traction_position_raw,
            front_encoder_data_.steering_position_raw,
            rear_encoder_data_.traction_position_raw,
            rear_encoder_data_.steering_position_raw
            #endif
            front_encoder_data_.traction_position,
            front_encoder_data_.steering_position,
            rear_encoder_data_.traction_position,
            rear_encoder_data_.steering_position
        };

        latest_velocities_ = {
            front_encoder_data_.traction_velocity,
            front_encoder_data_.steering_velocity,
            rear_encoder_data_.traction_velocity,
            rear_encoder_data_.steering_velocity
        };

    } // 뮤텍스 잠금 해제

    // JointState 메시지 publish (raw 위치, 변환된 속도)
    // JointState 메시지는 여기서도 발행 가능 (동기화 위해 뮤텍스 걸어도 됨)
    {
        std::lock_guard<std::mutex> lock(encoder_data_mutex_);
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->now();
        joint_msg.name = {
            "front_traction_wheel_joint", "front_steering_wheel_joint",
            "rear_traction_wheel_joint", "rear_steering_wheel_joint"
        };

        joint_msg.position = {
            static_cast<double>(front_encoder_data_.traction_position_raw),
            static_cast<double>(front_encoder_data_.steering_position_raw),
            static_cast<double>(rear_encoder_data_.traction_position_raw),
            static_cast<double>(rear_encoder_data_.steering_position_raw)
        };

        joint_msg.velocity = {
            front_encoder_data_.traction_velocity, front_encoder_data_.steering_velocity,
            rear_encoder_data_.traction_velocity, rear_encoder_data_.steering_velocity
        };

        joint_state_pub_->publish(joint_msg);
    }
}

void MotorDriver::processPidMotion(const std::vector<double>& wheel_velocities)
{
    RCLCPP_INFO(this->get_logger(), "[MotorDriver] processPidMotion called");

    double dt = control_dt_;

    // 목표 직진 속도 (cmdVelCallback에서 변환된 값)
    // 여기서 {steering_angle_front, velocity_front, steering_angle_rear, velocity_rear} 순서로 반환
    double target_traction = wheel_velocities.size() > 1 ? wheel_velocities[1] : 0.0;  // traction velocity는 wheel_velocities[1]

    // 현재 직진 속도: front + rear traction velocity 평균 (m/s)
    double current_traction = (front_encoder_data_.traction_velocity + rear_encoder_data_.traction_velocity) / 2.0;

    // 직진 PID 출력
    double traction_output = traction_pid_controller_.computeCommand(target_traction, current_traction, dt);

    // 목표 각속도 (cmdVelCallback에서 변환된 값)
    double target_angular_velocity = wheel_velocities.size() > 0 ? wheel_velocities[0] : 0.0;  // steering 각도/속도
    double steering_output = 0.0;
#if ENABLE_STEERING_PID
    // 현재 각속도: (front_steering_velocity - rear_steering_velocity) / wheel_base_ (rad/s)
    double current_angular_velocity = (front_encoder_data_.steering_velocity - rear_encoder_data_.steering_velocity) / wheel_base_;
    // 조향 PID 출력
    steering_output = steering_pid_controller_.computeCommand(target_angular_velocity, current_angular_velocity, dt);
#else //// PID 미사용 시, 목표 각속도를 그대로 steering 출력으로 사용
    steering_output = target_angular_velocity;
#endif
    // 제어 명령을 모터에 전달
    double out_linear = std::clamp(traction_output, -traction_pid_.max, traction_pid_.max);
    double out_angular = std::clamp(steering_output, -steering_pid_.max, steering_pid_.max); 
   
    // PID 출력 CAN 메시지 변환 및 전송
    auto can_frames = convertPidOutputToCanFrame(out_linear, out_angular);

    for (const auto& [can_id_enum, data] : can_frames) {
        uint32_t can_id = static_cast<uint32_t>(can_id_enum);
        if (!can_driver_->sendFrame(can_id, data)) {
        //if (!socket_can_driver_->sendFrame(can_id, data)) {
            RCLCPP_ERROR(this->get_logger(), "PID 제어 CAN 프레임 전송 실패 (CAN ID: 0x%X)", can_id);
        }
    }
}

void MotorDriver::processDirectMotion(const std::vector<double>& wheel_velocities)
{
    RCLCPP_INFO(this->get_logger(), "[MotorDriver] processDirectMotion called");

    // 32바이트 크기의 전체 데이터 (앞/뒤 바퀴 steering + traction 정보)
    auto full_data = kinematics_->convertWheelVelocitiesToCanFrame(wheel_velocities);

    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> can_frames;

    // 앞 견인 바퀴 위치+속도 (첫 8바이트: [0..7])
    can_frames.emplace_back(static_cast<uint32_t>(NodeId::FrontTraction), 
                            std::vector<uint8_t>(full_data.begin(), full_data.begin() + 8));

    // 앞 조향 위치+속도 (다음 8바이트: [8..15])
    can_frames.emplace_back(static_cast<uint32_t>(NodeId::FrontSteering), 
                            std::vector<uint8_t>(full_data.begin() + 8, full_data.begin() + 16));

    // 뒤 견인 바퀴 위치+속도 (다음 8바이트: [16..23])
    can_frames.emplace_back(static_cast<uint32_t>(NodeId::RearTraction), 
                            std::vector<uint8_t>(full_data.begin() + 16, full_data.begin() + 24));

    // 뒤 조향 위치+속도 (마지막 8바이트: [24..31])
    can_frames.emplace_back(static_cast<uint32_t>(NodeId::RearSteering), 
                            std::vector<uint8_t>(full_data.begin() + 24, full_data.begin() + 32));

    // CAN 프레임 전송
    for (const auto& [can_id, data] : can_frames) {
        if (!can_driver_->sendFrame(can_id, data)) {
        //if (!socket_can_driver_->sendFrame(can_id, data)) {
            RCLCPP_ERROR(this->get_logger(), "직접 제어 CAN 프레임 전송 실패 (CAN ID: 0x%X)", can_id);
        }
    }
}

// TODO: 사용자 구현 필요
//CAN 하나당 4바이트씩 전송 중
std::vector<std::pair<NodeId, std::vector<uint8_t>>> MotorDriver::convertPidOutputToCanFrame(double traction_output, double steering_output)
{
    std::vector<std::pair<NodeId, std::vector<uint8_t>>> frames;

    int32_t traction_cmd = static_cast<int32_t>(traction_output * 1000);
    int32_t steering_cmd = static_cast<int32_t>(steering_output * 1000);

    auto pack = [](int32_t val) -> std::vector<uint8_t> {
        return {
            static_cast<uint8_t>((val >> 24) & 0xFF),
            static_cast<uint8_t>((val >> 16) & 0xFF),
            static_cast<uint8_t>((val >> 8) & 0xFF),
            static_cast<uint8_t>(val & 0xFF)
        };
    };

    // 각 바퀴 CAN ID에 맞춰 구성
    frames.emplace_back(NodeId::FrontTraction, pack(traction_cmd));
    frames.emplace_back(NodeId::RearTraction, pack(traction_cmd));
    frames.emplace_back(NodeId::FrontSteering, pack(steering_cmd));
    frames.emplace_back(NodeId::RearSteering, pack(-steering_cmd));  // 후륜 반대 방향 예시

    return frames;
}